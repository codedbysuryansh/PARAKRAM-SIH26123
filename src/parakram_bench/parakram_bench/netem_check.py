"""
Network-level loss check (CLAUDE_CODE/05, the secondary mechanism): tc netem vs QoS.

Run as root through the wrapper, which applies netem to the loopback interface:

    sudo bash src/parakram_bench/scripts/netem_verify.sh

For each fabric (every installed one of Fast DDS, CycloneDDS, rmw_zenoh) and case (netem loss %,
netem correlation %) a publisher and a subscriber are started as the invoking user, in two
processes, on an isolated ROS domain: Fast DDS forced onto UDPv4 (its default shared-memory
transport would bypass the network stack, and so netem, entirely), CycloneDDS on UDP with the
static peer list of ``parakram_comms/config/cyclonedds.xml``, rmw_zenoh through a router with the
robot1 configs of ``parakram_comms/config`` (TCP links, the rmw_zenoh default). The
publisher sends two 10 Hz streams of ``parakram_msgs/Intent`` (the coordination payload; the send
time rides in ``lease_expiry``): one with ``INTENT_QOS`` (BEST_EFFORT, as state/intent) and one
with ``TASK_QOS`` (RELIABLE + TRANSIENT_LOCAL + KEEP_ALL, as task/award). Once both streams flow,
netem is applied, the streams are measured for ``--measure`` s, netem is removed and RELIABLE gets
``--drain`` s to retransmit. Per stream: messages sent in the window, delivered (by the end of the
drain), delivered within 100 ms, latency.

Expected on the UDP fabrics: BEST_EFFORT delivery ~ 1 - loss (the loss is visible); RELIABLE ~
100 % with growing latency (retransmission hides the loss: why state/intent are never benchmarked
on RELIABLE). On rmw_zenoh every stream rides a TCP link, which retransmits BEST_EFFORT traffic
too: what the check measures there is how much of netem's loss reaches the application at all.
Output: ``<log_root>/netem_check_<id>/`` (manifest, per-case publisher / subscriber logs,
``netem_check.csv``) and ``results/benchmark2_netem_check.csv``.
"""

import argparse
import csv
import datetime
import json
import math
import os
import pwd
import shutil
import signal
import statistics
import subprocess
import sys
import time
import uuid

STREAMS = {'best_effort': '/netem_probe/intent_best_effort',
           'reliable': '/netem_probe/intent_reliable'}
QOS_NAMES = {'best_effort': 'INTENT_QOS (BEST_EFFORT, VOLATILE, KEEP_LAST 5)',
             'reliable': 'TASK_QOS (RELIABLE, TRANSIENT_LOCAL, KEEP_ALL)'}
CASES = '0:0,10:0,30:0,60:0,30:25'
OTHER_CASES = '0:0,30:0,60:0'          # the fabrics other than Fast DDS
FABRICS = {'rmw_fastrtps_cpp': 'UDPv4 (Fast DDS, shared memory off)',
           'rmw_cyclonedds_cpp': 'UDP (CycloneDDS, static unicast peers)',
           'rmw_zenoh_cpp': 'TCP (rmw_zenoh default links, via a router)'}
ZENOH_PORT = 7447                      # the robot1 router of parakram_comms/config
ON_TIME_S = 0.1
COLUMNS = ['case', 'rmw', 'link', 'loss_pct', 'corr_pct', 'iface', 'stream', 'qos', 'n_sent',
           'n_delivered',
           'delivery_ratio', 'expected_best_effort_ratio', 'tolerance', 'within_tolerance',
           'on_time_ratio_100ms', 'latency_p50_ms', 'latency_p95_ms', 'latency_max_ms',
           'duplicates']


def parse_cases(text):
    """``'0:0,30:25'`` -> [(0, 0), (30, 25)] (loss %, netem correlation %)."""
    out = []
    for item in str(text).split(','):
        if item.strip():
            loss, _, corr = item.partition(':')
            loss, corr = float(loss), float(corr or 0)
            if not (0 <= loss < 100 and 0 <= corr < 100):
                raise ValueError(f'bad case {item!r}')
            out.append((loss, corr))
    return out


def _pct(values, q):
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * q / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def analyse(pub_rows, sub_rows, t0, t1, loss_pct, corr_pct):
    """
    Delivery per stream of the messages PUBLISHED in [t0, t1].

    ``pub_rows``: (seq, t_send); ``sub_rows``: (stream, seq, t_send, t_recv). The BEST_EFFORT
    tolerance is 3 binomial sigma + 0.01 around 1 - loss (Bernoulli cases only: netem's
    correlated mode does not keep the nominal mean).
    """
    sent = {int(seq): float(t) for seq, t in pub_rows if t0 <= float(t) <= t1}
    out = {}
    for stream in STREAMS:
        lat, dup = {}, 0
        for st, seq, t_send, t_recv in sub_rows:
            seq = int(seq)
            if st != stream or seq not in sent:
                continue
            if seq in lat:
                dup += 1
                continue
            lat[seq] = float(t_recv) - float(t_send)
        n = len(sent)
        vals = list(lat.values())
        p = loss_pct / 100.0
        tol = 3.0 * math.sqrt(p * (1 - p) / n) + 0.01 if n else None
        ratio = len(lat) / n if n else None
        if stream == 'best_effort':
            within = (ratio is not None and abs(ratio - (1 - p)) <= tol) if corr_pct == 0 \
                else None
        else:
            within = ratio is not None and ratio >= 0.99      # RELIABLE: loss hidden
        out[stream] = {
            'n_sent': n, 'n_delivered': len(lat), 'delivery_ratio': ratio,
            'expected_best_effort_ratio': 1 - p, 'tolerance': tol if stream == 'best_effort'
            else 0.01, 'within_tolerance': within,
            'on_time_ratio_100ms': (sum(1 for v in vals if v <= ON_TIME_S) / n) if n else None,
            'latency_p50_ms': None if not vals else 1000 * statistics.median(vals),
            'latency_p95_ms': None if not vals else 1000 * _pct(vals, 95),
            'latency_max_ms': None if not vals else 1000 * max(vals), 'duplicates': dup}
    return out


# ------------------------------------------------------------------ probe roles (invoking user)
def _stamp(msg_time, t):
    msg_time.sec = int(math.floor(t))
    msg_time.nanosec = int((t - math.floor(t)) * 1e9)


def _shutdown():
    import rclpy
    try:
        rclpy.try_shutdown()
    except Exception:  # noqa: BLE001 - the SIGINT handler may have shut the context down first
        pass


def _spin_until_signal(node):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def run_pub(args):
    """Publish both streams at ``--rate`` Hz; log (seq, t_send)."""
    import rclpy
    from parakram_comms.qos import INTENT_QOS, TASK_QOS
    from parakram_msgs.msg import Intent
    rclpy.init()
    node = rclpy.create_node('netem_probe_pub')
    pubs = {'best_effort': node.create_publisher(Intent, STREAMS['best_effort'], INTENT_QOS),
            'reliable': node.create_publisher(Intent, STREAMS['reliable'], TASK_QOS)}
    f = open(args.out, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['seq', 't_send'])
    state = {'seq': 0}

    def tick():
        state['seq'] += 1
        t = time.time()
        for pub in pubs.values():
            m = Intent()
            m.robot_id = 'netem_probe'
            m.seq = state['seq']
            _stamp(m.lease_expiry, t)
            pub.publish(m)
        w.writerow([state['seq'], f'{t:.6f}'])
        f.flush()

    node.create_timer(1.0 / args.rate, tick)
    _spin_until_signal(node)
    f.close()
    node.destroy_node()
    _shutdown()


def run_sub(args):
    """Log every received message: (stream, seq, t_send, t_recv)."""
    import rclpy
    from parakram_comms.qos import INTENT_QOS, TASK_QOS
    from parakram_msgs.msg import Intent
    rclpy.init()
    node = rclpy.create_node('netem_probe_sub')
    f = open(args.out, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['stream', 'seq', 't_send', 't_recv'])

    def cb(stream, msg):
        t = time.time()
        sent = msg.lease_expiry.sec + 1e-9 * msg.lease_expiry.nanosec
        w.writerow([stream, msg.seq, f'{sent:.6f}', f'{t:.6f}'])
        f.flush()

    for stream, qos in (('best_effort', INTENT_QOS), ('reliable', TASK_QOS)):
        node.create_subscription(Intent, STREAMS[stream], lambda m, s=stream: cb(s, m), qos)
    _spin_until_signal(node)
    f.close()
    node.destroy_node()
    _shutdown()


# ------------------------------------------------------------------ controller (root)
def _tc(*args, check=True):
    tc = shutil.which('tc') or '/usr/sbin/tc'
    res = subprocess.run([tc] + list(args), capture_output=True, text=True)
    if check and res.returncode != 0:
        raise RuntimeError(f"tc {' '.join(args)}: {res.stderr.strip()}")
    return res


def _read(path):
    try:
        with open(path, newline='') as f:
            rows = list(csv.reader(f))
    except OSError:
        return []
    return rows[1:]


def _sim_busy(log_root):
    if os.path.exists(os.path.join(log_root, 'SIM_BUSY')):
        return 'a loss sweep holds bench/logs/SIM_BUSY'
    for args in (['-x', 'ruby'], ['-f', '/opt/ros/jazzy/bin/ros2 launch']):
        if subprocess.run(['pgrep'] + args, capture_output=True).returncode == 0:
            return f"a simulation / launch is running (pgrep {' '.join(args)})"
    return None


def installed_fabrics():
    """Return the fabrics of ``FABRICS`` whose rmw package is installed (Fast DDS first)."""
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    out = []
    for rmw in FABRICS:
        try:
            get_package_prefix(rmw)
            out.append(rmw)
        except PackageNotFoundError:
            pass
    return out


def _comms_config(ws, name):
    """Return a parakram_comms config file: the installed copy, else the source tree's."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('parakram_comms'), 'config', name)
        if os.path.isfile(path):
            return path
    except Exception:  # noqa: BLE001 - not installed: use the source tree
        pass
    return os.path.join(ws, 'src', 'parakram_comms', 'config', name)


def fabric_env(rmw, base, ws):
    """Environment of the probe processes on ``rmw``."""
    env = dict(base, RMW_IMPLEMENTATION=rmw)
    if rmw == 'rmw_fastrtps_cpp':
        env['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
    elif rmw == 'rmw_cyclonedds_cpp':
        env['CYCLONEDDS_URI'] = 'file://' + _comms_config(ws, 'cyclonedds.xml')
    elif rmw == 'rmw_zenoh_cpp':
        env['ZENOH_SESSION_CONFIG_URI'] = _comms_config(ws, 'zenoh_session_robot1.json5')
        env['ZENOH_ROUTER_CONFIG_URI'] = _comms_config(ws, 'zenoh_router_robot1.json5')
    return env


def _port_open(port):
    import socket
    with socket.socket() as sock:
        sock.settimeout(0.5)
        return sock.connect_ex(('127.0.0.1', port)) == 0


def run_case(i, rmw, env, loss, corr, args, out_dir, demote):
    """One case on one fabric: probe up, netem on, measure, netem off, drain; return rows."""
    short = rmw.replace('rmw_', '').replace('_cpp', '')
    tag = f'case{i}_{short}_loss{loss:g}_corr{corr:g}'
    pub_csv = os.path.join(out_dir, f'{tag}_pub.csv')
    sub_csv = os.path.join(out_dir, f'{tag}_sub.csv')
    me = [sys.executable, '-m', 'parakram_bench.netem_check']
    procs = []
    try:
        for role, path in (('sub', sub_csv), ('pub', pub_csv)):
            with open(os.path.join(out_dir, f'{tag}_{role}.log'), 'w') as log:
                procs.append(subprocess.Popen(
                    me + ['--role', role, '--out', path, '--rate', str(args.rate)], env=env,
                    stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                    preexec_fn=demote, cwd=out_dir))
        end = time.monotonic() + 60.0
        while time.monotonic() < end:
            got = [r[0] for r in _read(sub_csv)]
            if all(got.count(st) >= 20 for st in STREAMS):
                break
            time.sleep(0.5)
        else:
            print(f'[netem] case {i} {rmw}: the probe streams never matched; skipped',
                  flush=True)
            return []
        _tc('qdisc', 'add', 'dev', args.iface, 'root', 'netem', 'loss', f'{loss:g}%',
            f'{corr:g}%')
        qdisc = _tc('qdisc', 'show', 'dev', args.iface).stdout.strip()
        time.sleep(args.settle)
        t0 = time.time()
        time.sleep(args.measure)
        t1 = time.time()
        _tc('qdisc', 'del', 'dev', args.iface, 'root')
        time.sleep(args.drain)
    finally:
        _tc('qdisc', 'del', 'dev', args.iface, 'root', check=False)
        _stop(procs)
    res = analyse(_read(pub_csv), _read(sub_csv), t0, t1, loss, corr)
    be, rel = res['best_effort'], res['reliable']

    def fmt(v, spec):
        return 'n/a' if v is None else format(v, spec)

    print(f'[netem] {rmw} loss {loss:g}% corr {corr:g}% ({qdisc}): BEST_EFFORT delivered '
          f"{fmt(be['delivery_ratio'], '.3f')} (1-loss {1 - loss / 100:.2f}), RELIABLE "
          f"{fmt(rel['delivery_ratio'], '.3f')}, RELIABLE on time "
          f"{fmt(rel['on_time_ratio_100ms'], '.3f')}, p95 {fmt(rel['latency_p95_ms'], '.0f')} ms",
          flush=True)
    return [dict(r, case=i, rmw=rmw, link=FABRICS[rmw], loss_pct=loss, corr_pct=corr,
                 iface=args.iface, stream=stream, qos=QOS_NAMES[stream])
            for stream, r in res.items()]


def run_controller(args):
    """Apply netem per fabric and case, run the probe as the invoking user, analyse."""
    if os.geteuid() != 0 or 'SUDO_UID' not in os.environ:
        sys.exit('run as root via sudo: sudo bash src/parakram_bench/scripts/netem_verify.sh')
    from parakram_bringup import run_manifest as rm
    uid, gid, user = int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']), \
        os.environ['SUDO_USER']
    home = pwd.getpwnam(user).pw_dir
    log_root = rm.default_log_root()
    busy = _sim_busy(log_root)
    if busy:
        sys.exit(f'refusing: {busy}. netem on {args.iface} would hit that run too.')
    fabrics = installed_fabrics() if args.rmws == 'auto' else \
        [r for r in args.rmws.split(',') if r.strip()]
    plan = {rmw: parse_cases(args.cases if rmw == 'rmw_fastrtps_cpp' else args.other_cases)
            for rmw in fabrics}
    check_id = str(uuid.uuid4())[:8]
    out_dir = os.path.join(log_root, f'netem_check_{check_id}')
    os.makedirs(out_dir, exist_ok=True)
    os.chown(out_dir, uid, gid)

    def demote():
        os.setgroups(os.getgrouplist(user, gid))
        os.setgid(gid)
        os.setuid(uid)
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    base = dict(os.environ, HOME=home, USER=user, LOGNAME=user, ROS_DOMAIN_ID=str(args.domain_id))
    git = subprocess.run(['git', '-C', args.ws, 'rev-parse', 'HEAD'], capture_output=True,
                         text=True, preexec_fn=demote, env=base)
    dirty = subprocess.run(['git', '-C', args.ws, 'status', '--porcelain'], capture_output=True,
                           text=True, preexec_fn=demote, env=base)
    manifest = {
        'kind': 'netem_check', 'check_id': check_id, 'work_order': 'CLAUDE_CODE/05',
        'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'host': os.uname().nodename, 'kernel': os.uname().release,
        'tc_version': _tc('-V', check=False).stdout.strip(), 'iface': args.iface,
        'fabrics': {rmw: FABRICS[rmw] for rmw in fabrics}, 'cases': plan,
        'rate_hz': args.rate, 'measure_s': args.measure, 'drain_s': args.drain,
        'settle_s': args.settle, 'ros_domain_id': args.domain_id,
        'fabric_env': {rmw: fabric_env(rmw, {}, args.ws) for rmw in fabrics},
        'streams': STREAMS, 'qos': QOS_NAMES,
        'git_sha': git.stdout.strip() or 'unknown', 'git_dirty': bool(dirty.stdout.strip()),
        'mechanism': f'tc qdisc add dev {args.iface} root netem loss <L>% <C>% (applied after '
                     'discovery, removed before the drain)'}
    with open(os.path.join(out_dir, 'manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=2)
    rows, i, router = [], 0, None
    _tc('qdisc', 'del', 'dev', args.iface, 'root', check=False)       # stale netem, if any
    try:
        for rmw in fabrics:
            env = fabric_env(rmw, base, args.ws)
            if rmw == 'rmw_zenoh_cpp':
                from ament_index_python.packages import get_package_prefix
                if _port_open(ZENOH_PORT):
                    print(f'[netem] port {ZENOH_PORT} is taken (another Zenoh router?): '
                          'rmw_zenoh skipped', flush=True)
                    continue
                zenohd = os.path.join(get_package_prefix('rmw_zenoh_cpp'), 'lib',
                                      'rmw_zenoh_cpp', 'rmw_zenohd')
                with open(os.path.join(out_dir, 'zenoh_router.log'), 'w') as log:
                    router = subprocess.Popen([zenohd], env=env, stdout=log,
                                              stderr=subprocess.STDOUT, preexec_fn=demote)
                end = time.monotonic() + 15.0
                while not _port_open(ZENOH_PORT) and time.monotonic() < end:
                    time.sleep(0.5)
            for loss, corr in plan[rmw]:
                i += 1
                rows += run_case(i, rmw, env, loss, corr, args, out_dir, demote)
            if router is not None:
                _stop([router])
                router = None
    finally:
        _tc('qdisc', 'del', 'dev', args.iface, 'root', check=False)
        if router is not None:
            _stop([router])
    csv_path = os.path.join(out_dir, 'netem_check.csv')
    _write(csv_path, rows)
    targets = [csv_path]
    if args.results_dir:
        os.makedirs(args.results_dir, exist_ok=True)
        res_path = os.path.join(args.results_dir, 'benchmark2_netem_check.csv')
        shutil.copyfile(csv_path, res_path)
        targets.append(res_path)
    for root, _dirs, files in os.walk(out_dir):
        for name in [root] + [os.path.join(root, f) for f in files]:
            os.chown(name, uid, gid)
    for path in targets[1:]:
        os.chown(path, uid, gid)
    print(f'[netem] {len(rows)} rows -> ' + ', '.join(targets))
    return 0


def _stop(procs):
    for p in procs:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    end = time.monotonic() + 10.0
    while any(p.poll() is None for p in procs) and time.monotonic() < end:
        time.sleep(0.2)
    for p in procs:
        if p.poll() is None:
            p.kill()


def _write(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else ('' if v is None else v))
                        for k, v in r.items()})


def main(argv=None):
    """Run the controller (root) or one probe role (the invoking user)."""
    ap = argparse.ArgumentParser(description='CLAUDE_CODE/05 netem check')
    ap.add_argument('--role', default='controller', choices=('controller', 'pub', 'sub'))
    ap.add_argument('--out', default='')
    ap.add_argument('--rate', type=float, default=10.0, help='[Hz] per stream')
    ap.add_argument('--cases', default=CASES, help='loss%%:corr%% pairs (Fast DDS)')
    ap.add_argument('--other-cases', default=OTHER_CASES,
                    help='loss%%:corr%% pairs for the other fabrics')
    ap.add_argument('--rmws', default='auto',
                    help="comma list of rmw implementations; 'auto' = every installed one")
    ap.add_argument('--iface', default='lo')
    ap.add_argument('--measure', type=float, default=40.0, help='[s] with netem on')
    ap.add_argument('--settle', type=float, default=1.0, help='[s] after netem is applied')
    ap.add_argument('--drain', type=float, default=10.0,
                    help='[s] after netem is removed (RELIABLE retransmissions)')
    ap.add_argument('--domain-id', type=int, default=87)
    ap.add_argument('--ws', default=os.getcwd())
    ap.add_argument('--results-dir', default='')
    args = ap.parse_args(argv)
    if args.role == 'pub':
        return run_pub(args)
    if args.role == 'sub':
        return run_sub(args)
    return run_controller(args)


if __name__ == '__main__':
    raise SystemExit(main())
