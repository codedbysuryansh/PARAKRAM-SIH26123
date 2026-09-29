"""
Benchmark #2 sweep: throughput and collisions vs app-level packet loss (CLAUDE_CODE/05).

    python3 -m parakram_bench.run_loss_sweep --scenario intersection --seeds 5

For every loss level (``--levels``, default 0,10,...,60 %) and seed (1..``--seeds``) it runs the
same fixed scenario once, headless, one run at a time:

1. ``fleet_sim.launch.py`` with ``loss:=``, ``seed:=`` and a fresh ``run_id:=`` (the full stack,
   reactive safety layer on);
2. the CLAUDE_CODE/02 monitor ``coord_acceptance`` (Gazebo ground truth, an observer only). Its
   roster-kill test is switched off and its PASS verdict is not used; the sweep keeps what it
   measured;
3. ``coord.launch.py``, which reads loss and seed from the run manifest and applies the seeded
   app-level drop (``parakram_comms.loss``) to peer state/intent BEFORE coordination processes
   a message.

Per run it records the monitor's ground-truth numbers (robot-robot contacts = footprint distance
<= 1 cm, legs completed, makespan, throughput, the same definitions as the 02 acceptance) and the
loss filters' counters (``comms_<ns>.csv``): per peer stream, the messages the sender published
(its seq span), delivered by the transport, dropped by the injector and processed by coordination.
The intent receive-rate check (the drop is real and not hidden by retransmission) compares the
processed fraction of the peers' published intents with 1 - loss.

Outputs: ``results/benchmark2_loss_curve.csv`` (mean and sample std per level) + ``.png``,
``results/benchmark2_loss_runs.csv`` (every run with its run_id) and ``.json`` (provenance and
checks); the sweep log directory ``<log_root>/loss_sweep_<id>/`` keeps the sweep manifest, every
attempt in ``runs.csv`` and each process' console log. ``--sweep-dir`` resumes a sweep (runs
already done are skipped); ``--aggregate-only`` rebuilds the outputs from a sweep directory.

These are SIMULATION acceptance numbers (CLAIMS_LEDGER): not a MEASURED hardware claim.
"""

import argparse
import csv
import datetime
import glob
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import time
import uuid

LEVELS = (0, 10, 20, 30, 40, 50, 60)
FIXED_GOAL_SCENARIOS = ('intersection', 'headon')   # coord config/crossing_goals.yaml
LEFTOVERS = ('ruby', 'sim_clock_bridg', 'ground_truth_pu', 'map_server', 'lifecycle_manag',
             'parameter_bridg', 'robot_state_pub', 'component_conta', 'load_components',
             'coordination_no', 'roster_helper', 'orca_filter', 'collision_monit',
             'coord_acceptanc')
LAUNCH_PATTERN = '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 (launch|run) parakram_'
BRINGUP_FAIL = re.compile(r'process has died|Traceback|Aborting bringup|loading failed')
RATE_TOL = 0.03          # |realized drop - loss| and receive-rate tolerance (absolute)
RUN_COLUMNS = [
    'loss_pct', 'seed', 'model', 'burst_corr', 'run_id', 'status', 'attempt', 'wall_s',
    'contacts', 'min_rr_distance_m', 'min_static_clearance_m', 'legs_completed',
    'legs_assigned', 'all_legs_completed', 'makespan_s', 'throughput_legs_per_min',
    'run_length_s', 'max_blocked_s', 'breaker_events', 'tick_p95_ms', 'processes_died',
    'intent_streams', 'intent_published', 'intent_received', 'intent_dropped', 'intent_passed',
    'intent_drop_ratio', 'intent_delivered_ratio', 'intent_processed_ratio',
    'intent_mean_drop_burst',
    'state_published', 'state_received', 'state_dropped', 'state_drop_ratio',
    'state_processed_ratio', 'safety_ticks', 'safety_intervention_frac',
    'safety_filter_active_frac', 'error']
TEXT_COLUMNS = ('model', 'run_id', 'status', 'error')
CURVE_COLUMNS = [
    'loss_pct', 'model', 'n_runs', 'collisions_total', 'collisions_mean', 'collisions_std',
    'runs_with_collision', 'min_rr_distance_m', 'throughput_mean', 'throughput_std',
    'throughput_retained', 'makespan_mean', 'makespan_std', 'n_makespan', 'legs_completed_mean',
    'legs_completed_std', 'runs_all_legs_completed', 'max_blocked_s_mean',
    'breaker_events_total', 'intent_expected_ratio', 'intent_processed_ratio_mean',
    'intent_processed_ratio_std', 'intent_delivered_ratio_mean', 'intent_drop_ratio_mean',
    'intent_drop_ratio_std', 'intent_mean_drop_burst_mean', 'state_drop_ratio_mean',
    'safety_intervention_frac_mean', 'safety_intervention_frac_std', 'run_ids']


# ---------------------------------------------------------------------------------- pure helpers
def parse_levels(text):
    """Loss levels in percent: ``'0,10,20'`` -> [0, 10, 20] (0 <= level < 100)."""
    levels = [int(v) for v in str(text).split(',') if v.strip()]
    if not levels or any(not 0 <= v < 100 for v in levels):
        raise ValueError(f'loss levels must be percentages in [0, 100): {text!r}')
    return levels


def _num(value):
    """CSV/JSON cell -> float, or None for an empty cell."""
    if value is None or value == '':
        return None
    return float(value)


def _stats(values):
    """(mean, sample std, n) over the non-None values (std 0.0 for one value)."""
    vals = [v for v in values if v is not None]
    if not vals:
        return None, None, 0
    return (statistics.fmean(vals), statistics.stdev(vals) if len(vals) > 1 else 0.0,
            len(vals))


def comms_summary(run_dir, topic):
    """
    Sum the final loss-filter counters of one run for ``topic`` over its peer streams.

    ``published`` is the sum of the senders' seq spans (first..last seq a receiver saw), i.e. what
    was sent while the receiver listened; ``received`` what the transport delivered; ``dropped``
    what the injector discarded; ``passed`` what coordination processed.
    """
    last = {}
    for path in sorted(glob.glob(os.path.join(run_dir, 'comms_*.csv'))):
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                if row.get('topic') != topic:
                    continue
                key = (row['receiver'], row['sender'])
                if key not in last or float(row['t']) >= float(last[key]['t']):
                    last[key] = row
    rec = sum(int(r['received']) for r in last.values())
    drop = sum(int(r['dropped']) for r in last.values())
    bursts = sum(int(r.get('drop_bursts') or 0) for r in last.values())
    pub = 0
    for r in last.values():
        first, final = r.get('first_seq', ''), r.get('last_seq', '')
        span = int(final) - int(first) + 1 if first != '' and final != '' else 0
        if span <= 0:
            pub = None                          # no seq, or a sender restart: no span
            break
        pub += span
    out = {'streams': len(last), 'published': pub, 'received': rec, 'dropped': drop,
           'passed': rec - drop, 'drop_ratio': drop / rec if rec else None,
           'delivered_ratio': rec / pub if pub else None,
           'processed_ratio': (rec - drop) / pub if pub else None,
           'mean_drop_burst': drop / bursts if bursts else None}
    return out


def safety_summary(run_dir, t_start, t_end):
    """
    Share of the reactive safety layer's ticks in [t_start, t_end] (sim s) that intervened.

    From the CLAUDE_CODE/03 filter logs ``safety_<ns>.csv``: ``intervention`` (filter or
    collision monitor changed the command) and ``filter_active`` (NH-ORCA changed it). A
    descriptive figure: how much of the separation the network-independent floor provided.
    """
    n = inter = active = 0
    if t_start is None or t_end is None:
        return {'safety_ticks': 0, 'safety_intervention_frac': None,
                'safety_filter_active_frac': None}
    for path in sorted(glob.glob(os.path.join(run_dir, 'safety_*.csv'))):
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                try:
                    t = float(row['t'])
                    if t_start <= t <= t_end:
                        inter += int(row['intervention'])
                        active += int(row['filter_active'])
                        n += 1
                except (KeyError, ValueError):
                    continue
    return {'safety_ticks': n, 'safety_intervention_frac': inter / n if n else None,
            'safety_filter_active_frac': active / n if n else None}


def enrich(row, log_root):
    """Fill the log-derived figures a row lacks (rows written before a metric existed)."""
    if row.get('safety_intervention_frac') not in (None, ''):
        return row
    run_dir = os.path.join(log_root, row['run_id'])
    try:
        with open(os.path.join(run_dir, 'coord_acceptance.json')) as f:
            summary = json.load(f)
    except (OSError, ValueError):
        return row
    row = dict(row)
    row.update(safety_summary(run_dir, summary.get('coord_start_sim_time'),
                              summary.get('end_sim_time')))
    cs = comms_summary(run_dir, 'intent')
    if row.get('intent_mean_drop_burst') in (None, ''):
        row['intent_mean_drop_burst'] = cs['mean_drop_burst']
    return row


def monitor_metrics(summary):
    """Per-run metrics from the 02 monitor's ``coord_acceptance.json``."""
    legs = summary.get('legs_completed') or {}
    assigned = summary.get('legs_assigned') or {}
    n_legs = sum(legs.values())
    thr = summary.get('throughput_legs_per_min')
    breakers = sum(sum(v.values()) for v in (summary.get('breaker_events') or {}).values())
    tick = ((summary.get('tick_compute_ms') or {}).get('all') or {}).get('p95')
    checks = summary.get('checks') or {}
    return {
        'contacts': len(summary.get('robot_robot_contacts') or []),
        'min_rr_distance_m': summary.get('min_robot_robot_distance_m'),
        'min_static_clearance_m': summary.get('min_robot_static_clearance_m'),
        'legs_completed': n_legs, 'legs_assigned': sum(assigned.values()),
        'all_legs_completed': int(bool(checks.get('every_assigned_leg_completed'))),
        'makespan_s': summary.get('makespan_s'),
        # no leg completed at all = zero throughput (the monitor reports None then)
        'throughput_legs_per_min': thr if thr is not None else (0.0 if n_legs == 0 else None),
        'run_length_s': summary.get('run_length_s'),
        'max_blocked_s': max((summary.get('max_blocked_s') or {}).values(), default=None),
        'breaker_events': breakers, 'tick_p95_ms': tick}


def aggregate(rows, levels, model):
    """Per-level mean / sample std over the runs with status ``ok``."""
    curve, base = [], None
    for lv in levels:
        rs = [r for r in rows if int(_num(r['loss_pct'])) == lv and r['status'] == 'ok']
        col = {k: [_num(r.get(k)) for r in rs] for k in RUN_COLUMNS if k not in TEXT_COLUMNS}
        contacts = [int(c) for c in col['contacts'] if c is not None]
        thr = _stats(col['throughput_legs_per_min'])
        if lv == 0:
            base = thr[0]
        mk = _stats(col['makespan_s'])
        legs = _stats(col['legs_completed'])
        proc = _stats(col['intent_processed_ratio'])
        drop = _stats(col['intent_drop_ratio'])
        mins = [v for v in col['min_rr_distance_m'] if v is not None]
        curve.append({
            'loss_pct': lv, 'model': model, 'n_runs': len(rs),
            'collisions_total': sum(contacts), 'collisions_mean': _stats(contacts)[0],
            'collisions_std': _stats(contacts)[1],
            'runs_with_collision': sum(1 for c in contacts if c > 0),
            'min_rr_distance_m': min(mins) if mins else None,
            'throughput_mean': thr[0], 'throughput_std': thr[1],
            'throughput_retained': (thr[0] / base) if base and thr[0] is not None else None,
            'makespan_mean': mk[0], 'makespan_std': mk[1], 'n_makespan': mk[2],
            'legs_completed_mean': legs[0], 'legs_completed_std': legs[1],
            'runs_all_legs_completed': int(sum(v or 0 for v in col['all_legs_completed'])),
            'max_blocked_s_mean': _stats(col['max_blocked_s'])[0],
            'breaker_events_total': int(sum(v or 0 for v in col['breaker_events'])),
            'intent_expected_ratio': 1.0 - lv / 100.0,
            'intent_processed_ratio_mean': proc[0], 'intent_processed_ratio_std': proc[1],
            'intent_delivered_ratio_mean': _stats(col['intent_delivered_ratio'])[0],
            'intent_drop_ratio_mean': drop[0], 'intent_drop_ratio_std': drop[1],
            'intent_mean_drop_burst_mean': _stats(col['intent_mean_drop_burst'])[0],
            'state_drop_ratio_mean': _stats(col['state_drop_ratio'])[0],
            'safety_intervention_frac_mean': _stats(col['safety_intervention_frac'])[0],
            'safety_intervention_frac_std': _stats(col['safety_intervention_frac'])[1],
            'run_ids': ' '.join(r['run_id'] for r in rs)})
    return curve


def checks(curve, planned_seeds, max_collision_free_pct=50):
    """
    Evaluate the 05 acceptance items that the numbers decide (plus descriptive figures).

    ``zero_collisions_up_to_50pct`` needs EVERY planned run at those levels to be present (a
    missing run is not evidence). The throughput figures are descriptive: the work order asks for
    a graceful (non-cliff) decay and gives no number, so none is invented here.
    """
    low = [c for c in curve if c['loss_pct'] <= max_collision_free_pct]
    complete = all(c['n_runs'] == planned_seeds for c in curve)
    rate_ok = []
    for c in curve:
        p = c['loss_pct'] / 100.0
        if c['intent_drop_ratio_mean'] is None or c['intent_processed_ratio_mean'] is None:
            rate_ok.append(False)
            continue
        delivered = c['intent_delivered_ratio_mean'] or 0.0
        rate_ok.append(abs(c['intent_drop_ratio_mean'] - p) <= RATE_TOL and
                       abs(c['intent_processed_ratio_mean'] - delivered * (1 - p)) <= RATE_TOL)
    retained = [c['throughput_retained'] for c in curve]
    steps = [retained[i - 1] - retained[i] for i in range(1, len(retained))
             if retained[i - 1] is not None and retained[i] is not None]
    return {
        'all_planned_runs_present': complete,
        'zero_collisions_up_to_50pct': bool(low) and all(
            c['n_runs'] == planned_seeds and c['collisions_total'] == 0 for c in low),
        'collisions_by_level': {c['loss_pct']: c['collisions_total'] for c in curve},
        'intent_rate_follows_loss': all(rate_ok) and bool(rate_ok),
        'intent_rate_check_by_level': {c['loss_pct']: ok for c, ok in zip(curve, rate_ok)},
        'throughput_positive_every_level': all((c['throughput_mean'] or 0) > 0 for c in curve),
        'throughput_retained_by_level': {c['loss_pct']: c['throughput_retained']
                                         for c in curve},
        'largest_step_drop_of_baseline': max(steps) if steps else None,
        'note': 'descriptive decay figures; "graceful (non-cliff)" is judged from the curve'}


def write_csv(path, columns, rows):
    """Write ``rows`` (dicts) with ``columns``; floats to 4 decimals."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else ('' if v is None else v))
                        for k, v in r.items()})


def read_runs(path):
    """All attempts recorded in a sweep's ``runs.csv``."""
    if not os.path.isfile(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def append_run(path, row):
    """
    Append one attempt to ``runs.csv``.

    A resumed sweep keeps the file's own header (a sweep started by an older version has fewer
    columns); figures missing from it are filled from the run's logs by ``enrich``.
    """
    fields = RUN_COLUMNS
    if os.path.isfile(path):
        with open(path, newline='') as f:
            fields = next(csv.reader(f), None) or RUN_COLUMNS
    new = not os.path.isfile(path)
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        if new:
            w.writeheader()
        w.writerow({k: ('' if v is None else v) for k, v in row.items()})


def latest_ok(rows):
    """Return the last ``ok`` attempt per (loss, seed)."""
    best = {}
    for r in rows:
        if r['status'] == 'ok':
            best[(int(_num(r['loss_pct'])), int(_num(r['seed'])))] = r
    return [best[k] for k in sorted(best)]


def plot(curve, rows, path, title, caption):
    """Throughput, collisions and intent receive-rate vs injected loss."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    curve = [c for c in curve if c['n_runs']]           # levels not run (yet) are not drawn
    if not curve:
        return
    x = [c['loss_pct'] for c in curve]
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
    pts = [(int(_num(r['loss_pct'])), _num(r['throughput_legs_per_min'])) for r in rows]
    ax[0].scatter([p[0] for p in pts if p[1] is not None], [p[1] for p in pts if p[1] is not None],
                  s=14, color='0.6', zorder=2, label='single run')
    ax[0].errorbar(x, [c['throughput_mean'] for c in curve],
                   yerr=[c['throughput_std'] or 0.0 for c in curve], marker='o', capsize=4,
                   color='tab:blue', zorder=3, label='mean +/- std')
    ax[0].set_xlabel('injected loss on peer state/intent [%]')
    ax[0].set_ylabel('throughput [legs/min]')
    ax[0].set_ylim(bottom=0)
    ax[0].set_title('throughput vs loss')
    ax[0].legend(loc='lower left', fontsize=8)

    cpts = [(int(_num(r['loss_pct'])), int(_num(r['contacts']))) for r in rows]
    ax[1].axvspan(-3, 52, color='tab:green', alpha=0.07, label='acceptance: 0 up to ~50 %')
    ax[1].scatter([p[0] for p in cpts], [p[1] for p in cpts], s=18, color='tab:red',
                  label='contacts in one run (ground truth)')
    ax[1].bar(x, [c['collisions_total'] for c in curve], width=4, color='tab:red', alpha=0.25,
              label='total per level')
    ax[1].set_xlabel('injected loss on peer state/intent [%]')
    ax[1].set_ylabel('robot-robot contacts (footprint distance <= 1 cm)')
    top = max([c['collisions_total'] for c in curve] + [1])
    ax[1].set_ylim(-0.1, top + 1)
    ax[1].yaxis.set_major_locator(MaxNLocator(integer=True))
    ax[1].set_title('collisions vs loss')
    ax[1].legend(loc='upper left', fontsize=8)

    ax[2].plot([0, 100], [1, 0], '--', color='0.5', label='1 - loss')
    ax[2].errorbar(x, [c['intent_processed_ratio_mean'] for c in curve],
                   yerr=[c['intent_processed_ratio_std'] or 0.0 for c in curve], marker='o',
                   capsize=4, color='tab:green', label='processed by coordination / published')
    ax[2].plot(x, [c['intent_delivered_ratio_mean'] for c in curve], 's:', color='tab:purple',
               label='delivered by transport / published')
    ax[2].set_xlim(-3, max(x) + 5)
    ax[2].set_ylim(0, 1.08)
    ax[2].set_xlabel('injected loss on peer state/intent [%]')
    ax[2].set_ylabel('fraction of peer intents')
    ax[2].set_title('intent receive-rate (drop is real)')
    ax[2].legend(loc='lower left', fontsize=8)
    for a in ax:
        a.set_xticks(x)
        a.grid(alpha=0.3)
    fig.suptitle(title, fontsize=11)
    fig.text(0.5, 0.005, caption, ha='center', fontsize=8, color='0.3')
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------------------- processes
def _child_setup():
    # a background job's children would inherit an ignored SIGINT; launch needs it to shut down
    signal.signal(signal.SIGINT, signal.SIG_DFL)


def spawn(cmd, log_path, env):
    """Start ``cmd`` in its own process group, console to ``log_path``."""
    with open(log_path, 'ab') as log:
        return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, env=env, start_new_session=True,
                                preexec_fn=_child_setup)


def _pids(args):
    out = subprocess.run(['pgrep'] + args, capture_output=True, text=True)
    return [int(p) for p in out.stdout.split() if int(p) != os.getpid()]


def leftovers():
    """Return the pids of simulation / launch processes still alive."""
    pids = _pids(['-f', LAUNCH_PATTERN])
    for name in LEFTOVERS:
        pids += _pids(['-x', name])
    return sorted(set(pids))


def cleanup(log=print):
    """SIGINT stray launches, then TERM / KILL every leftover sim process."""
    for pid in _pids(['-f', LAUNCH_PATTERN]):
        _signal(pid, signal.SIGINT)
    _wait_gone(lambda: _pids(['-f', LAUNCH_PATTERN]), 30.0)
    for sig, wait in ((signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        pids = leftovers()
        if not pids:
            return True
        if sig == signal.SIGKILL:
            log(f'SIGKILL leftovers {pids}')
        for pid in pids:
            _signal(pid, sig)
        _wait_gone(leftovers, wait)
    return not leftovers()


def _signal(pid, sig):
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        pass


def _wait_gone(probe, timeout):
    end = time.monotonic() + timeout
    while probe() and time.monotonic() < end:
        time.sleep(0.5)


def stop(procs, timeout=45.0):
    """SIGINT each process group (like Ctrl+C), escalate to SIGTERM / SIGKILL."""
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 10.0),
                      (signal.SIGKILL, 5.0)):
        alive = [p for p in procs if p.poll() is None]
        if not alive:
            return
        for p in alive:
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                pass
        end = time.monotonic() + wait
        while any(p.poll() is None for p in alive) and time.monotonic() < end:
            time.sleep(0.5)


def _count(path, text, limit=None):
    try:
        with open(path, 'rb') as f:
            data = f.read(limit) if limit else f.read()
    except OSError:
        return 0
    return data.decode(errors='replace').count(text)


def _grep(path, pattern, limit=None):
    try:
        with open(path, 'rb') as f:
            data = f.read(limit) if limit else f.read()
    except OSError:
        return []
    return [ln for ln in data.decode(errors='replace').splitlines() if pattern.search(ln)]


# ---------------------------------------------------------------------------------- one run
def run_one(args, sweep_dir, log_root, loss_pct, seed, attempt, env):
    """Run one (loss, seed) trial; return its row for ``runs.csv``."""
    run_id = str(uuid.uuid4())
    run_dir = os.path.join(log_root, run_id)
    tag = f'loss{loss_pct:02d}_seed{seed}_try{attempt}'
    logs = {k: os.path.join(sweep_dir, 'logs', f'{tag}_{k}.log')
            for k in ('fleet', 'monitor', 'coord')}
    row = {'loss_pct': loss_pct, 'seed': seed, 'model': args.model,
           'burst_corr': args.burst_corr, 'run_id': run_id, 'attempt': attempt,
           'status': 'running', 'error': ''}
    n = args.n_robots
    t0 = time.monotonic()
    procs = []
    fleet_bytes = coord_bytes = None
    try:
        if not cleanup():
            raise RuntimeError(f'could not clear leftover processes {leftovers()}')
        fleet = spawn(['ros2', 'launch', 'parakram_bringup', 'fleet_sim.launch.py',
                       f'n_robots:={n}', f'scenario:={args.scenario}', f'seed:={seed}',
                       f'loss:={loss_pct / 100.0}', f'loss_model:={args.model}',
                       f'loss_burst_corr:={args.burst_corr}', f'run_id:={run_id}'],
                      logs['fleet'], env)
        procs.append(fleet)
        end = time.monotonic() + args.bringup_timeout
        while _count(logs['fleet'], 'Managed nodes are active') < 2 * n + 1:
            bad = _grep(logs['fleet'], BRINGUP_FAIL)
            if bad or fleet.poll() is not None or time.monotonic() > end:
                row.update(status='bringup_failed',
                           error=(bad[0][-160:] if bad else 'fleet exited' if
                                  fleet.poll() is not None else 'bringup timeout'))
                return row
            time.sleep(2.0)
        monitor = spawn(['ros2', 'run', 'parakram_coord', 'coord_acceptance', '--n', str(n),
                         '--run-dir', run_dir, '--assign-duration', str(args.assign_duration),
                         '--kill-roster-at', '1e9', '--timeout', str(args.timeout),
                         '--start-timeout', str(args.start_timeout)], logs['monitor'], env)
        procs.append(monitor)
        time.sleep(3.0)
        coord = spawn(['ros2', 'launch', 'parakram_coord', 'coord.launch.py', f'n_robots:={n}',
                       f'scenario:={args.scenario}', f'assign_duration:={args.assign_duration}',
                       f'run_dir:={run_dir}'], logs['coord'], env)
        procs.append(coord)
        limit = time.monotonic() + args.start_timeout + 3.0 * args.timeout + 60.0
        while monitor.poll() is None:
            if fleet.poll() is not None or coord.poll() is not None:
                row.update(status='launch_exited', error='a launch exited during the run')
                return row
            if time.monotonic() > limit:
                row.update(status='monitor_timeout', error='monitor did not finish (wall limit)')
                return row
            time.sleep(2.0)
        # only what was logged during the run counts (shutdown prints 'process has died' too)
        fleet_bytes, coord_bytes = os.path.getsize(logs['fleet']), os.path.getsize(logs['coord'])
    finally:
        stop(procs)
        cleanup()
        row['wall_s'] = round(time.monotonic() - t0, 1)
    try:
        with open(os.path.join(run_dir, 'coord_acceptance.json')) as f:
            summary = json.load(f)
    except (OSError, ValueError) as exc:
        row.update(status='no_result', error=f'coord_acceptance.json: {exc}')
        return row
    if summary.get('error'):
        row.update(status='no_result', error=str(summary['error']))
        return row
    row.update(monitor_metrics(summary))
    row.update(safety_summary(run_dir, summary.get('coord_start_sim_time'),
                              summary.get('end_sim_time')))
    died = _grep(logs['fleet'], re.compile('process has died'), fleet_bytes)
    died += _grep(logs['coord'], re.compile('process has died'), coord_bytes)
    row['processes_died'] = len(died)
    for topic in ('intent', 'state'):
        cs = comms_summary(run_dir, topic)
        pre = f'{topic}_'
        row.update({pre + 'published': cs['published'], pre + 'received': cs['received'],
                    pre + 'dropped': cs['dropped'], pre + 'drop_ratio': cs['drop_ratio'],
                    pre + 'processed_ratio': cs['processed_ratio']})
        if topic == 'intent':
            row.update(intent_streams=cs['streams'], intent_passed=cs['passed'],
                       intent_delivered_ratio=cs['delivered_ratio'],
                       intent_mean_drop_burst=cs['mean_drop_burst'])
    if not row['intent_streams']:
        row.update(status='no_comms_log', error='no comms_<ns>.csv counters in the run dir')
    elif died:
        row.update(status='process_died', error=died[0][-160:])
    else:
        row['status'] = 'ok'
    return row


# ---------------------------------------------------------------------------------- driver
def _workspace_root():
    try:
        from ament_index_python.packages import get_package_prefix
        from parakram_bringup import run_manifest as rm
        return rm.find_workspace_root(get_package_prefix('parakram_bringup'))
    except Exception:  # noqa: BLE001 - not sourced: fall back to the current directory
        return None


def _suffix(model):
    return '' if model == 'bernoulli' else f'_{model}'


def write_outputs(sweep_dir, manifest, results_dir):
    """Aggregate the sweep's ok runs into CSV / JSON / PNG (sweep dir, and results dir)."""
    attempts = read_runs(os.path.join(sweep_dir, 'runs.csv'))
    rows = [enrich(r, manifest.get('log_root') or os.path.dirname(sweep_dir))
            for r in latest_ok(attempts)]
    levels, seeds = manifest['levels'], manifest['seeds']
    rows = [r for r in rows if int(_num(r['loss_pct'])) in levels
            and int(_num(r['seed'])) in seeds]
    model = manifest['model']
    curve = aggregate(rows, levels, model)
    result = {'sweep_id': manifest['sweep_id'], 'sweep_dir': sweep_dir,
              'generated_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'claim_status': 'SIMULATION acceptance verification (CLAUDE_CODE/05); not a '
                              'MEASURED hardware result',
              'manifest': manifest, 'checks': checks(curve, len(seeds)), 'curve': curve,
              'attempts_not_ok': [{k: r.get(k) for k in ('loss_pct', 'seed', 'attempt', 'run_id',
                                                         'status', 'error')}
                                  for r in attempts if r['status'] != 'ok']}
    name = f'benchmark2_loss_curve{_suffix(model)}'
    title = (f"Benchmark #2 (simulation): {manifest['n_robots']} robots, "
             f"'{manifest['scenario']}', app-level {model} loss on BEST_EFFORT peer "
             f'state/intent, seeds {seeds[0]}..{seeds[-1]}')
    caption = (f'SIMULATION, Gazebo ground truth; CLAUDE_CODE/05 acceptance verification, not a '
               f"hardware claim.  sweep {manifest['sweep_id']}  git {manifest['git_sha'][:10]}"
               f"{' (dirty)' if manifest.get('git_dirty') else ''}  window "
               f"{manifest['assign_duration']:.0f} s sim/run")
    targets = [sweep_dir] + ([results_dir] if results_dir else [])
    for d in targets:
        write_csv(os.path.join(d, name + '.csv'), CURVE_COLUMNS, curve)
        write_csv(os.path.join(d, f'benchmark2_loss_runs{_suffix(model)}.csv'), RUN_COLUMNS,
                  rows)
        with open(os.path.join(d, name + '.json'), 'w') as f:
            json.dump(result, f, indent=2, default=str)
            f.write('\n')
        if rows:
            plot(curve, rows, os.path.join(d, name + '.png'), title, caption)
    return result


def main(argv=None):
    """Run (or resume, or re-aggregate) a loss sweep."""
    ap = argparse.ArgumentParser(description='CLAUDE_CODE/05 Benchmark #2 loss sweep')
    ap.add_argument('--scenario', default='intersection', choices=FIXED_GOAL_SCENARIOS)
    ap.add_argument('--seeds', type=int, default=5, help='seeds 1..N')
    ap.add_argument('--seed-list', default='', help='explicit seeds, e.g. 1,3 (overrides)')
    ap.add_argument('--levels', default=','.join(map(str, LEVELS)), help='loss levels [%%]')
    ap.add_argument('--model', default='bernoulli', choices=('bernoulli', 'gilbert_elliott'))
    ap.add_argument('--burst-corr', type=float, default=0.8,
                    help='Gilbert-Elliott lag-1 correlation rho')
    ap.add_argument('--n-robots', type=int, default=3)
    ap.add_argument('--assign-duration', type=float, default=300.0,
                    help='[s sim] crossing window per run (the 02 acceptance window)')
    ap.add_argument('--timeout', type=float, default=480.0, help='[s sim] monitor timeout')
    ap.add_argument('--start-timeout', type=float, default=180.0,
                    help='[s wall] coordination start timeout')
    ap.add_argument('--bringup-timeout', type=float, default=240.0, help='[s wall]')
    ap.add_argument('--retries', type=int, default=2, help='extra attempts after an '
                    'infrastructure failure (bring-up, launch exit, no result)')
    ap.add_argument('--sweep-dir', default='', help='resume / re-aggregate this sweep')
    ap.add_argument('--aggregate-only', action='store_true')
    ap.add_argument('--results-dir', default='',
                    help='default: <ws>/src/parakram_bench/results')
    ap.add_argument('--no-results', action='store_true',
                    help='write the aggregate to the sweep dir only (verification runs)')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args(argv)

    from parakram_bringup import run_manifest as rm
    levels = parse_levels(args.levels)
    seeds = ([int(s) for s in args.seed_list.split(',') if s.strip()] if args.seed_list
             else list(range(1, args.seeds + 1)))
    log_root = rm.default_log_root()
    ws = _workspace_root()
    results_dir = None if args.no_results else (
        args.results_dir or os.path.join(ws or os.getcwd(), 'src', 'parakram_bench', 'results'))

    if args.sweep_dir:
        sweep_dir = os.path.abspath(args.sweep_dir)
        with open(os.path.join(sweep_dir, 'sweep_manifest.json')) as f:
            manifest = json.load(f)
    else:
        sweep_id = str(uuid.uuid4())[:8]
        sweep_dir = os.path.join(log_root, f'loss_sweep_{sweep_id}')
        manifest = rm.base_manifest(sweep_id, seeds[0], ws)
        manifest.pop('seed', None)
        manifest.pop('run_id', None)
        manifest.update({
            'kind': 'loss_sweep', 'sweep_id': sweep_id, 'work_order': 'CLAUDE_CODE/05',
            'scenario': args.scenario, 'n_robots': args.n_robots, 'levels': levels,
            'seeds': seeds, 'model': args.model, 'burst_corr': args.burst_corr,
            'assign_duration': args.assign_duration, 'timeout': args.timeout,
            'mechanism': 'app-level seeded drop on peer state/intent before processing '
                         '(parakram_comms.loss); BEST_EFFORT QoS; task/award traffic untouched',
            'ground_truth': 'Gazebo /ground_truth/poses via parakram_coord coord_acceptance',
            'command': ' '.join([os.path.basename(sys.executable), '-m',
                                 'parakram_bench.run_loss_sweep'] + list(argv or sys.argv[1:])),
            'log_root': log_root})
    plan = [(lv, s) for lv in manifest['levels'] for s in manifest['seeds']]
    runs_csv = os.path.join(sweep_dir, 'runs.csv')
    if args.aggregate_only:
        res = write_outputs(sweep_dir, manifest, results_dir)
        print(json.dumps(res['checks'], indent=1, default=str))
        return 0
    done = {(int(_num(r['loss_pct'])), int(_num(r['seed']))) for r in latest_ok(
        read_runs(runs_csv))}
    todo = [p for p in plan if p not in done]
    per_run = manifest['assign_duration'] + 120.0
    print(f'[sweep] {sweep_dir}: {len(plan)} runs planned, {len(todo)} to do '
          f'(~{len(todo) * per_run / 3600.0:.1f} h at ~{per_run / 60.0:.0f} min each)', flush=True)
    if args.dry_run:
        for lv, s in todo:
            print(f'  loss {lv:2d} %  seed {s}')
        return 0
    for k in ('scenario', 'model', 'burst_corr', 'n_robots', 'assign_duration', 'timeout'):
        setattr(args, k, manifest[k])            # a resumed sweep keeps its own settings
    os.makedirs(os.path.join(sweep_dir, 'logs'), exist_ok=True)
    rm.write_manifest(sweep_dir, manifest)
    os.rename(os.path.join(sweep_dir, 'run_manifest.json'),
              os.path.join(sweep_dir, 'sweep_manifest.json'))
    env = dict(os.environ)
    env.setdefault('TURTLEBOT3_MODEL', 'burger')
    lock = os.path.join(log_root, 'SIM_BUSY')
    with open(lock, 'w') as f:
        f.write(f'{os.getpid()} loss sweep {sweep_dir}\n')
    try:
        for i, (lv, s) in enumerate(todo, 1):
            for attempt in range(1, args.retries + 2):
                row = run_one(args, sweep_dir, log_root, lv, s, attempt, env)
                append_run(runs_csv, row)
                print(f'[sweep] {i}/{len(todo)} loss {lv}% seed {s} try {attempt}: '
                      f"{row['status']} run_id={row['run_id']} contacts={row.get('contacts')} "
                      f"thr={row.get('throughput_legs_per_min')} "
                      f"intent_drop={row.get('intent_drop_ratio')} wall={row.get('wall_s')}s"
                      + (f" ({row['error']})" if row['error'] else ''), flush=True)
                # retry infrastructure failures only; a finished run is a result, whatever it is
                if row['status'] in ('ok', 'process_died', 'no_comms_log'):
                    break
            try:
                write_outputs(sweep_dir, manifest, None)   # progress (sweep dir only)
            except Exception as exc:  # noqa: BLE001 - a report must never stop the sweep
                print(f'[sweep] progress report failed: {exc!r}', flush=True)
    except KeyboardInterrupt:
        print('[sweep] interrupted; resume with --sweep-dir ' + sweep_dir, flush=True)
        cleanup()
        return 130
    finally:
        if os.path.exists(lock):
            os.remove(lock)
    res = write_outputs(sweep_dir, manifest, results_dir)
    print(json.dumps(res['checks'], indent=1, default=str))
    print(f'[sweep] done: {sweep_dir}' + (f' -> {results_dir}' if results_dir else ''))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
