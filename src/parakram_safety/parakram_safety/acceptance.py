"""
CLAUDE_CODE/03 acceptance monitor (observer: it never commands a robot).

    ros2 run parakram_safety safety_acceptance --n 3 --mode on
    ros2 run parakram_safety safety_acceptance --n 3 --mode off
    ros2 run parakram_safety safety_acceptance --n 3 --mode peers_killed --kill-peers-at 3

Watches a ``fleet_sim scenario:=headon`` session whose coordination runs with conflict resolution
disabled (``coord.launch.py reactive_only:=true``), so only the reactive safety layer can keep the
robots apart, and checks the CLAUDE_CODE/03 PASS criteria:

1. ``--mode on``: safety ON -> 0 inter-robot collisions, and the layer did intervene.
2. ``--mode off``: safety OFF (``safety_enabled:=false``, ablation A1) -> >= 1 collision, and the
   layer was verifiably off (no filter, collision monitor toggled off).
3. ``--mode peers_killed``: every publisher of peer ``state`` / ``intent`` (the coordination nodes
   and the roster helper) is SIGKILLed before the encounter -> the safety layer still keeps the
   robots apart and still intervenes (compare with the ``on`` run), no peer message is seen after
   the kill, and the ROS graph shows that neither safety node subscribes to any peer topic.
4. The safety logs (``safety_<ns>.csv``, ``safety_status``) carry the required fields
   consistently: exact CSV header, #interventions equal to the status counter, and the logged
   ``min_obstacle_dist`` matches the clearance computed from ground truth within the lidar noise
   model: the logged value is the MINIMUM over noisy returns (3-beam median filtered; range noise
   sigma = ``lidar_noise_std`` of the run), so it sits below the noise-free clearance by up to
   ~2.5 sigma; it must never be biased HIGH (that would under-report danger).

Collisions come from INDEPENDENT ground truth (``/ground_truth/poses``, Gazebo) with the footprint
checker of 01/02 (``parakram_sim.footprint``), never from a robot's own pose. Results:
``<run_dir>/safety_acceptance_<mode>.json`` and ``safety_acceptance_<mode>_traj.csv``.
Acceptance checks are verification, not benchmark results.
"""

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

from parakram_msgs.msg import CoordStatus, Intent, RobotState, SafetyStatus
from parakram_safety.orca_filter import CSV_COLUMNS
from parakram_safety.run_paths import latest_run_dir
from parakram_sim.footprint import box_polygon, footprint_polygon, polygon_distance
from parakram_sim.grid_utils import default_grid_path, WarehouseGrid
from parakram_sim.warehouse import static_boxes
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

PEER_EXES = ('lib/parakram_coord/coordination_node', 'lib/parakram_coord/roster_helper')
LIDAR_X, TURRET_R = -0.032, 0.0508


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _sec(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def turret_polygon(x, y, yaw, n=24):
    """Return a peer's lidar turret (its only part in the scan plane) as a polygon."""
    cx, cy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
    angles = [2 * math.pi * k / n for k in range(n)]
    return [(cx + TURRET_R * math.cos(a), cy + TURRET_R * math.sin(a)) for a in angles]


class Monitor(Node):
    """Ground truth + safety status + peer-message traffic."""

    def __init__(self, robots, boxes, contact_tol):
        """Subscribe to everything the acceptance needs."""
        super().__init__('safety_acceptance')
        self.robots = robots
        self.static = [box_polygon(b.x_min, b.y_min, b.x_max, b.y_max) for b in boxes]
        self.contact_tol = contact_tol
        self.lock = threading.Lock()
        self.sim_now, self.t0 = None, None
        self.gt = {}
        self.min_rr = (math.inf, None, None)
        self.min_static = (math.inf, None, None)
        self.contacts, self._in_contact = [], {}
        self.traj, self._last_traj = [], None
        self.status = {r: None for r in robots}
        self.status_log = {r: [] for r in robots}         # (t, interventions, filter_active)
        self.peer_msgs = []                                 # (t, robot, 'state' | 'intent')
        self.start_pose = {}
        gt_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, '/ground_truth/poses', self._on_gt, gt_qos)
        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for r in robots:
            self.create_subscription(SafetyStatus, f'/{r}/safety_status',
                                     lambda m, r=r: self._on_status(r, m), 10)
            self.create_subscription(CoordStatus, f'/{r}/coord_status', self._on_coord,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
            self.create_subscription(RobotState, f'/{r}/state',
                                     lambda m, r=r: self._on_peer(r, 'state'), best)
            self.create_subscription(Intent, f'/{r}/intent',
                                     lambda m, r=r: self._on_peer(r, 'intent'), best)

    def _on_gt(self, msg):
        with self.lock:
            for t in msg.transforms:
                if t.child_frame_id in self.robots:
                    self.gt[t.child_frame_id] = (t.transform.translation.x,
                                                 t.transform.translation.y,
                                                 _yaw(t.transform.rotation))
                    self.sim_now = _sec(t.header.stamp)
            now = self.sim_now
            if self.t0 is not None and not self.start_pose and len(self.gt) == len(self.robots):
                self.start_pose = dict(self.gt)
            polys = {r: footprint_polygon(*self.gt[r]) for r in self.robots if r in self.gt}
            names = sorted(polys)
            for i, a in enumerate(names):
                for b in names[i + 1:]:
                    d = polygon_distance(polys[a], polys[b])
                    if d < self.min_rr[0]:
                        self.min_rr = (d, (a, b), now)
                    key = (a, b)
                    if d <= self.contact_tol:
                        if key not in self._in_contact:
                            self._in_contact[key] = [list(key), now, now, d]
                            self.contacts.append(self._in_contact[key])
                        c = self._in_contact[key]
                        c[2], c[3] = now, min(c[3], d)
                    else:
                        self._in_contact.pop(key, None)
                for poly in self.static:
                    d = polygon_distance(polys[a], poly)
                    if d < self.min_static[0]:
                        self.min_static = (d, a, now)
            if now is not None and (self._last_traj is None or now - self._last_traj >= 0.2):
                self._last_traj = now
                for r in names:
                    self.traj.append((now, r) + self.gt[r])

    def _on_status(self, robot, msg):
        with self.lock:
            self.status[robot] = msg
            self.status_log[robot].append((_sec(msg.stamp), msg.interventions,
                                           msg.filter_active, msg.intervention))

    def _on_coord(self, msg):
        with self.lock:
            if self.t0 is None and msg.goals_assigned >= 1:
                self.t0 = _sec(msg.stamp)

    def _on_peer(self, robot, kind):
        with self.lock:
            if self.sim_now is not None:
                self.peer_msgs.append((self.sim_now, robot, kind))

    def safety_subscriptions(self, timeout=15.0):
        """
        Topics each robot's safety nodes subscribe to (from the ROS graph).

        A node this monitor has not discovered yet raises "nonexistent node" (DDS discovery on a
        loaded host can lag by seconds); it is polled again until ``timeout`` before its entry is
        recorded as an error, which then fails the check.
        """
        out = {}
        for r in self.robots:
            for node in ('orca_filter', 'collision_monitor'):
                end = time.monotonic() + timeout
                while True:
                    try:
                        subs = self.get_subscriber_names_and_types_by_node(node, f'/{r}')
                        break
                    except Exception as exc:  # noqa: BLE001 - node not (yet) discovered
                        subs = [(f'<error: {exc}>', [])]
                        if time.monotonic() >= end:
                            break
                        time.sleep(0.5)
                out[f'/{r}/{node}'] = sorted(t for t, _ in subs)
        return out


def _peer_pids():
    pids = []
    for exe in PEER_EXES:
        out = subprocess.run(['pgrep', '-f', exe], capture_output=True, text=True)
        pids += [int(p) for p in out.stdout.split()]
    return pids


def expected_clearance(robot, poses, static_polys, max_range=3.5):
    """
    Return the clearance the robot's lidar should report, from ground truth.

    Footprint to the nearest shelf / wall / peer turret; objects beyond the lidar's range are
    ignored.
    """
    x, y, yaw = poses[robot]
    fp = footprint_polygon(x, y, yaw)
    lx, ly = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
    best = math.inf
    others = [turret_polygon(*poses[o]) for o in poses if o != robot]
    for poly in static_polys + others:
        near = min(math.hypot(px - lx, py - ly) for px, py in poly)
        if near > max_range:
            continue
        best = min(best, polygon_distance(fp, poly))
    return best


def check_logs(run_dir, robots, status, t0, t_end, traj, static_polys, noise_std):
    """
    Check acceptance item 4: the required log fields are present and consistent.

    Distance check against the noise-free ground-truth clearance: the median signed error must
    lie in ``[-(2.5 sigma + 0.02), +0.02]`` (minimum over noisy beams biases low; 0.02 m for the
    <= 0.1 s time alignment) and the 90th percentile of the absolute error below ``3.5 sigma +
    0.02``.
    """
    by_t = {}
    for row in traj:
        by_t.setdefault(round(row[0], 1), {})[row[1]] = row[2:]
    times = sorted(by_t)
    out = {}
    for r in robots:
        path = os.path.join(run_dir, f'safety_{r}.csv')
        res = {'csv': path, 'exists': os.path.isfile(path)}
        out[r] = res
        if not res['exists']:
            continue
        with open(path) as f:
            rows = list(csv.reader(f))
        res['header_ok'] = rows[0] == CSV_COLUMNS
        body = [dict(zip(rows[0], x)) for x in rows[1:] if x]
        body = [x for x in body if t0 - 1.0 <= float(x['t']) <= t_end]
        res['rows'] = len(body)
        flags = [int(x['intervention']) for x in body]
        res['csv_interventions'] = sum(1 for a, b in zip([0] + flags, flags) if b and not a)
        res['status_interventions'] = status[r].interventions if status[r] else None
        res['filter_active_rows'] = sum(int(x['filter_active']) for x in body)
        res['intervention_rows'] = sum(flags)
        numeric = all(math.isfinite(float(x[k])) for x in body
                      for k in ('cmd_in_v', 'cmd_in_w', 'cmd_out_v', 'cmd_out_w'))
        res['velocity_fields_numeric'] = numeric
        # logged distance vs ground truth, at the ground-truth samples (5 Hz)
        errs = []
        k = 0
        for x in body:
            t = float(x['t'])
            while k + 1 < len(times) and times[k + 1] <= t:
                k += 1
            if not times or abs(times[k] - t) > 0.11 or len(by_t[times[k]]) < len(robots):
                continue
            logged = float(x['min_obstacle_dist'])
            exp = expected_clearance(r, by_t[times[k]], static_polys)
            if math.isfinite(logged) and math.isfinite(exp) and exp < 1.0:
                errs.append(logged - exp)
        signed = sorted(errs)
        absolute = sorted(abs(e) for e in errs)
        res['distance_samples'] = len(errs)
        res['distance_err_median_m'] = signed[len(signed) // 2] if errs else None
        res['distance_abs_err_median_m'] = absolute[len(absolute) // 2] if errs else None
        res['distance_abs_err_p90_m'] = absolute[int(0.9 * (len(absolute) - 1))] if errs else None
        lo, hi, p90_max = -(2.5 * noise_std + 0.02), 0.02, 3.5 * noise_std + 0.02
        res['distance_tolerance'] = {'median_signed_in': [lo, hi], 'p90_abs_max': p90_max,
                                     'lidar_noise_std': noise_std}
        res['ok'] = bool(res['header_ok'] and res['rows'] > 0 and numeric and
                         res['csv_interventions'] in (res['status_interventions'],
                                                      (res['status_interventions'] or 0) - 1) and
                         errs and lo <= res['distance_err_median_m'] <= hi and
                         res['distance_abs_err_p90_m'] <= p90_max)
    return out


def run(args):
    """Monitor one acceptance run; return (passed, summary)."""
    run_dir = os.path.realpath(args.run_dir or latest_run_dir())
    robots = [f'robot{i + 1}' for i in range(args.n)]
    grid = WarehouseGrid.from_yaml(default_grid_path())
    boxes = static_boxes(grid)
    mon = Monitor(robots, boxes, args.contact_tol)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(mon)
    threading.Thread(target=ex.spin, daemon=True).start()
    log = mon.get_logger()
    summary = {'mode': args.mode, 'run_dir': run_dir, 'robots': robots, 'params': vars(args)}
    wall0 = time.monotonic()

    def since_start():
        with mon.lock:
            return None if mon.t0 is None or mon.sim_now is None else mon.sim_now - mon.t0

    try:
        # the safety nodes' subscriptions (static: created at node start), read before the run
        graph = mon.safety_subscriptions()
        log.info(f'[{args.mode}] waiting for the robots to get their goals...')
        while since_start() is None:
            if time.monotonic() - wall0 > args.start_timeout:
                summary['error'] = 'coordination (goal source) never started'
                return False, summary
            time.sleep(0.5)
        killed, kill_t = [], None
        while since_start() < args.duration:
            if args.mode == 'peers_killed' and kill_t is None and \
                    since_start() >= args.kill_peers_at:
                killed = _peer_pids()
                for pid in killed:
                    os.kill(pid, signal.SIGKILL)
                with mon.lock:
                    kill_t = mon.sim_now
                log.warn(f'SIGKILLed every peer state/intent publisher {killed} at '
                         f't={since_start():.1f}s')
            time.sleep(0.2)
        time.sleep(1.0)
        survivors = _peer_pids() if args.mode == 'peers_killed' else None
        if any(subs and subs[0].startswith('<error') for subs in graph.values()):
            graph = mon.safety_subscriptions()        # not discovered before the run: retry
    finally:
        ex.shutdown(timeout_sec=2.0)

    with mon.lock:
        t0, t_end = mon.t0, mon.sim_now
        status = dict(mon.status)
        traj = list(mon.traj)
        peer = list(mon.peer_msgs)
        moved = {r: math.hypot(mon.gt[r][0] - mon.start_pose[r][0],
                               mon.gt[r][1] - mon.start_pose[r][1])
                 for r in robots if r in mon.gt and r in mon.start_pose}
        summary.update({
            'coord_start_sim_time': t0, 'end_sim_time': t_end, 'run_length_s': t_end - t0,
            'robot_robot_contacts': [[p, s - t0, e - t0, d] for p, s, e, d in mon.contacts],
            'min_robot_robot_distance_m': mon.min_rr[0], 'min_rr_pair': mon.min_rr[1],
            'min_rr_time_s': (mon.min_rr[2] - t0) if mon.min_rr[2] else None,
            'min_robot_static_clearance_m': mon.min_static[0],
            'distance_travelled_m': moved,
            'final_gt_poses': {r: mon.gt.get(r) for r in robots},
            'safety': {r: None if s is None else {
                'safety_enabled': s.safety_enabled, 'monitor_enabled': s.monitor_enabled,
                'interventions': s.interventions, 'last_min_obstacle_dist': s.min_obstacle_dist,
                'monitor_action': s.monitor_action, 'monitor_polygon': s.monitor_polygon}
                for r, s in status.items()},
            'filter_active_ticks': {r: sum(1 for x in mon.status_log[r] if x[2]) for r in robots},
            'safety_subscriptions': graph,
        })
        if args.mode == 'peers_killed':
            before = [p for p in peer if kill_t is not None and t0 <= p[0] < kill_t]
            after = [p for p in peer if kill_t is not None and p[0] > kill_t + 1.0]
            summary.update({
                'killed_pids': killed, 'kill_time_s': (kill_t - t0) if kill_t else None,
                'surviving_peer_publishers': survivors,
                'peer_msgs_before_kill': len(before), 'peer_msgs_after_kill': len(after),
                'interventions_after_kill': {
                    r: sum(1 for t, _, _, iv in mon.status_log[r] if kill_t and t > kill_t
                           and iv) for r in robots}})
    static_polys = [box_polygon(b.x_min, b.y_min, b.x_max, b.y_max) for b in boxes]
    peer_topic = [t for subs in graph.values() for t in subs
                  if t.endswith(('/state', '/intent')) or t.startswith('/fleet')]
    safety_nodes_found = all(subs and not subs[0].startswith('<error')
                             for subs in graph.values())
    contacts = summary['robot_robot_contacts']
    on = [status[r] is not None and status[r].safety_enabled for r in robots]
    intervened = [r for r in robots if status[r] is not None and status[r].interventions > 0]
    checks = {'ran_full_duration': summary['run_length_s'] >= args.duration,
              'robots_moved': sum(1 for d in moved.values() if d > 0.3) >= 2}
    if args.mode in ('on', 'peers_killed'):
        checks['no_robot_robot_collision'] = not contacts
        checks['safety_on_everywhere'] = all(on) and all(
            status[r].monitor_enabled for r in robots)
        checks['safety_layer_intervened'] = len(intervened) >= 2
        noise_std = 0.02
        try:
            with open(os.path.join(run_dir, 'run_manifest.json')) as f:
                noise_std = float(json.load(f)['settings']['lidar_noise_std'])
        except (OSError, KeyError, ValueError):
            pass
        logs = check_logs(run_dir, robots, status, t0, t_end, traj, static_polys, noise_std)
        summary['logs'] = logs
        checks['logs_consistent'] = all(v.get('ok') for v in logs.values())
    if args.mode == 'off':
        checks['at_least_one_collision'] = len(contacts) >= 1
        checks['safety_verifiably_off'] = all(
            status[r] is not None and not status[r].safety_enabled and
            not status[r].monitor_enabled and status[r].interventions == 0 for r in robots)
    if args.mode == 'peers_killed':
        checks['peer_messages_flowed_before_kill'] = summary['peer_msgs_before_kill'] > 0
        checks['no_peer_message_after_kill'] = (summary['peer_msgs_after_kill'] == 0
                                                and not survivors)
        checks['safety_nodes_subscribe_no_peer_topic'] = safety_nodes_found and not peer_topic
        checks['safety_intervened_without_peers'] = sum(
            1 for v in summary['interventions_after_kill'].values() if v > 0) >= 2
    summary['checks'] = checks
    summary['pass'] = all(checks.values())
    with open(os.path.join(run_dir, f'safety_acceptance_{args.mode}_traj.csv'), 'w',
              newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'robot', 'gt_x', 'gt_y', 'gt_yaw'])
        for t, r, x, y, yaw in traj:
            w.writerow([f'{t - t0:.2f}', r, f'{x:.3f}', f'{y:.3f}', f'{yaw:.3f}'])
    return summary['pass'], summary


def main(argv=None):
    """Entry point."""
    ap = argparse.ArgumentParser(description='CLAUDE_CODE/03 acceptance monitor')
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--mode', choices=('on', 'off', 'peers_killed'), required=True)
    ap.add_argument('--run-dir', default=None)
    ap.add_argument('--duration', type=float, default=60.0, help='[s sim] after goals are set')
    ap.add_argument('--kill-peers-at', type=float, default=3.0, help='[s sim] (peers_killed)')
    ap.add_argument('--start-timeout', type=float, default=180.0, help='[s wall]')
    ap.add_argument('--contact-tol', type=float, default=0.01)
    args, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[sys.argv[0]] + ros_args)
    try:
        passed, summary = run(args)
    finally:
        rclpy.try_shutdown()
    with open(os.path.join(summary['run_dir'], f'safety_acceptance_{args.mode}.json'), 'w') as f:
        json.dump(summary, f, indent=2, default=str)
    keys = ('checks', 'robot_robot_contacts', 'min_robot_robot_distance_m',
            'min_robot_static_clearance_m', 'distance_travelled_m', 'safety', 'killed_pids',
            'kill_time_s', 'peer_msgs_before_kill', 'peer_msgs_after_kill',
            'interventions_after_kill', 'run_length_s')
    print(json.dumps({k: summary.get(k) for k in keys if k in summary}, indent=1, default=str))
    if 'logs' in summary:
        shown = ('header_ok', 'rows', 'csv_interventions', 'status_interventions',
                 'distance_samples', 'distance_err_median_m', 'distance_abs_err_p90_m', 'ok')
        print(json.dumps({r: {k: v[k] for k in shown if k in v}
                          for r, v in summary['logs'].items()}, indent=1, default=str))
    print(f"RESULT[{args.mode}]: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
