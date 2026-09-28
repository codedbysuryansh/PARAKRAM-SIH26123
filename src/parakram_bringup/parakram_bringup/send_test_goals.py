"""
CLAUDE_CODE/01 smoke test: drive every robot to its scenario goal and verify the substrate.

    ros2 run parakram_bringup send_test_goals --n 3

For each robot ``robot{i}`` it sends the scenario's ``test_goals[i-1]`` waypoints (grid cells) as
ONE ``NavigateThroughPoses`` goal, concurrently for all robots, then checks:

* every goal SUCCEEDED (the robot's own Nav2 stack drove it there);
* the robot really is at the final goal cell according to GROUND TRUTH (``/ground_truth/poses``,
  straight from Gazebo; never the robot's own state), within ``--goal-tol``;
* the robot localizes: its own map->base_footprint estimate (AMCL via its namespaced TF) agrees
  with ground truth within ``--loc-tol`` at the start and at the end;
* no contact: the minimum ground-truth footprint distance robot-robot and robot-shelf/wall stays
  above ``--contact-tol`` for the whole run;
* no TF/costmap WARN/ERROR (and no ERROR at all) was logged by any robot node on /rosout while
  navigating.

File 01 has NO coordination layer: the scenario's smoke-test routes are cell-disjoint by design.
Results go to ``<run_dir>/smoke_test_<utc>.{json,csv}`` (default run_dir: bench/logs/latest).
Exit code 0 = PASS.
"""

import argparse
import csv
import datetime
import json
import math
import os
import re
import sys
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from parakram_bringup import run_manifest as rm
from parakram_sim.footprint import box_polygon, footprint_polygon, polygon_distance
from parakram_sim.grid_utils import WarehouseGrid
from parakram_sim.robot_model import scenario as get_scenario
from parakram_sim.warehouse import static_boxes
from rcl_interfaces.msg import Log
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
import tf2_ros

# action_msgs/GoalStatus constants (explicit: rclpy exposes msg constants via the metaclass).
STATUS_NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED',
                5: 'CANCELED', 6: 'ABORTED'}
# rcl_interfaces/Log severities (the msg field is a `byte`, i.e. bytes in Python).
LOG_WARN, LOG_ERROR = 30, 40
TF_RE = re.compile(r'(?i)(\btf\b|tf2|transform|extrapolat|lookup|frame)')
COSTMAP_RE = re.compile(r'(?i)costmap')


def yaw_of(q):
    """Yaw of a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    """``(z, w)`` of a yaw-only quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SmokeTest(Node):
    """Collects ground truth, per-robot TF and /rosout while the goals execute."""

    def __init__(self, robots, boxes):
        """Subscribe to ground truth, each robot's TF, and /rosout."""
        # Wall-clock node: no /clock subscription. Simulation time is read from the
        # Gazebo-stamped ground-truth samples instead (see sim_now()).
        super().__init__('send_test_goals', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        self.robots = robots
        self.static_polys = [(b.name, box_polygon(b.x_min, b.y_min, b.x_max, b.y_max))
                             for b in boxes]
        self.lock = threading.Lock()
        self.gt = {}                       # ns -> (x, y, yaw, stamp_sec)
        self.gt_stamp = None               # latest ground-truth (= simulation) time [s]
        self.monitoring = False
        self.min_pair = (math.inf, None)   # (distance, (ns_a, ns_b))
        self.min_static = (math.inf, None)  # (distance, (ns, box))
        self.gt_samples = 0
        self.logs = []                     # (phase, level, logger, msg)
        self.phase = 'startup'
        self.buffers = {}
        self.nav_clients = {}

        gt_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, '/ground_truth/poses', self._on_gt, gt_qos)
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for ns in robots:
            buf = tf2_ros.Buffer(cache_time=Duration(seconds=60.0))
            self.buffers[ns] = buf
            self.create_subscription(
                TFMessage, f'/{ns}/tf',
                lambda m, b=buf: [b.set_transform(t, 'smoke') for t in m.transforms], tf_qos)
            self.create_subscription(
                TFMessage, f'/{ns}/tf_static',
                lambda m, b=buf: [b.set_transform_static(t, 'smoke') for t in m.transforms],
                static_qos)
            self.nav_clients[ns] = ActionClient(self, NavigateThroughPoses,
                                                f'/{ns}/navigate_through_poses')
        log_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Log, '/rosout', self._on_log, log_qos)

    # ------------------------------------------------------------------ callbacks
    def _on_gt(self, msg):
        with self.lock:
            for t in msg.transforms:
                if t.child_frame_id in self.robots:
                    stamp = t.header.stamp.sec + 1e-9 * t.header.stamp.nanosec
                    self.gt[t.child_frame_id] = (t.transform.translation.x,
                                                 t.transform.translation.y,
                                                 yaw_of(t.transform.rotation), stamp)
                    self.gt_stamp = stamp
            self.gt_samples += 1
            if not self.monitoring:
                return
            polys = {ns: footprint_polygon(*self.gt[ns][:3]) for ns in self.robots
                     if ns in self.gt}
            names = sorted(polys)
            for i, a in enumerate(names):
                for b in names[i + 1:]:
                    d = polygon_distance(polys[a], polys[b])
                    if d < self.min_pair[0]:
                        self.min_pair = (d, (a, b))
                for box_name, box_poly in self.static_polys:
                    d = polygon_distance(polys[a], box_poly)
                    if d < self.min_static[0]:
                        self.min_static = (d, (a, box_name))

    def _on_log(self, msg):
        level = msg.level if isinstance(msg.level, int) else ord(msg.level)
        if level < LOG_WARN:
            return
        if not (msg.name.startswith('robot') or msg.name.startswith('map_server')):
            return
        with self.lock:
            self.logs.append((self.phase, level, msg.name, msg.msg))

    # ------------------------------------------------------------------ helpers
    def sim_now(self):
        """Latest simulation time [s] seen on /ground_truth/poses (None before the first)."""
        with self.lock:
            return self.gt_stamp

    def gt_pose(self, ns):
        """Latest ground-truth ``(x, y, yaw, stamp)`` of ``ns`` or None."""
        with self.lock:
            return self.gt.get(ns)

    def estimate(self, ns):
        """Robot's own map->base_footprint estimate ``(x, y, yaw)`` or None."""
        try:
            t = self.buffers[ns].lookup_transform('map', 'base_footprint', Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_of(t.transform.rotation)

    def loc_error(self, ns):
        """Position error [m] between the robot's estimate and ground truth (None if n/a)."""
        est, gt = self.estimate(ns), self.gt_pose(ns)
        if est is None or gt is None:
            return None
        return math.hypot(est[0] - gt[0], est[1] - gt[1])


def _pose_stamped(grid, cell):
    r, c, yaw_deg = cell
    x, y = grid.cell_to_world(r, c)
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = quat_from_yaw(math.radians(yaw_deg))
    return p


def _wait(pred, timeout, period=0.2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(period)
    return pred()


def run(args):
    """Execute the smoke test; return (passed, summary dict)."""
    run_dir = os.path.realpath(args.run_dir or os.path.join(rm.default_log_root(), 'latest'))
    manifest = rm.load_manifest(run_dir) if os.path.isfile(
        os.path.join(run_dir, 'run_manifest.json')) else {}
    scenario_name = args.scenario or manifest.get('scenario', 'intersection')
    if manifest and args.scenario and args.scenario != manifest.get('scenario'):
        raise SystemExit(f'--scenario {args.scenario} != running scenario '
                         f"{manifest.get('scenario')}")
    if manifest and args.n > manifest.get('n_robots', args.n):
        raise SystemExit(f"--n {args.n} > n_robots {manifest.get('n_robots')} of the run")

    from ament_index_python.packages import get_package_share_directory
    grid = WarehouseGrid.from_yaml(args.grid or os.path.join(
        get_package_share_directory('parakram_sim'), 'config', 'warehouse_grid.yaml'))
    sc = get_scenario(grid, scenario_name)
    goals = sc.get('test_goals', [])
    if args.n > len(goals):
        raise SystemExit(f"scenario '{scenario_name}' has test goals for {len(goals)} robots")
    robots = [f'robot{i + 1}' for i in range(args.n)]

    node = SmokeTest(robots, static_boxes(grid))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    log = node.get_logger()
    summary = {'run_id': manifest.get('run_id'), 'run_dir': run_dir, 'scenario': scenario_name,
               'n_robots': args.n, 'robots': {}, 'checks': {}}
    try:
        # ---------------------------------------------------------------- readiness
        log.info(f'waiting for {robots} (action servers, ground truth, localization TF)...')
        ready = {
            'action_servers': _wait(lambda: all(
                node.nav_clients[ns].server_is_ready() for ns in robots), args.server_timeout),
            'ground_truth': _wait(lambda: all(node.gt_pose(ns) for ns in robots), 30.0),
            'localization_tf': _wait(lambda: all(node.estimate(ns) for ns in robots), 60.0),
        }
        summary['checks']['ready'] = ready
        if not all(ready.values()):
            missing = {k: [ns for ns in robots if not {
                'action_servers': node.nav_clients[ns].server_is_ready(),
                'ground_truth': node.gt_pose(ns) is not None,
                'localization_tf': node.estimate(ns) is not None}[k]]
                for k, ok in ready.items() if not ok}
            summary['checks']['not_ready'] = missing
            log.error(f'not ready: {missing}')
            return False, summary
        start_loc = {ns: node.loc_error(ns) for ns in robots}

        # ---------------------------------------------------------------- send goals
        with node.lock:
            node.monitoring = True
            node.phase = 'navigating'
        t_sim0 = node.sim_now()
        t_wall0 = time.monotonic()
        handles, results, feedback = {}, {}, {ns: {} for ns in robots}
        done = {ns: threading.Event() for ns in robots}

        def on_feedback(ns):
            def cb(fb):
                f = fb.feedback
                feedback[ns] = {'recoveries': int(f.number_of_recoveries),
                                'poses_remaining': int(f.number_of_poses_remaining),
                                'distance_remaining': float(f.distance_remaining)}
            return cb

        for i, ns in enumerate(robots):
            goal = NavigateThroughPoses.Goal()
            goal.poses = [_pose_stamped(grid, cell) for cell in goals[i]]
            fut = node.nav_clients[ns].send_goal_async(goal, feedback_callback=on_feedback(ns))

            def on_accept(f, ns=ns):
                h = f.result()
                handles[ns] = h
                if not h.accepted:
                    results[ns] = (GoalStatus.STATUS_ABORTED, None, 'goal rejected', 0.0)
                    done[ns].set()
                    return
                res_fut = h.get_result_async()

                def on_result(rf, ns=ns):
                    r = rf.result()
                    dt = node.sim_now() - t_sim0
                    results[ns] = (r.status, int(r.result.error_code), r.result.error_msg, dt)
                    done[ns].set()
                res_fut.add_done_callback(on_result)
            fut.add_done_callback(on_accept)
            log.info(f'{ns}: goal sent, waypoints {goals[i]}')

        finished = _wait(lambda: all(e.is_set() for e in done.values()), args.timeout, 0.5)
        if not finished:
            for ns, h in handles.items():
                if not done[ns].is_set():
                    log.error(f'{ns}: timed out after {args.timeout} s -> cancelling')
                    h.cancel_goal_async()
        wall = time.monotonic() - t_wall0
        time.sleep(1.0)  # let the last ground-truth / TF samples arrive
        with node.lock:
            node.monitoring = False
            node.phase = 'after'

        # ---------------------------------------------------------------- evaluate
        all_ok = finished
        for i, ns in enumerate(robots):
            status, err_code, err_msg, sim_dt = results.get(
                ns, (GoalStatus.STATUS_UNKNOWN, None, 'no result', None))
            r, c, _ = goals[i][-1]
            gx, gy = grid.cell_to_world(r, c)
            gt = node.gt_pose(ns)
            goal_err = math.hypot(gt[0] - gx, gt[1] - gy) if gt else None
            end_loc = node.loc_error(ns)
            final_cell = grid.world_to_cell(gt[0], gt[1]) if gt else None
            ok = (status == GoalStatus.STATUS_SUCCEEDED
                  and goal_err is not None and goal_err <= args.goal_tol
                  and start_loc[ns] is not None and start_loc[ns] <= args.loc_tol
                  and end_loc is not None and end_loc <= args.loc_tol)
            all_ok &= ok
            summary['robots'][ns] = {
                'status': STATUS_NAMES.get(status, str(status)), 'error_code': err_code,
                'error_msg': err_msg, 'sim_time_s': sim_dt,
                'goal_cell': [r, c], 'final_cell_ground_truth': list(final_cell or []),
                'final_goal_error_m': goal_err, 'start_localization_error_m': start_loc[ns],
                'end_localization_error_m': end_loc,
                'recoveries': feedback[ns].get('recoveries'), 'pass': ok}

        min_pair, pair = node.min_pair
        min_static, static_who = node.min_static
        contact_ok = min_pair > args.contact_tol and min_static > args.contact_tol
        with node.lock:
            nav_logs = [entry for entry in node.logs if entry[0] == 'navigating']
            all_logs = list(node.logs)
        tf_cm = [e for e in nav_logs if TF_RE.search(e[3]) or COSTMAP_RE.search(e[3])
                 or 'costmap' in e[2]]
        errors = [e for e in nav_logs if e[1] >= LOG_ERROR]
        logs_ok = not tf_cm and not errors
        all_ok &= contact_ok and logs_ok
        summary['checks'].update({
            'all_goals_finished_in_time': finished, 'wall_time_s': wall,
            'min_robot_robot_distance_m': min_pair, 'min_robot_robot_pair': pair,
            'min_robot_static_clearance_m': min_static, 'min_static_pair': static_who,
            'contact_tolerance_m': args.contact_tol, 'no_contact': contact_ok,
            'ground_truth_samples': node.gt_samples,
            'navigating_tf_or_costmap_warn_error': [list(e) for e in tf_cm],
            'navigating_error_logs': [list(e) for e in errors],
            'warn_error_logs_all_phases': [list(e) for e in all_logs],
            'no_tf_costmap_errors': logs_ok,
            'tolerances': {'goal_m': args.goal_tol, 'localization_m': args.loc_tol},
        })
        return all_ok, summary
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()


def _write(summary, passed, run_dir):
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    summary['pass'] = passed
    summary['finished_utc'] = stamp
    os.makedirs(run_dir, exist_ok=True)
    jpath = os.path.join(run_dir, f'smoke_test_{stamp}.json')
    with open(jpath, 'w') as f:
        json.dump(summary, f, indent=2, default=str)
    cpath = os.path.join(run_dir, f'smoke_test_{stamp}.csv')
    cols = ['run_id', 'robot', 'status', 'error_code', 'sim_time_s', 'goal_cell',
            'final_cell_ground_truth', 'final_goal_error_m', 'start_localization_error_m',
            'end_localization_error_m', 'recoveries', 'pass']
    with open(cpath, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        for ns, r in summary['robots'].items():
            w.writerow([summary.get('run_id'), ns] + [r.get(k) for k in cols[2:]])
    return jpath, cpath


def _fmt(v, nd=3):
    return '-' if v is None else (f'{v:.{nd}f}' if isinstance(v, float) else str(v))


def main(argv=None):
    """Entry point."""
    parser = argparse.ArgumentParser(description='CLAUDE_CODE/01 smoke test')
    parser.add_argument('--n', type=int, default=3, help='number of robots (robot1..robotN)')
    parser.add_argument('--scenario', default=None, help='default: the running scenario')
    parser.add_argument('--run-dir', default=None, help='default: bench/logs/latest')
    parser.add_argument('--grid', default=None)
    parser.add_argument('--timeout', type=float, default=300.0, help='[s wall] for all goals')
    parser.add_argument('--server-timeout', type=float, default=180.0)
    parser.add_argument('--goal-tol', type=float, default=0.20,
                        help='[m] ground-truth distance to the goal cell centre')
    parser.add_argument('--loc-tol', type=float, default=0.15,
                        help='[m] |own estimate - ground truth|')
    parser.add_argument('--contact-tol', type=float, default=0.01,
                        help='[m] min footprint distance counted as no contact')
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=[sys.argv[0]] + ros_args)
    try:
        passed, summary = run(args)
    finally:
        rclpy.try_shutdown()
    jpath, cpath = _write(summary, passed, summary['run_dir'])

    print('\n=== CLAUDE_CODE/01 smoke test ===')
    print(f"run_id={summary.get('run_id')} scenario={summary['scenario']} n={summary['n_robots']}")
    for ns, r in summary['robots'].items():
        print(f"  {ns}: {r['status']:<9} sim_t={_fmt(r['sim_time_s'], 1)}s "
              f"goal_err={_fmt(r['final_goal_error_m'])}m "
              f"loc_err start/end={_fmt(r['start_localization_error_m'])}/"
              f"{_fmt(r['end_localization_error_m'])}m recoveries={r['recoveries']} "
              f"-> {'PASS' if r['pass'] else 'FAIL'}")
    ch = summary['checks']
    if 'min_robot_robot_distance_m' in ch:
        print(f"  min footprint distance robot-robot={_fmt(ch['min_robot_robot_distance_m'])}m "
              f"robot-static={_fmt(ch['min_robot_static_clearance_m'])}m "
              f"(contact if <= {ch['contact_tolerance_m']} m) -> "
              f"{'OK' if ch['no_contact'] else 'CONTACT'}")
        print('  TF/costmap WARN+ERROR while navigating: '
              f"{len(ch['navigating_tf_or_costmap_warn_error'])}, ERROR logs while navigating: "
              f"{len(ch['navigating_error_logs'])} -> "
              f"{'OK' if ch['no_tf_costmap_errors'] else 'FAIL'}")
        for e in ch['navigating_tf_or_costmap_warn_error'] + ch['navigating_error_logs']:
            print(f'    [{e[1]}] {e[2]}: {e[3][:160]}')
    elif 'ready' in ch:
        print(f"  readiness: {ch['ready']} missing={ch.get('not_ready')}")
    print(f"RESULT: {'PASS' if passed else 'FAIL'}   ({jpath})")
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
