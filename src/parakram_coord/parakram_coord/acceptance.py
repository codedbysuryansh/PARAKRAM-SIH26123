"""
CLAUDE_CODE/02 acceptance monitor (observer only: it never commands a robot).

    ros2 run parakram_coord coord_acceptance --n 3

Watches a running ``fleet_sim`` + ``coord.launch.py`` session and checks the 02 PASS criteria:

* 0 inter-robot collisions — from INDEPENDENT ground truth (``/ground_truth/poses``, Gazebo),
  using the same footprint checker as the 01 smoke test (``parakram_sim.footprint``); never from
  a robot's own pose or state.
* no permanent deadlock — every leg assigned during the crossing window is completed.
* fully decentralized — the ``/fleet/roster`` helper is SIGKILLed mid-run; every robot must keep
  its peers, keep receiving peer intents and keep completing legs afterwards.
* ``tick_compute_ms`` p95 < 50 ms — from the robots' own ``coord_<ns>.csv`` logs.

It logs the makespan — sim time from coordination start until every robot has completed every
crossing leg assigned to it (the Benchmark #1 pre-baseline) — and, for reference only (they do
not decide PASS), the fixed-workload makespan of each robot's first ``--makespan-legs`` legs,
the throughput and how often the deadlock breaker fired. For diagnosis only it also records,
at 5 Hz, each robot's ground-truth pose next to the pose the robot itself reports on
``/<ns>/state`` (its localization error; the self-reported pose is never used for the
collision check). Results: ``<run_dir>/coord_acceptance.json``, ``coord_acceptance_legs.csv``,
``coord_acceptance_notes.csv`` (every change of a robot's coordination note) and
``coord_acceptance_traj.csv``.
Exit 0 = PASS. Acceptance checks are verification, not project results.
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

from parakram_bringup import run_manifest as rm
from parakram_comms.qos import INTENT_QOS, STATE_QOS, STATUS_QOS
from parakram_msgs.msg import CoordStatus, Intent, RobotState
from parakram_sim.footprint import box_polygon, footprint_polygon, polygon_distance
from parakram_sim.grid_utils import default_grid_path, WarehouseGrid
from parakram_sim.warehouse import static_boxes
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

ROSTER_EXE = 'lib/parakram_coord/roster_helper'


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _sec(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def percentile(values, q):
    """Linear-interpolated percentile ``q`` in [0, 100]."""
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * q / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


class Monitor(Node):
    """Collects ground truth, coordination status, intents and roster traffic."""

    def __init__(self, robots, boxes, contact_tol):
        """Subscribe to everything the acceptance needs."""
        super().__init__('coord_acceptance')
        self.robots = robots
        self.static = [(b.name, box_polygon(b.x_min, b.y_min, b.x_max, b.y_max)) for b in boxes]
        self.contact_tol = contact_tol
        self.lock = threading.Lock()
        self.sim_now = None
        self.gt = {}
        self.gt_samples = 0
        self.min_rr = (math.inf, None, None)       # distance, pair, sim time
        self.min_static = (math.inf, None, None)
        self.contacts = []                          # [pair, t_start, t_end, min_dist]
        self._in_contact = {}
        self.status = {r: None for r in robots}
        self.completions = {r: [] for r in robots}  # sim times of each completed leg
        self.assigned_at = {r: [] for r in robots}  # sim time each new goal was first seen
        self.first_active = None
        self.min_peers_after = {r: 255 for r in robots}
        self.max_blocked = {r: 0.0 for r in robots}
        self._blocked_since = {r: None for r in robots}
        self.intents = {r: [] for r in robots}      # receive sim times
        self.notes = []                             # (sim time, robot, cell, status, note)
        self.reported = {}                          # robot -> self-reported (x, y, yaw)
        self.traj = []                              # 5 Hz: t, robot, GT pose, reported pose
        self._last_traj = None
        self.roster_rx = []
        self.kill_time = None
        gt_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, '/ground_truth/poses', self._on_gt, gt_qos)
        for r in robots:
            self.create_subscription(CoordStatus, f'/{r}/coord_status',
                                     lambda m, r=r: self._on_status(r, m), STATUS_QOS)
            self.create_subscription(Intent, f'/{r}/intent',
                                     lambda m, r=r: self._on_intent(r, m), INTENT_QOS)
            self.create_subscription(RobotState, f'/{r}/state',
                                     lambda m, r=r: self._on_state(r, m), STATE_QOS)
        self.create_subscription(String, '/fleet/roster', self._on_roster,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

    def _on_gt(self, msg):
        with self.lock:
            for t in msg.transforms:
                if t.child_frame_id in self.robots:
                    self.gt[t.child_frame_id] = (t.transform.translation.x,
                                                 t.transform.translation.y,
                                                 _yaw(t.transform.rotation))
                    self.sim_now = _sec(t.header.stamp)
            self.gt_samples += 1
            now = self.sim_now
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
                for name, poly in self.static:
                    d = polygon_distance(polys[a], poly)
                    if d < self.min_static[0]:
                        self.min_static = (d, (a, name), now)
            if now is not None and (self._last_traj is None or now - self._last_traj >= 0.2):
                self._last_traj = now
                for r in names:
                    rep = self.reported.get(r)
                    self.traj.append((now, r) + self.gt[r] + (rep or (None, None, None)))

    def _on_state(self, robot, msg):
        with self.lock:
            self.reported[robot] = (msg.pose.x, msg.pose.y, msg.pose.theta)

    def _on_status(self, robot, msg):
        with self.lock:
            prev = self.status[robot]
            t = _sec(msg.stamp)
            if msg.goals_assigned >= 1 and self.first_active is None:
                self.first_active = t
            if prev is None or msg.goals_assigned > prev.goals_assigned:
                self.assigned_at[robot].extend([t] * (msg.goals_assigned -
                                                      (prev.goals_assigned if prev else 0)))
            if prev is not None and msg.goals_completed > prev.goals_completed:
                self.completions[robot].extend([t] * (msg.goals_completed -
                                                      prev.goals_completed))
            if self.kill_time is not None and t > self.kill_time:
                self.min_peers_after[robot] = min(self.min_peers_after[robot], msg.n_peers)
            if msg.blocked:
                if self._blocked_since[robot] is None:
                    self._blocked_since[robot] = t
                self.max_blocked[robot] = max(self.max_blocked[robot],
                                              t - self._blocked_since[robot])
            else:
                self._blocked_since[robot] = None
            if prev is None or msg.note != prev.note:
                self.notes.append((t, robot, msg.cell, msg.status, msg.note))
            self.status[robot] = msg

    def _on_intent(self, robot, _msg):
        with self.lock:
            if self.sim_now is not None:
                self.intents[robot].append(self.sim_now)

    def _on_roster(self, _msg):
        with self.lock:
            if self.sim_now is not None:
                self.roster_rx.append(self.sim_now)


def _roster_pids():
    out = subprocess.run(['pgrep', '-f', ROSTER_EXE], capture_output=True, text=True)
    return [int(p) for p in out.stdout.split()]


def _tick_stats(run_dir, robots, t_end):
    """Tick compute stats over the rows logged up to ``t_end`` (the end of the monitored run)."""
    per, allv = {}, []
    for r in robots:
        path = os.path.join(run_dir, f'coord_{r}.csv')
        vals = []
        if os.path.isfile(path):
            with open(path) as f:
                for row in csv.DictReader(f):
                    try:
                        if t_end is None or float(row['t']) <= t_end:
                            vals.append(float(row['tick_compute_ms']))
                    except (KeyError, ValueError):
                        pass
        per[r] = {'n': len(vals), 'p50': percentile(vals, 50), 'p95': percentile(vals, 95),
                  'max': max(vals) if vals else None}
        allv += vals
    return per, {'n': len(allv), 'p50': percentile(allv, 50), 'p95': percentile(allv, 95),
                 'max': max(allv) if allv else None}


def run(args):
    """Monitor one acceptance run; return (passed, summary)."""
    run_dir = os.path.realpath(args.run_dir or os.path.join(rm.default_log_root(), 'latest'))
    robots = [f'robot{i + 1}' for i in range(args.n)]
    grid = WarehouseGrid.from_yaml(default_grid_path())
    mon = Monitor(robots, static_boxes(grid), args.contact_tol)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(mon)
    threading.Thread(target=ex.spin, daemon=True).start()
    log = mon.get_logger()
    summary = {'run_dir': run_dir, 'robots': robots, 'params': vars(args)}
    wall0 = time.monotonic()

    def sim_since_start():
        with mon.lock:
            if mon.first_active is None or mon.sim_now is None:
                return None
            return mon.sim_now - mon.first_active

    try:
        log.info('waiting for coordination to start...')
        while sim_since_start() is None:
            if time.monotonic() - wall0 > args.start_timeout:
                summary['error'] = 'coordination never started'
                return False, summary
            time.sleep(0.5)
        log.info('coordination started; monitoring')
        roster_pids_before = _roster_pids()
        killed = []
        done_since = None
        while True:
            t = sim_since_start()
            if mon.kill_time is None and t >= args.kill_roster_at:
                pids = _roster_pids()
                for pid in pids:
                    os.kill(pid, signal.SIGKILL)
                with mon.lock:
                    mon.kill_time = mon.sim_now
                killed = pids
                log.warn(f'SIGKILLed the roster helper {pids} at t={t:.1f}s')
            with mon.lock:
                st = dict(mon.status)
            finished = t >= args.assign_duration and all(
                s is not None and s.goals_completed == s.goals_assigned and s.status == 0
                for s in st.values())
            if finished:
                done_since = done_since or t
                if t - done_since >= 3.0:
                    break
            else:
                done_since = None
            if t >= args.timeout:
                log.error(f'timeout at t={t:.1f}s')
                break
            time.sleep(0.5)
        time.sleep(1.0)
        roster_after = _roster_pids()
    finally:
        ex.shutdown(timeout_sec=2.0)

    with mon.lock:
        t0 = mon.first_active
        kill_t = mon.kill_time
        comp = {r: [c - t0 for c in mon.completions[r]] for r in robots}
        status = {r: mon.status[r] for r in robots}
        legs_after = {r: sum(1 for c in mon.completions[r] if kill_t and c > kill_t)
                      for r in robots}
        intents_after = {r: sum(1 for x in mon.intents[r] if kill_t and x > kill_t + 1.0)
                         for r in robots}
        roster_after_kill = sum(1 for x in mon.roster_rx if kill_t and x > kill_t + 1.0)
        summary.update({
            'coord_start_sim_time': t0, 'end_sim_time': mon.sim_now,
            'run_length_s': mon.sim_now - t0,
            'ground_truth_samples': mon.gt_samples,
            'min_robot_robot_distance_m': mon.min_rr[0], 'min_rr_pair': mon.min_rr[1],
            'min_rr_time': (mon.min_rr[2] - t0) if mon.min_rr[2] else None,
            'min_robot_static_clearance_m': mon.min_static[0],
            'min_static_pair': mon.min_static[1],
            'robot_robot_contacts': [[p, s - t0, e - t0, d] for p, s, e, d in mon.contacts],
            'legs_completed': {r: len(comp[r]) for r in robots},
            'legs_assigned': {r: status[r].goals_assigned if status[r] else 0 for r in robots},
            'completion_times_s': comp,
            'max_blocked_s': dict(mon.max_blocked),
            'roster_pids_before': roster_pids_before, 'roster_killed_pids': killed,
            'roster_kill_time_s': (kill_t - t0) if kill_t else None,
            'roster_pids_after': roster_after, 'roster_msgs_after_kill': roster_after_kill,
            'legs_after_kill': legs_after, 'peer_intents_after_kill': intents_after,
            'min_n_peers_after_kill': dict(mon.min_peers_after),
            'final_notes': {r: status[r].note if status[r] else None for r in robots},
            'breaker_events': {r: {stage: sum(1 for _, rr, _, _, n in mon.notes
                                              if rr == r and n == f'deadlock breaker: {stage}')
                                   for stage in ('priority bump + detour', 're-route requested')}
                               for r in robots},
            'note_changes': [(tt - t0, r, cell, st, n) for tt, r, cell, st, n in mon.notes],
            'traj': [(row[0] - t0,) + row[1:] for row in mon.traj],
        })
    loc = {}
    for r in robots:
        errs = [math.hypot(row[5] - row[2], row[6] - row[3]) for row in summary['traj']
                if row[1] == r and row[5] is not None and row[0] >= 0.0]
        loc[r] = {'p95': percentile(errs, 95), 'max': max(errs) if errs else None}
    summary['localization_error_m'] = loc      # diagnostic only (reported pose vs ground truth)
    all_done = all(status[r] is not None and status[r].goals_completed == status[r].goals_assigned
                   and status[r].goals_assigned > 0 for r in robots)
    last_leg = max((c[-1] for c in comp.values() if c), default=None)
    summary['makespan_s'] = last_leg if all_done else None
    summary['makespan_definition'] = ('sim time from coordination start until every robot had '
                                      'completed every crossing leg assigned to it')
    k = args.makespan_legs
    summary['makespan_first_k_legs_s'] = (max(comp[r][k - 1] for r in robots)
                                          if all(len(comp[r]) >= k for r in robots) else None)
    summary['makespan_first_k_legs_definition'] = (
        f'reference only: sim time from coordination start until every robot had completed its '
        f'first {k} crossing legs (a fixed {k * len(robots)}-leg workload)')
    n_legs = sum(len(c) for c in comp.values())
    summary['throughput_legs_per_min'] = 60.0 * n_legs / last_leg if last_leg else None
    per, allt = _tick_stats(run_dir, robots, summary['end_sim_time'])
    summary['tick_compute_ms'] = {'per_robot': per, 'all': allt}

    checks = {
        'no_robot_robot_collision': not summary['robot_robot_contacts'],
        'every_assigned_leg_completed': all_done,
        'roster_helper_was_running': bool(roster_pids_before),
        'roster_helper_killed': bool(killed) and not roster_after and roster_after_kill == 0,
        'coordination_continued_after_kill': all(
            legs_after[r] >= args.min_legs_after_kill and intents_after[r] > 0
            and mon.min_peers_after[r] >= len(robots) - 1 for r in robots),
        'tick_p95_below_50ms': allt['p95'] is not None and allt['p95'] < args.tick_p95_max,
        'ran_full_crossing_window': (summary['run_length_s'] or 0) >= args.assign_duration,
        'makespan_logged': summary['makespan_s'] is not None,
    }
    summary['checks'] = checks
    passed = all(checks.values())
    summary['pass'] = passed
    return passed, summary


def main(argv=None):
    """Entry point."""
    ap = argparse.ArgumentParser(description='CLAUDE_CODE/02 acceptance monitor')
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--run-dir', default=None)
    ap.add_argument('--assign-duration', type=float, default=300.0,
                    help='[s sim] crossing window (must match coord.launch.py)')
    ap.add_argument('--kill-roster-at', type=float, default=120.0, help='[s sim]')
    ap.add_argument('--timeout', type=float, default=480.0, help='[s sim]')
    ap.add_argument('--start-timeout', type=float, default=180.0, help='[s wall]')
    ap.add_argument('--contact-tol', type=float, default=0.01)
    ap.add_argument('--tick-p95-max', type=float, default=50.0)
    ap.add_argument('--makespan-legs', type=int, default=4)
    ap.add_argument('--min-legs-after-kill', type=int, default=1)
    args, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[sys.argv[0]] + ros_args)
    try:
        passed, summary = run(args)
    finally:
        rclpy.try_shutdown()
    os.makedirs(summary['run_dir'], exist_ok=True)
    traj = summary.pop('traj', [])
    with open(os.path.join(summary['run_dir'], 'coord_acceptance.json'), 'w') as f:
        json.dump(summary, f, indent=2, default=str)
    with open(os.path.join(summary['run_dir'], 'coord_acceptance_traj.csv'), 'w',
              newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'robot', 'gt_x', 'gt_y', 'gt_yaw', 'rep_x', 'rep_y', 'rep_yaw'])
        for row in traj:
            w.writerow([f'{row[0]:.2f}', row[1]] +
                       ['' if v is None else f'{v:.3f}' for v in row[2:]])
    if 'completion_times_s' in summary:
        with open(os.path.join(summary['run_dir'], 'coord_acceptance_legs.csv'), 'w',
                  newline='') as f:
            w = csv.writer(f)
            w.writerow(['robot', 'leg', 'completed_at_s'])
            for r, times in summary['completion_times_s'].items():
                for i, tc in enumerate(times, 1):
                    w.writerow([r, i, f'{tc:.2f}'])
    if 'note_changes' in summary:
        with open(os.path.join(summary['run_dir'], 'coord_acceptance_notes.csv'), 'w',
                  newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'robot', 'cell', 'status', 'note'])
            for tt, r, cell, st, n in summary['note_changes']:
                w.writerow([f'{tt:.2f}', r, cell, st, n])
    print(json.dumps({k: summary.get(k) for k in (
        'checks', 'legs_completed', 'legs_assigned', 'makespan_s', 'makespan_first_k_legs_s',
        'throughput_legs_per_min', 'min_robot_robot_distance_m', 'min_robot_static_clearance_m',
        'robot_robot_contacts', 'max_blocked_s', 'breaker_events', 'localization_error_m',
        'roster_killed_pids',
        'roster_kill_time_s', 'legs_after_kill', 'min_n_peers_after_kill', 'run_length_s')},
        indent=1, default=str))
    print('tick_compute_ms:', json.dumps(summary.get('tick_compute_ms', {}).get('all')))
    print(f"RESULT: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
