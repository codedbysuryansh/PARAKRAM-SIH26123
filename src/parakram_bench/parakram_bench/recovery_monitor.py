"""
One CLAUDE_CODE/06 recovery trial: observer, fault injector and ground-truth referee.

    python3 -m parakram_bench.recovery_monitor --run-dir <dir> --kind s3|s4|blackout

Run next to a ``junction_stream`` fleet (coordination, tasks, fault layer). Observer
subscriptions only (no loss applies to them); it never commands a live robot.

* s3 (silent kill): after the warm-up, waits for a robot that HOLDS a task and a reservation on
  the junction while its body is outside it and another robot waits for the junction (fallback
  after ``--trigger-primary`` s: another robot plans through it); then kills that robot's brain
  (SIGKILL of its coordination, auction and fault nodes: no goodbye message) and cancels its Nav2
  goal so its body stays where it is, like a dead robot. Then it observes ``--post-window`` s.
* s4 (partition): isolates ONE robot holding a task (100 % loss on its links only, through
  ``/fleet/fault_injection``) for ``--partition`` s, then observes the heal.
* blackout: every link down for ``--blackout`` s with every robot alive (false-positive test).

Ground truth: Gazebo ``/ground_truth/poses``; a collision = footprint distance <= ``contact_tol``
(the 02/03/05 checker), every pair including the dead body. Timings come from the fleet's own
recovery events (``/fleet/recovery_event``), the awards / completions it publishes and ground
truth. Liveness restoration (``restoration_s``) = kill -> a survivor HOLDS movement authority
over a cell the victim had reserved (space reclaimed and a robot sent through it); the trigger
keeps that survivor within claiming reach, so the approach travel stays out of it. For
reference, ``entered_s`` = kill -> a survivor's footprint enters the contested junction.
Writes ``<run_dir>/recovery_trial.json``. Simulation evidence, not a hardware claim.
"""

import argparse
from collections import deque
import json
import math
import os
import signal
import sys
import threading
import time

from action_msgs.srv import CancelGoal
from parakram_comms.link_loss import FAULT_TOPIC
from parakram_comms.qos import INTENT_QOS, STATUS_QOS, TASK_POOL_QOS
from parakram_msgs.msg import (Award, CoordStatus, Intent, RecoveryEvent, TaskComplete)
from parakram_sim.footprint import box_polygon, footprint_polygon, polygon_distance
from parakram_sim.grid_utils import default_grid_path, WarehouseGrid
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

JUNCTION = (5, 6)
BRAIN = ('parakram_coord/coordination_node', 'parakram_tasks/auction_node',
         'parakram_fault/heartbeat_node', 'parakram_fault/watchdog_node',
         'parakram_fault/recovery_coordinator')


def _sec(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def brain_pids(robot):
    """Return the pids of ``robot``'s coordination / task / fault processes (from /proc)."""
    out = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if f'__ns:=/{robot} ' in cmd + ' ' and any(b in cmd for b in BRAIN):
            out.append(int(pid))
    return out


class TrialMonitor(Node):
    """Collects ground truth and the fleet's recovery evidence."""

    def __init__(self, robots, grid, contact_tol):
        """Subscribe to everything the trial needs."""
        super().__init__('recovery_monitor')
        self.robots, self.grid, self.tol = robots, grid, contact_tol
        self.lock = threading.RLock()         # re-entrant: helpers lock too
        self.sim_now = None
        self.gt, self.gt_hist = {}, {r: [] for r in robots}
        x, y = grid.cell_to_world(*JUNCTION)
        h = grid.resolution / 2.0
        self.jbox = box_polygon(x - h, y - h, x + h, y + h)
        self.j_index = grid.cell_to_index(*JUNCTION)
        self.in_j = {r: False for r in robots}
        self.j_entries = []                               # (t, robot) footprint enters J
        self.contacts, self._in_contact = [], {}
        self.min_rr = (math.inf, None, None)
        self.intent, self.status = {}, {}
        self.holding = {r: None for r in robots}          # robot -> (task_id, since)
        self.last_intent_t = {}
        self.hold_intervals = []                          # (task_id, robot, t0, t1)
        self.events, self.awards, self.completions = [], [], []
        rel = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, '/ground_truth/poses', self._on_gt,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        for r in robots:
            self.create_subscription(Intent, f'/{r}/intent',
                                     lambda m, r=r: self._on_intent(r, m), INTENT_QOS)
            self.create_subscription(CoordStatus, f'/{r}/coord_status',
                                     lambda m, r=r: self._put(self.status, r, m), STATUS_QOS)
        self.create_subscription(RecoveryEvent, '/fleet/recovery_event', self._on_event, rel)
        self.create_subscription(Award, '/fleet/award', self._on_award, TASK_POOL_QOS)
        self.create_subscription(TaskComplete, '/fleet/task_complete', self._on_complete,
                                 TASK_POOL_QOS)
        self.pub_fault = self.create_publisher(
            String, FAULT_TOPIC, QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_event = self.create_publisher(RecoveryEvent, '/fleet/recovery_event', rel)

    def _put(self, table, robot, msg):
        with self.lock:
            table[robot] = msg

    def _on_intent(self, robot, m):
        """Keep the latest intent and the intervals over which it carried each task."""
        t = _sec(m.stamp)
        with self.lock:
            self.intent[robot] = m
            self.last_intent_t[robot] = t
            cur = self.holding[robot]
            if cur is not None and cur[0] != m.task_id:
                self.hold_intervals.append((cur[0], robot, cur[1], t))
                cur = None
            self.holding[robot] = cur if cur is not None else (
                (m.task_id, t) if m.task_id else None)

    def _on_event(self, m):
        with self.lock:
            self.events.append((m.t_event, m.kind, m.robot_id, m.dead_id, m.task_id, m.detail))

    def _on_award(self, m):
        with self.lock:
            self.awards.append((self.sim_now, m.task_id, m.winner_id, int(m.seq),
                                m.announcer_id))

    def _on_complete(self, m):
        with self.lock:
            self.completions.append((_sec(m.stamp), m.task_id, m.robot_id, int(m.seq)))

    def _on_gt(self, msg):
        with self.lock:
            for t in msg.transforms:
                if t.child_frame_id in self.robots:
                    self.sim_now = _sec(t.header.stamp)
                    pose = (t.transform.translation.x, t.transform.translation.y,
                            _yaw(t.transform.rotation))
                    self.gt[t.child_frame_id] = pose
                    hist = self.gt_hist[t.child_frame_id]
                    if not hist or self.sim_now - hist[-1][0] >= 0.1:
                        hist.append((self.sim_now, pose[0], pose[1]))
            now = self.sim_now
            polys = {r: footprint_polygon(*self.gt[r]) for r in self.robots if r in self.gt}
            names = sorted(polys)
            for i, a in enumerate(names):
                inside = polygon_distance(polys[a], self.jbox) <= 0.0
                if inside and not self.in_j[a]:
                    self.j_entries.append((now, a))
                self.in_j[a] = inside
                for b in names[i + 1:]:
                    d = polygon_distance(polys[a], polys[b])
                    if d < self.min_rr[0]:
                        self.min_rr = (d, (a, b), now)
                    key = (a, b)
                    if d <= self.tol:
                        if key not in self._in_contact:
                            self._in_contact[key] = [list(key), now, now, d]
                            self.contacts.append(self._in_contact[key])
                        c = self._in_contact[key]
                        c[2], c[3] = now, min(c[3], d)
                    else:
                        self._in_contact.pop(key, None)

    # ------------------------------------------------------------------ helpers
    def now(self):
        """Return the latest ground-truth sim time."""
        with self.lock:
            return self.sim_now

    def speed(self, robot, span=0.5):
        """Ground-truth speed of ``robot`` over the last ``span`` s (m/s)."""
        with self.lock:
            hist = self.gt_hist[robot]
            if len(hist) < 2:
                return 0.0
            t1, x1, y1 = hist[-1]
            old = [h for h in hist if h[0] <= t1 - span]
            if not old:
                return 0.0
            t0, x0, y0 = old[-1]
            return math.hypot(x1 - x0, y1 - y0) / max(t1 - t0, 1e-3)

    def event(self, kind, robot, detail='', task_id=''):
        """Publish a bench-side recovery record, observer data, to the fleet topic."""
        now = self.now() or 0.0
        ev = RecoveryEvent()
        ev.stamp.sec, ev.stamp.nanosec = int(now), int((now - int(now)) * 1e9)
        ev.kind, ev.robot_id, ev.dead_id, ev.task_id = kind, 'bench', robot, task_id
        ev.t_event, ev.detail = float(now), detail
        ev.t_kill = ev.t_lease_expire = ev.t_space_reclaimed = ev.t_task_reassigned = math.nan
        self.pub_event.publish(ev)

    def partition(self, robot, t0, t1):
        """Cut every link of ``robot`` ('*': all) for [t0, t1) sim s."""
        self.pub_fault.publish(String(data=json.dumps({'partition': robot, 't0': t0,
                                                       't1': t1})))

    def static_path(self, start, goal):
        """Shortest grid path start -> goal ignoring robots (BFS), [] if none."""
        if start == goal:
            return [start]
        prev, queue = {start: None}, deque([start])
        while queue:
            c = queue.popleft()
            for n in self.grid.neighbors(*c):
                if n not in prev:
                    prev[n] = c
                    if n == goal:
                        path = [n]
                        while prev[path[-1]] is not None:
                            path.append(prev[path[-1]])
                        return path[::-1]
                    queue.append(n)
        return []

    def s3_candidate(self, primary):
        """Return ``(victim, waiter)`` if a robot blocks the junction now, else None."""
        with self.lock:
            intents, status, gt = dict(self.intent), dict(self.status), dict(self.gt)
        for v in self.robots:
            iv = intents.get(v)
            if iv is None or not iv.task_id or self.j_index not in iv.reserved_cells or \
                    v not in gt or self.in_j[v]:
                continue
            if polygon_distance(footprint_polygon(*gt[v]), self.jbox) < 0.03:
                continue
            for w in self.robots:
                iw = intents.get(w)
                if w == v or iw is None or w not in gt or self.in_j[w] or \
                        self.j_index in iw.reserved_cells:
                    continue                             # w holds (or is in) J: not waiting
                # a yielding robot never plans through a cell a higher-priority peer holds:
                # its NEED is its static route to its goal (what the lease blocks)
                st = status.get(w)
                if st is None or st.goal_cell < 0:
                    continue
                here = self.grid.world_to_cell(gt[w][0], gt[w][1])
                route = self.static_path(here, self.grid.index_to_cell(st.goal_cell))
                if JUNCTION not in route[1:4]:
                    continue
                if not primary or st.blocked or self.speed(w) < 0.03:
                    return v, w
        return None


def kill(node, victim):
    """Silently kill ``victim``'s brain and stop its body; return (t_kill, pids)."""
    pids = brain_pids(victim)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    t_kill = node.now()
    client = node.create_client(CancelGoal, f'/{victim}/navigate_through_poses/_action/'
                                            'cancel_goal')
    if client.wait_for_service(timeout_sec=2.0):
        client.call_async(CancelGoal.Request())     # zero goal id + stamp: cancel every goal
    return t_kill, pids


def moved(mon, robot, t0, t1):
    """Ground-truth displacement of ``robot`` within [t0, t1] (m, largest from its t0 pose)."""
    with mon.lock:
        hist = [h for h in mon.gt_hist.get(robot, []) if t0 <= h[0] <= t1]
    if len(hist) < 2:
        return 0.0
    return max(math.hypot(h[1] - hist[0][1], h[2] - hist[0][2]) for h in hist)


def path_length(mon, robot, t0, t1):
    """Ground-truth distance ``robot`` travelled within [t0, t1] (m)."""
    with mon.lock:
        hist = [h for h in mon.gt_hist.get(robot, []) if t0 <= h[0] <= t1]
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(hist, hist[1:]))


def concurrent_holders(mon):
    """
    Pairs of robots whose intents carried the SAME task over overlapping times.

    Returns ``[(task, robot_a, robot_b, t0, t1, moved_a, moved_b)]``; both moving during the
    overlap (> 5 cm each) is concurrent EXECUTION, i.e. an at-most-once violation.
    """
    with mon.lock:
        spans = list(mon.hold_intervals) + [
            (h[0], r, h[1], mon.last_intent_t.get(r, h[1])) for r, h in mon.holding.items()
            if h is not None]
    out = []
    for i, (task, a, a0, a1) in enumerate(spans):
        for task_b, b, b0, b1 in spans[i + 1:]:
            if task_b != task or b == a:
                continue
            t0, t1 = max(a0, b0), min(a1, b1)
            if t1 > t0:
                out.append((task, a, b, t0, t1, moved(mon, a, t0, t1), moved(mon, b, t0, t1)))
    return out


def _first(values):
    values = [v for v in values if v is not None]
    return min(values) if values else None


def summarize(mon, args, t_event, victim, task, trigger):
    """Compute the trial result from what the monitor collected."""
    with mon.lock:
        events = list(mon.events)
        awards = list(mon.awards)
        completions = list(mon.completions)
        entries = list(mon.j_entries)
        contacts = [list(c) for c in mon.contacts]
        min_rr = mon.min_rr
        end = mon.sim_now
    survivors = [r for r in mon.robots if r != victim]
    about = [e for e in events if e[3] == victim and e[0] >= t_event - 0.05]
    per_survivor = {}
    for t, kind, rep, *_ in about:
        if kind in ('lease_expired', 'release') and rep in survivors:
            per_survivor.setdefault(rep, t)
    reassigned = _first([a[0] for a in awards if a[1] == task and a[2] != victim and
                         a[0] is not None and a[0] >= t_event]) if task else None
    by_task = {}
    for t, task_id, robot, seq in completions:
        by_task.setdefault(task_id, set()).add(robot)
    dup = sorted(t for t, robots in by_task.items() if len(robots) > 1)
    n_done = sum(1 for c in completions if c[0] >= t_event and c[0] <= t_event + args.post_window)
    overlap = concurrent_holders(mon)
    executing = [o for o in overlap if o[5] > 0.05 and o[6] > 0.05]
    res = {
        'kind': args.kind, 'trigger': trigger, 'victim': victim, 'victim_task': task,
        't_event': t_event, 'end_sim_time': end, 'post_window_s': args.post_window,
        'contacts': contacts, 'n_contacts': len(contacts),
        'min_rr_distance_m': min_rr[0] if math.isfinite(min_rr[0]) else None,
        'min_rr_pair': min_rr[1],
        'completions': len(completions), 'completed_tasks': sorted(by_task),
        'duplicate_completions': dup,
        'concurrent_holding': overlap, 'concurrent_execution': executing,
        'tasks_completed_post': n_done,
        'throughput_post_tasks_per_min': 60.0 * n_done / args.post_window,
        'lease_lost_events': [(e[0], e[2]) for e in events
                              if e[1] == 'lease_lost' and e[0] >= t_event - 0.05],
        'events': events,
    }
    if args.kind == 's3':
        t_first = _first(per_survivor.values())
        t_last = max(per_survivor.values()) if len(per_survivor) == len(survivors) else None
        t_reclaim = _first([e[0] for e in about if e[1] == 'space_reclaimed'])
        t_detect = _first([e[0] for e in about if e[1] in ('suspect', 'dead')])
        t_dead = _first([e[0] for e in about if e[1] == 'dead'])
        t_resume = _first([t for t, r in entries if r != victim and t >= t_event])
        res.update({
            't_kill': t_event,
            't_lease_expire_first': t_first, 't_lease_expire_last': t_last,
            't_space_reclaimed': t_reclaim, 't_resume': t_resume,
            't_detect': t_detect, 't_dead': t_dead, 't_task_reassigned': reassigned,
            'lease_expire_s': None if t_first is None else t_first - t_event,
            'lease_expire_all_s': None if t_last is None else t_last - t_event,
            'space_reclaimed_s': None if t_reclaim is None else t_reclaim - t_event,
            'restoration_s': None if t_reclaim is None else t_reclaim - t_event,
            'entered_s': None if t_resume is None else t_resume - t_event,
            'detect_s': None if t_detect is None else t_detect - t_event,
            'task_reassigned_s': None if reassigned is None else reassigned - t_event,
            'restored': t_reclaim is not None,
        })
    else:
        part = victim if args.kind == 's4' else '*'
        t_heal = t_event + (args.partition if args.kind == 's4' else args.blackout)
        moved = None
        if args.kind == 's4' and victim in mon.gt_hist:
            with mon.lock:
                hist = [h for h in mon.gt_hist[victim]
                        if t_event + 3.0 <= h[0] <= t_heal]
            if len(hist) > 1:
                moved = max(math.hypot(h[1] - hist[0][1], h[2] - hist[0][2]) for h in hist)
        res.update({
            'partitioned': part, 't_heal': t_heal,
            'victim_moved_while_isolated_m': moved,
            'travelled_during_m': {r: round(path_length(mon, r, t_event, t_heal), 3)
                                   for r in mon.robots},
            'lease_expired_about_victim': sorted(per_survivor.items()),
            'peer_back': [(e[0], e[2]) for e in about if e[1] == 'peer_back'],
            'completions_during': sum(1 for c in completions if t_event <= c[0] < t_heal),
            'victim_task_completed_by': sorted(by_task.get(task, [])) if task else [],
            't_task_reassigned': reassigned,
        })
    return res


def publish_summary(mon, result, loss):
    """CLAUDE_CODE/06 recovery record on /fleet/recovery_event (kind 'summary')."""
    def t(key):
        v = result.get(key)
        return float(v) if v is not None else math.nan
    now = mon.now() or 0.0
    ev = RecoveryEvent()
    ev.stamp.sec, ev.stamp.nanosec = int(now), int((now - int(now)) * 1e9)
    ev.kind, ev.robot_id = 'summary', 'bench'
    ev.dead_id, ev.task_id = str(result.get('victim') or ''), str(result.get('victim_task') or '')
    ev.loss_level, ev.t_event = float(loss), float(now)
    ev.t_kill = t('t_event')
    ev.t_lease_expire = t('t_lease_expire_last')
    ev.t_space_reclaimed = t('t_space_reclaimed')
    ev.t_task_reassigned = t('t_task_reassigned')
    ev.detail = (f"kind {result.get('kind')}, contacts {result.get('n_contacts')}, duplicates "
                 f"{len(result.get('duplicate_completions') or [])}")
    mon.pub_event.publish(ev)


def run(args):
    """Run one trial; return the result dict."""
    robots = [f'robot{i + 1}' for i in range(args.n)]
    grid = WarehouseGrid.from_yaml(default_grid_path())
    mon = TrialMonitor(robots, grid, args.contact_tol)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(mon)
    threading.Thread(target=ex.spin, daemon=True).start()
    log = mon.get_logger()
    wall0 = time.monotonic()
    result = {'robots': robots, 'params': vars(args)}
    try:
        while True:                                     # fleet running and tasks flowing
            with mon.lock:
                ready = len(mon.status) == len(robots) and mon.awards and mon.sim_now
            if ready:
                break
            if time.monotonic() - wall0 > args.start_timeout:
                result['error'] = 'fleet never started serving tasks'
                return result
            time.sleep(0.5)
        t_ready = mon.now()
        while mon.now() - t_ready < args.warmup:
            time.sleep(0.2)
        log.info('warm-up done')
        victim, task, trigger = None, '', None
        t_start = mon.now()
        if args.kind == 's3':
            while victim is None:
                t = mon.now() - t_start
                if t > args.trigger_timeout:
                    result['error'] = 'no blocking-junction moment found'
                    return result
                cand = mon.s3_candidate(primary=t < args.trigger_primary)
                if cand is not None:
                    victim = cand[0]
                    trigger = ('primary' if t < args.trigger_primary else 'fallback') + \
                        f' (waiter {cand[1]})'
                    task = mon.intent[victim].task_id
                    break
                time.sleep(0.1)
            t_event, pids = kill(mon, victim)
            mon.event('kill', victim, f'SIGKILL {pids}; Nav2 goal cancelled', task)
            log.warn(f'killed {victim} ({pids}) at t={t_event:.2f}s holding {task}; {trigger}')
            result['killed_pids'] = pids
        else:
            if args.kind == 's4':
                while victim is None:
                    if mon.now() - t_start > args.trigger_timeout:
                        result['error'] = 'no robot holding a task'
                        return result
                    with mon.lock:
                        busy = [r for r in robots if r in mon.intent and mon.intent[r].task_id
                                and mon.speed(r) > 0.02]
                    if busy:
                        victim = busy[0]
                        task = mon.intent[victim].task_id
                    time.sleep(0.1)
                length = args.partition
            else:
                victim, length = '*', args.blackout
            t_event = mon.now() + 0.5
            mon.partition(victim, t_event, t_event + length)
            mon.event('partition', victim, f'{length:.0f} s, all links', task)
            log.warn(f'partition {victim} {t_event:.1f}-{t_event + length:.1f}s ({task})')
        while mon.now() < t_event + args.post_window:
            time.sleep(0.2)
        time.sleep(0.5)
        result.update(summarize(mon, args, t_event, victim, task, trigger))
        publish_summary(mon, result, args.loss)
        time.sleep(1.0)                                  # let the logger write it
        return result
    finally:
        ex.shutdown(timeout_sec=2.0)


def main(argv=None):
    """Entry point."""
    ap = argparse.ArgumentParser(description='CLAUDE_CODE/06 recovery trial monitor')
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--kind', default='s3', choices=('s3', 's4', 'blackout'))
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--warmup', type=float, default=20.0, help='[s sim] after the first award')
    ap.add_argument('--trigger-primary', type=float, default=30.0, help='[s sim]')
    ap.add_argument('--trigger-timeout', type=float, default=180.0, help='[s sim]')
    ap.add_argument('--post-window', type=float, default=90.0, help='[s sim] observed after')
    ap.add_argument('--partition', type=float, default=30.0, help='[s sim] s4 isolation')
    ap.add_argument('--blackout', type=float, default=5.0, help='[s sim] fleet-wide blackout')
    ap.add_argument('--start-timeout', type=float, default=240.0, help='[s wall]')
    ap.add_argument('--contact-tol', type=float, default=0.01)
    ap.add_argument('--loss', type=float, default=0.0, help='loss level, for the record only')
    args, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[sys.argv[0]] + ros_args)
    try:
        result = run(args)
    finally:
        rclpy.try_shutdown()
    with open(os.path.join(args.run_dir, 'recovery_trial.json'), 'w') as f:
        json.dump(result, f, indent=1, default=str)
    print(json.dumps({k: result.get(k) for k in (
        'kind', 'victim', 'victim_task', 'trigger', 'n_contacts', 'restoration_s', 'entered_s',
        'lease_expire_s', 'lease_expire_all_s', 'space_reclaimed_s', 'task_reassigned_s',
        'detect_s', 'duplicate_completions', 'throughput_post_tasks_per_min', 'error')},
        indent=1, default=str))
    return 0 if 'error' not in result else 1


if __name__ == '__main__':
    raise SystemExit(main())
