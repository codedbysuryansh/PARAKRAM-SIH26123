r"""
CLAUDE_CODE/04 acceptance monitor (observer; its only intervention is the announcer kill).

    ros2 launch parakram_bringup fleet_sim.launch.py n_robots:=3 scenario:=warehouse_stream \\
        seed:=1 safety:=false
    ros2 launch parakram_coord coord.launch.py n_robots:=3 scenario:=warehouse_stream
    ros2 launch parakram_safety safety.launch.py n_robots:=3
    ros2 run parakram_tasks task_acceptance --n 3 --n-tasks 20        # before the tasks start
    ros2 launch parakram_tasks tasks.launch.py n_robots:=3 task_rate:=0.2

After ``--kill-after`` completed tasks it SIGKILLs the auction node of the robot that announces
the next round, right after its announcement (its bid window has not closed, so it cannot have
awarded). It then checks the PASS criteria of CLAUDE_CODE/04:

1. all ``--n-tasks`` streamed tasks are completed;
2. 0 inter-robot collisions, from INDEPENDENT ground truth (``/ground_truth/poses``, Gazebo,
   the footprint checker of 01-03), never from a robot's own pose;
3. each task is awarded exactly once, except intentional re-auctions: per task
   ``1 <= awards <= 1 + re-auctions``, no round awarded to two robots, one completion per task;
4. allocation is decentralized: only per-robot ``auction_node``s publish announcements and awards
   (graph), several robots announced and won, and no auctioneer node exists;
5. the task whose announcer was killed is re-announced by a peer and served by a live robot;
6. ``tasks.csv`` has the specified fields and events.

Result: ``<run_dir>/task_acceptance.json``. Acceptance checks are verification, not benchmark
results.
"""

import argparse
import csv
import json
import math
import os
import signal
import threading
import time

from parakram_comms.qos import STATUS_QOS, TASK_POOL_QOS
from parakram_msgs.msg import Award, Task, TaskAnnounce, TaskComplete, TaskStatus
from parakram_sim.footprint import box_polygon, footprint_polygon, polygon_distance
from parakram_sim.grid_utils import default_grid_path, WarehouseGrid
from parakram_sim.warehouse import static_boxes
from parakram_tasks.auction_node import LOG_COLUMNS
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

EVENTS = ('announce', 'bid', 'award', 'renew', 'complete', 'reauction')
REQUIRED = {'announce': ('task_id', 'robot_id'), 'bid': ('task_id', 'robot_id', 'cost'),
            'award': ('task_id', 'robot_id', 'cost', 'lease_expiry'),
            'renew': ('task_id', 'robot_id', 'lease_expiry'),
            'complete': ('task_id', 'robot_id', 'cost', 'lease_expiry'),
            'reauction': ('task_id', 'robot_id')}


def _sec(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def auction_pid(robot):
    """PID of ``robot``'s auction node (from /proc), or None."""
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = f.read().split(b'\0')
        except OSError:
            continue
        if any(a.endswith(b'parakram_tasks/auction_node') for a in argv) and \
                f'__ns:=/{robot}'.encode() in argv:
            return int(pid)
    return None


class Monitor(Node):
    """Ground truth + fleet task traffic + the announcer kill."""

    def __init__(self, robots, boxes, contact_tol, kill_after):
        """Subscribe to everything the acceptance needs."""
        super().__init__('task_acceptance')
        self.robots, self.contact_tol, self.kill_after = robots, contact_tol, kill_after
        self.static = [box_polygon(b.x_min, b.y_min, b.x_max, b.y_max) for b in boxes]
        self.lock = threading.Lock()
        self.sim_now, self.t0 = None, None
        self.gt, self.start_pose = {}, {}
        self.min_rr = (math.inf, None, None)
        self.min_static = (math.inf, None, None)
        self.contacts, self._in_contact = [], {}
        self.tasks, self.announces, self.awards, self.completes = {}, [], [], []
        self.status = {}
        self.kill = None
        self.create_subscription(TFMessage, '/ground_truth/poses', self._on_gt,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Task, '/fleet/tasks', self._on_task, TASK_POOL_QOS)
        self.create_subscription(TaskAnnounce, '/fleet/task_announce', self._on_announce,
                                 TASK_POOL_QOS)
        self.create_subscription(Award, '/fleet/award', self._on_award, TASK_POOL_QOS)
        self.create_subscription(TaskComplete, '/fleet/task_complete', self._on_complete,
                                 TASK_POOL_QOS)
        self.create_subscription(TaskStatus, '/fleet/task_status', self._on_status, STATUS_QOS)

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

    def _on_task(self, msg):
        with self.lock:
            if self.t0 is None:
                self.t0 = self.sim_now
            self.tasks.setdefault(msg.task_id, (self.sim_now, msg.source))

    def _on_announce(self, msg):
        with self.lock:
            self.announces.append((self.sim_now, msg.task_id, msg.seq, msg.announcer_id))
            if self.kill is None and len({c[1] for c in self.completes}) >= self.kill_after \
                    and msg.announcer_id in self.robots:
                pid = auction_pid(msg.announcer_id)
                if pid is not None:
                    os.kill(pid, signal.SIGKILL)   # mid-auction: its bid window is still open
                self.kill = {'robot': msg.announcer_id, 'task': msg.task_id, 'seq': msg.seq,
                             'pid': pid, 't': self.sim_now}
                self.get_logger().warn(f'SIGKILLed the announcer {msg.announcer_id} (pid {pid})'
                                       f' right after it announced {msg.task_id} round '
                                       f'{msg.seq}')

    def _on_award(self, msg):
        with self.lock:
            self.awards.append((self.sim_now, msg.task_id, msg.seq, msg.winner_id,
                                msg.announcer_id))

    def _on_complete(self, msg):
        with self.lock:
            self.completes.append((self.sim_now, msg.task_id, msg.robot_id, msg.seq))

    def _on_status(self, msg):
        with self.lock:
            self.status[msg.robot_id] = (msg.n_pending, msg.n_assigned, msg.n_completed)


def check_log(path):
    """Check acceptance item 6; also return the log rows (award counts for item 3)."""
    out = {'path': path, 'exists': os.path.isfile(path)}
    if not out['exists']:
        return out, []
    with open(path) as f:
        rows = list(csv.reader(f))
    out['header'] = rows[0] if rows else []
    out['header_ok'] = rows[:1] == [LOG_COLUMNS] and LOG_COLUMNS[:6] == [
        't', 'event', 'task_id', 'robot_id', 'cost', 'lease_expiry']
    body = [dict(zip(LOG_COLUMNS, r)) for r in rows[1:] if r]
    out['rows'] = len(body)
    out['events'] = {e: sum(1 for x in body if x['event'] == e) for e in EVENTS}
    bad = []
    for x in body:
        try:
            float(x['t'])
        except ValueError:
            bad.append(x)
            continue
        if x['event'] not in EVENTS or any(not x[k] for k in REQUIRED.get(x['event'], ())):
            bad.append(x)
    out['malformed_rows'] = len(bad)
    out['ok'] = bool(out['header_ok'] and not bad and all(out['events'][e] for e in EVENTS))
    return out, body


def run(args):
    """Monitor one W04 session; returns ``(passed, summary)``."""
    from parakram_bringup import run_manifest as rm

    robots = [f'robot{i}' for i in range(1, args.n + 1)]
    run_dir = args.run_dir or os.path.realpath(os.path.join(rm.default_log_root(), 'latest'))
    grid = WarehouseGrid.from_yaml(default_grid_path())
    rclpy.init()
    mon = Monitor(robots, static_boxes(grid), args.contact_tol, args.kill_after)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(mon)
    threading.Thread(target=ex.spin, daemon=True).start()
    log = mon.get_logger()
    summary = {'run_dir': run_dir, 'params': vars(args)}
    wall0 = time.monotonic()
    graph = {}
    try:
        log.info('waiting for the first streamed task...')
        while mon.t0 is None:
            if time.monotonic() - wall0 > args.start_timeout:
                summary['error'] = 'no task was ever streamed'
                return False, summary
            time.sleep(0.5)
        while True:
            with mon.lock:
                done = {c[1] for c in mon.completes}
                now, t0 = mon.sim_now, mon.t0
                tasks = set(mon.tasks)
            if not graph and len(tasks) >= 2:
                graph = fleet_graph(mon)          # while every node is still alive
            if len(tasks) >= args.n_tasks and tasks <= done:
                break
            if now is not None and now - t0 > args.timeout:
                summary['error'] = f'timeout after {args.timeout:.0f}s sim'
                break
            time.sleep(1.0)
        time.sleep(2.0)
    finally:
        ex.shutdown(timeout_sec=2.0)
    with mon.lock:
        t0, t_end = mon.t0, mon.sim_now
        tasks, announces = dict(mon.tasks), list(mon.announces)
        awards, completes, kill = list(mon.awards), list(mon.completes), mon.kill
        moved = {r: math.hypot(mon.gt[r][0] - mon.start_pose[r][0],
                               mon.gt[r][1] - mon.start_pose[r][1])
                 for r in robots if r in mon.gt and r in mon.start_pose}
        contacts = [[p, s - t0, e - t0, d] for p, s, e, d in mon.contacts]
        min_rr, min_static = mon.min_rr, mon.min_static
    rclpy.shutdown()

    log_info, body = check_log(os.path.join(run_dir, 'tasks.csv'))
    per_task = {}
    for t in sorted(tasks):
        aw = [a for a in awards if a[1] == t]
        rounds = {}
        for a in aw:
            rounds.setdefault(a[2], set()).add(a[3])
        per_task[t] = {
            'awards': len(aw),
            'reauctions': sum(1 for x in body if x['event'] == 'reauction' and x['task_id'] == t),
            'double_awarded_rounds': sorted(s for s, w in rounds.items() if len(w) > 1),
            'completions': [c[2] for c in completes if c[1] == t],
            'winners': sorted({a[3] for a in aw})}
    completed = {c[1] for c in completes}
    announcers = sorted({a[3] for a in announces})
    winners = sorted({a[3] for a in awards})
    summary.update({
        't0_sim': t0, 'end_sim': t_end, 'run_length_s': (t_end - t0) if t0 and t_end else None,
        'tasks_streamed': len(tasks), 'tasks_completed': len(completed & set(tasks)),
        'makespan_s': max((c[0] for c in completes), default=t0) - t0 if completes else None,
        'robot_robot_contacts': contacts, 'min_robot_robot_distance_m': min_rr[0],
        'min_rr_pair': min_rr[1], 'min_robot_static_clearance_m': min_static[0],
        'distance_travelled_m': moved, 'announcers': announcers, 'winners': winners,
        'announcements': len(announces), 'awards': len(awards), 'kill': kill,
        'per_task': per_task, 'task_log': log_info, 'fleet_graph': graph})

    checks = {}
    checks['all_tasks_completed'] = len(tasks) == args.n_tasks and set(tasks) <= completed
    checks['no_robot_robot_collision'] = not contacts
    checks['each_task_awarded_once_modulo_reauctions'] = all(
        1 <= p['awards'] <= 1 + p['reauctions'] and not p['double_awarded_rounds'] and
        len(p['completions']) == 1 for p in per_task.values())
    auction_pubs = graph.get('publishers', {})
    only_robots = all(n.startswith('/robot') and n.endswith('/auction_node')
                      for topic in ('/fleet/task_announce', '/fleet/award')
                      for n in auction_pubs.get(topic, []))
    checks['decentralized_no_auctioneer'] = bool(
        auction_pubs.get('/fleet/award') and only_robots and len(announcers) >= 2 and
        len(winners) >= 2 and not any('auctioneer' in n for n in graph.get('nodes', [])) and
        all(src != '' for _, src in tasks.values()))
    served = False
    if kill is not None:
        later = [a for a in announces if a[1] == kill['task'] and a[2] > kill['seq']]
        dead_award = any(a[1] == kill['task'] and a[2] == kill['seq'] for a in awards)
        done_by = per_task.get(kill['task'], {}).get('completions', [])
        served = bool(kill['pid'] and later and later[0][3] != kill['robot'] and
                      not dead_award and done_by and done_by[0] != kill['robot'])
        summary['kill'].update({'reannounced_by': later[0][3] if later else None,
                                'reannounce_delay_s': (later[0][0] - kill['t']) if later
                                else None, 'award_from_dead_announcer': dead_award,
                                'served_by': done_by[0] if done_by else None,
                                'dead_process_gone': auction_pid(kill['robot']) is None})
    checks['killed_announcer_task_served_by_peer'] = served
    checks['task_log_fields_and_events'] = bool(log_info.get('ok'))
    checks['robots_moved'] = sum(1 for d in moved.values() if d > 0.3) >= 2
    summary['checks'] = checks
    summary['pass'] = all(checks.values())
    with open(os.path.join(run_dir, 'task_acceptance.json'), 'w') as f:
        json.dump(summary, f, indent=1, default=str)
    return summary['pass'], summary


def fleet_graph(node):
    """Return the publishers of the auction topics and the node list (ROS graph)."""
    pubs = {}
    for topic in ('/fleet/tasks', '/fleet/task_announce', '/fleet/award', '/fleet/bid',
                  '/fleet/task_complete'):
        pubs[topic] = sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                             for i in node.get_publishers_info_by_topic(topic))
    nodes = sorted(f'{ns.rstrip("/")}/{name}'
                   for name, ns in node.get_node_names_and_namespaces())
    return {'publishers': pubs, 'nodes': nodes}


def main(argv=None):
    """Entry point."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--n-tasks', type=int, default=20)
    ap.add_argument('--kill-after', type=int, default=3,
                    help='kill the next announcer after this many completed tasks')
    ap.add_argument('--run-dir', default=None)
    ap.add_argument('--timeout', type=float, default=1500.0, help='[s sim] after the 1st task')
    ap.add_argument('--start-timeout', type=float, default=600.0, help='[s wall]')
    ap.add_argument('--contact-tol', type=float, default=0.01)
    args = ap.parse_args(argv)
    passed, summary = run(args)
    keys = ('checks', 'tasks_streamed', 'tasks_completed', 'makespan_s', 'robot_robot_contacts',
            'min_robot_robot_distance_m', 'announcers', 'winners', 'kill', 'error')
    print(json.dumps({k: summary.get(k) for k in keys}, indent=1, default=str))
    print(f"RESULT: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1
