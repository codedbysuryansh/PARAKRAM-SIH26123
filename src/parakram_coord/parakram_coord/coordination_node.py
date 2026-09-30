"""
Decentralized coordination node (CLAUDE_CODE/02): ROS 2 wrapper around ``PibtCoordinator``.

One instance per robot, in the robot's namespace, identical on every robot; no central node.

In : ``/<peer>/intent``, ``/<peer>/state`` (peers from the roster / graph scan),
     ``/<ns>/assigned_task`` (geometry_msgs/Pose2D goal in the map frame, from the task layer),
     ``/<ns>/amcl_pose`` (localization gate) and the robot's own TF (map -> base_footprint).
Out: ``/<ns>/state`` + ``/<ns>/intent`` (BEST_EFFORT, every tick), ``/<ns>/coord_status``,
     ``/<ns>/reroute_request``, and Nav2 ``NavigateThroughPoses`` goals over the HELD cells only.
     Never publishes cmd_vel.

``reactive_only:=true`` disables conflict resolution (the CLAUDE_CODE/03 acceptance setup,
"coordination running, conflict resolution disabled"): the node still ticks and publishes
state / intent / coord_status, but Nav2 gets the whole shortest path to the goal, ignoring every
peer, so only the reactive safety layer stands between robots. Default ``false`` (02 behaviour).

``loss`` (CLAUDE_CODE/05, default 0): seeded app-level loss (``parakram_comms.loss``) on the
peer ``state`` / ``intent`` subscriptions, applied before a message is processed; counters in
``bench/logs/<run_id>/comms_<ns>.csv``. ``loss_scope:=fleet`` (CLAUDE_CODE/06) uses the per-link
process of ``parakram_comms.link_loss`` instead (the one every inter-robot topic of every node
of this robot goes through, with the bench's partitions); counters in ``comms_<ns>_coord.csv``.

CLAUDE_CODE/06 lease protocol (``recovery_mode:=lease``, the default): every intent is the lease
renewal and carries acknowledgements of the peers' renewals, the held authority (body envelope)
and the task award it renews (``/<ns>/current_task``, from the task layer). A claim becomes
authority only once every live peer acknowledged it; when the own lease is no longer
acknowledged by every live peer, or less than a majority is heard, the robot stops (Nav2
cancelled), drops its claims and re-acquires before moving (``parakram_coord.leases``). A silent
peer's lease expires with no message: its reservations are reclaimed, its body envelope stays a
ghost obstacle until this robot's lidar sees the cells empty (``ghost_view``) or the peer is
heard again. ``recovery_mode:=release`` is the ``reauction_baseline``: no leases; a silent
peer's reservations stay until ``/<ns>/release_peer`` names it (sent by the task layer when the
peer's task was re-auctioned). Recovery events: ``/fleet/recovery_event`` and
``bench/logs/<run_id>/recovery_<ns>.csv``.

Until CLAUDE_CODE/04 provides task allocation, ``fixed_goals`` (flattened ``[r0, c0, r1, ...]``)
gives the robot a local, temporary cyclic assignment for ``fixed_assign_duration`` seconds.
Per tick it logs ``bench/logs/<run_id>/coord_<ns>.csv``.

Priority: ``Intent.priority`` carries the time-invariant rank that peers compare (see
pibt_rule); ``coord_status.priority`` and the CSV show the same priority as an age (rises while
away from the goal, 0 at the goal).
"""

import csv
import math
import os
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from parakram_comms.link_loss import attach_fault_injection, LinkLoss
from parakram_comms.loss import LossFilter
from parakram_comms.qos import INTENT_QOS, STATE_QOS, STATUS_QOS
from parakram_coord.ghost_view import classify_cell, FREE, OCCUPIED
from parakram_coord.leases import OwnLease
from parakram_coord.pibt_rule import (committed_claims, CoordParams, occupied_cells,
                                      PibtCoordinator)
from parakram_coord.reservation_table import Entry, ReservationTable
from parakram_coord.roster import Roster
from parakram_coord.spacetime_astar import shortest_path
from parakram_msgs.msg import CoordStatus, Intent, RecoveryEvent, RobotState, TaskProgress
from parakram_sim.grid_utils import default_grid_path, WarehouseGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

RECOVERY_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
GHOST_FREE_SCANS = 3              # consecutive scans that must see a ghost cell empty
RECOVERY_COLUMNS = ['t', 'robot_id', 'kind', 'peer', 'cells', 'detail']


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _sec(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def _stamp(msg_time, t):
    msg_time.sec = int(math.floor(t))
    msg_time.nanosec = int((t - math.floor(t)) * 1e9)
    return msg_time


def _cells_str(cells):
    return '|'.join(f'{r}:{c}' for r, c in cells)


class CoordinationNode(Node):
    """Per-robot decentralized coordination."""

    def __init__(self):
        """Declare parameters, create I/O and start the tick timer."""
        super().__init__('coordination')
        ns = self.get_namespace().strip('/')
        dp = self.declare_parameter
        self.robot_id = dp('robot_id', ns or 'robot1').value
        tick_hz = float(dp('tick_hz', 10.0).value)
        self.params = CoordParams(
            window=int(dp('window_W', 8).value), reserve_k=int(dp('reserve_k', 3).value),
            lease_ttl=float(dp('lease_ttl', 2.0).value),
            neighbor_timeout=float(dp('neighbor_timeout', 1.0).value),
            claim_settle=float(dp('claim_settle', 0.4).value),
            priority_grow_rate=float(dp('priority_grow_rate', 1.0).value),
            blocked_after=float(dp('blocked_after', 3.0).value),
            deadlock_timeout=float(dp('deadlock_timeout', 30.0).value),
            deadlock_bump=int(dp('deadlock_bump', 50).value),
            detour_ttl=float(dp('detour_ttl', 20.0).value),
            stuck_timeout=float(dp('stuck_timeout', 20.0).value),
            push_horizon=int(dp('push_horizon', 8).value),
            occupancy_radius=float(dp('occupancy_radius', 0.14).value))
        self.commit_margin = float(dp('commit_margin', 0.08).value)
        self.resend_period = float(dp('nav_resend_period', 0.25).value)
        # typed declaration: an empty-list default would be inferred as BYTE_ARRAY
        dp('fixed_goals', Parameter.Type.INTEGER_ARRAY)
        try:
            fixed = list(self.get_parameter('fixed_goals').value or [])
        except ParameterUninitializedException:
            fixed = []
        self.fixed_goals = [(int(fixed[i]), int(fixed[i + 1]))
                            for i in range(0, len(fixed) - 1, 2)]
        self.fixed_duration = float(dp('fixed_assign_duration', 300.0).value)
        log_dir = dp('log_dir', '').value
        grid_yaml = dp('grid_yaml', '').value
        roster_scan = float(dp('roster_scan_period', 5.0).value)
        self.reactive_only = bool(dp('reactive_only', False).value)
        # CLAUDE_CODE/06 lease protocol ('lease', default) or the reauction_baseline ('release')
        self.recovery_mode = dp('recovery_mode', 'lease').value
        if self.recovery_mode not in ('lease', 'release'):
            raise ValueError(f'recovery_mode must be lease or release, got {self.recovery_mode}')
        self.lease_gate = (bool(dp('lease_gate', True).value) and not self.reactive_only and
                           self.recovery_mode == 'lease')
        stop_margin = float(dp('lease_stop_margin', 0.3).value)
        self.ghost_sensing = bool(dp('ghost_sensing', True).value)
        loss_scope = dp('loss_scope', 'coordination').value
        # CLAUDE_CODE/05: seeded app-level loss on PEER state / intent, applied before processing
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        tag = ns or self.robot_id
        loss_args = (float(dp('loss', 0.0).value), int(dp('seed', 0).value), self.robot_id)
        loss_kw = {'model': dp('loss_model', 'bernoulli').value,
                   'burst_corr': float(dp('loss_burst_corr', 0.8).value)}
        if loss_scope == 'fleet':             # 06: the robot's shared per-link process
            self.loss = LinkLoss(*loss_args, **loss_kw, log_path=os.path.join(
                log_dir, f'comms_{tag}_coord.csv') if log_dir else None)
            attach_fault_injection(self, self.loss)
            self._wrap = lambda peer, topic, cb: self.loss.wrap(peer, topic, cb, self._now)
        elif loss_scope == 'coordination':
            self.loss = LossFilter(*loss_args, **loss_kw, log_path=os.path.join(
                log_dir, f'comms_{tag}.csv') if log_dir else None)
            self._wrap = self.loss.wrap
        else:
            raise ValueError(f'loss_scope must be coordination or fleet, got {loss_scope}')
        self.loss_logged_at = None
        self.own = OwnLease(self.params.lease_ttl, stop_margin)
        self.peer_seq = {}                    # peer -> latest renewal seq processed (my acks)
        self.known_peers = set()              # peers ever heard (quorum base)
        self.claim_seq = {}                   # claimed cell -> seq of the first intent with it
        self.lease_ok, self.lease_reason = True, ''
        self.current_task = ('', 0)
        self.current_announcer = ''
        self.reclaimable = {}                 # silent peer -> cells of its that are free now
        self.reclaim_reported = set()
        self.ghost_free = {}                  # ghost cell -> consecutive scans seeing it empty
        self.live = set()

        self.grid = WarehouseGrid.from_yaml(grid_yaml or default_grid_path())
        self.core = PibtCoordinator(self.robot_id, self.grid, self.params)
        self.table = ReservationTable(expire_by_lease=self.recovery_mode == 'lease')

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        # /tf, /tf_static are remapped to the robot namespace by the launch file.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_state = self.create_publisher(RobotState, 'state', STATE_QOS)
        self.pub_intent = self.create_publisher(Intent, 'intent', INTENT_QOS)
        self.pub_status = self.create_publisher(CoordStatus, 'coord_status', STATUS_QOS)
        self.pub_reroute = self.create_publisher(String, 'reroute_request', 10)
        self.pub_recovery = self.create_publisher(RecoveryEvent, '/fleet/recovery_event',
                                                  RECOVERY_QOS)
        self.create_subscription(TaskProgress, 'current_task', self._on_current_task,
                                 STATUS_QOS)
        if self.recovery_mode == 'release':
            self.create_subscription(String, 'release_peer', self._on_release, 10)
        if self.ghost_sensing:
            self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Pose2D, 'assigned_task', self._on_task, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.nav = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.peer_subs = {}
        self.intents_received = {}
        self.roster = Roster(self, self.robot_id, self._add_peer, scan_period=roster_scan)

        self.seq = 0
        self.started = False
        self.t_start = None
        self.fixed_idx = 0
        self.fixed_active = bool(self.fixed_goals)
        self.localized = False
        self.last_pose = None
        self.last_pose_t = None
        self.velocity = (0.0, 0.0)
        self.goal_handle = None
        self.goal_active = False
        self.sent_cells = ()
        self.last_send_t = -1e9
        self.nav_failures = 0
        self.nav_retry_after = 0.0

        self.csv_file = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            self.csv_file = open(os.path.join(log_dir, f'coord_{ns or self.robot_id}.csv'), 'w',
                                 newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['t', 'robot_id', 'cell', 'goal_cell', 'priority', 'reserved_cells',
                               'blocked', 'yields', 'replans', 'tick_compute_ms'])
            self.rows_since_flush = 0
        self.recovery_file = None
        if log_dir:
            path = os.path.join(log_dir, f'recovery_{ns or self.robot_id}.csv')
            self.recovery_file = open(path, 'w', newline='')
            self.recovery_csv = csv.writer(self.recovery_file)
            self.recovery_csv.writerow(RECOVERY_COLUMNS)
        self.timer = self.create_timer(1.0 / tick_hz, self._tick)
        self.get_logger().info(
            f'coordination {self.robot_id}: {tick_hz:.0f} Hz, W={self.params.window}, '
            f'k={self.params.reserve_k}, lease={self.params.lease_ttl}s, '
            f'fixed goals={self.fixed_goals} for {self.fixed_duration:.0f}s'
            + (' -- REACTIVE-ONLY: conflict resolution disabled' if self.reactive_only else '')
            + (f' -- app-level loss {self.loss.loss:.0%} ({self.loss.model}, seed '
               f'{self.loss.seed}, scope {loss_scope})' if self.loss.loss > 0 else '')
            + f' -- recovery {self.recovery_mode}, lease gate {self.lease_gate}')

    # ------------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _add_peer(self, peer_id):
        if peer_id in self.peer_subs:
            return
        self.intents_received[peer_id] = 0
        on_intent = self._wrap(peer_id, 'intent',
                               lambda m, pid=peer_id: self._on_intent(pid, m))
        self.peer_subs[peer_id] = (
            self.create_subscription(Intent, f'/{peer_id}/intent', on_intent, INTENT_QOS),
            self.create_subscription(RobotState, f'/{peer_id}/state',
                                     self._wrap(peer_id, 'state', lambda m: None), STATE_QOS))
        via = sorted(self.roster.sources.get(peer_id, []))
        self.get_logger().info(f'peer discovered: {peer_id} (via {via})')

    def _on_intent(self, peer_id, msg):
        if msg.robot_id != peer_id:
            return
        now = self._now()
        grid = self.grid
        n = grid.rows * grid.cols
        cells = tuple(grid.index_to_cell(i) for i in msg.reserved_cells if 0 <= i < n)
        pose = None
        planned = []
        if msg.planned_path:
            p0 = msg.planned_path[0]
            pose = (p0.x, p0.y, p0.theta)
            planned.append(grid.world_to_cell(p0.x, p0.y))
            for p in msg.planned_path[1:]:
                c = grid.world_to_cell(p.x, p.y)
                if c != planned[-1]:
                    planned.append(c)
        occ = occupied_cells(grid, pose[0], pose[1], self.params.occupancy_radius) \
            if pose else frozenset(cells[:1])
        authority = tuple(grid.index_to_cell(i) for i in msg.authority_cells if 0 <= i < n)
        stored = self.table.update(Entry(owner=peer_id, seq=msg.seq, priority=msg.priority,
                                         cells=cells, occupied=occ, planned=tuple(planned),
                                         lease_expiry=_sec(msg.lease_expiry), heard_at=now,
                                         pose=pose, authority=authority))
        if stored:
            self.peer_seq[peer_id] = msg.seq
            self.known_peers.add(peer_id)
            for who, seq in zip(msg.ack_ids, msg.ack_seqs):
                if who == self.robot_id:
                    self.own.on_ack(peer_id, seq)
        self.intents_received[peer_id] += 1

    def _on_current_task(self, msg):
        self.current_task = (msg.task_id, int(msg.seq)) if msg.task_id else ('', 0)
        self.current_announcer = msg.announcer_id if msg.task_id else ''

    def _on_release(self, msg):
        now = self._now()
        if self.table.release(msg.data, now):
            self.get_logger().warn(f'released {msg.data} (re-auction award received)')

    def _on_scan(self, msg):
        """Clear ghost cells this robot's lidar sees empty (CLAUDE_CODE/06)."""
        ghosts = self.table.ghost_cells()
        if not ghosts:
            self.ghost_free.clear()
            return
        try:
            t = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        tr = t.transform.translation
        sensor = (tr.x, tr.y, _yaw(t.transform.rotation))
        half = self.grid.resolution / 2.0 - 0.04
        now = self._now()
        for cell in ghosts:
            cx, cy = self.grid.cell_to_world(*cell)
            verdict = classify_cell((cx - half, cy - half, cx + half, cy + half), sensor,
                                    msg.ranges, msg.angle_min, msg.angle_increment,
                                    msg.range_min, msg.range_max)
            if verdict == FREE:
                self.ghost_free[cell] = self.ghost_free.get(cell, 0) + 1
                if self.ghost_free[cell] >= GHOST_FREE_SCANS:
                    self.table.clear_ghost_cell(cell, now)
                    self.ghost_free.pop(cell, None)
            elif verdict == OCCUPIED:
                self.ghost_free[cell] = 0

    def _recovery(self, now, kind, peer, cells=(), detail='', task_id=''):
        ev = RecoveryEvent()
        _stamp(ev.stamp, now)
        ev.kind, ev.robot_id, ev.dead_id, ev.task_id = kind, self.robot_id, peer, task_id
        ev.loss_level, ev.t_event = float(self.loss.loss), float(now)
        ev.t_kill = ev.t_lease_expire = ev.t_space_reclaimed = ev.t_task_reassigned = math.nan
        ev.detail = detail
        self.pub_recovery.publish(ev)
        if self.recovery_file is not None:
            self.recovery_csv.writerow([f'{now:.3f}', self.robot_id, kind, peer,
                                        _cells_str(sorted(cells)), detail])
            self.recovery_file.flush()

    def _table_events(self, now, authority):
        for kind, owner, t, cells in self.table.drain_events():
            if kind in ('lease_expired', 'release'):
                self.reclaimable[owner] = set(cells)
                self.reclaim_reported.discard(owner)
            elif kind == 'ghost_cleared':
                self.reclaimable.setdefault(owner, set()).update(cells)
            elif kind == 'peer_back':
                self.reclaimable.pop(owner, None)
            self._recovery(t, kind, owner, cells)
        held = set(authority[1:])
        for owner, cells in self.reclaimable.items():
            got = held & cells
            if got and owner not in self.reclaim_reported:
                self.reclaim_reported.add(owner)
                self._recovery(now, 'space_reclaimed', owner, got)

    def _on_task(self, msg):
        self.fixed_active = False            # the task layer takes over from the fixed list
        self.core.set_goal(self.grid.world_to_cell(msg.x, msg.y), self._now())

    def _on_amcl(self, _msg):
        self.localized = True

    def _pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = t.transform.translation
        return tr.x, tr.y, _yaw(t.transform.rotation)

    # ------------------------------------------------------------------ assignment
    def _assign_next(self, now):
        if not self.fixed_active or not self.fixed_goals:
            return
        if self.core.goals_assigned > 0 and now - self.t_start >= self.fixed_duration:
            self.fixed_active = False
            self.get_logger().info(f'fixed assignment window over after '
                                   f'{self.core.goals_completed} goals')
            return
        goal = self.fixed_goals[self.fixed_idx % len(self.fixed_goals)]
        self.fixed_idx += 1
        self.core.set_goal(goal, now)

    # ------------------------------------------------------------------ Nav2 authority
    def _send_goal(self, cells, now, from_cell, hold_yaw=0.0):
        goal = NavigateThroughPoses.Goal()
        prev = from_cell
        for c in cells:
            x, y = self.grid.cell_to_world(*c)
            px, py = self.grid.cell_to_world(*prev)
            yaw = math.atan2(y - py, x - px) if prev != c else hold_yaw
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            goal.poses.append(ps)
            prev = c
        self.sent_cells = tuple(cells)
        self.last_send_t = now
        self.goal_active = True
        future = self.nav.send_goal_async(goal)
        sent = self.sent_cells
        future.add_done_callback(lambda f: self._on_goal_response(f, sent))

    def _on_goal_response(self, future, sent):
        handle = future.result()
        if not handle.accepted:
            if sent == self.sent_cells:
                self.goal_active = False
                self.sent_cells = ()
            return
        if sent != self.sent_cells:
            handle.cancel_goal_async()             # superseded/revoked before it was accepted
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, sent))

    def _on_result(self, future, sent):
        if sent != self.sent_cells:
            return                             # a newer goal superseded this one
        status = future.result().status
        self.goal_active = False
        if status == GoalStatus.STATUS_ABORTED:
            self.nav_failures += 1
            self.sent_cells = ()
            self.nav_retry_after = self._now() + 1.0
            self.get_logger().warn(f'Nav2 aborted the goal to {sent} ({self.nav_failures})')

    def _update_nav(self, authority, now, pose, occ):
        center = authority[0]
        ahead = tuple(authority[1:])
        if not ahead:
            if self.goal_active and self.sent_cells and self.sent_cells[-1] == center:
                return                                 # already settling onto the centre
            if len(occ) > 1 and now >= self.nav_retry_after:
                # authority shrank to my own cell but my safety disk still spills into a
                # neighbour (which nobody else could then claim): settle onto the centre
                # (keeping my heading); within the Nav2 goal tolerance the disk fits the cell
                self._send_goal((center,), now, center, hold_yaw=pose[2])
            elif self.goal_active and self.sent_cells:
                self._cancel_nav()                     # authority revoked: stop
            return
        if now < self.nav_retry_after:
            return
        if self.goal_active and self.sent_cells:
            if self.sent_cells[-1] == ahead[-1]:
                return                             # already driving to the end of the authority
            grown = self.sent_cells[-1] in ahead   # else the goal runs past the authority
            if grown and now - self.last_send_t < self.resend_period:
                return                             # extend at a bounded rate; shrink at once
        self._send_goal(ahead, now, center)

    def _cancel_nav(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_active = False
        self.sent_cells = ()

    # ------------------------------------------------------------------ tick
    def _tick(self):
        t0 = time.perf_counter()
        now = self._now()
        pose = self._pose()
        if pose is None:
            return
        if not self.started:
            if not self.nav.server_is_ready():
                return
            self.started = True
            self.t_start = now
            self.roster.scan()
            self._assign_next(now)
        x, y, yaw = pose
        grid = self.grid
        center = grid.world_to_cell(x, y)
        occ = occupied_cells(grid, x, y, self.params.occupancy_radius)
        committed = committed_claims(
            grid, [c for c, _ in self.core.claims], x, y, self.velocity[0], self.velocity[1],
            self.sent_cells if self.goal_active else (), self.params.occupancy_radius,
            self.commit_margin)
        # CLAUDE_CODE/06: expire silent peers first (ghosts), then check my own lease
        self.table.prune(now)
        live = set(self.table.valid(now, exclude=self.robot_id))
        self.live = live
        ok, why = self.own.status(now, live, self.known_peers) if self.lease_gate else (True, '')
        if ok != self.lease_ok:
            self._recovery(now, 'lease_regained' if ok else 'lease_lost', self.robot_id,
                           detail=why or self.lease_reason)
            self.get_logger().warn(f'own lease {"re-acquired" if ok else "LOST: " + why}')
        self.lease_ok, self.lease_reason = ok, why

        def acked(cell):
            return self.own.acked_by_all(self.claim_seq.get(cell, 1 << 62), live)
        res = self.core.tick(now, center, occ, self.table, committed,
                             obstacles=self.table.ghost_cells(), frozen=not ok,
                             claim_acked=acked if self.lease_gate else None)
        authority = res.authority
        if self.reactive_only:
            goal = self.core.goal
            authority = [center] if goal is None or goal == center else shortest_path(
                center, goal, self.core._nbrs, self.core._h(goal))
        if res.goal_reached_event:
            self._assign_next(now)
        if ok:
            self._update_nav(authority, now, pose, occ)
        else:                                  # lease not acknowledged: stop where I am
            authority = [center]
            if self.goal_active or self.sent_cells:
                self._cancel_nav()
        self._table_events(now, authority)
        tick_ms = (time.perf_counter() - t0) * 1000.0
        self._publish(now, pose, res, tick_ms, authority, occ)

    def _publish(self, now, pose, res, tick_ms, authority=(), occ=()):
        grid = self.grid
        x, y, yaw = pose
        self.seq += 1
        intent = Intent()
        intent.robot_id = self.robot_id
        intent.seq = self.seq
        intent.reserved_cells = [grid.cell_to_index(*c) for c in res.reserved
                                 if grid.in_bounds(*c)]
        intent.planned_path = [Pose2D(x=x, y=y, theta=yaw)]
        for c in res.planned[1:]:
            cx, cy = grid.cell_to_world(*c)
            intent.planned_path.append(Pose2D(x=cx, y=cy, theta=0.0))
        intent.priority = int(res.priority)
        _stamp(intent.lease_expiry, now + self.params.lease_ttl)
        # CLAUDE_CODE/06: this intent is the lease renewal
        _stamp(intent.stamp, now)
        acks = sorted(self.peer_seq.items())
        intent.ack_ids = [p for p, _ in acks]
        intent.ack_seqs = [int(q) for _, q in acks]
        body = set(occ) | set(authority) | {res.planned[0]}
        intent.authority_cells = [grid.cell_to_index(*c) for c in sorted(body)
                                  if grid.in_bounds(*c)]
        intent.task_id, intent.task_seq = self.current_task
        intent.task_announcer = self.current_announcer
        intent.lease_valid = bool(self.lease_ok)
        self.pub_intent.publish(intent)
        self.own.sent_renewal(self.seq, now, self.current_task if self.current_task[0] else None)
        claims = {c for c, _ in self.core.claims}
        for c in claims:
            self.claim_seq.setdefault(c, self.seq)
        for c in [c for c in self.claim_seq if c not in claims]:
            del self.claim_seq[c]

        state = RobotState()
        state.robot_id = self.robot_id
        state.seq = self.seq
        _stamp(state.stamp, now)
        state.pose = Pose2D(x=x, y=y, theta=yaw)
        if self.last_pose is not None and now > self.last_pose_t:
            dt = now - self.last_pose_t
            self.velocity = ((x - self.last_pose[0]) / dt, (y - self.last_pose[1]) / dt)
            state.velocity.linear.x = math.hypot(*self.velocity)
            dyaw = math.atan2(math.sin(yaw - self.last_pose[2]), math.cos(yaw - self.last_pose[2]))
            state.velocity.angular.z = dyaw / dt
        self.last_pose, self.last_pose_t = pose, now
        state.battery = 1.0
        state.status = int(res.status)
        self.pub_state.publish(state)

        goal = self.core.goal
        st = CoordStatus()
        st.robot_id = self.robot_id
        _stamp(st.stamp, now)
        st.cell = grid.cell_to_index(*res.planned[0]) if grid.in_bounds(*res.planned[0]) else -1
        st.goal_cell = grid.cell_to_index(*goal) if goal is not None else -1
        age = self.core.age_priority(res.priority, now)
        st.priority = age
        st.reserved_cells = list(intent.reserved_cells)
        st.blocked = bool(res.blocked)
        st.status = int(res.status)
        st.yields = self.core.yields
        st.replans = self.core.replans
        st.goals_assigned = self.core.goals_assigned
        st.goals_completed = self.core.goals_completed
        st.n_peers = min(255, res.n_peers)
        st.tick_compute_ms = float(tick_ms)
        st.note = res.note
        st.lease_valid = bool(self.lease_ok)
        acked = 0.0
        if self.current_task[0]:
            t_ack = self.own.task_quorum_time(self.known_peers, self.current_task)
            acked = 0.0 if t_ack is None else (now if math.isinf(t_ack) else t_ack)
        _stamp(st.task_renewal_acked, acked)
        st.task_id, st.task_seq = self.current_task
        self.pub_status.publish(st)

        if res.reroute_request:
            self.pub_reroute.publish(String(data=f'{self.robot_id}: {res.note}'))
            self.get_logger().warn(f'{self.robot_id}: {res.note}')

        if self.loss_logged_at is None or now - self.loss_logged_at >= 5.0:
            self.loss.log(now)
            self.loss_logged_at = now
        if self.csv_file is not None:
            self.csv.writerow([f'{now:.3f}', self.robot_id, _cells_str(res.planned[:1]),
                               _cells_str([goal]) if goal is not None else '', age,
                               _cells_str(res.reserved), int(res.blocked), self.core.yields,
                               self.core.replans, f'{tick_ms:.3f}'])
            self.rows_since_flush += 1
            if self.rows_since_flush >= 20:
                self.csv_file.flush()
                self.rows_since_flush = 0

    def close(self):
        """Flush the CSV logs (and the loss counters)."""
        self.loss.log(self._now())
        if self.recovery_file is not None:
            self.recovery_file.close()
            self.recovery_file = None
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None


def main(args=None):
    """Run one coordination node."""
    rclpy.init(args=args)
    node = CoordinationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
