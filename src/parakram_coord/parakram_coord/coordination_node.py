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
from parakram_comms.qos import INTENT_QOS, STATE_QOS, STATUS_QOS
from parakram_coord.pibt_rule import (committed_claims, CoordParams, occupied_cells,
                                      PibtCoordinator)
from parakram_coord.reservation_table import Entry, ReservationTable
from parakram_coord.roster import Roster
from parakram_coord.spacetime_astar import shortest_path
from parakram_msgs.msg import CoordStatus, Intent, RobotState
from parakram_sim.grid_utils import default_grid_path, WarehouseGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros


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

        self.grid = WarehouseGrid.from_yaml(grid_yaml or default_grid_path())
        self.core = PibtCoordinator(self.robot_id, self.grid, self.params)
        self.table = ReservationTable()

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        # /tf, /tf_static are remapped to the robot namespace by the launch file.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_state = self.create_publisher(RobotState, 'state', STATE_QOS)
        self.pub_intent = self.create_publisher(Intent, 'intent', INTENT_QOS)
        self.pub_status = self.create_publisher(CoordStatus, 'coord_status', STATUS_QOS)
        self.pub_reroute = self.create_publisher(String, 'reroute_request', 10)
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
        self.timer = self.create_timer(1.0 / tick_hz, self._tick)
        self.get_logger().info(
            f'coordination {self.robot_id}: {tick_hz:.0f} Hz, W={self.params.window}, '
            f'k={self.params.reserve_k}, lease={self.params.lease_ttl}s, '
            f'fixed goals={self.fixed_goals} for {self.fixed_duration:.0f}s'
            + (' -- REACTIVE-ONLY: conflict resolution disabled' if self.reactive_only else ''))

    # ------------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _add_peer(self, peer_id):
        if peer_id in self.peer_subs:
            return
        self.intents_received[peer_id] = 0
        self.peer_subs[peer_id] = (
            self.create_subscription(Intent, f'/{peer_id}/intent',
                                     lambda m, pid=peer_id: self._on_intent(pid, m), INTENT_QOS),
            self.create_subscription(RobotState, f'/{peer_id}/state',
                                     lambda m: None, STATE_QOS))
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
        self.table.update(Entry(owner=peer_id, seq=msg.seq, priority=msg.priority, cells=cells,
                                occupied=occ, planned=tuple(planned),
                                lease_expiry=_sec(msg.lease_expiry), heard_at=now, pose=pose))
        self.intents_received[peer_id] += 1

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
        res = self.core.tick(now, center, occ, self.table, committed)
        authority = res.authority
        if self.reactive_only:
            goal = self.core.goal
            authority = [center] if goal is None or goal == center else shortest_path(
                center, goal, self.core._nbrs, self.core._h(goal))
        if res.goal_reached_event:
            self._assign_next(now)
        self._update_nav(authority, now, pose, occ)
        tick_ms = (time.perf_counter() - t0) * 1000.0
        self._publish(now, pose, res, tick_ms)

    def _publish(self, now, pose, res, tick_ms):
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
        self.pub_intent.publish(intent)

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
        self.pub_status.publish(st)

        if res.reroute_request:
            self.pub_reroute.publish(String(data=f'{self.robot_id}: {res.note}'))
            self.get_logger().warn(f'{self.robot_id}: {res.note}')

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
        """Flush the CSV log."""
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
