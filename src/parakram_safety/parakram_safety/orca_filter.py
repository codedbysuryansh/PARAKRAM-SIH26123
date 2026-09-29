"""
Reactive safety filter node, Part B of CLAUDE_CODE/03 (one per robot, in its namespace).

    controller/behaviors --cmd_vel_nav--> [orca_filter] --cmd_vel_safe--> collision_monitor
    (Part A) --cmd_vel_monitored--> velocity_smoother --cmd_vel--> base

In : ``cmd_vel_nav`` (nominal, Nav2), ``scan`` (the robot's own lidar), the robot's own TF
     (odom -> base_footprint, to compensate its motion since the 5 Hz scan), ``cmd_vel`` (the
     command its base receives: the robot's current velocity, NH-ORCA's v_opt, and the logged
     output) and, for the intervention flag, ``collision_monitor_state`` of the same robot.
Out: ``cmd_vel_safe`` (filtered), ``safety_status`` and ``bench/logs/<run_id>/safety_<ns>.csv``.

Comms-free by construction: the node subscribes to NO peer topic (no ``/<peer>/state`` or
``intent``, no roster), so peer messages disappearing cannot change what it does; a silent,
partitioned or dead robot is avoided as a sensed obstacle. Neighbour velocity is not used.

Fail-safe: no fresh scan within ``scan_timeout`` -> the command is ZERO (never the nominal).
Ablation hook: ``safety_enabled:=false`` (benchmark A1) makes this node pass the nominal
through AND switches the collision monitor off through its ``toggle`` service, so the whole
safety layer is bypassed; ``true`` switches it back on. The parameter can be changed at runtime.

Honest claim: probabilistically safe within sensing range, no liveness guarantee (a reciprocal
dance or a standoff at a chokepoint is expected; liveness is the coordination layer's job).
"""

import csv
import math
import os

from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import Toggle
from parakram_msgs.msg import SafetyStatus
from parakram_safety import orca_core as oc
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros

CSV_COLUMNS = ['t', 'robot_id', 'min_obstacle_dist', 'filter_active', 'intervention',
               'cmd_in_v', 'cmd_in_w', 'cmd_out_v', 'cmd_out_w']


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp(msg_time, t):
    msg_time.sec = int(math.floor(t))
    msg_time.nanosec = int((t - math.floor(t)) * 1e9)
    return msg_time


class OrcaFilterNode(Node):
    """Per-robot, lidar-only NH-ORCA velocity filter."""

    def __init__(self, **kwargs):
        """Declare parameters and wire the I/O (``kwargs`` go to rclpy's Node, e.g. tests)."""
        super().__init__('orca_filter', **kwargs)
        ns = self.get_namespace().strip('/')
        dp = self.declare_parameter
        self.robot_id = dp('robot_id', ns or 'robot').value
        self.safety_enabled = bool(dp('safety_enabled', True).value)
        rate = float(dp('rate_hz', 20.0).value)
        self.scan_timeout = float(dp('scan_timeout', 0.5).value)
        self.cmd_timeout = float(dp('cmd_timeout', 0.5).value)
        self.base_frame = dp('base_frame', 'base_footprint').value
        self.odom_frame = dp('odom_frame', 'odom').value
        half_width = float(dp('footprint_half_width', 0.090).value)
        self.fp = oc.Footprint(x_min=float(dp('footprint_x_min', -0.105).value),
                               x_max=float(dp('footprint_x_max', 0.040).value),
                               y_min=-half_width, y_max=half_width)
        self.sp = oc.ScanParams(
            max_range_used=float(dp('max_range_used', 2.0).value),
            segment_gap=float(dp('blob_segment_gap', 0.10).value),
            median_window=int(dp('blob_median_window', 3).value),
            blob_max_extent=float(dp('blob_max_extent', 0.16).value),
            blob_depth_jump=float(dp('blob_depth_jump', 0.10).value),
            turret_radius=float(dp('peer_turret_radius', 0.05).value),
            peer_radius=float(dp('peer_radius', 0.116).value))
        self.fparams = oc.FilterParams(
            safety_margin=float(dp('safety_margin', 0.04).value),
            static_margin=float(dp('static_margin', 0.0).value),
            tracking_error=float(dp('tracking_error', 0.012).value),
            heading_time=float(dp('heading_time', 0.4).value),
            time_horizon_peer=float(dp('time_horizon_peer', 0.8).value),
            time_horizon_static=float(dp('time_horizon_static', 0.4).value),
            v_max=float(dp('v_max', 0.22).value),
            w_max=float(dp('w_max', 0.6).value),
            v_reverse_max=float(dp('v_reverse_max', 0.05).value),
            creep_speed=float(dp('creep_speed', 0.05).value))
        if oc.max_tracking_error(self.fparams) > self.fparams.tracking_error + 1e-9:
            raise ValueError(
                f'NH-ORCA: allowed velocities need a tracking error of '
                f'{oc.max_tracking_error(self.fparams):.4f} m > tracking_error '
                f'{self.fparams.tracking_error} m (lower heading_time or raise tracking_error)')
        self.tracker = oc.PeerTracker(oc.TrackParams(
            memory=float(dp('peer_memory', 1.0).value),
            hold_max=float(dp('peer_hold_max', 5.0).value)))
        self.lidar_fallback = (float(dp('lidar_x', -0.032).value), 0.0, 0.0)
        log_dir = dp('log_dir', '').value
        self.add_on_set_parameters_callback(self._on_params)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        # /tf and /tf_static are remapped to the robot namespace by the launch file.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_cmd = self.create_publisher(Twist, dp('cmd_out_topic', 'cmd_vel_safe').value,
                                             10)
        self.pub_status = self.create_publisher(SafetyStatus, 'safety_status', 10)
        self.create_subscription(Twist, dp('cmd_in_topic', 'cmd_vel_nav').value, self._on_cmd,
                                 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, dp('base_cmd_topic', 'cmd_vel').value, self._on_base,
                                 10)
        self.create_subscription(CollisionMonitorState, 'collision_monitor_state',
                                 self._on_monitor, 10)
        self.toggle_cli = self.create_client(Toggle, 'collision_monitor/toggle')

        self.nominal, self.t_nominal = (0.0, 0.0), None
        self.base_cmd = (0.0, 0.0)
        self.scan = None
        self.lidar_pose = None
        self.monitor_action, self.monitor_polygon = CollisionMonitorState.DO_NOTHING, ''
        self.monitor_enabled = None         # last confirmed collision monitor state
        self._toggle_pending = None
        self.interventions = 0
        self._was_intervening = False
        self.tf_fallbacks = 0

        self.csv_file = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            self.csv_file = open(os.path.join(log_dir, f'safety_{ns or self.robot_id}.csv'), 'w',
                                 newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(CSV_COLUMNS)
            self.rows = 0
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0, self._sync_monitor)
        self.get_logger().info(
            f'safety filter {self.robot_id}: {"ON" if self.safety_enabled else "OFF (A1)"}, '
            f'{rate:.0f} Hz, NH-ORCA disc {oc.robot_radius(self.fp):.3f}+'
            f'{self.fparams.tracking_error} m, T={self.fparams.heading_time}s, tau peer/static='
            f'{self.fparams.time_horizon_peer}/{self.fparams.time_horizon_static}s, margins '
            f'peer/static={self.fparams.safety_margin}/{self.fparams.static_margin} m, lidar only')

    # ------------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_params(self, params):
        for p in params:
            if p.name == 'safety_enabled':
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(successful=False, reason='bool expected')
                self.safety_enabled = bool(p.value)
                self.get_logger().warn(f'safety_enabled -> {self.safety_enabled}')
                self._sync_monitor()
        return SetParametersResult(successful=True)

    def _on_cmd(self, msg):
        self.nominal = (msg.linear.x, msg.angular.z)
        self.t_nominal = self._now()

    def _on_base(self, msg):
        self.base_cmd = (msg.linear.x, msg.angular.z)

    def _on_monitor(self, msg):
        self.monitor_action, self.monitor_polygon = msg.action_type, msg.polygon_name

    def _on_scan(self, msg):
        if self.lidar_pose is None:
            self.lidar_pose = self._lidar_pose(msg.header.frame_id)
        pose = self.lidar_pose or self.lidar_fallback     # param until the static TF arrives
        pts, idx, _ = oc.filtered_scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                              pose, self.sp)
        centres, beams = oc.find_blobs(msg.ranges, msg.angle_min, msg.angle_increment, pose,
                                       self.sp)
        stamp = Time.from_msg(msg.header.stamp)
        t, odom = stamp.nanoseconds * 1e-9, self._odom_pose(stamp)
        # peers missed by this scan's detection but still sensed stay in (odom-frame memory)
        centres, beams = self.tracker.update(t, odom, centres, beams, msg.ranges, msg.angle_min,
                                             msg.angle_increment, pose, self.sp)
        self.scan = {'t': t, 'stamp': stamp, 'pts': pts, 'idx': idx, 'centres': centres,
                     'beams': beams, 'odom': odom}

    def _lidar_pose(self, frame):
        """Lidar pose in the robot frame from the static TF, or None (retried every scan)."""
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = t.transform.translation
        pose = (tr.x, tr.y, _yaw(t.transform.rotation))
        self.get_logger().info(f'lidar pose from TF {self.base_frame} <- {frame}: {pose}')
        return pose

    def _odom_pose(self, stamp):
        """Pose odom -> base at ``stamp`` (latest if not yet available), or None."""
        for when in (stamp, Time()):
            try:
                t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, when)
                tr = t.transform.translation
                return (tr.x, tr.y, _yaw(t.transform.rotation))
            except tf2_ros.ExtrapolationException:
                continue
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                return None
        return None

    # ------------------------------------------------------------------ collision monitor
    def _sync_monitor(self):
        """Keep the collision monitor's enabled state equal to ``safety_enabled``."""
        want = self.safety_enabled
        if self.monitor_enabled == want or self._toggle_pending is not None:
            return
        if not self.toggle_cli.service_is_ready():
            return
        req = Toggle.Request()
        req.enable = want
        fut = self.toggle_cli.call_async(req)
        self._toggle_pending = want

        def done(f, want=want):
            self._toggle_pending = None
            res = f.result()
            if res is not None and res.success:
                self.monitor_enabled = want
                self.get_logger().info(f'collision monitor {"ON" if want else "OFF"}')
        fut.add_done_callback(done)

    # ------------------------------------------------------------------ tick
    def _tick(self):
        now = self._now()
        nominal = self.nominal if (self.t_nominal is not None and
                                   now - self.t_nominal <= self.cmd_timeout) else (0.0, 0.0)
        scan = self.scan
        scan_ok = scan is not None and now - scan['t'] <= self.scan_timeout
        clearance, n_blobs, min_barrier = math.inf, 0, math.inf
        if scan is not None:
            clearance = oc.sensed_clearance(scan['pts'], self.fp)
        if not self.safety_enabled:
            out, filtered = nominal, False
        elif not scan_ok:
            out = (0.0, 0.0)                                    # fail-safe: stop
            filtered = nominal != (0.0, 0.0)
        else:
            pts, centres = scan['pts'], scan['centres']
            here = self._odom_pose(Time())
            if scan['odom'] is not None and here is not None:
                x0, y0, t0 = scan['odom']
                c0, s0 = math.cos(t0), math.sin(t0)
                ddx, ddy = here[0] - x0, here[1] - y0
                dx, dy = c0 * ddx + s0 * ddy, -s0 * ddx + c0 * ddy
                dth = math.atan2(math.sin(here[2] - t0), math.cos(here[2] - t0))
                pts = oc.transform_points(pts, dx, dy, dth)
                centres = oc.transform_points(centres, dx, dy, dth)
                clearance = oc.sensed_clearance(pts, self.fp)
            else:
                self.tf_fallbacks += 1
            statics, peers = oc.obstacles_from_scan(pts, scan['idx'], centres, scan['beams'],
                                                    self.sp, self.fparams)
            # v_opt: the robot's current velocity = the command its base receives
            u_cur = oc.unicycle_to_holonomic(self.base_cmd[0], self.base_cmd[1], self.fparams)
            d = oc.filter_command(nominal, u_cur, statics, peers, clearance, self.fp,
                                  self.fparams, self.sp.peer_radius)
            out, filtered = (d.v, d.w), d.active
            n_blobs, min_barrier = d.n_blobs, d.min_barrier
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(out[0]), float(out[1])
        self.pub_cmd.publish(cmd)

        monitor_acting = self.safety_enabled and \
            self.monitor_action != CollisionMonitorState.DO_NOTHING
        intervening = bool(filtered or monitor_acting)
        if intervening and not self._was_intervening:
            self.interventions += 1
        self._was_intervening = intervening

        st = SafetyStatus()
        st.robot_id = self.robot_id
        _stamp(st.stamp, now)
        st.safety_enabled = self.safety_enabled
        st.monitor_enabled = bool(self.monitor_enabled)
        st.scan_ok = bool(scan_ok)
        st.min_obstacle_dist = float(clearance)
        st.filter_active = bool(filtered)
        st.intervention = intervening
        st.interventions = self.interventions
        st.monitor_action = int(self.monitor_action)
        st.monitor_polygon = self.monitor_polygon
        st.n_peer_blobs = int(n_blobs)
        st.min_barrier = float(min_barrier)
        st.cmd_in_v, st.cmd_in_w = float(nominal[0]), float(nominal[1])
        st.cmd_safe_v, st.cmd_safe_w = float(out[0]), float(out[1])
        st.cmd_out_v, st.cmd_out_w = float(self.base_cmd[0]), float(self.base_cmd[1])
        self.pub_status.publish(st)

        if self.csv_file is not None:
            self.csv.writerow([f'{now:.3f}', self.robot_id,
                               'inf' if math.isinf(clearance) else f'{clearance:.4f}',
                               int(filtered), int(intervening),
                               f'{nominal[0]:.4f}', f'{nominal[1]:.4f}',
                               f'{self.base_cmd[0]:.4f}', f'{self.base_cmd[1]:.4f}'])
            self.rows += 1
            if self.rows % 20 == 0:
                self.csv_file.flush()

    def close(self):
        """Flush the CSV log."""
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None


def main(args=None):
    """Run one safety filter."""
    rclpy.init(args=args)
    node = OrcaFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
