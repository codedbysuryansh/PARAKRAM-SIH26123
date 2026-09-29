"""
Node-level integration tests of the safety filter (CLAUDE_CODE/03), in-process, no simulator.

A driver node plays the robot's lidar and Nav2: it publishes synthetic scans and nominal
commands in the filter's namespace and reads back cmd_vel_safe and safety_status.
"""

import csv
import math
import time

from geometry_msgs.msg import Twist
from parakram_msgs.msg import SafetyStatus
from parakram_safety.orca_filter import CSV_COLUMNS, OrcaFilterNode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

NS = '/filter_test'
LIDAR_X = -0.032


def turret_scan(peer_x=None, n=360):
    """Return a scan with (optionally) a peer's lidar turret (r = 0.05) at (peer_x, 0)."""
    ranges = []
    for i in range(n):
        a = -math.pi + i * 2 * math.pi / n
        r = math.inf
        if peer_x is not None:
            ox = peer_x - LIDAR_X
            b = ox * math.cos(a)
            disc = b * b - (ox * ox - 0.05 ** 2)
            if disc >= 0 and b - math.sqrt(disc) > 0:
                r = b - math.sqrt(disc)
        ranges.append(r if 0.12 < r < 3.5 else math.inf)
    return ranges


class Driver:
    def __init__(self, node):
        self.node = node
        self.pub_scan = node.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.pub_cmd = node.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.out, self.status = None, None
        node.create_subscription(Twist, 'cmd_vel_safe', lambda m: setattr(
            self, 'out', (m.linear.x, m.angular.z)), 10)
        node.create_subscription(SafetyStatus, 'safety_status',
                                 lambda m: setattr(self, 'status', m), 10)
        self.peer_x, self.scan_on, self.cmd = None, True, (0.18, 0.0)

    def publish(self):
        m = Twist()
        m.linear.x, m.angular.z = self.cmd
        self.pub_cmd.publish(m)
        if self.scan_on:
            s = LaserScan()
            s.header.stamp = self.node.get_clock().now().to_msg()
            s.header.frame_id = 'base_scan'
            s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / 360
            s.angle_max = s.angle_min + 359 * s.angle_increment
            s.range_min, s.range_max = 0.12, 3.5
            s.ranges = turret_scan(self.peer_x)
            self.pub_scan.publish(s)


@pytest.fixture(scope='module')
def rig(tmp_path_factory):
    rclpy.init()
    log_dir = tmp_path_factory.mktemp('safety_logs')
    filt = OrcaFilterNode(namespace=NS, parameter_overrides=[
        Parameter('log_dir', value=str(log_dir)), Parameter('robot_id', value='robotT')])
    drv_node = rclpy.create_node('filter_test_driver', namespace=NS)
    drv = Driver(drv_node)
    ex = SingleThreadedExecutor()
    ex.add_node(filt)
    ex.add_node(drv_node)

    def run(seconds):
        end = time.time() + seconds
        next_pub = 0.0
        while time.time() < end:
            if time.time() >= next_pub:
                drv.publish()
                next_pub = time.time() + 0.1          # scans + commands at 10 Hz
            ex.spin_once(timeout_sec=0.01)
    yield filt, drv, run, log_dir
    filt.close()
    ex.shutdown()
    filt.destroy_node()
    drv_node.destroy_node()
    rclpy.shutdown()


def test_free_space_passes_the_nominal_through(rig):
    _, drv, run, _ = rig
    drv.peer_x, drv.scan_on, drv.cmd = None, True, (0.18, 0.2)
    run(1.0)
    assert drv.out == pytest.approx((0.18, 0.2), abs=1e-6)
    assert drv.status.scan_ok and not drv.status.filter_active


def test_peer_ahead_is_filtered_and_counted(rig):
    _, drv, run, _ = rig
    before = drv.status.interventions
    # NH-ORCA peer radius: disc 0.1383 + 0.012 + peer 0.116 + margin 0.04 = 0.306 m from the axle.
    # Turret 0.40 m ahead: outside it, so the robot only slows (it shares the avoidance).
    drv.peer_x, drv.cmd = 0.40, (0.18, 0.0)
    run(1.0)
    assert 0.0 < drv.out[0] < 0.18 - 1e-3
    st = drv.status
    assert st.filter_active and st.intervention and st.n_peer_blobs == 1
    assert st.interventions == before + 1
    assert st.min_obstacle_dist == pytest.approx(0.40 - 0.05 - 0.04, abs=0.01)
    # turret 0.22 m ahead: inside the peer radius, so the robot backs away (creep escape)
    drv.peer_x = 0.22
    run(1.0)
    assert -0.05 - 1e-9 <= drv.out[0] < 0.0
    assert drv.status.min_barrier < 0.0
    assert drv.status.min_obstacle_dist == pytest.approx(0.22 - 0.05 - 0.04, abs=0.01)


def test_lidar_loss_fails_safe_to_zero(rig):
    _, drv, run, _ = rig
    drv.peer_x, drv.cmd = None, (0.18, 0.3)
    run(0.5)
    drv.scan_on = False
    run(1.0)                                       # > scan_timeout (0.5 s)
    assert drv.out == (0.0, 0.0)
    assert not drv.status.scan_ok and drv.status.filter_active
    drv.scan_on = True
    run(0.5)
    assert drv.out == pytest.approx((0.18, 0.3), abs=1e-6)


def test_safety_enabled_false_bypasses_the_filter_at_runtime(rig):
    filt, drv, run, _ = rig
    drv.peer_x, drv.cmd = 0.22, (0.18, 0.0)
    filt.set_parameters([Parameter('safety_enabled', value=False)])
    run(1.0)
    assert drv.out == pytest.approx((0.18, 0.0), abs=1e-6)     # ablation A1: pass-through
    assert not drv.status.safety_enabled and not drv.status.filter_active
    filt.set_parameters([Parameter('safety_enabled', value=True)])
    run(1.0)
    assert drv.out[0] < 0.18 - 1e-3


def test_comms_free_by_construction(rig):
    filt, _, _, _ = rig
    topics = {s.topic_name for s in filt.subscriptions}
    allowed = {f'{NS}/{t}' for t in ('cmd_vel_nav', 'scan', 'cmd_vel', 'collision_monitor_state')}
    allowed |= {'/tf', '/tf_static', '/parameter_events'}
    assert topics <= allowed, topics - allowed
    assert not any(t.endswith(('/state', '/intent')) or t.startswith('/fleet') for t in topics)


def test_csv_log_has_the_required_columns(rig):
    filt, _, _, log_dir = rig
    filt.csv_file.flush()
    with open(log_dir / 'safety_filter_test.csv') as f:
        rows = list(csv.reader(f))
    assert rows[0] == CSV_COLUMNS == ['t', 'robot_id', 'min_obstacle_dist', 'filter_active',
                                      'intervention', 'cmd_in_v', 'cmd_in_w', 'cmd_out_v',
                                      'cmd_out_w']
    assert len(rows) > 50 and all(r[1] == 'robotT' for r in rows[1:])
    assert any(r[3] == '1' for r in rows[1:]) and any(r[3] == '0' for r in rows[1:])
