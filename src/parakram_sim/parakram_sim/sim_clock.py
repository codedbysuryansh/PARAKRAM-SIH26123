"""
Rate-limited gz -> ROS ``/clock`` bridge.

Gazebo publishes its clock on every physics step (1 kHz with the warehouse world's 1 ms step).
Every ROS node running on sim time subscribes to ``/clock`` (~50 subscriptions for 3 robots), so
bridging every step costs tens of thousands of DDS deliveries per second and starved Nav2 on the
development VM (controller loops at ~9 Hz instead of 20 Hz). Physics stays at 1 ms; this node
forwards the clock only when sim time has advanced by ``period`` (default 5 ms = 200 Hz), which
is ample for 20 Hz control loops and 5 Hz lidar.
"""

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class SimClockBridge(Node):
    """Forward gz ``/clock`` to ROS ``/clock`` at most once per ``period`` of sim time."""

    def __init__(self):
        """Declare parameters and subscribe to the gz clock."""
        super().__init__('sim_clock_bridge')
        period = float(self.declare_parameter('period', 0.005).value)
        gz_topic = self.declare_parameter('gz_topic', '/clock').value
        self._period_ns = int(period * 1e9)
        # RELIABLE is compatible with both reliable and best-effort /clock subscribers.
        self._pub = self.create_publisher(
            Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._lock = threading.Lock()
        self._last_ns = None
        self._closed = False
        self.context.on_shutdown(self.close)

        from gz.msgs10.clock_pb2 import Clock as GzClock
        from gz.transport13 import Node as GzNode
        self._gz = GzNode()
        if not self._gz.subscribe(GzClock, gz_topic, self._on_clock):
            raise RuntimeError(f'could not subscribe to gz topic {gz_topic}')
        self.get_logger().info(f'sim clock: gz {gz_topic} -> /clock every {period * 1e3:.1f} ms '
                               'of sim time')

    def _on_clock(self, msg):
        now_ns = msg.sim.sec * 1_000_000_000 + msg.sim.nsec
        with self._lock:
            if self._closed:
                return
            if self._last_ns is not None and 0 <= now_ns - self._last_ns < self._period_ns:
                return
            self._last_ns = now_ns
            out = Clock()
            out.clock.sec = int(msg.sim.sec)
            out.clock.nanosec = int(msg.sim.nsec)
            self._pub.publish(out)

    def close(self):
        """Stop forwarding (idempotent; called on context shutdown)."""
        with self._lock:
            self._closed = True


def main(args=None):
    """Run the clock bridge."""
    rclpy.init(args=args)
    node = SimClockBridge()
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
