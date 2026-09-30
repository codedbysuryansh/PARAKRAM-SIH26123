"""
Heartbeat node (CLAUDE_CODE/06): the robot's heartbeat IS its lease renewal.

Relays every lease renewal its own coordination node publishes (``/<ns>/intent``, local) as a
compact ``/<ns>/heartbeat`` (same seq, send stamp) for consumers that only need liveness (the
task layer's stale-bidder check, the dashboard). It never beats on its own: when coordination
stops renewing (dead, hung), the heartbeat stops with it. One mechanism, not two.
"""

from parakram_comms.qos import HEARTBEAT_QOS, INTENT_QOS
from parakram_msgs.msg import Heartbeat, Intent
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class HeartbeatNode(Node):
    """Relays own lease renewals as heartbeats."""

    def __init__(self):
        """Wire own intent -> heartbeat."""
        super().__init__('heartbeat')
        ns = self.get_namespace().strip('/')
        self.me = self.declare_parameter('robot_id', ns or 'robot1').value
        self.pub = self.create_publisher(Heartbeat, 'heartbeat', HEARTBEAT_QOS)
        self.create_subscription(Intent, 'intent', self._on_intent, INTENT_QOS)

    def _on_intent(self, msg):
        hb = Heartbeat()
        hb.robot_id, hb.seq, hb.stamp = self.me, msg.seq, msg.stamp
        self.pub.publish(hb)


def main(args=None):
    """Run the relay."""
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
