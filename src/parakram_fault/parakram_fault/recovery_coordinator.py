"""
Recovery coordinator (CLAUDE_CODE/06): one per robot, the TASK side of recovery only.

On lease expiry nothing is needed for safety or liveness: coordination already treats the freed
cells as available and the silent robot's body as a ghost obstacle. This node only accelerates
task reallocation: when its watchdog declares a peer DEAD (``/<ns>/peer_status``), it asks its
own auction node to re-announce the peer's tasks (``/<ns>/reauction``) once per death episode,
and reports it on ``/fleet/recovery_event``. PARTITIONED / SUSPECT peers are left alone.
"""

import math

from parakram_msgs.msg import PeerStatus, RecoveryEvent
from parakram_msgs.srv import ReAuction
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

DEAD = PeerStatus.DEAD


class RecoveryCoordinator(Node):
    """Calls ReAuction on DEAD peers."""

    def __init__(self):
        """Subscribe to the watchdog, create the ReAuction client."""
        super().__init__('recovery_coordinator')
        ns = self.get_namespace().strip('/')
        self.me = self.declare_parameter('robot_id', ns or 'robot1').value
        self.loss_level = float(self.declare_parameter('loss', 0.0).value)
        self.client = self.create_client(ReAuction, 'reauction')
        self.done = set()
        self.pub_event = self.create_publisher(RecoveryEvent, '/fleet/recovery_event',
                                               QoSProfile(depth=100,
                                                          reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(PeerStatus, 'peer_status', self._on_status, 10)

    def _on_status(self, msg):
        if msg.status != DEAD:
            if msg.status in (PeerStatus.ALIVE, PeerStatus.FLAKY):
                self.done.discard(msg.peer_id)            # back: a later death counts again
            return
        if msg.peer_id in self.done or not self.client.service_is_ready():
            return
        self.done.add(msg.peer_id)
        req = ReAuction.Request()
        req.dead_robot_id = msg.peer_id
        fut = self.client.call_async(req)
        fut.add_done_callback(lambda f, p=msg.peer_id: self._reported(p, f))

    def _reported(self, peer, future):
        now = self.get_clock().now().nanoseconds * 1e-9
        n = future.result().tasks_reannounced if future.result() is not None else 0
        ev = RecoveryEvent()
        ev.stamp.sec, ev.stamp.nanosec = int(now), int((now - int(now)) * 1e9)
        ev.kind, ev.robot_id, ev.dead_id = 'reauction', self.me, peer
        ev.loss_level, ev.t_event = self.loss_level, float(now)
        ev.t_kill = ev.t_lease_expire = ev.t_space_reclaimed = ev.t_task_reassigned = math.nan
        ev.detail = f'{n} task(s) re-announced'
        self.pub_event.publish(ev)
        self.get_logger().warn(f'{peer} DEAD: ReAuction re-announced {n} task(s)')


def main(args=None):
    """Run one robot's recovery coordinator."""
    rclpy.init(args=args)
    node = RecoveryCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
