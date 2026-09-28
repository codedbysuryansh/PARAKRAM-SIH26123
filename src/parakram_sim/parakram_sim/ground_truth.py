"""
Simulator ground truth: named robot poses straight from Gazebo (bench/verification only).

CLAUDE_CODE/00: ground truth must be independent of every robot's own state (sim: the
simulator's model poses; hardware: the overhead camera). Gazebo Harmonic has no
``/model_states`` (that was Gazebo Classic), and bridging ``/world/<w>/dynamic_pose/info``
through ros_gz_bridge drops the model names, so this node reads the Pose_V directly with the gz
transport Python bindings and republishes the poses of the robot models as a single
``tf2_msgs/TFMessage`` on ``/ground_truth/poses``:

* ``header.frame_id``   = ``map`` (the map frame coincides with the Gazebo world frame)
* ``child_frame_id``    = model name (``robot1`` ... = the robot namespace)
* ``header.stamp``      = simulation time of the Gazebo sample
* pose                  = model origin = ``base_footprint``

It is one topic with all robots per timestamp, like the overhead camera rig. It is NOT routed
through any comms/loss layer and no robot node may subscribe to it.
"""

import re
import threading

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class GroundTruthPublisher(Node):
    """Republish Gazebo model poses of the robots on ``/ground_truth/poses``."""

    def __init__(self):
        """Declare parameters and subscribe to the gz pose stream."""
        super().__init__('ground_truth_publisher')
        self.world = self.declare_parameter('world_name', 'warehouse').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        pattern = self.declare_parameter('model_regex', r'^robot\d+$').value
        topic = self.declare_parameter('topic', '/ground_truth/poses').value
        self._re = re.compile(pattern)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(TFMessage, topic, qos)
        self._lock = threading.Lock()
        self._count = 0
        self._closed = False
        # The gz transport callback runs on its own thread; stop publishing before rclpy tears
        # the publisher down on shutdown.
        self.context.on_shutdown(self.close)

        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self._gz = GzNode()
        gz_topic = f'/world/{self.world}/dynamic_pose/info'
        if not self._gz.subscribe(Pose_V, gz_topic, self._on_pose_v):
            raise RuntimeError(f'could not subscribe to gz topic {gz_topic}')
        self.get_logger().info(f'ground truth: {gz_topic} -> {topic} (models /{pattern}/)')
        self.create_timer(10.0, self._report)

    def _on_pose_v(self, msg):
        out = TFMessage()
        sec, nsec = msg.header.stamp.sec, msg.header.stamp.nsec
        for p in msg.pose:
            if not self._re.match(p.name):
                continue
            t = TransformStamped()
            t.header.stamp.sec = int(sec)
            t.header.stamp.nanosec = int(nsec)
            t.header.frame_id = self.frame_id
            t.child_frame_id = p.name
            t.transform.translation.x = p.position.x
            t.transform.translation.y = p.position.y
            t.transform.translation.z = p.position.z
            t.transform.rotation.x = p.orientation.x
            t.transform.rotation.y = p.orientation.y
            t.transform.rotation.z = p.orientation.z
            t.transform.rotation.w = p.orientation.w
            out.transforms.append(t)
        if out.transforms:
            with self._lock:
                if self._closed:
                    return
                self._pub.publish(out)
                self._count += 1

    def close(self):
        """Stop republishing (idempotent; called on context shutdown)."""
        with self._lock:
            self._closed = True

    def _report(self):
        with self._lock:
            n, self._count = self._count, 0
        if n == 0:
            self.get_logger().warn('no robot poses received from Gazebo in the last 10 s')


def main(args=None):
    """Run the ground-truth publisher."""
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise  # a genuine failure while running
        # otherwise: an entity was taken while the context was being shut down
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
