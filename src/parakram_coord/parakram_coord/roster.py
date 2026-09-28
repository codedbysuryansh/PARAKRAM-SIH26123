"""
Peer discovery for the coordination node, and the optional ``/fleet/roster`` helper.

A robot learns its peers from any of three sources and remembers them:

* ``/fleet/roster`` — a lightweight helper that periodically publishes the robot ids it sees;
* its own periodic scan of the ROS graph for ``/<ns>/intent`` / ``/<ns>/state`` topics;
* (implicitly) the peer traffic it then receives.

The helper is a convenience, never a dependency: coordination itself only uses the peers'
``state`` / ``intent`` messages, and the node's own graph scan keeps discovering peers if the
helper dies (CLAUDE_CODE/02 acceptance: kill the roster helper, robots keep coordinating).
"""

import json
import re

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ROSTER_TOPIC = '/fleet/roster'
PEER_TOPIC_RE = re.compile(r'^/(robot\d+)/(intent|state)$')
ROSTER_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)


def robots_in_graph(topic_names_and_types, pattern=PEER_TOPIC_RE):
    """Robot ids found in a ``get_topic_names_and_types()`` listing."""
    out = set()
    for name, _types in topic_names_and_types:
        m = pattern.match(name)
        if m:
            out.add(m.group(1))
    return out


class Roster:
    """Tracks known peers of ``self_id``; calls ``on_new_peer(peer_id)`` once per new peer."""

    def __init__(self, node, self_id, on_new_peer, roster_topic=ROSTER_TOPIC, scan_period=5.0):
        """Subscribe to the helper and start the periodic graph scan on ``node``."""
        self.node = node
        self.self_id = self_id
        self.on_new_peer = on_new_peer
        self.sources = {}                 # peer id -> set of sources that reported it
        self.roster_msgs = 0
        node.create_subscription(String, roster_topic, self._on_roster, ROSTER_QOS)
        node.create_timer(scan_period, self.scan)

    def _add(self, peer_id, source):
        if peer_id == self.self_id:
            return
        is_new = peer_id not in self.sources
        self.sources.setdefault(peer_id, set()).add(source)
        if is_new:
            self.on_new_peer(peer_id)

    def _on_roster(self, msg):
        try:
            robots = json.loads(msg.data).get('robots', [])
        except (ValueError, AttributeError):
            return
        self.roster_msgs += 1
        for peer_id in robots:
            self._add(str(peer_id), 'roster')

    def scan(self):
        """Discover peers from the ROS graph (works without the helper)."""
        for peer_id in robots_in_graph(self.node.get_topic_names_and_types()):
            self._add(peer_id, 'graph')

    @property
    def peers(self):
        """Sorted list of known peer ids."""
        return sorted(self.sources)


class RosterHelper(Node):
    """Publishes the robot ids visible in the ROS graph on ``/fleet/roster`` (optional helper)."""

    def __init__(self):
        """Create the publisher and the periodic scan."""
        super().__init__('roster_helper')
        period = float(self.declare_parameter('period', 1.0).value)
        self.pub = self.create_publisher(String, ROSTER_TOPIC, ROSTER_QOS)
        self.robots = set()
        self.create_timer(period, self._tick)

    def _tick(self):
        self.robots |= robots_in_graph(self.get_topic_names_and_types())
        msg = String()
        msg.data = json.dumps({'robots': sorted(self.robots)})
        self.pub.publish(msg)


def helper_main(args=None):
    """Run the roster helper."""
    rclpy.init(args=args)
    node = RosterHelper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
