"""
Idempotent, verified loading of composable nodes into a running component container.

Replaces launch_ros ``LoadComposableNodes`` for the per-robot Nav2 container. Under discovery
load, rmw_fastrtps drops a service response if the requester's reply reader is not yet matched
("failed to send response ... client will not receive response"). ``LoadComposableNodes`` then
waits forever and that robot's Nav2 never comes up (seen in 1 of 8 composed bringups on the
development VM). This loader:

* calls ``<container>/_container/load_node`` with a timeout for each component, in order;
* if a response is lost, asks ``<container>/_container/list_nodes`` whether the node exists
  before retrying (never loads a duplicate);
* exits non-zero if a component cannot be loaded or verified, so the launch can fail fast instead
  of leaving a half-started fleet that a benchmark might record as a valid trial.

Usage: ``load_components --container /robot1/nav2_container --spec <components.json>`` where the
spec is a JSON list of ``{package, plugin, name, namespace, remaps: [[from, to]], parameters}``.
"""

import argparse
import json
import sys
import time

from composition_interfaces.srv import ListNodes, LoadNode
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def full_name(namespace, name):
    """Fully qualified node name."""
    ns = '/' + namespace.strip('/') if namespace.strip('/') else ''
    return f'{ns}/{name}'


class ComponentLoader(Node):
    """Loads a list of components into one container, verifying each."""

    def __init__(self, container):
        """Create the load/list clients for ``container``."""
        super().__init__('component_loader_' + container.strip('/').replace('/', '_'))
        self.container = container
        self.load = self.create_client(LoadNode, f'{container}/_container/load_node')
        self.list = self.create_client(ListNodes, f'{container}/_container/list_nodes')

    def _call(self, client, request, timeout):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.done():
            return future.result()
        client.remove_pending_request(future)
        return None

    def loaded_nodes(self, timeout=5.0, attempts=3):
        """Return the container's node names, or None if it never answered."""
        for _ in range(attempts):
            resp = self._call(self.list, ListNodes.Request(), timeout)
            if resp is not None:
                return set(resp.full_node_names)
        return None

    def load_one(self, spec, timeout=10.0, attempts=5):
        """Load one component; return True once it is verified present."""
        fqn = full_name(spec['namespace'], spec['name'])
        req = LoadNode.Request()
        req.package_name = spec['package']
        req.plugin_name = spec['plugin']
        req.node_name = spec['name']
        req.node_namespace = spec['namespace']
        req.remap_rules = [f'{a}:={b}' for a, b in spec.get('remaps', [])]
        req.parameters = [Parameter(k, value=v).to_parameter_msg()
                          for k, v in spec.get('parameters', {}).items()]
        for attempt in range(1, attempts + 1):
            resp = self._call(self.load, req, timeout)
            if resp is not None and resp.success:
                self.get_logger().info(f'loaded {resp.full_node_name}')
                return True
            if resp is not None:
                self.get_logger().error(f'load_node {fqn} failed: {resp.error_message}')
            else:
                self.get_logger().warn(f'load_node {fqn}: no response (attempt {attempt}); '
                                       'verifying with list_nodes')
            present = self.loaded_nodes()
            if present is None:
                self.get_logger().error(f'container {self.container} not answering list_nodes')
                return False
            if fqn in present:
                self.get_logger().warn(f'{fqn} is present (its load response was lost)')
                return True
        return False


def main(argv=None):
    """Entry point."""
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--container', required=True)
    parser.add_argument('--spec', required=True)
    parser.add_argument('--service-timeout', type=float, default=120.0)
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])
    with open(args.spec) as f:
        specs = json.load(f)

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = ComponentLoader(args.container)
    ok = False
    try:
        deadline = time.monotonic() + args.service_timeout
        while not (node.load.service_is_ready() and node.list.service_is_ready()):
            if time.monotonic() > deadline:
                node.get_logger().error(f'{args.container} services not available')
                return 1
            rclpy.spin_once(node, timeout_sec=0.2)
        ok = all(node.load_one(spec) for spec in specs)
        if ok:
            node.get_logger().info(f'all {len(specs)} components loaded into {args.container}')
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
