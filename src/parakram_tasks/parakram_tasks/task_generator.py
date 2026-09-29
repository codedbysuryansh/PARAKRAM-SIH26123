"""
Simulated task source (CLAUDE_CODE/04, sim only): streams pickup -> dropoff tasks.

Publishes ``n_tasks`` tasks on ``/fleet/tasks`` (RELIABLE + TRANSIENT_LOCAL, KEEP_ALL), one every
``1 / task_rate`` seconds of sim time, drawn with a seeded RNG from the scenario's
``task_stations`` (``warehouse_grid.yaml``). It is a task SOURCE, like a warehouse system
posting orders: it never announces, awards or tracks tasks (the robots do), and it keeps
running so robots that join late still receive every task.
"""

import random

from parakram_comms.qos import TASK_POOL_QOS
from parakram_msgs.msg import Task
from parakram_sim.grid_utils import default_grid_path, WarehouseGrid
from parakram_sim.robot_model import scenario as scenario_of
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def draw_tasks(grid, scenario_name, n_tasks, seed):
    """Return ``[(task_id, pickup_cell, dropoff_cell), ...]`` (deterministic in ``seed``)."""
    stations = scenario_of(grid, scenario_name).get('task_stations')
    if not stations:
        raise ValueError(f"scenario '{scenario_name}' defines no task_stations")
    pickups = [tuple(c) for c in stations['pickup']]
    dropoffs = [tuple(c) for c in stations['dropoff']]
    for cell in pickups + dropoffs:
        if not grid.is_free(*cell):
            raise ValueError(f'task station {cell} is not a free cell')
    rng = random.Random(seed)
    return [(f'task_{i + 1:03d}', rng.choice(pickups), rng.choice(dropoffs))
            for i in range(n_tasks)]


class TaskGenerator(Node):
    """Streams the scenario's tasks into the replicated pool."""

    def __init__(self, **kwargs):
        """Read parameters, draw the task list and start the stream."""
        super().__init__('task_generator', **kwargs)
        dp = self.declare_parameter
        self.n_tasks = int(dp('n_tasks', 20).value)
        rate = float(dp('task_rate', 0.2).value)
        seed = int(dp('seed', 1).value)
        scenario_name = dp('scenario', 'warehouse_stream').value
        self.start_delay = float(dp('start_delay', 5.0).value)
        grid_yaml = dp('grid_yaml', '').value
        self.grid = WarehouseGrid.from_yaml(grid_yaml or default_grid_path())
        self.tasks = draw_tasks(self.grid, scenario_name, self.n_tasks, seed)
        self.pub = self.create_publisher(Task, '/fleet/tasks', TASK_POOL_QOS)
        self.sent = 0
        self.t_first = None
        self.period = 1.0 / rate
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'task generator: {self.n_tasks} tasks at {rate} /s from '
                               f"scenario '{scenario_name}', seed {seed}")

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0 or self.sent >= self.n_tasks:
            return
        if self.t_first is None:
            self.t_first = now + self.start_delay
        if now < self.t_first + self.sent * self.period:
            return
        task_id, pickup, dropoff = self.tasks[self.sent]
        msg = Task()
        msg.task_id, msg.source = task_id, 'task_generator'
        px, py = self.grid.cell_to_world(*pickup)
        qx, qy = self.grid.cell_to_world(*dropoff)
        msg.pickup.x, msg.pickup.y = float(px), float(py)
        msg.dropoff.x, msg.dropoff.y = float(qx), float(qy)
        msg.stamp.sec = int(now)
        msg.stamp.nanosec = int((now - int(now)) * 1e9)
        self.pub.publish(msg)
        self.sent += 1
        self.get_logger().info(f'{task_id}: pickup {pickup} -> dropoff {dropoff} '
                               f'({self.sent}/{self.n_tasks})')


def main(args=None):
    """Run the simulated task stream."""
    rclpy.init(args=args)
    node = TaskGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
