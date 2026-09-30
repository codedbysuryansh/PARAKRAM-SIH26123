"""Recovery logger (CLAUDE_CODE/06): fleet recovery events -> ``recovery_events.csv``."""

import csv
import math
import os

from parakram_msgs.msg import RecoveryEvent
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

COLUMNS = ['t_event', 'kind', 'robot_id', 'dead_id', 'task_id', 'loss_level', 't_kill',
           't_lease_expire', 't_space_reclaimed', 't_task_reassigned', 'detail']


def _fmt(v):
    return '' if isinstance(v, float) and math.isnan(v) else (f'{v:.3f}' if isinstance(
        v, float) else v)


class RecoveryLogger(Node):
    """Observer only: writes every recovery event of the fleet."""

    def __init__(self):
        """Open the CSV in ``log_dir``."""
        super().__init__('recovery_logger')
        log_dir = self.declare_parameter('log_dir', '').value or '.'
        os.makedirs(log_dir, exist_ok=True)
        self.file = open(os.path.join(log_dir, 'recovery_events.csv'), 'a', newline='')
        self.csv = csv.writer(self.file)
        if self.file.tell() == 0:
            self.csv.writerow(COLUMNS)
        self.create_subscription(RecoveryEvent, '/fleet/recovery_event', self._on_event,
                                 QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))

    def _on_event(self, m):
        self.csv.writerow([_fmt(v) for v in (
            m.t_event, m.kind, m.robot_id, m.dead_id, m.task_id, m.loss_level, m.t_kill,
            m.t_lease_expire, m.t_space_reclaimed, m.t_task_reassigned, m.detail)])
        self.file.flush()

    def close(self):
        """Close the CSV."""
        self.file.close()


def main(args=None):
    """Run the logger."""
    rclpy.init(args=args)
    node = RecoveryLogger()
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
