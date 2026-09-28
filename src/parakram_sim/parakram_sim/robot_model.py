"""Per-robot Gazebo model + ros_gz bridge rendering, and scenario spawn poses."""

import math
import os

import yaml


def _share(*parts):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('parakram_sim'), *parts)


def robot_sdf_template_path():
    """Path of the templated Burger SDF."""
    return _share('models', 'parakram_burger', 'model.sdf.in')


def render_robot_sdf(namespace, lidar_noise_std=0.02, lidar_rate=5.0, template_path=None):
    """Render the Burger SDF for ``namespace`` (model name == namespace, e.g. ``robot1``)."""
    with open(template_path or robot_sdf_template_path(), 'r') as f:
        sdf = f.read()
    sdf = (sdf.replace('@NS@', namespace)
              .replace('@LIDAR_NOISE_STD@', repr(float(lidar_noise_std)))
              .replace('@LIDAR_RATE@', repr(float(lidar_rate))))
    if '@' in sdf.split('-->', 1)[-1]:
        raise ValueError('unrendered token left in robot SDF template')
    return sdf


def bridge_entries(namespace):
    """ros_gz_bridge entries for one robot. ROS and gz topic names are identical."""
    ns = '/' + namespace.strip('/')
    return [
        {'ros_topic_name': f'{ns}/odom', 'gz_topic_name': f'{ns}/odom',
         'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'{ns}/tf', 'gz_topic_name': f'{ns}/tf',
         'ros_type_name': 'tf2_msgs/msg/TFMessage', 'gz_type_name': 'gz.msgs.Pose_V',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'{ns}/joint_states', 'gz_topic_name': f'{ns}/joint_states',
         'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'{ns}/imu', 'gz_topic_name': f'{ns}/imu',
         'ros_type_name': 'sensor_msgs/msg/Imu', 'gz_type_name': 'gz.msgs.IMU',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'{ns}/scan', 'gz_topic_name': f'{ns}/scan',
         'ros_type_name': 'sensor_msgs/msg/LaserScan', 'gz_type_name': 'gz.msgs.LaserScan',
         'direction': 'GZ_TO_ROS'},
        # cmd_vel is geometry_msgs/Twist (CLAUDE_CODE/01 interface; Nav2 unstamped default).
        {'ros_topic_name': f'{ns}/cmd_vel', 'gz_topic_name': f'{ns}/cmd_vel',
         'ros_type_name': 'geometry_msgs/msg/Twist', 'gz_type_name': 'gz.msgs.Twist',
         'direction': 'ROS_TO_GZ'},
    ]


def render_bridge_yaml(namespace):
    """Render the ros_gz_bridge config YAML for one robot."""
    return yaml.safe_dump(bridge_entries(namespace), sort_keys=False)


def scenario(grid, name):
    """Return the scenario dict ``name`` from the grid YAML (raises KeyError with choices)."""
    scenarios = grid.raw.get('scenarios', {})
    if name not in scenarios:
        raise KeyError(f"unknown scenario '{name}'; available: {sorted(scenarios)}")
    return scenarios[name]


def spawn_poses(grid, scenario_name, n_robots):
    """Return ``[(namespace, x, y, yaw_rad, (r, c)), ...]`` for robot1..robotN."""
    sc = scenario(grid, scenario_name)
    starts = sc.get('start_cells', [])
    if n_robots > len(starts):
        raise ValueError(f"scenario '{scenario_name}' defines {len(starts)} start cells, "
                         f'n_robots={n_robots} requested')
    if n_robots < 1:
        raise ValueError('n_robots must be >= 1')
    poses = []
    for i in range(n_robots):
        r, c, yaw_deg = starts[i]
        if grid.is_blocked(r, c):
            raise ValueError(f'start cell {(r, c)} for robot{i + 1} is blocked')
        x, y = grid.cell_to_world(r, c)
        poses.append((f'robot{i + 1}', x, y, math.radians(yaw_deg), (int(r), int(c))))
    cells = [p[4] for p in poses]
    if len(set(cells)) != len(cells):
        raise ValueError(f"scenario '{scenario_name}' has duplicate start cells")
    return poses
