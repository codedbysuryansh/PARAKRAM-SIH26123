"""
Spawn N namespaced TurtleBot3 Burgers (robot1..robotN) at distinct scenario start cells.

Per robot ``robot{i}``: the Burger SDF rendered with namespaced gz topics, a ros_gz bridge
(odom, tf, joint_states, imu, scan -> ROS; cmd_vel -> gz), and robot_state_publisher with the
turtlebot3_description URDF (the same URDF the real robot uses). TF is isolated per namespace
(/robot{i}/tf, /robot{i}/tf_static) with un-prefixed frame ids: no tf_prefix.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    from parakram_sim.grid_utils import WarehouseGrid
    from parakram_sim.robot_model import render_bridge_yaml, render_robot_sdf, spawn_poses

    n_robots = int(LaunchConfiguration('n_robots').perform(context))
    scenario = LaunchConfiguration('scenario').perform(context)
    grid_path = LaunchConfiguration('grid').perform(context)
    out_dir = LaunchConfiguration('generated_dir').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    noise = float(LaunchConfiguration('lidar_noise_std').perform(context))
    rate = float(LaunchConfiguration('lidar_rate').perform(context))
    use_sim_time = _truthy(LaunchConfiguration('use_sim_time').perform(context))
    if not out_dir:
        out_dir = tempfile.mkdtemp(prefix='parakram_spawn_')
    os.makedirs(out_dir, exist_ok=True)

    grid = WarehouseGrid.from_yaml(grid_path)
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'),
                        'urdf', 'turtlebot3_burger.urdf')
    robot_description = ParameterValue(Command(['xacro ', urdf, ' namespace:=']),
                                       value_type=str)
    actions = []
    for ns, x, y, yaw, cell in spawn_poses(grid, scenario, n_robots):
        sdf_path = os.path.join(out_dir, f'{ns}.sdf')
        with open(sdf_path, 'w') as f:
            f.write(render_robot_sdf(ns, lidar_noise_std=noise, lidar_rate=rate))
        bridge_path = os.path.join(out_dir, f'{ns}_bridge.yaml')
        with open(bridge_path, 'w') as f:
            f.write(render_bridge_yaml(ns))
        actions += [
            Node(package='ros_gz_sim', executable='create', name=f'spawn_{ns}', output='screen',
                 arguments=['-world', world_name, '-name', ns, '-file', sdf_path,
                            '-x', f'{x:.4f}', '-y', f'{y:.4f}', '-z', '0.01',
                            '-Y', f'{yaw:.6f}']),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                 namespace=ns, output='screen',
                 parameters=[{'config_file': bridge_path, 'use_sim_time': use_sim_time}]),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', namespace=ns, output='screen',
                 parameters=[{'robot_description': robot_description,
                              'use_sim_time': use_sim_time}],
                 remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        ]
    return actions


def generate_launch_description():
    """Launch the robot spawners."""
    sim_share = get_package_share_directory('parakram_sim')
    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='3'),
        DeclareLaunchArgument('scenario', default_value='intersection'),
        DeclareLaunchArgument('grid', default_value=os.path.join(
            sim_share, 'config', 'warehouse_grid.yaml')),
        DeclareLaunchArgument('generated_dir', default_value='',
                              description='where per-robot SDF/bridge files are written'),
        DeclareLaunchArgument('world_name', default_value='warehouse'),
        DeclareLaunchArgument('lidar_noise_std', default_value='0.02',
                              description='lidar range noise sigma [m]'),
        DeclareLaunchArgument('lidar_rate', default_value='5.0',
                              description='lidar rate [Hz] (LDS-01/02: 5 Hz)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
