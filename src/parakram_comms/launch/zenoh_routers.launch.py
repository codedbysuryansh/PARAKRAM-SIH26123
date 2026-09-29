"""
One rmw_zenoh router per robot (CLAUDE_CODE/05 peer mesh).

    # single-host simulation: every robot's router on this machine (ports 7447 / 7448 / 7449)
    ros2 launch parakram_comms zenoh_routers.launch.py robots:=robot1,robot2,robot3
    # hardware: on each robot, only its own router (configs from parakram_comms.zenoh_mesh)
    ros2 launch parakram_comms zenoh_routers.launch.py robots:=robot2 config_dir:=$HOME/zenoh

Starts ``rmw_zenohd`` for each listed robot with ``ZENOH_ROUTER_CONFIG_URI`` =
``<config_dir>/zenoh_router_<robot>.json5``. Each router connects to the other robots' routers
(full mesh, retried forever), so there is no central router. A robot's nodes then run with
``RMW_IMPLEMENTATION=rmw_zenoh_cpp`` and ``ZENOH_SESSION_CONFIG_URI=<config_dir>/
zenoh_session_<robot>.json5``.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context):
    robots = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',')
              if r.strip()]
    config_dir = LaunchConfiguration('config_dir').perform(context) or os.path.join(
        get_package_share_directory('parakram_comms'), 'config')
    zenohd = os.path.join(get_package_prefix('rmw_zenoh_cpp'), 'lib', 'rmw_zenoh_cpp',
                          'rmw_zenohd')
    actions = []
    for robot in robots:
        cfg = os.path.join(config_dir, f'zenoh_router_{robot}.json5')
        if not os.path.isfile(cfg):
            raise RuntimeError(f'no router config for {robot}: {cfg} (generate it with '
                               'python3 -m parakram_comms.zenoh_mesh)')
        actions += [LogInfo(msg=f'[parakram_comms] Zenoh router of {robot}: {cfg}'),
                    ExecuteProcess(cmd=[zenohd], name=f'zenoh_router_{robot}', output='screen',
                                   additional_env={'ZENOH_ROUTER_CONFIG_URI': cfg})]
    return actions


def generate_launch_description():
    """Launch the listed robots' routers."""
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='robot1,robot2,robot3',
                              description='robots whose router runs on this machine'),
        DeclareLaunchArgument('config_dir', default_value='',
                              description='default: the installed parakram_comms/config'),
        OpaqueFunction(function=_setup)])
