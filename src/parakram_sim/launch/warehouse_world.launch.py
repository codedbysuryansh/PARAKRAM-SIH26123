"""
Gazebo Harmonic warehouse world: gz server (+ optional GUI), /clock bridge, ground truth.

Rendering note (lidar is a rendering sensor in Harmonic): on the Parallels/virgl VM used for
development, ogre2/virgl returns min-range on every lidar beam. The defaults
``render_engine:=ogre software_gl:=true`` (OGRE 1.x on Mesa llvmpipe, server only) produce
correct scans there; on a machine with a real GPU use ``render_engine:=ogre2 software_gl:=false``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent,
                            ExecuteProcess, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    world = LaunchConfiguration('world').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    seed = int(LaunchConfiguration('seed').perform(context))
    engine = LaunchConfiguration('render_engine').perform(context)
    gui_engine = LaunchConfiguration('gui_render_engine').perform(context)
    software_gl = _truthy(LaunchConfiguration('software_gl').perform(context))
    gui = _truthy(LaunchConfiguration('gui').perform(context))
    verbosity = LaunchConfiguration('gz_verbosity').perform(context)
    clock_period = LaunchConfiguration('clock_period').perform(context)

    gz_cmd = ['gz', 'sim', '-s', '-r', '-v', verbosity, '--seed', str(seed),
              '--render-engine-server', engine, world]
    if software_gl:
        # OGRE/llvmpipe segfaults while tearing down its GL context on SIGINT (harmless, but it
        # leaves an apport crash report and an ERROR at every shutdown). The simulator has no
        # state to flush, so run it in its own session (terminal Ctrl-C does not reach it) and
        # stop it with SIGKILL when launch asks this wrapper to stop.
        wrapper = ('trap \'[ -n "$GZ" ] && kill -KILL "$GZ" 2>/dev/null; exit 0\' INT TERM; '
                   'setsid "$@" & GZ=$!; wait "$GZ"')
        server = ExecuteProcess(cmd=['bash', '-c', wrapper, 'gz_server'] + gz_cmd,
                                additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
                                output='screen', name='gz_server')
    else:
        server = ExecuteProcess(cmd=gz_cmd, output='screen', name='gz_server')
    actions = [
        server,
        # If the simulator dies, take the whole launch down instead of leaving Nav2 orphaned.
        RegisterEventHandler(OnProcessExit(
            target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='gz server exited'))])),
        # /clock at most every `clock_period` of sim time (physics still steps at 1 ms).
        Node(package='parakram_sim', executable='sim_clock_bridge', name='sim_clock_bridge',
             output='screen', parameters=[{'period': float(clock_period),
                                           'use_sim_time': False}]),
        # Stamps come from Gazebo itself; this node needs no /clock subscription.
        Node(package='parakram_sim', executable='ground_truth_publisher',
             name='ground_truth_publisher', output='screen',
             parameters=[{'world_name': world_name, 'use_sim_time': False}]),
    ]
    if gui:
        actions.append(ExecuteProcess(
            cmd=['gz', 'sim', '-g', '-v', verbosity, '--render-engine-gui', gui_engine],
            output='screen', name='gz_gui'))
    return actions


def generate_launch_description():
    """Launch the warehouse world."""
    sim_share = get_package_share_directory('parakram_sim')
    tb3_models = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(
            sim_share, 'worlds', 'warehouse.sdf')),
        DeclareLaunchArgument('world_name', default_value='warehouse',
                              description='<world name=...> inside the SDF'),
        DeclareLaunchArgument('seed', default_value='1', description='gz random seed'),
        DeclareLaunchArgument('render_engine', default_value='ogre',
                              description='gz server render engine (ogre | ogre2)'),
        DeclareLaunchArgument('software_gl', default_value='true',
                              description='LIBGL_ALWAYS_SOFTWARE=1 for the gz server'),
        DeclareLaunchArgument('gui', default_value='false', description='start the gz GUI'),
        DeclareLaunchArgument('gui_render_engine', default_value='ogre2'),
        DeclareLaunchArgument('gz_verbosity', default_value='2'),
        DeclareLaunchArgument('clock_period', default_value='0.005',
                              description='[s sim] /clock publication period (200 Hz)'),
        # Meshes of the Burger model (model://turtlebot3_common/...).
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', tb3_models),
        OpaqueFunction(function=_setup),
    ])
