"""
ONE namespaced Nav2 stack for one robot (identical in sim and on hardware).

Nodes (all under ``<namespace>``): amcl, planner_server, controller_server (RPP),
behavior_server, bt_navigator, velocity_smoother, and two lifecycle managers. The cmd_vel chain
(CLAUDE_CODE/03: controller -> collision_monitor -> velocity_smoother -> base) is
controller/behaviors -> cmd_vel_nav -> [parakram_safety: orca_filter -> cmd_vel_safe ->
collision_monitor] -> cmd_vel_monitored -> velocity_smoother -> cmd_vel -> base; the safety layer
is brought up by parakram_safety/launch/safety.launch.py, and without it nothing reaches the base
(fail-safe, never a bypass). The map_server is shared and started elsewhere
(fleet_sim.launch.py in sim). TF is isolated per robot: /tf -> /<ns>/tf, /tf_static ->
/<ns>/tf_static, frame ids un-prefixed (no tf_prefix).

``use_composition:=true`` (default) loads everything into one ``component_container_isolated``
per robot. With ~40 separate processes on one host, rmw_fastrtps intermittently dropped a
lifecycle ``change_state`` response ("client will not receive response"), leaving a robot's
AMCL configured-but-inactive forever. Composition keeps each lifecycle manager in the same
process as its nodes and cuts DDS participants ~4x; it is also the lighter option on a Pi 5.
``use_composition:=false`` runs the same nodes as separate processes.

``params_file`` must be the per-robot namespaced file (parakram_bringup.nav2_params renders it
from config/nav2_params.yaml).
"""

import json
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import yaml

TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
CMD_VEL_NAV = [('cmd_vel', 'cmd_vel_nav')]
# the smoother is the last stage: it takes the collision monitor's output and drives the base
SMOOTHER_REMAPS = [('cmd_vel', 'cmd_vel_monitored'), ('cmd_vel_smoothed', 'cmd_vel')]
# (package, executable, component plugin, node name, extra remaps)
NAV2_NODES = [
    ('nav2_amcl', 'amcl', 'nav2_amcl::AmclNode', 'amcl', []),
    ('nav2_planner', 'planner_server', 'nav2_planner::PlannerServer', 'planner_server', []),
    ('nav2_controller', 'controller_server', 'nav2_controller::ControllerServer',
     'controller_server', CMD_VEL_NAV),
    ('nav2_behaviors', 'behavior_server', 'behavior_server::BehaviorServer', 'behavior_server',
     CMD_VEL_NAV),
    ('nav2_bt_navigator', 'bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator', []),
    # Subscribes cmd_vel_monitored (the safety layer's output), publishes cmd_vel (the base).
    ('nav2_velocity_smoother', 'velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
     'velocity_smoother', SMOOTHER_REMAPS),
]
LOCALIZATION_NODES = ['amcl']
NAVIGATION_NODES = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
                    'velocity_smoother']


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _flatten(params, prefix=''):
    """Flatten nested ros__parameters dicts into dotted parameter names."""
    out = {}
    for key, value in params.items():
        if isinstance(value, dict):
            out.update(_flatten(value, f'{prefix}{key}.'))
        else:
            out[f'{prefix}{key}'] = value
    return out


def _setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = _truthy(LaunchConfiguration('use_sim_time').perform(context))
    autostart = _truthy(LaunchConfiguration('autostart').perform(context))
    use_respawn = _truthy(LaunchConfiguration('use_respawn').perform(context))
    use_composition = _truthy(LaunchConfiguration('use_composition').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    bond_timeout = float(LaunchConfiguration('bond_timeout').perform(context))
    if not namespace:
        raise RuntimeError('nav2_robot.launch.py: namespace is required (e.g. robot1)')
    if not params_file:
        raise RuntimeError('nav2_robot.launch.py: params_file is required')

    ns = '/' + namespace
    common = [params_file, {'use_sim_time': use_sim_time}]
    args = ['--ros-args', '--log-level', log_level]
    managers = [('lifecycle_manager_localization', LOCALIZATION_NODES),
                ('lifecycle_manager_navigation', NAVIGATION_NODES)]

    def manager_params(nodes):
        return [{'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': nodes,
                 'bond_timeout': bond_timeout}]

    if use_composition:
        # The params file must also reach the container PROCESS (as a global --params-file):
        # the costmaps are sub-nodes created inside planner/controller and only see global
        # parameters, exactly as in nav2_bringup's composed bringup.
        container = Node(package='rclcpp_components', executable='component_container_isolated',
                         name='nav2_container', namespace=ns, output='screen',
                         respawn=use_respawn, respawn_delay=2.0, arguments=args,
                         parameters=[params_file, {'use_sim_time': use_sim_time,
                                                   'autostart': autostart}],
                         remappings=TF_REMAPS)
        # rclcpp_components creates components with use_global_arguments(false), so each one
        # gets its own section of the params file explicitly (as LoadComposableNodes does). The
        # costmap sub-nodes are plain nodes and read the container's global --params-file.
        with open(params_file) as f:
            robot_params = yaml.safe_load(f).get(namespace, {})

        def component_params(name):
            section = robot_params.get(name, {}).get('ros__parameters', {})
            return {**_flatten(section), 'use_sim_time': use_sim_time}

        specs = [{'package': pkg, 'plugin': plugin, 'name': name, 'namespace': ns,
                  'remaps': TF_REMAPS + remaps, 'parameters': component_params(name)}
                 for pkg, _, plugin, name, remaps in NAV2_NODES]
        specs += [{'package': 'nav2_lifecycle_manager',
                   'plugin': 'nav2_lifecycle_manager::LifecycleManager', 'name': name,
                   'namespace': ns, 'remaps': [], 'parameters': manager_params(nodes)[0]}
                  for name, nodes in managers]
        spec_path = os.path.join(os.path.dirname(os.path.abspath(params_file)),
                                 f'nav2_{namespace}_components.json')
        with open(spec_path, 'w') as f:
            json.dump(specs, f, indent=1)
        # Verified, idempotent loading (see parakram_bringup/component_loader.py); a robot whose
        # stack cannot be loaded takes the whole launch down instead of running half-started.
        loader = Node(package='parakram_bringup', executable='load_components',
                      name=f'component_loader_{namespace}', output='screen',
                      arguments=['--container', f'{ns}/nav2_container', '--spec', spec_path])
        return [container, loader, RegisterEventHandler(OnProcessExit(
            target_action=loader,
            on_exit=lambda event, _ctx: [EmitEvent(event=Shutdown(
                reason=f'Nav2 component loading failed for {ns}'))]
            if event.returncode != 0 else []))]

    nodes = [Node(package=pkg, executable=exe, name=name, output='screen',
                  respawn=use_respawn, respawn_delay=2.0, parameters=common, arguments=args,
                  remappings=TF_REMAPS + remaps)
             for pkg, exe, _, name, remaps in NAV2_NODES]
    nodes += [Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name=name,
                   output='screen', arguments=args, parameters=manager_params(mgr_nodes))
              for name, mgr_nodes in managers]
    return [GroupAction([PushRosNamespace(namespace)] + nodes)]


def generate_launch_description():
    """Launch one namespaced Nav2 stack."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='robot namespace'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='per-robot namespaced Nav2 params'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='true',
                              description='one component container per robot'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('bond_timeout', default_value='10.0',
                              description='lifecycle bond timeout [s] (slow VM startup)'),
        OpaqueFunction(function=_setup),
    ])
