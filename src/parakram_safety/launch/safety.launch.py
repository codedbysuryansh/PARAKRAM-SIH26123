"""
Reactive safety layer for N robots (CLAUDE_CODE/03): comms-free, lidar only.

    ros2 launch parakram_bringup fleet_sim.launch.py n_robots:=3 scenario:=headon seed:=1 \
        safety:=false
    ros2 launch parakram_safety safety.launch.py n_robots:=3

(``fleet_sim.launch.py`` includes this file itself by default, ``safety:=true``.)

Per robot, in its namespace, it brings up the Part-A ``collision_monitor`` (Nav2, config
``parakram_safety/config/collision_monitor.yaml``) and the Part-B NH-ORCA filter ``orca_filter``,
and so wires the cmd_vel chain of the work order (controller -> collision_monitor ->
velocity_smoother -> base)::

    controller/behaviors --cmd_vel_nav--> orca_filter --cmd_vel_safe--> collision_monitor
    --cmd_vel_monitored--> velocity_smoother --cmd_vel--> base

The velocity smoother is Nav2's (started by ``parakram_bringup/launch/nav2_robot.launch.py``,
input remapped to ``cmd_vel_monitored``). The monitor configures and activates itself
(``autostart_node``); the filter switches it on or off with ``safety_enabled``. Without this
launch nothing publishes ``cmd_vel_monitored``, so the base receives no command at all: an absent
safety layer fails safe, it is never bypassed.

``safety_enabled:=false`` is the ablation hook of benchmark A1 (filter pass-through and collision
monitor off). Logs: ``<run_dir>/safety_<ns>.csv`` (default run_dir: ``bench/logs/latest``),
``safety_manifest.json`` and the rendered monitor parameters under ``<run_dir>/generated``.
"""

import hashlib
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
FILTER_PARAMS = ('rate_hz', 'safety_margin', 'static_margin', 'tracking_error', 'heading_time',
                 'time_horizon_peer', 'time_horizon_static', 'scan_timeout', 'cmd_timeout',
                 'v_max', 'w_max', 'v_reverse_max', 'creep_speed', 'peer_radius')
DEFAULTS = {'rate_hz': '20.0', 'safety_margin': '0.04', 'static_margin': '0.0',
            'tracking_error': '0.012', 'heading_time': '0.4', 'time_horizon_peer': '0.8',
            'time_horizon_static': '0.4', 'scan_timeout': '0.5', 'cmd_timeout': '0.5',
            'v_max': '0.22', 'w_max': '0.6', 'v_reverse_max': '0.05', 'creep_speed': '0.05',
            'peer_radius': '0.116'}


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    from parakram_safety.run_paths import latest_run_dir

    cfg = {k: LaunchConfiguration(k).perform(context)
           for k in ('n_robots', 'safety_enabled', 'run_dir', 'use_sim_time') + FILTER_PARAMS}
    n_robots = int(cfg['n_robots'])
    safety_enabled = _truthy(cfg['safety_enabled'])
    use_sim_time = _truthy(cfg['use_sim_time'])
    run_dir = cfg['run_dir'] or latest_run_dir()
    gen_dir = os.path.join(run_dir, 'generated')
    os.makedirs(gen_dir, exist_ok=True)
    params = {k: float(cfg[k]) for k in FILTER_PARAMS}
    cm_file = os.path.join(get_package_share_directory('parakram_safety'), 'config',
                           'collision_monitor.yaml')
    with open(cm_file, 'rb') as f:
        raw = f.read()
    cm_sha = hashlib.sha256(raw).hexdigest()
    cm_section = yaml.safe_load(raw)['collision_monitor']
    actions = [LogInfo(msg=f'[parakram_safety] {n_robots} robots, safety '
                           f'{"ON" if safety_enabled else "OFF (ablation A1)"}, '
                           f'logging to {run_dir}')]
    monitor_files = {}
    for i in range(1, n_robots + 1):
        ns = f'robot{i}'
        # the monitor's parameters, nested under this robot's namespace (archived with the run)
        section = json.loads(json.dumps(cm_section))
        section['ros__parameters']['use_sim_time'] = use_sim_time
        path = os.path.join(gen_dir, f'collision_monitor_{ns}.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump({ns: {'collision_monitor': section}}, f, sort_keys=False)
        monitor_files[ns] = path
        actions.append(Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', namespace=ns, output='screen', parameters=[path],
            remappings=TF_REMAPS))
        actions.append(Node(
            package='parakram_safety', executable='orca_filter', name='orca_filter',
            namespace=ns, output='screen',
            parameters=[dict(params, robot_id=ns, safety_enabled=safety_enabled,
                             log_dir=run_dir, use_sim_time=use_sim_time)],
            remappings=TF_REMAPS))
    manifest = {'kind': 'safety', 'n_robots': n_robots, 'safety_enabled': safety_enabled,
                'filter': 'NH-ORCA', 'filter_params': params,
                'collision_monitor_config': cm_file, 'collision_monitor_config_sha256': cm_sha,
                'collision_monitor_params': monitor_files,
                'cmd_vel_chain': ['cmd_vel_nav', 'orca_filter', 'cmd_vel_safe',
                                  'collision_monitor', 'cmd_vel_monitored',
                                  'velocity_smoother', 'cmd_vel']}
    with open(os.path.join(run_dir, 'safety_manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
    return actions


def generate_launch_description():
    """Launch the collision monitor and the safety filter for n_robots robots."""
    return LaunchDescription(
        [DeclareLaunchArgument('n_robots', default_value='3'),
         DeclareLaunchArgument('safety_enabled', default_value='true',
                               description='false = ablation A1 (whole safety layer bypassed)'),
         DeclareLaunchArgument('run_dir', default_value='',
                               description='log dir (default: bench/logs/latest)'),
         DeclareLaunchArgument('use_sim_time', default_value='true')]
        + [DeclareLaunchArgument(k, default_value=v) for k, v in DEFAULTS.items()]
        + [OpaqueFunction(function=_setup)])
