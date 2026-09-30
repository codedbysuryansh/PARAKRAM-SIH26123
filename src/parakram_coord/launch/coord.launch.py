"""
Decentralized coordination for N robots (CLAUDE_CODE/02).

    ros2 launch parakram_bringup fleet_sim.launch.py n_robots:=3 scenario:=intersection seed:=1
    ros2 launch parakram_coord coord.launch.py n_robots:=3

Starts one identical ``coordination_node`` per robot (in its namespace) and the optional
``/fleet/roster`` helper. Robots get the scenario's temporary fixed crossing assignments
(config/crossing_goals.yaml) until task allocation exists (CLAUDE_CODE/04). Logs go to the
fleet run's directory (``bench/logs/latest`` unless ``run_dir:=`` is given).

``loss`` / ``seed`` (CLAUDE_CODE/05 app-level loss on peer state and intent) default to the
running fleet's (``run_manifest.json``), so a fleet started with ``loss:=0.3`` is coordinated
under 30 % loss without repeating it here.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

NODE_PARAMS = ('tick_hz', 'window_W', 'reserve_k', 'lease_ttl', 'neighbor_timeout',
               'deadlock_timeout', 'priority_grow_rate', 'claim_settle', 'occupancy_radius',
               'blocked_after', 'stuck_timeout')


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    from parakram_bringup import run_manifest as rm

    cfg = {k: LaunchConfiguration(k).perform(context) for k in (
        'n_robots', 'scenario', 'assign_duration', 'run_dir', 'roster', 'reactive_only',
        'loss', 'seed', 'loss_model', 'loss_burst_corr', 'loss_scope', 'recovery_mode')
        + NODE_PARAMS}
    n_robots = int(cfg['n_robots'])
    run_dir = cfg['run_dir'] or os.path.realpath(os.path.join(rm.default_log_root(), 'latest'))
    os.makedirs(run_dir, exist_ok=True)
    fleet = {}
    try:
        with open(os.path.join(run_dir, 'run_manifest.json')) as f:
            fleet = json.load(f)
    except (OSError, ValueError):
        pass
    fleet_args = fleet.get('launch_args', {})
    loss = {'loss': float(cfg['loss'] or fleet.get('loss', 0.0)),
            'seed': int(cfg['seed'] or fleet.get('seed', 0)),
            'loss_model': cfg['loss_model'] or fleet_args.get('loss_model') or 'bernoulli',
            'loss_burst_corr': float(cfg['loss_burst_corr'] or
                                     fleet_args.get('loss_burst_corr') or 0.8),
            'loss_scope': cfg['loss_scope'] or fleet_args.get('loss_scope') or 'coordination'}
    recovery_mode = cfg['recovery_mode'] or fleet_args.get('recovery_mode') or 'lease'
    goals_file = os.path.join(get_package_share_directory('parakram_coord'), 'config',
                              'crossing_goals.yaml')
    with open(goals_file) as f:
        goals = yaml.safe_load(f).get(cfg['scenario'], {})
    grid_yaml = os.path.join(get_package_share_directory('parakram_sim'), 'config',
                             'warehouse_grid.yaml')

    actions = []
    node_params = {k: (int(cfg[k]) if k in ('window_W', 'reserve_k') else float(cfg[k]))
                   for k in NODE_PARAMS}
    per_robot = {}
    for i in range(1, n_robots + 1):
        ns = f'robot{i}'
        fixed = [int(v) for cell in goals.get(ns, []) for v in cell]
        per_robot[ns] = fixed
        params = dict(node_params, robot_id=ns, **loss, recovery_mode=recovery_mode,
                      reactive_only=_truthy(cfg['reactive_only']),
                      fixed_assign_duration=float(cfg['assign_duration']), log_dir=run_dir,
                      grid_yaml=grid_yaml, use_sim_time=True)
        if fixed:        # none for task-driven scenarios (CLAUDE_CODE/04: goals come from tasks)
            params['fixed_goals'] = fixed
        actions.append(Node(package='parakram_coord', executable='coordination_node',
                            name='coordination', namespace=ns, output='screen',
                            parameters=[params],
                            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    if _truthy(cfg['roster']):
        actions.append(Node(package='parakram_coord', executable='roster_helper',
                            name='roster_helper', output='screen'))
    manifest = {'kind': 'coord', 'launch_args': cfg, 'node_params': node_params,
                'fixed_goals': per_robot, 'goals_file': goals_file, 'app_level_loss': loss,
                'recovery_mode': recovery_mode}
    with open(os.path.join(run_dir, 'coord_manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
    actions.insert(0, LogInfo(msg=f'[parakram_coord] logging to {run_dir}'))
    return actions


def generate_launch_description():
    """Launch coordination for n_robots robots."""
    defaults = {'tick_hz': '10.0', 'window_W': '8', 'reserve_k': '3', 'lease_ttl': '2.0',
                'neighbor_timeout': '1.0', 'deadlock_timeout': '30.0',
                'priority_grow_rate': '1.0', 'claim_settle': '0.4', 'occupancy_radius': '0.14',
                'blocked_after': '3.0', 'stuck_timeout': '20.0'}
    return LaunchDescription(
        [DeclareLaunchArgument('n_robots', default_value='3'),
         DeclareLaunchArgument('scenario', default_value='intersection'),
         DeclareLaunchArgument('assign_duration', default_value='300.0',
                               description='[s sim] how long fixed crossing goals keep coming'),
         DeclareLaunchArgument('run_dir', default_value='',
                               description='log dir (default: bench/logs/latest)'),
         DeclareLaunchArgument('roster', default_value='true',
                               description='start the optional /fleet/roster helper'),
         DeclareLaunchArgument('reactive_only', default_value='false',
                               description='disable conflict resolution (CLAUDE_CODE/03 '
                                           'acceptance: only the safety layer separates robots)'),
         DeclareLaunchArgument('loss', default_value='',
                               description="peer state/intent app-level loss (default: fleet's)"),
         DeclareLaunchArgument('seed', default_value='', description="default: the fleet's"),
         DeclareLaunchArgument('loss_model', default_value='',
                               description="bernoulli | gilbert_elliott (default: fleet's)"),
         DeclareLaunchArgument('loss_burst_corr', default_value='',
                               description='Gilbert-Elliott burst correlation (default 0.8)'),
         DeclareLaunchArgument('loss_scope', default_value='',
                               description="coordination | fleet (default: the fleet's)"),
         DeclareLaunchArgument('recovery_mode', default_value='',
                               description="lease | release (default: the fleet's)")]
        + [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=_setup)])
