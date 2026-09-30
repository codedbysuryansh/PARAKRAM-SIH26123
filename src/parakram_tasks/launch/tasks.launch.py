"""
Decentralized task allocation for N robots (CLAUDE_CODE/04).

    ros2 launch parakram_bringup fleet_sim.launch.py n_robots:=3 scenario:=warehouse_stream seed:=1
    ros2 launch parakram_coord coord.launch.py n_robots:=3 scenario:=warehouse_stream
    ros2 launch parakram_tasks tasks.launch.py n_robots:=3 task_rate:=0.2

Starts one identical ``auction_node`` per robot (in its namespace; there is no auctioneer node)
and, in simulation, the ``task_generator`` that streams ``n_tasks`` tasks. The seed and the
scenario default to the running fleet's (``bench/logs/latest/run_manifest.json``); events go to
``<run_dir>/tasks.csv`` and the launch settings to ``<run_dir>/tasks_manifest.json``.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

AUCTION_PARAMS = {'bid_window': '1.0', 'award_timeout': '2.0', 'award_lease_ttl': '10.0',
                  'renew_period': '2.0', 'announce_jitter_min': '0.2',
                  'announce_jitter_max': '1.0', 'backoff_max': '8.0', 'service_time': '1.0',
                  'stall_timeout': '60.0', 'plan_speed': '0.15', 'battery_min': '0.2',
                  'heartbeat_timeout': '3.0', 'award_margin': '2.0', 'digest_period': '1.0',
                  'award_settle': '1.0'}


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    from parakram_bringup import run_manifest as rm

    cfg = {k: LaunchConfiguration(k).perform(context) for k in (
        'n_robots', 'task_rate', 'n_tasks', 'seed', 'scenario', 'run_dir', 'generator',
        'use_sim_time') + tuple(AUCTION_PARAMS)}
    n_robots = int(cfg['n_robots'])
    run_dir = cfg['run_dir'] or os.path.realpath(os.path.join(rm.default_log_root(), 'latest'))
    os.makedirs(run_dir, exist_ok=True)
    fleet = {}
    try:
        with open(os.path.join(run_dir, 'run_manifest.json')) as f:
            fleet = json.load(f)
    except (OSError, ValueError):
        pass
    seed = int(cfg['seed'] or fleet.get('seed', 1))
    fleet_args = fleet.get('launch_args', {})
    # CLAUDE_CODE/06: loss on the task topics only with the fleet-wide scope (05 never touched
    # them); the recovery mode follows the fleet
    scope = fleet_args.get('loss_scope') or 'coordination'
    mode = fleet_args.get('recovery_mode') or 'lease'
    w06 = {'recovery_mode': mode, 'award_gate': mode == 'lease',
           'loss_scope': 'fleet' if scope == 'fleet' else 'none',
           'loss': float(fleet.get('loss', 0.0)) if scope == 'fleet' else 0.0,
           'loss_model': fleet_args.get('loss_model') or 'bernoulli',
           'loss_burst_corr': float(fleet_args.get('loss_burst_corr') or 0.8)}
    scenario = cfg['scenario'] or fleet.get('scenario') or 'warehouse_stream'
    grid_yaml = os.path.join(get_package_share_directory('parakram_sim'), 'config',
                             'warehouse_grid.yaml')
    use_sim_time = _truthy(cfg['use_sim_time'])
    robots = [f'robot{i}' for i in range(1, n_robots + 1)]
    params = {k: float(cfg[k]) for k in AUCTION_PARAMS}
    actions = [LogInfo(msg=f'[parakram_tasks] {n_robots} auction nodes (no auctioneer), '
                           f"scenario '{scenario}', seed {seed}, logging to {run_dir}")]
    for ns in robots:
        actions.append(Node(package='parakram_tasks', executable='auction_node',
                            name='auction_node', namespace=ns, output='screen',
                            parameters=[dict(params, robot_id=ns, peers=robots, seed=seed,
                                             log_dir=run_dir, grid_yaml=grid_yaml,
                                             use_sim_time=use_sim_time, **w06)]))
    if _truthy(cfg['generator']):
        actions.append(Node(package='parakram_tasks', executable='task_generator',
                            name='task_generator', output='screen',
                            parameters=[{'n_tasks': int(cfg['n_tasks']),
                                         'task_rate': float(cfg['task_rate']), 'seed': seed,
                                         'scenario': scenario, 'grid_yaml': grid_yaml,
                                         'use_sim_time': use_sim_time}]))
    manifest = {'kind': 'tasks', 'launch_args': cfg, 'seed': seed, 'scenario': scenario,
                'auction_params': params, 'robots': robots, 'w06': w06}
    with open(os.path.join(run_dir, 'tasks_manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
    return actions


def generate_launch_description():
    """Launch the auction participants (and the simulated task stream)."""
    return LaunchDescription(
        [DeclareLaunchArgument('n_robots', default_value='3'),
         DeclareLaunchArgument('task_rate', default_value='0.2', description='[tasks/s sim]'),
         DeclareLaunchArgument('n_tasks', default_value='20'),
         DeclareLaunchArgument('seed', default_value='', description="default: the fleet's"),
         DeclareLaunchArgument('scenario', default_value='',
                               description="default: the fleet's (task stations)"),
         DeclareLaunchArgument('run_dir', default_value='',
                               description='log dir (default: bench/logs/latest)'),
         DeclareLaunchArgument('generator', default_value='true',
                               description='start the simulated task stream'),
         DeclareLaunchArgument('use_sim_time', default_value='true')]
        + [DeclareLaunchArgument(k, default_value=v) for k, v in AUCTION_PARAMS.items()]
        + [OpaqueFunction(function=_setup)])
