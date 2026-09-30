"""
Fault layer for N robots (CLAUDE_CODE/06).

    ros2 launch parakram_fault fault.launch.py n_robots:=3

Per robot (in its namespace): ``heartbeat`` (relays its lease renewals), ``watchdog`` (peer
status) and ``recovery_coordinator`` (ReAuction on DEAD, task reallocation only); once:
``recovery_logger`` (``<run_dir>/recovery_events.csv``). Loss settings and the recovery mode
default to the running fleet's (``run_manifest.json``); parameters in ``config/fault.yaml``.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _setup(context):
    from parakram_bringup import run_manifest as rm

    cfg = {k: LaunchConfiguration(k).perform(context) for k in (
        'n_robots', 'run_dir', 'recovery_mode', 'use_sim_time')}
    n = int(cfg['n_robots'])
    run_dir = cfg['run_dir'] or os.path.realpath(os.path.join(rm.default_log_root(), 'latest'))
    fleet = {}
    try:
        with open(os.path.join(run_dir, 'run_manifest.json')) as f:
            fleet = json.load(f)
    except (OSError, ValueError):
        pass
    args = fleet.get('launch_args', {})
    with open(os.path.join(get_package_share_directory('parakram_fault'), 'config',
                           'fault.yaml')) as f:
        params = yaml.safe_load(f)
    mode = cfg['recovery_mode'] or args.get('recovery_mode') or 'lease'
    loss = {'loss': float(fleet.get('loss', 0.0)), 'seed': int(fleet.get('seed', 0)),
            'loss_model': args.get('loss_model') or 'bernoulli',
            'loss_burst_corr': float(args.get('loss_burst_corr') or 0.8),
            'loss_scope': args.get('loss_scope') or 'none'}
    robots = [f'robot{i}' for i in range(1, n + 1)]
    sim = cfg['use_sim_time'].lower() in ('1', 'true', 'yes')
    actions = []
    for ns in robots:
        common = {'robot_id': ns, 'use_sim_time': sim}
        actions += [
            Node(package='parakram_fault', executable='heartbeat_node', name='heartbeat',
                 namespace=ns, parameters=[common]),
            Node(package='parakram_fault', executable='watchdog_node', name='watchdog',
                 namespace=ns, output='screen',
                 parameters=[dict(common, peers=robots, recovery_mode=mode, log_dir=run_dir,
                                  **loss, **{k: float(v) for k, v in params.items()
                                             if k not in ('lease_time', 'clock_drift_rho')})]),
            Node(package='parakram_fault', executable='recovery_coordinator',
                 name='recovery_coordinator', namespace=ns, output='screen',
                 parameters=[dict(common, loss=loss['loss'])])]
    actions.append(Node(package='parakram_fault', executable='recovery_logger',
                        name='recovery_logger', parameters=[{'log_dir': run_dir,
                                                             'use_sim_time': sim}]))
    with open(os.path.join(run_dir, 'fault_manifest.json'), 'w') as f:
        json.dump({'kind': 'fault', 'recovery_mode': mode, 'params': params, 'loss': loss,
                   'robots': robots}, f, indent=2, sort_keys=True)
    return actions


def generate_launch_description():
    """Launch the fault layer."""
    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='3'),
        DeclareLaunchArgument('run_dir', default_value='',
                              description='log dir (default: bench/logs/latest)'),
        DeclareLaunchArgument('recovery_mode', default_value='',
                              description="lease | release (default: the fleet's)"),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup)])
