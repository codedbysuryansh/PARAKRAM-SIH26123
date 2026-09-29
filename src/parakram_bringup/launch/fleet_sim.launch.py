"""
PARAKRAM fleet simulation: warehouse world + N namespaced TurtleBot3 Burgers + N Nav2 stacks.

    ros2 launch parakram_bringup fleet_sim.launch.py n_robots:=3 scenario:=intersection seed:=1

Writes ``bench/logs/<run_id>/run_manifest.json`` (run_id, seed, n_robots, scenario, git SHA,
params hash) and archives every generated per-robot file under ``bench/logs/<run_id>/generated``.
``bench/logs/latest`` points at the newest run.

Every robot gets the reactive safety layer (CLAUDE_CODE/03): with ``safety:=true`` (default)
``parakram_safety/launch/safety.launch.py`` starts each robot's collision monitor and NH-ORCA
filter, which close the cmd_vel chain in front of the velocity smoother. ``safety:=false`` leaves
them out (start that launch file separately: the work order's two-step acceptance flow); until it
runs no command reaches a base. ``safety_enabled:=false`` is the A1 ablation (the whole safety
layer bypassed).

Launch args (CLAUDE_CODE/00): ``seed``, ``loss``, ``n_robots``, ``scenario``. Packet-loss
injection does not exist until CLAUDE_CODE/05, so any ``loss`` other than 0 is REFUSED rather
than silently ignored (a run must never be labelled with a loss level it did not have).
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    from parakram_bringup import run_manifest as rm
    from parakram_bringup.nav2_params import write_robot_params
    from parakram_sim.grid_utils import WarehouseGrid
    from parakram_sim.robot_model import robot_sdf_template_path, spawn_poses

    cfg = {k: LaunchConfiguration(k).perform(context) for k in (
        'n_robots', 'scenario', 'seed', 'loss', 'gui', 'render_engine', 'software_gl',
        'lidar_noise_std', 'lidar_rate', 'log_level', 'run_id', 'nav2_start_delay',
        'nav2_stagger', 'use_composition', 'clock_period', 'safety', 'safety_enabled')}
    n_robots = int(cfg['n_robots'])
    seed = int(cfg['seed'])
    loss = float(cfg['loss'])
    if loss != 0.0:
        raise RuntimeError(
            f'loss:={loss} requested, but packet-loss injection is implemented in '
            'CLAUDE_CODE/05 (not yet built). Refusing to start a run that would be mislabelled.')

    sim_share = get_package_share_directory('parakram_sim')
    bringup_share = get_package_share_directory('parakram_bringup')
    grid_file = os.path.join(sim_share, 'config', 'warehouse_grid.yaml')
    world_file = os.path.join(sim_share, 'worlds', 'warehouse.sdf')
    map_file = os.path.join(bringup_share, 'maps', 'warehouse.yaml')
    nav2_template = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    safety_share = get_package_share_directory('parakram_safety')
    monitor_file = os.path.join(safety_share, 'config', 'collision_monitor.yaml')

    run_id = cfg['run_id'] or rm.new_run_id()
    log_root = rm.default_log_root()
    run_dir = os.path.join(log_root, run_id)
    gen_dir = os.path.join(run_dir, 'generated')
    os.makedirs(gen_dir, exist_ok=True)

    grid = WarehouseGrid.from_yaml(grid_file)
    poses = spawn_poses(grid, cfg['scenario'], n_robots)  # validates scenario / n_robots
    robot_params = {ns: write_robot_params(nav2_template, gen_dir, ns, (x, y, yaw))
                    for ns, x, y, yaw, _ in poses}

    settings = {k: cfg[k] for k in ('n_robots', 'scenario', 'render_engine', 'software_gl',
                                    'lidar_noise_std', 'lidar_rate', 'use_composition',
                                    'clock_period', 'safety', 'safety_enabled')}
    hashed_files = {
        'parakram_sim/config/warehouse_grid.yaml': grid_file,
        'parakram_sim/worlds/warehouse.sdf': world_file,
        'parakram_sim/models/parakram_burger/model.sdf.in': robot_sdf_template_path(),
        'parakram_bringup/config/nav2_params.yaml': nav2_template,
        'parakram_safety/config/collision_monitor.yaml': monitor_file,
        'parakram_bringup/maps/warehouse.yaml': map_file,
        'parakram_bringup/maps/warehouse.pgm': os.path.join(bringup_share, 'maps',
                                                            'warehouse.pgm'),
    }
    hashed_files.update({f'generated/{os.path.basename(p)}': p for p in robot_params.values()})
    p_hash, per_file = rm.params_hash(hashed_files, settings)

    ws_root = rm.find_workspace_root(get_package_share_directory('parakram_bringup'))
    manifest = rm.base_manifest(run_id, seed, ws_root)
    manifest.update({
        'kind': 'fleet_sim',
        'n_robots': n_robots,
        'scenario': cfg['scenario'],
        'loss': loss,
        'params_hash': p_hash,
        'params_files_sha256': per_file,
        'settings': settings,
        'launch_args': cfg,
        'robots': [{'namespace': ns, 'start_cell': list(cell), 'x': x, 'y': y, 'yaw': yaw}
                   for ns, x, y, yaw, cell in poses],
        'run_dir': run_dir,
    })
    manifest_path = rm.write_manifest(run_dir, manifest)
    rm.update_latest_symlink(log_root, run_id)

    use_sim_time = 'true'
    actions = [
        LogInfo(msg=f'[parakram] run_id={run_id} seed={seed} n_robots={n_robots} '
                    f"scenario={cfg['scenario']} params_hash={p_hash[:12]}"),
        LogInfo(msg=f'[parakram] manifest: {manifest_path}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch',
                                                       'warehouse_world.launch.py')),
            launch_arguments={'world': world_file, 'seed': str(seed),
                              'render_engine': cfg['render_engine'],
                              'software_gl': cfg['software_gl'], 'gui': cfg['gui'],
                              'clock_period': cfg['clock_period']}.items()),
        # Shared map (one /map for the fleet in sim; on hardware each robot serves the same file).
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': map_file,
                                           'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server'], 'bond_timeout': 10.0}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch',
                                                       'spawn_robots.launch.py')),
            launch_arguments={'n_robots': str(n_robots), 'scenario': cfg['scenario'],
                              'grid': grid_file, 'generated_dir': gen_dir,
                              'lidar_noise_std': cfg['lidar_noise_std'],
                              'lidar_rate': cfg['lidar_rate'],
                              'use_sim_time': use_sim_time}.items()),
    ]
    nav2_launch = os.path.join(bringup_share, 'launch', 'nav2_robot.launch.py')
    # Let the robots spawn and their sensors come up before Nav2 starts pulling on TF, and
    # stagger the per-robot stacks so N stacks do not all hit DDS discovery at once.
    for i, (ns, *_) in enumerate(poses):
        actions.append(TimerAction(
            period=float(cfg['nav2_start_delay']) + i * float(cfg['nav2_stagger']),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={'namespace': ns, 'params_file': robot_params[ns],
                                  'use_sim_time': use_sim_time,
                                  'use_composition': cfg['use_composition'],
                                  'log_level': cfg['log_level']}.items())]))
    if _truthy(cfg['safety']):
        # the per-robot collision monitors and safety filters (they wait for their robot's topics)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(safety_share, 'launch',
                                                       'safety.launch.py')),
            launch_arguments={'n_robots': str(n_robots), 'run_dir': run_dir,
                              'safety_enabled': cfg['safety_enabled'],
                              'use_sim_time': use_sim_time}.items()))
    with open(os.path.join(gen_dir, 'launch_config.json'), 'w') as f:
        json.dump(cfg, f, indent=2, sort_keys=True)
    return actions


def generate_launch_description():
    """Launch the PARAKRAM fleet simulation."""
    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='3'),
        DeclareLaunchArgument('scenario', default_value='intersection',
                              description='scenario in parakram_sim/config/warehouse_grid.yaml'),
        DeclareLaunchArgument('seed', default_value='1', description='integer run seed'),
        DeclareLaunchArgument('loss', default_value='0.0',
                              description='packet loss (must be 0 until CLAUDE_CODE/05)'),
        DeclareLaunchArgument('gui', default_value='false', description='start the gz GUI'),
        DeclareLaunchArgument('render_engine', default_value='ogre',
                              description='gz server render engine (ogre on the VM; ogre2 on '
                                          'a real GPU)'),
        DeclareLaunchArgument('software_gl', default_value='true',
                              description='Mesa llvmpipe for the gz server (VM lidar fix)'),
        DeclareLaunchArgument('lidar_noise_std', default_value='0.02',
                              description='lidar gaussian range noise sigma [m] (LDS-01 spec: '
                                          '+/-15 mm short range, +/-5% long range)'),
        DeclareLaunchArgument('lidar_rate', default_value='5.0', description='lidar rate [Hz]'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('run_id', default_value='',
                              description='override the auto-generated uuid run id'),
        DeclareLaunchArgument('nav2_start_delay', default_value='5.0',
                              description='[s] wall time between spawning and Nav2 bringup'),
        DeclareLaunchArgument('nav2_stagger', default_value='2.0',
                              description='[s] extra delay per robot index for its Nav2 stack'),
        DeclareLaunchArgument('use_composition', default_value='true',
                              description='one Nav2 component container per robot'),
        DeclareLaunchArgument('clock_period', default_value='0.005',
                              description='[s sim] ROS /clock period (physics stays at 1 ms)'),
        DeclareLaunchArgument('safety', default_value='true',
                              description='start the per-robot safety layer (CLAUDE_CODE/03); '
                                          'false: start parakram_safety safety.launch.py '
                                          'yourself (no command reaches a base until then)'),
        DeclareLaunchArgument('safety_enabled', default_value='true',
                              description='false = ablation A1: whole safety layer bypassed'),
        OpaqueFunction(function=_setup),
    ])
