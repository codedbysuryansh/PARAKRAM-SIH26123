"""Sanity of the Part-A Collision Monitor configuration and the 'headon' acceptance scenario."""

import importlib.util
import json
import math
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from parakram_sim.footprint import BURGER_HALF_WIDTH, BURGER_X_MAX, BURGER_X_MIN
from parakram_sim.grid_utils import WarehouseGrid
from parakram_sim.robot_model import scenario, spawn_poses
import pytest
import yaml

HERE = os.path.dirname(__file__)
MONITOR = os.path.join(HERE, '..', 'config', 'collision_monitor.yaml')
GRID = os.path.join(HERE, '..', '..', 'parakram_sim', 'config', 'warehouse_grid.yaml')
NAV2 = os.path.join(HERE, '..', '..', 'parakram_bringup', 'config', 'nav2_params.yaml')
SAFETY_LAUNCH = os.path.join(HERE, '..', 'launch', 'safety.launch.py')
LIDAR_X, LIDAR_MIN_RANGE = -0.032, 0.12


@pytest.fixture(scope='module')
def cm():
    with open(MONITOR) as f:
        return yaml.safe_load(f)['collision_monitor']['ros__parameters']


def _points(s):
    return [tuple(p) for p in yaml.safe_load(s)]


def _polygons(cm):
    for name in cm['polygons']:
        z = cm[name]
        if z['type'] == 'velocity_polygon':
            for sub in z['velocity_polygons']:
                yield name, sub, _points(z[sub]['points'])
        else:
            yield name, None, _points(z['points'])


def test_chain_position_lidar_only_and_namespace_agnostic(cm):
    # the work order's chain: controller -> [filter] -> collision_monitor -> velocity_smoother
    topics = (cm['cmd_vel_in_topic'], cm['cmd_vel_out_topic'])
    assert topics == ('cmd_vel_safe', 'cmd_vel_monitored')
    # brings itself up in its own process (no cross-process lifecycle calls, no bond)
    assert cm['autostart_node'] is True and cm['bond_heartbeat_period'] == 0.0
    assert cm['observation_sources'] == ['scan']            # onboard lidar only, no network
    assert cm['scan']['type'] == 'scan' and cm['scan']['topic'] == 'scan'
    for key, value in cm.items():
        if isinstance(value, str) and key.endswith('topic'):
            assert not value.startswith('/'), key            # resolves in the robot namespace
    assert cm['source_timeout'] > 0.0                        # lidar dropout -> stop (fail-safe)


def test_zones_stay_clear_of_the_aisle_shelves(cm):
    # 0.40 m aisles: a centred robot has shelves 0.20 m from its axis; keep 0.10 m of slack
    for name, sub, pts in _polygons(cm):
        assert all(abs(y) <= 0.10 + 1e-9 for _, y in pts), (name, sub)


def test_turning_in_place_is_never_stopped(cm):
    # the fallback (last) sub-polygon of every velocity polygon is used when not translating;
    # it lies inside the lidar's blind radius, so it can never contain a scan point
    for name in ('StopZone', 'SlowdownZone'):
        z = cm[name]
        last = z['velocity_polygons'][-1]
        sub = z[last]
        assert sub['linear_min'] <= -1.0 and sub['linear_max'] >= 1.0
        for x, y in _points(sub['points']):
            assert math.hypot(x - LIDAR_X, y) < LIDAR_MIN_RANGE
        for other in z['velocity_polygons'][:-1]:            # the real zones need translation
            s = z[other]
            assert s['linear_min'] >= 0.01 or s['linear_max'] <= -0.01, (name, other)


def test_approach_zone_is_the_exact_footprint(cm):
    pts = _points(cm['FootprintApproach']['points'])
    assert cm['FootprintApproach']['action_type'] == 'approach'
    xs, ys = {x for x, _ in pts}, {y for _, y in pts}
    assert xs == {BURGER_X_MIN, BURGER_X_MAX} and ys == {-BURGER_HALF_WIDTH, BURGER_HALF_WIDTH}


def test_stop_zone_reaches_beyond_a_peers_body(cm):
    forward = _points(cm['StopZone']['forward']['points'])
    reach = max(x for x, _ in forward) - BURGER_X_MAX
    # the velocity smoother after the monitor stretches a stop by v^2 / (2 max_decel)
    with open(NAV2) as f:
        vs = yaml.safe_load(f)['velocity_smoother']['ros__parameters']
    stretch = vs['max_velocity'][0] ** 2 / (2 * -vs['max_decel'][0])
    # a peer's body reaches <= 0.07 m beyond its sensed turret; plus one 5 Hz scan of closing
    assert reach >= 0.07 + 2 * 0.18 * 0.2 + stretch


def test_headon_scenario_is_a_real_conflict():
    grid = WarehouseGrid.from_yaml(GRID)
    poses = spawn_poses(grid, 'headon', 4)
    sc = scenario(grid, 'headon')
    junction = (5, 6)
    for (ns, _, _, _, start), goals in zip(poses, sc['test_goals']):
        goal = tuple(goals[-1][:2])
        # start and goal lie on opposite sides of the junction on the same row / column
        if start[0] == junction[0]:
            assert goal[0] == junction[0] and (start[1] - 6) * (goal[1] - 6) < 0, ns
        else:
            assert start[1] == goal[1] == junction[1] and (start[0] - 5) * (goal[0] - 5) < 0, ns


def test_safety_launch_brings_up_monitor_and_filter_per_robot(tmp_path):
    """Check that safety.launch.py starts and wires a monitor and a filter per robot."""
    spec = importlib.util.spec_from_file_location('safety_launch', SAFETY_LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    context = LaunchContext()
    for entity in mod.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = entity.default_value[0].text
    context.launch_configurations.update({'n_robots': '3', 'run_dir': str(tmp_path)})
    nodes = [a for a in mod._setup(context) if type(a).__name__ == 'Node']
    assert sorted((n.node_package, n.node_executable) for n in nodes) == sorted(
        [('nav2_collision_monitor', 'collision_monitor'), ('parakram_safety', 'orca_filter')] * 3)
    for i in (1, 2, 3):
        with open(tmp_path / 'generated' / f'collision_monitor_robot{i}.yaml') as f:
            params = yaml.safe_load(f)[f'robot{i}']['collision_monitor']['ros__parameters']
        assert (params['cmd_vel_in_topic'], params['cmd_vel_out_topic']) == (
            'cmd_vel_safe', 'cmd_vel_monitored')
    with open(tmp_path / 'safety_manifest.json') as f:
        manifest = json.load(f)
    assert manifest['filter'] == 'NH-ORCA'
    assert manifest['cmd_vel_chain'] == ['cmd_vel_nav', 'orca_filter', 'cmd_vel_safe',
                                         'collision_monitor', 'cmd_vel_monitored',
                                         'velocity_smoother', 'cmd_vel']
