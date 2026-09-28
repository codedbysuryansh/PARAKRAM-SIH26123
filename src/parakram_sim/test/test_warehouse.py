"""Consistency tests: grid YAML <-> generated world SDF <-> occupancy map, topology, models."""

import os
import xml.etree.ElementTree as ET

from parakram_sim import footprint, robot_model, warehouse
from parakram_sim.grid_utils import WarehouseGrid
import pytest

HERE = os.path.dirname(__file__)
PKG = os.path.join(HERE, '..')
GRID_YAML = os.path.join(PKG, 'config', 'warehouse_grid.yaml')
WORLD_SDF = os.path.join(PKG, 'worlds', 'warehouse.sdf')
MAP_DIR = os.path.join(PKG, '..', 'parakram_bringup', 'maps')
ROBOT_TEMPLATE = os.path.join(PKG, 'models', 'parakram_burger', 'model.sdf.in')


@pytest.fixture(scope='module')
def grid():
    return WarehouseGrid.from_yaml(GRID_YAML)


def test_grid_validates(grid):
    warehouse.validate(grid)


def test_generated_world_is_up_to_date(grid):
    with open(WORLD_SDF) as f:
        assert f.read() == warehouse.world_sdf(grid), \
            'worlds/warehouse.sdf is stale: run `ros2 run parakram_sim generate_warehouse`'


@pytest.mark.skipif(not os.path.isdir(MAP_DIR), reason='parakram_bringup source not present')
def test_generated_map_is_up_to_date(grid):
    with open(os.path.join(MAP_DIR, 'warehouse.pgm'), 'rb') as f:
        assert f.read() == warehouse.map_pgm(grid), 'maps/warehouse.pgm is stale'
    with open(os.path.join(MAP_DIR, 'warehouse.yaml')) as f:
        assert f.read() == warehouse.map_yaml(grid), 'maps/warehouse.yaml is stale'


def test_world_contains_every_box(grid):
    root = ET.parse(WORLD_SDF).getroot()
    names = {m.get('name') for m in root.iter('model')}
    for b in warehouse.static_boxes(grid):
        assert b.name in names
    assert len([b for b in warehouse.static_boxes(grid) if b.kind == 'shelf']) == 8


def test_map_agrees_with_grid(grid):
    """Free cell centres are free in the map; blocked cell centres are occupied."""
    x_min, y_min, width, height, res = warehouse.map_extent(grid)
    rows = warehouse.occupancy_rows(grid)

    def occupied(x, y):
        j = int((x - x_min) / res)
        i = height - 1 - int((y - y_min) / res)
        return rows[i][j] == 0

    for r in range(grid.rows):
        for c in range(grid.cols):
            assert occupied(*grid.cell_to_world(r, c)) == grid.is_blocked(r, c), (r, c)


def test_shelves_sit_inside_their_cells(grid):
    for b in warehouse.static_boxes(grid):
        if b.kind != 'shelf':
            continue
        for x, y in ((b.x_min, b.y_min), (b.x_max, b.y_max)):
            assert grid.is_blocked(*grid.world_to_cell(x, y))


def test_robot_at_any_free_cell_centre_clears_static_obstacles(grid):
    """A Burger centred in any free cell, at any heading, does not touch a shelf or wall."""
    polys = [footprint.box_polygon(b.x_min, b.y_min, b.x_max, b.y_max)
             for b in warehouse.static_boxes(grid)]
    for r, c in grid.free_cells():
        x, y = grid.cell_to_world(r, c)
        for yaw in (0.0, 0.785, 1.571, 2.356, 3.1416):
            fp = footprint.footprint_polygon(x, y, yaw)
            assert min(footprint.polygon_distance(fp, p) for p in polys) > 0.05, (r, c, yaw)


def _articulation_points(nodes, adj):
    index, low, parent, aps, counter = {}, {}, {}, set(), [0]

    def dfs(u):
        index[u] = low[u] = counter[0]
        counter[0] += 1
        children = 0
        for v in adj(u):
            if v not in index:
                parent[v] = u
                children += 1
                dfs(v)
                low[u] = min(low[u], low[v])
                if u not in parent and children > 1:
                    aps.add(u)
                if u in parent and low[v] >= index[u]:
                    aps.add(u)
            elif v != parent.get(u):
                low[u] = min(low[u], index[v])
    start = nodes[0]
    dfs(start)
    return aps, len(index)


def test_free_graph_is_biconnected(grid):
    """PIBT reachability precondition (MASTER_BUILD_PLAN §4) and no single-cell partition."""
    nodes = grid.free_cells()
    aps, reached = _articulation_points(nodes, lambda cell: grid.neighbors(*cell))
    assert reached == len(nodes), 'free cells are not all connected'
    assert not aps, f'articulation points (a dead robot there would partition): {sorted(aps)}'


def test_scenarios(grid):
    sc = robot_model.scenario(grid, 'intersection')
    poses = robot_model.spawn_poses(grid, 'intersection', 4)
    assert [p[0] for p in poses] == ['robot1', 'robot2', 'robot3', 'robot4']
    assert len({p[4] for p in poses}) == 4
    with pytest.raises(ValueError):
        robot_model.spawn_poses(grid, 'intersection', len(sc['start_cells']) + 1)
    with pytest.raises(KeyError):
        robot_model.scenario(grid, 'nope')


def test_intersection_test_routes_are_cell_disjoint(grid):
    """File-01 smoke-test routes (L-shaped, via the listed waypoints) never share a cell."""
    sc = robot_model.scenario(grid, 'intersection')

    def cells_between(a, b):
        (r0, c0), (r1, c1) = a, b
        assert r0 == r1 or c0 == c1, 'waypoints must be axis-aligned'
        if r0 == r1:
            return {(r0, c) for c in range(min(c0, c1), max(c0, c1) + 1)}
        return {(r, c0) for r in range(min(r0, r1), max(r0, r1) + 1)}

    routes = []
    for start, goals in zip(sc['start_cells'], sc['test_goals']):
        pts = [tuple(start[:2])] + [tuple(g[:2]) for g in goals]
        cells = set()
        for a, b in zip(pts, pts[1:]):
            cells |= cells_between(a, b)
        assert all(grid.is_free(*cl) for cl in cells)
        routes.append(cells)
    for i in range(len(routes)):
        for j in range(i + 1, len(routes)):
            assert not routes[i] & routes[j], (i, j, routes[i] & routes[j])


def test_robot_sdf_renders_namespaced():
    sdf = robot_model.render_robot_sdf('robot2', 0.02, 5.0, template_path=ROBOT_TEMPLATE)
    root = ET.fromstring(sdf)
    model = root.find('model')
    assert model.get('name') == 'robot2'
    topics = [t.text for t in root.iter('topic')] + [t.text for t in root.iter('odom_topic')] + \
        [t.text for t in root.iter('tf_topic')]
    assert topics and all(t.startswith('/robot2/') for t in topics), topics
    assert '@' not in sdf.split('-->', 1)[1]
    assert root.find('.//lidar/noise/stddev').text == '0.02'


def test_bridge_entries():
    entries = robot_model.bridge_entries('robot3')
    names = {e['ros_topic_name'] for e in entries}
    assert names == {f'/robot3/{t}' for t in
                     ('odom', 'tf', 'joint_states', 'imu', 'scan', 'cmd_vel')}
    cmd = [e for e in entries if e['ros_topic_name'].endswith('cmd_vel')][0]
    assert cmd['ros_type_name'] == 'geometry_msgs/msg/Twist' and cmd['direction'] == 'ROS_TO_GZ'


def test_footprint_distance():
    a = footprint.footprint_polygon(0.0, 0.0, 0.0)
    b = footprint.footprint_polygon(1.0, 0.0, 0.0)
    # Front of a at x=0.04, rear of b at x=1-0.105.
    assert footprint.polygon_distance(a, b) == pytest.approx(1.0 - 0.105 - 0.04)
    assert footprint.polygon_distance(a, footprint.footprint_polygon(0.05, 0.0, 1.0)) == 0.0
