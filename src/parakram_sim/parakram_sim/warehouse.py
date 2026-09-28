"""
Warehouse geometry, world SDF and occupancy-map generation from ``warehouse_grid.yaml``.

``warehouse_grid.yaml`` is the single source of truth: the Gazebo world
(``parakram_sim/worlds/warehouse.sdf``) and the Nav2 map (``parakram_bringup/maps/``) are both
generated from it, so the grid, the physics world and the map cannot drift apart. Regenerate with::

    ros2 run parakram_sim generate_warehouse

The same box geometry is used by tests (grid/world/map consistency) and by the smoke test's
ground-truth clearance check.
"""

import argparse
import math
import os

from parakram_sim.grid_utils import WarehouseGrid

# Bump when the generated output format changes (recorded in the generated files).
GENERATOR_VERSION = 1


class Box:
    """An axis-aligned static box obstacle (walls, shelves) in world coordinates."""

    def __init__(self, name, kind, x_min, y_min, x_max, y_max, height):
        """Create a box from its world-frame extents."""
        self.name = name
        self.kind = kind
        self.x_min, self.y_min, self.x_max, self.y_max = x_min, y_min, x_max, y_max
        self.height = height

    @property
    def center(self):
        """Box centre ``(x, y)``."""
        return (0.5 * (self.x_min + self.x_max), 0.5 * (self.y_min + self.y_max))

    @property
    def size(self):
        """Box footprint ``(size_x, size_y)``."""
        return (self.x_max - self.x_min, self.y_max - self.y_min)

    def contains(self, x, y):
        """Return True if ``(x, y)`` lies inside the box footprint (closed interval)."""
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def distance(self, x, y):
        """Euclidean distance from ``(x, y)`` to the box footprint (0 inside)."""
        dx = max(self.x_min - x, 0.0, x - self.x_max)
        dy = max(self.y_min - y, 0.0, y - self.y_max)
        return math.hypot(dx, dy)


def _r(v):
    """Round to 1e-6 m to keep generated files free of float noise."""
    return round(v + 0.0, 6)


def shelf_blocks(raw):
    """Return the shelf blocks ``[r0, c0, r1, c1]`` from a parsed grid YAML."""
    return [tuple(int(v) for v in b) for b in raw.get('shelf_blocks', [])]


def cells_of_blocks(blocks):
    """Expand inclusive block rectangles into the set of covered cells."""
    cells = set()
    for r0, c0, r1, c1 in blocks:
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                cells.add((r, c))
    return cells


def validate(grid):
    """Validate the grid definition; raise ValueError with all problems found."""
    raw = grid.raw
    problems = []
    blocks = shelf_blocks(raw)
    for b in blocks:
        r0, c0, r1, c1 = b
        if r0 > r1 or c0 > c1:
            problems.append(f'shelf block {b} is not [r_min, c_min, r_max, c_max]')
        if not (grid.in_bounds(r0, c0) and grid.in_bounds(r1, c1)):
            problems.append(f'shelf block {b} is outside the grid')
    expanded = cells_of_blocks(blocks)
    listed = set(grid.blocked_cells)
    if expanded != listed:
        problems.append('blocked_cells != union(shelf_blocks): '
                        f'missing={sorted(expanded - listed)} extra={sorted(listed - expanded)}')
    robot = raw.get('robot', {})
    if robot:
        need = 1.5 * robot['width'] + robot['localization_margin']
        if grid.resolution + 1e-9 < need:
            problems.append(f'resolution {grid.resolution} < 1.5*width + margin = {need:.3f}')
    for name, sc in raw.get('scenarios', {}).items():
        starts = [tuple(s[:2]) for s in sc.get('start_cells', [])]
        if len(set(starts)) != len(starts):
            problems.append(f'scenario {name}: duplicate start cells')
        for s in starts:
            if grid.is_blocked(*s):
                problems.append(f'scenario {name}: start cell {s} is blocked')
        for i, goals in enumerate(sc.get('test_goals', [])):
            for g in goals:
                if grid.is_blocked(g[0], g[1]):
                    problems.append(f'scenario {name}: robot{i + 1} goal {g[:2]} is blocked')
    if problems:
        raise ValueError('invalid warehouse grid:\n  ' + '\n  '.join(problems))


def static_boxes(grid):
    """Return the wall and shelf boxes of the warehouse (world frame)."""
    geo = grid.raw.get('geometry', {})
    inset = float(geo.get('shelf_inset', 0.05))
    shelf_h = float(geo.get('shelf_height', 0.5))
    gap = float(geo.get('wall_gap', 0.05))
    thick = float(geo.get('wall_thickness', 0.1))
    wall_h = float(geo.get('wall_height', 0.5))
    res = grid.resolution
    x0, y0 = grid.origin_x, grid.origin_y
    x1, y1 = x0 + grid.cols * res, y0 + grid.rows * res

    boxes = []
    # Walls: inner faces `gap` outside the grid boundary.
    xi0, xi1, yi0, yi1 = x0 - gap, x1 + gap, y0 - gap, y1 + gap
    boxes.append(Box('wall_south', 'wall', xi0 - thick, yi0 - thick, xi1 + thick, yi0, wall_h))
    boxes.append(Box('wall_north', 'wall', xi0 - thick, yi1, xi1 + thick, yi1 + thick, wall_h))
    boxes.append(Box('wall_west', 'wall', xi0 - thick, yi0, xi0, yi1, wall_h))
    boxes.append(Box('wall_east', 'wall', xi1, yi0, xi1 + thick, yi1, wall_h))
    # Shelves: one box per block, inset from the block's cell rectangle.
    for r0, c0, r1, c1 in shelf_blocks(grid.raw):
        boxes.append(Box(f'shelf_r{r0}_c{c0}', 'shelf',
                         x0 + c0 * res + inset, y0 + r0 * res + inset,
                         x0 + (c1 + 1) * res - inset, y0 + (r1 + 1) * res - inset, shelf_h))
    for b in boxes:
        b.x_min, b.y_min, b.x_max, b.y_max = (_r(b.x_min), _r(b.y_min), _r(b.x_max), _r(b.y_max))
    return boxes


# ---------------------------------------------------------------------- world SDF
_COLORS = {'wall': '0.55 0.55 0.58 1', 'shelf': '0.85 0.45 0.10 1'}


def _box_model_sdf(box):
    cx, cy = box.center
    sx, sy = box.size
    color = _COLORS[box.kind]
    geom = f'<box><size>{_r(sx)} {_r(sy)} {_r(box.height)}</size></box>'
    return (
        f'    <model name="{box.name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{_r(cx)} {_r(cy)} {_r(box.height / 2)} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <collision name="collision"><geometry>{geom}</geometry></collision>\n'
        f'        <visual name="visual"><geometry>{geom}</geometry>\n'
        f'          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>\n')


def world_sdf(grid, world_name='warehouse'):
    """Render the Gazebo Harmonic world SDF for the grid."""
    models = ''.join(_box_model_sdf(b) for b in static_boxes(grid))
    return f"""<?xml version="1.0" ?>
<!-- GENERATED by `ros2 run parakram_sim generate_warehouse` (v{GENERATOR_VERSION}) from
     config/warehouse_grid.yaml. Do not edit by hand: edit the YAML and regenerate. -->
<sdf version="1.8">
  <world name="{world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <!-- The render engine is chosen on the command line (fleet_sim.launch.py arg
         render_engine), not here, so a GPU machine and a VM can share this file. -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.75 0.75 0.75 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>

{models}  </world>
</sdf>
"""


# ---------------------------------------------------------------------- occupancy map
def map_extent(grid):
    """Return ``(x_min, y_min, width_px, height_px, resolution)`` of the occupancy map."""
    geo = grid.raw.get('geometry', {})
    res = float(geo.get('map_resolution', 0.05))
    margin = float(geo.get('map_margin', 0.25))
    boxes = static_boxes(grid)
    x_min = min(b.x_min for b in boxes) - margin
    y_min = min(b.y_min for b in boxes) - margin
    x_max = max(b.x_max for b in boxes) + margin
    y_max = max(b.y_max for b in boxes) + margin
    # Snap the origin to the map resolution so box edges fall on pixel boundaries.
    x_min = math.floor(x_min / res + 1e-9) * res
    y_min = math.floor(y_min / res + 1e-9) * res
    width = int(math.ceil((x_max - x_min) / res - 1e-9))
    height = int(math.ceil((y_max - y_min) / res - 1e-9))
    return _r(x_min), _r(y_min), width, height, res


def occupancy_rows(grid):
    """Return the map as rows of bytes, top row first (PGM order): 0 = occupied, 254 = free."""
    x_min, y_min, width, height, res = map_extent(grid)
    boxes = static_boxes(grid)
    walls = [b for b in boxes if b.kind == 'wall']
    inner = (max(b.x_max for b in walls if b.name == 'wall_west'),
             max(b.y_max for b in walls if b.name == 'wall_south'),
             min(b.x_min for b in walls if b.name == 'wall_east'),
             min(b.y_min for b in walls if b.name == 'wall_north'))
    rows = []
    for i in range(height):
        y = y_min + (height - 1 - i + 0.5) * res
        row = bytearray(width)
        for j in range(width):
            x = x_min + (j + 0.5) * res
            inside = inner[0] < x < inner[2] and inner[1] < y < inner[3]
            occupied = (not inside) or any(b.contains(x, y) for b in boxes)
            row[j] = 0 if occupied else 254
        rows.append(bytes(row))
    return rows


def map_pgm(grid):
    """Render the occupancy map as a binary PGM (P5)."""
    rows = occupancy_rows(grid)
    header = (f'P5\n# PARAKRAM warehouse (generated v{GENERATOR_VERSION})\n'
              f'{len(rows[0])} {len(rows)}\n255\n')
    return header.encode('ascii') + b''.join(rows)


def map_yaml(grid, image_name='warehouse.pgm'):
    """Render the nav2_map_server YAML for the occupancy map."""
    x_min, y_min, _, _, res = map_extent(grid)
    return (f'# GENERATED by `ros2 run parakram_sim generate_warehouse` '
            f'(v{GENERATOR_VERSION}) from\n'
            f'# parakram_sim/config/warehouse_grid.yaml. Map frame == Gazebo world frame.\n'
            f'image: {image_name}\n'
            f'mode: trinary\n'
            f'resolution: {res}\n'
            f'origin: [{x_min}, {y_min}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n')


# ---------------------------------------------------------------------- CLI
def _default_paths(grid_path):
    """Derive source-tree output paths from the (symlink-resolved) grid YAML location."""
    sim_pkg = os.path.dirname(os.path.dirname(os.path.realpath(grid_path)))
    src = os.path.dirname(sim_pkg)
    return (os.path.join(sim_pkg, 'worlds', 'warehouse.sdf'),
            os.path.join(src, 'parakram_bringup', 'maps'))


def main(argv=None):
    """Generate the world SDF and the occupancy map from the grid YAML."""
    from parakram_sim.grid_utils import default_grid_path
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--grid', default=None, help='grid YAML (default: installed one)')
    parser.add_argument('--world-out', default=None, help='output world SDF path')
    parser.add_argument('--map-dir', default=None,
                        help='output directory for warehouse.{yaml,pgm}')
    args = parser.parse_args(argv)

    grid_path = args.grid or default_grid_path()
    grid = WarehouseGrid.from_yaml(grid_path)
    validate(grid)
    world_out, map_dir = _default_paths(grid_path)
    world_out = args.world_out or world_out
    map_dir = args.map_dir or map_dir

    os.makedirs(os.path.dirname(world_out), exist_ok=True)
    os.makedirs(map_dir, exist_ok=True)
    with open(world_out, 'w') as f:
        f.write(world_sdf(grid))
    with open(os.path.join(map_dir, 'warehouse.pgm'), 'wb') as f:
        f.write(map_pgm(grid))
    with open(os.path.join(map_dir, 'warehouse.yaml'), 'w') as f:
        f.write(map_yaml(grid))
    x_min, y_min, w, h, res = map_extent(grid)
    print(f'grid     : {os.path.realpath(grid_path)} '
          f'({grid.rows}x{grid.cols} @ {grid.resolution} m)')
    print(f'world    : {world_out} ({len(static_boxes(grid))} static boxes)')
    print(f'map      : {map_dir}/warehouse.{{yaml,pgm}} '
          f'({w}x{h} px @ {res} m, origin {x_min},{y_min})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
