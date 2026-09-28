"""
Cross-package consistency of the coordination geometry (CLAUDE_CODE/02 integration findings).

A parked robot sits within the Nav2 goal tolerance of its cell centre. Its safety disk
(``occupancy_radius``) must cover the Burger footprint, and disk + tolerance must fit inside
the cell: otherwise a parked robot "occupies" a neighbour cell nobody else can then claim
(this blocked a goal cell and the junction in the 02 integration runs).
"""

import importlib.util
import math
import os

from parakram_coord.pibt_rule import CoordParams
from parakram_sim.footprint import BURGER_HALF_WIDTH, BURGER_X_MAX, BURGER_X_MIN
from parakram_sim.grid_utils import WarehouseGrid
import yaml

HERE = os.path.dirname(__file__)
GRID_YAML = os.path.join(HERE, '..', '..', 'parakram_sim', 'config', 'warehouse_grid.yaml')
NAV2_YAML = os.path.join(HERE, '..', '..', 'parakram_bringup', 'config', 'nav2_params.yaml')
COORD_LAUNCH = os.path.join(HERE, '..', 'launch', 'coord.launch.py')


def _launch_defaults():
    spec = importlib.util.spec_from_file_location('coord_launch', COORD_LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    return {e.name: ''.join(s.text for s in e.default_value) for e in ld.entities
            if getattr(e, 'default_value', None)}


def test_parked_robot_safety_disk_fits_its_cell_and_covers_the_footprint():
    grid = WarehouseGrid.from_yaml(GRID_YAML)
    with open(NAV2_YAML) as f:
        nav2 = yaml.safe_load(f)
    checker = nav2['controller_server']['ros__parameters']['general_goal_checker']
    tolerance = checker['xy_goal_tolerance']
    circumscribed = max(math.hypot(x, y) for x in (BURGER_X_MIN, BURGER_X_MAX)
                        for y in (-BURGER_HALF_WIDTH, BURGER_HALF_WIDTH))
    launch = _launch_defaults()
    for radius in (CoordParams().occupancy_radius, float(launch['occupancy_radius'])):
        assert radius >= circumscribed
        assert radius + tolerance <= grid.resolution / 2.0


def test_launch_and_library_defaults_agree():
    launch = _launch_defaults()
    p = CoordParams()
    assert float(launch['deadlock_timeout']) == p.deadlock_timeout
    assert float(launch['priority_grow_rate']) == p.priority_grow_rate
    assert float(launch['occupancy_radius']) == p.occupancy_radius
