"""Unit tests for parakram_sim.grid_utils (CLAUDE_CODE/01 acceptance: world<->cell round trip)."""

import math
import os
import random

from parakram_sim import grid_utils
from parakram_sim.grid_utils import WarehouseGrid
import pytest

GRID_YAML = os.path.join(os.path.dirname(__file__), '..', 'config', 'warehouse_grid.yaml')


@pytest.fixture(scope='module')
def grid():
    return WarehouseGrid.from_yaml(GRID_YAML)


def test_grid_header(grid):
    assert (grid.rows, grid.cols) == (11, 13)
    assert grid.resolution == pytest.approx(0.40)
    assert (grid.origin_x, grid.origin_y) == (pytest.approx(-2.6), pytest.approx(-2.2))
    assert grid.frame_id == 'map'


def test_cell_to_world_to_cell_round_trip_every_cell(grid):
    for r in range(grid.rows):
        for c in range(grid.cols):
            x, y = grid.cell_to_world(r, c)
            assert grid.world_to_cell(x, y) == (r, c)


def test_world_to_cell_to_world_is_cell_centre(grid):
    rng = random.Random(1)
    half = grid.resolution / 2.0
    for _ in range(5000):
        x = rng.uniform(grid.origin_x, grid.origin_x + grid.cols * grid.resolution)
        y = rng.uniform(grid.origin_y, grid.origin_y + grid.rows * grid.resolution)
        r, c = grid.world_to_cell(x, y)
        assert grid.in_bounds(r, c)
        cx, cy = grid.cell_to_world(r, c)
        assert abs(cx - x) <= half + 1e-9 and abs(cy - y) <= half + 1e-9
        x_min, y_min, x_max, y_max = grid.cell_bounds(r, c)
        assert x_min - 1e-9 <= x <= x_max + 1e-9 and y_min - 1e-9 <= y <= y_max + 1e-9


def test_points_inside_a_cell_map_to_that_cell(grid):
    for r in (0, 5, grid.rows - 1):
        for c in (0, 6, grid.cols - 1):
            cx, cy = grid.cell_to_world(r, c)
            for dx in (-0.49, 0.0, 0.49):
                for dy in (-0.49, 0.0, 0.49):
                    assert grid.world_to_cell(cx + dx * grid.resolution,
                                              cy + dy * grid.resolution) == (r, c)


def test_axes_orientation(grid):
    # +x -> +c, +y -> +r ; central junction (5, 6) sits at the world origin.
    assert grid.cell_to_world(5, 6) == (pytest.approx(0.0), pytest.approx(0.0))
    assert grid.world_to_cell(0.0 + 0.4, 0.0) == (5, 7)
    assert grid.world_to_cell(0.0, 0.0 + 0.4) == (6, 6)
    assert grid.world_to_cell(grid.origin_x + 1e-6, grid.origin_y + 1e-6) == (0, 0)


def test_out_of_bounds(grid):
    assert grid.world_to_cell(grid.origin_x - 0.01, 0.0)[1] == -1
    for r, c in ((-1, 0), (0, -1), (grid.rows, 0), (0, grid.cols)):
        assert not grid.in_bounds(r, c)
        assert grid.is_blocked(r, c)
        assert not grid.is_free(r, c)


def test_blocked_cells_are_the_shelves(grid):
    blocked = set(grid.blocked_cells)
    assert len(blocked) == 64
    assert len(grid.free_cells()) == grid.rows * grid.cols - 64
    expected = set()
    for r0, c0, r1, c1 in grid.raw['shelf_blocks']:
        expected |= {(r, c) for r in range(r0, r1 + 1) for c in range(c0, c1 + 1)}
    assert blocked == expected
    for r, c in blocked:
        assert grid.is_blocked(r, c)
    # Aisles: rows 0/5/10 and columns 0/3/6/9/12 are entirely free.
    for r in (0, 5, 10):
        assert all(grid.is_free(r, c) for c in range(grid.cols))
    for c in (0, 3, 6, 9, 12):
        assert all(grid.is_free(r, c) for r in range(grid.rows))


def test_flatten_round_trip(grid):
    seen = set()
    for r in range(grid.rows):
        for c in range(grid.cols):
            i = grid.cell_to_index(r, c)
            assert i == r * grid.cols + c
            assert grid.index_to_cell(i) == (r, c)
            seen.add(i)
    assert seen == set(range(grid.rows * grid.cols))
    with pytest.raises(ValueError):
        grid.cell_to_index(-1, 0)
    with pytest.raises(ValueError):
        grid.index_to_cell(grid.rows * grid.cols)


def test_neighbors(grid):
    assert sorted(grid.neighbors(5, 6)) == [(4, 6), (5, 5), (5, 7), (6, 6)]
    assert sorted(grid.neighbors(0, 0)) == [(0, 1), (1, 0)]
    assert sorted(grid.neighbors(2, 3)) == [(1, 3), (3, 3)]


def test_cell_size_rule(grid):
    robot = grid.raw['robot']
    assert grid.resolution >= 1.5 * robot['width'] + robot['localization_margin']


def test_invalid_grids_rejected():
    with pytest.raises(ValueError):
        WarehouseGrid(0, 0, 0.0, 3, 3)
    with pytest.raises(ValueError):
        WarehouseGrid(0, 0, 0.4, 3, 3, blocked_cells=[(3, 0)])


def test_module_level_api(monkeypatch):
    monkeypatch.setenv(grid_utils.GRID_ENV_VAR, GRID_YAML)
    g = grid_utils.load_grid()
    assert grid_utils.get_grid() is g
    assert grid_utils.world_to_cell(*grid_utils.cell_to_world(3, 6)) == (3, 6)
    assert grid_utils.is_blocked(1, 1) and not grid_utils.is_blocked(0, 0)
    assert math.isclose(grid_utils.cell_to_world(0, 0)[0], -2.4)
