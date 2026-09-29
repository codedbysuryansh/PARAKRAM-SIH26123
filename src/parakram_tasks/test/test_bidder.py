"""Unit tests of the bid cost (CLAUDE_CODE/04): monotonic in distance and battery."""

import math
import os

from parakram_sim.grid_utils import WarehouseGrid
from parakram_tasks import bidder
import pytest

GRID = os.path.join(os.path.dirname(__file__), '..', '..', 'parakram_sim', 'config',
                    'warehouse_grid.yaml')


@pytest.fixture(scope='module')
def grid():
    return WarehouseGrid.from_yaml(GRID)


def test_grid_distance_is_the_shortest_free_path(grid):
    assert bidder.grid_distance(grid, (5, 2), (5, 2)) == 0
    assert bidder.grid_distance(grid, (5, 2), (5, 10)) == 8
    # (3, 3) -> (3, 9): no shortcut through the shelves, down to row 5 / 0 and back
    assert bidder.grid_distance(grid, (3, 3), (3, 9)) == 10
    assert bidder.grid_distance(grid, (5, 2), (2, 1)) is None      # a shelf cell


def test_cost_is_monotonic_in_distance(grid):
    task = bidder.TaskSpec('t', pickup=(3, 3), dropoff=(0, 0))
    costs = [bidder.cost(bidder.RobotSnapshot('r', cell), task, grid)
             for cell in ((3, 3), (4, 3), (5, 3), (5, 4), (5, 6), (5, 9), (5, 12))]
    assert all(b > a for a, b in zip(costs, costs[1:])), costs
    # pickup -> dropoff distance counts too
    near = bidder.cost(bidder.RobotSnapshot('r', (5, 3)),
                       bidder.TaskSpec('t', (3, 3), (0, 0)), grid)
    far = bidder.cost(bidder.RobotSnapshot('r', (5, 3)),
                      bidder.TaskSpec('t', (3, 3), (10, 12)), grid)
    assert far > near


def test_cost_is_monotonic_in_battery_and_load_and_withholds(grid):
    task = bidder.TaskSpec('t', (3, 3), (0, 0))
    costs = [bidder.cost(bidder.RobotSnapshot('r', (5, 6), battery=b), task, grid)
             for b in (1.0, 0.8, 0.6, 0.4, 0.21)]
    assert all(b > a for a, b in zip(costs, costs[1:])), costs
    assert bidder.cost(bidder.RobotSnapshot('r', (5, 6), battery=0.19), task, grid) == math.inf
    idle = bidder.cost(bidder.RobotSnapshot('r', (5, 6)), task, grid)
    busy = bidder.cost(bidder.RobotSnapshot('r', (5, 6), load=12.0), task, grid)
    assert busy == pytest.approx(idle + 12.0)
    # expected magnitude: (cells * 0.4 m / 0.15 m/s) + service time
    d = bidder.grid_distance(grid, (5, 6), (3, 3)) + bidder.grid_distance(grid, (3, 3), (0, 0))
    assert idle == pytest.approx(d * 0.4 / 0.15 + bidder.CostParams().service_time)
