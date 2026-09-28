"""Unit tests: windowed space-time A* (optimality, constraints, window truncation)."""

from collections import deque
import os
import random

from parakram_coord.spacetime_astar import (compress, distance_field, moving_prefix,
                                            spacetime_astar)
from parakram_sim.grid_utils import WarehouseGrid
import pytest

GRID_YAML = os.path.join(os.path.dirname(__file__), '..', '..', 'parakram_sim', 'config',
                         'warehouse_grid.yaml')


def grid_from_ascii(rows):
    """Hand-built grid: rows[0] is the TOP row; '#' = shelf, '.' = free."""
    n_rows, n_cols = len(rows), len(rows[0])
    blocked = [(n_rows - 1 - i, j) for i, line in enumerate(rows)
               for j, ch in enumerate(line) if ch == '#']
    return WarehouseGrid(0.0, 0.0, 0.4, n_rows, n_cols, blocked)


def nbrs_of(grid):
    return lambda c: grid.neighbors(c[0], c[1])


def brute_force(start, goal, window, nbrs, blocked=frozenset(), blocked_st=frozenset(),
                blocked_edges=frozenset()):
    """Earliest arrival by exhaustive BFS over (cell, t) — the reference optimum."""
    seen = {(start, 0)}
    q = deque([(start, 0)])
    while q:
        c, t = q.popleft()
        if c == goal:
            return t
        if t == window:
            continue
        for n in [c] + list(nbrs(c)):
            s = (n, t + 1)
            if n in blocked or s in blocked_st or (c, n, t) in blocked_edges or s in seen:
                continue
            seen.add(s)
            q.append(s)
    return None


def valid_path(path, nbrs):
    return all(b == a or b in nbrs(a) for a, b in zip(path, path[1:]))


@pytest.fixture(scope='module')
def warehouse():
    return WarehouseGrid.from_yaml(GRID_YAML)


def test_optimal_on_warehouse_grid(warehouse):
    nbrs = nbrs_of(warehouse)
    free = warehouse.free_cells()
    rng = random.Random(0)
    for _ in range(200):
        s, g = rng.choice(free), rng.choice(free)
        h = distance_field([g], nbrs)
        path, reached = spacetime_astar(s, g, 60, nbrs, h)
        assert reached and path[0] == s and path[-1] == g and valid_path(path, nbrs)
        assert len(path) - 1 == h[s]                       # = BFS shortest path length


def test_hand_built_detour_is_optimal():
    grid = grid_from_ascii(['.......',
                            '.#####.',
                            '.......'])
    nbrs = nbrs_of(grid)
    s, g = (0, 0), (0, 6)
    blocked = {(0, 3)}                                     # bottom lane blocked at every t>=1
    h = distance_field([g], nbrs)
    path, reached = spacetime_astar(s, g, 30, nbrs, h, blocked=frozenset(blocked))
    assert reached and (0, 3) not in path
    assert len(path) - 1 == brute_force(s, g, 30, nbrs, blocked=blocked) == 10


def test_vertex_constraint_forces_optimal_wait():
    grid = grid_from_ascii(['#####',
                            '.....',
                            '#####'])
    nbrs = nbrs_of(grid)
    s, g = (1, 0), (1, 4)
    blocked_st = {((1, 2), 2), ((1, 2), 3)}                # the middle cell is busy at t=2,3
    h = distance_field([g], nbrs)
    path, reached = spacetime_astar(s, g, 20, nbrs, h, blocked_st=frozenset(blocked_st))
    best = brute_force(s, g, 20, nbrs, blocked_st=blocked_st)
    assert reached and len(path) - 1 == best == 6
    assert all((path[t], t) not in blocked_st for t in range(len(path)))
    assert len(compress(path)) < len(path)                  # it waited somewhere


def test_edge_constraint_prevents_swap():
    grid = grid_from_ascii(['...'])
    nbrs = nbrs_of(grid)
    s, g = (0, 0), (0, 2)
    edges = {((0, 0), (0, 1), 0), ((0, 1), (0, 2), 1)}
    h = distance_field([g], nbrs)
    path, reached = spacetime_astar(s, g, 10, nbrs, h, blocked_edges=frozenset(edges))
    assert reached
    assert len(path) - 1 == brute_force(s, g, 10, nbrs, blocked_edges=edges)
    for t, (a, b) in enumerate(zip(path, path[1:])):
        assert (a, b, t) not in edges


def test_window_truncation(warehouse):
    nbrs = nbrs_of(warehouse)
    s, g = (0, 0), (10, 12)
    h = distance_field([g], nbrs)
    assert h[s] == 22
    for window in (1, 4, 8):
        path, reached = spacetime_astar(s, g, window, nbrs, h)
        assert not reached and len(path) == window + 1 and valid_path(path, nbrs)
        assert h[path[-1]] == h[s] - window                 # best truncated prefix
        assert moving_prefix(path) == path[1:]              # no waits in an open plan


def test_window_zero_and_unreachable():
    grid = grid_from_ascii(['..#..'])
    nbrs = nbrs_of(grid)
    h = distance_field([(0, 4)], nbrs)
    assert spacetime_astar((0, 0), (0, 4), 5, nbrs, h) == ([(0, 0)], False)
    h2 = distance_field([(0, 1)], nbrs)
    assert spacetime_astar((0, 0), (0, 1), 0, nbrs, h2) == ([(0, 0)], False)


def test_goal_predicate_finds_nearest(warehouse):
    nbrs = nbrs_of(warehouse)
    targets = {(4, 6), (6, 6)}
    h = distance_field(list(targets), nbrs)
    path, reached = spacetime_astar((5, 4), None, 10, nbrs, h,
                                    goal_test=lambda c: c in targets)
    assert reached and path[-1] in targets and len(path) - 1 == 3


def test_deterministic(warehouse):
    nbrs = nbrs_of(warehouse)
    h = distance_field([(10, 12)], nbrs)
    runs = {tuple(spacetime_astar((0, 0), (10, 12), 30, nbrs, h,
                                  blocked=frozenset({(5, 6)}))[0]) for _ in range(5)}
    assert len(runs) == 1


def test_compress_and_moving_prefix():
    path = [(0, 0), (0, 1), (0, 1), (0, 2)]
    assert compress(path) == [(0, 0), (0, 1), (0, 2)]
    assert moving_prefix(path) == [(0, 1)]
    assert moving_prefix([(0, 0)]) == []
