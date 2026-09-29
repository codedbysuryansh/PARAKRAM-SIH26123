"""
Bid cost (CLAUDE_CODE/04), pure and unit-tested: ``cost(robot_state, task, grid) -> float``.

cost = estimated time-to-serve [s] = (grid distance robot -> pickup + pickup -> dropoff) at the
planning speed, plus the pickup and dropoff service time, scaled up as the battery drains, plus
the robot's current load (time until it is free). A robot below ``battery_min`` withholds
(infinite cost), as does one that cannot reach the pickup or the dropoff. Distances are
shortest paths on the static warehouse grid (4-connected free cells), not straight lines.
"""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CostParams:
    """Planning constants of the cost estimate."""

    speed: float = 0.15            # [m/s] average travel speed incl. turns (Burger, Nav2 RPP)
    service_time: float = 2.0      # [s] pickup + dropoff dwell
    battery_min: float = 0.2       # below this state of charge: withhold (infinite cost)
    battery_weight: float = 1.0    # cost x (1 + weight * (1 - battery))


@dataclass(frozen=True)
class RobotSnapshot:
    """What a bidder knows about itself."""

    robot_id: str
    cell: tuple                    # (r, c) current grid cell
    battery: float = 1.0           # 0..1
    load: float = 0.0              # [s] until free of already-held work


@dataclass(frozen=True)
class TaskSpec:
    """A pickup -> dropoff task on the grid."""

    task_id: str
    pickup: tuple                  # (r, c)
    dropoff: tuple                 # (r, c)


_fields = {}


def grid_distance(grid, a, b):
    """Shortest 4-connected path length in cells from ``a`` to ``b`` (None if unreachable)."""
    a, b = (int(a[0]), int(a[1])), (int(b[0]), int(b[1]))
    if not (grid.is_free(*a) and grid.is_free(*b)):
        return None
    key = (id(grid), b)
    field = _fields.get(key)
    if field is None:
        field = {b: 0}
        queue = deque([b])
        while queue:
            cur = queue.popleft()
            for n in grid.neighbors(*cur):
                if n not in field:
                    field[n] = field[cur] + 1
                    queue.append(n)
        _fields[key] = field
    return field.get(a)


def cost(robot_state, task, grid, params=CostParams()):
    """Estimated time-to-serve [s] for ``robot_state`` to do ``task``; inf = withhold."""
    if robot_state.battery < params.battery_min:
        return math.inf
    to_pickup = grid_distance(grid, robot_state.cell, task.pickup)
    to_dropoff = grid_distance(grid, task.pickup, task.dropoff)
    if to_pickup is None or to_dropoff is None:
        return math.inf
    travel = (to_pickup + to_dropoff) * grid.resolution / params.speed
    battery = min(max(robot_state.battery, 0.0), 1.0)
    return (travel + params.service_time) * (1.0 + params.battery_weight * (1.0 - battery)) \
        + max(robot_state.load, 0.0)
