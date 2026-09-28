"""
Grid <-> world transforms for the PARAKRAM warehouse grid.

The warehouse is a 4-connected grid graph (CLAUDE_CODE/00). Cell ``(r, c)``: ``r`` is the row
index and grows with +y, ``c`` is the column index and grows with +x. The lower-left corner of
cell ``(0, 0)`` sits at world ``(origin_x, origin_y)``. The map frame coincides with the world
frame. Flattened cell index (``Intent.reserved_cells``) is ``r * cols + c``.

Typical use from any package::

    from parakram_sim import grid_utils
    r, c = grid_utils.world_to_cell(x, y)
    x, y = grid_utils.cell_to_world(r, c)
    if grid_utils.is_blocked(r, c): ...

The module-level helpers use a default grid loaded lazily from ``$PARAKRAM_GRID_YAML`` if set,
otherwise from ``share/parakram_sim/config/warehouse_grid.yaml``. Call :func:`load_grid` to point
them at another file (e.g. the hardware floor grid in file 09).
"""

import math
import os

import yaml

GRID_ENV_VAR = 'PARAKRAM_GRID_YAML'


class WarehouseGrid:
    """An occupancy grid with a fixed world origin and square cells."""

    def __init__(self, origin_x, origin_y, resolution, rows, cols, blocked_cells=(),
                 frame_id='map', raw=None):
        """Create a grid; ``blocked_cells`` is an iterable of ``(r, c)`` pairs."""
        if resolution <= 0.0:
            raise ValueError(f'resolution must be > 0, got {resolution}')
        if rows <= 0 or cols <= 0:
            raise ValueError(f'rows/cols must be > 0, got {rows}x{cols}')
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.resolution = float(resolution)
        self.rows = int(rows)
        self.cols = int(cols)
        self.frame_id = frame_id
        self.raw = raw if raw is not None else {}
        self._blocked = set()
        for r, c in blocked_cells:
            r, c = int(r), int(c)
            if not self.in_bounds(r, c):
                raise ValueError(f'blocked cell {(r, c)} is outside the {rows}x{cols} grid')
            self._blocked.add((r, c))

    @classmethod
    def from_yaml(cls, path):
        """Load a grid from a ``warehouse_grid.yaml``-style file."""
        with open(path, 'r') as f:
            raw = yaml.safe_load(f)
        g = raw['grid']
        return cls(origin_x=g['origin'][0], origin_y=g['origin'][1],
                   resolution=g['resolution'], rows=g['rows'], cols=g['cols'],
                   blocked_cells=raw.get('blocked_cells', []),
                   frame_id=g.get('frame_id', 'map'), raw=raw)

    # ------------------------------------------------------------------ transforms
    def world_to_cell(self, x, y):
        """
        Return the ``(r, c)`` cell containing world point ``(x, y)``.

        Uses floor semantics (a point on a cell boundary belongs to the cell above/right of it).
        The result may be out of bounds; check with :meth:`in_bounds` or :meth:`is_blocked`.
        """
        c = math.floor((float(x) - self.origin_x) / self.resolution)
        r = math.floor((float(y) - self.origin_y) / self.resolution)
        return int(r), int(c)

    def cell_to_world(self, r, c):
        """Return the world ``(x, y)`` of the centre of cell ``(r, c)``."""
        return (self.origin_x + (int(c) + 0.5) * self.resolution,
                self.origin_y + (int(r) + 0.5) * self.resolution)

    def cell_bounds(self, r, c):
        """Return ``(x_min, y_min, x_max, y_max)`` of cell ``(r, c)`` in world coordinates."""
        x_min = self.origin_x + int(c) * self.resolution
        y_min = self.origin_y + int(r) * self.resolution
        return x_min, y_min, x_min + self.resolution, y_min + self.resolution

    # ------------------------------------------------------------------ occupancy
    def in_bounds(self, r, c):
        """Return True if ``(r, c)`` is inside the grid."""
        return 0 <= int(r) < self.rows and 0 <= int(c) < self.cols

    def is_blocked(self, r, c):
        """Return True if ``(r, c)`` is a static obstacle (shelf) or outside the grid."""
        return (not self.in_bounds(r, c)) or (int(r), int(c)) in self._blocked

    def is_free(self, r, c):
        """Return True if ``(r, c)`` is inside the grid and not a static obstacle."""
        return not self.is_blocked(r, c)

    @property
    def blocked_cells(self):
        """Sorted list of blocked ``(r, c)`` cells."""
        return sorted(self._blocked)

    def free_cells(self):
        """Sorted list of free ``(r, c)`` cells."""
        return [(r, c) for r in range(self.rows) for c in range(self.cols)
                if (r, c) not in self._blocked]

    def neighbors(self, r, c):
        """Return the free 4-connected neighbours of ``(r, c)``."""
        out = []
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = int(r) + dr, int(c) + dc
            if self.is_free(nr, nc):
                out.append((nr, nc))
        return out

    # ------------------------------------------------------------------ flattening
    def cell_to_index(self, r, c):
        """Flatten ``(r, c)`` to the ``Intent.reserved_cells`` index ``r * cols + c``."""
        if not self.in_bounds(r, c):
            raise ValueError(f'cell {(r, c)} is outside the {self.rows}x{self.cols} grid')
        return int(r) * self.cols + int(c)

    def index_to_cell(self, index):
        """Inverse of :meth:`cell_to_index`."""
        index = int(index)
        if not 0 <= index < self.rows * self.cols:
            raise ValueError(f'index {index} is outside the {self.rows}x{self.cols} grid')
        return divmod(index, self.cols)


# ---------------------------------------------------------------------- module-level API
_default_grid = None


def default_grid_path():
    """Return the grid YAML used by the module-level helpers."""
    env = os.environ.get(GRID_ENV_VAR)
    if env:
        return env
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('parakram_sim'),
                        'config', 'warehouse_grid.yaml')


def load_grid(path=None):
    """Load (or reload) the default grid used by the module-level helpers and return it."""
    global _default_grid
    _default_grid = WarehouseGrid.from_yaml(path or default_grid_path())
    return _default_grid


def get_grid():
    """Return the default grid, loading it on first use."""
    return _default_grid if _default_grid is not None else load_grid()


def world_to_cell(x, y):
    """Return the ``(r, c)`` cell containing world point ``(x, y)`` (default grid)."""
    return get_grid().world_to_cell(x, y)


def cell_to_world(r, c):
    """Return the world ``(x, y)`` centre of cell ``(r, c)`` (default grid)."""
    return get_grid().cell_to_world(r, c)


def is_blocked(r, c):
    """Return True if ``(r, c)`` is a shelf cell or out of bounds (default grid)."""
    return get_grid().is_blocked(r, c)
