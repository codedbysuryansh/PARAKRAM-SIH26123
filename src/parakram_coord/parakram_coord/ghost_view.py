"""
Is a grid cell physically empty? Evidence from this robot's own lidar (CLAUDE_CODE/06), pure.

When a peer's lease expires, its RESERVED space is reclaimed, but its body has not vanished: the
cells its last renewal said it could occupy (its occupied cells and held authority) stay blocked
as a "ghost" until the peer is heard again or this robot has SEEN the cell empty. A cell is seen
empty when the lidar beams across it pass beyond its far side with no return inside it; a return
inside it means a body is there; a beam stopping short of it (occlusion) proves nothing.
"""

import math

OCCUPIED, FREE, UNKNOWN = 'occupied', 'free', 'unknown'


def _ray_box(ox, oy, dx, dy, x0, y0, x1, y1):
    """Parametric entry/exit [t_in, t_out] of a ray on an axis-aligned box, None if missed."""
    t_in, t_out = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if not lo <= o <= hi:
                return None
            continue
        a, b = (lo - o) / d, (hi - o) / d
        t_in, t_out = max(t_in, min(a, b)), min(t_out, max(a, b))
    if t_out < max(t_in, 0.0):
        return None
    return max(t_in, 0.0), t_out


def classify_cell(box, sensor, ranges, angle_min, angle_increment, range_min, range_max,
                  min_hits=2, min_through=0.7):
    """
    Classify a grid cell from one lidar scan.

    ``box`` = (x0, y0, x1, y1) [m, map frame]; ``sensor`` = (x, y, yaw) of the scan origin.
    Returns OCCUPIED (>= ``min_hits`` returns inside the box), FREE (no return inside and at
    least ``min_through`` of the beams that cross the box pass beyond it) or UNKNOWN.
    """
    sx, sy, syaw = sensor
    x0, y0, x1, y1 = box
    hits = through = crossing = 0
    for i, r in enumerate(ranges):
        a = syaw + angle_min + i * angle_increment
        dx, dy = math.cos(a), math.sin(a)
        span = _ray_box(sx, sy, dx, dy, x0, y0, x1, y1)
        if span is None:
            continue
        t_in, t_out = span
        if t_out > range_max:
            continue                                  # the box is (partly) out of range
        crossing += 1
        valid = math.isfinite(r) and range_min <= r <= range_max
        if valid and t_in <= r <= t_out:
            hits += 1
        elif not valid and (math.isinf(r) or r > range_max):
            through += 1                              # no return within range: passed through
        elif valid and r > t_out:
            through += 1
    if hits >= min_hits:
        return OCCUPIED
    if crossing and hits == 0 and through >= min_through * crossing:
        return FREE
    return UNKNOWN
