"""
Robot footprint geometry for ground-truth contact checks (never used for control).

The TurtleBot3 Burger footprint is taken from turtlebot3_description (base plate box 0.140 m at
x = -0.032 m, wheels at y = +/-0.080 m with 0.018 m tread, caster at x = -0.081 m), bounded by
the rectangle below in the base_footprint frame. Distances are exact for convex polygons, so a
distance of 0 means the footprints/obstacles overlap (contact).
"""

import math

# Bounding rectangle of the Burger in base_footprint: x in [X_MIN, X_MAX], y in [-HALF_W, HALF_W].
BURGER_X_MIN = -0.105
BURGER_X_MAX = 0.040
BURGER_HALF_WIDTH = 0.090


def footprint_polygon(x, y, yaw, x_min=BURGER_X_MIN, x_max=BURGER_X_MAX,
                      half_width=BURGER_HALF_WIDTH):
    """Return the robot footprint at pose ``(x, y, yaw)`` as CCW world-frame vertices."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    local = ((x_max, -half_width), (x_max, half_width), (x_min, half_width), (x_min, -half_width))
    return [(x + cy * px - sy * py, y + sy * px + cy * py) for px, py in local]


def box_polygon(x_min, y_min, x_max, y_max):
    """Return an axis-aligned box as CCW vertices."""
    return [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]


def _project(poly, ax, ay):
    dots = [px * ax + py * ay for px, py in poly]
    return min(dots), max(dots)


def polygons_overlap(p, q):
    """Separating-axis test for two convex polygons (touching counts as overlap)."""
    for poly in (p, q):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            ax, ay = y1 - y2, x2 - x1  # edge normal
            pmin, pmax = _project(p, ax, ay)
            qmin, qmax = _project(q, ax, ay)
            if pmax < qmin or qmax < pmin:
                return False
    return True


def _point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    t = 0.0 if denom == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def polygon_distance(p, q):
    """Minimum distance between two convex polygons (0 if they overlap)."""
    if polygons_overlap(p, q):
        return 0.0
    best = math.inf
    for poly_a, poly_b in ((p, q), (q, p)):
        n = len(poly_b)
        for px, py in poly_a:
            for i in range(n):
                ax, ay = poly_b[i]
                bx, by = poly_b[(i + 1) % n]
                best = min(best, _point_segment_distance(px, py, ax, ay, bx, by))
    return best
