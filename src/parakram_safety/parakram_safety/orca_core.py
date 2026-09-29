"""
Reactive safety filter math, Part B of CLAUDE_CODE/03 (pure: no ROS, unit-tested).

NH-ORCA (Alonso-Mora, Breitenmoser, Rufli, Beardsley, Siegwart, "Optimal reciprocal collision
avoidance for multiple non-holonomic robots", DARS 2010) on top of ORCA (van den Berg, Guy, Lin,
Manocha, "Reciprocal n-body collision avoidance", ISRR 2009), fed only by the onboard lidar:

1. The differential-drive robot is a holonomic DISC centred on its wheel axle (base_footprint):
   radius = the footprint's farthest corner (0.138 m for the Burger) + ``tracking_error``
   (epsilon), so that the real robot always lies inside the disc of its holonomic path.
2. A holonomic velocity ``u`` at angle ``theta`` to the heading is tracked by turning at
   ``w = theta / T`` with ``v = |u| (theta/2) / tan(theta/2)`` (the error-optimal arc); that
   manoeuvre never leaves the straight holonomic path by more than ``|u| T |sin(theta/2)|``. The
   allowed holonomic velocities (P^AHV) are the sector ``|theta| <= w_max T``, ``|u| <=
   v_max / f(w_max T)`` (every command of the velocity box maps into it), plus the mirrored
   reverse sector up to ``v_reverse_max``. The parameters must satisfy ``max_tracking_error(p)
   <= tracking_error``: then every allowed velocity is trackable within epsilon.
3. Every sensed obstacle gives one ORCA half-plane in u-space (RVO2's construction, time horizon
   tau; v_opt = the robot's current velocity for peers, 0 for shelves and walls: note (c)). A
   robot-like lidar blob (a peer: seen only by its turret) is a disc of ``peer_radius`` inflated
   by ``safety_margin`` and the avoidance is SHARED (1/2, reciprocal: every robot runs this
   filter); any other return (shelf, wall) is a point inflated by ``static_margin`` and avoided
   fully. ``safety_margin`` is the uncertainty
   inflation of Hennes et al., AAMAS 2012 (sensing noise, scan staleness). ``static_margin`` is 0
   by default: the disc about the axle already reaches >= 0.045 m beyond the body's sides and
   rear and 0.11 m beyond its front (only the rear corners are tight, at epsilon), and measured in
   the noisy 0.40 m aisles an extra 0.01-0.02 m made the filter push a centred robot about.
4. The new holonomic velocity is the point of P^AHV inside every half-plane that is closest to the
   nominal command's own holonomic velocity (the RVO2 incremental linear program), mapped back to
   ``(v, w)`` by the tracking law. A nominal that already satisfies every half-plane passes
   through unchanged. When no allowed velocity satisfies all half-planes (the robot already sits
   inside an inflated radius, e.g. robots spawned adjacent), the velocity of a slow creep sector
   that minimises the largest violation is used: the "creep escape".

Neighbour velocities are NOT estimated and no peer message is used: a peer is a static disc for
the half-plane (the work order's fallback, "treat neighbors as slow/static obstacles"); against a
moving peer the avoidance relies on the peer running the same filter (reciprocity), a
non-cooperating fast mover is outside this argument. Honest claim: probabilistically safe within
sensing range (not a guarantee: 5 Hz scans, 0.02 m range noise, turret-only sensing, 0.12 m blind
radius), no liveness guarantee.

Two documented differences from RVO2: (a) inside an inflated radius RVO2 projects onto the
cut-off circle of ONE time step, an escape demand of (R - d)/dt that no creep speed can meet.
Here the robot must move away at >= share * (R - d) / tau (inside a peer's radius: the creep
escape of a very close start; inside a shelf's or wall's: steering back to the aisle centre),
except that while the nominal is an in-place turn (or a stop) a shelf or wall only forbids
coming closer: the disc over-covers the Burger behind its axle, so being inside it is no contact,
and in Gazebo an escape demand near a shelf corner overrode Nav2's in-place turns until Nav2's
progress checker failed (measured). (b) When the solution is standing still (u = 0) and standing
still is allowed, the nominal's in-place rotation is kept: turning about the axle does not move
the disc (it tracks u = 0 exactly), and freezing the rotation would block Nav2's recoveries. The
body's corners during such a turn are guarded by the collision monitor's exact-footprint
time-to-collision zone. (c) For shelves and walls v_opt = 0, one of the choices of the ORCA paper
(van den Berg et al., Sec. 5.2): the half-plane is then the cut-off cap ``u . n <= (d - R) / tau``
and standing still is always allowed outside the radius. With RVO2's v_opt = current velocity
the tangent picked by a small leftover velocity near a shelf could exclude standing still, and
the filter then kept overriding Nav2's in-place turns for seconds (measured in Gazebo).
"""

from dataclasses import dataclass
import math
import warnings

import numpy as np

EPS = 1e-9


# ------------------------------------------------------------------------------ geometry
@dataclass(frozen=True)
class Footprint:
    """Axis-aligned rectangle in the robot (base_footprint) frame."""

    x_min: float = -0.105
    x_max: float = 0.040
    y_min: float = -0.090
    y_max: float = 0.090


def footprint_distance(points, fp):
    """
    Distance from each point to the rectangle ``fp`` and the unit normal (rectangle -> point).

    ``points``: (N, 2) array in the robot frame. Returns ``(d, n)``; ``d`` is 0 and ``n`` points
    out through the nearest edge for points inside the rectangle.
    """
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    qx = np.clip(p[:, 0], fp.x_min, fp.x_max)
    qy = np.clip(p[:, 1], fp.y_min, fp.y_max)
    dx, dy = p[:, 0] - qx, p[:, 1] - qy
    d = np.hypot(dx, dy)
    n = np.zeros_like(p)
    out = d > EPS
    n[out, 0] = dx[out] / d[out]
    n[out, 1] = dy[out] / d[out]
    inside = ~out
    if inside.any():
        # push out through the nearest edge
        e = np.stack([p[inside, 0] - fp.x_min, fp.x_max - p[inside, 0],
                      p[inside, 1] - fp.y_min, fp.y_max - p[inside, 1]], axis=1)
        k = np.argmin(e, axis=1)
        normals = np.array([[-1.0, 0.0], [1.0, 0.0], [0.0, -1.0], [0.0, 1.0]])
        n[inside] = normals[k]
        d[inside] = 0.0
    return d, n


# ------------------------------------------------------------------------------ lidar
@dataclass(frozen=True)
class ScanParams:
    """Scan-to-obstacle parameters (robot-like blob detection)."""

    range_min: float = 0.12
    range_max: float = 3.5
    max_range_used: float = 2.0      # points beyond this cannot constrain one tick
    segment_gap: float = 0.10        # [m] consecutive points closer than this: same segment
    median_window: int = 3           # beams; running median of the ranges (all consumers)
    blob_max_extent: float = 0.16    # [m] a lidar turret is ~0.10 m across
    blob_depth_jump: float = 0.10    # [m] a blob stands this much in front of both neighbours
    dropout_lookahead: int = 4       # missing beams bridged inside / looked past beside a segment
    turret_radius: float = 0.05      # [m] sensed turret surface -> turret axis
    peer_radius: float = 0.116       # [m] turret axis -> farthest point of a Burger body


def filtered_scan_points(ranges, angle_min, angle_increment, sensor_pose, sp):
    """
    Scan returns for the barriers: missing-aware ``median_window`` median, then valid returns.

    The median suppresses single-beam range noise (the minimum over noisy returns otherwise sits
    ~2.5 sigma inside every wall, which made the filter needlessly restrictive in the 0.40 m
    aisles) and bridges single dropouts; anything at least two beams wide survives it. The
    collision monitor (Part A) still works on the raw scan.
    """
    return scan_points(smooth_ranges(ranges, sp.median_window), angle_min, angle_increment,
                       sensor_pose, sp)


def scan_points(ranges, angle_min, angle_increment, sensor_pose, sp):
    """
    Return the valid scan returns in the robot frame.

    Returns ``(pts, idx, rng)``: (N, 2) points, their beam indices and ranges. ``sensor_pose`` is
    ``(x, y, yaw)`` of the lidar in the robot frame.
    """
    r = np.asarray(ranges, dtype=float)
    valid = np.isfinite(r) & (r > sp.range_min) & (r < sp.range_max)
    idx = np.nonzero(valid)[0]
    a = angle_min + idx * angle_increment + sensor_pose[2]
    rr = r[idx]
    pts = np.stack([sensor_pose[0] + rr * np.cos(a), sensor_pose[1] + rr * np.sin(a)], axis=1)
    return pts, idx, rr


def smooth_ranges(ranges, window):
    """
    Circular running median over ``window`` beams of the VALID returns only.

    A beam gets the median of the valid returns in its window when at least a majority of the
    window returned; otherwise it keeps its raw value. A single dropout inside a surface (noise
    below ``range_min``) is thereby bridged instead of splitting the surface in two.
    """
    r = np.asarray(ranges, dtype=float)
    if window <= 1 or len(r) < window:
        return r
    half = window // 2
    stack = np.stack([np.roll(r, k) for k in range(-half, half + 1)])
    valid = np.isfinite(stack) & (stack > 0.0)
    enough = valid.sum(axis=0) >= half + 1
    with np.errstate(all='ignore'), warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)       # all-missing windows -> nan
        med = np.nanmedian(np.where(valid, stack, np.nan), axis=0)
    out = r.copy()
    out[enough] = med[enough]
    return out


def find_blobs(ranges, angle_min, angle_increment, sensor_pose, sp):
    """
    Robot-like blobs: short segments standing in front of both neighbouring beams.

    A peer is seen only by its lidar turret (~0.10 m), which forms a short arc with a depth jump
    (or no return) on both sides. Shelf and wall faces form long segments, and a shelf corner has
    a continuing face on one side, so neither qualifies. Missing returns next to a segment are
    looked past: noisy returns from a shelf just outside ``range_min`` drop out and would cut it
    into turret-sized fragments, but there the same face continues at a similar depth beyond
    the gap. Classification runs on median-smoothed
    ranges: with 0.02 m range noise, raw consecutive returns on one turret often jumped more
    than the segment gap and split it, and a split turret was no longer recognised (measured on
    recorded 03 scans: 67 % of close peers found with raw 0.06 m segmentation, 100 % with this).
    Per-scan detection still misses a close peer now and then; ``PeerTracker`` bridges that.

    Returns ``(centres, beams)``: (M, 2) turret-axis estimates in the robot frame and, per blob,
    the array of beam indices it covers.
    """
    r_all = smooth_ranges(ranges, sp.median_window)
    n_beams = len(r_all)
    pts, idx, rng = scan_points(r_all, angle_min, angle_increment, sensor_pose, sp)
    if len(idx) == 0:
        return np.zeros((0, 2)), []
    # segments of neighbouring returns (circular scan) with small gaps; up to
    # ``dropout_lookahead`` missing beams between two returns are bridged: noise pushes the
    # nearest returns of a close turret below ``range_min`` and would otherwise split it
    skip = sp.dropout_lookahead + 1
    breaks = [0]
    for k in range(1, len(idx)):
        if idx[k] - idx[k - 1] > skip or np.hypot(*(pts[k] - pts[k - 1])) > sp.segment_gap:
            breaks.append(k)
    segs = [list(range(breaks[i], breaks[i + 1] if i + 1 < len(breaks) else len(idx)))
            for i in range(len(breaks))]
    if len(segs) > 1 and idx[0] + n_beams - idx[-1] <= skip and \
            np.hypot(*(pts[0] - pts[-1])) <= sp.segment_gap:
        segs[0] = segs[-1] + segs[0]              # wrap-around segment
        segs.pop()

    def background(beam, step, near):
        # the first valid return outward from the segment decides: farther by blob_depth_jump
        # -> background; at a similar depth -> the same surface continues past a dropout
        for k in range(sp.dropout_lookahead + 1):
            r = r_all[(beam + k * step) % n_beams]
            if np.isfinite(r) and sp.range_min < r < sp.range_max:
                return r > near + sp.blob_depth_jump
        return True                                # nothing in range beyond: open space

    centres, beams = [], []
    for seg in segs:
        if len(seg) < 2:
            continue
        p = pts[seg]
        extent = np.hypot(*(p[0] - p[-1]))
        if extent > sp.blob_max_extent:
            continue
        near = float(np.min(rng[seg]))
        if not (background(idx[seg[0]] - 1, -1, near) and background(idx[seg[-1]] + 1, 1, near)):
            continue
        c = p.mean(axis=0)
        ray = c - np.asarray(sensor_pose[:2])
        c = c + sp.turret_radius * ray / max(np.linalg.norm(ray), EPS)
        centres.append(c)
        beams.append(idx[seg])
    return (np.array(centres) if centres else np.zeros((0, 2))), beams


# ------------------------------------------------------------------------------ peer memory
@dataclass(frozen=True)
class TrackParams:
    """Short-term memory of detected peers (``PeerTracker``)."""

    memory: float = 1.0          # [s] an undetected peer is kept this long after its last support
    hold_max: float = 5.0        # [s] ... and never longer than this after its last detection
    assoc_radius: float = 0.12   # [m] a detection this close to a track is that track
    support_tol: float = 0.06    # [m] a return this close to the expected turret surface
    min_support: int = 2         # beams needed to support a track, or to see through it


class PeerTracker:
    """
    Short-term memory of robot-like blobs, kept in the odometry frame.

    Blob detection classifies every scan afresh and misses a peer now and then, mostly at close
    range, where the turret spans many beams, is flanked by shelf corners and partly lies inside
    the lidar's blind radius (noisy 3-robot junction simulation: ~1 % of scans with the bodies
    6-10 cm apart, most scans below 4 cm). One miss turns the peer's disc barrier into bare
    turret returns with the static margin, although its body reaches beyond them, and robots
    crept into contact that way. The tracker bridges misses: an undetected peer is kept while the
    scan still shows returns where its turret would be (or the turret is inside the blind
    radius), for ``memory`` s after that support ends and never longer than ``hold_max`` s after
    its last detection; it is dropped at once when the scan sees through it (the peer left).
    Pure sensing: no peer message is involved.
    """

    def __init__(self, tp=TrackParams()):
        """Start with no tracks."""
        self.tp = tp
        self.tracks = []       # [x, y] (odom frame), t_detected, t_supported

    def update(self, t, odom, centres, beams, ranges, angle_min, angle_increment, sensor_pose,
               sp):
        """
        Fold one scan's detections in; return ``(centres, beams)`` for ``obstacles_from_scan``.

        ``centres``/``beams``: ``find_blobs`` of this scan (robot frame at the scan). ``odom``:
        robot pose ``(x, y, yaw)`` in the odometry frame at the scan, or None, which clears the
        memory (the detections are returned unchanged). Remembered peers are appended with no
        beams.
        """
        centres = np.asarray(centres, dtype=float).reshape(-1, 2)
        if odom is None:
            self.tracks = []
            return centres, list(beams)
        x, y, yaw = odom
        c, s = math.cos(yaw), math.sin(yaw)

        def to_robot(p):
            dx, dy = p[0] - x, p[1] - y
            return np.array([c * dx + s * dy, -s * dx + c * dy])

        def to_odom(p):
            return [x + c * p[0] - s * p[1], y + s * p[0] + c * p[1]]

        local = [to_robot(tr[0]) for tr in self.tracks]
        pairs = sorted((float(np.hypot(*(centres[i] - local[j]))), i, j)
                       for i in range(len(centres)) for j in range(len(local)))
        det_used, trk_used = set(), set()
        for d, i, j in pairs:
            if d <= self.tp.assoc_radius and i not in det_used and j not in trk_used:
                det_used.add(i)
                trk_used.add(j)
                self.tracks[j] = [to_odom(centres[i]), t, t]
        kept, remembered = [], []
        rs = smooth_ranges(ranges, sp.median_window)
        for j, tr in enumerate(self.tracks):
            if j in trk_used:
                kept.append(tr)
                continue
            p = local[j]
            if len(centres) and np.min(np.hypot(*(centres - p).T)) <= self.tp.assoc_radius:
                continue                                  # a duplicate of a detected peer
            state = self._evidence(p, rs, angle_min, angle_increment, sensor_pose, sp)
            if state == 'free':
                continue                                  # seen through: the peer has left
            if state == 'supported':
                tr[2] = t
            if t - tr[2] <= self.tp.memory and t - tr[1] <= self.tp.hold_max:
                kept.append(tr)
                remembered.append(p)
        for i in range(len(centres)):
            if i not in det_used:
                kept.append([to_odom(centres[i]), t, t])
        self.tracks = kept
        out = np.vstack([centres] + [np.asarray(remembered).reshape(-1, 2)])
        return out, list(beams) + [np.zeros(0, dtype=int)] * len(remembered)

    def _evidence(self, p, rs, angle_min, angle_increment, sensor_pose, sp):
        """``'free'``, ``'supported'`` or ``'unknown'``: what the scan says about a turret at p."""
        n = len(rs)
        rel = p - np.asarray(sensor_pose[:2], dtype=float)
        d = float(np.hypot(*rel))
        rad = sp.turret_radius
        if d <= rad + 1e-3:
            return 'unknown'
        bearing = math.atan2(rel[1], rel[0]) - sensor_pose[2]
        half = math.asin(rad / d)
        full = abs(n * angle_increment - 2 * math.pi) < 1.5 * abs(angle_increment)
        k0 = math.ceil((bearing - half - angle_min) / angle_increment)
        k1 = math.floor((bearing + half - angle_min) / angle_increment)
        support = free = 0
        for k in range(k0, k1 + 1):
            if full:
                k %= n
            elif not 0 <= k < n:
                continue
            delta = angle_min + k * angle_increment - bearing
            delta = math.atan2(math.sin(delta), math.cos(delta))
            disc = rad * rad - (d * math.sin(delta)) ** 2
            if disc < 0.0:
                continue
            expected = d * math.cos(delta) - math.sqrt(disc)   # near surface along this beam
            r = rs[k]
            if not (np.isfinite(r) and sp.range_min < r < sp.range_max):
                if expected < sp.range_min + self.tp.support_tol:
                    support += 1                          # inside the blind radius
                continue                                  # otherwise a dropout: no evidence
            if abs(r - expected) <= self.tp.support_tol:
                support += 1
            elif r > expected + self.tp.support_tol:
                free += 1                                 # (a nearer return only occludes)
        if free >= self.tp.min_support and free > support:
            return 'free'
        return 'supported' if support >= self.tp.min_support else 'unknown'


# ------------------------------------------------------------------------------ NH-ORCA
@dataclass(frozen=True)
class FilterParams:
    """NH-ORCA filter parameters."""

    safety_margin: float = 0.04        # [m] inflation of a peer's radius (sensing noise + its
    #                                    motion during a stale 5 Hz scan)
    static_margin: float = 0.0         # [m] extra inflation for shelves / walls (see the note)
    tracking_error: float = 0.012      # [m] epsilon: bound on the holonomic tracking error
    heading_time: float = 0.4          # [s] T: time to turn onto a holonomic velocity's heading
    time_horizon_peer: float = 0.8     # [s] ORCA tau for robots (the avoidance is shared)
    time_horizon_static: float = 0.4   # [s] ORCA tau for shelves and walls (full avoidance)
    v_max: float = 0.22                # [m/s]
    w_max: float = 0.60                # [rad/s]
    v_reverse_max: float = 0.05        # [m/s] backwards speed (creep escape only)
    creep_speed: float = 0.05          # [m/s] |u| while no velocity satisfies every half-plane
    static_sector: float = 0.035       # [rad] nearest shelf/wall return kept per sector (~2 deg)
    change_eps: float = 1e-3           # command changes below this are not an intervention


def robot_radius(fp):
    """Radius of the disc about the axle (base_footprint origin) that contains the footprint."""
    return math.hypot(max(abs(fp.x_min), abs(fp.x_max)), max(abs(fp.y_min), abs(fp.y_max)))


def sector_half_angle(fparams):
    """Largest heading change reachable within the heading time: ``w_max * T`` [rad]."""
    return fparams.w_max * fparams.heading_time


def _arc_factor(theta):
    """``f(theta) = (theta/2) / tan(theta/2)``: arc speed / holonomic speed of the tracking law."""
    h = 0.5 * theta
    return 1.0 if abs(h) < 1e-9 else h / math.tan(h)


def tracking_error_bound(speed, theta, fparams):
    """Largest distance between the tracking manoeuvre and the straight holonomic path."""
    return speed * fparams.heading_time * abs(math.sin(0.5 * theta))


def forward_speed_limit(fparams):
    """Radius of the forward sector: every command ``|v| <= v_max`` maps inside it."""
    return fparams.v_max / _arc_factor(sector_half_angle(fparams))


def max_tracking_error(fparams):
    """Worst tracking error over the allowed holonomic velocities (must be <= tracking_error)."""
    return tracking_error_bound(forward_speed_limit(fparams), sector_half_angle(fparams), fparams)


def unicycle_to_holonomic(v, w, fparams):
    """Return the holonomic velocity (robot frame) whose tracking manoeuvre starts with (v, w)."""
    th = sector_half_angle(fparams)
    theta = min(max(w * fparams.heading_time, -th), th)
    speed = abs(v) / _arc_factor(theta)
    sign = 1.0 if v >= 0.0 else -1.0
    return np.array([sign * speed * math.cos(theta), sign * speed * math.sin(theta)])


def holonomic_to_unicycle(u, fparams):
    """Command ``(v, w)`` that tracks the holonomic velocity ``u`` (NH-ORCA tracking law)."""
    speed = math.hypot(u[0], u[1])
    if speed < EPS:
        return 0.0, 0.0
    sign = 1.0 if u[0] >= 0.0 else -1.0           # reverse: drive backwards onto the heading
    th = sector_half_angle(fparams)
    theta = min(max(math.atan2(sign * u[1], sign * u[0]), -th), th)
    return sign * speed * _arc_factor(theta), theta / fparams.heading_time


def orca_line(p, radius, tau, rel_v, u_cur, share, escape=True):
    """
    ORCA half-plane of one obstacle (RVO2 ``Agent::computeNewVelocity``), as an RVO2 line.

    ``p``: obstacle centre relative to the robot; ``radius``: combined radius; ``tau``: time
    horizon; ``rel_v``: v_opt(robot) - v_opt(obstacle); ``u_cur``: the robot's v_opt; ``share``:
    1/2 for a reciprocating robot, 1 for a static obstacle; ``escape``: what "already inside the
    radius" demands (module note (a)). Returns ``(point, direction)``: the allowed velocities lie
    on the left of the directed line.
    """
    px, py = float(p[0]), float(p[1])
    dist_sq = px * px + py * py
    r_sq = radius * radius
    if dist_sq <= r_sq:
        # already inside the radius (module note (a)): move away at >= share * (R - d) / tau
        # (escape), or at least do not come closer; whatever the current velocity
        d = math.sqrt(dist_sq)
        n = (px / d, py / d) if d > EPS else (1.0, 0.0)
        return _half_plane(n, share * (d - radius) / tau if escape else 0.0)
    wx, wy = rel_v[0] - px / tau, rel_v[1] - py / tau
    w_len_sq = wx * wx + wy * wy
    dot1 = wx * px + wy * py
    if dot1 < 0.0 and dot1 * dot1 > r_sq * w_len_sq:
        # project on the cut-off circle
        w_len = math.sqrt(w_len_sq)
        ux, uy = wx / w_len, wy / w_len
        direction = (uy, -ux)
        k = radius / tau - w_len
        u = (k * ux, k * uy)
    else:
        leg = math.sqrt(dist_sq - r_sq)
        if px * wy - py * wx > 0.0:                # left leg
            direction = ((px * leg - py * radius) / dist_sq, (px * radius + py * leg) / dist_sq)
        else:                                      # right leg
            direction = (-(px * leg + py * radius) / dist_sq,
                         -(-px * radius + py * leg) / dist_sq)
        dot2 = rel_v[0] * direction[0] + rel_v[1] * direction[1]
        u = (dot2 * direction[0] - rel_v[0], dot2 * direction[1] - rel_v[1])
    return (u_cur[0] + share * u[0], u_cur[1] + share * u[1]), direction


def line_violation(line, u):
    """How far ``u`` lies on the forbidden side of an RVO2 line (<= 0: allowed)."""
    (px, py), (dx, dy) = line
    return dx * (py - u[1]) - dy * (px - u[0])


# ------------------------------------------------------------------------------ 2-D QP
def _det(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _lp1(lines, i, radius, opt, direction_opt, result):
    """Optimise on line ``i`` subject to lines[:i] and the speed disc (RVO2 linearProgram1)."""
    pt, dr = lines[i]
    dot = pt[0] * dr[0] + pt[1] * dr[1]
    disc = dot * dot + radius * radius - (pt[0] * pt[0] + pt[1] * pt[1])
    if disc < 0.0:
        return False
    s = math.sqrt(disc)
    t_left, t_right = -dot - s, -dot + s
    for j in range(i):
        pj, dj = lines[j]
        den = _det(dr, dj)
        num = _det(dj, (pt[0] - pj[0], pt[1] - pj[1]))
        if abs(den) <= EPS:
            if num < 0.0:
                return False
            continue
        t = num / den
        if den >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return False
    if direction_opt:
        t = t_right if (opt[0] * dr[0] + opt[1] * dr[1]) > 0.0 else t_left
    else:
        t = dr[0] * (opt[0] - pt[0]) + dr[1] * (opt[1] - pt[1])
        t = min(max(t, t_left), t_right)
    result[0], result[1] = pt[0] + t * dr[0], pt[1] + t * dr[1]
    return True


def _lp2(lines, radius, opt, direction_opt, result):
    """Closest point to ``opt`` satisfying all lines (RVO2 linearProgram2); returns fail index."""
    if direction_opt:
        result[0], result[1] = opt[0] * radius, opt[1] * radius
    elif opt[0] * opt[0] + opt[1] * opt[1] > radius * radius:
        k = radius / math.hypot(opt[0], opt[1])
        result[0], result[1] = opt[0] * k, opt[1] * k
    else:
        result[0], result[1] = opt[0], opt[1]
    for i, (pt, dr) in enumerate(lines):
        if _det(dr, (pt[0] - result[0], pt[1] - result[1])) > 0.0:
            backup = list(result)
            if not _lp1(lines, i, radius, opt, direction_opt, result):
                result[0], result[1] = backup
                return i
    return len(lines)


def _lp3(lines, n_hard, begin, radius, result):
    """Minimise the largest violation of the soft lines, keeping the hard ones (RVO2 LP3)."""
    distance = 0.0
    for i in range(begin, len(lines)):
        pt, dr = lines[i]
        if _det(dr, (pt[0] - result[0], pt[1] - result[1])) > distance:
            proj = list(lines[:n_hard])
            for j in range(n_hard, i):
                pj, dj = lines[j]
                den = _det(dr, dj)
                if abs(den) <= EPS:
                    if dr[0] * dj[0] + dr[1] * dj[1] > 0.0:
                        continue            # same direction: j is implied by i
                    p = (0.5 * (pt[0] + pj[0]), 0.5 * (pt[1] + pj[1]))
                else:
                    t = _det(dj, (pt[0] - pj[0], pt[1] - pj[1])) / den
                    p = (pt[0] + t * dr[0], pt[1] + t * dr[1])
                d = (dj[0] - dr[0], dj[1] - dr[1])
                nd = math.hypot(*d)
                if nd <= EPS:
                    continue
                proj.append((p, (d[0] / nd, d[1] / nd)))
            backup = list(result)
            if _lp2(proj, radius, (-dr[1], dr[0]), True, result) < len(proj):
                result[0], result[1] = backup  # numerical corner case: keep the last result
            pt_r = (pt[0] - result[0], pt[1] - result[1])
            distance = _det(dr, pt_r)
    return result


def _half_plane(a, b):
    """Return ``a . p <= b`` as an RVO2 line (point, direction), feasible side on the left."""
    na = math.hypot(a[0], a[1])
    n = (a[0] / na, a[1] / na)
    c = b / na
    return ((n[0] * c, n[1] * c), (-n[1], n[0]))


def _sectors(fparams):
    """Hard lines of the forward and reverse P^AHV sectors, with their speed radii."""
    th = sector_half_angle(fparams)
    c, s_ = math.cos(th), math.sin(th)
    forward = [_half_plane((-s_, c), 0.0), _half_plane((-s_, -c), 0.0)]
    reverse = [_half_plane((s_, -c), 0.0), _half_plane((s_, c), 0.0)]
    return ((forward, forward_speed_limit(fparams)),
            (reverse, fparams.v_reverse_max / _arc_factor(th)))


def nh_orca_velocity(u_pref, lines, fparams):
    """
    Return the allowed holonomic velocity inside every ORCA line that is closest to ``u_pref``.

    Solved on the forward and on the reverse sector (P^AHV is their union; each is convex) with
    RVO2's incremental linear program; the closer feasible solution wins. If neither sector has a
    feasible velocity, the one of the creep-limited sectors that minimises the largest violation
    is returned (RVO2 linearProgram3). Returns ``(u, feasible)``.
    """
    best = None
    for hard, radius in _sectors(fparams):
        soft = [ln for ln in lines if line_violation(ln, (0.0, 0.0)) + radius > 0.0]
        result = [0.0, 0.0]
        if _lp2(hard + soft, radius, (float(u_pref[0]), float(u_pref[1])), False,
                result) >= len(hard) + len(soft):
            d = math.hypot(result[0] - u_pref[0], result[1] - u_pref[1])
            if best is None or d < best[0]:
                best = (d, result)
    if best is not None:
        return np.array(best[1]), True
    # creep escape: least worst violation within slow sectors (no full-speed corner solutions)
    best = None
    for hard, radius in _sectors(fparams):
        radius = min(radius, fparams.creep_speed)
        soft = [ln for ln in lines if line_violation(ln, (0.0, 0.0)) + radius > 0.0]
        result = [0.0, 0.0]
        _lp3(hard + soft, len(hard), 0, radius, result)
        worst = max((line_violation(ln, result) for ln in soft), default=0.0)
        if best is None or worst < best[0]:
            best = (worst, result)
    return np.array(best[1]), False


# ------------------------------------------------------------------------------ the filter
@dataclass
class Decision:
    """One filter decision."""

    v: float
    w: float
    active: bool              # the command was changed (or stopped for lack of lidar)
    feasible: bool
    min_obstacle_dist: float  # sensed clearance: footprint to nearest valid return [m]
    min_barrier: float        # smallest d - R over the obstacles [m] (< 0: inside a radius)
    n_blobs: int


def obstacles_from_scan(pts, idx, blob_centres, blob_beams, sp, fparams):
    """
    Obstacles ``(statics, peers)`` in the robot frame.

    ``pts`` / ``idx``: the returns (``filtered_scan_points``) and their beam indices. Robot-like
    blobs (detected or remembered) are the peers; the returns of the beams a blob was detected on
    are replaced by it (a remembered peer keeps its returns as statics too). Of the other returns
    within ``max_range_used`` only the nearest per ``static_sector`` is kept: neighbouring
    returns of one shelf face give nearly the same half-plane.
    """
    peers = np.asarray(blob_centres, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return np.zeros((0, 2)), peers
    in_blob = np.isin(np.asarray(idx), np.concatenate(blob_beams)) if len(blob_beams) else \
        np.zeros(len(pts), dtype=bool)
    rng = np.hypot(pts[:, 0], pts[:, 1])
    keep = ~in_blob & (rng <= sp.max_range_used)
    statics, rng = pts[keep], rng[keep]
    if len(statics):
        sector = np.floor((np.arctan2(statics[:, 1], statics[:, 0]) + math.pi)
                          / fparams.static_sector).astype(int)
        order = np.lexsort((rng, sector))
        first = np.ones(len(order), dtype=bool)
        first[1:] = sector[order][1:] != sector[order][:-1]
        statics = statics[order[first]]
    return statics, peers


def filter_command(nominal, u_cur, statics, peers, clearance, fp, fparams, peer_radius):
    """
    Apply NH-ORCA to ``nominal = (v, w)`` given the obstacles (robot frame, current pose).

    ``u_cur``: the robot's current holonomic velocity (its v_opt).
    """
    disc = robot_radius(fp) + fparams.tracking_error
    r_static = disc + fparams.static_margin
    r_peer = disc + peer_radius + fparams.safety_margin
    lines, h = [], []
    # module note (a): inside a shelf's radius, translating steers out of it; turning in place
    # (or stopping) must only not come closer
    translating = abs(nominal[0]) > fparams.change_eps
    zero = (0.0, 0.0)                      # v_opt of shelves and walls: module note (c)
    for p in np.asarray(statics, dtype=float).reshape(-1, 2):
        lines.append(orca_line(p, r_static, fparams.time_horizon_static, zero, zero, 1.0,
                               escape=translating))
        h.append(math.hypot(p[0], p[1]) - r_static)
    for p in np.asarray(peers, dtype=float).reshape(-1, 2):
        lines.append(orca_line(p, r_peer, fparams.time_horizon_peer, u_cur, u_cur, 0.5))
        h.append(math.hypot(p[0], p[1]) - r_peer)
    u_nom = unicycle_to_holonomic(nominal[0], nominal[1], fparams)
    n_peers = len(np.asarray(peers).reshape(-1, 2))
    min_h = float(min(h)) if h else math.inf
    if all(line_violation(ln, u_nom) <= 1e-9 for ln in lines):
        return Decision(v=nominal[0], w=nominal[1], active=False, feasible=True,
                        min_obstacle_dist=clearance, min_barrier=min_h, n_blobs=n_peers)
    u, feasible = nh_orca_velocity(u_nom, lines, fparams)
    v, w = holonomic_to_unicycle(u, fparams)
    standing_ok = all(line_violation(ln, (0.0, 0.0)) <= 1e-9 for ln in lines)
    if math.hypot(u[0], u[1]) < 1e-6 and standing_ok:
        w = nominal[1]                        # module note (b): rotation about the axle is free
    v = min(max(v, -fparams.v_reverse_max), fparams.v_max)
    w = min(max(w, -fparams.w_max), fparams.w_max)
    active = abs(v - nominal[0]) > fparams.change_eps or abs(w - nominal[1]) > fparams.change_eps
    return Decision(v=v, w=w, active=active, feasible=feasible, min_obstacle_dist=clearance,
                    min_barrier=min_h, n_blobs=n_peers)


def sensed_clearance(pts, fp):
    """Smallest distance from the footprint to any valid return (inf when there is none)."""
    if len(pts) == 0:
        return math.inf
    d, _ = footprint_distance(pts, fp)
    return float(np.min(d))


def transform_points(pts, dx, dy, dyaw):
    """Express points given in an old robot frame in a new one moved by ``(dx, dy, dyaw)``."""
    p = np.asarray(pts, dtype=float).reshape(-1, 2) - np.array([dx, dy])
    c, s = math.cos(-dyaw), math.sin(-dyaw)
    return np.stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]], axis=1)
