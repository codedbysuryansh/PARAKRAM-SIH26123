"""
Unit tests for the NH-ORCA safety filter math (orca_core, CLAUDE_CODE/03).

The encounter tests close the loop: every robot ray-casts its own lidar scan (5 Hz, 1 degree,
0.12 m blind radius) against the other robots' lidar TURRETS (the only part of a Burger in the
scan plane) and the walls, runs the filter at 20 Hz on its own scans, and the result is judged
by the ground-truth footprint geometry of the 01/02 collision checker (parakram_sim.footprint).
"""

import math

import numpy as np
from parakram_safety import orca_core as oc
from parakram_sim.footprint import footprint_polygon, polygon_distance
import pytest

FP = oc.Footprint()
SP = oc.ScanParams()
FPAR = oc.FilterParams()
LIDAR = (-0.032, 0.0, 0.0)          # lidar pose in base_footprint (Burger)
TURRET_R = 0.0508


# ------------------------------------------------------------------------------ helpers
NOISE = 0.02                        # lidar range noise sigma [m] (fleet_sim lidar_noise_std)
RNG = np.random.default_rng(2026)


def ray_scan(pose, discs, segments, n=360, noise=NOISE):
    """Ranges seen by a lidar at ``pose`` (world) of discs (cx, cy, r) and wall segments."""
    x, y, th = pose
    lx, ly = x + LIDAR[0] * math.cos(th), y + LIDAR[0] * math.sin(th)
    ang = -math.pi + np.arange(n) * 2 * math.pi / n + th
    dx, dy = np.cos(ang), np.sin(ang)
    best = np.full(n, np.inf)
    for cx, cy, r in discs:
        ox, oy = cx - lx, cy - ly
        b = ox * dx + oy * dy
        disc = b * b - (ox * ox + oy * oy - r * r)
        t = b - np.sqrt(np.maximum(disc, 0.0))
        hit = (disc >= 0) & (t > 0)
        best = np.where(hit, np.minimum(best, t), best)
    for x0, y0, x1, y1 in segments:
        ex, ey = x1 - x0, y1 - y0
        wx, wy = x0 - lx, y0 - ly
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (wx * ey - wy * ex) / den
            u = (wx * dy - wy * dx) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit, np.minimum(best, t), best)
    if noise:
        best = best + RNG.normal(0.0, noise, n)
    best[(best <= SP.range_min) | (best >= SP.range_max)] = np.inf
    return best


def turret(pose):
    x, y, th = pose
    return (x + LIDAR[0] * math.cos(th), y + LIDAR[0] * math.sin(th), TURRET_R)


def decide(pose, others, segments, nominal, fparams=FPAR, n=360, noise=NOISE, current=None):
    """
    One filter decision for a robot at ``pose`` from its own (instantaneous) scan.

    ``current``: the robot's current (v, w), NH-ORCA's v_opt; default: already moving as the
    nominal asks.
    """
    ranges = ray_scan(pose, [turret(o) for o in others], segments, n, noise)
    inc = 2 * math.pi / n
    pts, idx, _ = oc.filtered_scan_points(ranges, -math.pi, inc, LIDAR, SP)
    centres, beams = oc.find_blobs(ranges, -math.pi, inc, LIDAR, SP)
    statics, peers = oc.obstacles_from_scan(pts, idx, centres, beams, SP, fparams)
    u_cur = oc.unicycle_to_holonomic(*(current or nominal), fparams)
    return oc.filter_command(nominal, u_cur, statics, peers, oc.sensed_clearance(pts, FP), FP,
                             fparams, SP.peer_radius)


def gt_gap(a, b):
    return polygon_distance(footprint_polygon(*a), footprint_polygon(*b))


def simulate(poses, nominals, segments=(), seconds=12.0, fparams=FPAR, scan_period=0.2,
             dt=0.05, memory=True):
    """
    Run the closed loop with unicycle kinematics, as the node does.

    Each robot filters its nominal command on its OWN latest 5 Hz scan (its own motion since the
    scan compensated, the peers' motion NOT), with the peer memory fed by exact odometry.
    """
    poses = [list(p) for p in poses]
    k_scan = int(round(scan_period / dt))
    trackers = [oc.PeerTracker() for _ in poses]
    current = [(0.0, 0.0)] * len(poses)          # each robot's last command = its velocity
    scans = [None] * len(poses)
    min_gap, events = math.inf, []
    inc = 2 * math.pi / 360
    for k in range(int(seconds / dt)):
        if k % k_scan == 0:
            for i, p in enumerate(poses):
                others = [q for j, q in enumerate(poses) if j != i]
                ranges = ray_scan(p, [turret(o) for o in others], segments)
                pts, idx, _ = oc.filtered_scan_points(ranges, -math.pi, inc, LIDAR, SP)
                centres, beams = oc.find_blobs(ranges, -math.pi, inc, LIDAR, SP)
                centres, beams = trackers[i].update(k * dt, tuple(p) if memory else None,
                                                    centres, beams, ranges, -math.pi, inc,
                                                    LIDAR, SP)
                scans[i] = (list(p), pts, idx, centres, beams)
        cmds = []
        for i, p in enumerate(poses):
            p0, pts, idx, centres, beams = scans[i]
            # own motion since the scan (odometry): express scan points in the current frame
            c0, s0 = math.cos(p0[2]), math.sin(p0[2])
            ddx, ddy = p[0] - p0[0], p[1] - p0[1]
            dx, dy = c0 * ddx + s0 * ddy, -s0 * ddx + c0 * ddy
            dth = p[2] - p0[2]
            pts = oc.transform_points(pts, dx, dy, dth)
            centres = oc.transform_points(centres, dx, dy, dth)
            statics, peers = oc.obstacles_from_scan(pts, idx, centres, beams, SP, fparams)
            u_cur = oc.unicycle_to_holonomic(*current[i], fparams)
            d = oc.filter_command(nominals[i], u_cur, statics, peers,
                                  oc.sensed_clearance(pts, FP), FP, fparams, SP.peer_radius)
            cmds.append(d)
        current = [(d.v, d.w) for d in cmds]
        for p, d in zip(poses, cmds):
            p[0] += d.v * math.cos(p[2]) * dt
            p[1] += d.v * math.sin(p[2]) * dt
            p[2] += d.w * dt
        for i in range(len(poses)):
            for j in range(i + 1, len(poses)):
                min_gap = min(min_gap, gt_gap(poses[i], poses[j]))
        events.append(cmds)
    return poses, min_gap, events


AISLE = [(-3.0, 0.2, 3.0, 0.2), (-3.0, -0.2, 3.0, -0.2)]      # a 0.40 m aisle along x


# ------------------------------------------------------------------------------ geometry
def test_footprint_distance_outside_inside_and_normals():
    d, n = oc.footprint_distance([[0.14, 0.0], [0.0, 0.29], [-0.205, -0.19], [0.0, 0.0]], FP)
    assert d[:3] == pytest.approx([0.10, 0.20, math.hypot(0.1, 0.1)])
    assert n[0] == pytest.approx([1.0, 0.0]) and n[1] == pytest.approx([0.0, 1.0])
    assert d[3] == 0.0 and np.linalg.norm(n[3]) == pytest.approx(1.0)


# ------------------------------------------------------------------------------ NH-ORCA core
def test_tracking_law_round_trips_and_keeps_the_error_bound():
    """Every command maps to an allowed holonomic velocity and is tracked within epsilon."""
    assert oc.robot_radius(FP) == pytest.approx(math.hypot(0.105, 0.090))
    assert oc.max_tracking_error(FPAR) <= FPAR.tracking_error
    rng = np.random.default_rng(3)
    th = oc.sector_half_angle(FPAR)
    for _ in range(500):
        v, w = rng.uniform(-FPAR.v_reverse_max, FPAR.v_max), rng.uniform(-FPAR.w_max, FPAR.w_max)
        u = oc.unicycle_to_holonomic(v, w, FPAR)
        assert oc.holonomic_to_unicycle(u, FPAR) == pytest.approx((v, w) if v else (0.0, 0.0))
        ang = math.atan2(u[1], u[0]) if v >= 0 else math.atan2(-u[1], -u[0])
        assert abs(ang) <= th + 1e-9 and math.hypot(*u) <= oc.forward_speed_limit(FPAR) + 1e-9

    def deviation(u, dt=0.001):
        # the manoeuvre: arc (v, w) for T, then straight at |u|; vs the holonomic path u * t
        v, w = oc.holonomic_to_unicycle(u, FPAR)
        speed, sign = math.hypot(*u), (1.0 if u[0] >= 0 else -1.0)
        x = y = a = dev = 0.0
        for k in range(int(3 * FPAR.heading_time / dt)):
            t = (k + 1) * dt
            step = v if t <= FPAR.heading_time else sign * speed
            x, y = x + step * math.cos(a) * dt, y + step * math.sin(a) * dt
            a += w * dt if t <= FPAR.heading_time else 0.0
            dev = max(dev, math.hypot(x - u[0] * t, y - u[1] * t))
        return dev

    for ang in np.linspace(-th, th, 7):
        for sp in (oc.forward_speed_limit(FPAR), 0.08, -FPAR.v_reverse_max):
            u = sp * np.array([math.cos(ang), math.sin(ang)])
            assert deviation(u) <= FPAR.tracking_error + 1e-4, (ang, sp)


def test_orca_half_planes_are_collision_free_within_the_horizon():
    """Static: full avoidance; robots: shared avoidance with consistent v_opt (ORCA theorem)."""
    rng = np.random.default_rng(11)
    ts = np.linspace(0.0, 1.0, 60)
    tested = 0
    for _ in range(300):
        p = rng.uniform(-0.8, 0.8, 2)
        if math.hypot(*p) <= 0.18:
            continue
        u_cur = rng.uniform(-0.22, 0.22, 2)
        line = oc.orca_line(p, 0.17, 0.4, u_cur, u_cur, 1.0)
        for v in rng.uniform(-0.3, 0.3, (30, 2)):
            if oc.line_violation(line, v) <= 0.0:
                tested += 1
                d = np.hypot(p[0] - v[0] * 0.4 * ts, p[1] - v[1] * 0.4 * ts)
                assert d.min() >= 0.17 - 1e-9
    for _ in range(300):
        p = rng.uniform(-0.8, 0.8, 2)
        if math.hypot(*p) <= 0.31:
            continue
        va, vb = rng.uniform(-0.2, 0.2, 2), rng.uniform(-0.2, 0.2, 2)
        la = oc.orca_line(p, 0.3, 0.8, va - vb, va, 0.5)
        lb = oc.orca_line(-p, 0.3, 0.8, vb - va, vb, 0.5)
        for a, b in zip(rng.uniform(-0.3, 0.3, (30, 2)), rng.uniform(-0.3, 0.3, (30, 2))):
            if oc.line_violation(la, a) <= 0.0 and oc.line_violation(lb, b) <= 0.0:
                tested += 1
                rel = p[:, None] + (b - a)[:, None] * 0.8 * ts
                assert np.hypot(*rel).min() >= 0.3 - 1e-9
    assert tested > 5000


def brute_force(u_pref, lines, fparams, radius_cap=None, n=241):
    """Grid both sectors: return (points, worst violation, distance to u_pref)."""
    th = oc.sector_half_angle(fparams)
    pts = []
    for sign, rmax in ((1.0, oc.forward_speed_limit(fparams)),
                       (-1.0, fparams.v_reverse_max / oc._arc_factor(th))):
        rmax = min(rmax, radius_cap) if radius_cap else rmax
        for r in np.linspace(0.0, rmax, n // 4):
            for a in np.linspace(-th, th, n // 4):
                pts.append((sign * r * math.cos(a), sign * r * math.sin(a)))
    pts = np.array(pts)
    worst = np.max([[oc.line_violation(ln, q) for ln in lines] for q in pts], axis=1)
    return pts, worst, np.hypot(*(pts - u_pref).T)


def test_linear_program_is_optimal_against_brute_force():
    rng = np.random.default_rng(7)
    feasible = infeasible = 0
    for k in range(120):
        lines = []
        for _ in range(rng.integers(1, 5)):
            ang = rng.uniform(-math.pi, math.pi)
            n = np.array([math.cos(ang), math.sin(ang)])
            off = rng.uniform(-0.12 if k % 3 == 0 else -0.03, 0.2)
            lines.append(oc._half_plane(n, off))        # n . u <= off
        u_pref = oc.unicycle_to_holonomic(rng.uniform(-0.05, 0.22), rng.uniform(-0.6, 0.6), FPAR)
        u, ok = oc.nh_orca_velocity(u_pref, lines, FPAR)
        pts, worst, dist = brute_force(u_pref, lines, FPAR)
        if (worst <= 1e-9).any():
            assert ok
            feasible += 1
            assert max(oc.line_violation(ln, u) for ln in lines) <= 1e-7
            assert math.hypot(*(u - u_pref)) <= dist[worst <= 1e-9].min() + 3e-3   # grid
        else:
            assert not ok
            infeasible += 1
            assert math.hypot(*u) <= FPAR.creep_speed + 1e-9          # never a fast escape
            _, worst_c, _ = brute_force(u_pref, lines, FPAR, radius_cap=FPAR.creep_speed)
            got = max(oc.line_violation(ln, u) for ln in lines)
            assert got <= worst_c.min() + 3e-3
    assert feasible > 30 and infeasible > 10, (feasible, infeasible)


def test_unconstrained_nominal_passes_unchanged():
    for nominal in ((0.18, 0.3), (0.0, -0.6), (-0.03, 0.2)):
        d = oc.filter_command(nominal, np.zeros(2), np.zeros((0, 2)), np.zeros((0, 2)),
                              math.inf, FP, FPAR, SP.peer_radius)
        assert (d.v, d.w, d.active) == (nominal[0], nominal[1], False)


# ------------------------------------------------------------------------------ lidar blobs
def test_blob_detection_finds_a_turret_but_not_walls_or_corners():
    me = (0.0, 0.0, 0.0)
    peer = (0.8, 0.02, math.pi)
    ranges = ray_scan(me, [turret(peer)], AISLE)
    centres, _ = oc.find_blobs(ranges, -math.pi, 2 * math.pi / 360, LIDAR, SP)
    assert len(centres) == 1
    tx, ty, _ = turret(peer)
    assert centres[0] == pytest.approx([tx, ty], abs=0.03)
    # walls only; a shelf corner (L-shaped wall ending in the field of view)
    for segs in (AISLE, [(0.3, 0.2, 1.0, 0.2), (0.3, 0.2, 0.3, 1.0), (-2, -0.2, 2, -0.2)]):
        for _ in range(20):                                          # noisy scans
            c, _ = oc.find_blobs(ray_scan(me, [], segs), -math.pi, 2 * math.pi / 360, LIDAR, SP)
            assert len(c) == 0


def test_blob_detection_is_robust_to_range_noise_with_peers_close_together():
    """Three robots jammed at a junction (the 03 acceptance situation), 0.02 m range noise."""
    junction = [(-3, 0.2, -0.2, 0.2), (0.2, 0.2, 3, 0.2), (-3, -0.2, -0.2, -0.2),
                (0.2, -0.2, 3, -0.2), (-0.2, 0.2, -0.2, 3), (0.2, 0.2, 0.2, 3),
                (-0.2, -0.2, -0.2, -3), (0.2, -0.2, 0.2, -3)]
    poses = [(-0.11, 0.01, 0.0), (0.12, 0.01, math.pi), (0.02, -0.18, math.pi / 2)]
    found = missed = extra = 0
    for _ in range(100):
        for i, me in enumerate(poses):
            others = [q for j, q in enumerate(poses) if j != i]
            ranges = ray_scan(me, [turret(o) for o in others], junction)
            centres, _ = oc.find_blobs(ranges, -math.pi, 2 * math.pi / 360, LIDAR, SP)
            c, s = math.cos(-me[2]), math.sin(-me[2])
            truth = []
            for o in others:
                tx, ty, _ = turret(o)
                dx, dy = tx - me[0], ty - me[1]
                truth.append((c * dx - s * dy, s * dx + c * dy))
            hit = [any(math.hypot(cx - tx, cy - ty) < 0.06 for cx, cy in centres)
                   for tx, ty in truth]
            found += sum(hit)
            missed += len(hit) - sum(hit)
            extra += max(0, len(centres) - sum(hit))
    assert found / (found + missed) >= 0.99, (found, missed)
    assert extra <= 3, extra


# ------------------------------------------------------------------------------ peer memory
def track(tracker, t, me, peers, segments, detect=True, n=360):
    """One scan of ``me`` through the tracker; ``detect=False`` simulates a detection miss."""
    ranges = ray_scan(me, [turret(o) for o in peers], segments, n)
    inc = 2 * math.pi / n
    centres, beams = oc.find_blobs(ranges, -math.pi, inc, LIDAR, SP)
    if not detect:
        centres, beams = np.zeros((0, 2)), []
    return tracker.update(t, me, centres, beams, ranges, -math.pi, inc, LIDAR, SP)


def test_peer_memory_bridges_missed_detections():
    me, peer = (0.0, 0.0, 0.0), (0.6, 0.02, math.pi)
    tracker = oc.PeerTracker()
    centres, beams = track(tracker, 0.0, me, [peer], AISLE)
    assert len(centres) == 1 and len(beams[0]) > 0
    tx, ty, _ = turret(peer)
    for k in range(1, 20):                      # detection keeps failing, the peer is still there
        centres, beams = track(tracker, 0.2 * k, me, [peer], AISLE, detect=False)
        assert len(centres) == 1, k             # remembered: returns still where its turret is
        assert centres[0] == pytest.approx([tx, ty], abs=0.03)
        assert len(beams[0]) == 0               # its own returns stay obstacles as well
        if 0.2 * k > oc.TrackParams().hold_max:
            break
    # never longer than hold_max after the last real detection
    centres, _ = track(tracker, oc.TrackParams().hold_max + 0.2, me, [peer], AISLE, detect=False)
    assert len(centres) == 0


def test_peer_memory_forgets_a_departed_peer_at_once_and_follows_own_motion():
    me, peer = (0.0, 0.0, 0.0), (0.6, 0.02, math.pi)
    tracker = oc.PeerTracker()
    track(tracker, 0.0, me, [peer], AISLE)
    centres, _ = track(tracker, 0.2, me, [], AISLE, detect=False)      # the peer has gone
    assert len(centres) == 0                    # the scan sees through where it stood
    # own motion: a peer remembered in the odom frame stays put in the world
    tracker = oc.PeerTracker()
    track(tracker, 0.0, me, [peer], AISLE)
    moved = (0.1, 0.0, 0.2)
    centres, _ = track(tracker, 0.2, moved, [peer], AISLE, detect=False)
    tx, ty, _ = turret(peer)
    c, s = math.cos(-moved[2]), math.sin(-moved[2])
    want = (c * (tx - moved[0]) - s * (ty - moved[1]), s * (tx - moved[0]) + c * (ty - moved[1]))
    assert len(centres) == 1 and centres[0] == pytest.approx(want, abs=0.03)


def test_peer_memory_expires_without_evidence_and_holds_in_the_blind_radius():
    tp = oc.TrackParams()
    rs = np.full(360, np.inf)                   # nothing returned at all (e.g. dropouts)
    inc = 2 * math.pi / 360
    tracker = oc.PeerTracker(tp)
    far = np.array([[0.8, 0.0]])
    tracker.update(0.0, (0.0, 0.0, 0.0), far, [np.arange(3)], rs, -math.pi, inc, LIDAR, SP)
    c, _ = tracker.update(tp.memory - 0.1, (0.0, 0.0, 0.0), np.zeros((0, 2)), [], rs, -math.pi,
                          inc, LIDAR, SP)
    assert len(c) == 1                          # no evidence either way: kept for ``memory``
    c, _ = tracker.update(tp.memory + 0.1, (0.0, 0.0, 0.0), np.zeros((0, 2)), [], rs,
                          -math.pi, inc, LIDAR, SP)
    assert len(c) == 0
    # a turret inside the lidar's blind radius returns nothing either, but may well be there
    near = np.array([[LIDAR[0] + SP.range_min + SP.turret_radius - 0.02, 0.0]])
    tracker = oc.PeerTracker(tp)
    tracker.update(0.0, (0.0, 0.0, 0.0), near, [np.arange(3)], rs, -math.pi, inc, LIDAR, SP)
    for t in np.arange(0.2, tp.hold_max, 0.2):
        c, _ = tracker.update(t, (0.0, 0.0, 0.0), np.zeros((0, 2)), [], rs, -math.pi, inc,
                              LIDAR, SP)
        assert len(c) == 1, t


def test_three_robot_junction_is_contact_free_across_noise_realisations():
    """Noise seeds that produced contact before the peer memory existed (seed sweep, 03)."""
    starts = [(-1.2, 0.03, 0.0), (1.2, -0.04, math.pi), (-0.1, -1.2, math.pi / 2)]
    walls = [(-3, 0.2, -0.2, 0.2), (0.2, 0.2, 3, 0.2), (-3, -0.2, -0.2, -0.2),
             (0.2, -0.2, 3, -0.2), (-0.2, -0.2, -0.2, -3), (0.2, -0.2, 0.2, -3)]
    global RNG
    saved = RNG
    try:
        for seed in (4, 6, 8, 13, 18):
            RNG = np.random.default_rng(seed)
            _, gap, _ = simulate(starts, [(0.18, 0.0)] * 3, walls, seconds=16.0)
            assert gap > 0.01, (seed, gap)
    finally:
        RNG = saved


# ------------------------------------------------------------------------------ no freezing
def test_aisle_walls_leave_centred_driving_and_turning_free():
    """
    NH-ORCA's disc (0.150 m about the axle) against the 0.40 m aisle walls.

    Without range noise nothing is changed while the disc clears the walls (|offset| < 0.049 m);
    at 0.05 m off-centre the disc touches the near wall and the filter steers back towards the
    centre. With 0.02 m noise a centred robot is trimmed only slightly; off-centre it is nudged
    back. Measured over 150 noisy scans per case (see the CLAUDE_CODE/03 report): centred, 4 % of
    the driving commands changed, by <= 0.017 m/s, turns never; 0.02 m off-centre 62 % changed
    (median speed change 0.001 m/s, steering 0.09 rad/s); a robot started 0.03 m off-centre keeps
    97 % of its progress over 10 s (1.75 of 1.80 m).
    """
    nominals = ((0.18, 0.0), (0.0, 0.6), (0.0, -0.6), (0.1, 0.3))
    for dy in (0.0, 0.03, -0.03):
        for nominal in nominals:
            assert not decide((0.0, dy, 0.0), [], AISLE, nominal, noise=0.0).active, (dy, nominal)
    d = decide((0.0, -0.05, 0.0), [], AISLE, (0.18, 0.0), noise=0.0)
    assert d.active and d.w > 0.0 and d.v > 0.15    # steered back to the centre, still driving
    global RNG
    saved, RNG = RNG, np.random.default_rng(5)
    try:
        for nominal in nominals:
            ds = [decide((0.0, 0.0, 0.0), [], AISLE, nominal) for _ in range(100)]
            assert sum(d.active for d in ds) <= 10, nominal
            assert max(abs(d.v - nominal[0]) for d in ds) <= 0.03, nominal
            if nominal[0] == 0.0:                   # turning in place: never changed
                assert not any(d.active for d in ds), nominal
        RNG = np.random.default_rng(9)
        poses, _, _ = simulate([(-1.0, 0.03, 0.0)], [(0.18, 0.0)], AISLE, seconds=10.0)
        assert poses[0][0] + 1.0 >= 0.9 * 1.8, poses
    finally:
        RNG = saved


def test_in_place_turns_near_a_shelf_are_never_overridden():
    """
    A shelf inside the disc (it over-covers the Burger behind the axle) does not block a turn.

    Wall 0.13 m behind the axle: 0.025 m behind the rear edge, inside the 0.150 m disc. Turning in
    place passes unchanged (module note (a)); backing into it is stopped; driving away passes.
    """
    wall = [(-0.13, -1.0, -0.13, 1.0)]
    for w in (0.6, -0.6, 0.2):
        d = decide((0.0, 0.0, 0.0), [], wall, (0.0, w), noise=0.0)
        assert (d.v, d.w, d.active) == (0.0, w, False)
    d = decide((0.0, 0.0, 0.0), [], wall, (-0.05, 0.0), noise=0.0)
    assert d.active and d.v >= 0.0                 # does not back into the shelf
    assert not decide((0.0, 0.0, 0.0), [], wall, (0.18, 0.0), noise=0.0).active


def test_turning_in_a_junction_is_free():
    walls = [(-3, 0.2, -0.2, 0.2), (0.2, 0.2, 3, 0.2), (-3, -0.2, -0.2, -0.2),
             (0.2, -0.2, 3, -0.2), (-0.2, 0.2, -0.2, 3), (0.2, 0.2, 0.2, 3),
             (-0.2, -0.2, -0.2, -3), (0.2, -0.2, 0.2, -3)]
    for th in np.linspace(0, 2 * math.pi, 13):
        assert not decide((0.0, 0.0, th), [], walls, (0.0, 0.6)).active


# ------------------------------------------------------------------------------ encounters
def test_head_on_encounter_is_collision_free():
    poses, gap, events = simulate([(-1.0, 0.0, 0.0), (1.0, 0.0, math.pi)],
                                  [(0.18, 0.0), (0.18, 0.0)], AISLE)
    assert gap > 0.02                                     # never touched (ground truth)
    assert any(d.active for cmds in events for d in cmds)  # the filter acted
    assert poses[0][0] < poses[1][0]                      # they did not pass through


def test_head_on_with_lateral_offset_and_three_robots():
    starts = [(-1.2, 0.03, 0.0), (1.2, -0.04, math.pi), (-0.1, -1.2, math.pi / 2)]
    walls = [(-3, 0.2, -0.2, 0.2), (0.2, 0.2, 3, 0.2), (-3, -0.2, -0.2, -0.2),
             (0.2, -0.2, 3, -0.2), (-0.2, -0.2, -0.2, -3), (0.2, -0.2, 0.2, -3)]
    _, gap, _ = simulate(starts, [(0.18, 0.0)] * 3, walls, seconds=16.0)
    assert gap > 0.01


def test_crossing_encounter_is_collision_free():
    _, gap, events = simulate([(-1.0, 0.0, 0.0), (0.0, -1.0, math.pi / 2)],
                              [(0.18, 0.0), (0.18, 0.0)], (), seconds=14.0)
    assert gap > 0.01
    assert any(d.active for cmds in events for d in cmds)


def test_silent_static_obstacle_is_avoided_without_any_message():
    # a stopped / dead / silent robot straight ahead: its turret is just a sensed obstacle
    dead = (0.6, 0.0, 0.0)
    me = [-0.8, 0.0, 0.0]
    for _ in range(int(12 / 0.05)):
        d = decide(tuple(me), [dead], AISLE, (0.18, 0.0))
        me[0] += d.v * math.cos(me[2]) * 0.05
    assert gt_gap(tuple(me), dead) > 0.02


def test_margin_inflation_increases_clearance_monotonically():
    """
    Head-on in an aisle: the clearance grows with ``safety_margin``.

    Checked on the clearance NH-ORCA controls, disc to disc (my axle to the peer's turret), and on
    the body gap overall. (At the standoff the robots turn back and forth, which moves the body gap
    by ~1 cm: it is not exactly monotonic under every noise realisation, the disc clearance is.)
    """
    global RNG, gt_gap
    saved_rng, saved_gap = RNG, gt_gap
    discs, gaps = [], []
    try:
        for margin in (0.0, 0.02, 0.04, 0.06, 0.08, 0.10):
            closest = [math.inf]

            def body_gap(a, b, closest=closest):
                for me, other in ((a, b), (b, a)):
                    tx, ty, _ = turret(other)
                    closest[0] = min(closest[0], math.hypot(tx - me[0], ty - me[1]))
                return saved_gap(a, b)
            gt_gap, RNG = body_gap, np.random.default_rng(2026)
            _, gap, _ = simulate([(-1.0, 0.0, 0.0), (1.0, 0.0, math.pi)],
                                 [(0.18, 0.0), (0.18, 0.0)], AISLE,
                                 fparams=oc.FilterParams(safety_margin=margin))
            discs.append(closest[0])
            gaps.append(gap)
    finally:
        RNG, gt_gap = saved_rng, saved_gap
    assert all(b > a for a, b in zip(discs, discs[1:])), discs
    assert gaps[-1] > gaps[0] + 0.05 and min(gaps) > 0.05, gaps


def test_very_close_start_creeps_away_instead_of_deadlocking():
    # peer facing me with a 6 cm gap between the bodies: its turret (0.13 m from my front edge)
    # is inside peer_radius + safety_margin (0.156 m), but outside the lidar's blind radius
    peer = (0.14, 0.0, math.pi)
    assert gt_gap((0.0, 0.0, 0.0), peer) == pytest.approx(0.06, abs=1e-6)
    d = decide((0.0, 0.0, 0.0), [peer], AISLE, (0.18, 0.0))
    assert d.n_blobs == 1 and d.min_barrier < 0.0          # starts inside the margin
    assert d.active and d.v < 0.0        # backs away (slowly) rather than freezing
    assert d.v >= -FPAR.v_reverse_max - 1e-9


def test_wall_ahead_stops_before_contact():
    wall = [(0.5, -1.0, 0.5, 1.0)]
    me = [0.0, 0.0, 0.0]
    for _ in range(int(10 / 0.05)):
        d = decide(tuple(me), [], wall, (0.18, 0.0))
        me[0] += d.v * 0.05
    assert 0.5 - (me[0] + FP.x_max) > FPAR.static_margin * 0.5
