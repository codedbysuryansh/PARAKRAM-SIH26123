"""
PIBT-style decentralized coordination step with space-time reservation leases (pure: no ROS).

Every robot runs one ``PibtCoordinator`` and calls ``tick()`` at a fixed rate. There is no
referee: robots only exchange ``Intent`` (reserved cells + lease + desired path + priority) and
every robot applies the SAME deterministic rules to the same information, so contending robots
reach the same decision independently.

Rules (CLAUDE_CODE/02):
* Priority = age of the robot's current task (PIBT: priority grows while away from the goal and
  resets on arrival). Comparing ages ``now - start`` is the same as comparing ``-start``, so it
  is published as ``boost - floor(task_start * priority_grow_rate)``: a CONSTANT for the whole
  task. An older task always outranks a newer one, and the order cannot flip just because peers
  hear each other one message late (a counted-up age did flip at every increment, which
  livelocked evacuations). A robot resting at its goal takes the newest age (lowest rank); the
  deadlock breaker adds ``boost``. With the default 1 unit/s the value fits the int32 message
  field for sim time and wall time alike. For display (coord_status, CSV) the same priority is
  shown as an age, ``age_priority()`` = rank + floor(rate * now): it rises while the robot is
  away from its goal and is 0 at the goal. Total order:
  ``(priority, crc32(robot_id), robot_id)`` — stable across processes (Python's hash() is salted
  per process, so it is never used).
* Plan: windowed space-time A* to the goal; shelves, higher-priority robots' reservations and
  their near-future planned paths are blocked. Lower-priority robots are NOT obstacles in the
  plan — they get pushed.
* Push / priority inheritance: if a higher-priority robot's planned path runs through my
  occupied or claimed cells, I inherit ``its priority - 1`` and evacuate to the nearest cell off
  the paths of all robots pushing me (transit along the top pusher's path is allowed, through
  its reservations is not).
* Claims: I claim the next ``reserve_k`` cells of my plan's uninterrupted run of moves, in order,
  only if no other valid lease covers them and no higher-priority robot is about to take them.
  Simultaneous claims of the same cell (a race) are resolved by the total order; a cell someone
  physically occupies always beats a mere claim. A claim becomes HELD after ``claim_settle``
  (longer than one message round) and only held cells become movement authority for Nav2.
* Silent peers: their reservations are honoured until their lease expires, then ignored. Expiry
  is the only release mechanism.
* BLOCKED after ``blocked_after`` s of waiting (or ``stuck_timeout`` s with authority but no
  motion). Deadlock breaker: once BLOCKED for > ``deadlock_timeout``, bump the priority and
  re-plan with a detour around the blocking cells; if still BLOCKED after another
  ``deadlock_timeout``, request a re-route from the task layer.

It is designed for PIBT-style progress on biconnected grids such as the PARAKRAM warehouse
(exercised by the unit tests and acceptance runs, not proven for this decentralized variant). It
is NOT complete on graphs with dead ends / non-biconnected aisles; there the breaker and the task
layer's re-route are the safety valve (no completeness is claimed).
"""

from dataclasses import dataclass
import math
import zlib

from parakram_coord.spacetime_astar import (compress, distance_field, moving_prefix,
                                            spacetime_astar)

# RobotState.STATUS_* values
STATUS_IDLE, STATUS_MOVING, STATUS_BLOCKED = 0, 1, 2
INF = math.inf


def tie_key(robot_id):
    """Deterministic, process-independent tie-break value for ``robot_id``."""
    return zlib.crc32(robot_id.encode('utf-8'))


def priority_key(robot_id, priority):
    """Total order used by every robot: higher tuple wins."""
    return (int(priority), tie_key(robot_id), robot_id)


def occupied_cells(grid, x, y, radius):
    """
    Free cells whose square lies within ``radius`` of ``(x, y)``, plus the centre cell.

    Shelf cells are never contested, so they are left out (except the centre cell, which is kept
    even if localization error puts the centre on a shelf).
    """
    centre = grid.world_to_cell(x, y)
    cells = {centre}
    span = int(math.ceil(radius / grid.resolution))
    for dr in range(-span, span + 1):
        for dc in range(-span, span + 1):
            r, c = centre[0] + dr, centre[1] + dc
            if not grid.is_free(r, c):
                continue
            x_min, y_min, x_max, y_max = grid.cell_bounds(r, c)
            dx = max(x_min - x, 0.0, x - x_max)
            dy = max(y_min - y, 0.0, y - y_max)
            if math.hypot(dx, dy) <= radius:
                cells.add((r, c))
    return frozenset(cells)


def committed_claims(grid, claims, x, y, vx, vy, driving_to, radius, margin, min_speed=0.03):
    """
    Leading ``claims`` (cells, in order) the robot can no longer stop short of.

    A claim is committed only while Nav2 drives the robot towards it under earlier authority
    (``driving_to``: the cells of the active Nav2 goal), faster than ``min_speed``, with the
    safety disk (``radius``) within ``margin`` (stopping distance + latency) of the cell. A
    robot at rest has no committed claim, so every new claim still has to settle before it
    becomes movement authority; a cell its disk merely overlaps (e.g. the cell it just left) is
    covered by its occupancy instead.
    """
    committed = set()
    for c in claims:
        if c not in driving_to:
            break
        x0, y0, x1, y1 = grid.cell_bounds(*c)
        dist = math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))
        cx, cy = grid.cell_to_world(*c)
        closing = (vx * (cx - x) + vy * (cy - y)) / (math.hypot(cx - x, cy - y) or 1.0)
        if dist > radius + margin or closing <= min_speed:
            break
        committed.add(c)
    return committed


@dataclass
class CoordParams:
    """Coordination parameters (times in seconds of the robot's clock)."""

    window: int = 8
    reserve_k: int = 3
    lease_ttl: float = 2.0
    neighbor_timeout: float = 1.0
    claim_settle: float = 0.4
    priority_grow_rate: float = 1.0        # priority units per second of task age
    blocked_after: float = 3.0
    deadlock_timeout: float = 30.0         # > the longest normal crossing wait at Burger speeds
    deadlock_bump: int = 50               # = 50 s of task age at the default rate
    detour_ttl: float = 20.0
    stuck_timeout: float = 20.0
    push_horizon: int = 8
    occupancy_radius: float = 0.14         # >= Burger circumscribed radius 0.138 m; with the
    #                                        0.05 m goal tolerance a parked robot stays in one cell


@dataclass
class TickResult:
    """Everything one coordination tick decided."""

    priority: int              # effective priority to publish
    reserved: list             # occupied cells (centre first) then held/pending claims
    planned: list              # desired path, distinct cells, centre first
    authority: list            # centre + contiguous HELD claims (what Nav2 may drive)
    status: int
    blocked: bool
    at_goal: bool
    yielded: bool
    evacuating_for: object     # pusher id or None
    note: str
    reroute_request: bool
    goal_reached_event: bool
    n_peers: int


class PibtCoordinator:
    """Per-robot decentralized coordination state machine."""

    def __init__(self, robot_id, grid, params=None):
        """Create the coordinator for ``robot_id`` on ``grid`` (a WarehouseGrid)."""
        self.id = robot_id
        self.grid = grid
        self.p = params or CoordParams()
        self.goal = None
        self.claims = []                # [(cell, claim_time)] ahead of the centre, in order
        self.yields = 0
        self.replans = 0
        self.goals_assigned = 0
        self.goals_completed = 0
        self._goal_serial = 0
        self._counted_serial = 0
        self._task_start = 0.0
        self._boost = 0
        self._last_tick = None
        self._last_center = None
        self._last_move_time = None
        self._wait_since = None
        self._blocked_since = None
        self._stage = 0
        self._detour = {}
        self._yielding = False
        self._prev_planned = None
        self._hcache = {}

    # ------------------------------------------------------------------ helpers
    def _nbrs(self, cell):
        return self.grid.neighbors(cell[0], cell[1])

    def _h(self, goal):
        if goal not in self._hcache:
            self._hcache[goal] = distance_field([goal], self._nbrs)
        return self._hcache[goal]

    def set_goal(self, goal, now=None):
        """Assign a new goal cell (from the task layer or the temporary fixed assignment)."""
        self.goal = (int(goal[0]), int(goal[1]))
        self._goal_serial += 1
        self.goals_assigned += 1
        if now is not None:
            self._task_start = float(now)
        self._prev_planned = None
        self._stage = 0
        self._wait_since = None
        self._blocked_since = None

    def base_priority(self):
        """Priority before inheritance: task age rank (older = higher) plus breaker bumps."""
        return self._boost - int(math.floor(self.p.priority_grow_rate * self._task_start))

    def age_priority(self, priority, now):
        """Express a published priority rank as an age at ``now`` (for display only)."""
        return int(priority) + int(math.floor(self.p.priority_grow_rate * now))

    # ------------------------------------------------------------------ the tick
    def tick(self, now, center, occupied, table, committed=frozenset()):
        """
        Run one coordination step.

        ``center``: cell containing the robot centre; ``occupied``: cells its safety disk
        overlaps; ``table``: ReservationTable of peer intents (pruned here);
        ``committed``: claimed cells the robot can no longer stop before entering.
        """
        p = self.p
        self._last_tick = now
        center = tuple(center)
        occupied = frozenset(occupied) | {center}
        committed = frozenset(committed) - {center}
        at_goal = self.goal is not None and center == self.goal
        away_now = self.goal is not None and not at_goal

        if self.goal is None or at_goal:
            self._task_start = now          # resting: newest age, lowest rank (PIBT reset)
            self._boost = 0
        base = self.base_priority()

        goal_event = False
        if at_goal and self._counted_serial != self._goal_serial:
            self._counted_serial = self._goal_serial
            self.goals_completed += 1
            goal_event = True

        table.prune(now)
        valid = table.valid(now, exclude=self.id)
        fresh = table.fresh(now, p.neighbor_timeout, exclude=self.id)
        self._detour = {c: e for c, e in self._detour.items() if e > now}
        H = p.push_horizon

        def key_of(entry):
            return priority_key(entry.owner, entry.priority)

        # 1) pushers -> priority inheritance + evacuation
        mine = set(occupied) | {c for c, _ in self.claims}
        key0 = priority_key(self.id, base)
        pushers = [e for e in fresh.values()
                   if key_of(e) > key0 and set(e.planned[1:1 + H]) & mine]
        evac = max(pushers, key=key_of) if pushers else None
        eff = max(base, evac.priority - 1) if evac is not None else base
        my_key = priority_key(self.id, eff)
        # after inheritance I may outrank some of them (e.g. another robot evacuating for the
        # same pusher): only robots still above me push me
        pushers = [e for e in pushers if key_of(e) > my_key]

        # 2) planning obstacles: higher-priority reservations and near-future paths
        blocked = set(self._detour)
        desire = set()
        for e in valid.values():
            if key_of(e) > my_key:
                blocked |= set(e.cells) | set(e.occupied)
                # the path of the robot I evacuate for stays passable (I may transit along it
                # to get clear), but nothing else it crosses is unblocked
                if e.owner in fresh and (evac is None or e.owner != evac.owner):
                    near = set(e.planned[1:1 + H])
                    blocked |= near
                    desire |= near
        blocked -= occupied | committed

        # 3) plan
        if evac is not None:
            pushers.sort(key=key_of, reverse=True)
            plan = self._plan_evacuation(center, pushers, valid, frozenset(blocked), my_key)
        elif self.goal is not None and not at_goal:
            plan, _ = spacetime_astar(center, self.goal, p.window, self._nbrs,
                                      self._h(self.goal), blocked=frozenset(blocked))
        else:
            plan = [center]
        planned = compress(plan)
        self._count_replan(center, planned)

        # 4) claims (in order, contiguous) + race resolution
        want = moving_prefix(plan)[:p.reserve_k]
        keep_committed = [c for c, _ in self.claims if c in committed]
        if keep_committed and (not want or want[0] != keep_committed[0]):
            want = keep_committed[:1]          # already entering it: finish entering, stop there
        reserved_by = {}
        occupied_by = {}
        for e in valid.values():
            for c in e.cells:
                reserved_by.setdefault(c, []).append(e)
            for c in e.occupied:
                occupied_by.setdefault(c, []).append(e)
        # Races (two robots claimed the same free cell before hearing each other) need no extra
        # rule: higher-priority reservations are planning obstacles, so the lower robot's next
        # plan avoids the cell and its claim lapses, while the higher robot's plan ignores the
        # lower robot's reservation and keeps it — every robot applies the same total order.
        old = dict(self.claims)
        new_claims = []
        yielded = False
        for c in want:
            if c in committed:
                new_claims.append((c, old.get(c, now)))
                continue
            if c in occupied_by:
                yielded = True
                break
            if c not in old and (c in reserved_by or c in desire):
                yielded = True                      # held or about to be taken by someone else
                break
            new_claims.append((c, old.get(c, now)))
        if away_now and evac is None and not want:
            yielded = True                          # cannot move: my next cells are taken
        self.claims = new_claims

        # 5) movement authority = contiguous HELD claims
        authority = [center]
        for c, t0 in new_claims:
            if c in committed or now - t0 >= p.claim_settle:
                authority.append(c)
            else:
                break

        # 6) progress, BLOCKED, deadlock breaker
        away = self.goal is not None and not at_goal
        if center != self._last_center or self._last_move_time is None or not away:
            self._last_center = center
            self._last_move_time = now
            self._blocked_since = None               # progress / arrival ends a BLOCKED episode
            self._stage = 0
        has_auth = len(authority) > 1
        waiting = away and not has_auth
        if waiting:
            if self._wait_since is None:
                self._wait_since = now
        else:
            self._wait_since = None
        stuck = away and has_auth and now - self._last_move_time > p.stuck_timeout
        is_blocked = (waiting and now - self._wait_since >= p.blocked_after) or stuck
        # a BLOCKED episode lasts from the first BLOCKED tick until the robot is free to move
        # again (authority it is not stuck on), changes cell, arrives or gets a new goal; the
        # short non-BLOCKED gap when a stuck robot loses its authority to a detour does not
        # restart it. The breaker acts on how long the robot has been BLOCKED.
        if has_auth and not stuck:
            self._blocked_since = None
            self._stage = 0
        if is_blocked and self._blocked_since is None:
            self._blocked_since = now
        blocked_for = now - self._blocked_since if is_blocked else 0.0

        note = ''
        reroute = False
        if evac is not None:
            note = f'evacuating for {evac.owner}'
        if is_blocked:
            # CLAUDE_CODE/02: BLOCKED for > deadlock_timeout -> bump the priority and re-plan
            # with a detour around the blocking cells; still BLOCKED after another
            # deadlock_timeout -> request a re-route from the task layer
            if blocked_for > p.deadlock_timeout and self._stage < 1:
                self._stage = 1
                self._boost += p.deadlock_bump
                path_cells = planned[1:1 + H] or ([self.goal] if self.goal else [])
                for c in path_cells:
                    if c in reserved_by or c in occupied_by:
                        self._detour[c] = now + p.detour_ttl
                if stuck:
                    self._detour[authority[1]] = now + p.detour_ttl
                note = 'deadlock breaker: priority bump + detour'
            if blocked_for > 2 * p.deadlock_timeout and self._stage < 2:
                self._stage = 2
                reroute = True
                note = 'deadlock breaker: re-route requested'
            if not note:
                note = self._blocked_note(stuck, authority, reserved_by, occupied_by)

        if yielded and not self._yielding:
            self.yields += 1
        self._yielding = yielded

        if is_blocked:
            status = STATUS_BLOCKED
        elif (self.goal is None or at_goal) and evac is None:
            status = STATUS_IDLE
        else:
            status = STATUS_MOVING
        reserved = [center] + sorted(occupied - {center}) + \
            [c for c, _ in new_claims if c not in occupied]
        return TickResult(priority=eff, reserved=reserved, planned=planned, authority=authority,
                          status=status, blocked=is_blocked, at_goal=at_goal, yielded=yielded,
                          evacuating_for=evac.owner if evac is not None else None, note=note,
                          reroute_request=reroute, goal_reached_event=goal_event,
                          n_peers=len(valid))

    # ------------------------------------------------------------------ internals
    def _plan_evacuation(self, center, pushers, valid, blocked, my_key):
        """
        Shortest plan to the nearest cell off the pushers' paths and free of other robots.

        ``pushers`` is sorted by priority (highest first). A cell off every pusher's path is
        preferred; if there is none in reach, a cell off the top pusher's path is used. Cells
        other robots occupy are never targets; cells merely claimed by lower-priority robots
        are (those robots get pushed), so two evacuees do not flip-flop over each other's
        one-tick-old claims.
        """
        H = self.p.push_horizon
        others = set()
        for e in valid.values():
            others |= set(e.occupied)
            if priority_key(e.owner, e.priority) > my_key:
                others |= set(e.cells)
        dist = distance_field([center], self._nbrs,
                              passable=lambda c: c == center or c not in blocked)
        targets = []
        for group in (pushers, pushers[:1]):
            avoid = set()
            for e in group:
                avoid |= set(e.planned[:1 + H]) | set(e.cells) | set(e.occupied)
            targets = [c for c in dist if c not in avoid and c not in others]
            if targets:
                break
        if not targets:
            return [center]
        goal_h = self._h(self.goal) if self.goal is not None else {}
        best = min(targets, key=lambda c: (dist[c], goal_h.get(c, INF), c))
        plan, _ = spacetime_astar(center, best, self.p.window, self._nbrs,
                                  distance_field([best], self._nbrs), blocked=blocked)
        return plan

    def _count_replan(self, center, planned):
        prev = self._prev_planned
        self._prev_planned = planned
        if prev is None or prev == planned:
            return
        if center in prev:
            old_future = prev[prev.index(center) + 1:]
            new_future = planned[1:]
            n = min(len(old_future), len(new_future))
            if old_future[:n] == new_future[:n]:
                return
        self.replans += 1

    def _blocked_note(self, stuck, authority, reserved_by, occupied_by):
        goal = self.goal
        if stuck and len(authority) > 1:
            if authority[1] == goal:
                return 'goal cell physically occupied (parked/dead robot?)'
            return 'no progress with authority (cell ahead physically blocked?)'
        if goal in occupied_by or goal in reserved_by:
            entries = occupied_by.get(goal, []) + reserved_by.get(goal, [])
            owners = sorted({e.owner for e in entries})
            return f'goal occupied by {",".join(owners)}'
        return 'waiting for cells held by higher-priority robots'
