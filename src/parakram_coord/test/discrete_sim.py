"""
Discrete-time multi-robot simulator for unit-testing the decentralized coordination logic.

Each agent runs its own PibtCoordinator and only knows its peers through their published
intents, delivered with ``latency`` ticks of delay (no shared state, no referee). A move to an
adjacent cell takes ``move_ticks`` ticks; while moving an agent occupies both cells. The
simulator checks, every tick, that no two agents' occupied cells overlap, and that no agent is
ever granted authority into a cell a live peer physically occupies. A "dead" agent is a
physical obstacle that never publishes; a "silent" agent stops publishing but keeps occupying.
"""

import dataclasses

from parakram_coord.pibt_rule import CoordParams, PibtCoordinator
from parakram_coord.reservation_table import Entry, ReservationTable


class Agent:
    """One simulated robot."""

    def __init__(self, rid, start, goals, grid, params, publishing=True):
        """Create an agent at ``start`` with a list of goal cells (visited in order)."""
        self.id = rid
        self.core = PibtCoordinator(rid, grid, params)
        self.table = ReservationTable()
        self.pos = tuple(start)
        self.target = None
        self.progress = 0
        self.goals = [tuple(g) for g in goals]
        self.next_goal = 0
        self.publishing = publishing
        self.physical = True
        self.outbox = []          # (publish_tick, Entry)
        self.res = None
        self.results = []
        self.heading = None       # (dr, dc) of the last move
        self.turning = None       # [cell, ticks_left] while rotating in place towards a move

    def center(self, move_ticks):
        """Cell containing the robot centre (switches half-way through a move)."""
        if self.target is not None and 2 * self.progress >= move_ticks:
            return self.target
        return self.pos

    def occupied(self):
        """Cells physically covered."""
        return {self.pos} | ({self.target} if self.target is not None else set())

    def done(self):
        """Return True when every goal was completed."""
        return self.core.goals_completed >= len(self.goals)


class DiscreteSim:
    """Lock-step simulator (each agent still decides alone, from delayed peer messages)."""

    def __init__(self, grid, agents, params=None, dt=0.1, move_ticks=10, latency=1,
                 turn_ticks=0):
        """Create the simulation (``turn_ticks``: rotate-in-place time per 90 degrees)."""
        self.grid = grid
        self.agents = agents
        self.p = params or CoordParams()
        self.dt = dt
        self.move_ticks = move_ticks
        self.latency = latency
        self.turn_ticks = turn_ticks
        self.tick_no = 0
        self.violations = []
        self.trace = []
        for a in agents:
            if a.goals:
                a.core.set_goal(a.goals[0], 0.0)
                a.next_goal = 1

    def _deliver(self, recipient, now):
        horizon = self.tick_no - self.latency
        for a in self.agents:
            if a is recipient:
                continue
            latest = None
            for t_pub, entry in a.outbox:
                if t_pub <= horizon:
                    latest = (t_pub, entry)
            if latest is not None:
                t_pub, entry = latest
                recipient.table.update(dataclasses.replace(
                    entry, heard_at=(t_pub + self.latency) * self.dt))

    def step(self):
        """Advance one tick."""
        now = self.tick_no * self.dt
        mt = self.move_ticks
        for a in self.agents:
            if not a.publishing:
                continue
            self._deliver(a, now)
            committed = {a.target} if a.target is not None else set()
            res = a.core.tick(now, a.center(mt), a.occupied(), a.table, committed)
            a.res = res
            a.results.append((self.tick_no, res))
            if res.goal_reached_event and a.next_goal < len(a.goals):
                a.core.set_goal(a.goals[a.next_goal], now)
                a.next_goal += 1
            a.outbox.append((self.tick_no, Entry(
                owner=a.id, seq=self.tick_no + 1, priority=res.priority,
                cells=tuple(res.reserved), occupied=frozenset(a.occupied()),
                planned=tuple(res.planned), lease_expiry=now + self.p.lease_ttl,
                heard_at=now)))
            a.outbox = a.outbox[-(self.latency + 2):]
        # motion
        for a in self.agents:
            if a.target is not None:
                a.progress += 1
                if a.progress >= mt:
                    a.pos, a.target, a.progress = a.target, None, 0
                    self.trace.append((self.tick_no, a.id, a.pos))
                continue
            if not a.publishing or a.res is None or len(a.res.authority) < 2:
                a.turning = None
                continue
            nxt = a.res.authority[1]
            if abs(nxt[0] - a.pos[0]) + abs(nxt[1] - a.pos[1]) != 1:
                continue
            step = (nxt[0] - a.pos[0], nxt[1] - a.pos[1])
            if self.turn_ticks and a.heading is not None and step != a.heading:
                if a.turning is None or a.turning[0] != nxt:
                    reverse = step == (-a.heading[0], -a.heading[1])
                    a.turning = [nxt, self.turn_ticks * (2 if reverse else 1)]
                a.turning[1] -= 1
                if a.turning[1] > 0:
                    continue                                  # still rotating in place
            a.turning = None
            occupant = next((b for b in self.agents if b is not a and b.physical
                             and nxt in b.occupied()), None)
            if occupant is None:
                a.target, a.progress = nxt, 0
                a.heading = step
            elif occupant.publishing:
                self.violations.append((self.tick_no, a.id, 'authority into cell occupied by',
                                        occupant.id, nxt))
            # else: a dead/silent robot physically blocks the cell -> the agent stays put
        # invariant: physical occupancy is exclusive
        seen = {}
        for a in self.agents:
            if not a.physical:
                continue
            for c in a.occupied():
                if c in seen:
                    self.violations.append((self.tick_no, 'overlap', seen[c], a.id, c))
                seen[c] = a.id
        self.tick_no += 1

    def run(self, max_seconds, until_done=True):
        """Step until every publishing agent finished its goals (or ``max_seconds``)."""
        while self.tick_no * self.dt < max_seconds:
            self.step()
            if until_done and all(a.done() and a.target is None
                                  for a in self.agents if a.publishing and a.goals):
                return True
        return False
