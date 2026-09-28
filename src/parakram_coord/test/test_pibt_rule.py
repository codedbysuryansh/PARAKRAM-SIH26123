"""
Unit tests: PIBT rule + decentralized coordination behaviour (discrete multi-robot simulation).

Every simulated robot decides alone from its peers' delayed intents (see discrete_sim.py); the
simulator flags any physical overlap or any authority granted into a live robot's cell.
"""

import os
import zlib

from discrete_sim import Agent, DiscreteSim
from parakram_coord.pibt_rule import (committed_claims, CoordParams, occupied_cells,
                                      PibtCoordinator, priority_key, STATUS_BLOCKED, tie_key)
from parakram_coord.reservation_table import Entry, ReservationTable
from parakram_sim.grid_utils import WarehouseGrid
import pytest

GRID_YAML = os.path.join(os.path.dirname(__file__), '..', '..', 'parakram_sim', 'config',
                         'warehouse_grid.yaml')


def grid_from_ascii(rows):
    n_rows, n_cols = len(rows), len(rows[0])
    blocked = [(n_rows - 1 - i, j) for i, line in enumerate(rows)
               for j, ch in enumerate(line) if ch == '#']
    return WarehouseGrid(0.0, 0.0, 0.4, n_rows, n_cols, blocked)


@pytest.fixture(scope='module')
def warehouse():
    return WarehouseGrid.from_yaml(GRID_YAML)


# ---------------------------------------------------------------- pure helpers
def test_tie_key_is_stable_crc32_not_python_hash():
    assert tie_key('robot1') == zlib.crc32(b'robot1') == 610525864
    assert tie_key('robot2') == 3177886482


def test_priority_order_priority_then_tie_key():
    assert priority_key('robot1', 5) > priority_key('robot3', 4)
    # equal priority -> crc32 decides, identically for every robot
    winner = max(['robot1', 'robot2', 'robot3'], key=lambda r: priority_key(r, 0))
    assert winner == 'robot3'


def test_occupied_cells_disk(warehouse):
    x, y = warehouse.cell_to_world(5, 6)
    assert occupied_cells(warehouse, x, y, 0.19) == {(5, 6)}
    both = occupied_cells(warehouse, x + 0.18, y, 0.19)            # near the east boundary
    assert both == {(5, 6), (5, 7)}
    ax, ay = warehouse.cell_to_world(4, 6)                          # shelves at (4,5), (4,7)
    assert occupied_cells(warehouse, ax + 0.1, ay, 0.19) == {(4, 6)}   # shelf never occupied


def test_claims_are_committed_only_while_driving_into_them(warehouse):
    x, y = warehouse.cell_to_world(5, 3)
    ahead, behind = (5, 2), (5, 4)                  # arrived from the east, next move is west

    def committed(claims, px, vx, driving_to):
        return committed_claims(warehouse, claims, px, y, vx, 0.0, driving_to, 0.14, 0.08)

    # standing, disk still overlapping the cell just left: nothing is committed (that cell is
    # covered by occupancy; a new claim on it must settle like any other)
    assert committed([behind], x + 0.07, 0.0, (behind,)) == set()
    assert committed([ahead], x - 0.05, 0.0, (ahead,)) == set()
    # driving west into the claimed cell, within stopping reach: committed
    assert committed([ahead], x - 0.05, -0.15, (ahead,)) == {ahead}
    # driving away from a claimed cell, or towards one outside the active Nav2 goal: not
    assert committed([behind], x + 0.07, -0.15, (behind,)) == set()
    assert committed([ahead], x - 0.05, -0.15, ()) == set()


def test_priority_is_task_age_rank_constant_during_task_and_reset_at_goal(warehouse):
    """Older task outranks newer; the value never changes mid-task (no latency flicker)."""
    p = CoordParams(priority_grow_rate=1.0)
    old, new = (PibtCoordinator(r, warehouse, p) for r in ('robot1', 'robot2'))
    table = ReservationTable()
    old.set_goal((5, 9), 0.0)
    new.set_goal((3, 6), 3.0)
    seen = {old.tick(0.1 * k, (5, 4), {(5, 4)}, table).priority for k in range(51)}
    assert seen == {0}
    assert new.tick(5.0, (7, 6), {(7, 6)}, table).priority == -3
    assert priority_key('robot1', 0) > priority_key('robot2', -3)     # older task wins
    res = old.tick(5.2, (5, 9), {(5, 9)}, table)
    assert res.at_goal and res.goal_reached_event and res.priority == -5   # reset: newest
    assert old.goals_completed == 1 and old.goals_assigned == 1


def test_claims_become_authority_only_after_settle(warehouse):
    p = CoordParams(claim_settle=0.4, reserve_k=3)
    core = PibtCoordinator('robot1', warehouse, p)
    table = ReservationTable()
    core.set_goal((5, 9), 0.0)
    first = core.tick(0.0, (5, 4), {(5, 4)}, table)
    assert [c for c, _ in core.claims] == [(5, 5), (5, 6), (5, 7)]
    assert first.authority == [(5, 4)]                            # pending, not held
    later = core.tick(0.4, (5, 4), {(5, 4)}, table)
    assert later.authority == [(5, 4), (5, 5), (5, 6), (5, 7)]


def test_cell_reserved_by_higher_priority_peer_blocks_claim(warehouse):
    core = PibtCoordinator('robot1', warehouse, CoordParams())
    table = ReservationTable()
    table.update(Entry(owner='robot3', seq=1, priority=9, cells=((5, 6),),
                       occupied=frozenset({(5, 6)}), planned=((5, 6), (5, 5), (5, 4)),
                       lease_expiry=2.0, heard_at=0.0))
    core.set_goal((5, 9), 0.0)
    res = core.tick(0.0, (5, 4), {(5, 4)}, table)
    # robot3 (higher) plans through my cell -> I am pushed and evacuate off its path
    assert res.evacuating_for == 'robot3'
    assert res.priority == 8                                       # inherited: 9 - 1
    assert (5, 6) not in [c for c, _ in core.claims]
    assert res.planned[-1] not in {(5, 6), (5, 5), (5, 4)}


def test_evacuee_leaves_the_path_of_every_robot_pushing_it(warehouse):
    core = PibtCoordinator('robot2', warehouse, CoordParams())
    table = ReservationTable()
    core.set_goal((5, 9), 0.0)
    core.tick(0.0, (5, 6), {(5, 6)}, table)
    assert [c for c, _ in core.claims] == [(5, 7), (5, 8), (5, 9)]
    # equal priorities: robot3 (top by crc32) comes west into my claims, robot1 north through
    # my cell; my own cell is off robot3's path but on robot1's, so staying put is no refuge
    table.update(Entry(owner='robot3', seq=1, priority=10, cells=((5, 10),),
                       occupied=frozenset({(5, 10)}), planned=((5, 10), (5, 9), (5, 8), (5, 7)),
                       lease_expiry=2.1, heard_at=0.1))
    table.update(Entry(owner='robot1', seq=1, priority=10, cells=((4, 6),),
                       occupied=frozenset({(4, 6)}), planned=((4, 6), (5, 6), (6, 6), (7, 6)),
                       lease_expiry=2.1, heard_at=0.1))
    res = core.tick(0.1, (5, 6), {(5, 6)}, table)
    assert res.evacuating_for == 'robot3' and res.priority == 9
    assert res.planned == [(5, 6), (5, 5)]                         # off both robots' paths


def test_race_resolved_by_total_order_without_push(warehouse):
    """Both claimed (5,6); the peer is valid but not fresh (no push): the total order decides."""
    p = CoordParams(neighbor_timeout=0.5)
    for peer_prio, i_keep in ((9, False), (0, True)):
        core = PibtCoordinator('robot2', warehouse, p)
        core.set_goal((5, 9), 0.0)
        table = ReservationTable()
        core.tick(0.0, (5, 5), {(5, 5)}, table)
        assert (5, 6) in [c for c, _ in core.claims]
        table.update(Entry(owner='robot1', seq=1, priority=peer_prio, cells=((4, 6), (5, 6)),
                           occupied=frozenset({(4, 6)}), planned=((4, 6),),
                           lease_expiry=5.0, heard_at=0.0))
        res = core.tick(1.0, (5, 5), {(5, 5)}, table)            # peer heard 1 s ago: stale
        assert res.evacuating_for is None
        assert ((5, 6) in [c for c, _ in core.claims]) is i_keep
        assert res.yielded is (not i_keep)


# ---------------------------------------------------------------- scenarios
def _run(grid, spec, seconds, params=None, **kw):
    p = params or CoordParams()
    agents = [Agent(r, s, g, grid, p) for r, (s, g) in spec.items()]
    sim = DiscreteSim(grid, agents, p, **kw)
    finished = sim.run(seconds)
    return sim, agents, finished


def test_two_robot_head_on_corridor_resolved_deterministically():
    ring = grid_from_ascii(['.......',
                            '.#####.',
                            '.......'])
    spec = {'robot1': ((0, 0), [(0, 6)]), 'robot2': ((0, 6), [(0, 0)])}
    sim, agents, finished = _run(ring, spec, 120)
    assert finished and not sim.violations
    again, _, _ = _run(ring, spec, 120)
    assert sim.trace == again.trace                               # same decisions every time


def test_head_on_in_warehouse_aisle_uses_push(warehouse):
    spec = {'robot1': ((5, 4), [(5, 9)]), 'robot2': ((5, 8), [(5, 3)])}
    sim, agents, finished = _run(warehouse, spec, 200)
    assert finished and not sim.violations
    notes = {r.note for a in agents for _, r in a.results}
    assert any(n.startswith('evacuating for') for n in notes)     # PIBT push happened


def test_three_robot_intersection_resolved(warehouse):
    spec = {'robot1': ((5, 4), [(5, 9), (5, 4)] * 2),
            'robot2': ((5, 8), [(5, 3), (5, 8)] * 2),
            'robot3': ((3, 6), [(7, 6), (3, 6)] * 2)}
    sim, agents, finished = _run(warehouse, spec, 600)
    assert finished and not sim.violations
    assert all(a.core.goals_completed == 4 for a in agents)
    again, _, _ = _run(warehouse, spec, 600)
    assert sim.trace == again.trace


def test_intersection_with_burger_timing_needs_no_deadlock_breaker(warehouse):
    """Slow moves (2.2 s/cell) and turns (2.5 s per 90 deg), breaker off: no livelock."""
    p = CoordParams(deadlock_timeout=1e9)
    spec = {'robot1': ((5, 4), [(5, 9), (5, 4)] * 2),
            'robot2': ((5, 8), [(5, 3), (5, 8)] * 2),
            'robot3': ((3, 6), [(7, 6), (3, 6)] * 2)}
    sim, agents, finished = _run(warehouse, spec, 900, p, move_ticks=22, turn_ticks=25)
    assert finished and not sim.violations
    assert all(a.core.goals_completed == 4 for a in agents)


def test_four_robot_crossing_needs_no_deadlock_breaker(warehouse):
    """Two robots per axis through one junction: evacuees must not flip-flop or block."""
    p = CoordParams(deadlock_timeout=1e9)
    spec = {'robot1': ((5, 4), [(5, 9), (5, 4)] * 4),
            'robot2': ((5, 8), [(5, 3), (5, 8)] * 4),
            'robot3': ((3, 6), [(7, 6), (3, 6)] * 4),
            'robot4': ((7, 6), [(3, 6), (7, 6)] * 4)}
    sim, agents, finished = _run(warehouse, spec, 900, p)
    assert finished and not sim.violations
    assert all(a.core.goals_completed == 8 for a in agents)


def test_simultaneous_claim_tie_break(warehouse):
    # Both reach for the free junction (5,6) in the SAME tick, before hearing each other.
    spec = {'robot1': ((5, 5), [(5, 7)]), 'robot2': ((4, 6), [(6, 6)])}
    p = CoordParams()
    agents = [Agent(r, s, g, warehouse, p) for r, (s, g) in spec.items()]
    sim = DiscreteSim(warehouse, agents, p)
    sim.step()                                                     # tick 0: both claim
    claims0 = {a.id: [c for c, _ in a.core.claims] for a in agents}
    assert (5, 6) in claims0['robot1'] and (5, 6) in claims0['robot2']
    sim.step()                                                     # tick 1: both see the race
    winner = max(spec, key=lambda r: priority_key(r, 0))           # equal priority -> crc32
    loser = 'robot1' if winner == 'robot2' else 'robot2'
    held = {a.id: [c for c, _ in a.core.claims] for a in agents}
    assert (5, 6) in held[winner] and (5, 6) not in held[loser]
    assert all(a.target is None for a in agents)                   # nobody moved during the race
    assert sim.run(120) and not sim.violations


def test_silent_owner_lease_expiry_frees_cell(warehouse):
    p = CoordParams(lease_ttl=2.0)
    a = Agent('robot1', (5, 4), [(5, 8)], warehouse, p)
    b = Agent('robot2', (5, 6), [(5, 6)], warehouse, p)           # parked on the junction
    sim = DiscreteSim(warehouse, [a, b], p)
    for _ in range(3):
        sim.step()
    last_publish = b.outbox[-1][0] * sim.dt
    b.publishing = False                                           # goes silent...
    b.physical = False                                             # ...and is removed
    first_claim = None
    while sim.tick_no * sim.dt < 30 and first_claim is None:
        sim.step()
        if (5, 6) in [c for c, _ in a.core.claims]:
            first_claim = (sim.tick_no - 1) * sim.dt
    assert first_claim is not None
    assert first_claim >= last_publish + p.lease_ttl - 1e-9        # not before the lease ran out
    assert sim.run(60) and not sim.violations


def test_dead_end_no_completeness_claim_breaker_escalates():
    corridor = grid_from_ascii(['.....'])                          # a path graph: no passing place
    p = CoordParams(deadlock_timeout=5.0, blocked_after=1.0)
    spec = {'robot1': ((0, 0), [(0, 4)]), 'robot2': ((0, 4), [(0, 0)])}
    sim, agents, finished = _run(corridor, spec, 60, params=p)
    assert not finished and not sim.violations                    # safe, but not solvable
    T, eps = p.deadlock_timeout, 2 * sim.dt + 1e-9
    for a in agents:
        ticks = [(k * sim.dt, r) for k, r in a.results]
        bump = next(t for t, r in ticks if r.note == 'deadlock breaker: priority bump + detour')
        reroute = next(t for t, r in ticks if r.reroute_request)
        start = max(t for t, r in ticks if t < bump and not r.blocked) + sim.dt
        # BLOCKED for > deadlock_timeout -> bump + detour; still BLOCKED -> re-route request
        assert T - 1e-9 < bump - start <= T + eps
        assert 2 * T - 1e-9 < reroute - start <= 2 * T + eps
        assert all(r.blocked for t, r in ticks if start <= t <= reroute)


def test_breaker_times_the_blocked_episode_across_stuck_to_waiting(warehouse):
    """A silent obstacle on the goal: bump + detour, then re-route, timed on BLOCKED status."""
    p = CoordParams(stuck_timeout=5.0, blocked_after=1.0, deadlock_timeout=10.0)
    core = PibtCoordinator('robot1', warehouse, p)
    table = ReservationTable()
    core.set_goal((5, 8), 0.0)
    first = {}
    for k in range(300):
        t = round(0.1 * k, 1)
        res = core.tick(t, (5, 7), {(5, 7)}, table)            # never gets into (5, 8)
        for name, hit in (('blocked', res.blocked), ('reroute', res.reroute_request),
                          ('bump', res.note == 'deadlock breaker: priority bump + detour')):
            if hit:
                first.setdefault(name, t)
    assert 5.0 < first['blocked'] <= 5.2                          # stuck with authority
    assert 10.0 - 1e-6 < first['bump'] - first['blocked'] <= 10.2
    # the detour takes the authority away ("stuck" -> "waiting"); the episode goes on
    assert 20.0 - 1e-6 < first['reroute'] - first['blocked'] <= 20.2


def test_dead_robot_parked_on_goal_reports_blocked(warehouse):
    p = CoordParams(stuck_timeout=5.0, blocked_after=1.0, deadlock_timeout=10.0)
    a = Agent('robot1', (5, 4), [(5, 8)], warehouse, p)
    dead = Agent('robot9', (5, 8), [], warehouse, p, publishing=False)   # never publishes
    sim = DiscreteSim(warehouse, [a, dead], p)
    sim.run(40, until_done=False)
    assert not sim.violations and a.core.goals_completed == 0
    last = a.results[-1][1]
    assert last.blocked and last.status == STATUS_BLOCKED
    assert any(r.note == 'goal cell physically occupied (parked/dead robot?)'
               for _, r in a.results)
    # the breaker still escalates to a re-route request for the task layer, even though the
    # detour turns "stuck with authority" into "waiting" (same BLOCKED episode)
    assert any(r.reroute_request for _, r in a.results)
