"""Unit tests of the CLAUDE_CODE/06 lease protocol: own lease, ghosts, re-acquisition, sensing."""

import math
import os

from parakram_coord.ghost_view import classify_cell, FREE, OCCUPIED, UNKNOWN
from parakram_coord.leases import NEVER, OwnLease
from parakram_coord.pibt_rule import CoordParams, PibtCoordinator
from parakram_coord.reservation_table import Entry, ReservationTable
from parakram_sim.grid_utils import WarehouseGrid
import pytest

GRID_YAML = os.path.join(os.path.dirname(__file__), '..', '..', 'parakram_sim', 'config',
                         'warehouse_grid.yaml')


@pytest.fixture(scope='module')
def warehouse():
    return WarehouseGrid.from_yaml(GRID_YAML)


def entry(owner, seq, cells, occupied, heard, ttl=2.0, authority=(), priority=0):
    return Entry(owner=owner, seq=seq, priority=priority, cells=tuple(cells),
                 occupied=frozenset(occupied), planned=tuple(cells[:1]),
                 lease_expiry=heard + ttl, heard_at=heard, authority=tuple(authority))


# ---------------------------------------------------------------- own lease
def test_own_lease_needs_every_live_peer_to_acknowledge_within_L():
    lease = OwnLease(lease_ttl=2.0, stop_margin=0.3)
    for s in range(1, 31):
        lease.sent_renewal(s, 0.1 * s)
    assert lease.status(0.5, set(), set()) == (True, '')             # alone: nothing to ask
    ok, why = lease.status(0.5, {'robot2'}, {'robot2'})
    assert not ok and 'robot2' in why                                 # never acknowledged
    lease.on_ack('robot2', 10)                                        # sent at 1.0 s
    assert lease.acked_time('robot2') == pytest.approx(1.0)
    assert lease.horizon({'robot2'}) == pytest.approx(3.0)
    assert lease.status(2.6, {'robot2'}, {'robot2'})[0]
    assert not lease.status(2.75, {'robot2'}, {'robot2'})[0]         # within the stop margin
    lease.on_ack('robot2', 5)                                         # stale ack: ignored
    assert lease.acks['robot2'] == 10
    assert lease.acked_by_all(10, {'robot2'}) and not lease.acked_by_all(11, {'robot2'})


def test_quorum_isolated_robot_freezes_majority_continues():
    lease = OwnLease(lease_ttl=2.0)
    lease.sent_renewal(1, 0.0)
    lease.on_ack('robot2', 1)
    lease.on_ack('robot3', 1)
    known = {'robot2', 'robot3'}
    assert lease.status(0.5, {'robot2', 'robot3'}, known)[0]
    assert lease.status(0.5, {'robot2'}, known)[0]                   # 2 of 3: a majority
    ok, why = lease.status(0.5, set(), known)                         # 1 of 3: isolated
    assert not ok and 'quorum' in why
    two_of_four = OwnLease(2.0).status(0.0, {'r2'}, {'r2', 'r3', 'r4'})
    assert not two_of_four[0]                                          # 2 of 4: no majority


def test_task_renewal_needs_a_majority_acknowledgement():
    lease = OwnLease(lease_ttl=2.0)
    lease.sent_renewal(1, 1.0, None)
    lease.sent_renewal(2, 1.1, ('task_001', 3))
    lease.sent_renewal(3, 1.2, ('task_001', 3))
    task, known = ('task_001', 3), {'robot2', 'robot3'}
    lease.on_ack('robot3', 1)                                         # saw no renewal of it
    assert lease.task_quorum_time(known, task) is None                # isolated holder: none
    lease.on_ack('robot2', 3)
    assert lease.task_acked_by('robot2', task) == pytest.approx(1.2)
    assert lease.task_acked_by('robot3', task) is None
    # 2 of 3 is a majority: a peer back from a partition with a stale ack changes nothing
    assert lease.task_quorum_time(known, task) == pytest.approx(1.2)
    lease.on_ack('robot3', 2)
    assert lease.task_quorum_time(known, task) == pytest.approx(1.2)
    four = known | {'robot4'}                                         # 3 of 4 needed
    assert lease.task_quorum_time(four, task) == pytest.approx(1.1)
    assert lease.task_quorum_time(four, ('task_002', 1)) is None
    assert lease.task_quorum_time(set(), task) == math.inf            # alone: nobody to ask


def test_acks_older_than_the_log_fall_back_to_the_oldest_known():
    lease = OwnLease(lease_ttl=2.0, history=5)
    for s in range(1, 11):
        lease.sent_renewal(s, float(s))
    lease.on_ack('robot2', 3)                                         # seq 3 left the log
    assert lease.acked_time('robot2') == NEVER
    lease.on_ack('robot2', 7)
    assert lease.acked_time('robot2') == 7.0


# ---------------------------------------------------------------- ghosts / release
def test_expired_lease_frees_reservations_but_not_the_body():
    table = ReservationTable()
    table.update(entry('robot2', 5, [(5, 5), (5, 6), (5, 7)], {(5, 5)}, heard=0.0,
                       authority=[(5, 5), (5, 6)]))
    assert table.prune(1.9) == [] and table.ghost_cells() == frozenset()
    assert table.prune(2.0) == ['robot2']
    assert table.valid(2.0) == {}
    assert table.ghost_cells() == {(5, 5), (5, 6)}                    # body envelope stays
    [(kind, owner, t, reclaimed)] = table.drain_events()
    assert (kind, owner, t, reclaimed) == ('lease_expired', 'robot2', 2.0, {(5, 7)})
    assert table.clear_ghost_cell((5, 6), 3.0) == ['robot2']          # seen empty
    assert table.ghost_cells() == {(5, 5)}
    table.update(entry('robot2', 9, [(5, 5)], {(5, 5)}, heard=4.0))  # heard again
    assert table.ghost_cells() == frozenset()
    kinds = [e[0] for e in table.drain_events()]
    assert kinds == ['ghost_cleared', 'peer_back']


def test_baseline_table_never_expires_by_time_only_release_frees():
    table = ReservationTable(expire_by_lease=False)
    table.update(entry('robot2', 5, [(5, 6), (5, 7)], {(5, 6)}, heard=0.0))
    assert table.prune(100.0) == [] and 'robot2' in table.valid(100.0)
    assert table.release('robot2', 100.0)
    assert table.valid(100.0) == {} and table.ghost_cells() == {(5, 6)}
    assert not table.release('robot2', 101.0)
    assert table.drain_events()[0][:2] == ('release', 'robot2')


def test_ghost_cells_are_never_claimed_and_planned_around(warehouse):
    core = PibtCoordinator('robot1', warehouse, CoordParams(reserve_k=3))
    core.set_goal((5, 9), 0.0)
    res = core.tick(0.0, (5, 4), {(5, 4)}, ReservationTable(), obstacles=frozenset({(5, 6)}))
    assert (5, 6) not in [c for c, _ in core.claims]
    assert (5, 6) not in res.planned


def test_frozen_robot_claims_nothing_then_reacquires(warehouse):
    p = CoordParams(claim_settle=0.4, reserve_k=3)
    core = PibtCoordinator('robot1', warehouse, p)
    table = ReservationTable()
    core.set_goal((5, 9), 0.0)
    core.tick(0.0, (5, 4), {(5, 4)}, table)
    assert core.tick(0.5, (5, 4), {(5, 4)}, table).authority[-1] == (5, 7)
    frozen = core.tick(0.6, (5, 4), {(5, 4)}, table, frozen=True)
    assert frozen.authority == [(5, 4)] and core.claims == []
    assert 'lease' in frozen.note
    again = core.tick(0.7, (5, 4), {(5, 4)}, table)                   # lease valid again
    assert again.authority == [(5, 4)]                               # re-claimed, not yet held
    assert core.tick(1.1, (5, 4), {(5, 4)}, table).authority[-1] == (5, 7)


def test_claims_become_authority_only_once_acknowledged(warehouse):
    core = PibtCoordinator('robot1', warehouse, CoordParams(claim_settle=0.4, reserve_k=3))
    table = ReservationTable()
    core.set_goal((5, 9), 0.0)
    core.tick(0.0, (5, 4), {(5, 4)}, table)
    acked = {(5, 5)}
    res = core.tick(0.5, (5, 4), {(5, 4)}, table, claim_acked=lambda c: c in acked)
    assert res.authority == [(5, 4), (5, 5)]                         # contiguous acked prefix
    acked |= {(5, 6), (5, 7)}
    res = core.tick(0.6, (5, 4), {(5, 4)}, table, claim_acked=lambda c: c in acked)
    assert res.authority == [(5, 4), (5, 5), (5, 6), (5, 7)]


# ---------------------------------------------------------------- lidar evidence
def scan(ranges_fn, n=360):
    inc = 2 * math.pi / n
    return [ranges_fn(-math.pi + i * inc) for i in range(n)], -math.pi, inc


def test_cell_seen_through_is_free_return_inside_is_occupied_occlusion_unknown():
    box = (0.8, -0.2, 1.2, 0.2)                     # a 0.4 m cell 0.8-1.2 m ahead
    ranges, a0, inc = scan(lambda a: 3.0)
    assert classify_cell(box, (0.0, 0.0, 0.0), ranges, a0, inc, 0.12, 3.5) == FREE
    ranges, a0, inc = scan(lambda a: 1.0 if abs(a) < 0.08 else 3.0)  # a body at 1.0 m
    assert classify_cell(box, (0.0, 0.0, 0.0), ranges, a0, inc, 0.12, 3.5) == OCCUPIED
    ranges, a0, inc = scan(lambda a: 0.5 if abs(a) < 0.3 else 3.0)   # something in front
    assert classify_cell(box, (0.0, 0.0, 0.0), ranges, a0, inc, 0.12, 3.5) == UNKNOWN
    ranges, a0, inc = scan(lambda a: math.inf)                        # nothing within range
    assert classify_cell(box, (0.0, 0.0, 0.0), ranges, a0, inc, 0.12, 3.5) == FREE
    far = (3.3, -0.2, 3.7, 0.2)                                        # beyond the lidar range
    assert classify_cell(far, (0.0, 0.0, 0.0), ranges, a0, inc, 0.12, 3.5) == UNKNOWN
    # the sensor's heading matters: the same box behind a robot facing -x
    ranges, a0, inc = scan(lambda a: 1.0 if abs(abs(a) - math.pi) < 0.08 else 3.0)
    assert classify_cell(box, (0.0, 0.0, math.pi), ranges, a0, inc, 0.12, 3.5) == OCCUPIED
