"""Unit tests: reservation table lease expiry and conflict queries."""

from parakram_coord.reservation_table import Entry, ReservationTable


def entry(owner, cells, lease, seq=1, heard=0.0, prio=0, occupied=None, planned=()):
    return Entry(owner=owner, seq=seq, priority=prio, cells=tuple(cells),
                 occupied=frozenset(occupied if occupied is not None else cells[:1]),
                 planned=tuple(planned), lease_expiry=lease, heard_at=heard)


def test_lease_expiry_frees_cells_without_release_message():
    t = ReservationTable()
    t.update(entry('robot2', [(5, 6), (5, 7)], lease=2.0))
    assert t.is_reserved((5, 6), now=1.99)
    assert not t.is_reserved((5, 6), now=2.0)          # lease_expiry <= now -> expired
    assert t.valid(2.0) == {}
    assert t.prune(2.0) == ['robot2'] and len(t) == 0


def test_renewal_extends_lease():
    t = ReservationTable()
    t.update(entry('r', [(1, 1)], lease=2.0, seq=1))
    t.update(entry('r', [(1, 1)], lease=3.0, seq=2, heard=1.0))
    assert t.is_reserved((1, 1), now=2.5)
    assert not t.is_reserved((1, 1), now=3.0)


def test_silent_owner_expires_while_others_renew():
    t = ReservationTable()
    t.update(entry('silent', [(0, 0)], lease=2.0))
    for k in range(1, 6):                                   # 'live' keeps renewing
        t.update(entry('live', [(0, 1)], lease=k + 2.0, seq=k, heard=float(k)))
    assert set(t.valid(4.5)) == {'live'}
    assert t.owners_of((0, 0), 4.5) == []


def test_older_messages_ignored_and_restart_accepted():
    t = ReservationTable()
    assert t.update(entry('r', [(1, 1)], lease=5.0, seq=10))
    assert not t.update(entry('r', [(2, 2)], lease=5.0, seq=9, heard=0.1))   # reordered
    assert t.owners_of((1, 1), 1.0)[0].seq == 10
    assert t.update(entry('r', [(3, 3)], lease=9.0, seq=1, heard=6.0))        # old lease over
    assert t.is_reserved((3, 3), 6.5)
    assert t.update(entry('q', [(4, 4)], lease=9.0, seq=500))
    assert t.update(entry('q', [(4, 5)], lease=9.5, seq=2, heard=1.0))        # evident restart


def test_conflict_queries_and_exclude():
    t = ReservationTable()
    t.update(entry('a', [(1, 1), (1, 2)], lease=5.0))
    t.update(entry('b', [(1, 2), (1, 3)], lease=5.0))
    assert sorted(t.reserved_cells(1.0)[(1, 2)]) == ['a', 'b']
    assert t.conflicts([(1, 1), (1, 2), (9, 9)], 1.0, exclude='a') == {(1, 2): ['b']}
    assert not t.is_reserved((1, 1), 1.0, exclude='a')
    assert set(t.fresh(3.0, timeout=1.0)) == set()          # heard at 0.0, timeout 1 s
    assert set(t.fresh(0.5, timeout=1.0)) == {'a', 'b'}
