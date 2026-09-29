"""Unit tests of the replicated task pool (CLAUDE_CODE/04): leases, tie-breaks, versions."""

from parakram_tasks import task_pool as tp
import pytest


def pool_with(task='t1', now=0.0):
    pool = tp.TaskPool(bid_window=1.0, award_timeout=2.0)
    pool.add(task, (0.0, 0.0, 0.0), (1.0, 1.0, 0.0), now)
    return pool


def test_announce_bid_award_renew_complete():
    pool = pool_with()
    assert [r.task_id for r in pool.pending()] == ['t1']
    assert pool.on_announce('t1', 1, 'robot2', 1.0)
    pool.on_bid('t1', 1, 'robot1', 30.0)
    pool.on_bid('t1', 1, 'robot3', 20.0)
    pool.on_bid('t1', 7, 'robot2', 1.0)                 # a bid for another round: ignored
    assert pool.pick_winner('t1') == ('robot3', 20.0)
    assert pool.on_award('t1', 1, 'robot3', 12.0, 'robot2', 20.0, 2.0) == (True, None)
    assert pool.holder_of('t1') == 'robot3'
    assert pool.on_renew('t1', 1, 'robot3', 15.0)
    assert not pool.on_renew('t1', 1, 'robot1', 99.0)   # only the holder renews
    assert pool.tasks['t1'].lease_expiry == 15.0
    assert pool.on_complete('t1', 'robot3')
    assert not pool.on_complete('t1', 'robot3')
    assert pool.counts()[tp.DONE] == 1
    # a finished task ignores everything
    assert not pool.on_announce('t1', 2, 'robot1', 20.0)
    assert pool.on_award('t1', 2, 'robot1', 30.0, 'robot1', 1.0, 20.0) == (False, None)


def test_lease_frees_the_task_exactly_at_expiry():
    pool = pool_with()
    pool.on_announce('t1', 1, 'robot1', 0.0)
    pool.on_award('t1', 1, 'robot2', 10.0, 'robot1', 5.0, 1.0)
    assert pool.expire(9.999) == []
    assert pool.holder_of('t1') == 'robot2'
    assert pool.expire(10.0) == [('t1', 'lease_expired')]
    assert pool.tasks['t1'].state == tp.PENDING and pool.holder_of('t1') == ''
    assert pool.is_reauction('t1')                      # its next round is a re-auction
    pool.on_announce('t1', 2, 'robot3', 10.5)
    assert not pool.is_reauction('t1')                  # ...only once
    # a renewal keeps it alive past the first expiry
    pool = pool_with()
    pool.on_announce('t1', 1, 'robot1', 0.0)
    pool.on_award('t1', 1, 'robot2', 10.0, 'robot1', 5.0, 1.0)
    pool.on_renew('t1', 1, 'robot2', 14.0)
    assert pool.expire(10.0) == [] and pool.expire(14.0) == [('t1', 'lease_expired')]


def test_deterministic_tie_break_by_robot_id():
    for order in (('robot3', 'robot1', 'robot2'), ('robot2', 'robot3', 'robot1')):
        pool = pool_with()
        pool.on_announce('t1', 1, 'robot1', 0.0)
        for r in order:
            pool.on_bid('t1', 1, r, 25.0)
        assert pool.pick_winner('t1') == ('robot1', 25.0)
    pool = pool_with()
    pool.on_announce('t1', 1, 'robot1', 0.0)
    pool.on_bid('t1', 1, 'robot2', float('inf'))
    assert pool.pick_winner('t1') is None                # a withheld bid never wins


def test_two_announcers_same_round_lower_id_owns_it():
    pool = pool_with()
    assert pool.on_announce('t1', 1, 'robot3', 0.0)
    assert not pool.on_announce('t1', 1, 'robot1', 0.01)
    assert pool.tasks['t1'].announcer == 'robot1'
    # the award of the non-owner is ignored; the owner's is taken
    assert pool.on_award('t1', 1, 'robot2', 10.0, 'robot3', 4.0, 1.0) == (False, None)
    assert pool.on_award('t1', 1, 'robot3', 10.0, 'robot1', 3.0, 1.0) == (True, None)
    # and a late duplicate award of the same round from a higher announcer changes nothing
    assert pool.on_award('t1', 1, 'robot2', 10.0, 'robot3', 4.0, 1.1) == (False, None)
    assert pool.holder_of('t1') == 'robot3'


def test_duplicate_awards_higher_seq_wins_and_loser_drops():
    pool = pool_with()
    pool.on_announce('t1', 1, 'robot1', 0.0)
    pool.on_award('t1', 1, 'robot2', 10.0, 'robot1', 5.0, 1.0)
    # partition: another robot re-announced and awarded round 2
    assert pool.on_award('t1', 2, 'robot3', 12.0, 'robot3', 6.0, 2.0) == (True, 'robot2')
    assert pool.holder_of('t1') == 'robot3'
    # the older round's messages no longer count
    assert pool.on_award('t1', 1, 'robot2', 20.0, 'robot1', 5.0, 3.0) == (False, None)
    assert not pool.on_renew('t1', 1, 'robot2', 30.0)
    # a newer announcement voids the award in force (its holder must drop)
    assert pool.on_announce('t1', 3, 'robot1', 4.0)
    assert pool.holder_of('t1') == '' and pool.tasks['t1'].state == tp.AUCTION
    assert pool.tasks['t1'].awards == 2


def test_announcer_failure_mid_auction_returns_the_task():
    pool = pool_with()
    pool.on_announce('t1', 1, 'robot1', 0.0)
    pool.on_bid('t1', 1, 'robot2', 9.0)
    assert pool.expire(3.0) == []                        # bid window 1 s + award timeout 2 s
    assert pool.expire(3.01) == [('t1', 'announcer_timeout')]
    assert pool.is_reauction('t1') and pool.tasks['t1'].failed_rounds == 1
    # a stale announcement of the failed round cannot reopen it
    assert not pool.on_announce('t1', 1, 'robot1', 3.5)
    assert pool.on_announce('t1', 2, 'robot3', 4.0)
    # a round nobody bid on is not a re-auction: it just waits for the next round
    pool = pool_with()
    pool.on_announce('t1', 1, 'robot1', 0.0)
    assert pool.expire(3.5) == [('t1', 'no_bids')] and not pool.is_reauction('t1')


def test_release_and_reauction_of_a_dead_robots_tasks():
    pool = tp.TaskPool()
    for t in ('a', 'b', 'c'):
        pool.add(t, (0, 0, 0), (1, 1, 0), 0.0)
        pool.on_announce(t, 1, 'robot1', 0.0)
    pool.on_award('a', 1, 'robot2', 50.0, 'robot1', 1.0, 1.0)
    pool.on_award('b', 1, 'robot2', 50.0, 'robot1', 1.0, 1.0)
    pool.on_award('c', 1, 'robot3', 50.0, 'robot1', 1.0, 1.0)
    assert sorted(pool.release_robot('robot2')) == ['a', 'b']
    assert pool.holder_of('c') == 'robot3'
    assert pool.is_reauction('a') and pool.tasks['a'].back_reason == 'reauction:robot2'
    # a holder may also give a task back
    assert pool.on_renew('c', 1, 'robot3', 0.0, released=True)
    assert pool.tasks['c'].state == tp.PENDING and pool.is_reauction('c')


def test_counts_and_pending_order():
    pool = tp.TaskPool()
    for i, t in enumerate(('x', 'y', 'z')):
        pool.add(t, (0, 0, 0), (1, 1, 0), float(i))
    assert not pool.add('x', (0, 0, 0), (1, 1, 0), 9.0)
    pool.on_announce('y', 1, 'robot1', 5.0)
    assert [r.task_id for r in pool.pending()] == ['x', 'z']
    assert pool.counts() == {tp.PENDING: 2, tp.AUCTION: 1, tp.ASSIGNED: 0, tp.DONE: 0}
    assert pool.pick_winner('y') is None
    assert pool.pick_winner('nope') is None


@pytest.mark.parametrize('seq', [1, 2])
def test_award_for_a_round_whose_announcement_was_missed(seq):
    pool = pool_with()
    assert pool.on_award('t1', seq, 'robot2', 10.0, 'robot3', 2.0, 0.5) == (True, None)
    assert pool.holder_of('t1') == 'robot2' and pool.tasks['t1'].seq == seq
