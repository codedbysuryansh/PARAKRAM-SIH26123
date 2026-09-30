"""Unit tests of the CLAUDE_CODE/06 task-pool additions: intent renewals, digest, baseline."""

from parakram_tasks import task_pool as tp


def pool_with(task='t1', lease_enabled=True):
    p = tp.TaskPool(bid_window=1.0, award_timeout=2.0, lease_enabled=lease_enabled)
    p.add(task, (0.0, 0.0, 0.0), (1.0, 1.0, 0.0), 0.0)
    return p


def test_holder_renewal_extends_and_teaches_a_missed_award():
    p = pool_with()
    p.on_announce('t1', 1, 'robot2', 0.0)
    # the Award was lost here, but robot3's renewal says it holds round 1
    assert p.on_holder_renewal('t1', 1, 'robot3', 12.0) == (True, None)
    rec = p.tasks['t1']
    assert (rec.state, rec.winner, rec.award_seq, rec.lease_expiry) == \
        (tp.ASSIGNED, 'robot3', 1, 12.0)
    assert p.on_holder_renewal('t1', 1, 'robot3', 15.0)[0] and rec.lease_expiry == 15.0
    assert p.on_holder_renewal('t1', 1, 'robot2', 20.0) == (False, None)   # same round, other
    assert p.on_holder_renewal('t1', 0, 'robot3', 30.0) == (False, None)   # older round
    # a newer round's holder supersedes (at most one executor)
    assert p.on_holder_renewal('t1', 2, 'robot2', 20.0) == (True, 'robot3')
    p.on_complete('t1', 'robot2')
    assert p.on_holder_renewal('t1', 3, 'robot1', 40.0) == (False, None)   # done is final


def test_digest_round_trip_completions_are_final_and_newer_rounds_win():
    a, b = pool_with(), pool_with()
    for p in (a, b):
        p.add('t2', (0.0, 0.0, 0.0), (1.0, 1.0, 0.0), 0.0)
    a.on_award('t1', 1, 'robot1', 10.0, 'robot1', 5.0, 0.0)
    a.on_complete('t1', 'robot1')
    a.on_award('t2', 3, 'robot2', 20.0, 'robot3', 4.0, 0.0)
    b.on_award('t2', 2, 'robot3', 20.0, 'robot3', 4.0, 0.0)          # b missed round 3
    done, awards = a.digest()
    assert done == [('t1', 'robot1')] and awards == [('t2', 'robot2', 3, 20.0, 'robot3')]
    completed, superseded = b.apply_digest(done, awards, 1.0)
    assert completed == ['t1'] and b.tasks['t1'].state == tp.DONE
    assert superseded == [('t2', 'robot3')] and b.tasks['t2'].winner == 'robot2'
    assert b.apply_digest(done, awards, 2.0) == ([], [])                 # idempotent


def test_digest_never_wins_a_same_round_tie_or_extends_by_itself():
    p = pool_with()
    p.on_award('t1', 2, 'robot1', 10.0, 'robot1', 5.0, 0.0)
    assert p.apply_digest([], [('t1', 'robot3', 2, 50.0)], 1.0) == ([], [])
    assert p.tasks['t1'].winner == 'robot1' and p.tasks['t1'].lease_expiry == 10.0
    # a digest echoing the known award never extends it: only its holder's renewals do
    p.apply_digest([], [('t1', 'robot1', 2, 30.0)], 1.0)
    assert p.tasks['t1'].lease_expiry == 10.0


def test_baseline_pool_has_no_award_lease():
    p = pool_with(lease_enabled=False)
    p.on_award('t1', 1, 'robot2', 5.0, 'robot1', 3.0, 0.0)
    assert p.expire(100.0) == [] and p.tasks['t1'].state == tp.ASSIGNED
    assert p.release_robot('robot2') == ['t1']                          # only ReAuction frees
    leased = pool_with()
    leased.on_award('t1', 1, 'robot2', 5.0, 'robot1', 3.0, 0.0)
    assert leased.expire(5.0) == [('t1', 'lease_expired')]


def test_third_party_acknowledgement_keeps_a_partitioned_holders_award():
    p = pool_with()
    p.add('t2', (0.0, 0.0, 0.0), (1.0, 1.0, 0.0), 0.0)
    p.on_award('t1', 1, 'robot2', 10.0, 'robot1', 3.0, 0.0)
    p.on_award('t2', 1, 'robot3', 10.0, 'robot1', 3.0, 0.0)
    assert p.extend_holder('robot2', 18.0) == 1          # robot3 acked a new renewal of robot2
    assert p.extend_holder('robot2', 12.0) == 1          # never shortens
    assert p.tasks['t1'].lease_expiry == 18.0
    assert p.expire(15.0) == [('t2', 'lease_expired')]   # robot3's award ran out
    assert p.tasks['t1'].state == tp.ASSIGNED
    assert p.extend_holder('robot3', 30.0) == 0          # a freed task is not revived


def test_award_clock_stops_while_this_robot_is_cut_off():
    p = pool_with()
    p.on_award('t1', 1, 'robot2', 10.0, 'robot1', 3.0, 0.0)
    p.shift_leases(30.0 - 1.7)                           # cut off from 1.7 s to 30 s
    assert p.expire(30.0) == []
    assert p.expire(38.25) == [] and p.expire(38.3) == [('t1', 'lease_expired')]


def test_two_awards_of_one_round_settled_by_the_lower_announcer_on_renewals():
    # robot2 announced and awarded round 1 to itself; robot1 did the same, each missing the
    # other (loss); robot2's pool learns of robot1's award only from robot1's renewals
    p = pool_with()
    p.on_announce('t1', 1, 'robot2', 0.0)
    assert p.on_award('t1', 1, 'robot2', 10.0, 'robot2', 3.0, 0.0) == (True, None)
    assert p.on_holder_renewal('t1', 1, 'robot1', 12.0) == (False, None)            # no announcer
    assert p.on_holder_renewal('t1', 1, 'robot1', 12.0, 'robot3') == (False, None)  # higher one
    assert p.on_holder_renewal('t1', 1, 'robot1', 12.0, 'robot1') == (True, 'robot2')
    rec = p.tasks['t1']
    assert (rec.winner, rec.award_announcer, rec.award_seq) == ('robot1', 'robot1', 1)
    assert p.on_holder_renewal('t1', 1, 'robot2', 14.0, 'robot2') == (False, None)  # loser
    # the winner's own renewals keep extending it and teach its announcer
    q = pool_with()
    q.on_holder_renewal('t1', 1, 'robot1', 5.0)                       # learnt without announcer
    assert q.tasks['t1'].award_announcer == '~'
    assert q.on_holder_renewal('t1', 1, 'robot1', 6.0, 'robot1') == (True, None)
    assert q.tasks['t1'].award_announcer == 'robot1' and q.tasks['t1'].lease_expiry == 6.0


def test_renewal_of_a_round_owned_by_a_lower_announcer_is_not_taken_as_the_award():
    p = pool_with()
    p.on_announce('t1', 1, 'robot1', 0.0)                              # robot1 owns round 1
    assert p.on_holder_renewal('t1', 1, 'robot3', 12.0, 'robot2') == (False, None)
    assert p.tasks['t1'].state == tp.AUCTION
    assert p.on_holder_renewal('t1', 1, 'robot3', 12.0, 'robot1') == (True, None)


def test_digest_with_announcers_settles_a_same_round_tie():
    a, b = pool_with(), pool_with()
    a.on_award('t1', 1, 'robot1', 10.0, 'robot1', 3.0, 0.0)
    b.on_award('t1', 1, 'robot2', 10.0, 'robot2', 3.0, 0.0)
    done, awards = a.digest()
    assert awards == [('t1', 'robot1', 1, 10.0, 'robot1')]
    loser = b.digest()
    assert b.apply_digest(done, awards, 1.0) == ([], [('t1', 'robot2')])
    assert b.tasks['t1'].winner == 'robot1'
    # the loser's digest never takes it back, and an announcer-less entry never wins
    assert a.apply_digest(*loser, 2.0) == ([], []) and a.tasks['t1'].winner == 'robot1'
    c = pool_with()
    c.on_award('t1', 1, 'robot2', 10.0, 'robot2', 3.0, 0.0)
    assert c.apply_digest([], [('t1', 'robot1', 1, 10.0)], 1.0) == ([], [])
    assert c.tasks['t1'].winner == 'robot2'
