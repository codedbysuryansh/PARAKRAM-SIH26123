"""Unit tests of the CLAUDE_CODE/06 windowed watchdog."""

from parakram_fault.watchdog import (ALIVE, DEAD, FLAKY, PARTITIONED, SUSPECT,
                                     WindowedWatchdog)
import pytest


def feed(dog, peer, t0, t1, rate=10.0, keep=lambda i: True):
    seq = 0
    t = t0
    while t < t1:
        seq += 1
        if keep(seq):
            dog.on_renewal(peer, t, seq)
        t += 1.0 / rate
    return seq


def test_alive_flaky_suspect_dead_with_partition_grace():
    dog = WindowedWatchdog(detect_timeout=1.0, partition_grace=10.0)
    feed(dog, 'robot2', 0.0, 5.0)
    assert dog.status('robot2', 5.0)[0] == ALIVE
    feed(dog, 'robot3', 0.0, 5.0, keep=lambda i: i % 3 == 0)         # 1/3 of the renewals
    assert dog.status('robot3', 5.0)[0] == FLAKY
    assert dog.status('robot2', 5.9)[0] == ALIVE
    assert dog.status('robot2', 6.1)[0] == SUSPECT                   # silent > 1 s
    assert dog.status('robot2', 14.8)[0] == SUSPECT                  # not yet dead
    assert dog.status('robot2', 15.1)[0] == DEAD                     # silent > grace everywhere


def test_partitioned_peer_is_not_declared_dead():
    dog = WindowedWatchdog(detect_timeout=1.0, partition_grace=10.0)
    last = feed(dog, 'robot2', 0.0, 5.0)
    # robot3 keeps acknowledging NEW renewals of robot2 that never reach this robot
    for k in range(1, 200):
        t = 5.0 + 0.1 * k
        dog.on_third_party_ack('robot2', last + k, t)
        status = dog.status('robot2', t)[0]
        assert status in (ALIVE, PARTITIONED) and status != DEAD
    assert dog.status('robot2', 24.9)[0] == PARTITIONED
    # the third party's acks stop advancing (robot2 died): suspect, then dead after the grace
    assert dog.status('robot2', 26.0)[0] == SUSPECT
    assert dog.status('robot2', 35.0)[0] == DEAD


def test_stale_third_party_acks_are_no_evidence_of_life():
    dog = WindowedWatchdog(detect_timeout=1.0, partition_grace=10.0)
    last = feed(dog, 'robot2', 0.0, 5.0)
    for k in range(100):                                              # the same old seq, repeated
        dog.on_third_party_ack('robot2', last, 5.0 + 0.1 * k)
    assert dog.status('robot2', 15.1)[0] == DEAD


def test_baseline_policy_declares_dead_at_detection():
    dog = WindowedWatchdog(detect_timeout=1.0, partition_grace=10.0, policy='detect')
    feed(dog, 'robot2', 0.0, 5.0)
    assert dog.status('robot2', 5.5)[0] == ALIVE
    assert dog.status('robot2', 6.1)[0] == DEAD
    with pytest.raises(ValueError):
        WindowedWatchdog(policy='vote')


def test_delivery_estimate_is_windowed():
    dog = WindowedWatchdog(rate_hz=10.0, window_s=2.0)
    feed(dog, 'robot2', 0.0, 10.0, keep=lambda i: i % 2 == 0)
    _, age, delivery, others = dog.status('robot2', 10.0)
    assert delivery == pytest.approx(0.5, abs=0.06) and age < 0.25 and others == float('inf')
