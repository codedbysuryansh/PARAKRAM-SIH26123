"""Unit tests of the fleet-wide per-link loss process (CLAUDE_CODE/06)."""

from parakram_comms.link_loss import LinkChannel, LinkLoss, parse_fault_command, SLOT_S
import pytest


def rate(ll, sender, n=40000, dt=0.1):
    return sum(ll.dropped(sender, 1.0 + i * dt) for i in range(n)) / n


@pytest.mark.parametrize('loss', [0.1, 0.3, 0.6])
def test_bernoulli_rate_and_determinism(loss):
    a = LinkLoss(loss, 5, 'robot1')
    assert rate(a, 'robot2') == pytest.approx(loss, abs=0.01)
    # another process of the same robot (same seed) sees exactly the same fades
    b = LinkLoss(loss, 5, 'robot1')
    ts = [0.37 + 0.1 * i for i in range(3000)]
    assert [a.dropped('robot2', t) for t in ts] == [b.dropped('robot2', t) for t in ts]
    # ... evaluated in any order
    c = LinkLoss(loss, 5, 'robot1')
    assert [c.dropped('robot2', t) for t in reversed(ts)][::-1] == \
        [a.dropped('robot2', t) for t in ts]
    # other links and other seeds differ
    assert [a.dropped('robot3', t) for t in ts] != [a.dropped('robot2', t) for t in ts]
    assert [LinkLoss(loss, 6, 'robot1').dropped('robot2', t) for t in ts] != \
        [a.dropped('robot2', t) for t in ts]


def test_one_fate_per_slot_for_every_topic():
    ll = LinkLoss(0.5, 3, 'robot1')
    for i in range(500):
        t = 2.0 + i * SLOT_S + 0.004
        intent = ll.accept('robot2', 'intent', t)
        heartbeat = ll.accept('robot2', 'heartbeat', t + 0.01)   # relayed 10 ms later
        assert intent == heartbeat
    assert ll.counts[('robot2', 'intent')][1] == ll.counts[('robot2', 'heartbeat')][1]


def test_gilbert_elliott_mean_and_burstiness():
    rho = 0.8
    ch = LinkChannel(0.4, 'gilbert_elliott', rho, 11)
    d = [ch.dropped(0.5 + 0.1 * i) for i in range(100000)]     # sampled at 10 Hz
    m = sum(d) / len(d)
    assert m == pytest.approx(0.4, abs=0.02)
    cov = sum((d[i] - m) * (d[i + 1] - m) for i in range(len(d) - 1)) / (len(d) - 1)
    assert cov / (m * (1 - m)) == pytest.approx(rho, abs=0.03)


def test_partitions_isolate_one_robot_or_everyone():
    ll = LinkLoss(0.0, 1, 'robot1')
    ll.add_partition('robot2', 10.0, 20.0)
    assert not ll.dropped('robot2', 9.99) and ll.dropped('robot2', 10.0)
    assert ll.dropped('robot2', 19.99) and not ll.dropped('robot2', 20.0)
    assert not ll.dropped('robot3', 15.0)                 # robot3 <-> robot1 untouched
    r2 = LinkLoss(0.0, 1, 'robot2')                        # the isolated robot hears nobody
    r2.add_partition('robot2', 10.0, 20.0)
    assert r2.dropped('robot1', 15.0) and r2.dropped('robot3', 15.0)
    ll.add_partition('*', 30.0, 35.0)                      # fleet-wide blackout
    assert ll.dropped('robot3', 31.0) and not ll.dropped('robot3', 35.0)
    assert not ll.dropped('robot1', 31.0)                  # own messages are not a link


def test_wrap_by_field_and_counters(tmp_path):
    class Msg:
        def __init__(self, who, seq):
            self.announcer_id, self.seq = who, seq
    ll = LinkLoss(0.3, 2, 'robot1', log_path=str(tmp_path / 'c.csv'))
    seen = []
    cb = ll.wrap_by_field('announcer_id', 'task_announce', seen.append, lambda: 5.0)
    cb(Msg('robot1', 1))                                   # own: always delivered
    assert len(seen) == 1
    ll.add_partition('robot2', 0.0, 100.0)
    cb(Msg('robot2', 2))
    assert len(seen) == 1 and ll.counts[('robot2', 'task_announce')][:2] == [1, 1]
    ll.log(5.0)
    assert (tmp_path / 'c.csv').read_text().count('bernoulli@link') == 2


def test_invalid_settings_and_commands():
    for bad in (-0.1, 1.0):
        with pytest.raises(ValueError):
            LinkLoss(bad, 1, 'robot1')
    with pytest.raises(ValueError):
        LinkLoss(0.1, 1, 'robot1', model='uniform')
    assert parse_fault_command('{"partition": "robot2", "t0": 5, "t1": 35}') == \
        [('robot2', 5.0, 35.0)]
    assert parse_fault_command('[{"partition": "*", "t0": 1, "t1": 6}]') == [('*', 1.0, 6.0)]
