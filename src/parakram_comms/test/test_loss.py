"""Unit tests of the seeded app-level loss (CLAUDE_CODE/05) and the QoS profiles."""

import csv
import math
from types import SimpleNamespace

from parakram_comms.loss import DropProcess, LossFilter, stream_seed
import pytest


def drops(flt, sender, topic, n):
    return [not flt.accept(sender, topic) for _ in range(n)]


def test_zero_loss_passes_everything_and_counts():
    flt = LossFilter(0.0, 1, 'robot1')
    assert not any(drops(flt, 'robot2', 'intent', 1000))
    assert flt.counts[('robot2', 'intent')][:2] == [1000, 0]


@pytest.mark.parametrize('loss', [0.1, 0.3, 0.6])
def test_bernoulli_rate_matches_the_setting(loss):
    flt = LossFilter(loss, 7, 'robot1')
    got = sum(drops(flt, 'robot2', 'intent', 50000)) / 50000
    assert got == pytest.approx(loss, abs=0.01)


def test_drops_are_deterministic_per_seed_and_stream():
    a = drops(LossFilter(0.4, 3, 'robot1'), 'robot2', 'intent', 500)
    b = drops(LossFilter(0.4, 3, 'robot1'), 'robot2', 'intent', 500)
    assert a == b                                          # same run seed: same drops
    assert a != drops(LossFilter(0.4, 4, 'robot1'), 'robot2', 'intent', 500)
    assert a != drops(LossFilter(0.4, 3, 'robot1'), 'robot3', 'intent', 500)
    assert a != drops(LossFilter(0.4, 3, 'robot1'), 'robot2', 'state', 500)
    # streams are independent: interleaving another stream does not change this one
    flt = LossFilter(0.4, 3, 'robot1')
    c = []
    for _ in range(500):
        flt.accept('robot3', 'state')
        c.append(not flt.accept('robot2', 'intent'))
    assert c == a
    assert stream_seed(3, 'robot1', 'robot2', 'intent') == stream_seed(3, 'robot1', 'robot2',
                                                                       'intent')


@pytest.mark.parametrize('loss', [0.2, 0.5])
def test_gilbert_elliott_keeps_the_mean_and_is_bursty(loss):
    rho = 0.8
    flt = LossFilter(loss, 11, 'robot1', model='gilbert_elliott', burst_corr=rho)
    d = drops(flt, 'robot2', 'intent', 200000)
    assert sum(d) / len(d) == pytest.approx(loss, abs=0.02)
    bursts, run = [], 0
    for x in d:
        if x:
            run += 1
        elif run:
            bursts.append(run)
            run = 0
    p_bg = (1 - loss) * (1 - rho)
    assert sum(bursts) / len(bursts) == pytest.approx(1 / p_bg, rel=0.1)
    count = flt.counts[('robot2', 'intent')]
    assert count[4] == len(bursts) + (1 if run else 0)      # the logged burst counter agrees
    # lag-1 correlation of the drop indicator ~ rho (Bernoulli would give ~0)
    m = sum(d) / len(d)
    cov = sum((d[i] - m) * (d[i + 1] - m) for i in range(len(d) - 1)) / (len(d) - 1)
    assert cov / (m * (1 - m)) == pytest.approx(rho, abs=0.03)


def test_invalid_settings_are_refused():
    for bad in (-0.1, 1.0, 1.5):
        with pytest.raises(ValueError):
            LossFilter(bad, 1, 'robot1')
    with pytest.raises(ValueError):
        LossFilter(0.1, 1, 'robot1', model='uniform')
    with pytest.raises(ValueError):
        LossFilter(0.1, 1, 'robot1', model='gilbert_elliott', burst_corr=1.0)
    assert not DropProcess(0.0, None).drop()


def test_wrap_drops_before_the_callback_and_logs(tmp_path):
    path = tmp_path / 'comms_robot1.csv'
    flt = LossFilter(0.5, 2, 'robot1', log_path=str(path))
    seen = []
    cb = flt.wrap('robot2', 'intent', seen.append)
    for i in range(1000):
        cb(i)
    rec, dropped = flt.counts[('robot2', 'intent')][:2]
    assert rec == 1000 and len(seen) == rec - dropped
    assert seen == sorted(seen)                     # order of the survivors is kept
    # the sender's seq span says what was published, whatever the transport lost
    cb3 = flt.wrap('robot3', 'intent', lambda m: None)
    for q in range(100, 200, 2):                    # the transport lost every other message
        cb3(SimpleNamespace(seq=q))
    flt.log(12.5)
    last = {r['sender']: r for r in csv.DictReader(open(path))}
    assert int(last['robot2']['passed']) == len(seen) and last['robot2']['first_seq'] == ''
    assert (last['robot3']['first_seq'], last['robot3']['last_seq']) == ('100', '198')
    assert int(last['robot3']['received']) == 50
    # Bernoulli bursts: mean length 1 / (1 - loss) = 2 at 50 %
    assert int(last['robot2']['dropped']) / int(last['robot2']['drop_bursts']) == \
        pytest.approx(2.0, rel=0.15)
    assert float(last['robot2']['loss']) == 0.5 and last['robot2']['seed'] == '2'


def test_qos_profiles_match_the_spec():
    # imported here: rclpy at collection time makes the flake8 test fork a threaded process
    from parakram_comms import qos
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, LivelinessPolicy, ReliabilityPolicy
    for prof, depth in ((qos.STATE_QOS, 1), (qos.INTENT_QOS, 5)):
        assert prof.reliability == ReliabilityPolicy.BEST_EFFORT       # loss must be visible
        assert prof.durability == DurabilityPolicy.VOLATILE
        assert prof.history == HistoryPolicy.KEEP_LAST and prof.depth == depth
    assert qos.TASK_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert qos.TASK_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.TASK_QOS.history == HistoryPolicy.KEEP_ALL
    assert qos.TASK_POOL_QOS is qos.TASK_QOS
    hb = qos.HEARTBEAT_QOS
    assert hb.reliability == ReliabilityPolicy.BEST_EFFORT
    assert hb.liveliness == LivelinessPolicy.MANUAL_BY_TOPIC
    assert math.isclose(hb.liveliness_lease_duration.nanoseconds * 1e-9, qos.HEARTBEAT_LEASE_S)
