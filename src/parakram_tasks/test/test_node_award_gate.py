"""
Unit tests of the CLAUDE_CODE/06 quorum-lease rules inside one ``AuctionNode`` (no spinning).

The node runs in lease mode with the award gate on; its inputs (coordination status, peers'
intents, ReAuction requests) are fed directly, at explicit times.

(The module name sorts after ``test_flake8``: flake8 forks worker processes, and a fork taken
after DDS participants have run in this process crashes, as in parakram_safety's node tests.)
"""

import os

from parakram_msgs.msg import CoordStatus, Intent, TaskAnnounce
from parakram_msgs.srv import ReAuction
from parakram_sim.grid_utils import WarehouseGrid
from parakram_tasks import task_pool as tp
from parakram_tasks.auction_node import _stamp, AuctionNode, Job
import pytest
import rclpy
from rclpy.parameter import Parameter

GRID_YAML = os.path.join(os.path.dirname(__file__), '..', '..', 'parakram_sim', 'config',
                         'warehouse_grid.yaml')


def _make(mode):
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=97)
    params = [Parameter('robot_id', value='robot1'),
              Parameter('peers', value=['robot1', 'robot2', 'robot3']),
              Parameter('recovery_mode', value=mode), Parameter('award_gate', value=True),
              Parameter('award_lease_ttl', value=10.0), Parameter('award_margin', value=2.0),
              Parameter('grid_yaml', value=GRID_YAML)]
    return ctx, AuctionNode(namespace='robot1', context=ctx, parameter_overrides=params)


@pytest.fixture
def node():
    ctx, n = _make('lease')
    yield n
    n.close()
    n.destroy_node()
    rclpy.shutdown(context=ctx)


@pytest.fixture
def baseline_node():
    ctx, n = _make('release')
    yield n
    n.close()
    n.destroy_node()
    rclpy.shutdown(context=ctx)


def add_task(n, task_id):
    grid = WarehouseGrid.from_yaml(GRID_YAML)
    x0, y0 = grid.cell_to_world(3, 3)
    x1, y1 = grid.cell_to_world(7, 9)
    n.pool.add(task_id, (x0, y0, 0.0), (x1, y1, 0.0), 0.0)


def status(t, valid=True, task=('', 0), acked=0.0):
    st = CoordStatus()
    _stamp_into(st.stamp, t)
    st.lease_valid = valid
    st.task_id, st.task_seq = task
    _stamp_into(st.task_renewal_acked, acked)
    return st


def _stamp_into(field, t):
    s = _stamp(t)
    field.sec, field.nanosec = s.sec, s.nanosec


def test_cut_off_robot_stops_its_award_clock_and_takes_no_part_in_auctions(node):
    t = node._now()
    add_task(node, 't1')
    node.pool.on_award('t1', 1, 'robot2', t + 10.0, 'robot2', 1.0, t)
    node.cell = (5, 4)
    node.coord = status(t, valid=False)                   # no quorum / not acknowledged
    assert not node._auction_ok(t) and node.paused_since == t
    add_task(node, 't2')
    node.pool.on_announce('t2', 1, 'robot3', t)
    node._try_bid(t)
    assert node.my_bid is None                            # takes no task while cut off
    node.coord = status(t + 20.0)                         # back: the clock resumes
    assert node._auction_ok(t + 20.0) and node.paused_since is None
    assert node.pool.tasks['t1'].lease_expiry == pytest.approx(t + 30.0)
    node.pool.expire(t + 25.0)
    assert node.pool.tasks['t1'].state == tp.ASSIGNED    # not freed by its own isolation
    node.pool.on_announce('t2', 2, 'robot3', t + 20.0)
    node._try_bid(t + 20.0)
    assert node.my_bid == ('t2', 2)


def test_third_party_ack_of_a_new_renewal_keeps_the_holders_award(node):
    t = node._now()
    add_task(node, 't1')
    node.pool.on_award('t1', 1, 'robot2', t + 1.0, 'robot2', 1.0, t)
    rec = node.pool.tasks['t1']
    ack = Intent(robot_id='robot3', seq=5, ack_ids=['robot1', 'robot2'], ack_seqs=[3, 7])
    node._on_peer_intent('robot3', ack)
    assert rec.lease_expiry >= t + 10.0                   # robot2 alive, heard by robot3
    rec.lease_expiry = t + 1.0
    node._on_peer_intent('robot3', Intent(robot_id='robot3', seq=6, ack_ids=['robot2'],
                                          ack_seqs=[7]))
    assert rec.lease_expiry == t + 1.0                    # the same renewal proves nothing new
    node._on_peer_intent('robot2', Intent(robot_id='robot2', seq=9, ack_ids=['robot3'],
                                          ack_seqs=[4]))
    assert node.heard_seq['robot2'] == 9                  # heard directly: newest seq known
    node._on_peer_intent('robot3', Intent(robot_id='robot3', seq=7, ack_ids=['robot2'],
                                          ack_seqs=[8]))
    assert rec.lease_expiry == t + 1.0                    # older than one heard directly


def test_reauction_announces_only_tasks_whose_award_ran_out_here(node):
    t = node._now()
    add_task(node, 't1')
    node.pool.on_award('t1', 1, 'robot2', t + 5.0, 'robot2', 1.0, t)
    node.coord = status(node._now())
    req = ReAuction.Request()
    req.dead_robot_id = 'robot2'
    assert node._on_reauction(req, ReAuction.Response()).tasks_reannounced == 0
    assert node.pool.tasks['t1'].state == tp.ASSIGNED     # its holder may still hold it
    node.pool.tasks['t1'].lease_expiry = t - 1.0
    node.coord = status(node._now(), valid=False)
    assert node._on_reauction(req, ReAuction.Response()).tasks_reannounced == 0
    node.coord = status(node._now())
    assert node._on_reauction(req, ReAuction.Response()).tasks_reannounced == 1
    rec = node.pool.tasks['t1']
    assert (rec.state, rec.seq, rec.announcer) == (tp.AUCTION, 2, 'robot1')


def test_holder_releases_its_award_without_a_majority_acknowledgement(node):
    now = node._now()
    add_task(node, 't1')
    node.pool.on_award('t1', 1, 'robot1', now + 10.0, 'robot2', 1.0, now)
    node.job = Job('t1', 1, 1.0, now + 10.0, now - 9.0)
    node.coord = status(now, task=('t1', 1), acked=now - 1.0)
    assert node._award_gate(now) == 'ok'
    node.coord = status(now, task=('t1', 1), acked=now - 8.5)
    assert node._award_gate(now) == 'drop'                # a majority last heard it 8.5 s ago
    node.coord = status(now, task=('t1', 1), acked=0.0)
    assert node._award_gate(now) == 'drop'                # never acknowledged within 8 s
    node.job.started = now - 1.0
    assert node._award_gate(now) == 'wait'
    node.coord = status(now, task=('t1', 1), acked=now - 0.2)
    node.job.started = now - 0.5
    assert node._award_gate(now) == 'wait'                # acknowledged, but still settling
    node.job.started = now - 1.5
    assert node._award_gate(now) == 'ok'


def test_baseline_reauction_reannounces_a_task_another_robot_reopened(baseline_node):
    n = baseline_node
    assert not n.award_gate                               # the gate is lease-mode only
    t = n._now()
    add_task(n, 't1')
    n.pool.on_award('t1', 4, 'robot3', t + 10.0, 'robot2', 1.0, t)
    released = []
    n.pub_release.publish = lambda msg: released.append(msg.data)
    n._on_announce(TaskAnnounce(task_id='t1', announcer_id='robot2', seq=5))
    assert released == ['robot3']                         # another robot's newer round
    req = ReAuction.Request()
    req.dead_robot_id = 'robot3'
    # robot2 re-opened it first: this robot still sends its own round, which frees the dead
    # robot's space at robot2 (its own announcement never counts there)
    assert n._on_reauction(req, ReAuction.Response()).tasks_reannounced == 1
    rec = n.pool.tasks['t1']
    assert (rec.state, rec.seq, rec.announcer) == (tp.AUCTION, 6, 'robot1')
