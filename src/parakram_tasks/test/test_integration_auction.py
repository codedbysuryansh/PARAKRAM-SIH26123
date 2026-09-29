"""
Integration tests of the decentralized auction (CLAUDE_CODE/04), in-process, no simulator.

Three real ``AuctionNode``s and the real ``TaskGenerator`` talk over the real fleet topics;
three fake robots stand in for coordination: they take the ``assigned_task`` goal and walk the
shortest grid path, one cell per step, reporting ``coord_status``. Each test runs in its own
ROS domain, with protocol times scaled down (wall clock).

(The module name sorts after ``test_flake8``: flake8 forks worker processes, and a fork taken
after DDS participants have run in this process crashes, as in parakram_safety's node tests.)
"""

from collections import deque
import csv
import itertools
import os
import time

from geometry_msgs.msg import Pose2D
from parakram_msgs.msg import Award, CoordStatus, RobotState, TaskAnnounce, TaskComplete
from parakram_msgs.srv import ReAuction
from parakram_sim.grid_utils import WarehouseGrid
from parakram_tasks.auction_node import AuctionNode, LOG_COLUMNS
from parakram_tasks.task_generator import TaskGenerator
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

GRID_YAML = os.path.join(os.path.dirname(__file__), '..', '..', 'parakram_sim', 'config',
                         'warehouse_grid.yaml')
ROBOTS = ['robot1', 'robot2', 'robot3']
STARTS = {'robot1': (5, 2), 'robot2': (5, 10), 'robot3': (0, 6)}
FAST = {'bid_window': 0.3, 'award_timeout': 0.5, 'award_lease_ttl': 1.5, 'renew_period': 0.3,
        'announce_jitter_min': 0.02, 'announce_jitter_max': 0.15, 'backoff_max': 0.6,
        'service_time': 0.05, 'stall_timeout': 5.0, 'tick_hz': 20.0}
_domains = itertools.count(71)


def _path(grid, a, b):
    prev, queue = {a: None}, deque([a])
    while queue:
        cur = queue.popleft()
        if cur == b:
            break
        for n in grid.neighbors(*cur):
            if n not in prev:
                prev[n] = cur
                queue.append(n)
    out = [b]
    while prev[out[-1]] is not None:
        out.append(prev[out[-1]])
    return out[::-1][1:]


class FakeRobot:
    """Coordination stand-in: walks to the latest assigned goal, one cell per step."""

    def __init__(self, node, ns, grid, start):
        """Publish coord_status / state in ``ns``; take goals from ``ns/assigned_task``."""
        self.node, self.grid, self.cell, self.goal, self.path = node, grid, start, start, []
        self.pub_status = node.create_publisher(CoordStatus, f'/{ns}/coord_status', 10)
        self.pub_state = node.create_publisher(RobotState, f'/{ns}/state', 10)
        node.create_subscription(Pose2D, f'/{ns}/assigned_task', self._on_goal, 10)

    def _on_goal(self, msg):
        self.goal = self.grid.world_to_cell(msg.x, msg.y)
        self.path = _path(self.grid, self.cell, self.goal)

    def step(self):
        if self.path:
            self.cell = self.path.pop(0)
        st = CoordStatus()
        st.cell = self.grid.cell_to_index(*self.cell)
        st.goal_cell = self.grid.cell_to_index(*self.goal)
        self.pub_status.publish(st)
        state = RobotState()
        state.battery = 1.0
        self.pub_state.publish(state)


class Fleet:
    """Auction nodes + fake robots + the generator in one private ROS domain."""

    def __init__(self, log_dir, n_tasks=20, task_rate=10.0):
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=next(_domains))
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.grid = WarehouseGrid.from_yaml(GRID_YAML)
        self.log_dir = str(log_dir)
        self.auction = {}
        for ns in ROBOTS:
            params = [Parameter(k, value=v) for k, v in FAST.items()]
            params += [Parameter('robot_id', value=ns), Parameter('peers', value=ROBOTS),
                       Parameter('log_dir', value=self.log_dir),
                       Parameter('grid_yaml', value=GRID_YAML), Parameter('seed', value=7)]
            node = AuctionNode(namespace=ns, context=self.ctx, parameter_overrides=params)
            self.auction[ns] = node
            self.ex.add_node(node)
        self.world = rclpy.create_node('fake_world', context=self.ctx)
        self.ex.add_node(self.world)
        self.robots = {ns: FakeRobot(self.world, ns, self.grid, STARTS[ns]) for ns in ROBOTS}
        self.gen = TaskGenerator(context=self.ctx, parameter_overrides=[
            Parameter('n_tasks', value=n_tasks), Parameter('task_rate', value=task_rate),
            Parameter('seed', value=3), Parameter('start_delay', value=0.3),
            Parameter('scenario', value='warehouse_stream'),
            Parameter('grid_yaml', value=GRID_YAML)])
        self.ex.add_node(self.gen)
        self.announces, self.awards, self.completes = [], [], []
        self.world.create_subscription(TaskAnnounce, '/fleet/task_announce',
                                       lambda m: self.announces.append(m), 100)
        self.world.create_subscription(Award, '/fleet/award', lambda m: self.awards.append(m),
                                       100)
        self.world.create_subscription(TaskComplete, '/fleet/task_complete',
                                       lambda m: self.completes.append(m), 100)
        self.world.create_timer(0.01, self._step)

    def _step(self):
        for r in self.robots.values():
            r.step()

    def kill(self, ns):
        """SIGKILL stand-in: the node vanishes without a word."""
        node = self.auction.pop(ns)
        self.ex.remove_node(node)
        node.destroy_node()

    def spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self.ex.spin_once(timeout_sec=0.01)
            if cond():
                return True
        return False

    def completed(self):
        return {m.task_id for m in self.completes}

    def log(self):
        with open(os.path.join(self.log_dir, 'tasks.csv')) as f:
            return list(csv.reader(f))

    def close(self):
        for node in list(self.auction.values()):
            node.close()
        self.ex.shutdown()
        for node in list(self.auction.values()) + [self.world, self.gen]:
            node.destroy_node()
        rclpy.shutdown(context=self.ctx)


@pytest.fixture
def fleet_factory(tmp_path):
    made = []

    def make(**kw):
        fleet = Fleet(tmp_path, **kw)
        made.append(fleet)
        return fleet
    yield make
    for f in made:
        f.close()


def _events(rows, event):
    return [dict(zip(rows[0], r)) for r in rows[1:] if r[1] == event]


def test_twenty_streamed_tasks_all_completed_each_awarded_once(fleet_factory):
    fleet = fleet_factory(n_tasks=20)
    assert fleet.spin_until(lambda: len(fleet.completed()) == 20, 90.0), \
        sorted(fleet.completed())
    fleet.spin_until(lambda: False, 0.5)             # let the last renewals / logs settle
    rows = fleet.log()
    assert rows[0] == LOG_COLUMNS
    awards = _events(rows, 'award')
    completes = _events(rows, 'complete')
    reauctions = _events(rows, 'reauction')
    tasks = {f'task_{i:03d}' for i in range(1, 21)}
    assert {e['task_id'] for e in completes} == tasks and len(completes) == 20
    for t in tasks:
        n_awards = sum(1 for e in awards if e['task_id'] == t)
        n_re = sum(1 for e in reauctions if e['task_id'] == t)
        assert 1 <= n_awards <= 1 + n_re, (t, n_awards, n_re)
    # decentralized: announcements and awards came from several robots, never the generator
    announcers = {m.announcer_id for m in fleet.announces}
    assert len(announcers) >= 2 and announcers <= set(ROBOTS), announcers
    assert {m.winner_id for m in fleet.awards} <= set(ROBOTS)
    # every replica converged on the same finished pool
    for node in fleet.auction.values():
        assert node.pool.counts()['done'] == 20
    for kind in ('announce', 'bid', 'award', 'renew', 'complete'):
        assert _events(rows, kind), kind


def test_announcer_killed_mid_auction_task_is_served_by_a_peer(fleet_factory):
    fleet = fleet_factory(n_tasks=6, task_rate=2.0)
    victim = {}

    def on_announce(msg):
        if not victim and len(fleet.completed()) >= 1 and msg.announcer_id in fleet.auction:
            victim.update(robot=msg.announcer_id, task=msg.task_id, seq=msg.seq)
            fleet.kill(msg.announcer_id)             # dies before its bid window closes
    fleet.world.create_subscription(TaskAnnounce, '/fleet/task_announce', on_announce, 100)
    assert fleet.spin_until(lambda: len(fleet.completed()) == 6, 90.0), fleet.completed()
    task, dead = victim['task'], victim['robot']
    later = [m for m in fleet.announces if m.task_id == task and m.seq > victim['seq']]
    assert later and later[0].announcer_id != dead           # a peer re-announced it
    assert not any(m.task_id == task and m.seq == victim['seq'] for m in fleet.awards)
    winner = [m for m in fleet.completes if m.task_id == task][0].robot_id
    assert winner != dead
    rows = fleet.log()
    assert any(e['task_id'] == task and e['detail'].startswith('announcer_timeout')
               for e in _events(rows, 'reauction'))


def test_dead_winner_task_returns_at_lease_expiry(fleet_factory):
    fleet = fleet_factory(n_tasks=4, task_rate=2.0)
    victim = {}

    def on_award(msg):
        if not victim and msg.winner_id in fleet.auction:
            victim.update(robot=msg.winner_id, task=msg.task_id)
            fleet.kill(msg.winner_id)                # the holder dies: no renewal, no message
    fleet.world.create_subscription(Award, '/fleet/award', on_award, 100)
    assert fleet.spin_until(lambda: len(fleet.completed()) == 4, 90.0), fleet.completed()
    task = victim['task']
    done_by = [m.robot_id for m in fleet.completes if m.task_id == task]
    assert done_by and done_by[0] != victim['robot']
    re = [e for e in _events(fleet.log(), 'reauction') if e['task_id'] == task]
    assert re and re[0]['detail'].startswith('lease_expired')


def test_reauction_service_reannounces_a_dead_robots_tasks_now(fleet_factory):
    fleet = fleet_factory(n_tasks=3, task_rate=2.0)
    assert fleet.spin_until(lambda: any(n.pool.holder_of(t) for n in fleet.auction.values()
                                        for t in n.pool.tasks), 30.0)
    holder, task = next((n.pool.holder_of(t), t) for n in fleet.auction.values()
                        for t in n.pool.tasks if n.pool.holder_of(t))
    caller = next(ns for ns in ROBOTS if ns != holder)
    client = fleet.world.create_client(ReAuction, f'/{caller}/reauction')
    assert client.wait_for_service(timeout_sec=5.0)
    req = ReAuction.Request()
    req.dead_robot_id = holder
    fut = client.call_async(req)
    assert fleet.spin_until(fut.done, 5.0)
    assert fut.result().tasks_reannounced >= 1
    re = [e for e in _events(fleet.log(), 'reauction') if e['task_id'] == task]
    assert re and re[0]['robot_id'] == caller and re[0]['detail'].startswith('reauction:')
    assert fleet.spin_until(lambda: len(fleet.completed()) == 3, 60.0)
    # at-most-once: the re-auctioned task completed exactly once
    assert sum(1 for m in fleet.completes if m.task_id == task) == 1
