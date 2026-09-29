"""
Contract-Net auction node (CLAUDE_CODE/04): one per robot, in its namespace, identical everywhere.

There is no auctioneer node. Every robot keeps a replica of the task pool (``task_pool``) and
runs the same state machine:

1. **Pool:** tasks enter from ``/fleet/tasks`` (sim ``task_generator``, or any robot) and are
   reconciled from the RELIABLE + TRANSIENT_LOCAL fleet topics.
2. **Announce:** a robot that sees an unowned task waits a random (seeded) delay and, if nobody
   announced it meanwhile, announces the next round (``seq``) itself: the announcer role is
   transient and rotates. Rounds nobody bid on are retried with exponential backoff.
3. **Bid:** an idle robot bids its estimated time-to-serve (``bidder.cost``) on the open round it
   is cheapest for (one outstanding bid, so it can never win two tasks at once).
4. **Award:** after ``bid_window`` the round's announcer awards the lowest bid (ties: robot id)
   with ``lease_expiry = now + award_lease_ttl``.
5. **Lease:** the winner drives the task through its coordination layer (``/<ns>/assigned_task``
   goal: pickup, then dropoff) and renews the lease every ``renew_period`` while it makes
   progress. A dead or stuck holder stops renewing; at expiry every replica returns the task
   to the pool and any robot re-announces it. If an announcer dies mid-auction, the bidders see
   no award within ``bid_window + award_timeout`` and a peer re-announces (implicit re-election).
6. **Complete:** at the dropoff the winner publishes ``TaskComplete``.

``ReAuction`` (``/<ns>/reauction``, for the fault layer, CLAUDE_CODE/06): every task the dead
robot holds is re-announced at once. Peer heartbeats (``/<peer>/heartbeat``, published by
CLAUDE_CODE/06) are read when present: an announcer does not award to a bidder whose heartbeat
went stale. The lease stays the mechanism that frees a dead holder's task.

The auction is an opportunistic throughput accelerator: coordination (PIBT + spatial leases)
and the reactive safety layer never depend on it. Events: ``bench/logs/<run_id>/tasks.csv``
(shared, append-only), status: ``/fleet/task_status``.
"""

import math
import os
import random
import zlib

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose2D
from parakram_comms.qos import (HEARTBEAT_QOS, STATE_QOS, STATUS_QOS, TASK_EVENT_QOS,
                                TASK_POOL_QOS)
from parakram_msgs.msg import (Award, Bid, CoordStatus, Heartbeat, RobotState, Task,
                               TaskAnnounce, TaskComplete, TaskProgress, TaskStatus)
from parakram_msgs.srv import ReAuction
from parakram_sim.grid_utils import default_grid_path, WarehouseGrid
from parakram_tasks import bidder
from parakram_tasks import task_pool as tp
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

LOG_COLUMNS = ['t', 'event', 'task_id', 'robot_id', 'cost', 'lease_expiry', 'seq', 'detail']
TO_PICKUP, TO_DROPOFF, RELEASED = (TaskProgress.STAGE_TO_PICKUP, TaskProgress.STAGE_TO_DROPOFF,
                                   TaskProgress.STAGE_RELEASED)


def _sec(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def _stamp(t):
    msg = TimeMsg()
    msg.sec = int(math.floor(t))
    msg.nanosec = int((t - math.floor(t)) * 1e9)
    return msg


def _fmt(value):
    if value is None or value == '':
        return ''
    if isinstance(value, float):
        return 'inf' if math.isinf(value) else f'{value:.3f}'
    return str(value)


class TaskLog:
    """``tasks.csv``: one file per run, appended by every robot's node (single-write lines)."""

    def __init__(self, path):
        """Create the file with its header exactly once (atomic link), then append."""
        self.path = path
        if not os.path.exists(path):
            tmp = f'{path}.{os.getpid()}.tmp'
            with open(tmp, 'w') as f:
                f.write(','.join(LOG_COLUMNS) + '\n')
            try:
                os.link(tmp, path)                  # fails if another robot created it first
            except FileExistsError:
                pass
            os.unlink(tmp)
        self.fd = os.open(path, os.O_WRONLY | os.O_APPEND)

    def write(self, t, event, task_id, robot_id, cost=None, lease_expiry=None, seq='',
              detail=''):
        """Append one event (a single ``write``: lines of concurrent robots never interleave)."""
        line = ','.join([f'{t:.3f}', event, task_id, robot_id, _fmt(cost), _fmt(lease_expiry),
                         str(seq), detail]) + '\n'
        os.write(self.fd, line.encode())

    def close(self):
        """Close the file."""
        os.close(self.fd)


class Job:
    """The task this robot is executing."""

    def __init__(self, task_id, seq, cost, lease_expiry, now):
        """Start at the pickup stage."""
        self.task_id, self.seq, self.cost, self.lease_expiry = task_id, seq, cost, lease_expiry
        self.stage = TO_PICKUP
        self.goal_sent_at = None
        self.arrived_at = None
        self.best_dist = math.inf
        self.progress_t = now
        self.last_renew = now


class AuctionNode(Node):
    """Per-robot Contract-Net participant (announcer, bidder and winner roles)."""

    def __init__(self, **kwargs):
        """Declare parameters and wire the fleet topics (``kwargs`` go to rclpy's Node)."""
        super().__init__('auction_node', **kwargs)
        dp = self.declare_parameter
        ns = self.get_namespace().strip('/')
        self.me = dp('robot_id', ns or 'robot').value
        self.bid_window = float(dp('bid_window', 1.0).value)
        self.award_timeout = float(dp('award_timeout', 2.0).value)
        self.lease_ttl = float(dp('award_lease_ttl', 10.0).value)
        self.renew_period = float(dp('renew_period', 2.0).value)
        self.jitter = (float(dp('announce_jitter_min', 0.2).value),
                       float(dp('announce_jitter_max', 1.0).value))
        self.backoff_max = float(dp('backoff_max', 8.0).value)
        self.service_time = float(dp('service_time', 1.0).value)
        self.stall_timeout = float(dp('stall_timeout', 60.0).value)
        self.goal_resend = float(dp('goal_resend', 1.5).value)
        self.hb_timeout = float(dp('heartbeat_timeout', 3.0).value)
        self.cparams = bidder.CostParams(
            speed=float(dp('plan_speed', 0.15).value),
            service_time=2.0 * self.service_time,
            battery_min=float(dp('battery_min', 0.2).value))
        peers = [p for p in dp('peers', ['']).value if p and p != self.me]
        seed = int(dp('seed', 0).value)
        log_dir = dp('log_dir', '').value
        grid_yaml = dp('grid_yaml', '').value
        tick_hz = float(dp('tick_hz', 10.0).value)

        self.grid = WarehouseGrid.from_yaml(grid_yaml or default_grid_path())
        self.pool = tp.TaskPool(self.bid_window, self.award_timeout)
        # seeded, but different on every robot: the jittered announce delays decide who announces
        self.rng = random.Random(seed * 1000003 + zlib.crc32(self.me.encode()))
        self.announce_at = {}                 # task_id -> time this robot will announce it
        self.my_rounds = {}                   # task_id -> seq of rounds this robot announced
        self.my_bid = None                    # (task_id, seq) outstanding
        self.bid_done = set()                 # (task_id, seq) already bid on
        self.job = None
        self.cell, self.coord_goal, self.battery = None, -1, 1.0
        self.hb_seen = {}
        self.log = TaskLog(os.path.join(log_dir, 'tasks.csv')) if log_dir else None

        self.pub_announce = self.create_publisher(TaskAnnounce, '/fleet/task_announce',
                                                  TASK_POOL_QOS)
        self.pub_bid = self.create_publisher(Bid, '/fleet/bid', TASK_EVENT_QOS)
        self.pub_award = self.create_publisher(Award, '/fleet/award', TASK_POOL_QOS)
        self.pub_progress = self.create_publisher(TaskProgress, '/fleet/task_progress',
                                                  TASK_EVENT_QOS)
        self.pub_complete = self.create_publisher(TaskComplete, '/fleet/task_complete',
                                                  TASK_POOL_QOS)
        self.pub_status = self.create_publisher(TaskStatus, '/fleet/task_status', STATUS_QOS)
        self.pub_goal = self.create_publisher(Pose2D, 'assigned_task', STATUS_QOS)
        self.create_subscription(Task, '/fleet/tasks', self._on_task, TASK_POOL_QOS)
        self.create_subscription(TaskAnnounce, '/fleet/task_announce', self._on_announce,
                                 TASK_POOL_QOS)
        self.create_subscription(Bid, '/fleet/bid', self._on_bid, TASK_EVENT_QOS)
        self.create_subscription(Award, '/fleet/award', self._on_award, TASK_POOL_QOS)
        self.create_subscription(TaskProgress, '/fleet/task_progress', self._on_progress,
                                 TASK_EVENT_QOS)
        self.create_subscription(TaskComplete, '/fleet/task_complete', self._on_complete,
                                 TASK_POOL_QOS)
        self.create_subscription(CoordStatus, 'coord_status', self._on_coord, STATUS_QOS)
        self.create_subscription(RobotState, 'state', self._on_state, STATE_QOS)
        for peer in peers:
            self.create_subscription(Heartbeat, f'/{peer}/heartbeat',
                                     lambda m, p=peer: self._on_heartbeat(p), HEARTBEAT_QOS)
        self.create_service(ReAuction, 'reauction', self._on_reauction)
        self.create_timer(1.0 / tick_hz, self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f'auction {self.me}: bid window {self.bid_window}s, award lease '
            f'{self.lease_ttl}s (renew every {self.renew_period}s), peers {peers}; '
            'no central auctioneer')

    # ------------------------------------------------------------------ helpers
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _event(self, now, event, task_id, robot_id, cost=None, lease=None, seq='', detail=''):
        if self.log is not None:
            self.log.write(now, event, task_id, robot_id, cost, lease, seq, detail)

    def _cell_of(self, pose):
        return self.grid.world_to_cell(pose[0], pose[1])

    def _schedule(self, task_id, now):
        rec = self.pool.tasks[task_id]
        delay = self.rng.uniform(*self.jitter) * (2 ** min(rec.failed_rounds, 4))
        self.announce_at[task_id] = now + min(delay, self.backoff_max)

    def _known_stale(self, robot, now):
        seen = self.hb_seen.get(robot)
        return seen is not None and now - seen > self.hb_timeout

    # ------------------------------------------------------------------ fleet inputs
    def _on_task(self, msg):
        now = self._now()
        pickup = (msg.pickup.x, msg.pickup.y, msg.pickup.theta)
        dropoff = (msg.dropoff.x, msg.dropoff.y, msg.dropoff.theta)
        if self.pool.add(msg.task_id, pickup, dropoff, now):
            self._schedule(msg.task_id, now)

    def _on_announce(self, msg):
        now = self._now()
        if msg.task_id not in self.pool.tasks:     # discovered through its announcement
            self.pool.add(msg.task_id, (msg.pickup.x, msg.pickup.y, msg.pickup.theta),
                          (msg.dropoff.x, msg.dropoff.y, msg.dropoff.theta), now)
        if self.pool.on_announce(msg.task_id, msg.seq, msg.announcer_id, now):
            self.announce_at.pop(msg.task_id, None)
            if self.job is not None and self.job.task_id == msg.task_id:
                self._drop(now, 'task re-announced (round %d)' % msg.seq)
        self._try_bid(now)

    def _on_bid(self, msg):
        self.pool.on_bid(msg.task_id, msg.seq, msg.robot_id, msg.cost)

    def _on_award(self, msg):
        now = self._now()
        accepted, dropped = self.pool.on_award(msg.task_id, msg.seq, msg.winner_id,
                                               _sec(msg.lease_expiry), msg.announcer_id,
                                               msg.cost, now)
        if not accepted:
            return
        self.announce_at.pop(msg.task_id, None)
        if dropped == self.me and self.job is not None and self.job.task_id == msg.task_id:
            self._drop(now, f'superseded by the round-{msg.seq} award to {msg.winner_id}')
        if msg.winner_id == self.me:
            if self.job is None:
                self.job = Job(msg.task_id, msg.seq, msg.cost, _sec(msg.lease_expiry), now)
                self.get_logger().info(f'won {msg.task_id} (round {msg.seq}, cost '
                                       f'{msg.cost:.1f}s, announcer {msg.announcer_id})')
            elif self.job.task_id != msg.task_id:  # cannot happen with one outstanding bid
                self._renew(msg.task_id, msg.seq, now, RELEASED)

    def _on_progress(self, msg):
        self.pool.on_renew(msg.task_id, msg.seq, msg.robot_id, _sec(msg.lease_expiry),
                           released=msg.stage == RELEASED)

    def _on_complete(self, msg):
        self.pool.on_complete(msg.task_id, msg.robot_id)
        self.announce_at.pop(msg.task_id, None)
        if self.job is not None and self.job.task_id == msg.task_id and \
                msg.robot_id != self.me:
            self._drop(self._now(), f'completed by {msg.robot_id}')

    def _on_coord(self, msg):
        if msg.cell >= 0:
            self.cell = self.grid.index_to_cell(msg.cell)
        self.coord_goal = msg.goal_cell

    def _on_state(self, msg):
        self.battery = float(msg.battery)

    def _on_heartbeat(self, peer):
        self.hb_seen[peer] = self._now()

    def _on_reauction(self, request, response):
        """Fault layer (CLAUDE_CODE/06): re-announce every task a dead robot holds, now."""
        now = self._now()
        dead = request.dead_robot_id
        ids = self.pool.release_robot(dead)
        if self.job is not None and self.job.task_id in ids:
            self.job = None
        for task_id in ids:
            self._announce(task_id, now)
        response.tasks_reannounced = len(ids)
        self.get_logger().warn(f'ReAuction({dead}): re-announced {ids}')
        return response

    # ------------------------------------------------------------------ roles
    def _announce(self, task_id, now):
        rec = self.pool.tasks[task_id]
        reauction = self.pool.is_reauction(task_id)
        detail = ''
        if reauction:
            detail = rec.back_reason + (f' holder={rec.winner}' if rec.winner else '')
        seq = rec.seq + 1
        msg = TaskAnnounce()
        msg.task_id, msg.announcer_id, msg.seq = task_id, self.me, seq
        msg.pickup.x, msg.pickup.y, msg.pickup.theta = rec.pickup
        msg.dropoff.x, msg.dropoff.y, msg.dropoff.theta = rec.dropoff
        msg.announce_time = _stamp(now)
        self.pub_announce.publish(msg)
        self.pool.on_announce(task_id, seq, self.me, now)
        self.my_rounds[task_id] = seq
        self.announce_at.pop(task_id, None)
        self._event(now, 'reauction' if reauction else 'announce', task_id, self.me, seq=seq,
                    detail=detail)
        self._try_bid(now)

    def _close_rounds(self, now):
        """Award the rounds this robot announced once their bid window has closed."""
        for task_id, seq in list(self.my_rounds.items()):
            rec = self.pool.tasks.get(task_id)
            if rec is None or rec.state != tp.AUCTION or rec.seq != seq or \
                    rec.announcer != self.me:
                self.my_rounds.pop(task_id)      # superseded, awarded or owned by a lower id
                continue
            if now < rec.announced_at + self.bid_window:
                continue
            self.my_rounds.pop(task_id)
            live = {r: c for r, c in rec.bids.items() if not self._known_stale(r, now)}
            if not live:
                continue                         # no bidder: the round lapses, backoff retry
            winner, cost = min(live.items(), key=lambda kv: (kv[1], kv[0]))
            if not math.isfinite(cost):
                continue
            lease = now + self.lease_ttl
            msg = Award()
            msg.task_id, msg.winner_id, msg.seq = task_id, winner, seq
            msg.lease_expiry, msg.announcer_id, msg.cost = _stamp(lease), self.me, float(cost)
            self.pub_award.publish(msg)
            self._event(now, 'award', task_id, winner, cost, lease, seq,
                        f'announcer={self.me} bids={len(live)}')
            self._on_award(msg)

    def _try_bid(self, now):
        """Bid on the cheapest open round (one outstanding bid, never while busy)."""
        if self.my_bid is not None:
            rec = self.pool.tasks.get(self.my_bid[0])
            if rec is not None and rec.state == tp.AUCTION and rec.seq == self.my_bid[1]:
                return                           # still waiting for that round's award
            self.my_bid = None
        if self.job is not None or self.cell is None:
            return
        me = bidder.RobotSnapshot(self.me, self.cell, self.battery)
        best = None
        for rec in self.pool.tasks.values():
            if rec.state != tp.AUCTION or (rec.task_id, rec.seq) in self.bid_done or \
                    now > rec.announced_at + 0.8 * self.bid_window:
                continue
            task = bidder.TaskSpec(rec.task_id, self._cell_of(rec.pickup),
                                   self._cell_of(rec.dropoff))
            c = bidder.cost(me, task, self.grid, self.cparams)
            if math.isfinite(c) and (best is None or (c, rec.task_id) < (best[0], best[1])):
                best = (c, rec.task_id, rec.seq)
        if best is None:
            return
        c, task_id, seq = best
        msg = Bid()
        msg.task_id, msg.robot_id, msg.cost, msg.seq = task_id, self.me, float(c), seq
        self.pub_bid.publish(msg)
        self.pool.on_bid(task_id, seq, self.me, c)
        self.my_bid = (task_id, seq)
        self.bid_done.add((task_id, seq))
        self._event(now, 'bid', task_id, self.me, c, seq=seq)

    # ------------------------------------------------------------------ winner role
    def _renew(self, task_id, seq, now, stage):
        lease = now + self.lease_ttl
        msg = TaskProgress()
        msg.task_id, msg.robot_id, msg.seq, msg.stage = task_id, self.me, seq, stage
        msg.lease_expiry = _stamp(lease)
        self.pub_progress.publish(msg)
        self.pool.on_renew(task_id, seq, self.me, lease, released=stage == RELEASED)
        return lease

    def _send_goal(self, cell, now):
        x, y = self.grid.cell_to_world(*cell)
        self.pub_goal.publish(Pose2D(x=float(x), y=float(y), theta=0.0))
        self.job.goal_sent_at = now

    def _drop(self, now, why):
        self.get_logger().warn(f'dropping {self.job.task_id}: {why}')
        self.job = None
        if self.cell is not None:                # stop pursuing it: hold the current cell
            x, y = self.grid.cell_to_world(*self.cell)
            self.pub_goal.publish(Pose2D(x=float(x), y=float(y), theta=0.0))

    def _execute(self, now):
        job = self.job
        rec = self.pool.tasks.get(job.task_id)
        if rec is None or rec.state != tp.ASSIGNED or rec.winner != self.me or \
                rec.award_seq != job.seq:
            self._drop(now, "no longer this robot's award")
            return
        pose = rec.pickup if job.stage == TO_PICKUP else rec.dropoff
        target = self._cell_of(pose)
        index = self.grid.cell_to_index(*target)
        if self.coord_goal != index and (job.goal_sent_at is None or
                                         now - job.goal_sent_at >= self.goal_resend):
            self._send_goal(target, now)
        if self.cell is not None:
            dist = bidder.grid_distance(self.grid, self.cell, target)
            if dist is not None and dist < job.best_dist:
                job.best_dist, job.progress_t = dist, now
        if self.cell == target:
            job.progress_t = now
            if job.arrived_at is None:
                job.arrived_at = now
            elif now - job.arrived_at >= self.service_time:
                if job.stage == TO_PICKUP:
                    job.stage, job.arrived_at, job.best_dist = TO_DROPOFF, None, math.inf
                    self._send_goal(self._cell_of(rec.dropoff), now)
                    job.lease_expiry = self._renew(job.task_id, job.seq, now, TO_DROPOFF)
                    job.last_renew = now
                    self._event(now, 'renew', job.task_id, self.me, None, job.lease_expiry,
                                job.seq, 'picked_up')
                    return
                msg = TaskComplete()
                msg.task_id, msg.robot_id, msg.seq = job.task_id, self.me, job.seq
                msg.stamp = _stamp(now)
                self.pub_complete.publish(msg)
                self.pool.on_complete(job.task_id, self.me)
                self._event(now, 'complete', job.task_id, self.me, job.cost, job.lease_expiry,
                            job.seq)
                self.get_logger().info(f'completed {job.task_id}')
                self.job = None
                return
        else:
            job.arrived_at = None
        if now - job.progress_t > self.stall_timeout:
            # blocked: stop renewing; the lease expires and a peer re-announces the task
            self._drop(now, f'no progress for {self.stall_timeout:.0f}s, letting the lease lapse')
            return
        if now - job.last_renew >= self.renew_period:
            job.lease_expiry = self._renew(job.task_id, job.seq, now, job.stage)
            job.last_renew = now
            self._event(now, 'renew', job.task_id, self.me, None, job.lease_expiry, job.seq)

    # ------------------------------------------------------------------ tick
    def _tick(self):
        now = self._now()
        if now <= 0.0:
            return                                # no (sim) clock yet
        for task_id, _reason in self.pool.expire(now):
            self._schedule(task_id, now)
        self._close_rounds(now)
        for rec in self.pool.pending():
            if rec.task_id not in self.announce_at:
                self._schedule(rec.task_id, now)
            elif now >= self.announce_at[rec.task_id]:
                self._announce(rec.task_id, now)
        self._try_bid(now)
        if self.job is not None:
            self._execute(now)

    def _publish_status(self):
        now = self._now()
        counts = self.pool.counts()
        st = TaskStatus()
        st.robot_id, st.stamp = self.me, _stamp(max(now, 0.0))
        st.n_pending = counts[tp.PENDING] + counts[tp.AUCTION]
        st.n_auctioning = counts[tp.AUCTION]
        st.n_assigned = counts[tp.ASSIGNED]
        st.n_completed = counts[tp.DONE]
        assigned = sorted((r.task_id, r.winner) for r in self.pool.tasks.values()
                          if r.state == tp.ASSIGNED)
        st.assigned_task_ids = [t for t, _ in assigned]
        st.assigned_robot_ids = [w for _, w in assigned]
        st.current_task_id = self.job.task_id if self.job else ''
        st.current_stage = self.job.stage if self.job else 0
        self.pub_status.publish(st)

    def close(self):
        """Close the event log."""
        if self.log is not None:
            self.log.close()
            self.log = None


def main(args=None):
    """Run one robot's auction participant."""
    rclpy.init(args=args)
    node = AuctionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
