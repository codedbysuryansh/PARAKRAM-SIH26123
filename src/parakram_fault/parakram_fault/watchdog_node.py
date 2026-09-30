"""
Watchdog node (CLAUDE_CODE/06): one per robot, classifies its peers from their lease renewals.

In : ``/<peer>/intent`` (the lease renewal is the heartbeat), through this robot's per-link loss
     process when ``loss_scope:=fleet`` (the same fades its coordination node sees).
Out: ``/<ns>/peer_status`` (PeerStatus, ~2 Hz per peer) and ``/fleet/recovery_event`` on every
     status change. DEAD is what the recovery coordinator acts on (task reallocation only).
Not on the safety / liveness path: spatial recovery is the lease expiring in coordination.
"""

import math

from parakram_comms.link_loss import attach_fault_injection, LinkLoss
from parakram_comms.qos import INTENT_QOS, STATUS_QOS
from parakram_fault.watchdog import ALIVE, DEAD, FLAKY, NAMES, WindowedWatchdog
from parakram_msgs.msg import Intent, PeerStatus, RecoveryEvent
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def _stamp(msg_time, t):
    msg_time.sec = int(math.floor(t))
    msg_time.nanosec = int((t - math.floor(t)) * 1e9)
    return msg_time


class WatchdogNode(Node):
    """Per-robot windowed watchdog."""

    def __init__(self):
        """Declare parameters and subscribe to the peers' renewals."""
        super().__init__('watchdog')
        dp = self.declare_parameter
        ns = self.get_namespace().strip('/')
        self.me = dp('robot_id', ns or 'robot1').value
        peers = [p for p in dp('peers', ['']).value if p and p != self.me]
        mode = dp('recovery_mode', 'lease').value
        self.dog = WindowedWatchdog(
            rate_hz=float(dp('hb_hz', 10.0).value), window_s=float(dp('window', 2.0).value),
            detect_timeout=float(dp('detect_timeout', 1.0).value),
            partition_grace=float(dp('partition_grace', 10.0).value),
            flaky_below=float(dp('flaky_below', 0.5).value),
            policy='grace' if mode == 'lease' else 'detect')
        self.link_loss = None
        log_dir = dp('log_dir', '').value
        if dp('loss_scope', 'none').value == 'fleet':
            self.link_loss = LinkLoss(float(dp('loss', 0.0).value), int(dp('seed', 0).value),
                                      self.me, model=dp('loss_model', 'bernoulli').value,
                                      burst_corr=float(dp('loss_burst_corr', 0.8).value),
                                      log_path=f'{log_dir}/comms_{self.me}_fault.csv'
                                      if log_dir else None)
            attach_fault_injection(self, self.link_loss)
        self.loss_level = self.link_loss.loss if self.link_loss else 0.0
        self.state = {p: None for p in peers}
        self.pub = self.create_publisher(PeerStatus, 'peer_status', STATUS_QOS)
        self.pub_event = self.create_publisher(RecoveryEvent, '/fleet/recovery_event',
                                               QoSProfile(depth=100,
                                                          reliability=ReliabilityPolicy.RELIABLE))
        for p in peers:
            cb = (lambda m, p=p: self._on_intent(p, m))
            if self.link_loss is not None:
                cb = self.link_loss.wrap(p, 'intent', cb, self._now)
            self.create_subscription(Intent, f'/{p}/intent', cb, INTENT_QOS)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f'watchdog {self.me}: peers {peers}, policy {self.dog.policy}, '
                               f'detect {self.dog.detect}s, partition grace {self.dog.grace}s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_intent(self, peer, msg):
        now = self._now()
        if now <= 0.0:
            return                                       # no (sim) clock yet: no valid time
        self.dog.on_renewal(peer, now, msg.seq)
        for who, seq in zip(msg.ack_ids, msg.ack_seqs):
            if who not in (self.me, peer) and who in self.state:
                self.dog.on_third_party_ack(who, seq, now)

    def _tick(self):
        now = self._now()
        if now <= 0.0:
            return
        for peer in self.state:
            status, age, delivery, others = self.dog.status(peer, now)
            if self.dog.watch(peer).last is None:
                continue                                 # never heard: not a member yet
            msg = PeerStatus()
            msg.robot_id, msg.peer_id, msg.status = self.me, peer, status
            _stamp(msg.stamp, now)
            msg.last_heard_age = float(min(age, 1e6))
            msg.window_delivery = float(delivery)
            msg.heard_by_others_age = float(others) if math.isfinite(others) else -1.0
            self.pub.publish(msg)
            if status != self.state[peer]:
                prev, self.state[peer] = self.state[peer], status
                if status in (ALIVE, FLAKY):
                    if prev is None or prev in (ALIVE, FLAKY):
                        continue                         # alive <-> flaky: no recovery event
                    kind = 'peer_back'
                else:
                    kind = NAMES[status]
                ev = RecoveryEvent()
                _stamp(ev.stamp, now)
                ev.kind, ev.robot_id, ev.dead_id = kind, self.me, peer
                ev.loss_level, ev.t_event = float(self.loss_level), float(now)
                ev.t_kill = ev.t_lease_expire = math.nan
                ev.t_space_reclaimed = ev.t_task_reassigned = math.nan
                ev.detail = f'silent {age:.2f}s, delivery {delivery:.2f}' + (
                    ' -> tasks may be reallocated' if status == DEAD else '')
                self.pub_event.publish(ev)

    def close(self):
        """Log the loss counters."""
        if self.link_loss is not None:
            self.link_loss.log(self._now())


def main(args=None):
    """Run one robot's watchdog."""
    rclpy.init(args=args)
    node = WatchdogNode()
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
