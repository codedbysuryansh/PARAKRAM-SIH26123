"""
Fleet-wide, per-link packet loss (CLAUDE_CODE/06; extends the CLAUDE_CODE/05 injector).

``parakram_comms.loss.LossFilter`` (05) drops each (receiver, sender, topic) stream on its own
and was applied to state / intent only. A fair recovery benchmark needs more: every inter-robot
message a protocol depends on (lease renewals, heartbeats, task announcements, bids, awards,
digests...) must cross the SAME lossy link, and a fade must hit all of them at once. Here:

* A directed link (sender -> receiver) is a loss process over sim-time slots of ``slot``
  seconds (50 ms), seeded from the run seed and the two robot ids only. Every topic and every
  process on the receiving robot evaluates the same deterministic process, so a renewal and the
  heartbeat relayed from it share their fate, and the coordination, task and watchdog processes
  of one robot see identical fades. Messages in different slots are independent (Bernoulli) or
  correlated (Gilbert-Elliott, ``burst_corr`` = lag-1 correlation of the drop indicator at 10 Hz,
  as in 05: per slot rho ** (slot / 0.1)).
* Partitions: ``add_partition(robot, t0, t1)`` drops every message to or from ``robot`` (``'*'``:
  every robot, a fleet-wide blackout) with ``t0 <= t < t1``. The bench sends these on
  ``/fleet/fault_injection`` (the equivalent of a ``tc`` command: test-harness control plane,
  never read by the robots' own logic), so ONE robot's links are cut and nothing else.
* The drop happens after delivery, before the callback: RELIABLE retransmission cannot hide it.
  Messages a robot receives from itself on fleet topics are never dropped (not a link).

Counters are appended to ``comms_<ns>_<component>.csv`` with the 05 columns.
"""

import csv
import json
import math
import os
import random

from parakram_comms.loss import LOG_COLUMNS, MODELS, stream_seed

SLOT_S = 0.05
REF_INTERVAL_S = 0.1          # burst_corr is the lag-1 correlation at this message interval


class LinkChannel:
    """Drop process of one directed link over sim-time slots (deterministic in the seed)."""

    def __init__(self, loss, model, burst_corr, seed, slot=SLOT_S):
        """Create the process; ``seed``: the link's own 64-bit seed."""
        self.loss, self.model, self.slot = float(loss), model, float(slot)
        self.rng = random.Random(seed)
        self.bad = []                                # slot k -> dropped?
        if model == 'gilbert_elliott' and self.loss > 0.0:
            lam = burst_corr ** (self.slot / REF_INTERVAL_S)
            self.p_gb = self.loss * (1.0 - lam)
            self.p_bg = (1.0 - self.loss) * (1.0 - lam)

    def _extend(self, k):
        while len(self.bad) <= k:
            u = self.rng.random()
            if self.loss <= 0.0:
                self.bad.append(False)
            elif self.model == 'bernoulli':
                self.bad.append(u < self.loss)
            elif not self.bad:
                self.bad.append(u < self.loss)       # stationary start
            elif self.bad[-1]:
                self.bad.append(u >= self.p_bg)
            else:
                self.bad.append(u < self.p_gb)

    def dropped(self, t):
        """Return True if a message on this link at sim time ``t`` is lost."""
        if self.loss <= 0.0 or t < 0.0:
            return False
        k = int(math.floor(t / self.slot))
        self._extend(k)
        return self.bad[k]


class LinkLoss:
    """A receiver's view of every link into it, with counters and partitions."""

    def __init__(self, loss, seed, receiver, model='bernoulli', burst_corr=0.8, log_path=None,
                 slot=SLOT_S):
        """Validate the setting; ``log_path``: CSV the counters are appended to."""
        loss = float(loss)
        if not 0.0 <= loss < 1.0:
            raise ValueError(f'loss must be in [0, 1), got {loss}')
        if model not in MODELS:
            raise ValueError(f'loss model must be one of {MODELS}, got {model!r}')
        if not 0.0 <= burst_corr < 1.0:
            raise ValueError(f'burst_corr must be in [0, 1), got {burst_corr}')
        self.loss, self.seed, self.receiver = loss, int(seed), receiver
        self.model, self.burst_corr, self.slot = model, float(burst_corr), float(slot)
        self.links = {}
        self.partitions = []                          # (robot, t0, t1)
        self.counts = {}
        self.log_path = log_path
        if log_path and not os.path.exists(log_path):
            with open(log_path, 'w', newline='') as f:
                csv.writer(f).writerow(LOG_COLUMNS)

    def link(self, sender):
        """Return the channel sender -> this receiver."""
        ch = self.links.get(sender)
        if ch is None:
            ch = self.links[sender] = LinkChannel(
                self.loss, self.model, self.burst_corr,
                stream_seed(self.seed, 'link', self.receiver, sender), self.slot)
        return ch

    def add_partition(self, robot, t0, t1):
        """Cut every link to or from ``robot`` ('*' = all robots) for ``t0 <= t < t1``."""
        self.partitions.append((str(robot), float(t0), float(t1)))

    def partitioned(self, sender, t):
        """Return True if the link sender -> receiver is cut at ``t``."""
        for robot, t0, t1 in self.partitions:
            if t0 <= t < t1 and robot in ('*', sender, self.receiver):
                return True
        return False

    def dropped(self, sender, t):
        """Return True if a message from ``sender`` received at sim time ``t`` is lost."""
        if sender == self.receiver:
            return False
        return self.partitioned(sender, t) or self.link(sender).dropped(t)

    def accept(self, sender, topic, t, seq=None):
        """Count one received message; return False if it must be dropped (not processed)."""
        key = (sender, topic)
        count = self.counts.get(key)
        if count is None:
            count = self.counts[key] = [0, 0, None, None, 0, False]
        count[0] += 1
        if seq is not None:
            if count[2] is None:
                count[2] = seq
            count[3] = seq
        if self.dropped(sender, t):
            count[1] += 1
            if not count[5]:
                count[4] += 1
            count[5] = True
            return False
        count[5] = False
        return True

    def wrap(self, sender, topic, callback, now):
        """Subscription callback that drops before ``callback``; ``now()``: sim time [s]."""
        def filtered(msg):
            if self.accept(sender, topic, now(), getattr(msg, 'seq', None)):
                callback(msg)
        return filtered

    def wrap_by_field(self, field, topic, callback, now):
        """As ``wrap`` for fleet topics: the sender is ``getattr(msg, field)``."""
        def filtered(msg):
            sender = str(getattr(msg, field, ''))
            if self.accept(sender, topic, now(), getattr(msg, 'seq', None)):
                callback(msg)
        return filtered

    def rows(self, t):
        """Return the current cumulative counters, one row per (sender, topic)."""
        return [[f'{t:.3f}', self.receiver, sender, topic, rec, drop, rec - drop,
                 '' if first is None else first, '' if last is None else last, bursts,
                 self.loss, f'{self.model}@link', self.burst_corr, self.seed]
                for (sender, topic), (rec, drop, first, last, bursts, _)
                in sorted(self.counts.items())]

    def log(self, t):
        """Append the current counters to the CSV (no-op without a log path)."""
        if self.log_path and self.counts:
            with open(self.log_path, 'a', newline='') as f:
                csv.writer(f).writerows(self.rows(t))


def parse_fault_command(text):
    """``/fleet/fault_injection`` payload -> list of (robot, t0, t1) partitions."""
    cmd = json.loads(text)
    items = cmd if isinstance(cmd, list) else [cmd]
    return [(str(c['partition']), float(c['t0']), float(c['t1'])) for c in items
            if 'partition' in c]


FAULT_TOPIC = '/fleet/fault_injection'


def attach_fault_injection(node, link_loss):
    """Subscribe ``link_loss`` to the bench's partition commands (RELIABLE, TRANSIENT_LOCAL)."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    def on_command(msg):
        for robot, t0, t1 in parse_fault_command(msg.data):
            link_loss.add_partition(robot, t0, t1)
            node.get_logger().warn(f'fault injection: {robot} partitioned {t0:.1f}-{t1:.1f} s')
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    return node.create_subscription(String, FAULT_TOPIC, on_command, qos)
