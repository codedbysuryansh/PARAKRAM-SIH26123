"""
Windowed watchdog over peers' lease renewals (CLAUDE_CODE/06), pure (no ROS).

The watchdog is NOT on the safety / liveness path: spatial recovery is the lease expiring in
coordination, with no message. The watchdog only decides when a silent peer's TASKS may be
reallocated (the opportunistic accelerator), and must not do that to a robot that is merely
partitioned from this one. Its input is the peers' lease renewals (their intents: lease renewal
IS the heartbeat) as they arrive here, through the same lossy links as everything else.

Per peer: the renewals received in a sliding window give a delivery estimate; the age of the
last one gives silence. A peer is

* ALIVE   - heard within ``detect_timeout`` and delivery >= ``flaky_below``;
* FLAKY   - heard within ``detect_timeout`` but through a lossy link;
* SUSPECT - silent here for ``detect_timeout`` or more;
* PARTITIONED - silent here, but another peer acknowledged a NEW renewal of it within
  ``detect_timeout`` (every intent carries the renewals its sender processed): it is alive;
* DEAD    - policy ``grace`` (PARAKRAM): silent here and not heard by anyone else for
  ``partition_grace``; policy ``detect`` (the re-auction baseline): as soon as it is SUSPECT.
"""

from collections import deque
import math

ALIVE, FLAKY, SUSPECT, PARTITIONED, DEAD = 0, 1, 2, 3, 4
NAMES = {ALIVE: 'alive', FLAKY: 'flaky', SUSPECT: 'suspect', PARTITIONED: 'partitioned',
         DEAD: 'dead'}


class PeerWatch:
    """What this robot knows about one peer's renewals."""

    def __init__(self):
        """Nothing heard yet."""
        self.times = deque()
        self.last = None
        self.last_seq = -1
        self.others_seq = -1
        self.others_heard = None


class WindowedWatchdog:
    """Classifies every peer from its renewals and from third-party acknowledgements."""

    def __init__(self, rate_hz=10.0, window_s=2.0, detect_timeout=1.0, partition_grace=10.0,
                 flaky_below=0.5, policy='grace'):
        """Parameters in seconds; ``policy``: 'grace' (PARAKRAM) or 'detect' (baseline)."""
        if policy not in ('grace', 'detect'):
            raise ValueError(f'policy must be grace or detect, got {policy!r}')
        self.rate, self.window = float(rate_hz), float(window_s)
        self.detect, self.grace = float(detect_timeout), float(partition_grace)
        self.flaky_below, self.policy = float(flaky_below), policy
        self.peers = {}

    def watch(self, peer):
        """Return (creating) the record of ``peer``."""
        w = self.peers.get(peer)
        if w is None:
            w = self.peers[peer] = PeerWatch()
        return w

    def on_renewal(self, peer, t, seq):
        """Record a renewal of ``peer`` (seq ``seq``) received here at ``t``."""
        w = self.watch(peer)
        w.times.append(t)
        w.last = t if w.last is None else max(w.last, t)
        w.last_seq = max(w.last_seq, int(seq))
        self._trim(w, t)

    def on_third_party_ack(self, peer, acked_seq, t):
        """Record that another robot processed ``peer``'s renewal ``acked_seq`` (seen at ``t``)."""
        w = self.watch(peer)
        if int(acked_seq) > max(w.others_seq, w.last_seq):
            w.others_heard = t                       # a renewal this robot never received
        w.others_seq = max(w.others_seq, int(acked_seq))

    def _trim(self, w, now):
        while w.times and w.times[0] < now - self.window:
            w.times.popleft()

    def status(self, peer, now):
        """Return ``(status, last_heard_age, window_delivery, heard_by_others_age)``."""
        w = self.watch(peer)
        self._trim(w, now)
        age = math.inf if w.last is None else now - w.last
        delivery = min(1.0, len(w.times) / (self.rate * self.window))
        others = math.inf if w.others_heard is None else now - w.others_heard
        if age < self.detect:
            return (FLAKY if delivery < self.flaky_below else ALIVE), age, delivery, others
        if self.policy == 'detect':
            return DEAD, age, delivery, others
        if others < self.detect:
            return PARTITIONED, age, delivery, others
        if age >= self.grace and others >= self.grace:
            return DEAD, age, delivery, others
        return SUSPECT, age, delivery, others
