"""
This robot's side of the spatial-lease protocol (CLAUDE_CODE/06), pure (no ROS).

Every intent a robot publishes renews its lease: ``lease_expiry = send time + L`` for all the
cells it reserves. A peer honours those cells until the lease of the LAST renewal it received
runs out, then frees them without any message (communication-negative recovery). The holder can
only know which renewals arrived through acknowledgements: every intent carries, for each peer,
the latest seq it processed from that peer (``Intent.ack_ids`` / ``ack_seqs``). Hence:

* my lease is certainly still valid at peer p until ``t_send(latest renewal p acked) + L``;
* my OWN lease is valid while that holds, with a stop margin, for EVERY live peer (a peer whose
  lease is valid here) and I hear a strict majority of the robots I know (quorum). Otherwise it
  has (or may have) expired somewhere: I must stop and RE-ACQUIRE before moving again (claims
  are dropped and must be re-made and re-acknowledged), so a false positive costs throughput,
  never safety. An isolated robot therefore freezes, while a majority keeps going;
* a claim becomes movement authority only once every live peer has acknowledged an intent that
  contained it (and after ``claim_settle``), so every cell I may drive into is known to every
  peer that could contest it;
* the task award my intents renew (CLAUDE_CODE/04) is safe to execute while a strict MAJORITY
  of the fleet, me included, has acknowledged a renewal of it within ``award_lease_ttl`` (minus
  a margin): a robot that could re-auction it must hear a majority (parakram_tasks), which
  shares a robot with mine, so it has heard of the award recently (directly, or through that
  robot's acknowledgements). A peer returning from a partition with stale acknowledgements does
  not revoke an award the majority kept renewing; an isolated holder releases its award.
"""

from collections import OrderedDict
import math

NEVER = -math.inf


class OwnLease:
    """Send log of my renewals and the peers' acknowledgements of them."""

    def __init__(self, lease_ttl, stop_margin=0.3, history=600):
        """``history``: renewals remembered (at 10 Hz, 60 s)."""
        self.L, self.margin, self.history = float(lease_ttl), float(stop_margin), int(history)
        self.sent = OrderedDict()           # seq -> (t_send, task or None)
        self.acks = {}                      # peer -> highest seq of mine it acknowledged

    def sent_renewal(self, seq, t_send, task=None):
        """Record renewal ``seq`` sent at ``t_send`` carrying ``task`` = (task_id, round)."""
        self.sent[int(seq)] = (float(t_send), task)
        while len(self.sent) > self.history:
            self.sent.popitem(last=False)

    def on_ack(self, peer, acked_seq):
        """Peer ``peer`` has processed my renewals up to ``acked_seq``."""
        if int(acked_seq) > self.acks.get(peer, -1):
            self.acks[peer] = int(acked_seq)

    def forget(self, peer):
        """Drop a peer's acknowledgements (it restarted or left)."""
        self.acks.pop(peer, None)

    def acked_time(self, peer):
        """Send time of the latest renewal ``peer`` acknowledged (NEVER if none known)."""
        seq = self.acks.get(peer)
        if seq is None:
            return NEVER
        entry = self.sent.get(seq)
        if entry is not None:
            return entry[0]
        older = [s for s in self.sent if s <= seq]      # acked seq fell out of the log
        return self.sent[older[-1]][0] if older else NEVER

    def horizon(self, live):
        """Time until which my lease is known valid at every live peer (inf: no live peer)."""
        return min((self.acked_time(p) + self.L for p in live), default=math.inf)

    def status(self, now, live, known):
        """Return ``(valid, reason)`` for my own lease at ``now``."""
        n = 1 + len(known)
        if 2 * (1 + len(live)) <= n:
            return False, f'no quorum ({1 + len(live)}/{n} robots heard)'
        late = sorted(p for p in live if self.acked_time(p) + self.L - self.margin <= now)
        if late:
            return False, 'renewals not acknowledged by ' + ','.join(late)
        return True, ''

    def acked_by_all(self, seq, live):
        """Return True if every live peer has acknowledged renewal ``seq`` (or a later one)."""
        return all(self.acks.get(p, -1) >= seq for p in live)

    def task_acked_by(self, peer, task):
        """Send time of the latest renewal carrying ``task`` ``peer`` acknowledged (or None)."""
        seq = self.acks.get(peer, -1)
        for s, (t, carried) in reversed(self.sent.items()):
            if s <= seq and carried == task:
                return t
        return None

    def task_quorum_time(self, known, task):
        """
        Return since when a strict majority has acknowledged renewals carrying ``task``.

        The latest ``t`` such that a strict majority of the fleet (me + ``known`` peers) has
        acknowledged a renewal carrying ``task`` sent at ``t`` or later: inf when I know no
        peer (alone), None when no majority has acknowledged one yet.
        """
        need = (len(known) + 1) // 2                  # peers needed besides me
        if need == 0:
            return math.inf
        times = sorted((t for t in (self.task_acked_by(p, task) for p in known)
                        if t is not None), reverse=True)
        return times[need - 1] if len(times) >= need else None
