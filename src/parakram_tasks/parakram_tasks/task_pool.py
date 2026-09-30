"""
Replicated task pool with lease bookkeeping (CLAUDE_CODE/04), pure and unit-tested.

Every robot keeps its own ``TaskPool`` and feeds it the same fleet messages (tasks,
announcements, bids, awards, lease renewals, completions); nothing in here talks to ROS, and no
robot's pool is authoritative. Rules that make the replicas converge:

* A task's ``seq`` is its auction round (version). A newer round (higher ``seq``) supersedes
  everything older: an award is only valid for the newest round known, and a robot holding an
  older award must drop the task (at-most-once execution).
* Equal ``seq`` from two announcers (both saw the task unowned at once): the LOWER announcer id
  owns the round, and an award from the other one is ignored. Winner selection within a round is
  deterministic: lowest cost, then lowest robot id.
* An award is a lease: ``lease_expiry`` is extended only by the holder's renewals. At
  ``now >= lease_expiry`` the task returns to the pool (the dead or blocked holder needs to send
  nothing). An auction whose award does not arrive within ``bid_window + award_timeout``
  (announcer died mid-auction) also returns the task to the pool, where any robot re-announces it.
* A completion is final; later messages about the task are ignored.

CLAUDE_CODE/06: the holder's lease renewals also ride on its lease-renewal intents
(``on_holder_renewal``: a peer that missed the Award learns it from them), and every robot
repeats what it knows in a digest (``digest`` / ``apply_digest``: completions and newest award
rounds; only the holder's renewals extend a known award's lease), so a robot that missed
messages converges without executing a task twice. ``extend_holder``: another robot's
acknowledgement of a new renewal of the holder keeps its awards alive (partitioned is not
dead); ``shift_leases``: award time does not run while this robot itself is cut off (see
``auction_node``).
``lease_enabled=False`` is the ``reauction_baseline``: awards never expire; only a newer round
(``ReAuction``) takes a task away.
"""

from dataclasses import dataclass, field
import math

PENDING = 'pending'          # in the pool, no open auction and no valid award
AUCTION = 'auction'          # announced (round ``seq``), award not seen yet
ASSIGNED = 'assigned'        # awarded in round ``award_seq`` (the newest round), lease valid
DONE = 'done'


@dataclass
class TaskRecord:
    """One task as this robot sees it."""

    task_id: str
    pickup: tuple                    # world (x, y, theta)
    dropoff: tuple
    created: float
    state: str = PENDING
    seq: int = 0                     # newest round seen (announce or award)
    announcer: str = ''              # owner of round ``seq``
    announced_at: float = None       # local time round ``seq`` was first seen
    bids: dict = field(default_factory=dict)   # robot -> cost, for round ``seq``
    award_seq: int = 0               # round of the current award (0: never awarded)
    award_announcer: str = ''
    winner: str = ''
    lease_expiry: float = None
    award_cost: float = None
    completed_by: str = ''
    awards: int = 0                  # awards accepted over the task's life (log / checks)
    failed_rounds: int = 0           # consecutive rounds without an award (backoff)
    back_reason: str = ''            # why it last returned to the pool
    reauction_due: bool = False      # its next announcement is a re-auction (see is_reauction)


class TaskPool:
    """This robot's replica of the fleet's task pool."""

    def __init__(self, bid_window=1.0, award_timeout=2.0, lease_enabled=True):
        """Create an empty pool with the auction timing every robot uses."""
        self.lease_enabled = bool(lease_enabled)
        self.bid_window = float(bid_window)
        self.award_timeout = float(award_timeout)
        self.tasks = {}

    # ------------------------------------------------------------------ messages
    def add(self, task_id, pickup, dropoff, now):
        """Add a new task to the pool; return False if it was already known."""
        if task_id in self.tasks:
            return False
        self.tasks[task_id] = TaskRecord(task_id, tuple(pickup), tuple(dropoff), float(now))
        return True

    def on_announce(self, task_id, seq, announcer, now):
        """Round ``seq`` announced by ``announcer``; returns True if it became the open round."""
        rec = self.tasks.get(task_id)
        if rec is None or rec.state == DONE or seq < rec.seq:
            return False
        if seq == rec.seq:                          # a known round never reopens
            if rec.state == AUCTION and announcer < rec.announcer:
                rec.announcer = announcer           # two announcers, one round: lower id owns it
            return False
        rec.seq, rec.announcer, rec.announced_at = int(seq), announcer, float(now)
        rec.bids = {}
        rec.state = AUCTION                         # a newer round: older awards are void
        rec.reauction_due = False
        return True

    def on_bid(self, task_id, seq, robot, cost):
        """Record a bid for the open round."""
        rec = self.tasks.get(task_id)
        if rec is not None and rec.state == AUCTION and seq == rec.seq:
            rec.bids[robot] = float(cost)

    def pick_winner(self, task_id):
        """Deterministic winner of the open round: ``(robot, cost)``, lowest cost then robot id."""
        rec = self.tasks.get(task_id)
        if rec is None or not rec.bids:
            return None
        robot, bid = min(rec.bids.items(), key=lambda kv: (kv[1], kv[0]))
        return (robot, bid) if math.isfinite(bid) else None

    def on_award(self, task_id, seq, winner, lease_expiry, announcer, cost, now):
        """
        Apply an award; returns ``(accepted, dropped_winner)``.

        ``dropped_winner``: a robot that held the task under the award this one replaces (it
        must drop the task).
        """
        rec = self.tasks.get(task_id)
        if rec is None or rec.state == DONE or seq < rec.seq:
            return False, None                     # unknown, finished, or a newer round exists
        if seq == rec.award_seq and rec.state == ASSIGNED:
            if winner == rec.winner:               # duplicate delivery
                rec.lease_expiry = max(rec.lease_expiry, float(lease_expiry))
                return False, None
            if announcer >= rec.award_announcer:   # same round, two awards: lower announcer wins
                return False, None
        if seq == rec.seq and rec.state == AUCTION and rec.announcer and \
                announcer > rec.announcer:
            return False, None                     # not the owner of this round
        dropped = rec.winner if rec.state == ASSIGNED and rec.winner != winner else None
        rec.seq = max(rec.seq, int(seq))
        rec.award_seq, rec.award_announcer, rec.winner = int(seq), announcer, winner
        rec.lease_expiry, rec.award_cost = float(lease_expiry), float(cost)
        rec.state = ASSIGNED
        rec.awards += 1
        rec.failed_rounds = 0
        return True, dropped

    def on_renew(self, task_id, seq, robot, lease_expiry, released=False):
        """Holder's lease renewal; ``released`` gives the task back. Returns True if applied."""
        rec = self.tasks.get(task_id)
        if rec is None or rec.state != ASSIGNED or seq != rec.award_seq or robot != rec.winner:
            return False
        if released:
            self._back(rec, 'released', True)
        else:
            rec.lease_expiry = max(rec.lease_expiry, float(lease_expiry))
        return True

    def on_complete(self, task_id, robot):
        """Task served; returns True the first time."""
        rec = self.tasks.get(task_id)
        if rec is None or rec.state == DONE:
            return False
        rec.state, rec.completed_by = DONE, robot
        return True

    # ------------------------------------------------------------------ time
    def expire(self, now):
        """Return tasks to the pool whose lease ran out or whose award never came."""
        freed = []
        for rec in self.tasks.values():
            if rec.state == ASSIGNED and self.lease_enabled and now >= rec.lease_expiry:
                self._back(rec, 'lease_expired', True)
                freed.append((rec.task_id, 'lease_expired'))
            elif rec.state == AUCTION and \
                    now > rec.announced_at + self.bid_window + self.award_timeout:
                # bids but no award: the announcer failed mid-auction (a re-auction); no bids:
                # nobody was free, the task simply waits for its next round (backoff)
                with_bids = bool(rec.bids)
                self._back(rec, 'announcer_timeout' if with_bids else 'no_bids', with_bids)
                rec.failed_rounds += 1
                freed.append((rec.task_id, rec.back_reason))
        return freed

    def on_holder_renewal(self, task_id, seq, holder, lease_expiry, announcer=''):
        """
        Apply a holder renewal carried on its intent (``holder`` holds ``task_id``, round ``seq``).

        ``announcer``: who awarded that round ('' if unknown). Returns ``(applied,
        dropped_winner)``: an unknown round from its own holder is taken as the award (the Award
        message may have been lost); two awards of ONE round (two announcers that missed each
        other) are settled as for Award messages, the lower announcer winning, never without one.
        """
        rec = self.tasks.get(task_id)
        if rec is None or rec.state == DONE or seq < rec.seq:
            return False, None
        if rec.state == ASSIGNED and rec.award_seq == seq:
            if rec.winner != holder:
                if not announcer or announcer >= rec.award_announcer:
                    return False, None
                dropped = rec.winner                 # the lower announcer owns the round
                rec.award_announcer, rec.winner = announcer, holder
                rec.lease_expiry, rec.award_cost = float(lease_expiry), 0.0
                rec.awards += 1
                return True, dropped
            rec.lease_expiry = max(rec.lease_expiry, float(lease_expiry))
            if announcer and announcer < rec.award_announcer:
                rec.award_announcer = announcer      # learnt ('~' = unknown)
            return True, None
        if rec.state == ASSIGNED and rec.award_seq > seq:
            return False, None
        if announcer and seq == rec.seq and rec.state == AUCTION and rec.announcer and \
                announcer > rec.announcer:
            return False, None                       # not the owner of this round
        dropped = rec.winner if rec.state == ASSIGNED and rec.winner != holder else None
        rec.seq = max(rec.seq, int(seq))
        rec.award_seq, rec.award_announcer, rec.winner = int(seq), announcer or '~', holder
        rec.lease_expiry, rec.award_cost = float(lease_expiry), 0.0
        rec.state = ASSIGNED
        rec.awards += 1
        rec.failed_rounds = 0
        return True, dropped

    def digest(self):
        """
        Return ``(done, awards)`` for the digest.

        [(task_id, completed_by)], [(task_id, winner, seq, lease, announcer)].
        """
        done = sorted((r.task_id, r.completed_by) for r in self.tasks.values()
                      if r.state == DONE)
        awards = sorted((r.task_id, r.winner, r.award_seq, r.lease_expiry, r.award_announcer)
                        for r in self.tasks.values() if r.state == ASSIGNED)
        return done, awards

    def apply_digest(self, done, awards, now):
        """
        Merge a peer's digest.

        Returns ``(completed, superseded)``: task ids that just became DONE here, and
        (task_id, dropped_winner) pairs an award of a newer round replaced.
        """
        completed, superseded = [], []
        for task_id, by in done:
            if self.on_complete(task_id, by):
                completed.append(task_id)
        for entry in awards:
            task_id, winner, seq, lease = entry[:4]
            rec = self.tasks.get(task_id)
            if rec is None or rec.state == DONE or seq < rec.seq:
                continue
            if rec.state == ASSIGNED and rec.award_seq == seq and rec.winner == winner:
                continue                   # known award: only its holder's renewals extend it
            # its announcer settles a same-round tie as an Award would; '~' (unknown) sorts after
            # every robot id: such a digest entry never wins one
            announcer = (entry[4] if len(entry) > 4 else '') or '~'
            accepted, dropped = self.on_award(task_id, seq, winner, lease, announcer, 0.0, now)
            if accepted and dropped:
                superseded.append((task_id, dropped))
        return completed, superseded

    def extend_holder(self, robot, lease_expiry):
        """
        Extend every award ``robot`` holds to at least ``lease_expiry``; return how many.

        CLAUDE_CODE/06: another robot acknowledged a NEW renewal of ``robot``: it is alive and
        heard by the fleet (partitioned from this robot at most), so its awards stay its own.
        """
        n = 0
        for rec in self.tasks.values():
            if rec.state == ASSIGNED and rec.winner == robot:
                rec.lease_expiry = max(rec.lease_expiry, float(lease_expiry))
                n += 1
        return n

    def shift_leases(self, dt):
        """Push every award lease back by ``dt`` s (this robot's lease clock was stopped)."""
        for rec in self.tasks.values():
            if rec.state == ASSIGNED and rec.lease_expiry is not None:
                rec.lease_expiry += float(dt)

    def release_robot(self, robot):
        """``ReAuction``: every task ``robot`` holds goes back to the pool now; returns ids."""
        out = []
        for rec in self.tasks.values():
            if rec.state == ASSIGNED and rec.winner == robot:
                self._back(rec, f'reauction:{robot}', True)
                out.append(rec.task_id)
        return out

    def _back(self, rec, reason, reauction):
        rec.state, rec.back_reason = PENDING, reason
        rec.reauction_due = rec.reauction_due or reauction
        rec.bids = {}

    # ------------------------------------------------------------------ queries
    def is_reauction(self, task_id):
        """
        Would announcing ``task_id`` now be a re-auction rather than a normal round.

        True once after the task came back from an award (lease expiry, release, ``ReAuction``)
        or from a round whose announcer failed with bids in hand; False for a first round and
        for the rounds that follow one nobody bid on.
        """
        return self.tasks[task_id].reauction_due

    def pending(self):
        """Ids of tasks waiting for an auction, oldest first."""
        return sorted((r for r in self.tasks.values() if r.state == PENDING),
                      key=lambda r: (r.created, r.task_id))

    def holder_of(self, task_id):
        """Return the current holder (award of the newest round), or ''."""
        rec = self.tasks.get(task_id)
        return rec.winner if rec is not None and rec.state == ASSIGNED else ''

    def counts(self):
        """``{state: n}`` for every state."""
        out = {PENDING: 0, AUCTION: 0, ASSIGNED: 0, DONE: 0}
        for rec in self.tasks.values():
            out[rec.state] += 1
        return out
