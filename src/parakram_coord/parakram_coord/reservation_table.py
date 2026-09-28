"""
Reservation store with lease expiry and conflict queries (pure: no ROS).

Built from peers' ``Intent`` messages. Each owner's latest intent carries all of its reserved
cells and one absolute ``lease_expiry``; an owner's cells are valid while ``now < lease_expiry``.
Expiry is the ONLY release mechanism: a silent owner stops renewing and its cells become free
when the lease runs out, with no release message (CLAUDE_CODE/00, MASTER_BUILD_PLAN §1).
"""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Entry:
    """One owner's latest reservations."""

    owner: str
    seq: int
    priority: int
    cells: tuple            # all reserved cells: occupied first, then claims ahead
    occupied: frozenset     # cells physically occupied (derived from the owner's pose)
    planned: tuple          # desired path (distinct cells, owner's centre first)
    lease_expiry: float
    heard_at: float
    pose: tuple = field(default=None)   # (x, y, yaw) or None


class ReservationTable:
    """Latest reservations per owner, filtered by lease validity at query time."""

    def __init__(self):
        """Create an empty table."""
        self._entries = {}

    def update(self, entry):
        """
        Store ``entry`` if it is newer than the owner's current one; return True if stored.

        Newer = higher ``seq``; an out-of-order older message is ignored unless the stored lease
        has already expired or the sender evidently restarted (``seq`` jumped far backwards).
        """
        old = self._entries.get(entry.owner)
        if (old is None or entry.seq > old.seq or old.lease_expiry <= entry.heard_at
                or entry.seq + 100 < old.seq):
            self._entries[entry.owner] = entry
            return True
        return False

    def forget(self, owner):
        """Drop an owner entirely (e.g. tests); normal operation relies on expiry."""
        self._entries.pop(owner, None)

    def prune(self, now):
        """Remove owners whose lease has expired; return their ids."""
        expired = [o for o, e in self._entries.items() if e.lease_expiry <= now]
        for o in expired:
            del self._entries[o]
        return expired

    def valid(self, now, exclude=None):
        """Owners with an unexpired lease: ``{owner: Entry}``."""
        return {o: e for o, e in self._entries.items()
                if e.lease_expiry > now and o != exclude}

    def fresh(self, now, timeout, exclude=None):
        """Return valid owners heard within ``timeout`` (their plan/priority info is current)."""
        return {o: e for o, e in self.valid(now, exclude).items() if now - e.heard_at <= timeout}

    def owners_of(self, cell, now, exclude=None):
        """Entries of valid owners reserving ``cell``."""
        return [e for e in self.valid(now, exclude).values() if cell in e.cells]

    def is_reserved(self, cell, now, exclude=None):
        """Return True if any valid owner (other than ``exclude``) reserves ``cell``."""
        return bool(self.owners_of(cell, now, exclude))

    def reserved_cells(self, now, exclude=None):
        """Map ``cell -> [owner, ...]`` over all valid reservations."""
        out = {}
        for e in self.valid(now, exclude).values():
            for c in e.cells:
                out.setdefault(c, []).append(e.owner)
        return out

    def conflicts(self, cells, now, exclude=None):
        """Map each of ``cells`` that another valid owner also reserves to those owners."""
        table = self.reserved_cells(now, exclude)
        return {c: table[c] for c in cells if c in table}

    def __len__(self):
        """Return the number of owners stored (valid or not)."""
        return len(self._entries)
