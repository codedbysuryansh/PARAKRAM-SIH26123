"""
Reservation store with lease expiry and conflict queries (pure: no ROS).

Built from peers' ``Intent`` messages. Each owner's latest intent carries all of its reserved
cells and one absolute ``lease_expiry``; an owner's cells are valid while ``now < lease_expiry``.
Expiry is the ONLY release mechanism: a silent owner stops renewing and its cells become free
when the lease runs out, with no release message (CLAUDE_CODE/00, MASTER_BUILD_PLAN §1).

CLAUDE_CODE/06: an expired lease frees the owner's RESERVED space, not its body. Its last known
body envelope (the cells it occupied and the held authority it could still drive through) becomes
a GHOST: blocked for planning until the owner is heard again or a cell is seen empty
(``clear_ghost_cell``, from this robot's lidar). ``expire_by_lease=False`` is the
``reauction_baseline`` (no leases): entries never expire by time; only ``release()`` frees them.
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
    authority: tuple = ()               # CLAUDE_CODE/06: held cells it may drive through


class ReservationTable:
    """Latest reservations per owner, filtered by lease validity at query time."""

    def __init__(self, expire_by_lease=True):
        """Create an empty table (``expire_by_lease=False``: baseline, release-only)."""
        self._entries = {}
        self.expire_by_lease = bool(expire_by_lease)
        self.ghosts = {}                # owner -> set of cells its body may still occupy
        self.events = []                # (kind, owner, t, reclaimed cells) for the node to drain

    def update(self, entry):
        """
        Store ``entry`` if it is newer than the owner's current one; return True if stored.

        Newer = higher ``seq``; an out-of-order older message is ignored unless the stored lease
        has already expired or the sender evidently restarted (``seq`` jumped far backwards).
        """
        old = self._entries.get(entry.owner)
        if (old is None or entry.seq > old.seq or old.lease_expiry <= entry.heard_at
                or entry.seq + 100 < old.seq):
            if self.ghosts.pop(entry.owner, None) is not None:
                self.events.append(('peer_back', entry.owner, entry.heard_at, frozenset()))
            self._entries[entry.owner] = entry
            return True
        return False

    def forget(self, owner):
        """Drop an owner entirely (e.g. tests); normal operation relies on expiry."""
        self._entries.pop(owner, None)

    def prune(self, now):
        """Remove owners whose lease has expired (they become ghosts); return their ids."""
        if not self.expire_by_lease:
            return []
        expired = [o for o, e in self._entries.items() if e.lease_expiry <= now]
        for o in expired:
            self._to_ghost(o, now, 'lease_expired')
        return expired

    def release(self, owner, now):
        """Free ``owner``'s reservations on a release message (baseline); True if it held any."""
        if owner not in self._entries:
            return False
        self._to_ghost(owner, now, 'release')
        return True

    def _to_ghost(self, owner, now, kind):
        e = self._entries.pop(owner)
        body = set(e.occupied) | set(e.authority) or set(e.cells[:1])
        self.ghosts[owner] = body
        self.events.append((kind, owner, now, frozenset(set(e.cells) - body)))

    def ghost_cells(self):
        """Return every cell a silent robot's body may still occupy."""
        out = set()
        for cells in self.ghosts.values():
            out |= cells
        return frozenset(out)

    def clear_ghost_cell(self, cell, now):
        """Cell seen empty: no ghost there any more. Returns the owners it was cleared for."""
        owners = [o for o, cells in self.ghosts.items() if cell in cells]
        for o in owners:
            self.ghosts[o].discard(cell)
            self.events.append(('ghost_cleared', o, now, frozenset([cell])))
        return owners

    def drain_events(self):
        """Return and forget the events since the last call."""
        out, self.events = self.events, []
        return out

    def valid(self, now, exclude=None):
        """Owners with an unexpired lease: ``{owner: Entry}`` (baseline: not released)."""
        return {o: e for o, e in self._entries.items()
                if (e.lease_expiry > now or not self.expire_by_lease) and o != exclude}

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
