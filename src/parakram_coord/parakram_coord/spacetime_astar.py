"""
Windowed space-time A* over a 4-connected grid (pure: no ROS).

States are ``(cell, t)``; every action (move to a 4-neighbour, or wait) costs 1, so ``g == t``.
The heuristic is the exact static shortest-path distance (BFS over the shelf-free grid), which
is admissible and consistent, so the first goal state popped is an earliest arrival.

Windowing (WHCA*-style): the search never expands past ``t == window``. If no goal state is
reachable within the window, the depth-``window`` state with the smallest ``window + h`` is
returned instead (the best truncated plan), with ``reached = False``.

Dynamic constraints:
* ``blocked``        cells forbidden at every ``t >= 1`` (the start cell is always allowed at t=0)
* ``blocked_st``     ``(cell, t)`` vertex constraints
* ``blocked_edges``  ``(u, v, t)``: moving ``u -> v`` between ``t`` and ``t + 1`` is forbidden
"""

from collections import deque
import heapq
import math

INF = math.inf


def distance_field(sources, neighbors, passable=None):
    """Exact BFS distance from every reachable cell to the nearest of ``sources``."""
    dist = {}
    queue = deque()
    for s in sources:
        if s not in dist and (passable is None or passable(s)):
            dist[s] = 0
            queue.append(s)
    while queue:
        c = queue.popleft()
        for n in neighbors(c):
            if n not in dist and (passable is None or passable(n)):
                dist[n] = dist[c] + 1
                queue.append(n)
    return dist


def spacetime_astar(start, goal, window, neighbors, heuristic, blocked=frozenset(),
                    blocked_st=frozenset(), blocked_edges=frozenset(), goal_test=None,
                    max_expansions=200000):
    """
    Plan from ``start`` towards ``goal`` within ``window`` steps.

    ``neighbors(cell)`` yields the static 4-neighbours (shelves already excluded).
    ``heuristic`` is a dict ``cell -> distance-to-goal`` (missing = unreachable) or a callable.
    ``goal_test(cell)`` overrides ``cell == goal`` (e.g. "any cell off a pusher's path").
    Returns ``(path, reached)`` where ``path[t]`` is the cell at step ``t`` (``path[0] == start``).
    """
    if window < 0:
        raise ValueError('window must be >= 0')
    if goal_test is None:
        def goal_test(cell):
            return cell == goal
    if callable(heuristic):
        h_of = heuristic
    else:
        def h_of(cell):
            return heuristic.get(cell, INF)

    h0 = h_of(start)
    if h0 == INF:
        return [start], False
    counter = 0
    open_heap = [(h0, h0, counter, start, 0)]
    parent = {(start, 0): None}
    closed = set()
    expansions = 0
    while open_heap:
        _, h, _, cell, t = heapq.heappop(open_heap)
        state = (cell, t)
        if state in closed:
            continue
        closed.add(state)
        if goal_test(cell):
            return _reconstruct(parent, state), True
        if t >= window:
            return _reconstruct(parent, state), False
        expansions += 1
        if expansions > max_expansions:
            break
        nt = t + 1
        for nxt in _successors(cell, neighbors):
            if nxt in blocked:
                continue
            if (nxt, nt) in blocked_st or (cell, nxt, t) in blocked_edges:
                continue
            nstate = (nxt, nt)
            if nstate in closed or nstate in parent:
                continue
            nh = h_of(nxt)
            if nh == INF:
                continue
            parent[nstate] = state
            counter += 1
            heapq.heappush(open_heap, (nt + nh, nh, counter, nxt, nt))
    return [start], False


def _successors(cell, neighbors):
    yield cell  # wait
    yield from neighbors(cell)


def _reconstruct(parent, state):
    path = []
    while state is not None:
        path.append(state[0])
        state = parent[state]
    path.reverse()
    return path


def compress(path):
    """Drop consecutive duplicates (waits): the sequence of distinct cells visited."""
    out = []
    for c in path:
        if not out or out[-1] != c:
            out.append(c)
    return out


def moving_prefix(path):
    """Cells entered by the initial uninterrupted run of moves in ``path`` (stops at a wait)."""
    out = []
    for prev, cur in zip(path, path[1:]):
        if cur == prev:
            break
        out.append(cur)
    return out
