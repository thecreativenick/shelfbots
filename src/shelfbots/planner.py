"""Prioritized planning: robots plan one at a time with space-time A*, each avoiding
the cells and edges reserved by the robots planned before it.

Prioritized planning is fast but incomplete (an unlucky priority order can
fail where a solution exists). Later milestones add priority reordering,
windowed replanning for a continuous order stream, and CBS for small
conflict clusters.
"""

from __future__ import annotations

import heapq
import itertools
from dataclasses import dataclass, field

from .grid import Cell, Grid

Path = list[Cell]  # position at t = 0, 1, 2, ...; the robot stays at path[-1] afterwards


@dataclass
class Reservations:
    vertex: set[tuple[Cell, int]] = field(default_factory=set)
    edge: set[tuple[Cell, Cell, int]] = field(default_factory=set)   # (from, to, t) moving t -> t+1
    parked: dict[Cell, int] = field(default_factory=dict)            # cell -> time from which it's occupied forever
    last_use: dict[Cell, int] = field(default_factory=dict)          # cell -> latest reserved time

    def blocked(self, cell: Cell, t: int) -> bool:
        return (cell, t) in self.vertex or (cell in self.parked and t >= self.parked[cell])

    def add_path(self, path: Path) -> None:
        for t, c in enumerate(path):
            self.vertex.add((c, t))
            self.last_use[c] = max(self.last_use.get(c, -1), t)
            if t + 1 < len(path):
                self.edge.add((c, path[t + 1], t))
        self.parked[path[-1]] = len(path) - 1


def space_time_astar(grid: Grid, start: Cell, goal: Cell, res: Reservations,
                     max_time: int) -> Path | None:
    h = grid.distances_from(goal)
    if start not in h:
        return None
    tie = itertools.count()
    open_ = [(h[start], next(tie), 0, start)]
    parent: dict[tuple[Cell, int], tuple[Cell, int] | None] = {(start, 0): None}
    while open_:
        _, _, t, cell = heapq.heappop(open_)
        # Stop only if nobody needs this cell later (we will park here forever).
        if cell == goal and res.last_use.get(goal, -1) < t:
            path, node = [], (cell, t)
            while node:
                path.append(node[0])
                node = parent[node]
            return path[::-1]
        if t >= max_time:
            continue
        for n in grid.neighbours(cell):
            nt = t + 1
            if res.blocked(n, nt) or (n, cell, t) in res.edge:  # vertex conflict or head-on swap
                continue
            if (n, nt) in parent:
                continue
            parent[(n, nt)] = (cell, t)
            heapq.heappush(open_, (nt + h.get(n, 10 ** 9), next(tie), nt, n))
    return None


def plan_prioritized(grid: Grid, tasks: list[tuple[Cell, Cell]], max_time: int | None = None
                     ) -> list[Path] | None:
    """Plan (start, goal) pairs in the given priority order. None if some robot can't be planned."""
    if len({s for s, _ in tasks}) != len(tasks) or len({g for _, g in tasks}) != len(tasks):
        raise ValueError("starts and goals must be distinct")
    max_time = max_time or (grid.width * grid.height + 4 * len(tasks))
    res = Reservations()
    # Earlier robots may plan through a later robot's start cell; the later
    # robot then has to step out of the way in time, or planning fails.
    paths: list[Path] = []
    for start, goal in tasks:
        path = space_time_astar(grid, start, goal, res, max_time)
        if path is None:
            return None
        res.add_path(path)
        paths.append(path)
    return paths


def plan_with_restarts(grid: Grid, tasks: list[tuple[Cell, Cell]], attempts: int = 20,
                       seed: int = 0) -> list[Path] | None:
    """Prioritized planning over shuffled priority orders; paths come back in task order."""
    import random

    rng = random.Random(seed)
    order = list(range(len(tasks)))
    for attempt in range(attempts):
        if attempt:
            rng.shuffle(order)
        paths = plan_prioritized(grid, [tasks[i] for i in order])
        if paths is not None:
            out: list[Path] = [[] for _ in tasks]
            for i, p in zip(order, paths):
                out[i] = p
            return out
    return None
