"""Warehouse grid map: '.' free floor, '#' racks/walls. Cells are (row, col)."""

from __future__ import annotations

from collections import deque

Cell = tuple[int, int]
MOVES = ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1))  # wait, up, down, left, right


class Grid:
    def __init__(self, rows: list[str]):
        if not rows or len({len(r) for r in rows}) != 1:
            raise ValueError("grid rows must be non-empty and equal length")
        self.rows = rows
        self.height, self.width = len(rows), len(rows[0])

    @classmethod
    def parse(cls, text: str) -> "Grid":
        return cls([line for line in text.strip().splitlines()])

    def free(self, c: Cell) -> bool:
        r, k = c
        return 0 <= r < self.height and 0 <= k < self.width and self.rows[r][k] != "#"

    def neighbours(self, c: Cell):
        """Free cells reachable in one step, including staying put."""
        for dr, dc in MOVES:
            n = (c[0] + dr, c[1] + dc)
            if self.free(n):
                yield n

    def free_cells(self) -> list[Cell]:
        return [(r, c) for r in range(self.height) for c in range(self.width) if self.free((r, c))]

    def distances_from(self, goal: Cell) -> dict[Cell, int]:
        """Exact BFS distance to goal from every reachable cell (the A* heuristic)."""
        dist = {goal: 0}
        q = deque([goal])
        while q:
            c = q.popleft()
            for n in self.neighbours(c):
                if n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
        return dist
