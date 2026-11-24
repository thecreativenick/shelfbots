"""Independent plan checker. Shares no code with the planner on purpose."""

from __future__ import annotations

from .grid import Cell, Grid


def check_paths(grid: Grid, tasks: list[tuple[Cell, Cell]], paths: list[list[Cell]]) -> list[str]:
    problems: list[str] = []
    if len(paths) != len(tasks):
        return [f"{len(paths)} paths for {len(tasks)} tasks"]
    horizon = max((len(p) for p in paths), default=0)

    def at(i: int, t: int) -> Cell:
        p = paths[i]
        return p[min(t, len(p) - 1)]

    for i, ((start, goal), p) in enumerate(zip(tasks, paths)):
        if not p or p[0] != start:
            problems.append(f"robot {i}: does not start at {start}")
        if p and p[-1] != goal:
            problems.append(f"robot {i}: ends at {p[-1]}, not {goal}")
        for t, c in enumerate(p):
            if not grid.free(c):
                problems.append(f"robot {i}: in blocked cell {c} at t={t}")
            if t and abs(c[0] - p[t - 1][0]) + abs(c[1] - p[t - 1][1]) > 1:
                problems.append(f"robot {i}: jumps {p[t - 1]} -> {c} at t={t}")

    for t in range(horizon + 1):
        seen: dict[Cell, int] = {}
        for i in range(len(paths)):
            c = at(i, t)
            if c in seen:
                problems.append(f"robots {seen[c]} and {i}: both at {c} at t={t}")
            seen[c] = i
        if t < horizon:
            moves = {(at(i, t), at(i, t + 1)): i for i in range(len(paths)) if at(i, t) != at(i, t + 1)}
            for (a, b), i in moves.items():
                j = moves.get((b, a))
                if j is not None and i < j:
                    problems.append(f"robots {i} and {j}: swap {a}<->{b} at t={t}")
    return problems


def makespan(paths: list[list[Cell]]) -> int:
    return max(len(p) for p in paths) - 1 if paths else 0


def sum_of_costs(paths: list[list[Cell]]) -> int:
    return sum(len(p) - 1 for p in paths)
