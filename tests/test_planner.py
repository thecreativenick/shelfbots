import random

import pytest

from shelfbots import Grid, check_paths, makespan, plan_prioritized
from shelfbots.planner import plan_with_restarts


def test_single_robot_shortest_path():
    g = Grid.parse("""
.....
.###.
.....
""")
    [path] = plan_prioritized(g, [((0, 0), (2, 4))])
    assert len(path) - 1 == 6
    assert check_paths(g, [((0, 0), (2, 4))], [path]) == []


@pytest.mark.xfail(strict=True, reason="needs CBS: prioritized planning can't make one robot "
                   "step into the pocket for the other (milestone 2)")
def test_corridor_swap_uses_side_pocket():
    # Two robots must swap ends of a 1-wide corridor; the pocket at (0, 2) makes it possible.
    g = Grid.parse("""
##.##
.....
""")
    tasks = [((1, 0), (1, 4)), ((1, 4), (1, 0))]
    paths = plan_with_restarts(g, tasks)
    assert paths is not None
    assert check_paths(g, tasks, paths) == []


def test_impossible_swap_returns_none():
    g = Grid.parse("...")
    assert plan_prioritized(g, [((0, 0), (0, 2)), ((0, 2), (0, 0))]) is None


def test_later_robot_waits_for_parked_goal():
    # Robot 0 parks at (0, 1), which robot 1 would like to drive through.
    g = Grid.parse("""
...
...
""")
    tasks = [((0, 0), (0, 1)), ((0, 2), (0, 0))]
    paths = plan_prioritized(g, tasks)
    assert check_paths(g, tasks, paths) == []
    assert (0, 1) not in paths[1][2:]  # never enters the parked cell once robot 0 sits there


def test_checker_catches_conflicts():
    g = Grid.parse("....")
    tasks = [((0, 0), (0, 1)), ((0, 1), (0, 0))]
    swap = [[(0, 0), (0, 1)], [(0, 1), (0, 0)]]
    assert any("swap" in p for p in check_paths(g, tasks, swap))
    tasks = [((0, 0), (0, 2)), ((0, 3), (0, 2))]
    clash = [[(0, 0), (0, 1), (0, 2)], [(0, 3), (0, 2)]]
    assert any("both at" in p for p in check_paths(g, tasks, clash))
    assert any("jumps" in p for p in check_paths(g, [((0, 0), (0, 3))], [[(0, 0), (0, 3)]]))


WAREHOUSE = """
..................
.##.##.##.##.##.##
..................
.##.##.##.##.##.##
..................
.##.##.##.##.##.##
..................
"""


@pytest.mark.parametrize("seed", range(10))
def test_random_fleet_in_warehouse(seed):
    g = Grid.parse(WAREHOUSE)
    rng = random.Random(seed)
    cells = g.free_cells()
    n = 15
    tasks = list(zip(rng.sample(cells, n), rng.sample(cells, n)))
    paths = plan_with_restarts(g, tasks)
    assert paths is not None
    assert check_paths(g, tasks, paths) == []
    lower_bound = max(g.distances_from(goal)[start] for start, goal in tasks)
    assert makespan(paths) >= lower_bound
