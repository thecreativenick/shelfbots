# shelfbots

Plans a fleet of up to 200 warehouse robots fetching shelves for a stream of orders, with no
collisions or deadlocks, plus a replay page to watch them move.

## Status

Early scaffold. Done so far:

- Grid maps (`.` floor, `#` racks), BFS distance heuristics.
- Space-time A* against a reservation table (vertex, swap and "parked at goal" constraints).
- Prioritized planning, plus randomized priority restarts.
- An independent checker (shares no code with the planner) for vertex conflicts, head-on
  swaps, teleports, blocked cells and wrong start/goal.

Baseline on the 18×7 test warehouse, 20 seeds: 15 robots succeed 20/20; 25 robots (about 28%
of the floor occupied) only 5/20. The corridor-swap test is a strict `xfail` until CBS exists.

## Planned milestones

1. Conflict-Based Search for small conflict clusters inside prioritized planning, making the
   corridor-swap test pass and raising the 25-robot success rate.
2. Order stream: batching orders to shelves, task assignment (Hungarian / auction).
3. Lifelong planning: rolling-horizon replanning as new tasks arrive; deadlock detection.
4. Scale to 200 robots on a realistic layout; profile and speed up the hot loops.
5. Localhost replay page stepping through robot positions.

## Success bar

10,000 orders are fulfilled with zero collisions or deadlocks according to the independent
checker, and makespan stays within 15% of a shortest-paths lower bound.

## Development

```bash
pip install -e .[dev]
pytest
```
