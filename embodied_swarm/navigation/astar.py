import heapq

from embodied_swarm.navigation.dijkstra import (
    get_neighbors,
    reconstruct_path,
)
from embodied_swarm.navigation.occupancy_grid import (
    OccupancyGrid,
)

def manhattan_distance(
    node: tuple[int, int],
    goal: tuple[int, int],
) -> float:
    row, col = node
    goal_row, goal_col = goal
    h = abs(row - goal_row) + abs(col - goal_col)
    return h

def astar(
    grid: OccupancyGrid,
    start: tuple[int, int],
    goal: tuple[int, int],
    stats: dict[str, int] | None = None
) -> list[tuple[int, int]]:

    if not grid.is_free(*start):
        return []

    if not grid.is_free(*goal):
        return []

    g_score = {
        start: 0.0
    }

    came_from = {}

    start_h = manhattan_distance(
        start,
        goal,
    )

    start_f = start_h

    open_heap = [
        (start_f, 0.0, start)
    ]

    expanded = 0

    while open_heap:
        current_f, current_g, current = heapq.heappop(open_heap)
        if current_g > g_score[current]:
            continue
        if current == goal:
            break

        expanded += 1
        if stats is not None:
            stats["expanded"] = expanded

        for neighbor in get_neighbors(grid, current):
            new_g = current_g + 1.0
            old_g = g_score.get(
                neighbor,
                float("inf"),
            )
            if new_g < old_g:
                g_score[neighbor] = new_g
                came_from[neighbor] = current

                h=manhattan_distance(neighbor, goal)

                new_f = new_g + h

                heapq.heappush(open_heap, (new_f, new_g, neighbor))

    if goal not in g_score:
        return []
    return reconstruct_path(
        came_from,
        start,
        goal
    )
