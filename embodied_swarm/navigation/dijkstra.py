from __future__ import annotations

from embodied_swarm.navigation.occupancy_grid import OccupancyGrid
import heapq

def get_neighbors(
    grid: OccupancyGrid,
    node: tuple[int, int],
) -> list[tuple[int, int]]:

    row, col = node
    candidates = [(row + 1, col),
                 (row - 1, col),
                 (row, col + 1),
                 (row, col - 1)]

    neighbors = []

    for candidate in candidates:
        if grid.is_free(*candidate):
            neighbors.append(candidate)

    return neighbors

def dijkstra(
        grid:OccupancyGrid,
        start: tuple[int, int],
        goal: tuple[int, int],
        stats: dict[str, int] | None = None,
)-> list[tuple[int, int]]:
    if not grid.is_free(*start):
        return []

    if not grid.is_free(*goal):
        return []

    expanded = 0

    open_heap = [
        (0.0, start)
    ]
    g_score = {
        start: 0.0
    }
    came_from = {}

    while open_heap:
        current_cost, current = heapq.heappop(open_heap)
        if current_cost > g_score[current]:
            continue
        if current == goal:
            break

        expanded += 1
        if stats is not None:
            stats["expanded"] = expanded

        for neighbor in get_neighbors(grid, current):
            new_cost = current_cost + 1.0
            old_cost = g_score.get(
                neighbor,
                float("inf"),
            )
            if new_cost < old_cost:
                g_score[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(
                    open_heap,
                    (new_cost, neighbor),
                )
    if goal not in g_score:
        return []
    return reconstruct_path(
            came_from,
            start,
            goal
        )

def reconstruct_path(
        came_from:dict[
            tuple[int, int],
            tuple[int, int],
        ],
        start:tuple[int, int],
        goal:tuple[int, int],
) -> list[tuple[int, int]]:
    path = [goal]
    current = goal
    while current != start:
        current = came_from[current]
        path.append(current)
    return path[::-1]

