import numpy as np

from embodied_swarm.navigation.astar import astar
from embodied_swarm.navigation.dijkstra import dijkstra
from embodied_swarm.navigation.occupancy_grid import OccupancyGrid
from embodied_swarm.navigation.path_smoothing import (
    simplify_path,
    line_is_free,
)


def main() -> None:
    # 30 x 30 occupancy grid
    # 0 = free
    # 1 = obstacle
    data = np.zeros(
        (30, 30),
        dtype=int,
    )

    # Vertical wall with one opening.
    #
    # The wall forces both planners to make a detour,
    # so we can compare Dijkstra and A* search efficiency.
    data[0:25, 15] = 1

    # Opening in the wall
    data[20, 15] = 0

    grid = OccupancyGrid(
        data=data,
        resolution=1.0,
    )

    start = (2, 2)
    goal = (27, 27)

    # --------------------------------------------------
    # Dijkstra
    # --------------------------------------------------

    dijkstra_stats: dict[str, int] = {}

    dijkstra_path = dijkstra(
        grid,
        start,
        goal,
        stats=dijkstra_stats,
    )

    # --------------------------------------------------
    # A*
    # --------------------------------------------------

    astar_stats: dict[str, int] = {}

    astar_path = astar(
        grid,
        start,
        goal,
        stats=astar_stats,
    )

    # --------------------------------------------------
    # Compare Dijkstra and A*
    # --------------------------------------------------

    print("=== Search Benchmark ===")
    print()

    print("Dijkstra")
    print(
        "path nodes:",
        len(dijkstra_path),
    )
    print(
        "expanded:",
        dijkstra_stats["expanded"],
    )

    print()

    print("A*")
    print(
        "path nodes:",
        len(astar_path),
    )
    print(
        "expanded:",
        astar_stats["expanded"],
    )

    print()

    # Both algorithms should find a shortest path
    # with the same number of nodes.
    assert len(dijkstra_path) == len(
        astar_path
    )

    print(
        "Shortest path length: MATCH"
    )

    if (
        astar_stats["expanded"]
        <= dijkstra_stats["expanded"]
    ):
        print(
            "A* search efficiency: PASS"
        )
    else:
        print(
            "A* search efficiency: FAIL"
        )

    # --------------------------------------------------
    # Path simplification
    # --------------------------------------------------

    simplified_path = simplify_path(
        grid,
        astar_path,
    )

    print()
    print("=== Path Simplification ===")
    print()

    print(
        "A* raw path nodes:",
        len(astar_path),
    )

    print(
        "Simplified path nodes:",
        len(simplified_path),
    )

    print()

    print(
        "Simplified path:"
    )

    for node in simplified_path:
        print(node)

    # --------------------------------------------------
    # Collision check for every simplified segment
    # --------------------------------------------------

    for start_node, end_node in zip(
        simplified_path,
        simplified_path[1:],
    ):
        assert line_is_free(
            grid,
            start_node,
            end_node,
        )

    print()
    print(
        "Simplified path collision check: PASS"
    )

    # --------------------------------------------------
    # Basic endpoint checks
    # --------------------------------------------------

    assert simplified_path[0] == start
    assert simplified_path[-1] == goal

    print(
        "Simplified path endpoints: PASS"
    )

    # --------------------------------------------------
    # Summary
    # --------------------------------------------------

    print()
    print("=== Summary ===")
    print()

    print(
        f"Dijkstra expanded : "
        f"{dijkstra_stats['expanded']}"
    )

    print(
        f"A* expanded       : "
        f"{astar_stats['expanded']}"
    )

    print(
        f"A* raw nodes      : "
        f"{len(astar_path)}"
    )

    print(
        f"Simplified nodes  : "
        f"{len(simplified_path)}"
    )


if __name__ == "__main__":
    main()