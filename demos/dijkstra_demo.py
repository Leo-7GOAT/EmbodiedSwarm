import numpy as np

from embodied_swarm.navigation.dijkstra import dijkstra
from embodied_swarm.navigation.occupancy_grid import OccupancyGrid


def print_grid(
    grid: OccupancyGrid,
    path: list[tuple[int, int]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> None:

    canvas = []

    for row in range(grid.height):
        line = []

        for col in range(grid.width):
            node = (row, col)

            if node == start:
                char = "S"

            elif node == goal:
                char = "G"

            elif node in path:
                char = "*"

            elif not grid.is_free(row, col):
                char = "#"

            else:
                char = "."

            line.append(char)

        canvas.append(line)

    # 倒着打印，让 row 越大视觉上越靠上，
    # 与我们 world y 轴向上的习惯一致。
    for row in reversed(canvas):
        print(" ".join(row))


def main() -> None:

    data = np.array(
        [
            [0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    grid = OccupancyGrid(
        data=data,
        resolution=1.0,
    )

    start = (0, 0)
    goal = (4, 4)

    path = dijkstra(
        grid,
        start,
        goal,
    )

    print("=== Dijkstra Path ===")
    print(path)

    print()
    print("=== Path Length ===")
    print(len(path))

    print()
    print("=== Map ===")
    print_grid(
        grid,
        path,
        start,
        goal,
    )


if __name__ == "__main__":
    main()