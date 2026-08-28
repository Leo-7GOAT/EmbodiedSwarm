import numpy as np

from embodied_swarm.navigation.dijkstra import (
    dijkstra,
    get_neighbors,
)
from embodied_swarm.navigation.occupancy_grid import (
    OccupancyGrid,
)


def test_get_neighbors():
    grid = OccupancyGrid(
        data=np.array([
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ]),
        resolution=1.0,
    )

    neighbors = get_neighbors(
        grid,
        (0, 1),
    )

    assert set(neighbors) == {
        (0, 0),
        (0, 2),
    }


def test_dijkstra_open_grid():
    grid = OccupancyGrid(
        data=np.zeros((3, 3)),
        resolution=1.0,
    )

    path = dijkstra(
        grid,
        start=(0, 0),
        goal=(2, 2),
    )

    assert path[0] == (0, 0)
    assert path[-1] == (2, 2)

    # Manhattan shortest path:
    # 4 moves -> 5 nodes
    assert len(path) == 5


def test_dijkstra_avoids_obstacle():
    grid = OccupancyGrid(
        data=np.array([
            [0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0],
        ]),
        resolution=1.0,
    )

    path = dijkstra(
        grid,
        start=(0, 0),
        goal=(4, 4),
    )

    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)

    for row, col in path:
        assert grid.is_free(row, col)


def test_dijkstra_unreachable():
    grid = OccupancyGrid(
        data=np.array([
            [0, 1, 0],
            [0, 1, 0],
            [0, 1, 0],
        ]),
        resolution=1.0,
    )

    path = dijkstra(
        grid,
        start=(0, 0),
        goal=(0, 2),
    )

    assert path == []


def test_dijkstra_rejects_blocked_start():
    grid = OccupancyGrid(
        data=np.array([
            [1, 0],
            [0, 0],
        ]),
        resolution=1.0,
    )

    assert dijkstra(
        grid,
        start=(0, 0),
        goal=(1, 1),
    ) == []


def test_start_equals_goal():
    grid = OccupancyGrid(
        data=np.zeros((3, 3)),
        resolution=1.0,
    )

    path = dijkstra(
        grid,
        start=(1, 1),
        goal=(1, 1),
    )

    assert path == [(1, 1)]