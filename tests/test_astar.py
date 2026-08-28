import numpy as np
import pytest

from embodied_swarm.navigation.astar import (
    astar,
    manhattan_distance,
)
from embodied_swarm.navigation.dijkstra import dijkstra
from embodied_swarm.navigation.occupancy_grid import OccupancyGrid


def test_manhattan_distance():
    assert manhattan_distance(
        (2, 1),
        (5, 4),
    ) == pytest.approx(6.0)

    assert manhattan_distance(
        (0, 0),
        (0, 0),
    ) == pytest.approx(0.0)

    assert manhattan_distance(
        (0, 0),
        (2, 3),
    ) == pytest.approx(5.0)


def test_astar_open_grid():
    grid = OccupancyGrid(
        data=np.zeros((3, 3)),
        resolution=1.0,
    )

    path = astar(
        grid,
        start=(0, 0),
        goal=(2, 2),
    )

    assert path[0] == (0, 0)
    assert path[-1] == (2, 2)

    # Manhattan shortest path:
    # 4 moves = 5 nodes
    assert len(path) == 5


def test_astar_avoids_obstacles():
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

    path = astar(
        grid,
        start=(0, 0),
        goal=(4, 4),
    )

    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)

    for node in path:
        assert grid.is_free(*node)


def test_astar_unreachable():
    grid = OccupancyGrid(
        data=np.array([
            [0, 1, 0],
            [0, 1, 0],
            [0, 1, 0],
        ]),
        resolution=1.0,
    )

    path = astar(
        grid,
        start=(0, 0),
        goal=(0, 2),
    )

    assert path == []


def test_astar_blocked_start():
    grid = OccupancyGrid(
        data=np.array([
            [1, 0],
            [0, 0],
        ]),
        resolution=1.0,
    )

    path = astar(
        grid,
        start=(0, 0),
        goal=(1, 1),
    )

    assert path == []


def test_astar_blocked_goal():
    grid = OccupancyGrid(
        data=np.array([
            [0, 0],
            [0, 1],
        ]),
        resolution=1.0,
    )

    path = astar(
        grid,
        start=(0, 0),
        goal=(1, 1),
    )

    assert path == []


def test_astar_start_equals_goal():
    grid = OccupancyGrid(
        data=np.zeros((3, 3)),
        resolution=1.0,
    )

    path = astar(
        grid,
        start=(1, 1),
        goal=(1, 1),
    )

    assert path == [
        (1, 1)
    ]


def test_astar_matches_dijkstra_shortest_length():
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

    start = (0, 0)
    goal = (4, 4)

    astar_path = astar(
        grid,
        start,
        goal,
    )

    dijkstra_path = dijkstra(
        grid,
        start,
        goal,
    )

    assert len(astar_path) == len(
        dijkstra_path
    )