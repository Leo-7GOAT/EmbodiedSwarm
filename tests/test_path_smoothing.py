import numpy as np

from embodied_swarm.navigation.occupancy_grid import OccupancyGrid
from embodied_swarm.navigation.path_smoothing import (
    bresenham_cells,
    line_is_free,
    simplify_path,
)


def test_bresenham_horizontal():
    cells = bresenham_cells(
        (2, 1),
        (2, 5),
    )

    assert cells == [
        (2, 1),
        (2, 2),
        (2, 3),
        (2, 4),
        (2, 5),
    ]


def test_bresenham_vertical():
    cells = bresenham_cells(
        (1, 3),
        (5, 3),
    )

    assert cells == [
        (1, 3),
        (2, 3),
        (3, 3),
        (4, 3),
        (5, 3),
    ]


def test_line_is_free_clear():
    grid = OccupancyGrid(
        data=np.zeros((5, 5), dtype=int),
        resolution=1.0,
    )

    assert line_is_free(
        grid,
        (0, 0),
        (0, 4),
    )


def test_line_is_free_blocked():
    data = np.zeros((5, 5), dtype=int)
    data[1, 2] = 1

    grid = OccupancyGrid(
        data=data,
        resolution=1.0,
    )

    assert not line_is_free(
        grid,
        (1, 0),
        (1, 4),
    )


def test_simplify_empty_path():
    grid = OccupancyGrid(
        data=np.zeros((3, 3), dtype=int),
        resolution=1.0,
    )

    assert simplify_path(
        grid,
        [],
    ) == []


def test_simplify_single_point():
    grid = OccupancyGrid(
        data=np.zeros((3, 3), dtype=int),
        resolution=1.0,
    )

    assert simplify_path(
        grid,
        [(1, 1)],
    ) == [(1, 1)]


def test_simplify_clear_path_to_endpoints():
    grid = OccupancyGrid(
        data=np.zeros((5, 5), dtype=int),
        resolution=1.0,
    )

    path = [
        (0, 0),
        (0, 1),
        (0, 2),
        (1, 2),
        (2, 2),
    ]

    simplified = simplify_path(
        grid,
        path,
    )

    assert simplified == [
        (0, 0),
        (2, 2),
    ]


def test_simplify_keeps_turn_around_obstacle():
    data = np.zeros((4, 4), dtype=int)
    data[1, 1] = 1

    grid = OccupancyGrid(
        data=data,
        resolution=1.0,
    )

    path = [
        (0, 0),
        (0, 1),
        (0, 2),
        (1, 2),
        (2, 2),
    ]

    simplified = simplify_path(
        grid,
        path,
    )

    assert simplified == [
        (0, 0),
        (1, 2),
        (2, 2),
    ]

    for start, end in zip(
        simplified,
        simplified[1:],
    ):
        assert line_is_free(
            grid,
            start,
            end,
        )