import numpy as np
import pytest

from embodied_swarm.navigation.occupancy_grid import (
    OccupancyGrid,
)


@pytest.fixture
def grid():
    data = np.array(
        [
            [0, 0, 0],
            [0, 1, 0],
            [0, 0, 0],
        ]
    )

    return OccupancyGrid(
        data=data,
        resolution=0.5,
        origin_x=-1.0,
        origin_y=-2.0,
    )


def test_shape(grid):
    assert grid.height == 3
    assert grid.width == 3


def test_world_to_grid(grid):
    row, col = grid.world_to_grid(
        0.2,
        -0.6,
    )

    assert row == 2
    assert col == 2

    assert isinstance(row, int)
    assert isinstance(col, int)


def test_grid_to_world(grid):
    x, y = grid.grid_to_world(
        2,
        2,
    )

    assert x == pytest.approx(0.25)
    assert y == pytest.approx(-0.75)


@pytest.mark.parametrize(
    "row,col,expected",
    [
        (0, 0, True),
        (2, 2, True),
        (-1, 0, False),
        (0, -1, False),
        (3, 0, False),
        (0, 3, False),
    ],
)
def test_is_inside(
    grid,
    row,
    col,
    expected,
):
    assert grid.is_inside(row, col) is expected


def test_free_cell(grid):
    assert grid.is_free(0, 0)


def test_occupied_cell(grid):
    assert not grid.is_free(1, 1)


def test_outside_is_not_free(grid):
    assert not grid.is_free(-1, 0)


def test_rejects_non_2d_data():
    with pytest.raises(ValueError):
        OccupancyGrid(
            data=np.zeros(10),
            resolution=0.5,
        )


@pytest.mark.parametrize(
    "resolution",
    [
        0.0,
        -0.1,
        -1.0,
    ],
)
def test_rejects_invalid_resolution(
    resolution,
):
    with pytest.raises(ValueError):
        OccupancyGrid(
            data=np.zeros((3, 3)),
            resolution=resolution,
        )