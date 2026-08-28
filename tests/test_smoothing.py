import numpy as np

from embodied_swarm.navigation.occupancy_grid import (
    OccupancyGrid,
)
from embodied_swarm.navigation.path_smoothing import (
    continuous_line_is_free,
    smooth_path,
)


def make_free_grid(
    height: int = 10,
    width: int = 10,
) -> OccupancyGrid:
    return OccupancyGrid(
        data=np.zeros(
            (height, width),
            dtype=int,
        ),
        resolution=1.0,
    )


def test_smooth_path_keeps_endpoints():
    grid = make_free_grid()

    path = [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
        (2.0, 1.0),
    ]

    smoothed = smooth_path(
        grid,
        path,
    )

    assert smoothed[0] == path[0]
    assert smoothed[-1] == path[-1]


def test_smooth_path_does_not_modify_input():
    grid = make_free_grid()

    path = [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
        (2.0, 1.0),
    ]

    original_copy = path.copy()

    smooth_path(
        grid,
        path,
    )

    assert path == original_copy


def test_smooth_path_empty():
    grid = make_free_grid()

    assert smooth_path(
        grid,
        [],
    ) == []


def test_smooth_path_single_point():
    grid = make_free_grid()

    path = [
        (1.0, 2.0),
    ]

    assert smooth_path(
        grid,
        path,
    ) == path


def test_smooth_path_reduces_corner_sharpness():
    grid = make_free_grid()

    path = [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
        (2.0, 1.0),
    ]

    smoothed = smooth_path(
        grid,
        path,
        weight_data=0.5,
        weight_smooth=0.2,
        tolerance=1e-8,
        max_iterations=1000,
    )

    # 两个中间点都应该移动
    assert not np.allclose(
        smoothed[1],
        path[1],
    )

    assert not np.allclose(
        smoothed[2],
        path[2],
    )

    # 原来第一个折点 col=0，
    # 平滑后应该往内侧移动
    assert smoothed[1][1] > 0.0

    # 第二个折点原来 col=1，
    # 平滑后也应往内侧移动
    assert smoothed[2][1] < 1.0


def test_smooth_path_straight_line_stays_straight():
    grid = make_free_grid()

    path = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
    ]

    smoothed = smooth_path(
        grid,
        path,
    )

    assert np.allclose(
        np.asarray(smoothed),
        np.asarray(path),
        atol=1e-5,
    )


def test_continuous_line_is_free_clear():
    grid = make_free_grid(
        height=5,
        width=5,
    )

    assert continuous_line_is_free(
        grid,
        (0.0, 0.0),
        (4.0, 4.0),
    )


def test_continuous_line_is_free_blocked():
    data = np.zeros(
        (5, 5),
        dtype=int,
    )

    data[2, 2] = 1

    grid = OccupancyGrid(
        data=data,
        resolution=1.0,
    )

    assert not continuous_line_is_free(
        grid,
        (0.0, 0.0),
        (4.0, 4.0),
    )


def test_smoothing_remains_collision_free():
    data = np.zeros(
        (6, 6),
        dtype=int,
    )

    # Vertical obstacle wall.
    #
    # row
    # 5  . . . . * .
    # 4  . . # . * .
    # 3  . . # . * .
    # 2  . . # . * .
    # 1  . . # . * .
    # 0  S * * * * .
    #
    data[1:5, 2] = 1

    grid = OccupancyGrid(
        data=data,
        resolution=1.0,
    )

    # Dense path goes around the wall.
    path = [
        (0.0, 0.0),
        (0.0, 1.0),
        (0.0, 2.0),
        (0.0, 3.0),
        (0.0, 4.0),
        (1.0, 4.0),
        (2.0, 4.0),
        (3.0, 4.0),
        (4.0, 4.0),
        (5.0, 4.0),
        (5.0, 3.0),
        (5.0, 2.0),
        (5.0, 1.0),
        (5.0, 0.0),
    ]

    smoothed = smooth_path(
        grid,
        path,
        weight_data=0.5,
        weight_smooth=0.2,
        tolerance=1e-8,
        max_iterations=1000,
    )

    # 起终点必须保持
    assert smoothed[0] == path[0]
    assert smoothed[-1] == path[-1]

    # 至少有中间点真的被平滑过
    assert any(
        not np.allclose(
            old_point,
            new_point,
        )
        for old_point, new_point in zip(
            path[1:-1],
            smoothed[1:-1],
        )
    )

    # 最重要的安全验收：
    # 平滑后的每一段都不能穿障碍。
    for start, end in zip(
        smoothed,
        smoothed[1:],
    ):
        assert continuous_line_is_free(
            grid,
            start,
            end,
        )