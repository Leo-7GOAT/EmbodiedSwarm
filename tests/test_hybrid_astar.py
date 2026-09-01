import math

import numpy as np
import pytest

from embodied_swarm.navigation.hybrid_astar import (
    goal_reached,
    hybrid_astar,
    normalize_angle,
    pose_to_key,
    primitive_is_collision_free,
    propagate_motion,
    reconstruct_hybrid_path,
    simulate_motion_primitive,
    yaw_to_bin,
)
from embodied_swarm.navigation.occupancy_grid import OccupancyGrid


@pytest.fixture
def open_grid() -> OccupancyGrid:
    return OccupancyGrid(np.zeros((12, 12), dtype=int), resolution=1.0)


@pytest.mark.parametrize(
    ("angle", "expected"),
    [
        (0.0, 0.0),
        (2.0 * math.pi, 0.0),
        (3.0 * math.pi / 2.0, -math.pi / 2.0),
        (-3.0 * math.pi / 2.0, math.pi / 2.0),
    ],
)
def test_normalize_angle(angle: float, expected: float) -> None:
    assert normalize_angle(angle) == pytest.approx(expected)


def test_yaw_to_bin_wraps_and_rejects_invalid_bin_count() -> None:
    assert yaw_to_bin(0.0, 8) == 0
    assert yaw_to_bin(2.0 * math.pi, 8) == 0
    assert yaw_to_bin(-math.pi / 2.0, 8) == 6

    with pytest.raises(ValueError, match="positive"):
        yaw_to_bin(0.0, 0)


def test_propagate_motion_straight_and_turning() -> None:
    straight = propagate_motion((1.0, 2.0, 0.0), 2.0, 0.0, 0.5)
    assert straight == pytest.approx((2.0, 2.0, 0.0))

    turning = propagate_motion((0.0, 0.0, math.pi / 2.0), 1.0, 0.5, 0.2)
    assert turning == pytest.approx((0.0, 0.2, math.pi / 2.0 + 0.1))


def test_motion_primitive_uses_exact_duration() -> None:
    trajectory = simulate_motion_primitive(
        pose=(0.0, 0.0, 0.0),
        linear_velocity=1.0,
        angular_velocity=0.0,
        duration=1.0,
        integration_dt=0.3,
    )

    assert len(trajectory) == 5
    assert trajectory[0] == (0.0, 0.0, 0.0)
    assert trajectory[-1] == pytest.approx((1.0, 0.0, 0.0))


def test_primitive_collision_checks_intermediate_samples(open_grid: OccupancyGrid) -> None:
    open_grid.data[1, 2] = 1

    safe = [(0.5, 0.5, 0.0), (1.5, 0.5, 0.0)]
    blocked = [(1.5, 1.5, 0.0), (2.5, 1.5, 0.0), (3.5, 1.5, 0.0)]

    assert primitive_is_collision_free(open_grid, safe)
    assert not primitive_is_collision_free(open_grid, blocked)


def test_pose_to_key_combines_grid_cell_and_heading(open_grid: OccupancyGrid) -> None:
    assert pose_to_key(open_grid, (3.5, 4.5, math.pi / 2.0), 8) == (4, 3, 2)


def test_goal_reached_checks_position_and_heading() -> None:
    goal = (5.0, 5.0, math.pi)

    assert goal_reached((5.5, 5.0, -math.pi), goal)
    assert not goal_reached((6.0, 5.0, math.pi), goal)
    assert not goal_reached((5.0, 5.0, math.pi / 2.0), goal)


def test_hybrid_astar_open_map_returns_continuous_safe_path(open_grid: OccupancyGrid) -> None:
    start = (1.5, 2.5, 0.0)
    goal = (9.5, 2.5, 0.0)
    stats: dict[str, int] = {}

    path = hybrid_astar(open_grid, start, goal, stats=stats)

    assert path
    assert path[0] == start
    assert goal_reached(path[-1], goal)
    assert primitive_is_collision_free(open_grid, path)
    assert stats["expanded"] > 0
    assert any(not float(x).is_integer() for x, _, _ in path)


def test_hybrid_astar_routes_through_wall_opening() -> None:
    data = np.zeros((20, 20), dtype=int)
    data[:, 10] = 1
    data[8:13, 10] = 0
    grid = OccupancyGrid(data, resolution=1.0)

    start = (2.5, 3.5, 0.0)
    goal = (17.5, 3.5, 0.0)
    path = hybrid_astar(grid, start, goal)

    assert path
    assert primitive_is_collision_free(grid, path)
    assert goal_reached(path[-1], goal)
    assert any(8.0 <= y < 13.0 for x, y, _ in path if 10.0 <= x < 11.0)


def test_reconstruct_hybrid_path_preserves_segment_samples() -> None:
    start_key = (0, 0, 0)
    middle_key = (0, 1, 0)
    goal_key = (0, 2, 0)
    start = (0.5, 0.5, 0.0)
    middle = (1.5, 0.5, 0.0)
    goal = (2.5, 0.5, 0.0)

    came_from = {
        middle_key: (start_key, [start, middle]),
        goal_key: (middle_key, [middle, goal]),
    }

    assert reconstruct_hybrid_path(
        came_from, start_key, goal_key, start
    ) == [start, middle, goal]
