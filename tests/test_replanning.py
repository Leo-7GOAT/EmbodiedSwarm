import numpy as np

from embodied_swarm.navigation.occupancy_grid import OccupancyGrid
from embodied_swarm.navigation.replanning import (
    path_is_collision_free,
    replan_if_needed,
)


def test_path_is_collision_free_detects_map_update() -> None:
    data = np.zeros((8, 8), dtype=int)
    grid = OccupancyGrid(data, resolution=1.0)
    path = [(1.5, 1.5, 0.0), (2.5, 1.5, 0.0), (3.5, 1.5, 0.0)]

    assert path_is_collision_free(grid, path)

    grid.data[1, 2] = 1
    assert not path_is_collision_free(grid, path)


def test_replan_not_triggered_when_remaining_path_is_valid() -> None:
    grid = OccupancyGrid(np.zeros((8, 8), dtype=int), resolution=1.0)
    remaining = [(1.5, 1.5, 0.0), (2.5, 1.5, 0.0)]
    stats: dict = {}

    active, replanned = replan_if_needed(
        grid=grid,
        current_pose=remaining[0],
        goal=remaining[-1],
        remaining_path=remaining,
        stats=stats,
    )

    assert active is remaining
    assert not replanned
    assert stats == {"replanned": False, "replan_success": True, "expanded": 0}


def test_replan_starts_from_current_pose_after_path_invalidation() -> None:
    data = np.zeros((20, 20), dtype=int)
    grid = OccupancyGrid(data, resolution=1.0)
    current = (2.5, 5.5, 0.0)
    goal = (16.5, 5.5, 0.0)
    remaining = [
        (x + 0.5, 5.5, 0.0)
        for x in range(2, 17)
    ]
    grid.data[5, 9] = 1
    stats: dict = {}

    active, replanned = replan_if_needed(
        grid=grid,
        current_pose=current,
        goal=goal,
        remaining_path=remaining,
        stats=stats,
    )

    assert replanned
    assert active
    assert active[0] == current
    assert path_is_collision_free(grid, active)
    assert stats["replanned"] is True
    assert stats["replan_success"] is True
    assert stats["expanded"] > 0


def test_failed_replan_is_reported_without_reusing_invalid_path() -> None:
    data = np.zeros((10, 10), dtype=int)
    grid = OccupancyGrid(data, resolution=1.0)
    current = (1.5, 1.5, 0.0)
    goal = (8.5, 1.5, 0.0)
    remaining = [current, (4.5, 1.5, 0.0), goal]
    grid.data[1, 4] = 1
    grid.data[1, 8] = 1
    stats: dict = {}

    active, replanned = replan_if_needed(
        grid=grid,
        current_pose=current,
        goal=goal,
        remaining_path=remaining,
        stats=stats,
    )

    assert replanned
    assert active == []
    assert stats["replanned"] is True
    assert stats["replan_success"] is False
    assert stats["expanded"] == 0
