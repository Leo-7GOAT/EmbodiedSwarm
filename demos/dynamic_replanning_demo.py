import math

import numpy as np

from embodied_swarm.navigation.hybrid_astar import hybrid_astar
from embodied_swarm.navigation.occupancy_grid import OccupancyGrid
from embodied_swarm.navigation.replanning import (
    path_is_collision_free,
    replan_if_needed,
)


def main() -> None:
    # ==================================================
    # Build map
    # ==================================================

    data = np.zeros(
        (30, 30),
        dtype=int,
    )

    # Vertical wall
    data[:, 15] = 1

    # Opening
    data[11:19, 15] = 0

    grid = OccupancyGrid(
        data=data,
        resolution=1.0,
    )

    start = (
        3.5,
        5.5,
        0.0,
    )

    goal = (
        26.5,
        5.5,
        0.0,
    )

    # ==================================================
    # Initial planning
    # ==================================================

    initial_path = hybrid_astar(
        grid=grid,
        start=start,
        goal=goal,
    )

    if not initial_path:
        raise RuntimeError(
            "Initial planning failed"
        )

    print(
        "initial path points:",
        len(initial_path),
    )

    print(
        "initial safe:",
        path_is_collision_free(
            grid,
            initial_path,
        ),
    )

    # ==================================================
    # Robot executes about one third of the path
    # ==================================================

    current_index = (
        len(initial_path) // 3
    )

    current_pose = initial_path[
        current_index
    ]

    remaining_path = initial_path[
        current_index:
    ]

    print(
        "current pose:",
        current_pose,
    )

    # ==================================================
    # Simulate a newly observed obstacle
    # ==================================================

    blocked_index = (
        current_index
        + len(remaining_path) // 2
    )

    blocked_pose = initial_path[
        blocked_index
    ]

    blocked_row, blocked_col = (
        grid.world_to_grid(
            blocked_pose[0],
            blocked_pose[1],
        )
    )

    grid.data[
        blocked_row,
        blocked_col,
    ] = 1

    print(
        "injected obstacle cell:",
        (
            blocked_row,
            blocked_col,
        ),
    )

    print(
        "old remaining safe:",
        path_is_collision_free(
            grid,
            remaining_path,
        ),
    )

    # ==================================================
    # Online replanning
    # ==================================================

    replan_stats = {}

    active_path, replanned = (
        replan_if_needed(
            grid=grid,
            current_pose=current_pose,
            goal=goal,
            remaining_path=remaining_path,
            stats=replan_stats,
        )
    )

    print(
        "replanned:",
        replanned,
    )

    print(
        "replan success:",
        bool(active_path),
    )

    print(
        "expanded:",
        replan_stats["expanded"],
    )

    if not active_path:
        raise RuntimeError(
            "Replanning failed"
        )

    print(
        "new path safe:",
        path_is_collision_free(
            grid,
            active_path,
        ),
    )

    # ==================================================
    # Verify final goal
    # ==================================================

    final_pose = active_path[-1]

    position_error = math.hypot(
        goal[0] - final_pose[0],
        goal[1] - final_pose[1],
    )

    yaw_error = abs(
        math.atan2(
            math.sin(
                goal[2] - final_pose[2]
            ),
            math.cos(
                goal[2] - final_pose[2]
            ),
        )
    )

    print(
        "new end:",
        final_pose,
    )

    print(
        "goal position error:",
        position_error,
    )

    print(
        "goal yaw error:",
        yaw_error,
    )

    assert replanned

    assert path_is_collision_free(
        grid,
        active_path,
    )

    assert position_error <= 0.75

    assert yaw_error <= math.radians(
        15.0
    )

    print()
    print(
        "Dynamic replanning: PASS"
    )


if __name__ == "__main__":
    main()