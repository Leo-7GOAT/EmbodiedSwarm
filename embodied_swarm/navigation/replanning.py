from embodied_swarm.navigation.hybrid_astar import hybrid_astar
from embodied_swarm.navigation.occupancy_grid import OccupancyGrid


Pose = tuple[float, float, float]


def path_is_collision_free(
    grid: OccupancyGrid,
    path: list[Pose],
) -> bool:
    """
    Check whether every pose on a path is collision-free.
    """

    for pose in path:
        row, col = grid.world_to_grid(
            pose[0],
            pose[1],
        )

        if not grid.is_free(
            row,
            col,
        ):
            return False

    return True


def replan_if_needed(
    grid: OccupancyGrid,
    current_pose: Pose,
    goal: Pose,
    remaining_path: list[Pose],
    linear_velocity: float = 1.0,
    max_angular_velocity: float = 0.6,
    primitive_duration: float = 1.0,
    integration_dt: float = 0.1,
    num_yaw_bins: int = 72,
    turn_penalty: float = 0.2,
    stats: dict | None = None,
) -> tuple[list[Pose], bool]:
    """
    Replan from the current robot pose if the remaining path
    becomes invalid.

    Returns:
        path:
            Existing remaining path if it is still valid,
            otherwise the newly planned path.

        replanned:
            True if replanning was triggered.
    """

    # Existing plan is still valid.
    if path_is_collision_free(
        grid,
        remaining_path,
    ):
        if stats is not None:
            stats["replanned"] = False
            stats["replan_success"] = True
            stats["expanded"] = 0

        return remaining_path, False

    # Existing plan has become invalid.
    planner_stats: dict[str, int] = {}

    new_path = hybrid_astar(
        grid=grid,
        start=current_pose,
        goal=goal,
        linear_velocity=linear_velocity,
        max_angular_velocity=max_angular_velocity,
        primitive_duration=primitive_duration,
        integration_dt=integration_dt,
        num_yaw_bins=num_yaw_bins,
        turn_penalty=turn_penalty,
        stats=planner_stats,
    )

    if stats is not None:
        stats["replanned"] = True
        stats["replan_success"] = bool(new_path)
        stats["expanded"] = planner_stats.get(
            "expanded",
            0,
        )

    return new_path, True