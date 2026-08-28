import math

import numpy as np

from embodied_swarm.control.pure_pursuit import (
    pure_pursuit_control,
)
from embodied_swarm.navigation.astar import astar
from embodied_swarm.navigation.occupancy_grid import (
    OccupancyGrid,
)
from embodied_swarm.navigation.path_smoothing import (
    continuous_line_is_free,
    simplify_path,
    smooth_path,
)


def continuous_grid_to_world(
    grid: OccupancyGrid,
    point: tuple[float, float],
) -> tuple[float, float]:
    """
    Convert continuous grid coordinates:

        (row, col)

    into world coordinates:

        (x, y)

    Integer grid coordinates represent cell centers.
    """

    row, col = point

    x = (
        grid.origin_x
        + (col + 0.5) * grid.resolution
    )

    y = (
        grid.origin_y
        + (row + 0.5) * grid.resolution
    )

    return float(x), float(y)


def world_point_is_free(
    grid: OccupancyGrid,
    x: float,
    y: float,
) -> bool:
    """
    Runtime collision check for the simulated robot.
    """

    col = int(
        math.floor(
            (x - grid.origin_x)
            / grid.resolution
        )
    )

    row = int(
        math.floor(
            (y - grid.origin_y)
            / grid.resolution
        )
    )

    return grid.is_free(
        row,
        col,
    )


def normalize_angle(
    angle: float,
) -> float:
    """
    Normalize yaw into [-pi, pi].
    """

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def initial_heading(
    path: list[tuple[float, float]],
) -> float:
    """
    Initialize robot heading toward the
    second path waypoint.
    """

    if len(path) < 2:
        return 0.0

    x0, y0 = path[0]
    x1, y1 = path[1]

    return math.atan2(
        y1 - y0,
        x1 - x0,
    )


def main() -> None:
    # ==================================================
    # 1. Build occupancy grid
    # ==================================================

    data = np.zeros(
        (30, 30),
        dtype=int,
    )

    # Vertical obstacle wall.
    #
    # One opening exists at:
    #
    #     (row=20, col=15)
    #
    data[0:25, 15] = 1
    data[19:22, 15] = 0

    grid = OccupancyGrid(
        data=data,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )

    start = (2, 2)
    goal = (27, 27)

    # ==================================================
    # 2. Global planning with A*
    # ==================================================

    raw_path = astar(
        grid,
        start,
        goal,
    )

    if not raw_path:
        raise RuntimeError(
            "A* failed to find a path"
        )

    print(
        "Planning: PASS"
    )

    print(
        "Raw grid path nodes:",
        len(raw_path),
    )

    # ==================================================
    # 3. Path simplification
    #
    # Used here for comparison / diagnostics.
    # We do NOT feed this sparse 3-point path directly
    # into the smoother.
    # ==================================================

    simplified_path = simplify_path(
        grid,
        raw_path,
    )

    print(
        "Simplified path nodes:",
        len(simplified_path),
    )

    print(
        "Simplified path:",
        simplified_path,
    )

    # ==================================================
    # 4. Collision-safe dense smoothing
    # ==================================================

    raw_path_float = [
        (
            float(row),
            float(col),
        )
        for row, col in raw_path
    ]

    smoothed_grid_path = smooth_path(
        grid,
        raw_path_float,
        weight_data=0.5,
        weight_smooth=0.2,
        tolerance=1e-6,
        max_iterations=1000,
    )

    # Validate every smoothed segment.
    for point_a, point_b in zip(
        smoothed_grid_path,
        smoothed_grid_path[1:],
    ):
        if not continuous_line_is_free(
            grid,
            point_a,
            point_b,
        ):
            raise RuntimeError(
                "Smoothed path is not collision-free"
            )

    print(
        "Smoothing: PASS"
    )

    # ==================================================
    # 5. Grid path -> world path
    # ==================================================

    world_path = [
        continuous_grid_to_world(
            grid,
            point,
        )
        for point in smoothed_grid_path
    ]

    start_x, start_y = world_path[0]
    goal_x, goal_y = world_path[-1]

    yaw = initial_heading(
        world_path
    )

    robot_x = start_x
    robot_y = start_y

    # ==================================================
    # 6. Pure Pursuit parameters
    # ==================================================

    linear_speed = 0.3

    lookahead_distance = 0.6

    goal_tolerance = 0.25

    dt = 0.02

    max_steps = 8000

    # Store simulated trajectory for statistics.
    trajectory: list[
        tuple[float, float]
    ] = [
        (
            robot_x,
            robot_y,
        )
    ]

    reached_goal = False
    collision = False

    # ==================================================
    # 7. Closed-loop navigation
    # ==================================================

    for step in range(max_steps):
        robot_pose = (
            robot_x,
            robot_y,
            yaw,
        )

        v, omega = pure_pursuit_control(
            robot_pose=robot_pose,
            path=world_path,
            lookahead_distance=lookahead_distance,
            linear_speed=linear_speed,
            goal_tolerance=goal_tolerance,
        )

        # Pure Pursuit returns zero command
        # when goal tolerance is satisfied.
        if (
            abs(v) < 1e-12
            and abs(omega) < 1e-12
        ):
            reached_goal = True
            break

        # ==============================================
        # Unicycle / differential-drive kinematics
        #
        # x_dot   = v cos(yaw)
        # y_dot   = v sin(yaw)
        # yaw_dot = omega
        # ==============================================

        robot_x += (
            v
            * math.cos(yaw)
            * dt
        )

        robot_y += (
            v
            * math.sin(yaw)
            * dt
        )

        yaw += (
            omega
            * dt
        )

        yaw = normalize_angle(
            yaw
        )

        trajectory.append(
            (
                robot_x,
                robot_y,
            )
        )

        # Runtime collision check.
        if not world_point_is_free(
            grid,
            robot_x,
            robot_y,
        ):
            collision = True

            print()
            print(
                "Collision detected at:"
            )

            print(
                (
                    robot_x,
                    robot_y,
                )
            )

            break

    # ==================================================
    # 8. Final metrics
    # ==================================================

    final_goal_distance = math.hypot(
        goal_x - robot_x,
        goal_y - robot_y,
    )

    print()
    print(
        "=== Navigation Result ==="
    )

    print()

    print(
        "Simulation steps:",
        len(trajectory) - 1,
    )

    print(
        "Final robot pose:"
    )

    print(
        (
            robot_x,
            robot_y,
            yaw,
        )
    )

    print(
        "Goal:"
    )

    print(
        (
            goal_x,
            goal_y,
        )
    )

    print(
        "Final goal distance:",
        final_goal_distance,
    )

    print()

    # ==================================================
    # 9. Acceptance checks
    # ==================================================

    if collision:
        raise RuntimeError(
            "Navigation failed: collision"
        )

    print(
        "Collision check: PASS"
    )

    if not reached_goal:
        raise RuntimeError(
            "Navigation failed: goal not reached"
        )

    print(
        "Tracking: PASS"
    )

    if (
        final_goal_distance
        > goal_tolerance
    ):
        raise RuntimeError(
            "Navigation failed: "
            "goal tolerance exceeded"
        )

    print(
        "Goal reached: PASS"
    )

    print()
    print(
        "=== W03 Navigation Pipeline ==="
    )

    print()

    print(
        "OccupancyGrid       : PASS"
    )

    print(
        "A* planning         : PASS"
    )

    print(
        "Path smoothing      : PASS"
    )

    print(
        "Pure Pursuit        : PASS"
    )

    print(
        "Collision avoidance : PASS"
    )

    print(
        "Goal convergence    : PASS"
    )


if __name__ == "__main__":
    main()