import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from embodied_swarm.control.pure_pursuit import (
    pure_pursuit_control,
)
from embodied_swarm.navigation.hybrid_astar import (
    hybrid_astar,
)
from embodied_swarm.navigation.occupancy_grid import (
    OccupancyGrid,
)
from embodied_swarm.navigation.replanning import (
    path_is_collision_free,
    replan_if_needed,
)


Pose = tuple[float, float, float]


def normalize_angle(
    angle: float,
) -> float:
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def simulate_step(
    pose: Pose,
    linear_velocity: float,
    angular_velocity: float,
    dt: float,
) -> Pose:
    """
    Propagate differential-drive/unicycle pose
    for one control step.
    """

    x, y, yaw = pose

    x_new = (
        x
        + linear_velocity
        * math.cos(yaw)
        * dt
    )

    y_new = (
        y
        + linear_velocity
        * math.sin(yaw)
        * dt
    )

    yaw_new = normalize_angle(
        yaw
        + angular_velocity * dt
    )

    return (
        x_new,
        y_new,
        yaw_new,
    )


def nearest_path_index(
    pose: Pose,
    path: list[Pose],
) -> int:
    """
    Find the trajectory point closest to
    the current robot pose.
    """

    if not path:
        raise ValueError(
            "path must not be empty"
        )

    best_index = 0
    best_distance = float("inf")

    for i, point in enumerate(path):
        distance = math.hypot(
            point[0] - pose[0],
            point[1] - pose[1],
        )

        if distance < best_distance:
            best_distance = distance
            best_index = i

    return best_index


def mark_obstacle_with_margin(
    grid: OccupancyGrid,
    row: int,
    col: int,
    radius_cells: int = 1,
) -> None:
    """
    Insert an obstacle together with a safety
    margin.

    radius_cells = 1 means:

        XXX
        XXX
        XXX

    where the center cell is the real obstacle
    and surrounding cells provide tracking /
    footprint safety margin.
    """

    if radius_cells < 0:
        raise ValueError(
            "radius_cells must be non-negative"
        )

    rows, cols = grid.data.shape

    row_min = max(
        0,
        row - radius_cells,
    )

    row_max = min(
        rows,
        row + radius_cells + 1,
    )

    col_min = max(
        0,
        col - radius_cells,
    )

    col_max = min(
        cols,
        col + radius_cells + 1,
    )

    grid.data[
        row_min:row_max,
        col_min:col_max,
    ] = 1


def main() -> None:
    # ==================================================
    # 1. Build map
    # ==================================================

    data = np.zeros(
        (30, 30),
        dtype=int,
    )

    # Vertical wall.
    data[:, 15] = 1

    # Opening in the wall.
    data[11:19, 15] = 0

    # Keep a copy for GIF visualization.
    initial_map_data = data.copy()

    grid = OccupancyGrid(
        data=data,
        resolution=1.0,
    )

    start: Pose = (
        3.5,
        5.5,
        0.0,
    )

    goal: Pose = (
        26.5,
        5.5,
        0.0,
    )

    # ==================================================
    # 2. Initial Hybrid A* planning
    # ==================================================

    initial_stats: dict[str, int] = {}

    initial_path = hybrid_astar(
        grid=grid,
        start=start,
        goal=goal,
        linear_velocity=1.0,
        max_angular_velocity=0.6,
        primitive_duration=1.0,
        integration_dt=0.1,
        num_yaw_bins=72,
        turn_penalty=0.2,
        stats=initial_stats,
    )

    if not initial_path:
        raise RuntimeError(
            "Initial Hybrid A* planning failed"
        )

    initial_safe = path_is_collision_free(
        grid,
        initial_path,
    )

    if not initial_safe:
        raise RuntimeError(
            "Initial Hybrid A* path is not safe"
        )

    print(
        "initial path points:",
        len(initial_path),
    )

    print(
        "initial expanded:",
        initial_stats.get(
            "expanded",
            0,
        ),
    )

    print(
        "initial path safe:",
        initial_safe,
    )

    # ==================================================
    # 3. Tracking configuration
    # ==================================================

    # Lower speed gives the controller more time
    # to follow curved Hybrid A* trajectories.
    tracking_speed = 0.35

    dt = 0.05

    # Smaller lookahead reduces corner cutting.
    lookahead_distance = 0.5

    max_angular_velocity = 1.5

    pose = start

    active_path = initial_path

    executed_path: list[Pose] = [
        pose
    ]

    replanned_path: list[Pose] | None = None

    injected = False
    replanned = False

    injected_cell: tuple[int, int] | None = (
        None
    )

    injection_step: int | None = None

    replan_stats: dict = {}

    max_steps = 3000

    # ==================================================
    # 4. Closed-loop execution
    # ==================================================

    for step in range(max_steps):
        current_index = (
            nearest_path_index(
                pose,
                active_path,
            )
        )

        remaining_path = active_path[
            current_index:
        ]

        # ==============================================
        # 4.1 Inject a new obstacle
        # ==============================================

        if (
            not injected
            and step >= 200
            and len(remaining_path) > 80
        ):
            # Choose a future point sufficiently
            # far ahead of the robot.
            blocked_index = min(
                current_index + 60,
                len(active_path) - 1,
            )

            blocked_pose = active_path[
                blocked_index
            ]

            blocked_row, blocked_col = (
                grid.world_to_grid(
                    blocked_pose[0],
                    blocked_pose[1],
                )
            )

            injected_cell = (
                blocked_row,
                blocked_col,
            )

            # ------------------------------------------
            # IMPORTANT:
            #
            # Do not insert only one occupied cell.
            #
            # Hybrid A* plans for a point robot while
            # Pure Pursuit has tracking error and cuts
            # corners.
            #
            # Inflate the obstacle to create safety
            # clearance.
            # ------------------------------------------

            mark_obstacle_with_margin(
                grid=grid,
                row=blocked_row,
                col=blocked_col,
                radius_cells=1,
            )

            injected = True
            injection_step = step

            print(
                "injected obstacle cell:",
                injected_cell,
            )

            old_safe = (
                path_is_collision_free(
                    grid,
                    remaining_path,
                )
            )

            print(
                "old remaining safe:",
                old_safe,
            )

            if old_safe:
                raise RuntimeError(
                    "Injected obstacle did not "
                    "invalidate remaining path"
                )

            # ==========================================
            # 4.2 Dynamic replanning
            # ==========================================

            new_path, did_replan = (
                replan_if_needed(
                    grid=grid,
                    current_pose=pose,
                    goal=goal,
                    remaining_path=remaining_path,
                    linear_velocity=1.0,
                    max_angular_velocity=0.6,
                    primitive_duration=1.0,
                    integration_dt=0.1,
                    num_yaw_bins=72,
                    turn_penalty=0.2,
                    stats=replan_stats,
                )
            )

            print(
                "replanned:",
                did_replan,
            )

            print(
                "replan success:",
                bool(new_path),
            )

            print(
                "replan expanded:",
                replan_stats.get(
                    "expanded",
                    0,
                ),
            )

            if not did_replan:
                raise RuntimeError(
                    "Replanning was not triggered"
                )

            if not new_path:
                raise RuntimeError(
                    "Dynamic replanning failed"
                )

            new_path_safe = (
                path_is_collision_free(
                    grid,
                    new_path,
                )
            )

            print(
                "replanned path safe:",
                new_path_safe,
            )

            if not new_path_safe:
                raise RuntimeError(
                    "Replanned path is not safe"
                )

            active_path = new_path
            replanned_path = new_path
            replanned = True

        # ==============================================
        # 4.3 Pure Pursuit tracking
        # ==============================================

        path_xy = [
            (
                point[0],
                point[1],
            )
            for point in active_path
        ]

        (
            linear_velocity,
            angular_velocity,
        ) = pure_pursuit_control(
            robot_pose=pose,
            path=path_xy,
            lookahead_distance=(
                lookahead_distance
            ),
            linear_speed=tracking_speed,
            goal_tolerance=0.25,
        )

        angular_velocity = float(
            np.clip(
                angular_velocity,
                -max_angular_velocity,
                max_angular_velocity,
            )
        )

        pose = simulate_step(
            pose=pose,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            dt=dt,
        )

        executed_path.append(
            pose
        )

        # ==============================================
        # 4.4 Runtime collision validation
        # ==============================================

        row, col = grid.world_to_grid(
            pose[0],
            pose[1],
        )

        if not grid.is_free(
            row,
            col,
        ):
            raise RuntimeError(
                "Robot collided at "
                f"{(row, col)}"
            )

        # ==============================================
        # 4.5 Goal check
        # ==============================================

        final_reference = (
            active_path[-1]
        )

        distance_to_path_end = (
            math.hypot(
                final_reference[0]
                - pose[0],
                final_reference[1]
                - pose[1],
            )
        )

        if (
            distance_to_path_end
            <= 0.25
        ):
            print(
                "goal region reached at step:",
                step,
            )
            break

    else:
        raise RuntimeError(
            "Navigation did not converge"
        )

    # ==================================================
    # 5. Final validation
    # ==================================================

    goal_position_error = math.hypot(
        goal[0] - pose[0],
        goal[1] - pose[1],
    )

    print()
    print(
        "final pose:",
        pose,
    )

    print(
        "goal position error:",
        goal_position_error,
    )

    print(
        "executed path points:",
        len(executed_path),
    )

    print(
        "dynamic obstacle injected:",
        injected,
    )

    print(
        "dynamic replanning triggered:",
        replanned,
    )

    if not injected:
        raise RuntimeError(
            "Dynamic obstacle was never injected"
        )

    if not replanned:
        raise RuntimeError(
            "Dynamic replanning was never triggered"
        )

    # Hybrid A* accepts position tolerance 0.75 m.
    # Pure Pursuit then tracks the Hybrid A* endpoint.
    # 1.0 m leaves a small integration margin.
    if goal_position_error > 1.0:
        raise RuntimeError(
            "Robot did not reach goal region: "
            f"error={goal_position_error}"
        )

    print()
    print(
        "W04 integrated navigation: PASS"
    )

    # ==================================================
    # 6. Generate GIF
    # ==================================================

    output_dir = Path(
        "artifacts"
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    gif_path = (
        output_dir
        / "w04_navigation_replanning.gif"
    )

    fig, ax = plt.subplots(
        figsize=(7, 7)
    )

    # Occupancy map.
    ax.imshow(
        initial_map_data,
        origin="lower",
        interpolation="nearest",
        extent=(
            0,
            initial_map_data.shape[1],
            0,
            initial_map_data.shape[0],
        ),
    )

    # ----------------------------------------------
    # Initial Hybrid A* trajectory
    # ----------------------------------------------

    initial_x = [
        point[0]
        for point in initial_path
    ]

    initial_y = [
        point[1]
        for point in initial_path
    ]

    ax.plot(
        initial_x,
        initial_y,
        "--",
        linewidth=1.5,
        label="Initial Hybrid A*",
    )

    # ----------------------------------------------
    # Replanned Hybrid A* trajectory
    # ----------------------------------------------

    if replanned_path is not None:
        replanned_x = [
            point[0]
            for point in replanned_path
        ]

        replanned_y = [
            point[1]
            for point in replanned_path
        ]

        ax.plot(
            replanned_x,
            replanned_y,
            "--",
            linewidth=1.5,
            label="Replanned Hybrid A*",
        )

    # ----------------------------------------------
    # Start / Goal
    # ----------------------------------------------

    ax.scatter(
        [start[0]],
        [start[1]],
        marker="o",
        s=60,
        label="Start",
    )

    ax.scatter(
        [goal[0]],
        [goal[1]],
        marker="*",
        s=140,
        label="Goal",
    )

    # ----------------------------------------------
    # Animated artists
    # ----------------------------------------------

    trajectory_line, = ax.plot(
        [],
        [],
        linewidth=2.0,
        label="Executed trajectory",
    )

    robot_point, = ax.plot(
        [],
        [],
        marker="o",
        markersize=7,
    )

    obstacle_point, = ax.plot(
        [],
        [],
        marker="s",
        markersize=10,
        label="Dynamic obstacle",
    )

    ax.set_xlim(
        0,
        data.shape[1],
    )

    ax.set_ylim(
        0,
        data.shape[0],
    )

    ax.set_aspect(
        "equal"
    )

    ax.set_xlabel(
        "x [m]"
    )

    ax.set_ylabel(
        "y [m]"
    )

    ax.set_title(
        "W04 Hybrid A* + Dynamic Replanning"
    )

    ax.legend(
        loc="upper right"
    )

    # Reduce GIF size.
    frame_stride = 8

    frame_indices = list(
        range(
            0,
            len(executed_path),
            frame_stride,
        )
    )

    if (
        not frame_indices
        or frame_indices[-1]
        != len(executed_path) - 1
    ):
        frame_indices.append(
            len(executed_path) - 1
        )

    def update(
        frame_number: int,
    ):
        index = frame_indices[
            frame_number
        ]

        history = executed_path[
            :index + 1
        ]

        xs = [
            point[0]
            for point in history
        ]

        ys = [
            point[1]
            for point in history
        ]

        trajectory_line.set_data(
            xs,
            ys,
        )

        robot_point.set_data(
            [
                executed_path[index][0]
            ],
            [
                executed_path[index][1]
            ],
        )

        # ------------------------------------------
        # Show dynamic obstacle only after the
        # injection moment.
        # ------------------------------------------

        if (
            injected_cell is not None
            and injection_step is not None
            and index >= injection_step
        ):
            obstacle_row, obstacle_col = (
                injected_cell
            )

            obstacle_x = (
                obstacle_col + 0.5
            )

            obstacle_y = (
                obstacle_row + 0.5
            )

            obstacle_point.set_data(
                [obstacle_x],
                [obstacle_y],
            )

        else:
            obstacle_point.set_data(
                [],
                [],
            )

        return (
            trajectory_line,
            robot_point,
            obstacle_point,
        )

    animation = FuncAnimation(
        fig,
        update,
        frames=len(frame_indices),
        interval=50,
        blit=True,
    )

    try:
        animation.save(
            gif_path,
            writer=PillowWriter(
                fps=20
            ),
        )

        print(
            "GIF saved:",
            gif_path,
        )

    except Exception as exc:
        print(
            "GIF save skipped:",
            exc,
        )

        print(
            "If Pillow is missing, run:"
        )

        print(
            "pip install pillow"
        )

    plt.close(
        fig
    )


if __name__ == "__main__":
    main()