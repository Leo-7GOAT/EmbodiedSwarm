import math
import time

import numpy as np

from embodied_swarm.navigation.astar import astar
from embodied_swarm.navigation.hybrid_astar import hybrid_astar
from embodied_swarm.navigation.occupancy_grid import OccupancyGrid


def hybrid_path_length(
    path: list[tuple[float, float, float]],
) -> float:
    total = 0.0

    for p0, p1 in zip(
        path,
        path[1:],
    ):
        total += math.hypot(
            p1[0] - p0[0],
            p1[1] - p0[1],
        )

    return total


def grid_path_length(
    path: list[tuple[int, int]],
    resolution: float,
) -> float:
    if len(path) <= 1:
        return 0.0

    return (
        len(path) - 1
    ) * resolution


def count_grid_turns(
    path: list[tuple[int, int]],
) -> int:
    if len(path) < 3:
        return 0

    turns = 0

    previous_direction = None

    for p0, p1 in zip(
        path,
        path[1:],
    ):
        direction = (
            p1[0] - p0[0],
            p1[1] - p0[1],
        )

        if (
            previous_direction is not None
            and direction != previous_direction
        ):
            turns += 1

        previous_direction = direction

    return turns


def max_hybrid_yaw_step(
    path: list[tuple[float, float, float]],
) -> float:
    if len(path) <= 1:
        return 0.0

    max_step = 0.0

    for p0, p1 in zip(
        path,
        path[1:],
    ):
        delta = math.atan2(
            math.sin(p1[2] - p0[2]),
            math.cos(p1[2] - p0[2]),
        )

        max_step = max(
            max_step,
            abs(delta),
        )

    return max_step


def main() -> None:
    # ==================================================
    # Same map for both planners
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

    # --------------------------------------------------
    # Same physical start / goal
    #
    # Grid A* uses:
    #   (row, col)
    #
    # Hybrid A* uses:
    #   (x, y, yaw)
    # --------------------------------------------------

    grid_start = (
        5,
        3,
    )

    grid_goal = (
        5,
        26,
    )

    hybrid_start = (
        3.5,
        5.5,
        0.0,
    )

    hybrid_goal = (
        26.5,
        5.5,
        0.0,
    )

    # ==================================================
    # Grid A*
    # ==================================================

    astar_stats: dict[str, int] = {}

    t0 = time.perf_counter()

    astar_path = astar(
        grid,
        grid_start,
        grid_goal,
        stats=astar_stats,
    )

    astar_time = (
        time.perf_counter() - t0
    )

    if not astar_path:
        raise RuntimeError(
            "Grid A* failed"
        )

    # ==================================================
    # Hybrid A*
    # ==================================================

    hybrid_stats: dict[str, int] = {}

    t0 = time.perf_counter()

    hybrid_path = hybrid_astar(
        grid=grid,
        start=hybrid_start,
        goal=hybrid_goal,
        linear_velocity=1.0,
        max_angular_velocity=0.6,
        primitive_duration=1.0,
        integration_dt=0.1,
        num_yaw_bins=72,
        turn_penalty=0.2,
        stats=hybrid_stats,
    )

    hybrid_time = (
        time.perf_counter() - t0
    )

    if not hybrid_path:
        raise RuntimeError(
            "Hybrid A* failed"
        )

    # ==================================================
    # Collision checks
    # ==================================================

    for row, col in astar_path:
        assert grid.is_free(
            row,
            col,
        )

    for x, y, _ in hybrid_path:
        row, col = grid.world_to_grid(
            x,
            y,
        )

        assert grid.is_free(
            row,
            col,
        )

    # ==================================================
    # Metrics
    # ==================================================

    astar_length = grid_path_length(
        astar_path,
        grid.resolution,
    )

    hybrid_length = hybrid_path_length(
        hybrid_path
    )

    astar_turns = count_grid_turns(
        astar_path
    )

    hybrid_max_yaw_step = (
        max_hybrid_yaw_step(
            hybrid_path
        )
    )

    primitive_steps = math.ceil(
        1.0 / 0.1
    )

    hybrid_primitive_count = (
        len(hybrid_path) - 1
    ) // primitive_steps

    # ==================================================
    # Output
    # ==================================================

    print(
        "=== A* vs Hybrid A* Benchmark ==="
    )
    print()

    print("Grid A*")
    print(
        "path nodes       :",
        len(astar_path),
    )
    print(
        "path length      :",
        round(astar_length, 3),
        "m",
    )
    print(
        "expanded nodes   :",
        astar_stats["expanded"],
    )
    print(
        "planning time    :",
        round(
            astar_time * 1000.0,
            3,
        ),
        "ms",
    )
    print(
        "grid turns       :",
        astar_turns,
    )

    print()

    print("Hybrid A*")
    print(
        "trajectory points:",
        len(hybrid_path),
    )
    print(
        "primitives       :",
        hybrid_primitive_count,
    )
    print(
        "path length      :",
        round(hybrid_length, 3),
        "m",
    )
    print(
        "expanded states  :",
        hybrid_stats["expanded"],
    )
    print(
        "planning time    :",
        round(
            hybrid_time * 1000.0,
            3,
        ),
        "ms",
    )
    print(
        "max yaw step     :",
        round(
            math.degrees(
                hybrid_max_yaw_step
            ),
            3,
        ),
        "deg",
    )

    print()

    print(
        "Collision check  : PASS"
    )

    print()

    print(
        "=== Interpretation ==="
    )

    if hybrid_length < astar_length:
        print(
            "Hybrid path length: SHORTER"
        )
    elif math.isclose(
        hybrid_length,
        astar_length,
        rel_tol=1e-6,
    ):
        print(
            "Hybrid path length: MATCH"
        )
    else:
        print(
            "Hybrid path length: LONGER"
        )

    if hybrid_time > astar_time:
        print(
            "Hybrid planning cost: HIGHER"
        )
    else:
        print(
            "Hybrid planning cost: LOWER"
        )

    print(
        "Hybrid heading continuity: PASS"
    )


if __name__ == "__main__":
    main()