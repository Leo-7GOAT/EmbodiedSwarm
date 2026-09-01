import heapq
import itertools
import math

from embodied_swarm.navigation.occupancy_grid import OccupancyGrid


def propagate_motion(
    pose: tuple[float, float, float],
    linear_velocity: float,
    angular_velocity: float,
    dt: float,
) -> tuple[float, float, float]:

    x_new = pose[0] + linear_velocity * math.cos(pose[2]) * dt
    y_new = pose[1] + linear_velocity * math.sin(pose[2]) * dt
    yaw_new = normalize_angle(
        pose[2] + angular_velocity * dt
    )

    return x_new, y_new, yaw_new

def normalize_angle(angle: float) -> float:
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )

def yaw_to_bin(
        yaw:float,
        num_bins: int = 72,
) -> int:
    if num_bins <= 0:
        raise ValueError("num_bins must be positive")
    yaw_positive = yaw % (2.0 * math.pi)

    bin_size = 2.0 * math.pi / num_bins
    return int(yaw_positive / bin_size)

def generate_successors(
    grid: OccupancyGrid,
    pose: tuple[float, float, float],
    linear_velocity: float = 1.0,
    max_angular_velocity: float = 0.6,
    duration: float = 1.0,
    integration_dt: float = 0.1,
) -> list[
    tuple[
        tuple[float, float, float],
        float,
        list[tuple[float, float, float]],
    ]
]:

    angular_velocities = [
        -max_angular_velocity,
        0.0,
        max_angular_velocity,
    ]

    successors = []

    for omega in angular_velocities:
        trajectory = simulate_motion_primitive(
            pose,
            linear_velocity,
            omega,
            duration,
            integration_dt,
        )

        if not primitive_is_collision_free(
            grid,
            trajectory,
        ):
            continue

        next_pose = trajectory[-1]

        successor = (
            next_pose,
            omega,
            trajectory,
        )

        successors.append(successor)

    return successors

def simulate_motion_primitive(
        pose: tuple[float, float, float],
        linear_velocity: float,
        angular_velocity: float,
        duration: float = 1.0,
        integration_dt:float = 0.1,
) -> list[tuple[float, float, float]]:
    if duration <= 0:
        raise ValueError("duration must be positive")
    if integration_dt <= 0:
        raise ValueError("integration_dt must be positive")

    num_steps = max(
        1,
        math.ceil(
            duration / integration_dt
        ),
    )

    actual_dt = (
            duration / num_steps
    )

    current_pose = pose
    trajectory = [pose]

    for _ in range(num_steps):
        current_pose = propagate_motion(
            current_pose,
            linear_velocity,
            angular_velocity,
            actual_dt,
        )

        trajectory.append(
            current_pose
        )

    return trajectory

def primitive_is_collision_free(
        grid:OccupancyGrid,
        trajectory:list[tuple[float, float, float]],
) -> bool:
    for pose in trajectory:
        row, col = grid.world_to_grid(pose[0], pose[1])
        if not grid.is_free(
                row,
                col,
        ):
            return False

    return True

def pose_to_key(
        grid: OccupancyGrid,
        pose: tuple[float, float, float],
        num_yaw_bins: int = 72,
) -> tuple[int, int, int]:
    row, col = grid.world_to_grid(
        pose[0],
        pose[1],
    )
    yaw_bin = yaw_to_bin(
        pose[2],
        num_yaw_bins,
    )
    return row, col, yaw_bin

def hybrid_heuristic(
    pose: tuple[float, float, float],
    goal: tuple[float, float, float],
) -> float:
    h = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
    return h

def primitive_cost(
    linear_velocity: float,
    angular_velocity: float,
    duration: float,
    turn_penalty: float = 0.2,
) -> float:
    distance_cost = abs(linear_velocity) * duration
    turn_cost = turn_penalty * abs(angular_velocity) * duration
    cost = distance_cost + turn_cost

    return cost

def hybrid_astar(
        grid: OccupancyGrid,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
        linear_velocity: float = 1.0,
        max_angular_velocity: float = 0.6,
        primitive_duration: float = 1.0,
        integration_dt: float = 0.1,
        num_yaw_bins: int = 72,
        turn_penalty: float = 0.2,
        stats: dict[str, int] | None = None,
) -> list[tuple[float, float, float]]:

    start_row, start_col = grid.world_to_grid(
        start[0],
        start[1],
    )

    goal_row, goal_col = grid.world_to_grid(
        goal[0],
        goal[1],
    )

    if not grid.is_free(start_row, start_col):
        if stats is not None:
            stats["expanded"] = 0
        return []

    if not grid.is_free(goal_row, goal_col):
        if stats is not None:
            stats["expanded"] = 0
        return []

    start_key = pose_to_key(grid, start, num_yaw_bins)

    g_score = {start_key: 0.0}
    came_from = {}

    start_h = hybrid_heuristic(start, goal)
    start_f = start_h

    counter = itertools.count()

    open_heap = [
        (start_f, next(counter), 0.0, start_key, start,)
    ]

    goal_key = None

    expanded = 0

    while open_heap:
        current_f, _, current_g, current_key, current_pose = heapq.heappop(
            open_heap
        )
        if current_g > g_score[current_key]:
            continue

        expanded += 1

        if goal_reached(
                current_pose,
                goal,
        ):
            goal_key = current_key
            break

        for (
                next_pose,
                omega,
                trajectory,
        ) in generate_successors(
            grid=grid,
            pose=current_pose,
            linear_velocity=linear_velocity,
            max_angular_velocity=max_angular_velocity,
            duration=primitive_duration,
            integration_dt=integration_dt,
        ):
            next_key = pose_to_key(
                grid,
                next_pose,
                num_yaw_bins,
            )
            step_cost = primitive_cost(
                linear_velocity,
                omega,
                primitive_duration,
                turn_penalty,
            )

            new_g = current_g + step_cost
            old_g = g_score.get(
                next_key,
                float("inf"),
            )
            if new_g < old_g:
                g_score[next_key] = new_g

                came_from[next_key] = (
                    current_key,
                    trajectory,
                )
                h = hybrid_heuristic(
                    next_pose,
                    goal,
                )

                new_f = new_g + h
                heapq.heappush(
                    open_heap,
                    (
                        new_f,
                        next(counter),
                        new_g,
                        next_key,
                        next_pose,
                    ),
                )

    if stats is not None:
        stats["expanded"] = expanded

    if goal_key is None:
        return []

    return reconstruct_hybrid_path(
        came_from,
        start_key,
        goal_key,
        start,
    )

def reconstruct_hybrid_path(
    came_from: dict[
        tuple[int, int, int],
        tuple[
            tuple[int, int, int],
            list[tuple[float, float, float]],
        ],
    ],
    start_key: tuple[int, int, int],
    goal_key: tuple[int, int, int],
    start_pose: tuple[float, float, float],
) -> list[tuple[float, float, float]]:

    if start_key == goal_key:
        return [start_pose]

    segments = []
    current_key = goal_key

    while current_key != start_key:
        parent_key, trajectory = came_from[current_key]
        segments.append(trajectory)
        current_key = parent_key

    segments.reverse()

    path = [start_pose]

    for segment in segments:
        path = path + segment[1:]

    return path

def goal_reached(
        pose: tuple[float, float, float],
        goal: tuple[float, float, float],
        position_tolerance: float = 0.75,
        yaw_tolerance: float = math.radians(15.0),
) -> bool:
    position_error = math.hypot(
        goal[0] - pose[0],
        goal[1] - pose[1],
    )
    yaw_error = abs(
        normalize_angle(
            goal[2] - pose[2]
        )
    )

    if position_error <= position_tolerance and yaw_error <= yaw_tolerance:
        return True
    return False
