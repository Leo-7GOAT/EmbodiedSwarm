import math

def world_to_body(
    robot_pose: tuple[float, float, float],
    target: tuple[float, float],
) -> tuple[float, float]:

    x, y, yaw = robot_pose
    target_x, target_y = target

    dx = target_x - x
    dy = target_y - y

    x_r = (
        math.cos(yaw) * dx
        + math.sin(yaw) * dy
    )

    y_r = (
        -math.sin(yaw) * dx
        + math.cos(yaw) * dy
    )

    return x_r, y_r

def pure_pursuit_curvature(
    target_body: tuple[float, float],
) -> float:
    x_r, y_r = target_body

    ld_sq = x_r ** 2 + y_r ** 2

    eps = 1e-12

    if ld_sq < eps:
        return 0.0

    return 2.0 * y_r / ld_sq

def find_lookahead_point(
        robot_xy: tuple[float, float],
        path: list[tuple[float, float]],
        lookahead_distance: float,
) -> tuple[float, float]:

    if not path:
        raise ValueError("path must not be empty")

    nearest_index = 0
    nearest_distance = float("inf")

    for i, point in enumerate(path):
        distance = math.hypot(
            point[0] - robot_xy[0],
            point[1] - robot_xy[1],
        )

        if distance < nearest_distance:
            nearest_distance = distance
            nearest_index = i

    for i in range(nearest_index, len(path)):
        point = path[i]
        distance = math.hypot(
            point[0] - robot_xy[0],
            point[1] - robot_xy[1],
        )

        if distance >= lookahead_distance:
            return path[i]
    return path[-1]

def pure_pursuit_control(
    robot_pose: tuple[float, float, float],
    path: list[tuple[float, float]],
    lookahead_distance: float,
    linear_speed: float,
    goal_tolerance: float = 0.2,
) -> tuple[float, float]:

    if not path:
        raise ValueError(
            "path must not be empty"
        )

    if lookahead_distance <= 0.0:
        raise ValueError(
            "lookahead_distance must be positive"
        )

    robot_x, robot_y, _ = robot_pose

    goal_x, goal_y = path[-1]

    goal_distance = math.hypot(
        goal_x - robot_x,
        goal_y - robot_y,
    )

    # 已经到达终点
    if goal_distance <= goal_tolerance:
        return 0.0, 0.0

    target = find_lookahead_point(
        (robot_x, robot_y),
        path,
        lookahead_distance,
    )

    target_body = world_to_body(
        robot_pose,
        target,
    )

    curvature = pure_pursuit_curvature(
        target_body
    )

    angular_speed = (
        linear_speed * curvature
    )

    return (
        linear_speed,
        angular_speed,
    )