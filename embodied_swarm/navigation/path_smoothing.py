from embodied_swarm.navigation.occupancy_grid import OccupancyGrid
import numpy as np

def bresenham_cells(
    start: tuple[int, int],
    end: tuple[int, int],
) -> list[tuple[int, int]]:
    row0, col0 = start
    row1, col1 = end

    x0 = col0
    y0 = row0
    x1 = col1
    y1 = row1

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    if x1 > x0:
        sx = 1
    elif x1 == x0:
        sx = 0
    else:
        sx = -1

    if y1 > y0:
        sy = 1
    elif y1 == y0:
        sy = 0
    else:
        sy = -1

    error = dx - dy

    cells = []

    while True:
        cells.append((y0, x0))
        if (y0, x0) == end:
            break

        e2 = 2 * error

        if e2 > -dy:
            error -= dy
            x0 += sx

        if e2 < dx:
            error += dx
            y0 += sy

    return cells

def line_is_free(
        grid: OccupancyGrid,
        start: tuple[int, int],
        end: tuple[int, int],
) -> bool:
    cells = bresenham_cells(start, end)
    for cell in cells:
        if not grid.is_free(*cell):
            return False
    return True

def simplify_path(
    grid: OccupancyGrid,
    path: list[tuple[int, int]],
) -> list[tuple[int, int]]:

    if len(path) <= 1:
        return path.copy()

    simplified = [
        path[0]
    ]
    anchor_index = 0

    while anchor_index < len(path) - 1:

        candidate_index = len(path) - 1

        while candidate_index > anchor_index + 1:
            if line_is_free(grid, path[anchor_index], path[candidate_index]):
                break

            candidate_index -= 1

        simplified.append(path[candidate_index])
        anchor_index = candidate_index
    return simplified

def smooth_path(
    grid: OccupancyGrid,
    path: list[tuple[float, float]],
    weight_data: float = 0.5,
    weight_smooth: float = 0.1,
    tolerance: float = 1e-6,
    max_iterations: int = 1000,
) -> list[tuple[float, float]]:

    if len(path) <= 1:
        return path.copy()

    original = np.asarray(
        path,
        dtype=float,
    )

    if (
        original.ndim != 2
        or original.shape[1] != 2
    ):
        raise ValueError(
            "path must contain (row, col) points"
        )

    smoothed = original.copy()

    for _ in range(max_iterations):
        total_change = 0.0

        # 起点和终点固定，只调整中间点
        for i in range(
            1,
            len(path) - 1,
        ):
            old_point = (
                smoothed[i].copy()
            )

            candidate = (
                smoothed[i].copy()
            )

            # 1. Data term
            # 把路径往原始路径拉，
            # 防止为了平滑偏离得太远。
            candidate += (
                weight_data
                * (
                    original[i]
                    - candidate
                )
            )

            # 2. Smoothness term
            # 把当前点往前后邻居中间拉。
            candidate += (
                weight_smooth
                * (
                    smoothed[i - 1]
                    + smoothed[i + 1]
                    - 2.0 * candidate
                )
            )

            previous_point = (
                float(smoothed[i - 1][0]),
                float(smoothed[i - 1][1]),
            )

            candidate_point = (
                float(candidate[0]),
                float(candidate[1]),
            )

            next_point = (
                float(smoothed[i + 1][0]),
                float(smoothed[i + 1][1]),
            )

            safe_before = (
                continuous_line_is_free(
                    grid,
                    previous_point,
                    candidate_point,
                )
            )

            safe_after = (
                continuous_line_is_free(
                    grid,
                    candidate_point,
                    next_point,
                )
            )

            # 两边都安全才接受 candidate
            if safe_before and safe_after:
                smoothed[i] = candidate
            else:
                # 撞墙，回滚
                smoothed[i] = old_point

            total_change += float(
                np.linalg.norm(
                    smoothed[i]
                    - old_point
                )
            )

        if total_change < tolerance:
            break

    return [
        (
            float(point[0]),
            float(point[1]),
        )
        for point in smoothed
    ]

def continuous_line_is_free(
    grid: OccupancyGrid,
    start: tuple[float, float],
    end: tuple[float, float],
    sample_step: float = 0.25,
) -> bool:
    start_array = np.asarray(
        start,
        dtype=float,
    )
    end_array = np.asarray(
        end,
        dtype=float,
    )

    delta = end_array - start_array

    distance = float(
        np.linalg.norm(delta)
    )

    if distance == 0.0:
        row = int(round(start_array[0]))
        col = int(round(start_array[1]))

        return grid.is_free(
            row,
            col,
        )

    num_samples = max(
        1,
        int(np.ceil(distance / sample_step)),
    )

    for t in np.linspace(
        0.0,
        1.0,
        num_samples + 1,
    ):
        point = (
            start_array
            + t * delta
        )

        row = int(round(point[0]))
        col = int(round(point[1]))

        if not grid.is_free(
            row,
            col,
        ):
            return False

    return True
