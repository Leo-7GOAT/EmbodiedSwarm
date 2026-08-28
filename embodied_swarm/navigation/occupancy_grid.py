import numpy as np

class OccupancyGrid:

    def __init__(
            self,
            data: np.ndarray,
            resolution: float,
            origin_x:float = 0.0,
            origin_y:float = 0.0
    ) -> None:
        self.data = np.asarray(data)
        if self.data.ndim != 2:
            raise ValueError("OccupancyGrid expects 2d numpy array")
        self.resolution = float(resolution)
        if self.resolution <= 0:
            raise ValueError("resolution must be positive")
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.height = self.data.shape[0]
        self.width = self.data.shape[1]

    def world_to_grid(
            self,
            x:float,
            y:float,
    ) -> tuple[int, int]:

        col = int(
            np.floor((x - self.origin_x) / self.resolution))
        row = int(
            np.floor((y - self.origin_y) / self.resolution))

        return row, col

    def grid_to_world(
            self,
            row: int,
            col: int,
    ) -> tuple[float, float]:

        x = self.origin_x + (col +0.5) * self.resolution
        y = self.origin_y + (row +0.5) * self.resolution

        return x, y

    def is_inside(
            self,
            row: int,
            col: int,
    ) -> bool:

        if 0 <= row <self.height and 0 <= col <self.width:
            return True
        else:
            return False

    def is_free(
            self,
            row: int,
            col: int,
    ) -> bool:
        if self.is_inside(row,col)  and self.data[row,col] == 0:
            return True
        else:
            return False