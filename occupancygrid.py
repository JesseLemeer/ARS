import math
import numpy as np
import pygame

class OccupancyGrid:
    def __init__(
        self,
        x_min:    float = -600.0, # world-space x-axis, y-axis bounds of the grid (world units)
        x_max:    float =  450.0,
        y_min:    float = -400.0,
        y_max:    float =  400.0,
        cell_size: float = 10.0,
        p_occ:    float = 0.70, # probability assigned to the hit cell per beam (default 0.70)
        p_free:   float = 0.30, # probability assigned to free cells along beam (default 0.30)
        p_prior:  float = 0.50, # prior probability (no evidence) (default 0.50)
        l_max:    float =  5.0,
        l_min:    float = -5.0,
    ):
        self.x_min     = x_min
        self.x_max     = x_max
        self.y_min     = y_min
        self.y_max     = y_max
        self.cell_size = cell_size
        self.l_max     = l_max
        self.l_min     = l_min

        # Grid dimensions (number of rows and columns)
        self.cols = int(math.ceil((x_max - x_min) / cell_size))
        self.rows = int(math.ceil((y_max - y_min) / cell_size))

        self.log_odds = np.zeros((self.rows, self.cols), dtype=np.float32)

        # Pre-compute log-odds constants from the given probabilities
        self.l_0    = math.log(p_prior / (1.0 - p_prior))  # = 0.0
        self.l_occ  = math.log(p_occ   / (1.0 - p_occ))    # ≈ +0.847
        self.l_free = math.log(p_free  / (1.0 - p_free))   # ≈ -0.847

    def world_to_grid(self, wx: float, wy: float):
        col = int((wx - self.x_min) / self.cell_size)
        row = int((wy - self.y_min) / self.cell_size)
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return row, col
        return None

    def grid_to_world_center(self, row: int, col: int):
        wx = self.x_min + (col + 0.5) * self.cell_size
        wy = self.y_min + (row + 0.5) * self.cell_size
        return wx, wy

    def get_probability(self, row: int, col: int) -> float:
        l = float(self.log_odds[row, col])
        return 1.0 - 1.0 / (1.0 + math.exp(l))

    def _cells_along_ray(self, robot_x: float, robot_y: float,
                          hit_x: float, hit_y: float,
                          ray_hits_obstacle: bool):
        ray_dx  = hit_x - robot_x
        ray_dy  = hit_y - robot_y
        ray_len = math.hypot(ray_dx, ray_dy)

        if ray_len < 1e-6:
            return [], None

        # Normalised ray direction
        ux = ray_dx / ray_len
        uy = ray_dy / ray_len

        # Sample every half cell-width along the ray
        step_size = self.cell_size * 0.5
        n_steps   = int(ray_len / step_size)

        visited = set()     # avoid updating the same cell twice per ray
        ordered = []        # cells in order from robot outward

        for i in range(n_steps + 1):
            d  = i * step_size
            wx = robot_x + ux * d
            wy = robot_y + uy * d
            rc = self.world_to_grid(wx, wy)
            if rc is not None and rc not in visited:
                visited.add(rc)
                ordered.append(rc)

        # If the ray hit something, the last cell is the hit cell and all previous cells are free.
        # If the ray reached max range, all cells are free (no occupied update).
        if ray_hits_obstacle and ordered:
            return ordered[:-1], ordered[-1] # free cells + occupied cell
        else:
            return ordered, None # all free, no hit

    # Update the occupancy grid
    def update(self, robot_x: float, robot_y: float, robot_theta: float,
               sensor_readings: list, max_range: float):
        for reading in sensor_readings:
            #Rebuild the ray endpoint from the robot's estimated pose and the local sensor reading
            #This avoids giving direct access to the true world hit_point.
            distance = min(float(reading["distance"]), max_range)
            angle = robot_theta + math.radians(reading["angle_deg"])
            hit_x = robot_x + distance * math.cos(angle)
            hit_y = robot_y + distance * math.sin(angle)

            #If the distance is less than the maximum range, then the ray hit an obstacle
            #Otherwise the cells along the ray are free
            ray_hits_obstacle = float(reading["distance"]) < max_range - 0.5

            # Get the free cells and the optional hit cell for this ray
            free_cells, hit_cell = self._cells_along_ray(
                robot_x, robot_y, hit_x, hit_y, ray_hits_obstacle
            )

            # Applying inverse sensor model:
            for row, col in free_cells:
                self.log_odds[row, col] += self.l_free - self.l_0

            # Cell at the hit point -> evidence of OCCUPIED space
            if hit_cell is not None:
                row, col = hit_cell
                self.log_odds[row, col] += self.l_occ - self.l_0

        np.clip(self.log_odds, self.l_min, self.l_max, out=self.log_odds)

    # Drawing
    def draw(self, surface, world_to_screen_fn,
             screen_width: int, screen_height: int,
             robot_x: float, robot_y: float):
        cell_px = max(1, int(self.cell_size))  # cell size in screen pixels

        margin = self.cell_size  # one cell of padding around the edge

        wx_vis_min = robot_x - screen_width  / 2.0 - margin
        wx_vis_max = robot_x + screen_width  / 2.0 + margin
        wy_vis_min = robot_y - screen_height / 2.0 - margin
        wy_vis_max = robot_y + screen_height / 2.0 + margin

        # Convert visible world bounds to grid indices
        col_start = max(0,         int((wx_vis_min - self.x_min) / self.cell_size))
        col_end   = min(self.cols, int((wx_vis_max - self.x_min) / self.cell_size) + 1)
        row_start = max(0,         int((wy_vis_min - self.y_min) / self.cell_size))
        row_end   = min(self.rows, int((wy_vis_max - self.y_min) / self.cell_size) + 1)

        for row in range(row_start, row_end):
            for col in range(col_start, col_end):
                l = float(self.log_odds[row, col])

                # # Skip cells that have no meaningful evidence yet
                # if abs(l) < 0.05:
                #     continue

                # Convert log-odds to probability, then to greyscale colour. High occupancy probability -> darker colour.
                prob  = 1.0 - 1.0 / (1.0 + math.exp(l))
                grey  = int(255.0 * (1.0 - prob))
                color = (grey, grey, grey)

                # World centre of this cell -> screen position
                wx, wy = self.grid_to_world_center(row, col)
                sx, sy = world_to_screen_fn(wx, wy, screen_width, screen_height)

                # Draw the cell as a filled rectangle (top-left aligned)
                rect_x = sx - cell_px // 2
                rect_y = sy - cell_px // 2
                pygame.draw.rect(surface, color, (rect_x, rect_y, cell_px, cell_px))
