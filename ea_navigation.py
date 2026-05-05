import math
import os
from dataclasses import dataclass

import numpy as np

import filter as kf
import motionmodel as mm
from occupancygrid import OccupancyGrid


DEFAULT_GRID = dict(
    x_min=-600.0,
    x_max=450.0,
    y_min=-400.0,
    y_max=400.0,
    cell_size=10.0,
    p_occ=0.70,
    p_free=0.30,
    p_prior=0.50,
    l_max=5.0,
    l_min=-5.0,
)

DEFAULT_SLAM = dict(
    sigma_sq_x=25.0,
    sigma_sq_y=25.0,
    sigma_sq_theta=math.radians(10.0) ** 2,
    sigma_sq_Rx=2.0,
    sigma_sq_Ry=2.0,
    sigma_sq_Rtheta=math.radians(2.0) ** 2,
    sigma_sq_R=4.0,
    sigma_sq_phi=math.radians(3.0) ** 2,
)

#Thresholds for determining if a cell is occupied or explored based on log-odds values
OCCUPIED_LOG_ODDS_THRESHOLD = 0.1
EXPLORED_LOG_ODDS_THRESHOLD = 0.1

#Keep track of state the robot thinks it is in and the map it has learned so far
class NavigationState:
    def __init__(
        self,
        grid,
        slam_mu,
        slam_sigma,
        landmark_index,
        est_x,
        est_y,
        est_theta,
    ):
        self.grid = grid
        self.slam_mu = slam_mu
        self.slam_sigma = slam_sigma
        self.landmark_index = landmark_index
        self.est_x = est_x
        self.est_y = est_y
        self.est_theta = est_theta
        self.occupied_cells = 0
        self.free_cells = 0
        self.explored_cells = 0


#Initialize empty map and SLAM at starting position
def make_navigation_state(start_x=0.0, start_y=0.0, start_theta=0.0):
    grid = OccupancyGrid(**DEFAULT_GRID)
    slam_mu = np.array([start_x, start_y, start_theta], dtype=float)
    slam_sigma = np.diag(
        [
            DEFAULT_SLAM["sigma_sq_x"],
            DEFAULT_SLAM["sigma_sq_y"],
            DEFAULT_SLAM["sigma_sq_theta"],
        ]
    )

    return NavigationState(
        grid=grid,
        slam_mu=slam_mu,
        slam_sigma=slam_sigma,
        landmark_index={},
        est_x=float(start_x),
        est_y=float(start_y),
        est_theta=float(start_theta),
    )


def slam_noise_matrices():
    sigma_R = np.diag(
        [
            DEFAULT_SLAM["sigma_sq_Rx"],
            DEFAULT_SLAM["sigma_sq_Ry"],
            DEFAULT_SLAM["sigma_sq_Rtheta"],
        ]
    )
    sigma_Q = np.diag(
        [
            DEFAULT_SLAM["sigma_sq_R"],
            DEFAULT_SLAM["sigma_sq_phi"],
        ]
    )
    return sigma_R, sigma_Q


#Determine explored grid units and whether they are occupied or free 
def refresh_grid_stats(state):
    occupied = state.grid.log_odds > EXPLORED_LOG_ODDS_THRESHOLD
    free = state.grid.log_odds < -EXPLORED_LOG_ODDS_THRESHOLD
    
    state.occupied_cells = int(np.sum(occupied))
    state.free_cells = int(np.sum(free))
    
    state.explored_cells = state.occupied_cells + state.free_cells

#Update map and pose estimate using sensor measurements, motion, and EKF SLAM
def update_navigation(state, wall_sensor_readings, landmark_measurements, v, omega, dt):
    sigma_R, sigma_Q = slam_noise_matrices()
    
    visible_landmarks = [
        reading for reading in landmark_measurements
        if reading["distance"] <= mm.LANDMARK_SENSOR_RANGE
    ]

    state.slam_mu, state.slam_sigma, state.landmark_index = kf.ekf_slam(
        state.slam_mu,
        state.slam_sigma,
        sigma_R,
        sigma_Q,
        v,
        omega,
        dt,
        visible_landmarks,
        state.landmark_index,
    )
    
    state.est_x = float(state.slam_mu[0])
    state.est_y = float(state.slam_mu[1])
    state.est_theta = float(state.slam_mu[2])

    state.grid.update(
        robot_x=state.est_x,
        robot_y=state.est_y,
        robot_theta=state.est_theta,
        sensor_readings=wall_sensor_readings,
        max_range=mm.SENSOR_MAX_RANGE,
    )
    
    refresh_grid_stats(state)
    return state


#Do one initial SLAM and map update before the controller starts moving
def bootstrap_navigation(state, wall_sensor_readings, landmark_measurements):
    return update_navigation(
        state,
        wall_sensor_readings,
        landmark_measurements,
        0.0,
        0.0,
        0.0,
    )


#Convert real sensor distances into [0,1] activations
def raw_sensor_activations(wall_sensor_readings):
    activations = []

    for reading in wall_sensor_readings:
        distance = float(reading["distance"])
        activation = 1.0 - distance / mm.SENSOR_MAX_RANGE
        activations.append(activation)

    activations = np.array(activations, dtype=float)
    return np.clip(activations, 0.0, 1.0)


#Get distance to occupied cell along a ray in the grid, up to max_range
def grid_distance_to_occupied(state, angle_rad, max_range):
    step_size = max(1.0, state.grid.cell_size * 0.5)
    n_steps = max(1, int(math.ceil(max_range / step_size)))

    for i in range(1, n_steps + 1):
        distance = min(i * step_size, max_range)
        wx = state.est_x + distance * math.cos(angle_rad)
        wy = state.est_y + distance * math.sin(angle_rad)
        grid_rc = state.grid.world_to_grid(wx, wy)
        if grid_rc is None:
            continue

        row, col = grid_rc
        
        if state.grid.log_odds[row, col] > OCCUPIED_LOG_ODDS_THRESHOLD:
            return distance

    return max_range


#Convert the learned occupancy grid into controller inputs
def mapped_sensor_activations(state, sensor_angles_deg=None, max_range=None):
    if sensor_angles_deg is None:
        sensor_angles_deg = mm.SENSOR_ANGLES_DEG
    if max_range is None:
        max_range = mm.SENSOR_MAX_RANGE

    distances = []
    for angle_deg in sensor_angles_deg:
        angle_rad = state.est_theta + math.radians(angle_deg)
        distance = grid_distance_to_occupied(
            state,
            angle_rad,
            max_range,
        )
        distances.append(distance)

    activations = []
    for distance in distances:
        activation = 1.0 - distance / max_range
        activations.append(activation)

    activations = np.array(activations, dtype=float)
    return np.clip(activations, 0.0, 1.0)
