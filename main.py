import pygame
import sys
import math
import numpy as np
import motionmodel as mm
import map as mp
import filter as kf
from ellipse import draw_covariance_ellipse
from occupancygrid import OccupancyGrid

pygame.init()

ORANGE = (255, 77, 0)
BLACK  = (0, 0, 0)
BLUE   = (70, 130, 180)
RED    = (200, 0, 0)
GREEN  = (0, 200, 0)
WHITE  = (255, 255, 255)

OMEGA    = 5.0
VELOCITY = 100.0
CAR_LENGTH = 24
CAR_WIDTH  = 14

KF_SIGMA_SQ_X      = 25.0
KF_SIGMA_SQ_Y      = 25.0
KF_SIGMA_SQ_THETA  = math.radians(10.0) ** 2
KF_SIGMA_SQ_RX     = 2.0
KF_SIGMA_SQ_RY     = 2.0
KF_SIGMA_SQ_RTHETA = math.radians(2.0) ** 2
 
# KF-only noise (3x3 observation, triangulated pose)
KF_SIGMA_SQ_QX     = 16.0
KF_SIGMA_SQ_QY     = 16.0
KF_SIGMA_SQ_QTHETA = math.radians(8.0) ** 2
 
# EKF-only noise (2x2 observation: range + bearing)
EKF_SIGMA_SQ_R   = 4.0
EKF_SIGMA_SQ_PHI = math.radians(3.0) ** 2

SCREEN_WIDTH,  SCREEN_HEIGHT  = 800, 600
WORLD_WIDTH,   WORLD_HEIGHT   = 1600, 1600
TRAIL_LEN = 300

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("ARS Group 21")
clock = pygame.time.Clock()
font  = pygame.font.SysFont(None, 20)

# walls, landmarks = mp.create_map()
walls, landmarks, landmark_groups = mp.create_map()
obstacles = walls + landmarks

kf_est_x      = mm.x
kf_est_y      = mm.y
kf_est_theta  = mm.theta
kf_sigma_sq_x     = KF_SIGMA_SQ_X
kf_sigma_sq_y     = KF_SIGMA_SQ_Y
kf_sigma_sq_theta = KF_SIGMA_SQ_THETA
kf_sigma_xy       = 0.0
                                                                                                                                                                                                                                                                                                                                                            
ekf_est_x     = mm.x
ekf_est_y     = mm.y
ekf_est_theta = mm.theta
ekf_sigma_mat = np.diag([KF_SIGMA_SQ_X, KF_SIGMA_SQ_Y, KF_SIGMA_SQ_THETA])
ekf_sigma_R   = np.diag([KF_SIGMA_SQ_RX, KF_SIGMA_SQ_RY, KF_SIGMA_SQ_RTHETA])
ekf_sigma_Q   = np.diag([EKF_SIGMA_SQ_R, EKF_SIGMA_SQ_PHI])

true_trail = []
est_trail   = []

grid = OccupancyGrid(
    x_min=-600.0, x_max=450.0,
    y_min=-400.0, y_max=400.0,
    cell_size=10.0,
    p_occ=0.70,
    p_free=0.30,
    p_prior=0.50,
    l_max=5.0,
    l_min=-5.0,
)
show_map = True    # toggle with M key
ekf = True #switches between ekf and kf
running = True
while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            # M → toggle map overlay
            if event.key == pygame.K_m:
                show_map = not show_map

    # Robot Movement
    keys = pygame.key.get_pressed()
    if   keys[pygame.K_LEFT]:  mm.omega =  OMEGA
    elif keys[pygame.K_RIGHT]: mm.omega = -OMEGA
    else:                       mm.omega =  0.0

    if   keys[pygame.K_UP]:   mm.v =  VELOCITY
    elif keys[pygame.K_DOWN]: mm.v = -VELOCITY
    else:                      mm.v =  0.0

    mm.dt      = clock.tick(60) / 1000
    collisions = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)

    # Sensor Readings
    landmark_measurements = mm.get_landmark_measurements(landmark_groups) # For Kalman filter correction step
    wall_sensor_readings = mm.get_sensor_readings(walls) # For Occupancy grid mapping

    true_trail.append((mm.x, mm.y))
    if len(true_trail) > TRAIL_LEN:
        true_trail.pop(0)

    if not ekf:
        kf_mu, kf_sigma_mat = kf.kalman_filter(
            kf_est_x, kf_est_y, kf_est_theta,
            kf_sigma_sq_x, kf_sigma_sq_y, kf_sigma_sq_theta,
            KF_SIGMA_SQ_RX, KF_SIGMA_SQ_RY, KF_SIGMA_SQ_RTHETA,
            KF_SIGMA_SQ_QX, KF_SIGMA_SQ_QY, KF_SIGMA_SQ_QTHETA,
            mm.v, mm.omega, mm.dt,
            landmark_measurements,
        )
        kf_est_x = float(kf_mu[0, 0])
        kf_est_y = float(kf_mu[1, 0])
        kf_est_theta = float(kf_mu[2, 0])
        kf_sigma_sq_x = float(kf_sigma_mat[0, 0])
        kf_sigma_sq_y = float(kf_sigma_mat[1, 1])
        kf_sigma_sq_theta = float(kf_sigma_mat[2, 2])
        kf_sigma_xy = float(kf_sigma_mat[0, 1])
    
        est_x = kf_est_x
        est_y = kf_est_y
        est_theta = kf_est_theta
        est_sig_xx = kf_sigma_sq_x
        est_sig_yy = kf_sigma_sq_y
        est_sig_xy = kf_sigma_xy
    else:
        ekf_mu, ekf_sigma_mat = kf.ekf_filter(
            ekf_est_x, ekf_est_y, ekf_est_theta,
            ekf_sigma_mat, ekf_sigma_R, ekf_sigma_Q,
            mm.v, mm.omega, mm.dt,
            landmark_measurements,
        )
        ekf_est_x = float(ekf_mu[0, 0])
        ekf_est_y = float(ekf_mu[1, 0])
        ekf_est_theta = float(ekf_mu[2, 0])
        
        est_x = ekf_est_x
        est_y = ekf_est_y
        est_theta = ekf_est_theta
        est_sig_xx = float(ekf_sigma_mat[0, 0])
        est_sig_yy = float(ekf_sigma_mat[1, 1])
        est_sig_xy = float(ekf_sigma_mat[0, 1])
    
    est_trail.append((est_x, est_y))
    if len(est_trail) > TRAIL_LEN:
        est_trail.pop(0)
        
    # Occupancy grid update
    grid.update(
        robot_x=est_x,
        robot_y=est_y,
        sensor_readings=wall_sensor_readings,
        max_range=mm.SENSOR_MAX_RANGE,
    )

    # Drawing
    screen.fill(WHITE)

    # Occupancy grid
    if show_map:
        grid.draw(
            surface=screen,
            world_to_screen_fn=mm.world_to_screen,
            screen_width=SCREEN_WIDTH,
            screen_height=SCREEN_HEIGHT,
            robot_x=mm.x,   # true robot position for correct viewport culling
            robot_y=mm.y,
        )

    # Draw Walls and landmarks
    for obstacle in obstacles:
        s = mm.world_to_screen(obstacle[0][0], obstacle[0][1], SCREEN_WIDTH, SCREEN_HEIGHT)
        e = mm.world_to_screen(obstacle[1][0], obstacle[1][1], SCREEN_WIDTH, SCREEN_HEIGHT)
        pygame.draw.line(screen, BLACK, s, e, 3)

    screen_x, screen_y = mm.world_to_screen(mm.x, mm.y, SCREEN_WIDTH, SCREEN_HEIGHT)
    
    #Draw landmark sensor range (light green circle)
    pygame.draw.circle(screen, (190, 235, 190), (screen_x, screen_y), int(mm.LANDMARK_SENSOR_RANGE), 1)

    # Draw trajectory (solid blue line)
    if len(true_trail) >= 2:
        trail_pts = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                     for px, py in true_trail]
        pygame.draw.lines(screen, (30, 80, 200), False, trail_pts, 2)

    # Draw KF (estimated) trajectory trail (dashed orange)
    if len(est_trail) >= 2:
        est_pts = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                  for px, py in est_trail]
        for i in range(0, len(est_pts) - 1, 4):
            pygame.draw.line(screen, ORANGE, est_pts[i], est_pts[min(i+2, len(est_pts)-1)], 2)

    # Wall sensor - shows what the mapping sensor sees
    for reading in wall_sensor_readings:
        hit_x, hit_y = reading["hit_point"]
        hit_sx, hit_sy = mm.world_to_screen(hit_x, hit_y, SCREEN_WIDTH, SCREEN_HEIGHT)
        pygame.draw.line(screen, (210, 210, 230),
                         (screen_x, screen_y), (hit_sx, hit_sy), 1)

    #Omnidirectional landmark detections
    for reading in landmark_measurements:
        landmark_x, landmark_y = reading["landmark_center"]
        landmark_sx, landmark_sy = mm.world_to_screen(landmark_x, landmark_y, SCREEN_WIDTH, SCREEN_HEIGHT)

        #Draw green line from robot to landmark and green circle at landmark position if detected
        pygame.draw.line(screen, GREEN, (screen_x, screen_y), (landmark_sx, landmark_sy), 1)
        pygame.draw.circle(screen, GREEN, (landmark_sx, landmark_sy), 6, 3)

        #Position label slightly to the right and above the landmark
        label_x = landmark_x + 10
        label_y = landmark_y + 10
        label_sx, label_sy = mm.world_to_screen(label_x, label_y, SCREEN_WIDTH, SCREEN_HEIGHT)
        label = (
            f"L{reading['landmark_id']} "
            f"({landmark_x:.0f},{landmark_y:.0f}) "
            f"r={reading['distance']:.1f} "
            f"φ={math.degrees(reading['bearing_rad']):.1f}°"
        )
        screen.blit(font.render(label, True, BLACK), (label_sx, label_sy))

    # Draw actual (green/red) robot
    robot_color = GREEN if not collisions else RED
    car_corners_world  = mm.get_robot_corners(CAR_LENGTH, CAR_WIDTH)
    car_corners_screen = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                          for px, py in car_corners_world]
    pygame.draw.polygon(screen, robot_color, car_corners_screen)
    pygame.draw.polygon(screen, BLACK, car_corners_screen, 2)

    front_x = mm.x + (CAR_LENGTH / 2) * math.cos(mm.theta)
    front_y = mm.y + (CAR_LENGTH / 2) * math.sin(mm.theta)
    fx_s, fy_s = mm.world_to_screen(front_x, front_y, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.draw.line(screen, BLACK, (screen_x, screen_y), (fx_s, fy_s), 2)

    
    est_sx, est_sy = mm.world_to_screen(est_x, est_y, SCREEN_WIDTH, SCREEN_HEIGHT)
    est_corners_screen = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                          for px, py in mm.get_robot_corners_at(
                              est_x, est_y, est_theta, CAR_LENGTH, CAR_WIDTH)]
    pygame.draw.polygon(screen, ORANGE, est_corners_screen, 2)
    efx = est_x + (CAR_LENGTH / 2) * math.cos(est_theta)
    efy = est_y + (CAR_LENGTH / 2) * math.sin(est_theta)
    efx_s, efy_s = mm.world_to_screen(efx, efy, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.draw.line(screen, ORANGE, (est_sx, est_sy), (efx_s, efy_s), 2)

    # Draw COVARIANCE ELLIPSE around KF estimate
    draw_covariance_ellipse(
        surface = screen,
        kf_est_x = est_x,
        kf_est_y = est_y,
        sigma_xx = est_sig_xx,
        sigma_yy = est_sig_yy,
        sigma_xy = est_sig_xy,
        world_to_screen= mm.world_to_screen,
        screen_width = SCREEN_WIDTH,
        screen_height = SCREEN_HEIGHT,
        color = ORANGE,
        n_std = 2,
        n_points = 64,
        thickness = 2,
    )

    # Compute live map statistics for display
    total_cells    = grid.rows * grid.cols
    occupied_cells = int(np.sum(grid.log_odds > 0.1))
    free_cells     = int(np.sum(grid.log_odds < -0.1))
    explored_pct   = 100.0 * (occupied_cells + free_cells) / total_cells

    visible_count = len(landmark_measurements)
    hud_lines = [
        f"True:  x={mm.x:.1f}  y={mm.y:.1f}  θ={math.degrees(mm.theta) % 360:.1f}°",
        f"KF:    x={est_x:.1f}  y={est_y:.1f}  θ={math.degrees(est_theta):.1f}°",
        f"Error: {math.hypot(mm.x-est_x, mm.y-est_y):.1f}  |  Landmarks visible: {visible_count}",
        # f"σx²={est_sig_xx:.1f}  σy²={est_sig_yy:.1f}  (ellipse shows 2σ)",
    ]
    for row, line in enumerate(hud_lines):
        surf = font.render(line, True, (20, 20, 20))
        screen.blit(surf, (8, 8 + row * 18))

    pygame.display.flip()

pygame.quit()
sys.exit()