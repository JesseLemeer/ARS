import pygame
import sys
import math
import motionmodel as mm
import map as mp
import filter as kf
from ellipse import draw_covariance_ellipse

pygame.init()

ORANGE = (255, 77, 0)
BLACK  = (0, 0, 0)
BLUE   = (70, 130, 180)
RED    = (200, 0, 0)
GREEN  = (0, 200, 0)

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
KF_SIGMA_SQ_QX     = 16.0
KF_SIGMA_SQ_QY     = 16.0
KF_SIGMA_SQ_QTHETA = math.radians(8.0) ** 2

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

kf_est_x     = mm.x
kf_est_y     = mm.y
kf_est_theta = mm.theta
kf_sigma_sq_x     = KF_SIGMA_SQ_X
kf_sigma_sq_y     = KF_SIGMA_SQ_Y
kf_sigma_sq_theta = KF_SIGMA_SQ_THETA
kf_sigma_xy       = 0.0

true_trail = []
kf_trail   = []

running = True
while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if   keys[pygame.K_LEFT]:  mm.omega =  OMEGA
    elif keys[pygame.K_RIGHT]: mm.omega = -OMEGA
    else:                       mm.omega =  0.0

    if   keys[pygame.K_UP]:   mm.v =  VELOCITY
    elif keys[pygame.K_DOWN]: mm.v = -VELOCITY
    else:                      mm.v =  0.0

    mm.dt      = clock.tick(60) / 1000
    collisions = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)

    collision_readings = mm.get_sensor_readings(obstacles)
    landmark_readings  = mm.get_sensor_readings(landmarks)

    
    true_trail.append((mm.x, mm.y))
    if len(true_trail) > TRAIL_LEN:
        true_trail.pop(0)

    screen.fill((255, 255, 255))

    # Draw Walls
    for obstacle in obstacles:
        s = mm.world_to_screen(obstacle[0][0], obstacle[0][1], SCREEN_WIDTH, SCREEN_HEIGHT)
        e = mm.world_to_screen(obstacle[1][0], obstacle[1][1], SCREEN_WIDTH, SCREEN_HEIGHT)
        pygame.draw.line(screen, BLACK, s, e, 3)

    screen_x, screen_y = mm.world_to_screen(mm.x, mm.y, SCREEN_WIDTH, SCREEN_HEIGHT)

    # Draw trajectory (solid blue line)
    if len(true_trail) >= 2:
        trail_pts = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                     for px, py in true_trail]
        pygame.draw.lines(screen, (30, 80, 200), False, trail_pts, 2)

    # Draw KF (estimated) trajectory trail (dashed orange)
    if len(kf_trail) >= 2:
        kf_pts = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                  for px, py in kf_trail]
        for i in range(0, len(kf_pts) - 1, 4):
            pygame.draw.line(screen, ORANGE, kf_pts[i], kf_pts[min(i+2, len(kf_pts)-1)], 2)

    # Landmark sensor rays
    landmark_measurements = []
    for reading in landmark_readings:
        hit_x, hit_y  = reading["hit_point"]
        hit_sx, hit_sy = mm.world_to_screen(hit_x, hit_y, SCREEN_WIDTH, SCREEN_HEIGHT)

        if reading["distance"] < mm.SENSOR_MAX_RANGE:
            pygame.draw.line(screen, GREEN, (screen_x, screen_y), (hit_sx, hit_sy), 1)
            pygame.draw.circle(screen, GREEN, (hit_sx, hit_sy), 3)
            landmark_measurements.append(reading)
        else:
            pygame.draw.line(screen, BLUE, (screen_x, screen_y), (hit_sx, hit_sy), 1)
            pygame.draw.circle(screen, RED, (hit_sx, hit_sy), 3)

        sensor_angle = mm.theta + math.radians(reading["angle_deg"])
        dist = reading["distance"]
        lx = mm.x + 0.6 * dist * math.cos(sensor_angle)
        ly = mm.y + 0.6 * dist * math.sin(sensor_angle)
        lx += -math.sin(sensor_angle) * 10
        ly +=  math.cos(sensor_angle) * 10
        lsx, lsy = mm.world_to_screen(lx, ly, SCREEN_WIDTH, SCREEN_HEIGHT)
        screen.blit(font.render(f"{dist:.1f}", True, BLACK), (lsx, lsy))

    # Kalman Filter step 
    kf_mu, kf_sigma_mat = kf.kalman_filter(
        kf_est_x, kf_est_y, kf_est_theta,
        kf_sigma_sq_x, kf_sigma_sq_y, kf_sigma_sq_theta,
        KF_SIGMA_SQ_RX, KF_SIGMA_SQ_RY, KF_SIGMA_SQ_RTHETA,
        KF_SIGMA_SQ_QX, KF_SIGMA_SQ_QY, KF_SIGMA_SQ_QTHETA,
        mm.v, mm.omega, mm.dt,
        landmark_measurements,
    )

    kf_est_x     = float(kf_mu[0, 0])
    kf_est_y     = float(kf_mu[1, 0])
    kf_est_theta = float(kf_mu[2, 0])
    kf_sigma_sq_x     = float(kf_sigma_mat[0, 0])
    kf_sigma_sq_y     = float(kf_sigma_mat[1, 1])
    kf_sigma_sq_theta = float(kf_sigma_mat[2, 2])
    kf_sigma_xy       = float(kf_sigma_mat[0, 1])

    # Record KF position for trail
    kf_trail.append((kf_est_x, kf_est_y))
    if len(kf_trail) > TRAIL_LEN:
        kf_trail.pop(0)

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

    # Draw KF estimate car (orange outline)
    kf_screen_x, kf_screen_y = mm.world_to_screen(kf_est_x, kf_est_y, SCREEN_WIDTH, SCREEN_HEIGHT)
    kf_corners_world  = mm.get_robot_corners_at(kf_est_x, kf_est_y, kf_est_theta, CAR_LENGTH, CAR_WIDTH)
    kf_corners_screen = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                         for px, py in kf_corners_world]
    pygame.draw.polygon(screen, ORANGE, kf_corners_screen, 2)

    kf_fx = kf_est_x + (CAR_LENGTH / 2) * math.cos(kf_est_theta)
    kf_fy = kf_est_y + (CAR_LENGTH / 2) * math.sin(kf_est_theta)
    kfx_s, kfy_s = mm.world_to_screen(kf_fx, kf_fy, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.draw.line(screen, ORANGE, (kf_screen_x, kf_screen_y), (kfx_s, kfy_s), 2)

    # Draw COVARIANCE ELLIPSE around KF estimate
    draw_covariance_ellipse(
        surface        = screen,
        kf_est_x       = kf_est_x,
        kf_est_y       = kf_est_y,
        sigma_xx       = kf_sigma_sq_x,
        sigma_yy       = kf_sigma_sq_y,
        sigma_xy       = kf_sigma_xy,
        world_to_screen= mm.world_to_screen,
        screen_width   = SCREEN_WIDTH,
        screen_height  = SCREEN_HEIGHT,
        color          = ORANGE,
        n_std          = 2,
        n_points       = 64,
        thickness      = 2,
    )

    visible_count = len(landmark_measurements)
    hud_lines = [
        f"True:  x={mm.x:.1f}  y={mm.y:.1f}  θ={math.degrees(mm.theta):.1f}°",
        f"KF:    x={kf_est_x:.1f}  y={kf_est_y:.1f}  θ={math.degrees(kf_est_theta):.1f}°",
        f"Error: {math.hypot(mm.x-kf_est_x, mm.y-kf_est_y):.1f}  |  Landmarks visible: {visible_count}",
        f"σx²={kf_sigma_sq_x:.1f}  σy²={kf_sigma_sq_y:.1f}  (ellipse shows 2σ)",
    ]
    for row, line in enumerate(hud_lines):
        surf = font.render(line, True, (20, 20, 20))
        screen.blit(surf, (8, 8 + row * 18))

    pygame.display.flip()

pygame.quit()
sys.exit()