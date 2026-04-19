import pygame
import sys
import math
import motionmodel as mm
import map as mp
import filter as kf

pygame.init()

ORANGE = (255, 77, 0)
BLACK = (0, 0, 0)
BLUE = (70, 130, 180)
RED = (200, 0, 0)
GREEN = (0, 200, 0) 

OMEGA = 5.0
VELOCITY = 100.0

# Car size for drawing
CAR_LENGTH = 24
CAR_WIDTH = 14

KF_SIGMA_SQ_X = 25.0
KF_SIGMA_SQ_Y = 25.0
KF_SIGMA_SQ_THETA = math.radians(10.0) ** 2
KF_SIGMA_SQ_RX = 2.0
KF_SIGMA_SQ_RY = 2.0
KF_SIGMA_SQ_RTHETA = math.radians(2.0) ** 2
KF_SIGMA_SQ_QX = 16.0
KF_SIGMA_SQ_QY = 16.0
KF_SIGMA_SQ_QTHETA = math.radians(8.0) ** 2


SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
WORLD_WIDTH, WORLD_HEIGHT = 1600,1600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("ARS Group 21")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

walls,landmarks = mp.create_map()
obstacles = walls + landmarks

kf_est_x = mm.x
kf_est_y = mm.y
kf_est_theta = mm.theta
kf_sigma_sq_x = KF_SIGMA_SQ_X
kf_sigma_sq_y = KF_SIGMA_SQ_Y
kf_sigma_sq_theta = KF_SIGMA_SQ_THETA

running = True
while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Moving
    keys = pygame.key.get_pressed()

    if keys[pygame.K_LEFT]:
        mm.omega = OMEGA
    elif keys[pygame.K_RIGHT]:
        mm.omega = -OMEGA
    else:
        mm.omega = 0.0

    if keys[pygame.K_UP]:
        mm.v = VELOCITY
    elif keys[pygame.K_DOWN]:
        mm.v = -VELOCITY
    else:
        mm.v = 0.0
    mm.dt = clock.tick(60) / 1000
    collisions = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)

    # Current sensor readings
    collision_readings = mm.get_sensor_readings(obstacles)
    landmark_readings = mm.get_sensor_readings(landmarks)
    # Draw background
    screen.fill((255, 255, 255))

    # Draw walls
    for obstacle in obstacles:
        start_x, start_y = mm.world_to_screen(obstacle[0][0], obstacle[0][1], SCREEN_WIDTH, SCREEN_HEIGHT)
        end_x, end_y = mm.world_to_screen(obstacle[1][0], obstacle[1][1], SCREEN_WIDTH, SCREEN_HEIGHT)
        pygame.draw.line(screen, BLACK, (start_x, start_y), (end_x, end_y), 3)
    # Robot center on screen
    screen_x, screen_y = mm.world_to_screen(mm.x, mm.y, SCREEN_WIDTH, SCREEN_HEIGHT)

    # Draw sensor rays + distance text
    '''
    for reading in collision_readings:
        hit_x, hit_y = reading["hit_point"]
        hit_screen_x, hit_screen_y = mm.world_to_screen(hit_x, hit_y, SCREEN_WIDTH, SCREEN_HEIGHT)

        pygame.draw.line(screen, BLUE, (screen_x, screen_y), (hit_screen_x, hit_screen_y), 1)
        pygame.draw.circle(screen, RED, (hit_screen_x, hit_screen_y), 3)

        sensor_angle = mm.theta + math.radians(reading["angle_deg"])
        dist = reading["distance"]

        label_world_x = mm.x + 0.6 * dist * math.cos(sensor_angle)
        label_world_y = mm.y + 0.6 * dist * math.sin(sensor_angle)

        offset = 10
        label_world_x += -math.sin(sensor_angle) * offset
        label_world_y +=  math.cos(sensor_angle) * offset

        label_screen_x, label_screen_y = mm.world_to_screen(label_world_x, label_world_y, SCREEN_WIDTH, SCREEN_HEIGHT)

        distance_text = font.render(f"{dist:.1f}", True, BLACK)
        screen.blit(distance_text, (label_screen_x, label_screen_y))
    '''
    
    landmark_measurements = []
    
    for reading in landmark_readings:
        hit_x, hit_y = reading["hit_point"]
        hit_screen_x, hit_screen_y = mm.world_to_screen(hit_x, hit_y, SCREEN_WIDTH, SCREEN_HEIGHT)
        if reading["distance"] < mm.SENSOR_MAX_RANGE:
            pygame.draw.line(screen, GREEN, (screen_x, screen_y), (hit_screen_x, hit_screen_y), 1)
            pygame.draw.circle(screen, GREEN, (hit_screen_x, hit_screen_y), 3)
        else:
            pygame.draw.line(screen, BLUE, (screen_x, screen_y), (hit_screen_x, hit_screen_y), 1)
            pygame.draw.circle(screen, RED, (hit_screen_x, hit_screen_y), 3)

        sensor_angle = mm.theta + math.radians(reading["angle_deg"])
        dist = reading["distance"]
        
        #Checking phi and r if sensor sees landmark (distance is lower than 100)
        #We need these for the correction step
        #It makes sense that the phi is integer, it is just the different beams every 10 degrees
        if dist < mm.SENSOR_MAX_RANGE:
            # print(dist) #r
            # print(reading["angle_deg"]) #phi
            # print(reading["hitpoint"])
            landmark_measurements.append(reading)
     

        label_world_x = mm.x + 0.6 * dist * math.cos(sensor_angle)
        label_world_y = mm.y + 0.6 * dist * math.sin(sensor_angle)

        offset = 10
        label_world_x += -math.sin(sensor_angle) * offset
        label_world_y +=  math.cos(sensor_angle) * offset

        label_screen_x, label_screen_y = mm.world_to_screen(label_world_x, label_world_y, SCREEN_WIDTH, SCREEN_HEIGHT)

        distance_text = font.render(f"{dist:.1f}", True, BLACK)
        screen.blit(distance_text, (label_screen_x, label_screen_y))
        
    kf_mu, kf_sigma = kf.kalman_filter(
        kf_est_x,
        kf_est_y,
        kf_est_theta,
        kf_sigma_sq_x,
        kf_sigma_sq_y,
        kf_sigma_sq_theta,
        KF_SIGMA_SQ_RX,
        KF_SIGMA_SQ_RY,
        KF_SIGMA_SQ_RTHETA,
        KF_SIGMA_SQ_QX,
        KF_SIGMA_SQ_QY,
        KF_SIGMA_SQ_QTHETA,
        mm.v,
        mm.omega,
        mm.dt,
        landmark_measurements,
    )
    
    #Update Kalman filter estimates
    kf_est_x = float(kf_mu[0, 0])
    kf_est_y = float(kf_mu[1, 0])
    kf_est_theta = float(kf_mu[2, 0])
    kf_sigma_sq_x = float(kf_sigma[0, 0])
    kf_sigma_sq_y = float(kf_sigma[1, 1])
    kf_sigma_sq_theta = float(kf_sigma[2, 2])

    # Draw rectangular car
    # Robot color: green normally, red on collision
    robot_color = GREEN # green
    if collisions:
        robot_color = RED  # red

    # Draw rectangular car
    car_corners_world = mm.get_robot_corners(CAR_LENGTH, CAR_WIDTH)
    car_corners_screen = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT) for px, py in car_corners_world]
    pygame.draw.polygon(screen, robot_color, car_corners_screen)
    pygame.draw.polygon(screen, BLACK, car_corners_screen, 2)

    # Front direction line
    front_x = mm.x + (CAR_LENGTH / 2) * math.cos(mm.theta)
    front_y = mm.y + (CAR_LENGTH / 2) * math.sin(mm.theta)
    fx_screen, fy_screen = mm.world_to_screen(front_x, front_y, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.draw.line(screen, BLACK, (screen_x, screen_y), (fx_screen, fy_screen), 2)

    #Localization from Kalman filter to screen
    kf_screen_x, kf_screen_y = mm.world_to_screen(kf_est_x, kf_est_y, SCREEN_WIDTH, SCREEN_HEIGHT)
    kf_corners_world = mm.get_robot_corners_at(kf_est_x, kf_est_y, kf_est_theta, CAR_LENGTH, CAR_WIDTH)
    kf_corners_screen = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT) for px, py in kf_corners_world]
    pygame.draw.polygon(screen, ORANGE, kf_corners_screen, 2)

    kf_front_x = kf_est_x + (CAR_LENGTH / 2) * math.cos(kf_est_theta)
    kf_front_y = kf_est_y + (CAR_LENGTH / 2) * math.sin(kf_est_theta)
    kf_fx_screen, kf_fy_screen = mm.world_to_screen(kf_front_x, kf_front_y, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.draw.line(screen, ORANGE, (kf_screen_x, kf_screen_y), (kf_fx_screen, kf_fy_screen), 2)

    #Kalman filter text in the top corner
    # kf_text = font.render(
    #     f"KF x={kf_est_x:.1f} y={kf_est_y:.1f} theta={math.degrees(kf_est_theta):.1f} deg visible={len(landmark_measurements)}",
    #     True,
    #     ORANGE,
    # )
    # screen.blit(kf_text, (10, 10))

    # # v and omega displayed in the top corner
    #v_text = font.render(f"v = {mm.v:.1f}", True, BLACK)
    #omega_text = font.render(f"omega = {mm.omega:.2f}", True, BLACK)
    #screen.blit(v_text, (10, 10))
    #screen.blit(omega_text, (10, 30))

    pygame.display.flip()

    

pygame.quit()
sys.exit()
