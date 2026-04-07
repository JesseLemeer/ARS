import pygame
import sys
import math
import motionmodel as mm
import map as mp

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

# Collision still uses circular approximation
COLLISION_RADIUS = 10

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("ARS Group 21")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

walls = mp.create_map()

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

    # Current sensor readings
    sensor_readings = mm.get_sensor_readings(walls)

    # Draw background
    screen.fill((255, 255, 255))

    # Draw walls
    for wall in walls:
        start_x, start_y = mm.world_to_screen(wall[0][0], wall[0][1], WIDTH, HEIGHT)
        end_x, end_y = mm.world_to_screen(wall[1][0], wall[1][1], WIDTH, HEIGHT)
        pygame.draw.line(screen, BLACK, (start_x, start_y), (end_x, end_y), 3)

    # Robot center on screen
    screen_x, screen_y = mm.world_to_screen(mm.x, mm.y, WIDTH, HEIGHT)

    # Draw sensor rays + distance text
    for reading in sensor_readings:
        hit_x, hit_y = reading["hit_point"]
        hit_screen_x, hit_screen_y = mm.world_to_screen(hit_x, hit_y, WIDTH, HEIGHT)

        pygame.draw.line(screen, BLUE, (screen_x, screen_y), (hit_screen_x, hit_screen_y), 1)
        pygame.draw.circle(screen, RED, (hit_screen_x, hit_screen_y), 3)

        sensor_angle = mm.theta + math.radians(reading["angle_deg"])
        dist = reading["distance"]

        label_world_x = mm.x + 0.6 * dist * math.cos(sensor_angle)
        label_world_y = mm.y + 0.6 * dist * math.sin(sensor_angle)

        offset = 10
        label_world_x += -math.sin(sensor_angle) * offset
        label_world_y +=  math.cos(sensor_angle) * offset

        label_screen_x, label_screen_y = mm.world_to_screen(label_world_x, label_world_y, WIDTH, HEIGHT)

        distance_text = font.render(f"{dist:.1f}", True, BLACK)
        screen.blit(distance_text, (label_screen_x, label_screen_y))

    collisions = mm.update(walls, COLLISION_RADIUS)
    # Draw rectangular car
    # Robot color: green normally, red on collision
    robot_color = GREEN # green
    if collisions:
        robot_color = RED  # red

    # Draw rectangular car
    car_corners_world = mm.get_robot_corners(CAR_LENGTH, CAR_WIDTH)
    car_corners_screen = [mm.world_to_screen(px, py, WIDTH, HEIGHT) for px, py in car_corners_world]
    pygame.draw.polygon(screen, robot_color, car_corners_screen)
    pygame.draw.polygon(screen, BLACK, car_corners_screen, 2)

    # Front direction line
    front_x = mm.x + (CAR_LENGTH / 2) * math.cos(mm.theta)
    front_y = mm.y + (CAR_LENGTH / 2) * math.sin(mm.theta)
    fx_screen, fy_screen = mm.world_to_screen(front_x, front_y, WIDTH, HEIGHT)
    pygame.draw.line(screen, BLACK, (screen_x, screen_y), (fx_screen, fy_screen), 2)

    # # v and omega displayed in the top corner
    # v_text = font.render(f"v = {mm.v:.1f}", True, BLACK)
    # omega_text = font.render(f"omega = {mm.omega:.2f}", True, BLACK)
    # screen.blit(v_text, (10, 10))
    # screen.blit(omega_text, (10, 30))

    pygame.display.flip()

    # FPS and timestep
    mm.dt = clock.tick(60) / 1000

    # Update robot pose
    

pygame.quit()
sys.exit()