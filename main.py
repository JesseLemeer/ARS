import pygame
import sys
import motionmodel as mm

pygame.init()

ORANGE = (255, 77, 0)
BLACK = (0, 0, 0)

OMEGA = 5.0
VELOCITY = 100.0

RADIUS_ROBOT = 10

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("ARS Group 21")
clock = pygame.time.Clock()


running = True
while running:
    
    for event in pygame.event.get():
        #Closing
        if event.type == pygame.QUIT:
            running = False

    #Moving
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

    #Draw
    #Background
    screen.fill((255, 255, 255))
    
    #Robot
    screen_x, screen_y = mm.world_to_screen(mm.x, mm.y, WIDTH, HEIGHT)
    pygame.draw.circle(screen, ORANGE, (screen_x, screen_y), RADIUS_ROBOT)
    
    #Line indicating forward direction
    line_x, line_y = mm.line_endpoint(RADIUS_ROBOT)
    lx_screen, ly_screen = mm.world_to_screen(line_x, line_y, WIDTH, HEIGHT)
    #Uses center of robot as beginning point 
    pygame.draw.line(screen, BLACK, (screen_x, screen_y), (lx_screen, ly_screen), 2)
    
    #Updates drawings
    pygame.display.flip()

    #Limits FPS and coverts stepsize to seconds
    mm.dt = clock.tick(60) / 1000
    
    #Updates x, y and theta
    mm.update()

pygame.quit()
sys.exit()
