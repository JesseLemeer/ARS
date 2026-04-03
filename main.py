import pygame
import sys

pygame.init()

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("ARS Group 21")
clock = pygame.time.Clock()

x, y = 100,100
v = 5

running = True
while running:
    
    for event in pygame.event.get():
        #Closing
        if event.type == pygame.QUIT:
            running = False

    #Moving
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        x -= v
    if keys[pygame.K_RIGHT]:
        x += v
    if keys[pygame.K_UP]:
        y -= v
    if keys[pygame.K_DOWN]:
        y += v

    #Draw
    #Background
    screen.fill((255, 255, 255))
    
    #Objects
    pygame.draw.circle(screen, (255, 50, 50), (x, y), 10)
    
    #Updating
    pygame.display.flip()

    #Limits FPS
    clock.tick(60)

pygame.quit()
sys.exit()
