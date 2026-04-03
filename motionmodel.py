import math

#Starting coordinates
x, y, theta = 0.0, 0.0, 0.0

#Velocities set by input each frame, for now just key inputs
v = 0.0
omega = 0.0

#Timestep in seconds, set by main loop each frame
dt = 0.0


#Map world (0,0) to screen center
def world_to_screen(world_x, world_y, width, height):
    screen_x = width / 2 + int(world_x)
    screen_y = height / 2 - int(world_y)
    return screen_x, screen_y


def update():
    global x, y, theta
    #No change in theta (omega=0), just straight line movement
    if abs(omega) < 1e-6:
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += omega * dt
    #Noise-free Velocity Model
    else:
        x_c = x - (v / omega) * math.sin(theta)
        y_c = y + (v / omega) * math.cos(theta)
        
        theta = theta + omega * dt
        x = x_c + (v / omega) * math.sin(theta)
        y = y_c - (v / omega) * math.cos(theta)
        

#Calculates endpoint of the line indicating forward direction
def line_endpoint(length):
    line_x = x + length * math.cos(theta)
    line_y = y + length * math.sin(theta)
    return line_x, line_y
