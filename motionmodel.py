import math
EPS = 1e-9
# Starting coordinates
x, y, theta = 0.0, 0.0, 0.0 #position and angle the robot is facing

#velocity and angular velocity, respectively
v = 0.0
omega = 0.0

# Timestep
dt = 0.0

#Map world (0,0) to screen center
def world_to_screen(world_x, world_y, width, height):
    screen_x = width / 2 + int(world_x)
    screen_y = height / 2 - int(world_y)
    return screen_x, screen_y


def get_distance_to_segment(px, py, x1, y1, x2, y2):
    #px,py is position of robot, x1...y2 is a wall segment (need to loop over all segments)
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:#for the case a segment is a single point. We might want to do this e.g. to represent pedestrians.
        return math.hypot(px - x1, py - y1)

    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    return math.hypot(px - closest_x, py - closest_y)


def wall_normal_for_point(wall, px, py):
    (x1, y1), (x2, y2) = wall

    dx = x2 - x1
    dy = y2 - y1

    length = math.hypot(dx, dy)
    # Unit tangent
    tx = dx / length
    ty = dy / length
    #compute unit normal by rotation tangent
    nx = -ty
    ny = tx
    #two possible normals, choose the one that faces the robot
    vx = px - x1
    vy = py - y1

    if vx * nx + vy * ny < 0:
        nx = -nx
        ny = -ny

    return nx, ny

def velocity_step():
    #noise free velocity update model
    global x, y, theta, v, omega, dt

    if abs(omega) < EPS: #the robot is not turning
        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt
    else: #the robot is turning, need to consider angular velocity
        theta_new = theta + omega * dt
        dx = (v / omega) * (math.sin(theta_new) - math.sin(theta))
        dy = -(v / omega) * (math.cos(theta_new) - math.cos(theta))

    dtheta = omega * dt
    return dx, dy, dtheta


def resolve_sliding(px, py, dx, dy, walls, radius, iterations=4):
    #if hitting wall, only keep the motion tangential to (i.e. along) the wall 
    for _ in range(iterations):#necessary because robot may hit multiple walls at once. One iteration only ensure slide conforms to the direction of a single wall, need to double check otehr walls each time
        nx = px + dx
        ny = py + dy

        collided = False

        for wall in walls:
            dist = get_distance_to_segment(nx, ny, wall[0][0], wall[0][1], wall[1][0], wall[1][1])

            if dist < radius:
                collided = True
                normal_x, normal_y = wall_normal_for_point(wall, nx, ny)

                # Remove motion component that goes into the wall
                dot = dx * normal_x + dy * normal_y
                if dot < 0.0:
                    dx -= dot * normal_x
                    dy -= dot * normal_y

                # Recompute position after projection
                nx = px + dx
                ny = py + dy

                # Push out if still slightly penetrating
                dist = get_distance_to_segment(nx, ny, wall[0][0], wall[0][1], wall[1][0], wall[1][1])
                if dist < radius:
                    push = (radius - dist) + EPS
                    nx += normal_x * push
                    ny += normal_y * push
                    dx = nx - px
                    dy = ny - py

        if not collided:
            break

    return px + dx, py + dy


def update(walls, radius):
    global x, y, theta

    dx, dy, dtheta = velocity_step()

    # Determine how many substeps we need
    max_step = radius * 0.5   # safe distance per step
    dist = math.hypot(dx, dy)

    steps = max(1, int(dist / max_step) + 1)

    step_dx = dx / steps
    step_dy = dy / steps
    step_dtheta = dtheta / steps

    for _ in range(steps):
        new_x, new_y = resolve_sliding(x, y, step_dx, step_dy, walls, radius)
        x, y = new_x, new_y
        theta += step_dtheta


def line_endpoint(length):
    line_x = x + length * math.cos(theta)
    line_y = y + length * math.sin(theta)
    return line_x, line_y