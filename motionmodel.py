import math
EPS = 1e-9

# Starting coordinates
x, y, theta = 0.0, 0.0, 0.0  # position and angle the robot is facing

# velocity and angular velocity
v = 0.0
omega = 0.0

# Timestep
dt = 0.0

# 12 sensors, every 30 degrees...
SENSOR_ANGLES_DEG = [i * 30 for i in range(12)]
SENSOR_MAX_RANGE = 200.0


# Map world (0,0) to screen center
def world_to_screen(world_x, world_y, width, height):
    screen_x = width / 2 + int(world_x)
    screen_y = height / 2 - int(world_y)
    return screen_x, screen_y


def get_distance_to_segment(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1

    if dx == 0 and dy == 0:
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

    tx = dx / length
    ty = dy / length

    nx = -ty
    ny = tx

    vx = px - x1
    vy = py - y1

    if vx * nx + vy * ny < 0:
        nx = -nx
        ny = -ny

    return nx, ny

def velocity_step():
    global x, y, theta, v, omega, dt

    if abs(omega) < EPS:
        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt
    else:
        theta_new = theta + omega * dt
        dx = (v / omega) * (math.sin(theta_new) - math.sin(theta))
        dy = -(v / omega) * (math.cos(theta_new) - math.cos(theta))

    dtheta = omega * dt
    return dx, dy, dtheta


def resolve_sliding(px, py, dx, dy, walls, radius, iterations=4):
    for _ in range(iterations):
        nx = px + dx
        ny = py + dy
        collided = False

        for wall in walls:
            dist = get_distance_to_segment(nx, ny, wall[0][0], wall[0][1], wall[1][0], wall[1][1])

            if dist < radius:
                collided = True
                normal_x, normal_y = wall_normal_for_point(wall, nx, ny)

                dot = dx * normal_x + dy * normal_y
                if dot < 0.0:
                    dx -= dot * normal_x
                    dy -= dot * normal_y

                nx = px + dx
                ny = py + dy

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

    max_step = radius * 0.5
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

def ray_segment_intersection(rx, ry, rdx, rdy, x1, y1, x2, y2):
    sx = x2 - x1
    sy = y2 - y1
    cross_rs = rdx * sy - rdy * sx
    if abs(cross_rs) < EPS:
        return None  # parallel

    qpx = x1 - rx
    qpy = y1 - ry

    t = (qpx * sy - qpy * sx) / cross_rs
    u = (qpx * rdy - qpy * rdx) / cross_rs

    if t >= 0.0 and 0.0 <= u <= 1.0:
        ix = rx + t * rdx
        iy = ry + t * rdy
        return t, ix, iy
    return None

def get_sensor_readings(walls, sensor_angles_deg=None, max_range=SENSOR_MAX_RANGE):
    if sensor_angles_deg is None:
        sensor_angles_deg = SENSOR_ANGLES_DEG
    readings = []

    for angle_deg in sensor_angles_deg:
        angle = theta + math.radians(angle_deg)
        dx = math.cos(angle)
        dy = math.sin(angle)

        best_dist = max_range
        hit_x = x + max_range * dx
        hit_y = y + max_range * dy

        for wall in walls:
            result = ray_segment_intersection(
                x, y, dx, dy,
                wall[0][0], wall[0][1],
                wall[1][0], wall[1][1]
            )
            if result is None:
                continue
            dist, ix, iy = result
            if dist < best_dist:
                best_dist = dist
                hit_x = ix
                hit_y = iy
        readings.append({
            "angle_deg": angle_deg,
            "distance": best_dist,
            "hit_point": (hit_x, hit_y)
        })

    return readings

def get_robot_corners(length, width):
    half_l = length / 2
    half_w = width / 2

    # local rectangle corners around center
    local_corners = [
        ( half_l,  half_w),   # front-left
        ( half_l, -half_w),   # front-right
        (-half_l, -half_w),   # rear-right
        (-half_l,  half_w),   # rear-left
    ]

    world_corners = []
    c = math.cos(theta)
    s = math.sin(theta)

    for lx, ly in local_corners:
        wx = x + lx * c - ly * s
        wy = y + lx * s + ly * c
        world_corners.append((wx, wy))

    return world_corners