import math
EPS = 1e-9

# Starting coordinates
x, y, theta = 0.0, 0.0, 0.0

# velocity and angular velocity
v = 0.0
omega = 0.0

# Timestep
dt = 0.0

SENSOR_ANGLES_DEG = [i * 30 for i in range(12)]
SENSOR_MAX_RANGE = 200.0


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


def segment_intersects_segment(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2):
    """Returns True if segment A intersects segment B."""
    dax, day = ax2 - ax1, ay2 - ay1
    dbx, dby = bx2 - bx1, by2 - by1
    cross = dax * dby - day * dbx
    if abs(cross) < EPS:
        return False
    dx, dy = bx1 - ax1, by1 - ay1
    t = (dx * dby - dy * dbx) / cross
    u = (dx * day - dy * dax) / cross
    return 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0


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


def get_robot_corners_at(target_x, target_y, target_theta, length, width):
    half_l, half_w = length / 2, width / 2
    local_corners = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]
    c, s = math.cos(target_theta), math.sin(target_theta)
    return [(target_x + lx*c - ly*s, target_y + lx*s + ly*c) for lx, ly in local_corners]


def robot_collides_with_walls(rx, ry, rtheta, length, width, walls, margin=2.0):
    corners = get_robot_corners_at(rx, ry, rtheta, length, width)
    n = len(corners)
    edges = [(corners[i], corners[(i + 1) % n]) for i in range(n)]

    for wall in walls:
        (w1x, w1y), (w2x, w2y) = wall

        # Corner proximity check
        for cx, cy in corners:
            if get_distance_to_segment(cx, cy, w1x, w1y, w2x, w2y) < margin:
                return True, wall, (cx, cy)

        # Edge crossing check
        for (ex1, ey1), (ex2, ey2) in edges:
            if segment_intersects_segment(ex1, ey1, ex2, ey2, w1x, w1y, w2x, w2y):
                best_corner = min(
                    corners,
                    key=lambda c: get_distance_to_segment(c[0], c[1], w1x, w1y, w2x, w2y)
                )
                return True, wall, best_corner

    return False, None, None


def resolve_sliding(px, py, ptheta, dx, dy, walls, length, width, iterations=6):
    collision_happened = False

    for _ in range(iterations):
        collided, hit_wall, hit_corner = robot_collides_with_walls(
            px + dx, py + dy, ptheta, length, width, walls
        )
        if not collided:
            break

        collision_happened = True
        cx, cy = hit_corner
        normal_x, normal_y = wall_normal_for_point(hit_wall, cx, cy)

        dot = dx * normal_x + dy * normal_y
        if dot < 0.0:
            dx -= dot * normal_x
            dy -= dot * normal_y
        else:
            # Already sliding parallel or moving away — stop to avoid infinite loop
            dx, dy = 0.0, 0.0
            break

    return px + dx, py + dy, collision_happened


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


def update(walls, length, width):
    global x, y, theta
    dx, dy, dtheta = velocity_step()

    max_step = width * 0.4
    dist = math.hypot(dx, dy)
    steps = max(1, int(dist / max_step) + 1)

    step_dx = dx / steps
    step_dy = dy / steps
    step_dtheta = dtheta / steps

    collision_occurred = False
    for _ in range(steps):
        # Check rotation separately before applying it
        new_theta = theta + step_dtheta
        rot_collided, _, _ = robot_collides_with_walls(x, y, new_theta, length, width, walls)
        if not rot_collided:
            theta = new_theta
        else:
            collision_occurred = True
            # Block the rotation, don't update theta

        # Handle translation with sliding
        new_x, new_y, trans_collided = resolve_sliding(x, y, theta, step_dx, step_dy, walls, length, width)
        x, y = new_x, new_y
        if trans_collided:
            collision_occurred = True

    return collision_occurred


def line_endpoint(length):
    line_x = x + length * math.cos(theta)
    line_y = y + length * math.sin(theta)
    return line_x, line_y


def ray_segment_intersection(rx, ry, rdx, rdy, x1, y1, x2, y2):
    sx = x2 - x1
    sy = y2 - y1
    cross_rs = rdx * sy - rdy * sx
    if abs(cross_rs) < EPS:
        return None

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
    local_corners = [
        ( half_l,  half_w),
        ( half_l, -half_w),
        (-half_l, -half_w),
        (-half_l,  half_w),
    ]
    c = math.cos(theta)
    s = math.sin(theta)
    world_corners = []
    for lx, ly in local_corners:
        wx = x + lx * c - ly * s
        wy = y + lx * s + ly * c
        world_corners.append((wx, wy))
    return world_corners