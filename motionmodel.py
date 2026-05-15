import math
import numpy as np

EPS = 1e-9

x, y, theta = 0.0, 0.0, 0.0
v = 0.0
omega = 0.0
dt = 0.0

SENSOR_ANGLES_DEG = [i * 30 for i in range(12)]
SENSOR_MAX_RANGE = 100.0
LANDMARK_SENSOR_RANGE = 150.0

#Normalizes any radian angle into the range [-pi, pi]
def normalize_angle(angle_rad):
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi

#Converts world coordinates to screen pixel coordinates
#Centers the view on the robot and flips the y-axis
def world_to_screen(wx, wy, screen_width, screen_height):
    screen_x = wx - x + screen_width / 2
    screen_y = -(wy - y) + screen_height / 2
    return int(screen_x), int(screen_y)

#Finds the closest point on a line segment (x1,y1)-(x2,y2) to point (px,py)
def closest_point_on_segment(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return x1, y1, 0.0, math.hypot(px - x1, py - y1)
    raw_t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, raw_t))
    cx = x1 + t * dx
    cy = y1 + t * dy
    return cx, cy, t, math.hypot(px - cx, py - cy)

#Returns distance to this closest point
def get_distance_to_segment(px, py, x1, y1, x2, y2):
    return closest_point_on_segment(px, py, x1, y1, x2, y2)[3]

#Used to detect robot edges crossing through a wall
def segment_crosses_wall_interior(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2):
    dax, day = ax2 - ax1, ay2 - ay1
    dbx, dby = bx2 - bx1, by2 - by1
    cross = dax * dby - day * dbx
    if abs(cross) < EPS:
        return False
    dx, dy = bx1 - ax1, by1 - ay1
    t = (dx * dby - dy * dbx) / cross
    u = (dx * day - dy * dax) / cross
    
    U_EPS = 1e-6
    return 0.0 <= t <= 1.0 and U_EPS < u < 1.0 - U_EPS

#Returns the wall unit normal facing towards point (px, py)
def wall_normal_for_point(wall, px, py):
    (x1, y1), (x2, y2) = wall
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    tx = dx / length
    ty = dy / length
    nx = -ty
    ny = tx
    #Flip if pointing in the wrong way
    if (px - x1) * nx + (py - y1) * ny < 0:
        nx, ny = -nx, -ny
    return nx, ny

#Calculates the four robot corners in world coordinates from some pose
def get_robot_corners_at(target_x, target_y, target_theta, length, width):
    half_l, half_w = length / 2, width / 2
    
    #Corner offsets in local robot frame
    local_corners = [(half_l, half_w), (half_l, -half_w),
                     (-half_l, -half_w), (-half_l, half_w)]
    c, s = math.cos(target_theta), math.sin(target_theta)
    
    #Rotate and translate to world coordinates
    return [(target_x + lx*c - ly*s, target_y + lx*s + ly*c)
            for lx, ly in local_corners]

#Checks if the robot collides with a single wall segment
#Detects both edge and corners collisions
def wall_collision(wall, corners, edges, ref_x, ref_y, margin):
    (w1x, w1y), (w2x, w2y) = wall
    edge_hit = any(segment_crosses_wall_interior(ex1, ey1, ex2, ey2, w1x, w1y, w2x, w2y)for (ex1, ey1), (ex2, ey2) in edges)

    #Find closest robot corner to wall
    best_dist = float('inf')
    best_t = None
    for cx, cy in corners:
        _, _, t, d = closest_point_on_segment(cx, cy, w1x, w1y, w2x, w2y)
        if d < best_dist:
            best_dist = d
            best_t = t

    INTERIOR_EPS = 1e-3
    prox_hit = (best_dist < margin and INTERIOR_EPS < best_t < 1.0 - INTERIOR_EPS)

    if not (edge_hit or prox_hit):
        return None

    nx, ny = wall_normal_for_point(wall, ref_x, ref_y)#using reference point that ensure the normal points the correct way

    return (nx, ny, best_dist)

#Returns a list of wall normals the robot is colliding with for sliding resolver
def get_all_collisions(rx, ry, rtheta, length, width, walls, margin=2.0, ref_point=None):
    if ref_point is None:
        ref_point = (rx, ry)
    ref_x, ref_y = ref_point

    corners = get_robot_corners_at(rx, ry, rtheta, length, width)
    n = len(corners)
    edges = [(corners[i], corners[(i + 1) % n]) for i in range(n)]

    out = []
    for wall in walls:
        hit = wall_collision(wall, corners, edges, ref_x, ref_y, margin)
        if hit is not None:
            nx, ny, _ = hit
            out.append((nx, ny))
    return out


#Checks if any robot edge crosses through any wall
def any_edge_crossing(rx, ry, rtheta, length, width, walls):
    corners = get_robot_corners_at(rx, ry, rtheta, length, width)
    n = len(corners)
    for i in range(n):
        e1x, e1y = corners[i]
        e2x, e2y = corners[(i + 1) % n]
        for wall in walls:
            (w1x, w1y), (w2x, w2y) = wall
            if segment_crosses_wall_interior(e1x, e1y, e2x, e2y,w1x, w1y, w2x, w2y):
                return True
    return False


#Checks if robot collides with any wall
def robot_collides_with_walls(rx, ry, rtheta, length, width, walls, margin=2.0):
    corners = get_robot_corners_at(rx, ry, rtheta, length, width)
    n = len(corners)
    edges = [(corners[i], corners[(i + 1) % n]) for i in range(n)]

    best = None
    for wall in walls:
        hit = wall_collision(wall, corners, edges, rx, ry, margin)
        if hit is None:
            continue
        _, _, depth = hit
        (w1x, w1y), (w2x, w2y) = wall
        closest_corner = min(
            corners,
            key=lambda c: get_distance_to_segment(c[0], c[1],
                                                  w1x, w1y, w2x, w2y))
        #Keep deepest collision
        if best is None or depth < best[0]:
            best = (depth, wall, closest_corner)
    if best is None:
        return False, None, None
    _, wall, corner = best
    return True, wall, corner

#Safely push the robot away from wall/corner contact, updated during swarm intelligence
def resolve_wall_normal_response(rx, ry, rtheta, length, width, walls,
                                 margin=2.0, push_step=2.0, iterations=12):
    resolved = False

    for _ in range(iterations):
        current_crossing = any_edge_crossing(rx, ry, rtheta, length, width, walls)
        normals = get_all_collisions(
            rx, ry, rtheta, length, width, walls,
            margin=margin,
            ref_point=(rx, ry),
        )

        if not normals and not current_crossing:
            break

        if normals:
            nx = sum(n[0] for n in normals)
            ny = sum(n[1] for n in normals)
        else:
            nearest_wall = min(
                walls,
                key=lambda w: get_distance_to_segment(
                    rx, ry, w[0][0], w[0][1], w[1][0], w[1][1]
                ),
            )
            nx, ny = wall_normal_for_point(nearest_wall, rx, ry)

        norm = math.hypot(nx, ny)
        if norm < EPS:
            break
        nx /= norm
        ny /= norm

        accepted = False
        step = push_step

        for _attempt in range(6):
            cand_x = rx + nx * step
            cand_y = ry + ny * step

            if not any_edge_crossing(cand_x, cand_y, rtheta, length, width, walls):
                rx, ry = cand_x, cand_y
                resolved = True
                accepted = True
                break

            step *= 0.5

        if not accepted:
            break

    if any_edge_crossing(rx, ry, rtheta, length, width, walls):
        return rx, ry, False

    return rx, ry, resolved

def resolve_sliding(px, py, ptheta, dx, dy, walls, length, width,iterations=8):
    collision_happened = False
    ref = (px, py)

    for _ in range(iterations):
        normals = get_all_collisions(
            px + dx, py + dy, ptheta, length, width, walls,
            ref_point=ref)
        if not normals:
            break

        collision_happened = True

        #Remove the component of movement pushing into each wall
        any_blocked = False
        for nx, ny in normals:
            dot = dx * nx + dy * ny
            if dot < 0.0:
                dx -= dot * nx
                dy -= dot * ny
                any_blocked = True

        if not any_blocked:
            break
        
        #Stop if remaining movement is insignificant
        if dx * dx + dy * dy < 1e-10:
            dx = dy = 0.0
            break

    #Sanity check
    if any_edge_crossing(px + dx, py + dy, ptheta, length, width, walls):
        return px, py, True

    return px + dx, py + dy, collision_happened

#Computes the displacement for x, y and theta for the current time step
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

#Moves the robot by one time step and resolves wall collisions
def update(walls, length, width):
    global x, y, theta
    dx, dy, dtheta = velocity_step()
    
    #Splits the movement into sub-steps to prevent tunneling 
    max_step = width * 0.4
    dist = math.hypot(dx, dy)
    steps = max(1, int(dist / max_step) + 1)

    step_dx = dx / steps
    step_dy = dy / steps
    step_dtheta = dtheta / steps

    collision_occurred = False
    for _ in range(steps):
         #Try to apply rotation
        new_theta = theta + step_dtheta
        rot_collided, _, _ = robot_collides_with_walls(
            x, y, new_theta, length, width, walls)
        if not rot_collided:
            theta = new_theta
        else:
            collision_occurred = True
            x, y, _ = resolve_wall_normal_response(
                x, y, theta, length, width, walls
            )

        new_x, new_y, trans_collided = resolve_sliding(
            x, y, theta, step_dx, step_dy, walls, length, width)
        x, y = new_x, new_y
        if trans_collided:
            collision_occurred = True
            x, y, _ = resolve_wall_normal_response(
                x, y, theta, length, width, walls
            )

    return collision_occurred

#Computes the intersection of a ray and a line segment
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


#Finds all landmarks in a certain range
def get_landmark_measurements(landmark_groups, max_range=LANDMARK_SENSOR_RANGE):
    readings = []
    for landmark in landmark_groups:
        landmark_id = landmark["id"]
        landmark_x, landmark_y = landmark["center"]
        delta_x = landmark_x - x
        delta_y = landmark_y - y
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
        
        if distance > max_range:
            continue
        #Compute bearing relative to robot heading
        global_bearing = math.atan2(delta_y, delta_x)
        relative_bearing = normalize_angle(global_bearing - theta)
        readings.append({
            "landmark_id": landmark_id,
            "landmark_center": (landmark_x, landmark_y),
            "distance": distance,
            "bearing_rad": relative_bearing,
        })
    return readings

#Casts rays at each sensor angle and returns the distance and hit point for nearest wall
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
        
        #Finds nearest wall along ray
        for wall in walls:
            result = ray_segment_intersection(
                x, y, dx, dy,
                wall[0][0], wall[0][1],
                wall[1][0], wall[1][1])
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
            "hit_point": (hit_x, hit_y),
        })
    return readings


#Calculates the four corner positions of the robot using its current global pose
def get_robot_corners(length, width):
    half_l = length / 2
    half_w = width / 2
    local_corners = [
        (half_l, half_w),
        (half_l, -half_w),
        (-half_l, -half_w),
        (-half_l, half_w),
    ]
    c = math.cos(theta)
    s = math.sin(theta)
    world_corners = []
    for lx, ly in local_corners:
        wx = x + lx * c - ly * s
        wy = y + lx * s + ly * c
        world_corners.append((wx, wy))
    return world_corners
