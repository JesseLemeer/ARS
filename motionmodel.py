import math
import numpy as np
EPS = 1e-9

x, y, theta = 0.0, 0.0, 0.0
v = 0.0
omega = 0.0
dt = 0.0

SENSOR_ANGLES_DEG = [i * 10 for i in range(36)]
SENSOR_MAX_RANGE = 100.0
LANDMARK_SENSOR_RANGE = 150.0


def normalize_angle(angle_rad):
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi

def world_to_screen(wx, wy, screen_width, screen_height):
    screen_x = wx - x + screen_width / 2
    screen_y = -(wy - y) + screen_height / 2
    return int(screen_x), int(screen_y)


# ─────────────────────────────────────────────────────────────────────────────
# Geometry primitives
# ─────────────────────────────────────────────────────────────────────────────

def closest_point_on_segment(px, py, x1, y1, x2, y2):
    """Return (closest_x, closest_y, t, dist) where t is the *clamped*
    parameter along the segment (0 = start, 1 = end), so 0 < t < 1 means the
    closest point is interior to the segment. dist is Euclidean."""
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return x1, y1, 0.0, math.hypot(px - x1, py - y1)
    raw_t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, raw_t))
    cx = x1 + t * dx
    cy = y1 + t * dy
    return cx, cy, t, math.hypot(px - cx, py - cy)


def get_distance_to_segment(px, py, x1, y1, x2, y2):
    return closest_point_on_segment(px, py, x1, y1, x2, y2)[3]


def segment_intersects_segment(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2):
    dax, day = ax2 - ax1, ay2 - ay1
    dbx, dby = bx2 - bx1, by2 - by1
    cross = dax * dby - day * dbx
    if abs(cross) < EPS:
        return False
    dx, dy = bx1 - ax1, by1 - ay1
    t = (dx * dby - dy * dbx) / cross
    u = (dx * day - dy * dax) / cross
    return 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0


def segment_crosses_wall_interior(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2):
    """True iff segment A crosses segment B with the intersection point
    strictly INTERIOR to B (the wall). Endpoint-only contacts (u≈0 or u≈1)
    do NOT count, because past an endpoint the wall does not physically
    exist — a robot edge passing through a wall endpoint is touching empty
    space, not crossing wall material."""
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


def wall_normal_for_point(wall, px, py):
    """Wall normal pointing toward (px, py)."""
    (x1, y1), (x2, y2) = wall
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    tx = dx / length
    ty = dy / length
    nx = -ty
    ny = tx
    if (px - x1) * nx + (py - y1) * ny < 0:
        nx, ny = -nx, -ny
    return nx, ny


def get_robot_corners_at(target_x, target_y, target_theta, length, width):
    half_l, half_w = length / 2, width / 2
    local_corners = [(half_l, half_w), (half_l, -half_w),
                     (-half_l, -half_w), (-half_l, half_w)]
    c, s = math.cos(target_theta), math.sin(target_theta)
    return [(target_x + lx*c - ly*s, target_y + lx*s + ly*c)
            for lx, ly in local_corners]


# ─────────────────────────────────────────────────────────────────────────────
# Collision detection
#
# Two failure modes the previous version had, both fixed here:
#
# (1) Phantom wall extension. A wall is a finite segment, but
#     `wall_normal_for_point` builds the normal of the wall's *infinite* line.
#     A robot corner sitting near a wall ENDPOINT (within `margin`) was
#     treated as colliding with the wall surface even though geometrically
#     the wall has ended. Fix: proximity hits only count when the closest
#     point on the segment is INTERIOR (0 < t < 1).
#
# (2) Wrong-side normal. The normal direction was chosen by passing the
#     closest robot corner to `wall_normal_for_point`. If that corner has
#     already crossed the wall, the normal points toward the WRONG side,
#     `dot >= 0` for any velocity, and the resolver concludes the robot is
#     "moving away" and lets it teleport through. Fix: always pass the
#     pre-move robot CENTER (or another reference known to be on the safe
#     side) when computing the normal.
# ─────────────────────────────────────────────────────────────────────────────

def _wall_collision(wall, corners, edges, ref_x, ref_y, margin):
    """Test one wall against the robot footprint.

    `ref_x, ref_y` is a point on the SAFE side of the wall (typically the
    pre-move robot center) — used to choose normal direction reliably even if
    a robot corner has crossed.

    Returns (nx, ny, depth) or None.
    `depth` is the smallest signed distance from any robot corner to the
    wall's infinite line, measured in the *outward* normal direction. More
    negative = deeper penetration.
    """
    (w1x, w1y), (w2x, w2y) = wall

    # 1. Edge-intersection — robot edge crosses the wall's interior surface.
    #    We require the crossing to be INTERIOR to the wall segment (u strictly
    #    in (0,1)) because the wall only physically exists between its two
    #    endpoints; touching an endpoint means the robot has reached the end of
    #    the wall, not passed through it.  Using the non-strict version here
    #    was causing phantom "blocked" results at convex corners where the
    #    robot legitimately slides past the end of a finite wall.
    edge_hit = any(
        segment_crosses_wall_interior(ex1, ey1, ex2, ey2, w1x, w1y, w2x, w2y)
        for (ex1, ey1), (ex2, ey2) in edges
    )

    # 2. Closest robot corner to the wall segment, plus the parameter t.
    best_dist = float('inf')
    best_t = None
    for cx, cy in corners:
        _, _, t, d = closest_point_on_segment(cx, cy, w1x, w1y, w2x, w2y)
        if d < best_dist:
            best_dist = d
            best_t = t

    # 3. Proximity hit only counts when the closest point is interior to the
    #    wall — past an endpoint the wall doesn't physically exist, so a
    #    proximity-to-endpoint must NOT be treated as a wall surface hit.
    INTERIOR_EPS = 1e-3
    prox_hit = (best_dist < margin
                and INTERIOR_EPS < best_t < 1.0 - INTERIOR_EPS)

    if not (edge_hit or prox_hit):
        return None

    # 4. Normal direction: anchor on the safe-side reference point so a
    #    crossed corner can never invert it.
    nx, ny = wall_normal_for_point(wall, ref_x, ref_y)

    return (nx, ny, best_dist)


def get_all_collisions(rx, ry, rtheta, length, width, walls,
                       margin=2.0, ref_point=None):
    """Return a list of (nx, ny) outward normals for every wall the robot
    is currently in contact with.

    `ref_point` is a (refx, refy) on the safe side of all walls; if omitted
    the robot center (rx, ry) is used (acceptable for the rotation check
    where the body has not been moved)."""
    if ref_point is None:
        ref_point = (rx, ry)
    ref_x, ref_y = ref_point

    corners = get_robot_corners_at(rx, ry, rtheta, length, width)
    n = len(corners)
    edges = [(corners[i], corners[(i + 1) % n]) for i in range(n)]

    out = []
    for wall in walls:
        hit = _wall_collision(wall, corners, edges, ref_x, ref_y, margin)
        if hit is not None:
            nx, ny, _ = hit
            out.append((nx, ny))
    return out


def _any_edge_crossing(rx, ry, rtheta, length, width, walls):
    """Strict test: does any robot edge cross any wall's interior surface?
    No margin, no normals — a final sanity check that the resolved position
    genuinely doesn't penetrate wall material.  Uses the same interior-only
    criterion as edge_hit in _wall_collision so the two are consistent."""
    corners = get_robot_corners_at(rx, ry, rtheta, length, width)
    n = len(corners)
    for i in range(n):
        e1x, e1y = corners[i]
        e2x, e2y = corners[(i + 1) % n]
        for wall in walls:
            (w1x, w1y), (w2x, w2y) = wall
            if segment_crosses_wall_interior(e1x, e1y, e2x, e2y,
                                             w1x, w1y, w2x, w2y):
                return True
    return False


def robot_collides_with_walls(rx, ry, rtheta, length, width, walls, margin=2.0):
    """Used for the rotation pre-check in update(). The robot is at (rx,ry)
    in the *current* (already accepted) translation state, so (rx,ry) itself
    is on the safe side."""
    corners = get_robot_corners_at(rx, ry, rtheta, length, width)
    n = len(corners)
    edges = [(corners[i], corners[(i + 1) % n]) for i in range(n)]

    best = None
    for wall in walls:
        hit = _wall_collision(wall, corners, edges, rx, ry, margin)
        if hit is None:
            continue
        nx, ny, depth = hit
        # find the corner that generated this depth, for the legacy
        # (collided, wall, corner) return signature
        (w1x, w1y), (w2x, w2y) = wall
        closest_corner = min(
            corners,
            key=lambda c: get_distance_to_segment(c[0], c[1],
                                                  w1x, w1y, w2x, w2y))
        if best is None or depth < best[0]:
            best = (depth, wall, closest_corner)
    if best is None:
        return False, None, None
    _, wall, corner = best
    return True, wall, corner


# ─────────────────────────────────────────────────────────────────────────────
# Sliding resolver
#
# Strategy:
#   * Each iteration, gather ALL outward normals of currently-touching walls.
#   * Project the desired displacement against every normal whose dot with
#     velocity is negative (i.e. velocity has a component INTO that wall).
#   * If no normal has dot < 0 we're either truly cornered (≥ 2 normals
#     spanning all directions of motion) or only brushing margin — either
#     way, stop iterating.
#   * Final sanity check: if the resolved displacement still produces any
#     edge crossing, fall back to no movement. This is the guarantee against
#     teleporting through walls.
# ─────────────────────────────────────────────────────────────────────────────

def resolve_sliding(px, py, ptheta, dx, dy, walls, length, width,
                    iterations=8):
    collision_happened = False
    ref = (px, py)  # the original center is on the safe side by precondition

    for _ in range(iterations):
        normals = get_all_collisions(
            px + dx, py + dy, ptheta, length, width, walls,
            ref_point=ref)
        if not normals:
            break

        collision_happened = True

        any_blocked = False
        for nx, ny in normals:
            dot = dx * nx + dy * ny
            if dot < 0.0:
                dx -= dot * nx
                dy -= dot * ny
                any_blocked = True

        if not any_blocked:
            # Touching wall(s) but velocity is parallel or moving away from
            # all of them. Treat as a margin artifact and accept.
            break

        if dx * dx + dy * dy < 1e-10:
            dx = dy = 0.0
            break

    # Hard guarantee: if the final position has any geometric wall crossing,
    # don't move. Better to be stationary than to phase through.
    if _any_edge_crossing(px + dx, py + dy, ptheta, length, width, walls):
        return px, py, True

    return px + dx, py + dy, collision_happened


# ─────────────────────────────────────────────────────────────────────────────
# Motion update — unchanged in structure, just calls the new sliding code.
# ─────────────────────────────────────────────────────────────────────────────

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
        new_theta = theta + step_dtheta
        rot_collided, _, _ = robot_collides_with_walls(
            x, y, new_theta, length, width, walls)
        if not rot_collided:
            theta = new_theta
        else:
            collision_occurred = True

        new_x, new_y, trans_collided = resolve_sliding(
            x, y, theta, step_dx, step_dy, walls, length, width)
        x, y = new_x, new_y
        if trans_collided:
            collision_occurred = True

    return collision_occurred


# ─────────────────────────────────────────────────────────────────────────────
# Below: sensor / measurement code unchanged from the original.
# ─────────────────────────────────────────────────────────────────────────────

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


def nearest_intersection_on_segments(ray_origin_x, ray_origin_y,
                                     ray_dir_x, ray_dir_y, segments,
                                     max_range):
    best_dist = max_range
    best_hit = None
    for seg in segments:
        result = ray_segment_intersection(
            ray_origin_x, ray_origin_y, ray_dir_x, ray_dir_y,
            seg[0][0], seg[0][1], seg[1][0], seg[1][1])
        if result is None:
            continue
        dist, ix, iy = result
        if dist < best_dist:
            best_dist = dist
            best_hit = (ix, iy)
    if best_hit is None:
        return None
    return best_dist, best_hit[0], best_hit[1]


def get_landmark_readings_with_occlusion(landmark_groups, walls,
                                         sensor_angles_deg=None,
                                         max_range=SENSOR_MAX_RANGE):
    if sensor_angles_deg is None:
        sensor_angles_deg = SENSOR_ANGLES_DEG
    readings = []
    for landmark in landmark_groups:
        landmark_id = landmark["id"]
        center = landmark["center"]
        segments = landmark["segments"]
        best_reading = None
        for angle_deg in sensor_angles_deg:
            angle = theta + math.radians(angle_deg)
            dx = math.cos(angle)
            dy = math.sin(angle)
            landmark_hit = nearest_intersection_on_segments(
                x, y, dx, dy, segments, max_range)
            if landmark_hit is None:
                continue
            landmark_dist, hit_x, hit_y = landmark_hit
            wall_hit = nearest_intersection_on_segments(
                x, y, dx, dy, walls, max_range)
            if wall_hit is not None:
                wall_dist, _, _ = wall_hit
                if wall_dist < landmark_dist:
                    continue
            if best_reading is None or landmark_dist < best_reading["distance"]:
                best_reading = {
                    "landmark_id": landmark_id,
                    "landmark_center": center,
                    "angle_deg": angle_deg,
                    "distance": landmark_dist,
                    "hit_point": (hit_x, hit_y),
                }
        if best_reading is not None:
            readings.append(best_reading)
    return readings


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
        global_bearing = math.atan2(delta_y, delta_x)
        relative_bearing = normalize_angle(global_bearing - theta)
        readings.append({
            "landmark_id": landmark_id,
            "landmark_center": (landmark_x, landmark_y),
            "distance": distance,
            "bearing_rad": relative_bearing,
        })
    return readings


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