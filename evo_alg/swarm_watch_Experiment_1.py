from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pygame

BASE_DIR = Path(__file__).resolve().parent

try:
    from evo_alg._path_setup import ensure_project_root_on_path
except ModuleNotFoundError:
    from _path_setup import ensure_project_root_on_path

ensure_project_root_on_path(__file__)

import map as mp
import motionmodel as mm
from occupancygrid import OccupancyGrid
from evo_alg.ea_tools import FeedforwardController, WallFollowRecovery
from evo_alg.ea_navigation import (
    DEFAULT_GRID,
    bootstrap_navigation,
    make_navigation_state,
    raw_sensor_activations,
    refresh_grid_stats,
    update_navigation,
)

GENOME_FILE = sys.argv[1] if len(sys.argv) > 1 else str(BASE_DIR / "best_genome_staged_second_attempt.npy")

DT = 0.05
CAR_LENGTH = 24
CAR_WIDTH = 14

MAX_V = 70.0
MAX_OMEGA = 5.0

N_ROBOTS = 4
TRAIL_LEN = 250
GOAL_RADIUS = 25.0

# Workload balancing: after 3 goals, a robot stops taking new goals.
MAX_GOALS_PER_ROBOT = 3

# Recovery related
RECOVERY_STEPS = 45
RECOVERY_COOLDOWN_STEPS = 35
STUCK_WINDOW_SWARM = 80
STUCK_DIST_SWARM = 8.0
COLLISION_RECOVERY_THRESHOLD = 5
RECOVERY_V = -35.0
RECOVERY_OMEGA = 2.2

# Robot-robot physical interaction settings.
ROBOT_WARNING_DIST = 95.0
ROBOT_SEPARATION_DIST = 62.0
ROBOT_COLLISION_MARGIN = 2.0
ROBOT_COLLISION_COUNT_COOLDOWN = 20
WALL_COLLISION_COUNT_COOLDOWN = 8
GOAL_ASSIGNMENT_SPACING = 115.0

# Anti-circling / no-progress detection.
CIRCLING_WINDOW = 160
GOAL_PROGRESS_EPS = 8.0
STAGNATION_RECOVERY_STEPS = 35

# Safer robot-robot push and stronger repeated-corner escape.
SAFE_ROBOT_PUSH_ATTEMPTS = 7
CORNER_TRAP_DIST = 28.0
CORNER_TRAP_HITS = 4
CORNER_ESCAPE_STEPS = 95

N_SENSORS = len(mm.SENSOR_ANGLES_DEG)
N_GOAL_INPUTS = 3
N_INPUTS = N_SENSORS + N_GOAL_INPUTS
N_OUTPUTS = 2

MAP_W, MAP_H = 1000, 780
PANEL_W = 360
SCREEN_W, SCREEN_H = MAP_W + PANEL_W, MAP_H
CAMERA_X, CAMERA_Y = -70.0, 0.0

TASK_GOALS = [
    (-300, 100),
    (-312, 145),
    (278, -203),
    (341, 217),
    (203, -189),
    (-367, 112),
    (94, 261),
    (-241, -238),
    (318, 43),
]

START_POSES = [
    (-200.0, 0.0, 0.0),
    (0.0, -200.0, math.radians(90.0)),
    (200.0, 0.0, math.radians(180.0)),
    (0.0, 200.0, math.radians(-90.0)),
]

ROBOT_COLOURS = [
    (220, 40, 40),
    (40, 140, 220),
    (40, 170, 80),
    (210, 140, 30),
]

WHITE = (255, 255, 255)
BLACK = (20, 20, 20)
ORANGE = (255, 100, 0)
PURPLE = (130, 60, 200)
RED = (200, 0, 0)
GREY = (80, 80, 80)
CYAN = (0, 180, 180)

@dataclass
class SwarmRobot:
    robot_id: int
    x: float
    y: float
    theta: float
    nav_state: object
    goal: tuple[float, float] | None = None
    v: float = 0.0
    omega: float = 0.0
    collisions: int = 0
    wall_collisions: int = 0
    robot_collisions: int = 0
    goals_reached: int = 0
    last_wall_readings: list = field(default_factory=list)
    last_raw_acts: np.ndarray = field(default_factory=lambda: np.zeros(N_SENSORS))
    trail: list[tuple[float, float]] = field(default_factory=list)
    est_trail: list[tuple[float, float]] = field(default_factory=list)

    wall_follower: object = field(default_factory=WallFollowRecovery)
    recovery_steps: int = 0
    recovery_cooldown: int = 0
    recent_positions: list[tuple[float, float]] = field(default_factory=list)
    recent_collision_steps: int = 0
    last_collision_type: str = "none"
    robot_contact_cooldown: int = 0
    wall_contact_cooldown: int = 0
    best_goal_distance: float = float("inf")
    no_progress_steps: int = 0
    previous_goal: tuple[float, float] | None = None
    active_steps: int = 0
    stopped_at_step: int | None = None
    stop_reason: str = "active"
    wall_trap_anchor: tuple[float, float] | None = None
    wall_trap_hits: int = 0

    @property
    def pose_error(self) -> float:
        return math.hypot(self.x - self.nav_state.est_x, self.y - self.nav_state.est_y)

def world_to_screen(wx: float, wy: float, screen_width: int = MAP_W, screen_height: int = MAP_H):
    sx = wx - CAMERA_X + screen_width / 2
    sy = -(wy - CAMERA_Y) + screen_height / 2
    return int(sx), int(sy)

def infer_hidden_size(genome_len: int) -> int:
    numerator = genome_len - N_OUTPUTS
    denominator = N_INPUTS + 1 + N_OUTPUTS
    if numerator <= 0 or numerator % denominator != 0:
        raise ValueError(
            f"Cannot infer hidden size from genome length {genome_len}. "
            f"Expected length = hidden*{denominator}+{N_OUTPUTS}."
        )
    return numerator // denominator

def goal_activations(est_x: float, est_y: float, est_theta: float, goal_x: float, goal_y: float) -> np.ndarray:
    dx, dy = goal_x - est_x, goal_y - est_y
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return np.array([0.0, 0.0, 1.0], dtype=float)

    bearing = mm.normalize_angle(math.atan2(dy, dx) - est_theta)
    return np.array(
        [
            1.0 - math.exp(-dist / 100.0),
            math.sin(bearing),
            math.cos(bearing),
        ],
        dtype=float,
    )

def load_robot_into_motion_model(robot: SwarmRobot) -> None:
    mm.x = robot.x
    mm.y = robot.y
    mm.theta = robot.theta
    mm.v = robot.v
    mm.omega = robot.omega
    mm.dt = DT

def save_robot_from_motion_model(robot: SwarmRobot) -> None:
    robot.x = float(mm.x)
    robot.y = float(mm.y)
    robot.theta = float(mm.theta)
    robot.v = float(mm.v)
    robot.omega = float(mm.omega)

def get_sensors(walls, landmark_groups):
    wall_readings = mm.get_sensor_readings(walls)
    landmark_measurements = mm.get_landmark_measurements(landmark_groups)
    return wall_readings, landmark_measurements

def make_shared_grid() -> OccupancyGrid:
    return OccupancyGrid(**DEFAULT_GRID)

def make_robot(robot_id: int, pose, shared_grid, walls, landmark_groups) -> SwarmRobot:
    x, y, theta = pose
    nav_state = make_navigation_state(x, y, theta)
    nav_state.grid = shared_grid

    robot = SwarmRobot(robot_id=robot_id, x=x, y=y, theta=theta, nav_state=nav_state)
    load_robot_into_motion_model(robot)
    wall_readings, landmark_measurements = get_sensors(walls, landmark_groups)
    bootstrap_navigation(nav_state, wall_readings, landmark_measurements)
    robot.last_wall_readings = wall_readings
    robot.last_raw_acts = raw_sensor_activations(wall_readings)
    robot.recent_positions.append((robot.x, robot.y))
    return robot

def reset_swarm(walls, landmark_groups):
    shared_grid = make_shared_grid()
    robots = [
        make_robot(i, START_POSES[i], shared_grid, walls, landmark_groups)
        for i in range(N_ROBOTS)
    ]
    reached_goals: set[tuple[float, float]] = set()
    assign_initial_goals(robots, reached_goals)
    return robots, shared_grid, reached_goals

def assigned_goals(robots: list[SwarmRobot], exclude_robot_id: int | None = None):
    out = set()
    for robot in robots:
        if exclude_robot_id is not None and robot.robot_id == exclude_robot_id:
            continue
        if robot.goal is not None:
            out.add(robot.goal)
    return out

def assign_initial_goals(robots: list[SwarmRobot], reached_goals: set[tuple[float, float]]) -> None:
    for robot in robots:
        assign_next_goal(robot, robots, reached_goals)


def assign_next_goal(robot: SwarmRobot, robots: list[SwarmRobot], reached_goals: set[tuple[float, float]]) -> None:
    if robot.goals_reached >= MAX_GOALS_PER_ROBOT:
        robot.goal = None
        return
    
    blocked = reached_goals | assigned_goals(robots, exclude_robot_id=robot.robot_id)
    candidates = [g for g in TASK_GOALS if g not in blocked]

    if not candidates:
        robot.goal = None
        return

    other_assigned = assigned_goals(robots, exclude_robot_id=robot.robot_id)
    spaced_candidates = [
        g for g in candidates
        if all(math.hypot(g[0] - og[0], g[1] - og[1]) >= GOAL_ASSIGNMENT_SPACING for og in other_assigned)
    ]

    usable = spaced_candidates if spaced_candidates else candidates
    robot.goal = min(usable, key=lambda g: math.hypot(robot.x - g[0], robot.y - g[1]))
    robot.best_goal_distance = float("inf")
    robot.no_progress_steps = 0
    robot.previous_goal = robot.goal

def robot_corners(robot: SwarmRobot, margin: float = 0.0):
    return mm.get_robot_corners_at(
        robot.x,
        robot.y,
        robot.theta,
        CAR_LENGTH + 2.0 * margin,
        CAR_WIDTH + 2.0 * margin,
    )

def polygon_axes(poly):
    axes = []
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        edge_x = x2 - x1
        edge_y = y2 - y1
        axis = np.array([-edge_y, edge_x], dtype=float)
        norm = np.linalg.norm(axis)
        if norm > 1e-9:
            axes.append(axis / norm)
    return axes

def project_polygon(poly, axis):
    dots = [p[0] * axis[0] + p[1] * axis[1] for p in poly]
    return min(dots), max(dots)

def polygons_overlap(poly_a, poly_b) -> bool:
    for axis in polygon_axes(poly_a) + polygon_axes(poly_b):
        min_a, max_a = project_polygon(poly_a, axis)
        min_b, max_b = project_polygon(poly_b, axis)
        if max_a < min_b or max_b < min_a:
            return False
    return True

def robot_robot_collision(robot: SwarmRobot, robots: list[SwarmRobot]) -> SwarmRobot | None:
    this_poly = robot_corners(robot, ROBOT_COLLISION_MARGIN)
    for other in robots:
        if other.robot_id == robot.robot_id:
            continue
        if math.hypot(robot.x - other.x, robot.y - other.y) > ROBOT_WARNING_DIST:
            continue
        other_poly = robot_corners(other, ROBOT_COLLISION_MARGIN)
        if polygons_overlap(this_poly, other_poly):
            return other
    return None

def robot_pose_is_wall_safe(rx: float, ry: float, rtheta: float, walls) -> bool:
    collided, _, _ = mm.robot_collides_with_walls(rx, ry, rtheta, CAR_LENGTH, CAR_WIDTH, walls)
    if collided:
        return False

    if hasattr(mm, "_any_edge_crossing"):
        if mm._any_edge_crossing(rx, ry, rtheta, CAR_LENGTH, CAR_WIDTH, walls):
            return False

    return True

def set_robot_pose(robot: SwarmRobot, pose: tuple[float, float, float, float, float]) -> None:
    robot.x, robot.y, robot.theta, robot.v, robot.omega = pose


def push_robot_away_from_other(robot: SwarmRobot, other: SwarmRobot, walls) -> bool:
    start_pose = (robot.x, robot.y, robot.theta, robot.v, robot.omega)

    dx = robot.x - other.x
    dy = robot.y - other.y
    dist = math.hypot(dx, dy)

    if dist < 1e-6:
        base_angle = robot.theta + math.pi
    else:
        base_angle = math.atan2(dy, dx)

    candidate_angles = [
        base_angle,
        base_angle + math.radians(30),
        base_angle - math.radians(30),
        base_angle + math.radians(60),
        base_angle - math.radians(60),
        base_angle + math.radians(90),
        base_angle - math.radians(90),
        base_angle + math.pi,
    ]

    base_push = max(8.0, ROBOT_SEPARATION_DIST - max(dist, 1.0) + 8.0)

    for angle in candidate_angles:
        ux, uy = math.cos(angle), math.sin(angle)
        push = base_push

        for _ in range(SAFE_ROBOT_PUSH_ATTEMPTS):
            cand_x = start_pose[0] + ux * push
            cand_y = start_pose[1] + uy * push
            cand_theta = mm.normalize_angle(start_pose[2] + (0.25 if robot.robot_id % 2 == 0 else -0.25))

            if robot_pose_is_wall_safe(cand_x, cand_y, cand_theta, walls):
                robot.x = cand_x
                robot.y = cand_y
                robot.theta = cand_theta
                robot.v = 0.0
                robot.omega = 0.0
                return True

            push *= 0.5

    set_robot_pose(robot, start_pose)
    return False

def nearest_robot_info(robot: SwarmRobot, robots: list[SwarmRobot]):
    nearest = None
    nearest_dist = 999999.0

    for other in robots:
        if other.robot_id == robot.robot_id:
            continue

        d = math.hypot(robot.x - other.x, robot.y - other.y)
        if d < nearest_dist:
            nearest = other
            nearest_dist = d

    return nearest, nearest_dist

def trigger_recovery(robot: SwarmRobot, reason: str, steps: int = RECOVERY_STEPS) -> None:
    if robot.recovery_cooldown > 0 and robot.recovery_steps == 0:
        return
    robot.recovery_steps = max(robot.recovery_steps, steps)
    robot.recovery_cooldown = RECOVERY_COOLDOWN_STEPS
    robot.last_collision_type = reason
    robot.recent_collision_steps = 0
    robot.recent_positions.clear()
    robot.recent_positions.append((robot.x, robot.y))

def update_recovery_state(robot: SwarmRobot, hit: bool) -> None:
    if robot.recovery_cooldown > 0:
        robot.recovery_cooldown -= 1
    if robot.robot_contact_cooldown > 0:
        robot.robot_contact_cooldown -= 1
    if robot.wall_contact_cooldown > 0:
        robot.wall_contact_cooldown -= 1

    robot.recent_positions.append((robot.x, robot.y))
    if len(robot.recent_positions) > STUCK_WINDOW_SWARM:
        robot.recent_positions.pop(0)

    if hit:
        robot.recent_collision_steps += 1
    else:
        robot.recent_collision_steps = max(0, robot.recent_collision_steps - 1)

    stuck = False
    if len(robot.recent_positions) >= STUCK_WINDOW_SWARM:
        old_x, old_y = robot.recent_positions[0]
        moved = math.hypot(robot.x - old_x, robot.y - old_y)
        stuck = moved < STUCK_DIST_SWARM

    if stuck:
        trigger_recovery(robot, "stuck")
    elif robot.recent_collision_steps >= COLLISION_RECOVERY_THRESHOLD:
        trigger_recovery(robot, "wall")

def apply_robot_robot_separation(robot: SwarmRobot, robots: list[SwarmRobot]) -> None:
    nearest, nearest_dist = nearest_robot_info(robot, robots)
    if nearest is None:
        return

    if nearest_dist < ROBOT_SEPARATION_DIST:
        direction_to_other = math.atan2(nearest.y - robot.y, nearest.x - robot.x)
        relative_angle = mm.normalize_angle(direction_to_other - robot.theta)
        turn_away = -1.0 if relative_angle > 0 else 1.0

        mm.v = min(mm.v, -15.0)
        mm.omega += turn_away * 2.5

    elif nearest_dist < ROBOT_WARNING_DIST:
        direction_to_other = math.atan2(nearest.y - robot.y, nearest.x - robot.x)
        relative_angle = mm.normalize_angle(direction_to_other - robot.theta)
        turn_away = -1.0 if relative_angle > 0 else 1.0
        scale = max(0.25, (nearest_dist - ROBOT_SEPARATION_DIST) / (ROBOT_WARNING_DIST - ROBOT_SEPARATION_DIST))
        mm.v *= scale
        mm.omega += turn_away * 1.0


def closest_obstacle_turn_sign(robot: SwarmRobot, raw_acts: np.ndarray) -> float:
    if raw_acts.size == 0:
        return 1.0 if robot.robot_id % 2 == 0 else -1.0

    closest_idx = int(np.argmax(raw_acts))
    closest_activation = float(raw_acts[closest_idx])
    closest_angle_deg = float(mm.SENSOR_ANGLES_DEG[closest_idx])

    if closest_activation < 0.25:
        return 1.0 if robot.robot_id % 2 == 0 else -1.0
    if closest_angle_deg > 10.0:
        return -1.0
    if closest_angle_deg < -10.0:
        return 1.0
    return 1.0 if robot.robot_id % 2 == 0 else -1.0

def corner_aware_recovery_command(robot: SwarmRobot, raw_acts: np.ndarray) -> tuple[float, float]:
    # Robot-robot recovery: reverse/turn deterministically.
    if robot.last_collision_type == "robot":
        turn = RECOVERY_OMEGA if robot.robot_id % 2 == 0 else -RECOVERY_OMEGA
        return RECOVERY_V, turn

    # No-progress/circling recovery: face current goal and move forward slowly.
    if robot.last_collision_type == "stagnation" and robot.goal is not None:
        desired = math.atan2(robot.goal[1] - robot.y, robot.goal[0] - robot.x)
        err = mm.normalize_angle(desired - robot.theta)
        return 35.0, float(np.clip(2.0 * err, -RECOVERY_OMEGA, RECOVERY_OMEGA))

    turn_sign = closest_obstacle_turn_sign(robot, raw_acts)

    # Treat wall, corner, and stuck similarly because the label can switch during repeated wall/corner contact.
    if robot.last_collision_type in ("wall", "corner", "stuck"):
        if raw_acts.size >= 12:
            front_clear = max(raw_acts[11], raw_acts[0], raw_acts[1]) < 0.45
        else:
            front_clear = False

        # If the front is clear, the robot is likely stuck by its side/corner, so a slow forward motion can help it escape.
        if front_clear:
            return 30.0, turn_sign * 0.8

        # If the front is blocked, moving forward would make it worse. Reverse and rotate away from the closest obstacle.
        return -35.0, turn_sign * 2.2

    # Default recovery fallback.
    elapsed_normal = RECOVERY_STEPS - robot.recovery_steps
    if elapsed_normal < 12:
        return RECOVERY_V, 0.0

    return RECOVERY_V, turn_sign * RECOVERY_OMEGA

def update_goal_progress_state(robot: SwarmRobot, goal_dist_true: float, collision_or_recovery: bool) -> None:
    if robot.goal != robot.previous_goal:
        robot.previous_goal = robot.goal
        robot.best_goal_distance = goal_dist_true
        robot.no_progress_steps = 0
        return

    if goal_dist_true < robot.best_goal_distance - GOAL_PROGRESS_EPS:
        robot.best_goal_distance = goal_dist_true
        robot.no_progress_steps = 0
        return

    if collision_or_recovery:
        robot.no_progress_steps = 0
        return

    robot.no_progress_steps += 1

    if robot.no_progress_steps >= CIRCLING_WINDOW:
        robot.last_collision_type = "stagnation"
        trigger_recovery(robot, "stagnation", steps=STAGNATION_RECOVERY_STEPS)
        robot.no_progress_steps = 0
        robot.best_goal_distance = goal_dist_true

def update_corner_trap_state(robot: SwarmRobot, wall_hit: bool) -> None:
    if not wall_hit:
        if robot.wall_trap_hits > 0:
            robot.wall_trap_hits = max(0, robot.wall_trap_hits - 1)
        return

    current = (robot.x, robot.y)

    if robot.wall_trap_anchor is None:
        robot.wall_trap_anchor = current
        robot.wall_trap_hits = 1
        return

    d = math.hypot(current[0] - robot.wall_trap_anchor[0], current[1] - robot.wall_trap_anchor[1])

    if d < CORNER_TRAP_DIST:
        robot.wall_trap_hits += 1
    else:
        robot.wall_trap_anchor = current
        robot.wall_trap_hits = 1

    if robot.wall_trap_hits >= CORNER_TRAP_HITS:
        trigger_recovery(robot, "corner", steps=CORNER_ESCAPE_STEPS)
        robot.wall_trap_anchor = current
        robot.wall_trap_hits = 0


def update_one_robot(robot: SwarmRobot, robots: list[SwarmRobot], controller, genome, walls, landmark_groups, reached_goals) -> None:
    if robot.goal is None:
        assign_next_goal(robot, robots, reached_goals)
        if robot.goal is None:
            robot.v = 0.0
            robot.omega = 0.0
            return

    load_robot_into_motion_model(robot)
    old_pose = (robot.x, robot.y, robot.theta, robot.v, robot.omega)

    wall_readings_before, _ = get_sensors(walls, landmark_groups)
    raw_acts = raw_sensor_activations(wall_readings_before)
    goal_acts = goal_activations(
        robot.nav_state.est_x,
        robot.nav_state.est_y,
        robot.nav_state.est_theta,
        robot.goal[0],
        robot.goal[1],
    )
    acts = np.concatenate([raw_acts, goal_acts])

    if robot.recovery_steps > 0:
        mm.v, mm.omega = corner_aware_recovery_command(robot, raw_acts)
        robot.recovery_steps -= 1
    else:
        wf_cmd = robot.wall_follower.command(raw_acts)
        if wf_cmd is not None:
            mm.v, mm.omega = wf_cmd
        else:
            out = controller.forward(acts, genome)
            mm.v = float(np.clip(out[0], -1.0, 1.0)) * MAX_V
            mm.omega = float(np.clip(out[1], -1.0, 1.0)) * MAX_OMEGA

    apply_robot_robot_separation(robot, robots)

    wall_hit = mm.update(walls, CAR_LENGTH, CAR_WIDTH)
    if wall_hit:
        if robot.wall_contact_cooldown == 0:
            robot.wall_collisions += 1
            robot.collisions += 1
            robot.wall_contact_cooldown = WALL_COLLISION_COUNT_COOLDOWN
        robot.last_collision_type = "wall"

    update_corner_trap_state(robot, wall_hit)

    save_robot_from_motion_model(robot)
    other_hit = robot_robot_collision(robot, robots)
    robot_hit = other_hit is not None

    if robot_hit:
        robot.x, robot.y, robot.theta, robot.v, robot.omega = old_pose
        push_ok = False
        if other_hit is not None:
            push_ok = push_robot_away_from_other(robot, other_hit, walls)

        if not push_ok:
            robot.v = 0.0
            robot.omega = 0.0

        load_robot_into_motion_model(robot)

        if robot.robot_contact_cooldown == 0:
            robot.robot_collisions += 1
            robot.collisions += 1
            robot.robot_contact_cooldown = ROBOT_COLLISION_COUNT_COOLDOWN

        if other_hit is not None and other_hit.robot_contact_cooldown == 0:
            other_hit.robot_collisions += 1
            other_hit.collisions += 1
            other_hit.robot_contact_cooldown = ROBOT_COLLISION_COUNT_COOLDOWN

        trigger_recovery(robot, "robot")
        if other_hit is not None:
            trigger_recovery(other_hit, "robot", steps=20)

    robot.wall_follower.tick(wall_hit, mm.x, mm.y)

    wall_readings, landmark_measurements = get_sensors(walls, landmark_groups)
    update_navigation(
        state=robot.nav_state,
        wall_sensor_readings=wall_readings,
        landmark_measurements=landmark_measurements,
        v=mm.v,
        omega=mm.omega,
        dt=DT,
    )

    save_robot_from_motion_model(robot)
    update_recovery_state(robot, wall_hit or robot_hit)

    robot.last_wall_readings = wall_readings
    robot.last_raw_acts = raw_sensor_activations(wall_readings)

    robot.trail.append((robot.x, robot.y))
    robot.est_trail.append((robot.nav_state.est_x, robot.nav_state.est_y))
    if len(robot.trail) > TRAIL_LEN:
        robot.trail.pop(0)
    if len(robot.est_trail) > TRAIL_LEN:
        robot.est_trail.pop(0)

    goal_dist_true = math.hypot(robot.x - robot.goal[0], robot.y - robot.goal[1])
    if goal_dist_true < GOAL_RADIUS:
        reached_goals.add(robot.goal)
        robot.goals_reached += 1
        assign_next_goal(robot, robots, reached_goals)
    else:
        update_goal_progress_state(
            robot,
            goal_dist_true,
            collision_or_recovery=(wall_hit or robot_hit or robot.recovery_steps > 0),
        )

def draw_robot(screen, robot: SwarmRobot, colour):
    corners = [world_to_screen(px, py) for px, py in robot_corners(robot)]

    body_colour = PURPLE if robot.recovery_steps > 0 else colour
    pygame.draw.polygon(screen, body_colour, corners)
    pygame.draw.polygon(screen, BLACK, corners, 2)

    sx, sy = world_to_screen(robot.x, robot.y)
    pygame.draw.circle(screen, CYAN, (sx, sy), int(ROBOT_SEPARATION_DIST), 1)

    fx = robot.x + (CAR_LENGTH / 2) * math.cos(robot.theta)
    fy = robot.y + (CAR_LENGTH / 2) * math.sin(robot.theta)
    pygame.draw.line(screen, BLACK, (sx, sy), world_to_screen(fx, fy), 2)

    est_corners = [
        world_to_screen(px, py)
        for px, py in mm.get_robot_corners_at(
            robot.nav_state.est_x,
            robot.nav_state.est_y,
            robot.nav_state.est_theta,
            CAR_LENGTH,
            CAR_WIDTH,
        )
    ]
    pygame.draw.polygon(screen, ORANGE, est_corners, 2)

def draw_trails(screen, robot: SwarmRobot, colour):
    if len(robot.trail) >= 2:
        pts = [world_to_screen(px, py) for px, py in robot.trail]
        pygame.draw.lines(screen, colour, False, pts, 2)

    if len(robot.est_trail) >= 2:
        pts = [world_to_screen(px, py) for px, py in robot.est_trail]
        for i in range(0, len(pts) - 1, 10):
            pygame.draw.line(screen, ORANGE, pts[i], pts[min(i + 5, len(pts) - 1)], 2)

def draw_sensor_rays(screen, robot: SwarmRobot):
    sx, sy = world_to_screen(robot.x, robot.y)
    for i, reading in enumerate(robot.last_wall_readings):
        hx, hy = reading["hit_point"]
        alpha = int(robot.last_raw_acts[i] * 180)
        pygame.draw.line(screen, (alpha, alpha, 200), (sx, sy), world_to_screen(hx, hy), 1)


def save_metrics(robots: list[SwarmRobot], shared_grid: OccupancyGrid, reached_goals, step: int):
    if robots:
        refresh_grid_stats(robots[0].nav_state)
        explored_cells = robots[0].nav_state.explored_cells
    else:
        explored_cells = 0

    metrics = {
        "steps": step,
        "n_robots": len(robots),
        "shared_explored_cells": int(explored_cells),
        "total_collisions": int(sum(r.collisions for r in robots)),
        "wall_collisions": int(sum(r.wall_collisions for r in robots)),
        "robot_robot_collisions": int(sum(r.robot_collisions for r in robots)),
        "unique_goals_reached": int(len(reached_goals)),
        "avg_pose_error": float(np.mean([r.pose_error for r in robots])) if robots else 0.0,
        "per_robot": [
            {
                "robot_id": r.robot_id,
                "collisions": int(r.collisions),
                "wall_collisions": int(r.wall_collisions),
                "robot_robot_collisions": int(r.robot_collisions),
                "goals_reached": int(r.goals_reached),
                "pose_error": float(r.pose_error),
                "recovery_steps_remaining": int(r.recovery_steps),
                "no_progress_steps": int(r.no_progress_steps),
                "wall_trap_hits": int(r.wall_trap_hits),
                "last_collision_type": r.last_collision_type,
                "current_goal": None if r.goal is None else [float(r.goal[0]), float(r.goal[1])],
                "true_goal_distance": None if r.goal is None else float(math.hypot(r.x - r.goal[0], r.y - r.goal[1])),
                "true_pose": [float(r.x), float(r.y), float(r.theta)],
                "estimated_pose": [
                    float(r.nav_state.est_x),
                    float(r.nav_state.est_y),
                    float(r.nav_state.est_theta),
                ],
            }
            for r in robots
        ],
    }

    out_path = BASE_DIR / "swarm_metrics" / "swarm_metrics_latest_genome.json"
    with open(out_path, "w") as fh:
        json.dump(metrics, fh, indent=2)
    print(f"Saved swarm metrics → {out_path}")

def robot_goal_status(robot: SwarmRobot, reached_goals: set[tuple[float, float]]) -> str:
    if robot.goal is not None:
        return f"({robot.goal[0]:.0f},{robot.goal[1]:.0f})"

    if robot.goals_reached >= MAX_GOALS_PER_ROBOT:
        return "max goals reached"

    if len(reached_goals) >= len(TASK_GOALS):
        return "all goals done"

    return "no free goals available"

def draw_info_panel(screen, font, hidden, step, robots, reached_goals) -> None:
    panel_x = MAP_W

    PANEL_BG = (245, 245, 245)
    CARD_BG = (255, 255, 255)
    CARD_BORDER = (210, 210, 210)
    TITLE = (20, 20, 20)
    TEXT = (45, 45, 45)
    MUTED = (95, 95, 95)
    GREEN = (0, 135, 35)
    BLUE = (25, 90, 180)
    ORANGE_UI = (210, 110, 0)
    RED_UI = (190, 20, 20)
    PURPLE_UI = (120, 70, 180)

    pygame.draw.rect(screen, PANEL_BG, (panel_x, 0, PANEL_W, SCREEN_H))
    pygame.draw.line(screen, BLACK, (panel_x, 0), (panel_x, SCREEN_H), 2)

    # Fonts.
    title_font = pygame.font.SysFont(None, 22, bold=True)
    label_font = pygame.font.SysFont(None, 18)
    small_font = pygame.font.SysFont(None, 18)

    total_collisions = sum(r.collisions for r in robots)
    wall_collisions = sum(r.wall_collisions for r in robots)
    robot_collisions = sum(r.robot_collisions for r in robots)
    avg_pose_error = np.mean([r.pose_error for r in robots]) if robots else 0.0
    recovery_active = sum(1 for r in robots if r.recovery_steps > 0)

    if robots:
        refresh_grid_stats(robots[0].nav_state)
        explored = robots[0].nav_state.explored_cells
    else:
        explored = 0

    def draw_card(x, y, w, h):
        pygame.draw.rect(screen, CARD_BG, (x, y, w, h), border_radius=8)
        pygame.draw.rect(screen, CARD_BORDER, (x, y, w, h), 1, border_radius=8)

    def draw_text(text, x, y, colour=TEXT, font_obj=None):
        if font_obj is None:
            font_obj = small_font
        surf = font_obj.render(str(text), True, colour)
        screen.blit(surf, (x, y))

    margin = 12
    gap = 10
    card_w = (PANEL_W - 2 * margin - gap) // 2
    top_y = 14
    top_h = 135

    status_x = panel_x + margin
    coll_x = status_x + card_w + gap

    draw_card(status_x, top_y, card_w, top_h)
    draw_card(coll_x, top_y, card_w, top_h)

    draw_text("SWARM", status_x + 10, top_y + 10, TITLE, title_font)
    draw_text(f"Controller: {N_INPUTS}->{hidden}->{N_OUTPUTS}", status_x + 10, top_y + 38)
    draw_text(f"Step: {step}", status_x + 10, top_y + 58, BLUE)
    draw_text(f"Goals: {len(reached_goals)}/{len(TASK_GOALS)}", status_x + 10, top_y + 78, GREEN, label_font)
    draw_text(f"Explored: {explored}", status_x + 10, top_y + 100)

    draw_text("COLLISIONS", coll_x + 10, top_y + 10, TITLE, title_font)
    draw_text(f"Total: {total_collisions}", coll_x + 10, top_y + 38, RED_UI if total_collisions > 0 else GREEN)
    draw_text(f"Walls: {wall_collisions}", coll_x + 10, top_y + 58, RED_UI if wall_collisions > 0 else GREEN)
    draw_text(f"Car-car: {robot_collisions}", coll_x + 10, top_y + 78, RED_UI if robot_collisions > 0 else GREEN)
    draw_text(f"Recovery: {recovery_active}", coll_x + 10, top_y + 98, ORANGE_UI if recovery_active > 0 else GREEN)
    draw_text(f"Pose err: {avg_pose_error:.1f}", coll_x + 10, top_y + 118, PURPLE_UI)

    section_y = top_y + top_h + 14
    draw_text("ROBOT PROGRESS", panel_x + margin, section_y, TITLE, title_font)

    robot_card_x = panel_x + margin
    robot_card_w = PANEL_W - 2 * margin
    robot_card_h = 132
    y = section_y + 26

    for r in robots:
        draw_card(robot_card_x, y, robot_card_w, robot_card_h)

        goal_text = robot_goal_status(r, reached_goals)
        true_d = 0.0 if r.goal is None else math.hypot(r.x - r.goal[0], r.y - r.goal[1])

        if r.stopped_at_step is None:
            step_text = f"active steps: {r.active_steps}"
            step_colour = BLUE
        else:
            step_text = f"stopped at: {r.stopped_at_step}"
            step_colour = PURPLE_UI

        draw_text(f"R{r.robot_id}", robot_card_x + 10, y + 8, TITLE, title_font)
        draw_text(goal_text, robot_card_x + 48, y + 10, GREEN, label_font)

        draw_text(f"reached: {r.goals_reached}   dist: {true_d:.0f}", robot_card_x + 10, y + 36, TEXT)
        draw_text(step_text, robot_card_x + 10, y + 56, step_colour, label_font)
        draw_text(f"wall: {r.wall_collisions}   car: {r.robot_collisions}", robot_card_x + 10, y + 78, RED_UI if (r.wall_collisions + r.robot_collisions) > 0 else GREEN)
        draw_text(f"pose err: {r.pose_error:.1f}", robot_card_x + 175, y + 78, PURPLE_UI)
        draw_text(f"rec: {r.recovery_steps}   last: {r.last_collision_type}", robot_card_x + 10, y + 100, ORANGE_UI if r.recovery_steps > 0 else MUTED)
        draw_text(f"no_prog: {r.no_progress_steps}   trap: {r.wall_trap_hits}", robot_card_x + 175, y + 100, MUTED)

        y += robot_card_h + 8

def main():
    try:
        genome = np.load(GENOME_FILE)
        print(f"Loaded genome from {GENOME_FILE} ({len(genome)} genes)")
    except FileNotFoundError:
        print(f"ERROR: {GENOME_FILE} not found.")
        sys.exit(1)

    hidden = infer_hidden_size(len(genome))
    controller = FeedforwardController(N_INPUTS, hidden, N_OUTPUTS)
    if hasattr(controller, "reset"):
        controller.reset()

    expected = controller.genome_size
    if len(genome) != expected:
        raise ValueError(
            f"Genome/controller mismatch: genome has {len(genome)} genes, "
            f"but controller {N_INPUTS}->{hidden}->{N_OUTPUTS} expects {expected}."
        )

    print(f"Controller architecture inferred from genome: {N_INPUTS}→{hidden}→{N_OUTPUTS}")

    walls, landmarks, landmark_groups = mp.create_map()
    obstacles = walls + landmarks

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Swarm Intelligence — Faster Genome - Experiment:1")
    font = pygame.font.SysFont(None, 20)
    clock = pygame.time.Clock()

    robots, shared_grid, reached_goals = reset_swarm(walls, landmark_groups)

    step = 0
    running = True
    simulation_complete = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                save_metrics(robots, shared_grid, reached_goals, step)
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    save_metrics(robots, shared_grid, reached_goals, step)
                    running = False
                elif event.key == pygame.K_r:
                    robots, shared_grid, reached_goals = reset_swarm(walls, landmark_groups)
                    if hasattr(controller, "reset"):
                        controller.reset()
                    step = 0
                    simulation_complete = False

        if not simulation_complete:
            for robot in robots:
                was_active = robot.goal is not None
                update_one_robot(robot, robots, controller, genome, walls, landmark_groups, reached_goals)

                if was_active:
                    robot.active_steps += 1

                if robot.goal is None and robot.stopped_at_step is None:
                    robot.stopped_at_step = step
                    robot.stop_reason = robot_goal_status(robot, reached_goals)

            step += 1

            if len(reached_goals) == len(TASK_GOALS):
                simulation_complete = True
                save_metrics(robots, shared_grid, reached_goals, step)
                print("All goals reached. Simulation paused.")

        screen.fill(WHITE)

        shared_grid.draw(
            surface=screen,
            world_to_screen_fn=world_to_screen,
            screen_width=MAP_W,
            screen_height=MAP_H,
            robot_x=CAMERA_X,
            robot_y=CAMERA_Y,
        )

        for seg in obstacles:
            pygame.draw.line(screen, BLACK, world_to_screen(*seg[0]), world_to_screen(*seg[1]), 2)

        for goal in TASK_GOALS:
            color = GREY if goal in reached_goals else RED
            gx, gy = world_to_screen(*goal)
            pygame.draw.circle(screen, color, (gx, gy), int(GOAL_RADIUS), 2)
            pygame.draw.line(screen, color, (gx - 5, gy), (gx + 5, gy), 2)
            pygame.draw.line(screen, color, (gx, gy - 5), (gx, gy + 5), 2)

        for robot, colour in zip(robots, ROBOT_COLOURS):
            draw_sensor_rays(screen, robot)
            draw_trails(screen, robot, colour)
            draw_robot(screen, robot, colour)

            if robot.goal is not None:
                sx, sy = world_to_screen(robot.x, robot.y)
                gx, gy = world_to_screen(*robot.goal)
                pygame.draw.line(screen, colour, (sx, sy), (gx, gy), 1)

        if simulation_complete:
            big_font = pygame.font.SysFont(None, 48)
            msg = big_font.render("ALL GOALS REACHED!", True, (0, 120, 0))
            sub = font.render("Press R to reset or ESC to quit", True, BLACK)

            box_w, box_h = 430, 90
            box_x = MAP_W // 2 - box_w // 2
            box_y = 30

            pygame.draw.rect(screen, (245, 255, 245), (box_x, box_y, box_w, box_h))
            pygame.draw.rect(screen, (0, 120, 0), (box_x, box_y, box_w, box_h), 3)

            screen.blit(msg, (box_x + 35, box_y + 18))
            screen.blit(sub, (box_x + 110, box_y + 58))   

        draw_info_panel(screen, font, hidden, step, robots, reached_goals)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()