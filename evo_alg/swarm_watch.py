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
from evo_alg.ea_tools import NeuralController
from evo_alg.ea_navigation import (
    DEFAULT_GRID,
    bootstrap_navigation,
    make_navigation_state,
    mapped_sensor_activations,
    navigation_inputs,
    raw_sensor_activations,
    refresh_grid_stats,
    update_navigation,
)

GENOME_FILE = sys.argv[1] if len(sys.argv) > 1 else str(BASE_DIR / "best_genome.npy")

DT = 1 / 60
CAR_LENGTH = 24
CAR_WIDTH = 14

# Speed of the robot
MAX_V = 100.0
MAX_OMEGA = 5.0

N_ROBOTS = 4
TRAIL_LEN = 250
GOAL_RADIUS = 25.0

N_SENSORS = len(mm.SENSOR_ANGLES_DEG)
N_GOAL_INPUTS = 3
N_INPUTS = N_SENSORS + N_GOAL_INPUTS
N_HIDDEN = 10
N_OUTPUTS = 2

SCREEN_W, SCREEN_H = 1000, 760
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
    (100.0, -100.0, 0.0),
    (-300.0, -120.0, 0.0),
    (80.0, 120.0, math.radians(180.0)),
    (320.0, 80.0, math.radians(-90.0)),
]

ROBOT_COLOURS = [
    (220, 40, 40),
    (40, 140, 220),
    (40, 170, 80),
    (210, 140, 30),
]

WHITE = (255, 255, 255)
BLACK = (20, 20, 20)
GREY = (120, 120, 120)
ORANGE = (255, 100, 0)


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
    goals_reached: int = 0
    last_wall_readings: list = field(default_factory=list)
    last_raw_acts: np.ndarray = field(default_factory=lambda: np.zeros(N_SENSORS))
    trail: list[tuple[float, float]] = field(default_factory=list)
    est_trail: list[tuple[float, float]] = field(default_factory=list)

    @property
    def pose_error(self) -> float:
        return math.hypot(self.x - self.nav_state.est_x, self.y - self.nav_state.est_y)


def world_to_screen(wx: float, wy: float, screen_width: int = SCREEN_W, screen_height: int = SCREEN_H):
    sx = wx - CAMERA_X + screen_width / 2
    sy = -(wy - CAMERA_Y) + screen_height / 2
    return int(sx), int(sy)


def load_robot_into_motion_model(robot: SwarmRobot) -> None:
    # Same as motionmodel.py
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

    # Swarm Intelligence. All robots write to same map
    nav_state.grid = shared_grid

    robot = SwarmRobot(robot_id=robot_id, x=x, y=y, theta=theta, nav_state=nav_state)
    load_robot_into_motion_model(robot)
    wall_readings, landmark_measurements = get_sensors(walls, landmark_groups)
    bootstrap_navigation(nav_state, wall_readings, landmark_measurements)
    robot.last_wall_readings = wall_readings
    robot.last_raw_acts = raw_sensor_activations(wall_readings)
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


def assign_initial_goals(robots: list[SwarmRobot], reached_goals: set[tuple[float, float]]) -> None:
    for i, robot in enumerate(robots):
        robot.goal = TASK_GOALS[i % len(TASK_GOALS)]


def assigned_goals(robots: list[SwarmRobot], exclude_robot_id: int | None = None):
    out = set()
    for robot in robots:
        if exclude_robot_id is not None and robot.robot_id == exclude_robot_id:
            continue
        if robot.goal is not None:
            out.add(robot.goal)
    return out

# Simple cooperative task allocation. The robot chooses the nearest goal that has not already been reached
# and is not currently assigned to another robot to avoids all robots moving to the same place.
def assign_next_goal(robot: SwarmRobot, robots: list[SwarmRobot], reached_goals: set[tuple[float, float]]) -> None:
    blocked = reached_goals | assigned_goals(robots, exclude_robot_id=robot.robot_id)
    candidates = [g for g in TASK_GOALS if g not in blocked]

    if not candidates:
        robot.goal = None
        return

    robot.goal = min(
        candidates,
        key=lambda g: math.hypot(robot.nav_state.est_x - g[0], robot.nav_state.est_y - g[1]),
    )


def update_one_robot(robot: SwarmRobot, robots: list[SwarmRobot], controller, genome, walls, landmark_groups, reached_goals) -> None:
    if robot.goal is None:
        assign_next_goal(robot, robots, reached_goals)
        if robot.goal is None:
            robot.v = 0.0
            robot.omega = 0.0
            return

    load_robot_into_motion_model(robot)

    sensor_acts = mapped_sensor_activations(robot.nav_state)
    acts = navigation_inputs(
        robot.nav_state,
        robot.goal[0],
        robot.goal[1],
        sensor_activations=sensor_acts,
    )

    out = controller.forward(acts, genome)

    mm.v = float(np.clip(out[0], -1.0, 1.0)) * MAX_V
    mm.omega = float(np.clip(out[1], -1.0, 1.0)) * MAX_OMEGA

    # Small safety layer for robot-robot separation. It does not plan paths, only slows down when another robot is too close.
    nearest_other = min(
        (math.hypot(robot.x - other.x, robot.y - other.y) for other in robots if other.robot_id != robot.robot_id),
        default=9999.0,
    )
    if nearest_other < 35.0:
        mm.v *= 0.25
        mm.omega += 1.0 if robot.robot_id % 2 == 0 else -1.0

    hit = mm.update(walls, CAR_LENGTH, CAR_WIDTH)
    if hit:
        robot.collisions += 1

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

    robot.last_wall_readings = wall_readings
    robot.last_raw_acts = raw_sensor_activations(wall_readings)

    robot.trail.append((robot.x, robot.y))
    robot.est_trail.append((robot.nav_state.est_x, robot.nav_state.est_y))
    if len(robot.trail) > TRAIL_LEN:
        robot.trail.pop(0)
    if len(robot.est_trail) > TRAIL_LEN:
        robot.est_trail.pop(0)

    # Goal
    goal_dist = math.hypot(robot.nav_state.est_x - robot.goal[0], robot.nav_state.est_y - robot.goal[1])
    if goal_dist < GOAL_RADIUS:
        reached_goals.add(robot.goal)
        robot.goals_reached += 1
        assign_next_goal(robot, robots, reached_goals)


def draw_robot(screen, robot: SwarmRobot, colour):
    corners = [
        world_to_screen(px, py)
        for px, py in mm.get_robot_corners_at(robot.x, robot.y, robot.theta, CAR_LENGTH, CAR_WIDTH)
    ]
    pygame.draw.polygon(screen, colour, corners)
    pygame.draw.polygon(screen, BLACK, corners, 2)

    sx, sy = world_to_screen(robot.x, robot.y)
    fx = robot.x + (CAR_LENGTH / 2) * math.cos(robot.theta)
    fy = robot.y + (CAR_LENGTH / 2) * math.sin(robot.theta)
    pygame.draw.line(screen, BLACK, (sx, sy), world_to_screen(fx, fy), 2)

    # Estimated pose outline.
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
        "unique_goals_reached": int(len(reached_goals)),
        "avg_pose_error": float(np.mean([r.pose_error for r in robots])) if robots else 0.0,
        "per_robot": [
            {
                "robot_id": r.robot_id,
                "collisions": int(r.collisions),
                "goals_reached": int(r.goals_reached),
                "pose_error": float(r.pose_error),
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

    out_path = BASE_DIR / "swarm_metrics.json"
    with open(out_path, "w") as fh:
        json.dump(metrics, fh, indent=2)
    print(f"Saved swarm metrics → {out_path}")


def main():
    controller = NeuralController(N_INPUTS, N_HIDDEN, N_OUTPUTS)

    try:
        genome = np.load(GENOME_FILE)
        print(f"Loaded genome from {GENOME_FILE} ({len(genome)} genes)")
    except FileNotFoundError:
        print(f"ERROR: {GENOME_FILE} not found. Run python -m evo_alg.ea first.")
        sys.exit(1)

    walls, landmarks, landmark_groups = mp.create_map()
    obstacles = walls + landmarks

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Swarm Intelligence — 4 Cooperative Robots")
    font = pygame.font.SysFont(None, 20)
    clock = pygame.time.Clock()

    robots, shared_grid, reached_goals = reset_swarm(walls, landmark_groups)

    step = 0
    running = True

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
                    step = 0

        for robot in robots:
            update_one_robot(robot, robots, controller, genome, walls, landmark_groups, reached_goals)

        step += 1

        # Draw.
        screen.fill(WHITE)

        shared_grid.draw(
            surface=screen,
            world_to_screen_fn=world_to_screen,
            screen_width=SCREEN_W,
            screen_height=SCREEN_H,
            robot_x=CAMERA_X,
            robot_y=CAMERA_Y,
        )

        for seg in obstacles:
            pygame.draw.line(screen, BLACK, world_to_screen(*seg[0]), world_to_screen(*seg[1]), 2)

        for goal in TASK_GOALS:
            color = (80, 80, 80) if goal in reached_goals else (200, 0, 0)
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

        refresh_grid_stats(robots[0].nav_state)
        total_collisions = sum(r.collisions for r in robots)
        avg_pose_error = np.mean([r.pose_error for r in robots])

        hud = [
            "Swarm mode: 4 robots | shared occupancy grid | cooperative goal allocation | R reset | ESC quit",
            f"Step: {step}  Shared explored cells: {robots[0].nav_state.explored_cells}",
            f"Unique goals reached: {len(reached_goals)}/{len(TASK_GOALS)}  Total collisions: {total_collisions}",
            f"Average pose error: {avg_pose_error:.1f}",
        ]

        for r in robots:
            g = "None" if r.goal is None else f"({r.goal[0]:.0f},{r.goal[1]:.0f})"
            hud.append(
                f"R{r.robot_id}: goal={g} collisions={r.collisions} "
                f"reached={r.goals_reached} pose_err={r.pose_error:.1f}"
            )

        for row, line in enumerate(hud):
            surf = font.render(line, True, BLACK)
            screen.blit(surf, (8, 8 + row * 18))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
