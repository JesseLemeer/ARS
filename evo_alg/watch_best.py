import sys
import math
from pathlib import Path

import numpy as np
import pygame

BASE_DIR = Path(__file__).resolve().parent

try:
    from evo_alg._path_setup import ensure_project_root_on_path
except ModuleNotFoundError:
    from _path_setup import ensure_project_root_on_path

ensure_project_root_on_path(__file__)

import motionmodel as mm
import map as mp
from evo_alg.ea import GOAL_RADIUS, GOAL_X as DEFAULT_GOAL_X, GOAL_Y as DEFAULT_GOAL_Y
from evo_alg.ea_tools import NeuralController, RecurrentController
from evo_alg.ea_navigation import bootstrap_navigation, make_navigation_state, mapped_sensor_activations, navigation_inputs, update_navigation, raw_sensor_activations

# ── Config ─────────────────────────────────────────────────────────────────────
GENOME_FILE  = sys.argv[1] if len(sys.argv) > 1 else str(BASE_DIR / "best_genome.npy")
GOAL_X = float(sys.argv[2]) if len(sys.argv) > 3 else DEFAULT_GOAL_X
GOAL_Y = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_GOAL_Y
DT = 1 / 60
CAR_LENGTH  = 24
CAR_WIDTH = 14
MAX_V = 100.0
MAX_OMEGA = 5.0
TRAIL_LEN = 300

N_SENSORS = len(mm.SENSOR_ANGLES_DEG)
N_GOAL_INPUTS = 3
N_INPUTS = N_SENSORS + N_GOAL_INPUTS
N_HIDDEN1 = 20
N_HIDDEN2 = 12
N_OUTPUTS = 2

SCREEN_W, SCREEN_H = 800, 600

# Colours
WHITE = (255, 255, 255)
BLACK = (  0,   0,   0)
BLUE = ( 70, 130, 180)
GREEN = (  0, 200,   0)
RED = (200,   0,   0)
ORANGE = (255,  77,   0)


def get_sensors(walls, landmark_groups):
    wall_readings = mm.get_sensor_readings(walls)
    landmark_measurements = mm.get_landmark_measurements(landmark_groups)
    return wall_readings, landmark_measurements


def main():
    controller = RecurrentController(N_INPUTS, 20, N_OUTPUTS)
    #loads genome
    try:
        genome = np.load(GENOME_FILE)
        print(f"Loaded genome from {GENOME_FILE}  ({len(genome)} genes)")
    except FileNotFoundError:
        print(f"ERROR: {GENOME_FILE} not found. Run python -m evo_alg.ea first.")
        sys.exit(1)

    walls, landmarks, landmark_groups = mp.create_map()
    obstacles = walls + landmarks

    #pygame setup
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Best Evolved Controller")
    font   = pygame.font.SysFont(None, 20)
    #reset robot start position
    def reset_robot():
        controller.reset()
        mm.x = -380
        mm.y = 337
        mm.theta = mm.v = mm.omega = 0.0
        mm.dt = DT

        nav_state = make_navigation_state(mm.x, mm.y, mm.theta)
        wall_readings, landmark_measurements = get_sensors(walls, landmark_groups)
        bootstrap_navigation(nav_state, wall_readings, landmark_measurements)
        return nav_state

    nav_state = reset_robot()
    trail: list[tuple[float, float]] = []
    est_trail: list[tuple[float, float]] = []

    step = 0
    total_fitness = 0.0
    collisions = 0

    running = True
    py_clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:#allow reset without restarting the whole simulation
                    nav_state = reset_robot()
                    trail.clear()
                    est_trail.clear()
                    step = 0
                    total_fitness = 0.0
                    collisions    = 0
                elif event.key == pygame.K_ESCAPE:
                    running = False

        mm.dt = DT

        
        sensor_acts = mapped_sensor_activations(nav_state)
        acts = navigation_inputs(nav_state, GOAL_X, GOAL_Y, sensor_activations=sensor_acts)
        out = controller.forward(acts, genome)
        mm.v = float(out[0]) * MAX_V
        mm.omega = float(out[1]) * MAX_OMEGA
        hit = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        wall_readings, landmark_measurements = get_sensors(walls, landmark_groups)
        update_navigation(
            state=nav_state,
            wall_sensor_readings=wall_readings,
            landmark_measurements=landmark_measurements,
            v=mm.v,
            omega=mm.omega,
            dt=mm.dt,
        )
        raw_acts = raw_sensor_activations(wall_readings)

        if hit:
            collisions += 1

        V = abs(mm.v)/ MAX_V
        delta = abs(mm.omega) / MAX_OMEGA
        i_max = float(np.max(sensor_acts))
        phi = max(0.0, V * (1.0 - math.sqrt(delta)) * (1.0 - i_max))
        total_fitness += phi
        step  += 1

        # Trail
        trail.append((mm.x, mm.y))
        if len(trail) > TRAIL_LEN:
            trail.pop(0)
        est_trail.append((nav_state.est_x, nav_state.est_y))
        if len(est_trail) > TRAIL_LEN:
            est_trail.pop(0)

        screen.fill(WHITE)

        nav_state.grid.draw(
            surface=screen,
            world_to_screen_fn=mm.world_to_screen,
            screen_width=SCREEN_W,
            screen_height=SCREEN_H,
            robot_x=mm.x,
            robot_y=mm.y,
        )

        for seg in obstacles:
            s = mm.world_to_screen(seg[0][0], seg[0][1], SCREEN_W, SCREEN_H)
            e = mm.world_to_screen(seg[1][0], seg[1][1], SCREEN_W, SCREEN_H)
            pygame.draw.line(screen, BLACK, s, e, 2)

        # Sensor rays
        sx, sy = mm.world_to_screen(mm.x, mm.y, SCREEN_W, SCREEN_H)
        for i, r in enumerate(wall_readings):
            hx, hy = r["hit_point"]
            hs = mm.world_to_screen(hx, hy, SCREEN_W, SCREEN_H)
            alpha = int(raw_acts[i] * 180)# brighter = closer obstacle
            pygame.draw.line(screen, (alpha, alpha, 200), (sx, sy), hs, 1)

        #Mark goal
        goal_sx, goal_sy = mm.world_to_screen(GOAL_X, GOAL_Y, SCREEN_W, SCREEN_H)
        pygame.draw.circle(screen, RED, (goal_sx, goal_sy), int(GOAL_RADIUS), 2)
        pygame.draw.line(screen, RED, (goal_sx - int(GOAL_RADIUS), goal_sy), (goal_sx + int(GOAL_RADIUS), goal_sy), 2)
        pygame.draw.line(screen, RED, (goal_sx, goal_sy - int(GOAL_RADIUS)), (goal_sx, goal_sy + int(GOAL_RADIUS)), 2)

        # Trail
        if len(trail) >= 2:
            pts = [mm.world_to_screen(px, py, SCREEN_W, SCREEN_H) for px, py in trail]
            pygame.draw.lines(screen, BLUE, False, pts, 2)
        if len(est_trail) >= 2:
            pts = [mm.world_to_screen(px, py, SCREEN_W, SCREEN_H) for px, py in est_trail]
            for i in range(0, len(pts) - 1, 8):
                pygame.draw.line(screen, ORANGE, pts[i], pts[min(i + 4, len(pts) - 1)], 2)

        # Robot body
        color = RED if hit else GREEN
        corners = [mm.world_to_screen(px, py, SCREEN_W, SCREEN_H)
                   for px, py in mm.get_robot_corners(CAR_LENGTH, CAR_WIDTH)]
        pygame.draw.polygon(screen, color, corners)
        pygame.draw.polygon(screen, BLACK, corners, 2)

        # Heading arrow
        fx = mm.x + (CAR_LENGTH / 2) * math.cos(mm.theta)
        fy = mm.y + (CAR_LENGTH / 2) * math.sin(mm.theta)
        fsx, fsy = mm.world_to_screen(fx, fy, SCREEN_W, SCREEN_H)
        pygame.draw.line(screen, BLACK, (sx, sy), (fsx, fsy), 2)

        #Estimated pose from EKF-SLAM
        est_sx, est_sy = mm.world_to_screen(nav_state.est_x, nav_state.est_y, SCREEN_W, SCREEN_H)
        est_corners = [
            mm.world_to_screen(px, py, SCREEN_W, SCREEN_H)
            for px, py in mm.get_robot_corners_at(
                nav_state.est_x,
                nav_state.est_y,
                nav_state.est_theta,
                CAR_LENGTH,
                CAR_WIDTH,
            )
        ]
        pygame.draw.polygon(screen, ORANGE, est_corners, 2)
        est_fx = nav_state.est_x + (CAR_LENGTH / 2) * math.cos(nav_state.est_theta)
        est_fy = nav_state.est_y + (CAR_LENGTH / 2) * math.sin(nav_state.est_theta)
        est_fsx, est_fsy = mm.world_to_screen(est_fx, est_fy, SCREEN_W, SCREEN_H)
        pygame.draw.line(screen, ORANGE, (est_sx, est_sy), (est_fsx, est_fsy), 2)

        # HUD
        pose_error = math.hypot(mm.x - nav_state.est_x, mm.y - nav_state.est_y)
        goal_distance = math.hypot(nav_state.est_x - GOAL_X, nav_state.est_y - GOAL_Y)
        hud = [
            f"Evolved controller  |  mapped input  |  press R to reset",
            f"Step: {step:5d}   Collisions: {collisions}",
            f"v={mm.v:6.1f}  ω={mm.omega:5.2f}",
            f"Fitness (step): {phi:.4f}   Total: {total_fitness:.1f}",
            f"Goal: x={GOAL_X:7.1f} y={GOAL_Y:7.1f}   distance={goal_distance:.1f}",
            f"True: x={mm.x:7.1f} y={mm.y:7.1f} θ={math.degrees(mm.theta) % 360:6.1f}°",
            f"SLAM: x={nav_state.est_x:7.1f} y={nav_state.est_y:7.1f} θ={math.degrees(nav_state.est_theta) % 360:6.1f}°",
            f"Pose error: {pose_error:.1f}   Explored cells: {nav_state.explored_cells}",
            f"Landmarks in SLAM: {len(nav_state.landmark_index)}   Max mapped act: {i_max:.3f}",
        ]
        for row, line in enumerate(hud):
            surf = font.render(line, True, (20, 20, 20))
            screen.blit(surf, (8, 8 + row * 18))

        pygame.display.flip()
        py_clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
