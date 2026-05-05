import sys
import math
import numpy as np
import pygame

import motionmodel as mm
import map as mp
from ea_tools import NeuralController, EA

# ── Config ─────────────────────────────────────────────────────────────────────
GENOME_FILE  = sys.argv[1] if len(sys.argv) > 1 else "best_genome.npy"
DT           = 1 / 60      # match 60-fps display
CAR_LENGTH   = 24
CAR_WIDTH    = 14
MAX_V        = 100.0
MAX_OMEGA    = 5.0
TRAIL_LEN    = 300

N_SENSORS = len(mm.SENSOR_ANGLES_DEG)
N_HIDDEN  = 10
N_OUTPUTS = 2

SCREEN_W, SCREEN_H = 800, 600

# Colours
WHITE      = (255, 255, 255)
BLACK      = (  0,   0,   0)
BLUE       = ( 70, 130, 180)
GREEN      = (  0, 200,   0)
RED        = (200,   0,   0)
ORANGE     = (255,  77,   0)
LIGHT_GRAY = (210, 210, 230)


def sensor_activations(walls) -> np.ndarray:
    readings = mm.get_sensor_readings(walls)
    acts = np.array([1.0 - r["distance"] / mm.SENSOR_MAX_RANGE for r in readings])
    return np.clip(acts, 0.0, 1.0)


def main():
    # ── Load genome ────────────────────────────────────────────────────────────
    try:
        genome = np.load(GENOME_FILE)
        print(f"Loaded genome from {GENOME_FILE}  ({len(genome)} genes)")
    except FileNotFoundError:
        print(f"ERROR: {GENOME_FILE} not found. Run  python main_ea.py  first.")
        sys.exit(1)

    controller = NeuralController(N_SENSORS, N_HIDDEN, N_OUTPUTS)

    # ── Map ────────────────────────────────────────────────────────────────────
    walls, landmarks, landmark_groups = mp.create_map()
    obstacles = walls + landmarks

    # ── Pygame setup ───────────────────────────────────────────────────────────
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Best Evolved Controller")
    clock  = pygame.font.SysFont(None, 20)
    font   = pygame.font.SysFont(None, 20)

    # ── Reset robot ────────────────────────────────────────────────────────────
    mm.x = mm.y = mm.theta = mm.v = mm.omega = 0.0
    trail: list[tuple[float, float]] = []

    step          = 0
    total_fitness = 0.0
    collisions    = 0

    running = True
    py_clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:         # R → reset robot
                    mm.x = mm.y = mm.theta = mm.v = mm.omega = 0.0
                    trail.clear()
                    step = 0
                    total_fitness = 0.0
                    collisions    = 0
                elif event.key == pygame.K_ESCAPE:
                    running = False

        mm.dt = DT

        # Sense → think → act
        acts      = sensor_activations(walls)
        out       = controller.forward(acts, genome)
        mm.v      = float(out[0]) * MAX_V
        mm.omega  = float(out[1]) * MAX_OMEGA
        hit       = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)

        if hit:
            collisions += 1

        # Fitness bookkeeping
        V     = abs(mm.v)    / MAX_V
        delta = abs(mm.omega) / MAX_OMEGA
        i_max = float(np.max(acts))
        phi   = max(0.0, V * (1.0 - math.sqrt(delta)) * (1.0 - i_max))
        total_fitness += phi
        step  += 1

        # Trail
        trail.append((mm.x, mm.y))
        if len(trail) > TRAIL_LEN:
            trail.pop(0)

        # ── Draw ───────────────────────────────────────────────────────────────
        screen.fill(WHITE)

        # Walls & landmarks
        for seg in obstacles:
            s = mm.world_to_screen(seg[0][0], seg[0][1], SCREEN_W, SCREEN_H)
            e = mm.world_to_screen(seg[1][0], seg[1][1], SCREEN_W, SCREEN_H)
            pygame.draw.line(screen, BLACK, s, e, 2)

        # Sensor rays
        sx, sy = mm.world_to_screen(mm.x, mm.y, SCREEN_W, SCREEN_H)
        for i, r in enumerate(mm.get_sensor_readings(walls)):
            hx, hy = r["hit_point"]
            hs, _ = mm.world_to_screen(hx, hy, SCREEN_W, SCREEN_H), 0
            hs     = mm.world_to_screen(hx, hy, SCREEN_W, SCREEN_H)
            alpha  = int(acts[i] * 180)        # brighter = closer obstacle
            pygame.draw.line(screen, (alpha, alpha, 200), (sx, sy), hs, 1)

        # Trail
        if len(trail) >= 2:
            pts = [mm.world_to_screen(px, py, SCREEN_W, SCREEN_H) for px, py in trail]
            pygame.draw.lines(screen, BLUE, False, pts, 2)

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

        # HUD
        hud = [
            f"Evolved controller  |  press R to reset",
            f"Step: {step:5d}   Collisions: {collisions}",
            f"v={mm.v:6.1f}  ω={mm.omega:5.2f}",
            f"Fitness (step): {phi:.4f}   Total: {total_fitness:.1f}",
            f"Max sensor act: {i_max:.3f}",
        ]
        for row, line in enumerate(hud):
            surf = font.render(line, True, (20, 20, 20))
            screen.blit(surf, (8, 8 + row * 18))

        pygame.display.flip()
        py_clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()