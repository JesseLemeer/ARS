import math
import sys
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
import filter as kf

try:
    from evo_alg.ea_goal import (
        FeedforwardController, WallFollowRecovery,
        N_INPUTS, N_HIDDEN, N_OUTPUTS,
        START_X, START_Y, GOAL_X, GOAL_Y, GOAL_RADIUS,
        SIGMA_R, SIGMA_Q, SIGMA_0,
        DT, CAR_LENGTH, CAR_WIDTH, MAX_V, MAX_OMEGA,
        walls, landmarks, landmark_groups, obstacles,
        _sensor_acts, _goal_acts,
    )
except ModuleNotFoundError:
    from ea_goal import (
        FeedforwardController, WallFollowRecovery,
        N_INPUTS, N_HIDDEN, N_OUTPUTS,
        START_X, START_Y, GOAL_X, GOAL_Y, GOAL_RADIUS,
        SIGMA_R, SIGMA_Q, SIGMA_0,
        DT, CAR_LENGTH, CAR_WIDTH, MAX_V, MAX_OMEGA,
        walls, landmarks, landmark_groups, obstacles,
        _sensor_acts, _goal_acts,
    )

# ── Config ─────────────────────────────────────────────────────────────────────
GENOME_FILE = sys.argv[1] if len(sys.argv) > 1 else str(BASE_DIR / "best_genome_goal.npy")

# World bounds (match map.py)
WORLD_X_MIN, WORLD_X_MAX = -600.0, 450.0
WORLD_Y_MIN, WORLD_Y_MAX = -400.0, 400.0
WORLD_W = WORLD_X_MAX - WORLD_X_MIN   # 1050
WORLD_H = WORLD_Y_MAX - WORLD_Y_MIN   # 800

SCALE    = 0.75
SCREEN_W = int(WORLD_W * SCALE)   # 787
SCREEN_H = int(WORLD_H * SCALE)   # 600

TRAIL_LEN = 400

# Colours
BG          = (230, 230, 230)
BLACK       = (  0,   0,   0)
WALL_COLOR  = ( 80,  80,  80)
LM_COLOR    = (200, 150,   0)
TRAIL_TRUE  = ( 70, 130, 180)
TRAIL_EKF   = (255, 100,   0)
ROBOT_OK    = (  0, 180,   0)
ROBOT_HIT   = (220,   0,   0)
EKF_COLOR   = (255, 100,   0)
GOAL_COLOR  = (220,   0,   0)
START_COLOR = ( 50, 150, 255)


def w2s(wx: float, wy: float) -> tuple:
    """World coordinates → fixed-camera screen pixels."""
    sx = int((wx - WORLD_X_MIN) / WORLD_W * SCREEN_W)
    sy = int((1.0 - (wy - WORLD_Y_MIN) / WORLD_H) * SCREEN_H)
    return sx, sy


def main() -> None:
    controller = FeedforwardController(N_INPUTS, N_HIDDEN, N_OUTPUTS)
    try:
        genome = np.load(GENOME_FILE)
        print(f"Loaded genome ({len(genome)} genes) from {GENOME_FILE}")
    except FileNotFoundError:
        print(f"ERROR: {GENOME_FILE} not found — run python ea_goal.py first.")
        sys.exit(1)

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Goal Navigation — ea_goal best genome  (R=reset  ESC=quit)")
    font = pygame.font.SysFont(None, 20)

    def reset():
        controller.reset()
        mm.x, mm.y, mm.theta = START_X, START_Y, 0.0
        mm.v = mm.omega = 0.0
        mm.dt = DT
        return (START_X, START_Y, 0.0, SIGMA_0.copy(),
                WallFollowRecovery(), [], [], 0, 0,
                math.hypot(START_X - GOAL_X, START_Y - GOAL_Y))

    (est_x, est_y, est_theta, sigma_mat,
     wall_follower, trail_true, trail_ekf,
     step, collisions, best_d) = reset()

    curr_d  = math.hypot(START_X - GOAL_X, START_Y - GOAL_Y)
    running = True
    clock   = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    (est_x, est_y, est_theta, sigma_mat,
                     wall_follower, trail_true, trail_ekf,
                     step, collisions, best_d) = reset()
                    curr_d = math.hypot(START_X - GOAL_X, START_Y - GOAL_Y)
                elif event.key == pygame.K_ESCAPE:
                    running = False

        mm.dt = DT

        # Sense
        wall_readings = mm.get_sensor_readings(walls)
        raw_acts      = _sensor_acts(wall_readings)
        goal_acts_v   = _goal_acts(est_x, est_y, est_theta, GOAL_X, GOAL_Y)
        nn_input      = np.concatenate([raw_acts, goal_acts_v])

        # Motor commands (wall-follow takes priority when active)
        wf_cmd = wall_follower.command(raw_acts)
        if wf_cmd is not None:
            mm.v, mm.omega = wf_cmd
        else:
            out      = controller.forward(nn_input, genome)
            mm.v     = float(out[0]) * MAX_V
            mm.omega = float(out[1]) * MAX_OMEGA

        # Physics
        hit = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        wall_follower.tick(hit)
        if hit:
            collisions += 1

        # EKF
        eff_v     = 0.0 if hit else mm.v
        eff_omega = 0.0 if hit else mm.omega
        lm_meas   = mm.get_landmark_measurements(landmark_groups)
        mu_bar, sigma_mat = kf.ekf_filter(
            est_x, est_y, est_theta, sigma_mat,
            SIGMA_R, SIGMA_Q, eff_v, eff_omega, DT, lm_meas,
        )
        est_x     = float(mu_bar[0, 0])
        est_y     = float(mu_bar[1, 0])
        est_theta = float(mu_bar[2, 0])

        curr_d = math.hypot(mm.x - GOAL_X, mm.y - GOAL_Y)
        best_d = min(best_d, curr_d)
        step  += 1

        # Trails
        trail_true.append((mm.x, mm.y))
        if len(trail_true) > TRAIL_LEN:
            trail_true.pop(0)
        trail_ekf.append((est_x, est_y))
        if len(trail_ekf) > TRAIL_LEN:
            trail_ekf.pop(0)

        # ── Draw ──────────────────────────────────────────────────────────────
        screen.fill(BG)

        # Walls
        for seg in walls:
            pygame.draw.line(screen, WALL_COLOR, w2s(*seg[0]), w2s(*seg[1]), 2)

        # Landmarks (amber circles)
        for grp in landmark_groups:
            sx, sy = w2s(*grp["center"])
            pygame.draw.circle(screen, LM_COLOR, (sx, sy), 5, 2)

        # Goal (red crosshair)
        gsx, gsy = w2s(GOAL_X, GOAL_Y)
        r_px = max(4, int(GOAL_RADIUS * SCALE))
        pygame.draw.circle(screen, GOAL_COLOR, (gsx, gsy), r_px, 2)
        pygame.draw.line(screen, GOAL_COLOR, (gsx - r_px, gsy), (gsx + r_px, gsy), 2)
        pygame.draw.line(screen, GOAL_COLOR, (gsx, gsy - r_px), (gsx, gsy + r_px), 2)

        # Start marker (blue dot)
        pygame.draw.circle(screen, START_COLOR, w2s(START_X, START_Y), 5)

        # Sensor rays
        rsx, rsy = w2s(mm.x, mm.y)
        for i, r in enumerate(wall_readings):
            hs    = w2s(*r["hit_point"])
            alpha = int(float(raw_acts[i]) * 180)
            pygame.draw.line(screen, (alpha, alpha, 200), (rsx, rsy), hs, 1)

        # True-pose trail (blue)
        if len(trail_true) >= 2:
            pygame.draw.lines(screen, TRAIL_TRUE, False,
                              [w2s(px, py) for px, py in trail_true], 2)

        # EKF trail (orange dashed)
        if len(trail_ekf) >= 2:
            pts = [w2s(px, py) for px, py in trail_ekf]
            for i in range(0, len(pts) - 1, 6):
                pygame.draw.line(screen, TRAIL_EKF, pts[i], pts[min(i+3, len(pts)-1)], 1)

        # True robot body (green / red on hit)
        color   = ROBOT_HIT if hit else ROBOT_OK
        corners = [w2s(px, py) for px, py in mm.get_robot_corners(CAR_LENGTH, CAR_WIDTH)]
        pygame.draw.polygon(screen, color, corners)
        pygame.draw.polygon(screen, BLACK, corners, 2)
        fx = mm.x + (CAR_LENGTH / 2) * math.cos(mm.theta)
        fy = mm.y + (CAR_LENGTH / 2) * math.sin(mm.theta)
        pygame.draw.line(screen, BLACK, (rsx, rsy), w2s(fx, fy), 2)

        # EKF estimated pose (orange outline)
        est_corners = [
            w2s(px, py)
            for px, py in mm.get_robot_corners_at(
                est_x, est_y, est_theta, CAR_LENGTH, CAR_WIDTH
            )
        ]
        pygame.draw.polygon(screen, EKF_COLOR, est_corners, 2)

        # HUD
        pose_err = math.hypot(mm.x - est_x, mm.y - est_y)
        hud = [
            "ea_goal best genome  |  green=true  orange=EKF  |  R=reset  ESC=quit",
            f"Step: {step:5d}   Collisions: {collisions}"
            + ("  [WALL-FOLLOW]" if wall_follower.active else ""),
            f"v={mm.v:6.1f}  omega={mm.omega:5.2f}",
            f"True:  ({mm.x:6.1f}, {mm.y:6.1f})  theta={math.degrees(mm.theta)%360:5.1f}deg",
            f"EKF:   ({est_x:6.1f}, {est_y:6.1f})  theta={math.degrees(est_theta)%360:5.1f}deg",
            f"Pose error: {pose_err:5.1f}",
            f"Goal distance (true): {curr_d:6.1f}   best: {best_d:6.1f}",
        ]
        for row, line in enumerate(hud):
            surf = font.render(line, True, (20, 20, 20))
            screen.blit(surf, (8, 8 + row * 18))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
