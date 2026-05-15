"""
record_demo.py — Play all 6 saved genomes in sequence in one pygame window.

For each genome: reset robot to (0, 0), run for 600 frames (10s at 60 FPS) with
the predetermined goal order, hold a 1-second "DEMO COMPLETE" overlay, then
reset and move to the next genome. Capture the window with an external screen
recorder; this script does not write a video file.

All 6 genomes are 146-element feedforward (15→8→2), so the pipeline is the
same for each.

Outputs on exit:
    demo_all_trajectories.png   2×3 grid, one trajectory per genome
    demo_all_stats.json         per-genome goal timings + collisions

Usage:
    python record_demo.py
"""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame

BASE_DIR = Path(__file__).resolve().parent

try:
    from evo_alg._path_setup import ensure_project_root_on_path
except ModuleNotFoundError:
    sys.path.insert(0, str(BASE_DIR.parent))

import motionmodel as mm
import map as mp
import filter as kf

from ea_tools import FeedforwardController, WallFollowRecovery



GENOMES = [
    "best_genome_goal.npy",
    "best_genome_goal2.npy",
    "best_genome_goal3.npy",
    "best_genome_staged.npy",
    "best_genome_staged_nomovement.npy",
    "best_genome_staged_second_attempt.npy",
]

PER_GENOME_FRAMES = 900
TRANSITION_FRAMES = 60
FPS = 60
DT = 1.0 / FPS

CAR_LENGTH = 24
CAR_WIDTH = 14
MAX_V = 100.0
MAX_OMEGA = 5.0
GOAL_RADIUS = 12.0
SWITCH_RADIUS = GOAL_RADIUS + 8

START = (0.0, 0.0)

GOAL_ORDER = [
    (0,-250),
    (-300,-300),
    (-250,0),
    (-300,300)
]

N_INPUTS = len(mm.SENSOR_ANGLES_DEG) + 3
N_HIDDEN = 8
N_OUTPUTS = 2
EXPECTED_GENOME_SIZE = FeedforwardController(N_INPUTS, N_HIDDEN, N_OUTPUTS).genome_size

SIGMA_R = np.diag([2.0, 2.0, math.radians(2.0) ** 2])
SIGMA_Q = np.diag([4.0, math.radians(3.0) ** 2])
SIGMA_0 = np.diag([0.1, 0.1, math.radians(1.0) ** 2])

# Display (same scheme as watch_goal.py)
WORLD_X_MIN, WORLD_X_MAX = -600.0, 450.0
WORLD_Y_MIN, WORLD_Y_MAX = -400.0, 400.0
WORLD_W = WORLD_X_MAX - WORLD_X_MIN
WORLD_H = WORLD_Y_MAX - WORLD_Y_MIN
SCALE = 0.75
SCREEN_W = int(WORLD_W * SCALE)
SCREEN_H = int(WORLD_H * SCALE)

#define colors
BG, BLACK = (240, 240, 240), (0, 0, 0)
WALL_C = ( 80,  80,  80)
LM_C = (200, 150,   0)
TRAIL_C = ( 70, 130, 180)
EKF_C = (255, 100,   0)
ROBOT_OK  = (  0, 180,   0)
ROBOT_HIT = (220,   0,   0)
GOAL_CURR= (220,   0,   0)
GOAL_DONE= (  0, 150,   0)
GOAL_FUTURE= (160, 160, 160)
START_C = ( 50, 150, 255)


def w2s(wx, wy):
    sx = int((wx - WORLD_X_MIN) / WORLD_W * SCREEN_W)
    sy = int((1.0 - (wy - WORLD_Y_MIN) / WORLD_H) * SCREEN_H)
    return sx, sy


def sensor_acts(wall_readings):
    return np.clip(
        np.array([1.0 - r["distance"] / mm.SENSOR_MAX_RANGE for r in wall_readings]),
        0.0, 1.0,
    )


def goal_acts(ex, ey, eth, gx, gy):
    dx, dy = gx - ex, gy - ey
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    bearing = mm.normalize_angle(math.atan2(dy, dx) - eth)
    return np.array([1.0 - math.exp(-dist / 100.0),
                     math.sin(bearing), math.cos(bearing)])


class Run:
    def __init__(self, name, genome):
        self.name = name
        self.genome = genome
        self.controller = FeedforwardController(N_INPUTS, N_HIDDEN, N_OUTPUTS)
        self.controller.reset()
        mm.x, mm.y, mm.theta = START[0], START[1], 0.0
        mm.v = mm.omega = 0.0
        mm.dt = DT
        self.est_x, self.est_y, self.est_theta = START[0], START[1], 0.0
        self.sigma_mat  = SIGMA_0.copy()
        self.wall_follower = WallFollowRecovery()
        self.trail_true= []
        self.trail_ekf = []
        self.history = []
        self.goal_reaches = []
        self.goal_idx = 0
        self.collisions= 0
        self.frame = 0


def step(run, walls, obstacles, landmark_groups):
    t = run.frame * DT
    mm.dt = DT

    wall_readings = mm.get_sensor_readings(walls)
    raw_acts      = sensor_acts(wall_readings)

    if run.goal_idx < len(GOAL_ORDER):
        gx, gy   = GOAL_ORDER[run.goal_idx]
        goal_a   = goal_acts(run.est_x, run.est_y, run.est_theta, gx, gy)
        nn_input = np.concatenate([raw_acts, goal_a])

        wf_cmd = run.wall_follower.command(raw_acts)
        if wf_cmd is not None:
            mm.v, mm.omega = wf_cmd
        else:
            out = run.controller.forward(nn_input, run.genome)
            mm.v     = float(out[0]) * MAX_V
            mm.omega = float(out[1]) * MAX_OMEGA
    else:
        mm.v = mm.omega = 0.0

    hit = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
    run.wall_follower.tick(hit, mm.x, mm.y)
    if hit:
        run.collisions += 1

    eff_v     = 0.0 if hit else mm.v
    eff_omega = 0.0 if hit else mm.omega
    lm_meas   = mm.get_landmark_measurements(landmark_groups)
    mu_bar, run.sigma_mat = kf.ekf_filter(
        run.est_x, run.est_y, run.est_theta, run.sigma_mat,
        SIGMA_R, SIGMA_Q, eff_v, eff_omega, DT, lm_meas,
    )
    run.est_x, run.est_y, run.est_theta = (float(mu_bar[0, 0]),
                                           float(mu_bar[1, 0]),
                                           float(mu_bar[2, 0]))

    run.trail_true.append((mm.x, mm.y))
    run.trail_ekf.append((run.est_x, run.est_y))
    run.history.append({
        "x": float(mm.x), "y": float(mm.y), "theta": float(mm.theta),
        "est_x": run.est_x, "est_y": run.est_y,
        "goal_idx": int(run.goal_idx), "time": float(t), "hit": bool(hit),
    })

    if run.goal_idx < len(GOAL_ORDER):
        cd = math.hypot(mm.x - GOAL_ORDER[run.goal_idx][0],
                        mm.y - GOAL_ORDER[run.goal_idx][1])
        if cd < SWITCH_RADIUS:
            run.goal_reaches.append({
                "time": float(t),
                "goal_idx": int(run.goal_idx),
                "robot_position": [float(mm.x), float(mm.y)],
            })
            run.goal_idx += 1

    run.frame += 1
    return wall_readings, raw_acts, hit


def render(screen, font, big_font, run, wall_readings, raw_acts, hit,
           walls, landmark_groups, finished, gi, n_total):
    screen.fill(BG)

    for seg in walls:
        pygame.draw.line(screen, WALL_C, w2s(*seg[0]), w2s(*seg[1]), 2)
    for grp in landmark_groups:
        sx, sy = w2s(*grp["center"])
        pygame.draw.circle(screen, LM_C, (sx, sy), 4, 2)

    r_px = max(4, int(GOAL_RADIUS * SCALE))
    for i, (gx, gy) in enumerate(GOAL_ORDER):
        gsx, gsy = w2s(gx, gy)
        if i < run.goal_idx:
            pygame.draw.circle(screen, GOAL_DONE, (gsx, gsy), r_px, 2)
        elif i == run.goal_idx and not finished:
            pygame.draw.circle(screen, GOAL_CURR, (gsx, gsy), r_px, 2)
            pygame.draw.line(screen, GOAL_CURR, (gsx - r_px, gsy), (gsx + r_px, gsy), 2)
            pygame.draw.line(screen, GOAL_CURR, (gsx, gsy - r_px), (gsx, gsy + r_px), 2)
        else:
            pygame.draw.circle(screen, GOAL_FUTURE, (gsx, gsy), r_px, 1)
        lab = font.render(f"G{i + 1}", True, BLACK)
        screen.blit(lab, (gsx + r_px + 3, gsy - 10))

    pygame.draw.circle(screen, START_C, w2s(*START), 5)

    if not finished:
        rsx_t, rsy_t = w2s(mm.x, mm.y)
        for i, r in enumerate(wall_readings):
            hs = w2s(*r["hit_point"])
            alpha = int(float(raw_acts[i]) * 180)
            pygame.draw.line(screen, (alpha, alpha, 200), (rsx_t, rsy_t), hs, 1)

    if len(run.trail_true) >= 2:
        pygame.draw.lines(screen, TRAIL_C, False,
                          [w2s(px, py) for px, py in run.trail_true], 2)
    if len(run.trail_ekf) >= 2:
        pts = [w2s(px, py) for px, py in run.trail_ekf]
        for i in range(0, len(pts) - 1, 6):
            pygame.draw.line(screen, EKF_C, pts[i],
                             pts[min(i + 3, len(pts) - 1)], 1)

    try:
        corners = [w2s(px, py) for px, py in
                   mm.get_robot_corners(CAR_LENGTH, CAR_WIDTH)]
        col = ROBOT_HIT if hit else ROBOT_OK
        pygame.draw.polygon(screen, col, corners)
        pygame.draw.polygon(screen, BLACK, corners, 2)
        rsx, rsy = w2s(mm.x, mm.y)
        fx = mm.x + (CAR_LENGTH / 2) * math.cos(mm.theta)
        fy = mm.y + (CAR_LENGTH / 2) * math.sin(mm.theta)
        pygame.draw.line(screen, BLACK, (rsx, rsy), w2s(fx, fy), 2)
    except (AttributeError, TypeError):
        rsx, rsy = w2s(mm.x, mm.y)
        pygame.draw.circle(screen, ROBOT_OK, (rsx, rsy), 8)

    try:
        est_corners = [w2s(px, py) for px, py in
                       mm.get_robot_corners_at(run.est_x, run.est_y, run.est_theta,
                                               CAR_LENGTH, CAR_WIDTH)]
        pygame.draw.polygon(screen, EKF_C, est_corners, 2)
    except (AttributeError, TypeError):
        pass

    t = run.frame * DT
    target_label = f"G{run.goal_idx + 1}" if run.goal_idx < len(GOAL_ORDER) else "—"
    hud = [
        f"[{gi + 1}/{n_total}]  {run.name}",
        f"t={t:5.2f}s    target={target_label}    "
        f"reached={len(run.goal_reaches)}/{len(GOAL_ORDER)}    col={run.collisions}",
    ]
    for row, line in enumerate(hud):
        screen.blit(font.render(line, True, BLACK), (10, 10 + row * 22))

    if finished:
        overlay = big_font.render("DEMO COMPLETE", True, (50, 50, 50))
        screen.blit(overlay, overlay.get_rect(center=(SCREEN_W // 2, SCREEN_H - 45)))

    pygame.display.flip()


def render_grid_figure(runs, walls, landmark_groups, out_path):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    n = len(runs)
    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 5.5, rows * 4.5))
    axes = np.atleast_1d(axes).flatten()

    cmap = plt.cm.viridis
    n_g  = len(GOAL_ORDER)

    for ax, run in zip(axes, runs):
        for seg in walls:
            ax.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]],
                    'k-', linewidth=1.1, alpha=0.7)
        for grp in landmark_groups:
            cx, cy = grp["center"]
            ax.plot(cx, cy, 'o', color='goldenrod', markersize=3, alpha=0.6)

        if run.history:
            xs = np.array([h["x"] for h in run.history])
            ys = np.array([h["y"] for h in run.history])
            g_idx = np.array([h["goal_idx"] for h in run.history])
            for i in range(n_g + 1):
                mask = g_idx == i
                if not np.any(mask):
                    continue
                color = cmap(i / max(1, n_g - 1)) if i < n_g else 'lightgray'
                ax.plot(xs[mask], ys[mask], '-', color=color, linewidth=2, alpha=0.9)

        reached_ids = {r["goal_idx"] for r in run.goal_reaches}
        for i, (gx, gy) in enumerate(GOAL_ORDER):
            c = 'green' if i in reached_ids else 'red'
            ax.add_patch(Circle((gx, gy), GOAL_RADIUS, color=c, fill=False, linewidth=2))

        ax.plot(START[0], START[1], '*', color='steelblue', markersize=14,
                markeredgecolor='black')
        ax.set_xlim(-600, 450); ax.set_ylim(-400, 400)
        ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.tick_params(labelsize=7)
        ax.set_title(
            f"{run.name}\n{len(run.goal_reaches)}/{n_g} reached"
            f"   collisions={run.collisions}",
            fontsize=9,
        )

    for ax in axes[len(runs):]:
        ax.set_visible(False)

    plt.tight_layout()
    plt.savefig(out_path, dpi=140, bbox_inches='tight')
    plt.close(fig)


def main():
    missing = [g for g in GENOMES if not (BASE_DIR / g).exists()]
    if missing:
        sys.exit(f"Missing genome files (place next to this script): {missing}")

    walls, landmarks, landmark_groups = mp.create_map()
    obstacles = walls + landmarks

    pygame.init()
    screen   = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Demo — 6 genomes sequenced")
    font     = pygame.font.SysFont(None, 22)
    big_font = pygame.font.SysFont(None, 40)
    clock    = pygame.time.Clock()

    runs = []
    quit_early = False

    for gi, name in enumerate(GENOMES):
        if quit_early:
            break

        genome = np.load(BASE_DIR / name)
        if len(genome) != EXPECTED_GENOME_SIZE:
            print(f"[skip] {name}: size {len(genome)} ≠ {EXPECTED_GENOME_SIZE}")
            continue

        run = Run(name, genome)
        runs.append(run)
        print(f"[{gi + 1}/{len(GENOMES)}] {name}")

        # 10 seconds of active simulation
        for _ in range(PER_GENOME_FRAMES):
            for e in pygame.event.get():
                if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN
                                              and e.key == pygame.K_ESCAPE):
                    quit_early = True
                    break
            if quit_early:
                break

            wall_readings, raw_acts, hit = step(run, walls, obstacles, landmark_groups)
            render(screen, font, big_font, run, wall_readings, raw_acts, hit,
                   walls, landmark_groups, finished=False,
                   gi=gi, n_total=len(GENOMES))
            clock.tick(FPS)

        if quit_early:
            break

        # 1-second "DEMO COMPLETE" freeze
        wall_readings = mm.get_sensor_readings(walls)
        raw_acts      = sensor_acts(wall_readings)
        for _ in range(TRANSITION_FRAMES):
            for e in pygame.event.get():
                if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN
                                              and e.key == pygame.K_ESCAPE):
                    quit_early = True
                    break
            if quit_early:
                break
            render(screen, font, big_font, run, wall_readings, raw_acts, False,
                   walls, landmark_groups, finished=True,
                   gi=gi, n_total=len(GENOMES))
            clock.tick(FPS)

        print(f"    reached {len(run.goal_reaches)}/{len(GOAL_ORDER)}   "
              f"collisions={run.collisions}")
        for r in run.goal_reaches:
            print(f"      G{r['goal_idx'] + 1} @ t={r['time']:.2f}s")

    pygame.quit()

    stats = {
        "duration_per_genome_s": PER_GENOME_FRAMES * DT,
        "fps":                   FPS,
        "start":                 list(START),
        "goal_order":            GOAL_ORDER,
        "runs": [
            {
                "genome":        run.name,
                "goals_reached": len(run.goal_reaches),
                "total_goals":   len(GOAL_ORDER),
                "collisions":    int(run.collisions),
                "reaches":       run.goal_reaches,
            }
            for run in runs
        ],
    }
    stats_path = BASE_DIR / "demo_all_stats.json"
    with open(stats_path, "w") as fh:
        json.dump(stats, fh, indent=2)
    print(f"\nSaved → {stats_path}")

    if runs:
        fig_path = BASE_DIR / "demo_all_trajectories.png"
        render_grid_figure(runs, walls, landmark_groups, fig_path)
        print(f"Saved → {fig_path}")


if __name__ == "__main__":
    main()