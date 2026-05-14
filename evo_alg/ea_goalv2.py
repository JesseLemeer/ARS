import os

# Must be set BEFORE motionmodel imports pygame — but only when running headless
if __name__ == "__main__":
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    os.environ["SDL_AUDIODRIVER"] = "dummy"
    os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import collections
import json
import math
import sys
import time
from multiprocessing import Pool, cpu_count
from pathlib import Path

import numpy as np

# This file lives in <project>/evo_alg/. motionmodel.py, map.py, filter.py
# live one level up. Put the project root on sys.path so they import.
BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR.parent))

import motionmodel as mm
import map as mp
import filter as kf
from ea_tools import FeedforwardController as ffc
from ea_tools import WallFollowRecovery as wfr
from ea_tools import NoveltyArchive as na
from ea_tools import Curriculum as crc
from ea_tools import EA_new as EA

DT         = 0.05
CAR_LENGTH = 24
CAR_WIDTH  = 14
MAX_V      = 100.0
MAX_OMEGA  = 5.0

SIGMA_R = np.diag([2.0, 2.0, math.radians(2.0) ** 2])
SIGMA_Q = np.diag([4.0, math.radians(3.0) ** 2])
SIGMA_0 = np.diag([0.1, 0.1, math.radians(1.0) ** 2])

START_X, START_Y = 0.0, 0.0   # default / watch-mode start

START_POSITIONS = [
    (   0.0,    0.0),
    (-200.0,    0.0),
    ( 200.0,    0.0),
    (   0.0, -200.0),
    (   0.0,  200.0),
]

CURRICULUM_GOALS = [
    (-300.0,  100.0),
    (203.0, -189.0),
    ( 94.0,  261.0),
    (278.0, -203.0),
    (318.0,   43.0),
]
INIT_POOL_SIZE  = 1
POOL_GROW_EVERY = 12
POOL_MASTER_THR = 3500

GOAL_X, GOAL_Y = CURRICULUM_GOALS[1]

GOAL_RADIUS    = 12.0
EVAL_STEPS     = 1200
MAX_STAGNATION = 300

GOAL_BONUS      = 5000.0
GOAL_TIME_BONUS = 5000.0
PROGRESS_GAIN   = 5.0
HEADING_GAIN    = 0.5    # per step: reward for velocity component pointing toward goal
SMOOTH_TERMINAL   = 25.0
CLOSENESS_BONUS = 300.0
COLLISION_PENALTY = 50.0
COLLISION_TERMINATE = 50
TERMINATE_PENALTY = 500.0

NOVELTY_WEIGHT = 0.3
NOVELTY_K = 15
ARCHIVE_CAP = 300
ARCHIVE_PROB = 0.05

POP_SIZE = 50
N_GENERATIONS = 100
K_EVAL_GOALS = 3
MUTATION_RATE = 0.05
MUTATION_SCALE = 0.25
ELITE_COUNT = 2
CROSSOVER_RATE = 0.70
TOURNAMENT_K = 3
#neural controller with 15 inputs, 8 hidden states, 2 outputs (v and omega)
N_INPUTS  = len(mm.SENSOR_ANGLES_DEG) + 3
N_HIDDEN  = 8
N_OUTPUTS = 2

WALLFOLLOW_TRIGGER = 4
WALLFOLLOW_WINDOW = 25
WALLFOLLOW_STEPS = 80

# World objects, created once. Each subprocess will re-create these on import
# (separate process memory) — that's fine and the cost is negligible.
walls, landmarks, landmark_groups = mp.create_map()
obstacles = walls + landmarks

# Set to None or 1 if you want to disable parallelism for debugging.
N_WORKERS = min(cpu_count(), POP_SIZE)


def sensor_acts(wall_readings):
    return np.clip(np.array([1.0 - r["distance"] / mm.SENSOR_MAX_RANGE for r in wall_readings]),0.0, 1.0)


def goal_acts(est_x, est_y, est_theta, goal_x, goal_y):
    dx, dy = goal_x - est_x, goal_y - est_y
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    bearing = mm.normalize_angle(math.atan2(dy, dx) - est_theta)
    return np.array([1.0 - math.exp(-dist / 100.0), math.sin(bearing),math.cos(bearing)])


def evaluate_episode(genome, controller, goal_x, goal_y, start_x=START_X, start_y=START_Y):
    controller.reset()
    mm.x, mm.y, mm.theta = start_x, start_y, 0.0
    mm.v = mm.omega = 0.0
    mm.dt = DT

    est_x, est_y, est_theta = start_x, start_y, 0.0
    sigma_mat = SIGMA_0.copy()

    wall_follower = wfr()
    initial_d = math.hypot(mm.x - goal_x, mm.y - goal_y)
    best_d = initial_d
    last_progress = 0
    collisions = 0
    objective = 0.0

    speed_sum = 0.0
    turn_sum = 0.0
    n_steps = 0

    goal_reached = False
    steps_used   = EVAL_STEPS

    for step in range(EVAL_STEPS):
        mm.dt = DT

        wall_readings = mm.get_sensor_readings(walls)
        raw_acts = sensor_acts(wall_readings)
        goal_a = goal_acts(est_x, est_y, est_theta, goal_x, goal_y)
        nn_input = np.concatenate([raw_acts, goal_a])

        wf_cmd = wall_follower.command(raw_acts)
        if wf_cmd is not None:
            mm.v, mm.omega = wf_cmd
        else:
            out = controller.forward(nn_input, genome)
            mm.v     = float(out[0]) * MAX_V
            mm.omega = float(out[1]) * MAX_OMEGA

        hit = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        wall_follower.tick(hit, mm.x, mm.y)

        eff_v = 0.0 if hit else mm.v
        eff_omega = 0.0 if hit else mm.omega
        lm_meas = mm.get_landmark_measurements(landmark_groups)
        mu_bar, sigma_mat = kf.ekf_filter(
            est_x, est_y, est_theta, sigma_mat,
            SIGMA_R, SIGMA_Q, eff_v, eff_omega, DT, lm_meas,
        )
        est_x = float(mu_bar[0, 0])
        est_y = float(mu_bar[1, 0])
        est_theta = float(mu_bar[2, 0])

        if hit:
            objective  -= COLLISION_PENALTY
            collisions += 1
            if collisions >= COLLISION_TERMINATE:
                objective -= TERMINATE_PENALTY
                steps_used = step + 1
                break

        speed_sum += max(0.0, mm.v) / MAX_V 
        turn_sum  += abs(mm.omega) / MAX_OMEGA
        n_steps   += 1

        curr_d = math.hypot(mm.x - goal_x, mm.y - goal_y)
        if curr_d < best_d - 0.5:
            objective += PROGRESS_GAIN * (best_d - curr_d)
            best_d = curr_d
            last_progress = step

        # Per-step reward for heading toward the goal
        if curr_d > GOAL_RADIUS:
            goal_dir = math.atan2(goal_y - mm.y, goal_x - mm.x)
            align = math.cos(mm.theta - goal_dir)
            objective += HEADING_GAIN * max(0.0, align) * abs(mm.v) / MAX_V

        if step - last_progress > MAX_STAGNATION:
            steps_used = step + 1
            break

        if curr_d < GOAL_RADIUS:
            goal_reached = True
            steps_used   = step + 1
            break
    if goal_reached:
        objective += GOAL_BONUS
        objective += GOAL_TIME_BONUS * max(0.0, 1.0 - steps_used / EVAL_STEPS)
    else:
        if initial_d > 1.0:
            objective += CLOSENESS_BONUS * max(0.0, 1.0 - best_d / initial_d)

    if n_steps > 0:
        mean_speed = speed_sum / n_steps
        mean_turn  = turn_sum / n_steps
        objective += SMOOTH_TERMINAL * mean_speed * (1.0 - mean_turn)

    endpoint = (float(mm.x), float(mm.y))
    return objective, endpoint


def evaluate_slate(genome, controller, slate, start_x=START_X, start_y=START_Y):
    objs, ends = [], []
    for (gx, gy) in slate:
        obj, end = evaluate_episode(genome, controller, gx, gy, start_x, start_y)
        objs.append(obj)
        ends.append(end)
    return float(np.min(objs)), np.mean(ends, axis=0)


def _eval_worker(args):
    genome, slate = args
    controller = ffc(N_INPUTS, N_HIDDEN, N_OUTPUTS)
    sx, sy = START_POSITIONS[np.random.randint(len(START_POSITIONS))]
    return evaluate_slate(genome, controller, slate, sx, sy)


def main():
    controller = ffc(N_INPUTS, N_HIDDEN, N_OUTPUTS)

    ea = EA(POP_SIZE, controller.genome_size, controller.random_genome,
            mutation_rate=MUTATION_RATE, mutation_scale=MUTATION_SCALE,
            elite_count=ELITE_COUNT, crossover_rate=CROSSOVER_RATE,
            tournament_k=TOURNAMENT_K)
    archive = na()
    curriculum = crc(CURRICULUM_GOALS, INIT_POOL_SIZE,
                            POOL_GROW_EVERY, POOL_MASTER_THR)

    print(f"Parallel evaluation: {N_WORKERS} workers (detected {cpu_count()} cores)")

    t0 = time.time()
    with Pool(processes=N_WORKERS) as pool:
        for gen in range(N_GENERATIONS):
            gen_t0 = time.time()
            slate  = curriculum.slate(K_EVAL_GOALS)

            # Parallel fitness evaluation — replaces the per-genome for loop.
            # Each worker gets (genome, slate) and returns (objective, endpoint).
            worker_args = [(g, slate) for g in ea.population]
            results = pool.map(_eval_worker, worker_args)
            objectives = [r[0] for r in results]
            behaviours = [r[1] for r in results]

            # Novelty + EA step run in the main process (cheap relative to sim).
            novelties = [archive.novelty(b, behaviours) for b in behaviours]
            combined = [o + NOVELTY_WEIGHT * n for o, n in zip(objectives, novelties)]

            for b in behaviours:
                archive.maybe_add(b)

            stats = ea.step(objectives, combined)
            grew  = curriculum.maybe_grow(stats["gen_obj"])

            elapsed = time.time() - gen_t0
            slate_str = ", ".join(f"({gx:.0f},{gy:.0f})" for gx, gy in slate)
            print(
                f"Gen {gen:3d}  slate=[{slate_str}]  "
                f"best={stats['best_obj']:8.1f}  "
                f"gen={stats['gen_obj']:8.1f}  "
                f"avg={stats['avg_obj']:7.1f}  "
                f"nov={np.mean(novelties):5.1f}  "
                f"pool={len(curriculum.pool)}"
                + ("  +GOAL" if grew else "")
                + f"  ({elapsed:.1f}s)"
            )

    print(f"\nDone in {time.time() - t0:.1f}s  |  best objective: {ea.best_obj:.2f}")

    genome_path = BASE_DIR / "best_genome_goal3.npy"
    hist_path   = BASE_DIR / "fitness_history_goal3.json"
    np.save(genome_path, ea.best_genome)
    with open(hist_path, "w") as fh:
        json.dump(ea.history, fh, indent=2)
    print(f"Saved → {genome_path.name}  |  {hist_path.name}")


if __name__ == "__main__":
    main()