import os
#hide pygame frame for training
if __name__ == "__main__":
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    os.environ["SDL_AUDIODRIVER"] = "dummy"
    os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import collections
import json
import math
import sys
import time
from multiprocessing import Pool, cpu_count  #asked claude to parallelize fitness evaluations, roughly 4x speed up on my machine
from pathlib import Path

import numpy as np

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR.parent))

import motionmodel as mm
import map as mp
import filter as kf
from ea_tools import FeedforwardController as ffc
from ea_tools import WallFollowRecovery as wfr
from ea_tools import NoveltyArchive as na
from ea_tools import EA_new as EA

#simulation hyperparameters
DT = 0.05
CAR_LENGTH = 24
CAR_WIDTH = 14
MAX_V = 100.0
MAX_OMEGA = 5.0

#noise matrices for position estimation
SIGMA_R = np.diag([2.0, 2.0, math.radians(2.0) ** 2])
SIGMA_Q = np.diag([4.0, math.radians(3.0) ** 2])
SIGMA_0 = np.diag([0.1, 0.1, math.radians(1.0) ** 2])

START_X, START_Y = 0.0, 0.0   # default / watch-mode start

START_POSITIONS = [(0.0,0.0),(-200.0,0.0),(200.0,0.0),(0.0,-200.0),(0.0,200.0)]#to help combat trajectory memorization

CURRICULUM_GOALS = [(-300.0,  100.0),(203.0, -189.0),( 94.0,  261.0),(278.0, -203.0),(318.0,43.0)]

# Staged training parameters
SPECIALISTS_PER_GOAL = 7 # top-k genomes kept from each goal training
N_SPECIALIST_GENS = 40 # max generations per each goal run (early stop usually fires first)
N_FINAL_GENS = 100 # generations for the final all-goals run

#early stop when top-k mean fitness exceeds this (goal reliably reached)
SPECIALIST_CONVERGE_THR = 5500.0
# early-stop if best fitness hasn't improved for this many gens
SPECIALIST_STAGNATION = 15

# Final-run fitness aggregation: mean per goal + with penalty for scoring below threshold for a goal (so they dont just learn tonavigate to some goals but not others)
PENALTY_THR = 0.0 # goals scoring below this get penalised
PENALTY_WEIGHT = 0.5 # multiplied by the sum of per-goal shortfalls

GOAL_RADIUS = 12.0 #since the landmark is round, we assume the robot to have reached the goal
EVAL_STEPS = 1200 #how long the simulation runs for accumulating fitness function
MAX_STAGNATION = 300 #stop if stuck for this long

GOAL_BONUS = 5000.0 #large fitness boost to any genome that actually reaches the goal
GOAL_TIME_BONUS = 5000.0 #bonus for reaching goal directly, discourages excessive roaming
PROGRESS_GAIN = 5.0 #reward for moving in the right direction
HEADING_GAIN = 0.5 # per step: reward for velocity component pointing toward goal
SMOOTH_TERMINAL = 0.0   # disabled — no movement bonus in this experiment to avoid weird spinning behaviour
CLOSENESS_BONUS = 300.0 #partial credit at end of episode for goal proximity
COLLISION_PENALTY = 50.0 #disincentivize wall collisions
COLLISION_TERMINATE = 50 #assume car breaks after excessive number of collisions
TERMINATE_PENALTY = 500.0 #large penalty for breaking

#keep the novelty search
NOVELTY_WEIGHT = 0.3


POP_SIZE = 50
MUTATION_RATE = 0.05
MUTATION_SCALE = 0.25
ELITE_COUNT = 2
CROSSOVER_RATE = 0.70
TOURNAMENT_K = 3

N_INPUTS = len(mm.SENSOR_ANGLES_DEG) + 3   # 12 lidar + 3 goal = 15
N_HIDDEN = 8
N_OUTPUTS = 2

walls, landmarks, landmark_groups = mp.create_map()
obstacles = walls + landmarks

#AI implemented, aims to give speed boost by parallelizing fitness evaluations on multiple cores
N_WORKERS = min(cpu_count(), POP_SIZE)


def sensor_acts(wall_readings):
    #converts sensor readings into proximity measures, highest values where wall is closest
    return np.clip(np.array([1.0 - r["distance"] / mm.SENSOR_MAX_RANGE for r in wall_readings]),0.0, 1.0)


def goal_acts(est_x, est_y, est_theta, goal_x, goal_y):
    #enables passing goal relative to robot position; bearing encoded as sin/cos to avoid angle wrap issues
    dx, dy = goal_x - est_x, goal_y - est_y
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    bearing = mm.normalize_angle(math.atan2(dy, dx) - est_theta)
    return np.array([1.0 - math.exp(-dist / 100.0), math.sin(bearing), math.cos(bearing)])


def evaluate_episode(genome, controller, goal_x, goal_y, start_x=START_X, start_y=START_Y):
    controller.reset()
    mm.x = start_x
    mm.y = start_y
    mm.theta = 0.0
    mm.v = 0.0
    mm.omega = 0.0
    mm.dt = DT

    est_x, est_y, est_theta = start_x, start_y, 0.0
    sigma_mat = SIGMA_0.copy()

    wall_follower = wfr()
    initial_d = math.hypot(mm.x - goal_x, mm.y - goal_y)
    best_d = initial_d
    last_progress = 0
    collisions = 0
    objective = 0.0
    speed_sum = turn_sum = 0.0
    n_steps = 0
    goal_reached = False
    steps_used = EVAL_STEPS

    for step in range(EVAL_STEPS):
        mm.dt = DT

        # Sense
        wall_readings = mm.get_sensor_readings(walls)
        raw_acts = sensor_acts(wall_readings)
        goal_a = goal_acts(est_x, est_y, est_theta, goal_x, goal_y)
        nn_input = np.concatenate([raw_acts, goal_a])

        # Motor commands
        wf_cmd = wall_follower.command(raw_acts)
        #failsafe, override command if necessary
        if wf_cmd is not None:
            mm.v, mm.omega = wf_cmd
        else:
            out = controller.forward(nn_input, genome)
            mm.v = float(out[0]) * MAX_V
            mm.omega = float(out[1]) * MAX_OMEGA

        hit = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        wall_follower.tick(hit, mm.x, mm.y)

        # EKF, assumes contact sensors allow detection of hitting wall
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

        # Collision penalty
        if hit:
            objective -= COLLISION_PENALTY
            collisions+= 1
            if collisions >= COLLISION_TERMINATE:
                objective-= TERMINATE_PENALTY
                steps_used = step + 1
                break

        speed_sum += max(0.0, mm.v) / MAX_V # forward-only/de incentivize robot from moving backwards
        turn_sum += abs(mm.omega) / MAX_OMEGA
        n_steps += 1

        curr_d = math.hypot(mm.x - goal_x, mm.y - goal_y)
        if curr_d < best_d - 0.5:
            objective += PROGRESS_GAIN * (best_d - curr_d)
            best_d = curr_d
            last_progress = step

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
        mean_turn  = turn_sum  / n_steps
        objective += SMOOTH_TERMINAL * mean_speed * (1.0 - mean_turn)

    return objective, (float(mm.x), float(mm.y))



def specialist_worker(args):
    genome, goal_x, goal_y = args
    controller = ffc(N_INPUTS, N_HIDDEN, N_OUTPUTS)
    sx, sy = START_POSITIONS[np.random.randint(len(START_POSITIONS))]
    obj, end = evaluate_episode(genome, controller, goal_x, goal_y, sx, sy)
    return obj, end


def final_worker(args):
    genome, all_goals = args
    controller = ffc(N_INPUTS, N_HIDDEN, N_OUTPUTS)
    # Same start position for all goals in one evaluation — consistent test of
    # "from this position, can you reach every goal?"
    sx, sy = START_POSITIONS[np.random.randint(len(START_POSITIONS))]
    objs, ends = [], []
    for gx, gy in all_goals:
        obj, end = evaluate_episode(genome, controller, gx, gy, sx, sy)
        objs.append(obj)
        ends.append(end)
    base = float(np.mean(objs))
    penalty = sum(max(0.0, PENALTY_THR - o) for o in objs)
    fit = base - PENALTY_WEIGHT * penalty
    return fit, np.mean(ends, axis=0), objs

#first, evolve a pool on one goal each
def run_specialist(pool, goal_x, goal_y, goal_idx):
    controller = ffc(N_INPUTS, N_HIDDEN, N_OUTPUTS)
    ea = EA(POP_SIZE, controller.genome_size, controller.random_genome,
            mutation_rate=MUTATION_RATE, mutation_scale=MUTATION_SCALE,
            elite_count=ELITE_COUNT, crossover_rate=CROSSOVER_RATE,
            tournament_k=TOURNAMENT_K, selection='roulette')
    archive = na()

    print(f"\n--- Specialist {goal_idx + 1}/{len(CURRICULUM_GOALS)}: "
          f"goal=({goal_x:.0f}, {goal_y:.0f}) ---")
    t0 = time.time()

    best_so_far = -np.inf
    gens_no_improve = 0
    stop_reason = f"reached {N_SPECIALIST_GENS} gens"

    for gen in range(N_SPECIALIST_GENS):
        gen_t0 = time.time()
        worker_args = [(g, goal_x, goal_y) for g in ea.population]
        results = pool.map(specialist_worker, worker_args)
        objectives = [r[0] for r in results]
        behaviours = [r[1] for r in results]

        novelties = [archive.novelty(b, behaviours) for b in behaviours]
        combined = [o + NOVELTY_WEIGHT * n for o, n in zip(objectives, novelties)]

        for b in behaviours:
            archive.maybe_add(b)

        stats = ea.step(objectives, combined)
        elapsed = time.time() - gen_t0

        # tracking early stop conditions
        top_k_mean = float(np.mean(sorted(objectives, reverse=True)[:SPECIALISTS_PER_GOAL]))#mean fitness of top_k genomes (7 in the case)
        if stats["gen_obj"] > best_so_far + 1.0:
            best_so_far = stats["gen_obj"]
            gens_no_improve = 0
        else:
            gens_no_improve += 1

        print(
            f"  Gen {gen:3d}  best={stats['best_obj']:8.1f}  "
            f"gen={stats['gen_obj']:8.1f}  avg={stats['avg_obj']:7.1f}  "
            f"top{SPECIALISTS_PER_GOAL}_mean={top_k_mean:7.1f}  "
            f"nov={np.mean(novelties):5.1f}  ({elapsed:.1f}s)"
        )

        if top_k_mean >= SPECIALIST_CONVERGE_THR:
            stop_reason = f"converged (top-{SPECIALISTS_PER_GOAL} mean={top_k_mean:.0f})"
            break
        if gens_no_improve >= SPECIALIST_STAGNATION:
            stop_reason = f"stagnated ({SPECIALIST_STAGNATION} gens without improvement)"
            break

    # Re-evaluate the final population once to pick top-k fairly.
    final_results = pool.map(specialist_worker,[(g, goal_x, goal_y) for g in ea.population])
    final_fits = np.array([r[0] for r in final_results])
    top_idx = np.argsort(final_fits)[::-1][:SPECIALISTS_PER_GOAL]
    top_genomes = [ea.population[i].copy() for i in top_idx]

    print(f"  Done in {time.time() - t0:.1f}s | {stop_reason} | "
          f"top-{SPECIALISTS_PER_GOAL} range: "
          f"{final_fits[top_idx[0]]:.1f} – {final_fits[top_idx[-1]]:.1f}")
    return top_genomes


def main():
    controller = ffc(N_INPUTS, N_HIDDEN, N_OUTPUTS)
    print(f"Parallel evaluation: {N_WORKERS} workers ({cpu_count()} cores detected)")

    with Pool(processes=N_WORKERS) as pool:
        #first, run an evolutionary algorithm on each goal separately
        all_specialists = []
        for i, (gx, gy) in enumerate(CURRICULUM_GOALS):
            specialists = run_specialist(pool, gx, gy, i)
            all_specialists.extend(specialists)
        #add some random individuals to the populations to promote diversity
        n_random = POP_SIZE - len(all_specialists)
        seed_pop = all_specialists + [controller.random_genome() for _ in range(n_random)]
        #population is intialized with 35 specialists and 15 random genomes
        ea = EA(POP_SIZE, controller.genome_size, controller.random_genome,
                mutation_rate=MUTATION_RATE, mutation_scale=MUTATION_SCALE,
                elite_count=ELITE_COUNT, crossover_rate=CROSSOVER_RATE,
                tournament_k=TOURNAMENT_K, selection='roulette')
        ea.population = seed_pop
        archive = na()

        print(f"\n{'='*70}")
        print(f"Phase 2: generalist run — "
              f"{len(all_specialists)} specialists + {n_random} random")
        print(f"{'='*70}")
        t0 = time.time()

        for gen in range(N_FINAL_GENS):
            gen_t0 = time.time()
            worker_args = [(g, CURRICULUM_GOALS) for g in ea.population]
            results = pool.map(final_worker, worker_args)
            fitnesses = [r[0] for r in results]
            behaviours = [r[1] for r in results]
            per_goal = [r[2] for r in results]

            novelties = [archive.novelty(b, behaviours) for b in behaviours]
            combined  = [f + NOVELTY_WEIGHT * n for f, n in zip(fitnesses, novelties)]

            for b in behaviours:
                archive.maybe_add(b)

            stats = ea.step(fitnesses, combined)
            elapsed = time.time() - gen_t0

            per_goal_means = np.mean(per_goal, axis=0)
            goal_str = "  ".join(
                f"g{i+1}={v:6.0f}" for i, v in enumerate(per_goal_means)
            )
            print(
                f"Gen {gen:3d}  best={stats['best_obj']:8.1f}  "
                f"gen={stats['gen_obj']:8.1f}  avg={stats['avg_obj']:7.1f}  "
                f"nov={np.mean(novelties):5.1f}  [{goal_str}]  ({elapsed:.1f}s)"
            )

    print(f"\nDone in {time.time() - t0:.1f}s  |  best objective: {ea.best_obj:.2f}")

    genome_path = BASE_DIR / "best_genome_staged_nomovement.npy"
    hist_path   = BASE_DIR / "fitness_history_staged_nomovement.json"
    np.save(genome_path, ea.best_genome)
    with open(hist_path, "w") as fh:
        json.dump(ea.history, fh, indent=2)

if __name__ == "__main__":
    main()
