"""
ea_goal.py — Point-to-point navigation: (0, 0) → landmark (278, -203)
=======================================================================
Minimal EA for learning to drive to a single goal using:
  - EKF localisation in a known map (bumper correction on collision)
  - 12 raw lidar + 3 goal-bearing inputs → 15→8→2 feedforward network
  - Fitness: Floreano smooth-driving + coverage + progress + goal bonus
  - Large per-collision penalty

Run:
    cd ARS/evo_alg
    python ea_goal.py
"""

import collections, json, math, os, time
from pathlib import Path

import numpy as np

BASE_DIR = Path(__file__).resolve().parent

try:
    from evo_alg._path_setup import ensure_project_root_on_path
except ModuleNotFoundError:
    from _path_setup import ensure_project_root_on_path

ensure_project_root_on_path(__file__)

import motionmodel as mm
import map as mp
import filter as kf
from ea_tools import EA_goal, FeedforwardController, WallFollowRecovery
DT         = 0.05
CAR_LENGTH = 24
CAR_WIDTH  = 14
MAX_V      = 100.0
MAX_OMEGA  = 5.0

SIGMA_R = np.diag([2.0, 2.0, math.radians(2.0) ** 2])
SIGMA_Q = np.diag([4.0, math.radians(3.0) ** 2])
SIGMA_0 = np.diag([0.1, 0.1, math.radians(1.0) ** 2])

START_X, START_Y = -200.0, 0.0

# 5 reachable landmarks for goal curriculum (all in open corridors/areas)
CURRICULUM_GOALS = [
    (278.0, -203.0),   # bottom-right dead-end corridor
    ( 94.0,  261.0),   # top area above vertical corridor
    (100.0,  100.0),   # upper-right open area
    (318.0,   43.0),   # right-side dead-end road
    (203.0, -189.0),   # bottom-right area
]
GOAL_X, GOAL_Y  = -250,0#CURRICULUM_GOALS[0]   # default (used by watch_goal.py)
GENS_PER_GOAL   = 20                     # generations before rotating to next goal

GOAL_RADIUS      = 12.0
EVAL_STEPS       = 1200
MAX_STAGNATION   = 300

GOAL_BONUS          = 2000.0   # dominant — reaching the goal is the objective
PROGRESS_GAIN       = 5.0      # reward per unit of new closest-distance improvement
FLOREANO_GAIN       = 1.5      # per-step: speed × (1−turn) × clearance
COVERAGE_GAIN       = 0.5      # per new coarse grid cell entered
COV_CELL_SIZE       = 25.0
CLOSENESS_BONUS     = 300.0    # terminal partial-credit multiplier
COLLISION_PENALTY   = 150.0    # per wall contact
COLLISION_TERMINATE = 8
TERMINATE_PENALTY   = 500.0

POP_SIZE       = 50
N_GENERATIONS  = 100
MUTATION_RATE  = 0.12
MUTATION_SCALE = 0.20
ELITE_COUNT    = 4
CROSSOVER_RATE = 0.70
TOURNAMENT_K   = 3

N_INPUTS  = len(mm.SENSOR_ANGLES_DEG) + 3   # 12 lidar + 3 goal = 15
N_HIDDEN  = 8
N_OUTPUTS = 2
WALLFOLLOW_TRIGGER = 4
WALLFOLLOW_WINDOW  = 25
WALLFOLLOW_STEPS   = 80

walls, landmarks, landmark_groups = mp.create_map()
obstacles = walls + landmarks


def _sensor_acts(wall_readings) -> np.ndarray:
    """Raw lidar distances → [0, 1] activations (1 = touching wall)."""
    return np.clip(
        np.array([1.0 - r["distance"] / mm.SENSOR_MAX_RANGE for r in wall_readings]),
        0.0, 1.0,
    )


def _goal_acts(est_x: float, est_y: float, est_theta: float,
               goal_x: float, goal_y: float) -> np.ndarray:
    """
    Goal inputs from EKF *estimated* pose (controller never sees true position).
    Returns [distance_activation, sin(bearing), cos(bearing)].
    """
    dx, dy = goal_x - est_x, goal_y - est_y
    dist   = math.hypot(dx, dy)
    if dist < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    bearing = mm.normalize_angle(math.atan2(dy, dx) - est_theta)
    return np.array([
        1.0 - math.exp(-dist / 100.0),
        math.sin(bearing),
        math.cos(bearing),
    ])


def _cov_cell(x: float, y: float) -> tuple:
    return (int(math.floor(x / COV_CELL_SIZE)), int(math.floor(y / COV_CELL_SIZE)))

def evaluate(genome: np.ndarray, controller: FeedforwardController,
             goal_x: float = GOAL_X, goal_y: float = GOAL_Y) -> float:
    """
    Evaluate one genome. goal_x/goal_y default to CURRICULUM_GOALS[0] but are
    set by main() to rotate through the curriculum every GENS_PER_GOAL generations.
    """
    controller.reset()
    mm.x, mm.y, mm.theta = START_X, START_Y, 0.0
    mm.v = mm.omega = 0.0
    mm.dt = DT

    est_x, est_y, est_theta = START_X, START_Y, 0.0
    sigma_mat = SIGMA_0.copy()

    wall_follower = WallFollowRecovery()
    initial_d     = math.hypot(mm.x - goal_x, mm.y - goal_y)
    best_d        = initial_d

    visited       = set()
    last_progress = 0
    total         = 0.0
    collisions    = 0

    for step in range(EVAL_STEPS):
        mm.dt = DT

        # Sense
        wall_readings = mm.get_sensor_readings(walls)
        raw_acts      = _sensor_acts(wall_readings)
        goal_acts     = _goal_acts(est_x, est_y, est_theta, goal_x, goal_y)
        nn_input      = np.concatenate([raw_acts, goal_acts])

        # Motor commands
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

        # EKF (bumper zeros velocity prediction on contact)
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

        # Collision penalty
        if hit:
            total -= COLLISION_PENALTY
            collisions += 1
            if collisions >= COLLISION_TERMINATE:
                total -= TERMINATE_PENALTY
                break

        # Floreano smooth-driving reward
        speed     = abs(mm.v) / MAX_V
        turning   = abs(mm.omega) / MAX_OMEGA
        clearance = 1.0 - float(np.max(raw_acts))
        total    += FLOREANO_GAIN * speed * (1.0 - turning) * clearance

        # Coverage
        cell = _cov_cell(mm.x, mm.y)
        if cell not in visited:
            visited.add(cell)
            total += COVERAGE_GAIN

        # Progress toward goal (true position — external evaluator)
        curr_d = math.hypot(mm.x - goal_x, mm.y - goal_y)
        if curr_d < best_d - 0.5:
            total        += PROGRESS_GAIN * (best_d - curr_d)
            best_d        = curr_d
            last_progress = step

        if step - last_progress > MAX_STAGNATION:
            break

        # Goal reached
        if curr_d < GOAL_RADIUS:
            total += GOAL_BONUS
            break

    # Terminal closeness bonus
    if initial_d > 1.0:
        total += CLOSENESS_BONUS * max(0.0, 1.0 - best_d / initial_d)

    return total



def main() -> None:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

    controller = FeedforwardController(N_INPUTS, N_HIDDEN, N_OUTPUTS)

    n_goals = len(CURRICULUM_GOALS)
    print("=" * 72)
    print("  Goal Navigation EA — curriculum training")
    print(f"  Start      : ({START_X}, {START_Y})")
    print(f"  Goals      : {n_goals} landmarks, {GENS_PER_GOAL} gens each "
          f"→ {n_goals * GENS_PER_GOAL} gens / cycle")
    print(f"  Controller : {N_INPUTS}→{N_HIDDEN}→{N_OUTPUTS}  ({controller.genome_size} params)")
    print(f"  Population : {POP_SIZE} × {N_GENERATIONS} generations total")
    print(f"  Eval steps : {EVAL_STEPS}  (DT={DT}s → {EVAL_STEPS*DT:.0f}s / episode)")
    print("=" * 72)

    ea = EA(
        pop_size       = POP_SIZE,
        genome_size    = controller.genome_size,
        mutation_rate  = MUTATION_RATE,
        mutation_scale = MUTATION_SCALE,
        elite_count    = ELITE_COUNT,
        crossover_rate = CROSSOVER_RATE,
        tournament_k   = TOURNAMENT_K,
        init_fn        = controller.random_genome,
    )

    t0 = time.time()
    for gen in range(N_GENERATIONS):
        # Rotate goal every GENS_PER_GOAL generations
        goal_idx      = (gen // GENS_PER_GOAL) % n_goals
        g_x, g_y      = CURRICULUM_GOALS[goal_idx]

        gen_t0    = time.time()
        fitnesses = [evaluate(gn, controller, g_x, g_y) for gn in ea.population]
        stats     = ea.evolve(fitnesses)
        elapsed   = time.time() - gen_t0

        print(
            f"Gen {gen:3d}/{N_GENERATIONS}  "
            f"goal=({g_x:5.0f},{g_y:5.0f})  "
            f"best={stats['best_alltime']:8.2f}  "
            f"gen={stats['best_gen']:8.2f}  "
            f"avg={stats['avg']:7.2f}  "
            f"({elapsed:.1f}s)"
        )

    total_time = time.time() - t0
    print(f"\nDone in {total_time:.1f}s  |  all-time best: {ea.best_fitness:.2f}")

    genome_path  = BASE_DIR / "best_genome_goal.npy"
    history_path = BASE_DIR / "fitness_history_goal.json"
    np.save(genome_path, ea.best_genome)
    with open(history_path, "w") as fh:
        json.dump(ea.fitness_history, fh, indent=2)
    print(f"Saved → {genome_path.name}  |  {history_path.name}")
    print("Visualise: python watch_goal.py")


main()
