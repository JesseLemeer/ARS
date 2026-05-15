import json, math, time, os
from pathlib import Path

import numpy as np

from evo_alg.ea_tools import RecurrentController, EA

import motionmodel as mm
import map as mp
from evo_alg.ea_navigation import (
    bootstrap_navigation,
    make_navigation_state,
    navigation_inputs,
    update_navigation,
)
BASE_DIR = Path(__file__).resolve().parent
FITNESS_MODE = "combined"

EVAL_STEPS = 800
DT = 0.05
CAR_LENGTH = 24
CAR_WIDTH = 14
MAX_V = 100.0
MAX_OMEGA = 5.0


POP_SIZE = 60
N_GENERATIONS = 100
MUTATION_RATE = 0.10
MUTATION_SCALE = 0.25
ELITE_COUNT = 4

PROGRESS_GAIN = 4.0 #reward for moving in the right direction

DISPLACEMENT_WINDOW = 30 #steps over which net displacement is measured
DISPLACEMENT_GAIN = 0.4 #reward for actually covering ground, discourages spinning in place

GOAL_RADIUS = 5.0 #robot must come within this distance to count as reaching the goal
GOAL_BONUS = 500.0 #large fitness boost to any genome that actually reaches the goal

TERMINAL_CLOSENESS_BONUS = 300.0 #partial credit at end of episode for goal proximity

COLLISION_PENALTY = 50.0 #disincentivize wall collisions
COLLISION_TERMINATE = 10 #assume car breaks after excessive number of collisions
TERMINATE_PENALTY = 300.0 #large penalty for breaking

WALL_PROXIMITY_PENALTY = 2.0 #per-step penalty for driving close to walls
# Spin penalty: rotating without moving.
SPIN_PENALTY = 3.0 #penalises turning without forward motion

MAX_STAGNATION = 250 #stop if stuck for this long

ACCESSIBLE_LM = [
    [-300, 100], [-312, 145], [278, -203],
    [341, 217], [203, -189],
    [-367, 112], [94, 261], [-241, -238], [318, 43],
]

GOAL_X, GOAL_Y = 278, -203  # fallback single goal
GOALS_PER_EVAL = 4 #number of goals sampled per evaluation episode
TRAINING_GOALS = [tuple(g) for g in ACCESSIBLE_LM]

START_X, START_Y = -480, 337

CURRICULUM_START_RADIUS = 200.0 #initial radius of goals considered reachable from start
CURRICULUM_FRACTION = 0.6 #fraction of training over which curriculum radius expands to full range

STUCK_DIST = 5.0
STUCK_WINDOW = 120

N_SENSORS = len(mm.SENSOR_ANGLES_DEG)
N_GOAL_INPUTS = 3
N_INPUTS = N_SENSORS + N_GOAL_INPUTS
N_HIDDEN = 20
N_OUTPUTS = 2

controller = RecurrentController(N_INPUTS, N_HIDDEN, N_OUTPUTS)

walls, landmarks, landmark_groups = mp.create_map()
obstacles = walls + landmarks

def get_sensors():
    wall_readings = mm.get_sensor_readings(walls)
    landmark_measurements = mm.get_landmark_measurements(landmark_groups)
    return wall_readings, landmark_measurements


def reset_robot():
    mm.x = START_X
    mm.y = START_Y
    mm.theta = mm.v = mm.omega = 0.0
    mm.dt = DT

    nav_state = make_navigation_state(mm.x, mm.y, mm.theta)
    wall_readings, landmark_measurements = get_sensors()
    bootstrap_navigation(nav_state, wall_readings, landmark_measurements)
    return nav_state


def update_after_movement(nav_state):
    wall_readings, landmark_measurements = get_sensors()
    update_navigation(
        state=nav_state,
        wall_sensor_readings=wall_readings,
        landmark_measurements=landmark_measurements,
        v=mm.v,
        omega=mm.omega,
        dt=mm.dt,
    )

def evaluate_combined(genome: np.ndarray, goal_sequence) -> float:
    #runs one episode navigating through a sequence of goals, accumulating fitness
    # Normalize input — allow passing a single (x,y) tuple or a list of goals
    if isinstance(goal_sequence, tuple) and len(goal_sequence) == 2 \
            and not isinstance(goal_sequence[0], (tuple, list)):
        goal_sequence = [goal_sequence]
    goals = list(goal_sequence) or [(GOAL_X, GOAL_Y)]

    controller.reset()
    nav_state = reset_robot()

    total = 0.0
    pos_history: list[tuple[float, float]] = []
    collisions = 0
    goals_reached = 0

    goal_x, goal_y = goals.pop(0)
    initial_d = math.hypot(nav_state.est_x - goal_x, nav_state.est_y - goal_y)
    best_d = initial_d
    last_progress_step = 0
    closeness_accumulator = 0.0

    for step in range(EVAL_STEPS):
        mm.dt = DT
        acts = navigation_inputs(nav_state, goal_x, goal_y)
        sensor_acts = acts[:N_SENSORS]

        out = controller.forward(acts, genome)
        mm.v = float(out[0]) * MAX_V
        mm.omega = float(out[1]) * MAX_OMEGA
        hit = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        update_after_movement(nav_state)

        if hit:
            total -= COLLISION_PENALTY
            collisions += 1
            if collisions >= COLLISION_TERMINATE:
                total -= TERMINATE_PENALTY
                break

        px, py = nav_state.est_x, nav_state.est_y
        pos_history.append((px, py))

        curr_d = math.hypot(px - goal_x, py - goal_y)
        if curr_d < best_d - 0.5:
            total += PROGRESS_GAIN * (best_d - curr_d)
            best_d = curr_d
            last_progress_step = step

        if step - last_progress_step > MAX_STAGNATION:
            break  # silent termination

        if step >= DISPLACEMENT_WINDOW:
            ox, oy = pos_history[-DISPLACEMENT_WINDOW - 1]
            displacement = math.hypot(px - ox, py - oy)
            total += DISPLACEMENT_GAIN * displacement

        total -= WALL_PROXIMITY_PENALTY * float(np.max(sensor_acts))

        spin = abs(mm.omega) / MAX_OMEGA
        move = abs(mm.v) / MAX_V
        total -= SPIN_PENALTY * spin * (1.0 - move)

        if curr_d < GOAL_RADIUS:
            total += GOAL_BONUS
            goals_reached += 1
            closeness_accumulator += 1.0  # full credit for this goal
            if not goals:
                break
            goal_x, goal_y = goals.pop(0)
            initial_d = math.hypot(px - goal_x, py - goal_y)
            best_d = initial_d
            last_progress_step = step

        if step >= STUCK_WINDOW:
            ox, oy = pos_history[-STUCK_WINDOW]
            if math.hypot(px - ox, py - oy) < STUCK_DIST:
                break

    if initial_d > 1.0:
        closeness_accumulator += max(0.0, 1.0 - best_d / initial_d)
    total += TERMINAL_CLOSENESS_BONUS * closeness_accumulator

    return total


MODES = {"combined": evaluate_combined}



def select_eval_goals(generation: int):
    #curriculum: gradually expand the reachable goal radius so the robot learns nearby goals first
    if not TRAINING_GOALS:
        return [(GOAL_X, GOAL_Y)]

    distances = [math.hypot(g[0] - START_X, g[1] - START_Y) for g in TRAINING_GOALS]
    max_dist = max(distances)

    progress = min(1.0, generation / max(1, int(N_GENERATIONS * CURRICULUM_FRACTION)))
    radius = CURRICULUM_START_RADIUS + progress * (max_dist - CURRICULUM_START_RADIUS)

    eligible = [g for g, d in zip(TRAINING_GOALS, distances) if d <= radius]
    if not eligible:
        eligible = list(TRAINING_GOALS)

    n_goals = max(1, min(GOALS_PER_EVAL, len(eligible)))
    idx = np.random.choice(len(eligible), n_goals, replace=False)
    return [eligible[i] for i in idx]


def evaluate(genome, eval_goals):
    return float(MODES[FITNESS_MODE](genome, eval_goals))

def main() -> None:
    #turns off the pygame frame so I don't have to watch the entire evolution process (takes hours, better for battery)
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

    print("=" * 64)
    print(f"  Evolutionary Robotics — mode: {FITNESS_MODE.upper()}")
    print(f"  Controller   : RECURRENT (Elman)  {N_INPUTS}→{N_HIDDEN}→{N_OUTPUTS}")
    print("=" * 64)
    print(f"  Genome size  : {controller.genome_size} floats")
    print(f"  Population   : {POP_SIZE}  ×  {N_GENERATIONS} generations")
    print(f"  Eval steps   : {EVAL_STEPS}  (DT={DT}s)")
    print(f"  Goals/eval   : {GOALS_PER_EVAL} from {len(TRAINING_GOALS)} (curriculum)")
    print(f"  Progress     : new-minimum-only (detours free)")
    print(f"  Stagnation   : abort after {MAX_STAGNATION} steps without progress")
    print(f"  Collision    : −{COLLISION_PENALTY}/step, abort@{COLLISION_TERMINATE}")
    print("=" * 64)

    ea = EA(
        pop_size=POP_SIZE,
        genome_size=controller.genome_size,
        mutation_rate=MUTATION_RATE,
        mutation_scale=MUTATION_SCALE,
        elite_count=ELITE_COUNT,
        init_genome_fn=controller.random_genome,   # smart recurrent init
    )

    t0 = time.time()

    for gen in range(N_GENERATIONS):
        eval_goals = select_eval_goals(gen)
        gen_t0 = time.time()
        fitnesses = [evaluate(g, eval_goals) for g in ea.population]
        stats = ea.evolve(fitnesses)
        elapsed = time.time() - gen_t0

        print(
            f"Gen {gen:3d}/{N_GENERATIONS}  "
            f"best={stats['best']:8.2f}  avg={stats['avg']:7.2f}  "
            f"worst={stats['worst']:7.2f}  ({elapsed:.1f}s)"
        )

    total_time = time.time() - t0
    print(f"\nDone in {total_time:.1f}s  |  all-time best: {ea.best_fitness:.2f}")

    genome_path = BASE_DIR / "best_genome.npy"
    history_path = BASE_DIR / "fitness_history.json"
    np.save(genome_path, ea.best_genome)
    with open(history_path, "w") as fh:
        json.dump(ea.fitness_history, fh, indent=2)

    print(f"Saved → {genome_path.name}  |  {history_path.name}")
    print("Run   python -m evo_alg.watch_best   to visualise.")



if __name__ == "__main__":
    main()