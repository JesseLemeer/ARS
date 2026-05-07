import json, math, time, os
from pathlib import Path

import numpy as np

BASE_DIR = Path(__file__).resolve().parent

try:
    from evo_alg._path_setup import ensure_project_root_on_path
except ModuleNotFoundError:
    from _path_setup import ensure_project_root_on_path

ensure_project_root_on_path(__file__)

from evo_alg.ea_tools import NeuralController, EA

import motionmodel as mm
import map as mp
from evo_alg.ea_navigation import bootstrap_navigation, make_navigation_state, navigation_inputs, update_navigation


FITNESS_MODE = "combined" # options are "floreano","coverage","goal"
#floreano does the same as in the slides, coverage promotes exploration, goal promotes going from A to B

EVAL_STEPS = 800
DT = 0.05
CAR_LENGTH = 24
CAR_WIDTH = 14
MAX_V = 100.0
MAX_OMEGA = 5.0

POP_SIZE = 20
N_GENERATIONS = 10
MUTATION_RATE = 0.10
MUTATION_SCALE = 0.30
ELITE_COUNT = 2

#Coverage bonus for exploration
GRID_CELL = 60.0
COVERAGE_BONUS_PER_CELL = 5.0
MAP_COVERAGE_BONUS_PER_CELL = 0.25

#For reference to different goals
LANDMARKS = [
        [-300, 0], [100, 100], [-312, 145], [278, -203],
        [-87, 91], [341, 217], [-156, -74], [203, -189],
        [-367, 112], [94, 261], [-241, -238], [318, 43],
        [-300, 100]
    ]

ACCESSIBLE_LM = [
        [-300, 100], [-312, 145], [278, -203],
        [341, 217], [203, -189],
        [-367, 112], [94, 261], [-241, -238], [318, 43]
]

#Default goal used when a single target is needed.
GOAL_X, GOAL_Y = 278, -203

GOAL_RADIUS = 5.0          # "reached" threshold
GOAL_BONUS = 200.0         # one-time bonus for reaching goal

GOALS_PER_EVAL = 3
#If you want one goal, making this an empty []
TRAINING_GOALS = [tuple(goal) for goal in ACCESSIBLE_LM]

#Early-stop in case robot did not move more than STUCK_DIST for STUCK_WINDOW
STUCK_DIST = 5.0
STUCK_WINDOW = 60 

N_SENSORS = len(mm.SENSOR_ANGLES_DEG)
N_GOAL_INPUTS = 3
N_INPUTS = N_SENSORS + N_GOAL_INPUTS
N_HIDDEN  = 10
N_OUTPUTS = 2

controller = NeuralController(N_INPUTS, N_HIDDEN, N_OUTPUTS)
walls, landmarks, landmark_groups = mp.create_map()
obstacles = walls + landmarks


def get_sensors():
    wall_readings = mm.get_sensor_readings(walls)
    landmark_measurements = mm.get_landmark_measurements(landmark_groups)
    return wall_readings, landmark_measurements

def reset_robot():
    mm.x = 100
    mm.y = -100 
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


def floreano_step(v_cmd: float, omega_cmd: float, acts: np.ndarray) -> float:
    V      = abs(v_cmd)    / MAX_V
    delta  = abs(omega_cmd) / MAX_OMEGA
    i_max  = float(np.max(acts))
    return max(0.0, V * (1.0 - math.sqrt(delta)) * (1.0 - i_max))


def cell_key(x: float, y: float) -> tuple:
    return (int(x // GRID_CELL), int(y // GRID_CELL))


def evaluate_floreano(genome: np.ndarray, goal_x=GOAL_X, goal_y=GOAL_Y) -> float:
    """
    Pure Floreano fitness – never negative.
    Episode ends early if robot is stuck (saves compute + ends hopeless runs).
    """
    nav_state = reset_robot()
    total = 0.0
    pos_history: list[tuple[float, float]] = []

    for step in range(EVAL_STEPS):
        mm.dt = DT
        acts = navigation_inputs(nav_state, goal_x, goal_y)
        sensor_acts = acts[:N_SENSORS]

        out       = controller.forward(acts, genome)
        mm.v      = float(out[0]) * MAX_V
        mm.omega  = float(out[1]) * MAX_OMEGA
        mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        update_after_movement(nav_state)

        total += floreano_step(mm.v, mm.omega, sensor_acts)

        pos_history.append((nav_state.est_x, nav_state.est_y))

        # Early stop: robot stuck against a wall
        if step >= STUCK_WINDOW:
            oldest = pos_history[-STUCK_WINDOW]
            dist   = math.hypot(nav_state.est_x - oldest[0], nav_state.est_y - oldest[1])
            if dist < STUCK_DIST:
                break   # zero fitness for remaining steps — fair and fast

    return total


def evaluate_coverage(genome: np.ndarray, goal_x=GOAL_X, goal_y=GOAL_Y) -> float:
    nav_state = reset_robot()
    total        = 0.0
    pos_history  : list[tuple[float, float]] = []
    visited_cells: set[tuple] = set()
    prev_explored_cells = nav_state.explored_cells

    for step in range(EVAL_STEPS):
        mm.dt = DT
        acts = navigation_inputs(nav_state, goal_x, goal_y)
        sensor_acts = acts[:N_SENSORS]

        out      = controller.forward(acts, genome)
        mm.v     = float(out[0]) * MAX_V
        mm.omega = float(out[1]) * MAX_OMEGA
        mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        update_after_movement(nav_state)

        # Base fitness (never negative)
        total += floreano_step(mm.v, mm.omega, sensor_acts)

        #Bonus for entering a new area, based on the estimated pose
        current_cell = cell_key(nav_state.est_x, nav_state.est_y)
        if current_cell not in visited_cells:
            visited_cells.add(current_cell)
            total += COVERAGE_BONUS_PER_CELL

        #Bonus for newly explored grid units
        newly_explored = max(0, nav_state.explored_cells - prev_explored_cells)
        total += MAP_COVERAGE_BONUS_PER_CELL * newly_explored
        #Keep track of explored cells
        prev_explored_cells = max(prev_explored_cells, nav_state.explored_cells)

        pos_history.append((nav_state.est_x, nav_state.est_y))

        # Early stop
        if step >= STUCK_WINDOW:
            oldest = pos_history[-STUCK_WINDOW]
            if math.hypot(nav_state.est_x - oldest[0], nav_state.est_y - oldest[1]) < STUCK_DIST:
                break

    return total


def evaluate_goal(genome: np.ndarray, goal_x=GOAL_X, goal_y=GOAL_Y) -> float:
    nav_state = reset_robot()
    total   = 0.0
    prev_d  = math.hypot(nav_state.est_x - goal_x, nav_state.est_y - goal_y)
    pos_history: list[tuple[float, float]] = []

    for step in range(EVAL_STEPS):
        mm.dt = DT
        acts = navigation_inputs(nav_state, goal_x, goal_y)
        sensor_acts = acts[:N_SENSORS]

        out      = controller.forward(acts, genome)
        mm.v     = float(out[0]) * MAX_V
        mm.omega = float(out[1]) * MAX_OMEGA
        mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        update_after_movement(nav_state)

        curr_d = math.hypot(nav_state.est_x - goal_x, nav_state.est_y - goal_y)

        # Progress reward
        progress = prev_d - curr_d
        total += progress

        # Small Floreano term so the robot avoids walls while navigating
        total += 0.2 * floreano_step(mm.v, mm.omega, sensor_acts)

        prev_d = curr_d

        # Goal reached according to the localized pose estimate
        if curr_d < GOAL_RADIUS:
            total += GOAL_BONUS
            break

        pos_history.append((nav_state.est_x, nav_state.est_y))

        # Early stop
        if step >= STUCK_WINDOW:
            oldest = pos_history[-STUCK_WINDOW]
            if math.hypot(nav_state.est_x - oldest[0], nav_state.est_y - oldest[1]) < STUCK_DIST:
                break

    return total

def evaluate_combined(genome: np.ndarray, goal_x=GOAL_X, goal_y=GOAL_Y) -> float:
    a = 0.4  # coverage weight
    b = 0.4  # floreano weight
    c = 0.2  # goal weight

    nav_state = reset_robot()
    total = 0.0
    prev_d = math.hypot(nav_state.est_x - goal_x, nav_state.est_y - goal_y)
    pos_history: list[tuple[float, float]] = []
    visited_cells: set[tuple] = set()
    prev_explored_cells = nav_state.explored_cells

    for step in range(EVAL_STEPS):
        mm.dt = DT
        acts = navigation_inputs(nav_state, goal_x, goal_y)
        sensor_acts = acts[:N_SENSORS]

        out      = controller.forward(acts, genome)
        mm.v     = float(out[0]) * MAX_V
        mm.omega = float(out[1]) * MAX_OMEGA
        mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        update_after_movement(nav_state)

        floreano_score = floreano_step(mm.v, mm.omega, sensor_acts)

        # Coverage bonus
        current_cell = cell_key(nav_state.est_x, nav_state.est_y)
        coverage_score = floreano_score
        if current_cell not in visited_cells:
            visited_cells.add(current_cell)
            coverage_score += COVERAGE_BONUS_PER_CELL
        newly_explored = max(0, nav_state.explored_cells - prev_explored_cells)
        coverage_score += MAP_COVERAGE_BONUS_PER_CELL * newly_explored
        prev_explored_cells = max(prev_explored_cells, nav_state.explored_cells)

        # Goal progress
        curr_d = math.hypot(nav_state.est_x - goal_x, nav_state.est_y - goal_y)
        goal_score = (prev_d - curr_d) + 0.2 * floreano_score
        prev_d = curr_d

        total += a * coverage_score + b * floreano_score + c * goal_score

        pos_history.append((nav_state.est_x, nav_state.est_y))

        if curr_d < GOAL_RADIUS:
            total += c * GOAL_BONUS
            break

        if step >= STUCK_WINDOW:
            oldest = pos_history[-STUCK_WINDOW]
            if math.hypot(nav_state.est_x - oldest[0], nav_state.est_y - oldest[1]) < STUCK_DIST:
                break

    return total
MODES = {
    "floreano": evaluate_floreano,
    "coverage": evaluate_coverage,
    "goal": evaluate_goal,
    "combined": evaluate_combined,
}


def select_eval_goals():
    if not TRAINING_GOALS:
        return [(GOAL_X, GOAL_Y)]

    n_goals = max(1, min(GOALS_PER_EVAL, len(TRAINING_GOALS)))
    random_goals = np.random.choice(len(TRAINING_GOALS), n_goals, replace=False)
    return [TRAINING_GOALS[i] for i in random_goals]


def evaluate(genome, eval_goals):
    scores = [
        MODES[FITNESS_MODE](genome, goal_x, goal_y)
        for goal_x, goal_y in eval_goals
    ]
    #Average them
    return float(np.mean(scores))


def main() -> None:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    #Prevent hello message from pygame when training
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

    print("=" * 64)
    print(f"  Evolutionary Robotics — mode: {FITNESS_MODE.upper()}")
    print("=" * 64)
    print(f"  Architecture : {N_INPUTS}→{N_HIDDEN}→{N_OUTPUTS}")
    print(f"  Inputs       : {N_SENSORS} sensor beams + {N_GOAL_INPUTS} goal inputs")
    print(f"  Genome size  : {controller.genome_size} floats")
    print(f"  Population   : {POP_SIZE}  ×  {N_GENERATIONS} generations")
    print(f"  Eval steps   : {EVAL_STEPS}  (DT={DT}s)")
    if FITNESS_MODE in ("goal", "combined"):
        print(f"  Goals/eval   : {GOALS_PER_EVAL} random goals from {len(TRAINING_GOALS)} goals")
    print("=" * 64)

    ea = EA(
        pop_size = POP_SIZE,
        genome_size = controller.genome_size,
        mutation_rate = MUTATION_RATE,
        mutation_scale = MUTATION_SCALE,
        elite_count = ELITE_COUNT,
    )

    t0 = time.time()

    for gen in range(N_GENERATIONS):
        eval_goals = select_eval_goals()
        gen_t0    = time.time()
        fitnesses = [evaluate(g, eval_goals) for g in ea.population]
        stats     = ea.evolve(fitnesses)
        elapsed   = time.time() - gen_t0

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
