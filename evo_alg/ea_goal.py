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
from ea_tools import EA_goal as EA
from ea_tools import FeedforwardController as ffc 
from ea_tools import WallFollowRecovery as wfr
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

START_X, START_Y = 0.0, 0.0

#training on 5 landmarks (from map, not inside obstacle)
CURRICULUM_GOALS = [
    (278.0, -203.0),
    ( 94.0,  261.0),
    (100.0,  100.0),
    (318.0,   43.0),
    (203.0, -189.0),
]
GOAL_X, GOAL_Y = CURRICULUM_GOALS[0]   # default (used by watch_goal.py)
GENS_PER_GOAL = 20 #training the same genomes on different goals, 20 generations per goal

GOAL_RADIUS = 12.0 #since the landmark is round, we assume the robot to have reached the goal
EVAL_STEPS = 1200 #how long the simulation runs for accumulating fitness function
MAX_STAGNATION = 300 #stop if stuck for this long

GOAL_BONUS = 2000.0 #give a large fitness boost to any genome that actually gets to goal
PROGRESS_GAIN = 5.0 #reward for moving in the right direction
FLOREANO_GAIN = 1.5 #movement reward
COVERAGE_GAIN = 0.5 #exploration reward
COV_CELL_SIZE = 25.0 #exploration reward
CLOSENESS_BONUS = 300.0 #give partial credit at end of episode for goal proximity
COLLISION_PENALTY = 150.0 #deincentivize wall collisions
COLLISION_TERMINATE = 8 #assume car breaks after excessive number of collisions
TERMINATE_PENALTY = 500.0 #large penalty for breaking

#EA hyperparameters
POP_SIZE = 50
N_GENERATIONS  = 100
MUTATION_RATE  = 0.12
MUTATION_SCALE = 0.20
ELITE_COUNT = 4
CROSSOVER_RATE = 0.70
TOURNAMENT_K = 3

N_INPUTS = len(mm.SENSOR_ANGLES_DEG) + 3   # 12 lidar + 3 goal = 15
N_HIDDEN = 8
N_OUTPUTS = 2
WALLFOLLOW_TRIGGER = 4
WALLFOLLOW_WINDOW = 25
WALLFOLLOW_STEPS = 80

walls, landmarks, landmark_groups = mp.create_map()
obstacles = walls + landmarks


def sensor_acts(wall_readings):
    #converts sensor readings into proximity measures, to have highest values where wall is closest
    return np.clip(np.array([1.0 - r["distance"] / mm.SENSOR_MAX_RANGE for r in wall_readings]),0.0, 1.0)


def goal_acts(est_x, est_y, est_theta,goal_x, goal_y):
    #enables to pass goal relative to robot position
    #encodes the bearing as a sin/cos pair to avoid weird numerical behaviour when facing opposite the goal
    dx, dy = goal_x - est_x, goal_y - est_y
    dist   = math.hypot(dx, dy)
    if dist < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    bearing = mm.normalize_angle(math.atan2(dy, dx) - est_theta)
    return np.array([1.0 - math.exp(-dist / 100.0),math.sin(bearing),math.cos(bearing)])


def _cov_cell(x: float, y: float) -> tuple:
    return (int(math.floor(x / COV_CELL_SIZE)), int(math.floor(y / COV_CELL_SIZE)))

def evaluate(genome, controller,goal_x= GOAL_X, goal_y= GOAL_Y):
    controller.reset()
    #start simulation parameters
    mm.x = START_X
    mm.y = START_Y
    mm.theta = 0.0
    mm.v = 0.0
    mm.omega = 0.0
    mm.dt = DT

    #initialize SLAM
    est_x, est_y, est_theta = START_X, START_Y, 0.0
    sigma_mat = SIGMA_0.copy()

    wall_follower = wfr()
    initial_d = math.hypot(mm.x - goal_x, mm.y - goal_y)
    best_d = initial_d

    visited = set()
    last_progress = 0
    total = 0.0
    collisions = 0

    for step in range(EVAL_STEPS):
        mm.dt = DT

        # Sense
        wall_readings = mm.get_sensor_readings(walls)
        proximities = sensor_acts(wall_readings)
        goal = goal_acts(est_x, est_y, est_theta, goal_x, goal_y)
        nn_input = np.concatenate([proximities, goal])

        # Motor commands
        wf_cmd = wall_follower.command(proximities)
        #give priority to cases in which robot is tryin to get "through" wall
        if wf_cmd is not None:
            mm.v, mm.omega = wf_cmd
        else:
            out = controller.forward(nn_input, genome)
            mm.v = float(out[0]) * MAX_V
            mm.omega = float(out[1]) * MAX_OMEGA

        #Simulate controller input
        hit = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
        wall_follower.tick(hit, mm.x, mm.y)

        # EKF, assumes contact sensors allow detection of hitting wall
        eff_v = 0.0 if hit else mm.v
        eff_omega = 0.0 if hit else mm.omega
        lm_meas = mm.get_landmark_measurements(landmark_groups)
        mu_bar, sigma_mat = kf.ekf_filter(est_x, est_y, est_theta, sigma_mat,SIGMA_R, SIGMA_Q, eff_v, eff_omega, DT, lm_meas,)
        est_x = float(mu_bar[0, 0])
        est_y = float(mu_bar[1, 0])
        est_theta = float(mu_bar[2, 0])

        # Collision penalty
        if hit:
            total -= COLLISION_PENALTY
            collisions += 1
            if collisions >= COLLISION_TERMINATE:
                total -= TERMINATE_PENALTY
                break

        # Floreano smooth-driving reward
        speed = abs(mm.v) / MAX_V
        turning = abs(mm.omega) / MAX_OMEGA
        clearance = 1.0 - float(np.max(proximities)) #bonus for not being unnecessarily close to obstacles
        total += FLOREANO_GAIN * speed * (1.0 - turning) * clearance

        # Coverage
        cell = _cov_cell(mm.x, mm.y)
        if cell not in visited:
            visited.add(cell)
            total += COVERAGE_GAIN

        curr_d = math.hypot(mm.x - goal_x, mm.y - goal_y)
        if curr_d < best_d - 0.5:
            total += PROGRESS_GAIN * (best_d - curr_d)
            best_d = curr_d
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



def main():
    #turns off the py game frame so I don't have to watch the entire evolution process (takes hours, better for battery)
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

    controller = ffc(N_INPUTS, N_HIDDEN, N_OUTPUTS)

    n_goals = len(CURRICULUM_GOALS)

    ea = EA(
        pop_size = POP_SIZE,
        genome_size = controller.genome_size,
        mutation_rate = MUTATION_RATE,
        mutation_scale = MUTATION_SCALE,
        elite_count = ELITE_COUNT,
        crossover_rate = CROSSOVER_RATE,
        tournament_k = TOURNAMENT_K,
        init_fn = controller.random_genome,
    )

    t0 = time.time()
    for gen in range(N_GENERATIONS):
        # Rotate goal every GENS_PER_GOAL generations
        goal_idx = (gen // GENS_PER_GOAL) % n_goals
        g_x, g_y = CURRICULUM_GOALS[goal_idx]

        gen_t0 = time.time() #helpful for me to get an estimation of how long it will take
        #evaluation
        fitnesses = [evaluate(gn, controller, g_x, g_y) for gn in ea.population]
        #selection+reproduction
        stats = ea.evolve(fitnesses)
        elapsed = time.time() - gen_t0

        #print statement formatted by claude
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

if __name__ == "__main__":#otherwise watch_best retrains the whole thing from scratch
    main()
