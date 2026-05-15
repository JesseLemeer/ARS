"""
Headless swarm experiment runner.
Purpose:
- Compare 1 vs 2 vs 4 vs 5 robots
- Compare with/without shared occupancy map
- Compare with/without robot-robot avoidance
- Compare different EA genomes
- Saved as JSON, CSV, and plots
"""

from __future__ import annotations
import argparse
import csv
import importlib
import json
import math
import os
import time
from pathlib import Path
from statistics import mean

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import matplotlib.pyplot as plt
import numpy as np

BASE_DIR = Path(__file__).resolve().parent

try:
    from evo_alg._path_setup import ensure_project_root_on_path
except ModuleNotFoundError:
    from _path_setup import ensure_project_root_on_path

ensure_project_root_on_path(__file__)

import map as mp

swarm = importlib.import_module("evo_alg.swarm_watch_Main")

GOALS_9 = [
    (-300, 100),
    (-312, 145),
    (278, -203),
    (341, 217),
    (203, -189),
    (-367, 112),
    (94, 261),
    (-241, -238),
    (318, 43),
]

GOALS_12 = GOALS_9 + [
    (-120, 210),
    (40, -245),
    (250, 95),
]

START_POSES_5 = [
    (-200.0, 0.0, 0.0),
    (0.0, -200.0, math.radians(90.0)),
    (200.0, 0.0, math.radians(180.0)),
    (0.0, 200.0, math.radians(-90.0)),
    (260.0, -190.0, math.radians(-90.0)),
]

def available_genomes() -> list[Path]:
    candidates = [
        BASE_DIR / "best_genome" / "best_genome_staged.npy",
        BASE_DIR / "best_genome" / "best_genome_latest_staged.npy",
    ]
    return [p for p in candidates if p.exists()]

def build_robots(n_robots: int, shared_map: bool, walls, landmark_groups):
    starts = START_POSES_5[:n_robots]

    if shared_map:
        shared_grid = swarm.make_shared_grid()
        robots = [
            swarm.make_robot(i, starts[i], shared_grid, walls, landmark_groups)
            for i in range(n_robots)
        ]
        return robots, shared_grid

    robots = [
        swarm.make_robot(i, starts[i], swarm.make_shared_grid(), walls, landmark_groups)
        for i in range(n_robots)
    ]
    return robots, None

def explored_cells_metric(robots, shared_map: bool) -> tuple[int, float]:
    if not robots:
        return 0, 0.0

    explored_each = []
    for r in robots:
        swarm.refresh_grid_stats(r.nav_state)
        explored_each.append(int(r.nav_state.explored_cells))

    if shared_map:
        return explored_each[0], float(explored_each[0])

    return int(sum(explored_each)), float(mean(explored_each))

def run_single_trial(
    *,
    n_robots: int,
    goals,
    shared_map: bool,
    car_avoidance: bool,
    genome_path: Path,
    max_steps: int,
    trial_id: int,
) -> dict:
    walls, landmarks, landmark_groups = mp.create_map()

    genome = np.load(genome_path)
    hidden = swarm.infer_hidden_size(len(genome))
    controller = swarm.FeedforwardController(swarm.N_INPUTS, hidden, swarm.N_OUTPUTS)
    if hasattr(controller, "reset"):
        controller.reset()

    old_n_robots = swarm.N_ROBOTS
    old_task_goals = list(swarm.TASK_GOALS)
    old_start_poses = list(swarm.START_POSES)
    old_goal_limit = getattr(swarm, "MAX_GOALS_PER_ROBOT", None)
    old_sep_func = swarm.apply_robot_robot_separation
    old_rr_collision_func = swarm.robot_robot_collision

    swarm.N_ROBOTS = n_robots
    swarm.TASK_GOALS = list(goals)
    swarm.START_POSES = START_POSES_5[:n_robots]

    swarm.MAX_GOALS_PER_ROBOT = max(3, math.ceil(len(goals) / n_robots))

    if not car_avoidance:
        swarm.apply_robot_robot_separation = lambda robot, robots: None
        swarm.robot_robot_collision = lambda robot, robots: None

    reached_goals: set[tuple[float, float]] = set()
    robots, shared_grid = build_robots(n_robots, shared_map, walls, landmark_groups)

    start_time = time.time()
    recovery_robot_steps = 0
    completed = False

    try:
        for step in range(1, max_steps + 1):
            for robot in robots:
                swarm.update_one_robot(
                    robot,
                    robots,
                    controller,
                    genome,
                    walls,
                    landmark_groups,
                    reached_goals,
                )

            recovery_robot_steps += sum(1 for r in robots if r.recovery_steps > 0)

            if len(reached_goals) == len(goals):
                completed = True
                break

        runtime_s = time.time() - start_time

        explored_total, explored_avg = explored_cells_metric(robots, shared_map)

        result = {
            "trial": trial_id,
            "genome": genome_path.name,
            "n_robots": n_robots,
            "n_goals": len(goals),
            "shared_map": shared_map,
            "car_avoidance": car_avoidance,
            "completed": completed,
            "steps": step,
            "goals_reached": len(reached_goals),
            "goal_success_rate": len(reached_goals) / len(goals),
            "explored_cells_total": explored_total,
            "explored_cells_avg_per_robot": explored_avg,
            "total_collisions": int(sum(r.collisions for r in robots)),
            "wall_collisions": int(sum(r.wall_collisions for r in robots)),
            "car_car_collisions": int(sum(r.robot_collisions for r in robots)),
            "avg_pose_error": float(mean([r.pose_error for r in robots])) if robots else 0.0,
            "recovery_robot_steps": int(recovery_robot_steps),
            "runtime_s": runtime_s,
            "per_robot_goals": [int(r.goals_reached) for r in robots],
            "per_robot_wall_collisions": [int(r.wall_collisions) for r in robots],
            "per_robot_car_collisions": [int(r.robot_collisions) for r in robots],
            "per_robot_pose_error": [float(r.pose_error) for r in robots],
        }
        return result

    finally:
        swarm.N_ROBOTS = old_n_robots
        swarm.TASK_GOALS = old_task_goals
        swarm.START_POSES = old_start_poses
        if old_goal_limit is not None:
            swarm.MAX_GOALS_PER_ROBOT = old_goal_limit
        swarm.apply_robot_robot_separation = old_sep_func
        swarm.robot_robot_collision = old_rr_collision_func

def make_experiment_plan(genomes: list[Path], max_robot_count: int) -> list[dict]:
    stable = next((g for g in genomes if g.name == "best_genome_staged.npy"), genomes[0])
    plans = []

    for n in [1, 2, 4]:
        if n <= max_robot_count:
            plans.append({
                "n_robots": n,
                "goals": GOALS_9,
                "shared_map": True,
                "car_avoidance": True,
                "genome": stable,
            })

    if max_robot_count >= 5:
        plans.append({
            "n_robots": 5,
            "goals": GOALS_12,
            "shared_map": True,
            "car_avoidance": True,
            "genome": stable,
        })

    if max_robot_count >= 4:
        plans.extend([
            {
                "n_robots": 4,
                "goals": GOALS_9,
                "shared_map": False,
                "car_avoidance": True,
                "genome": stable,
            },
            {
                "n_robots": 4,
                "goals": GOALS_9,
                "shared_map": True,
                "car_avoidance": False,
                "genome": stable,
            },
            {
                "n_robots": 4,
                "goals": GOALS_9,
                "shared_map": False,
                "car_avoidance": False,
                "genome": stable,
            },
        ])

    for genome in genomes:
        if genome == stable:
            continue
        plans.append({
            "n_robots": 4,
            "goals": GOALS_9,
            "shared_map": True,
            "car_avoidance": True,
            "genome": genome,
        })

    return plans

def config_label(row: dict) -> str:
    shared = "shared" if row["shared_map"] else "no-share"
    avoid = "avoid" if row["car_avoidance"] else "no-avoid"
    genome = row["genome"].replace("best_genome_", "").replace(".npy", "")
    return f'{row["n_robots"]}R-{row["n_goals"]}G-{shared}-{avoid}-{genome}'

def save_results(results: list[dict]) -> None:
    json_path = BASE_DIR / "swarm_experiment_results" / "swarm_experiment_results.json"
    csv_path = BASE_DIR / "swarm_experiment_results" / "swarm_experiment_results.csv"

    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)

    if results:
        fieldnames = list(results[0].keys())
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

    print(f"Saved JSON -> {json_path}")
    print(f"Saved CSV  -> {csv_path}")

def plot_metric(results: list[dict], metric: str, ylabel: str, filename: str) -> None:
    plot_dir = BASE_DIR / "swarm_experiment_plots"
    plot_dir.mkdir(exist_ok=True)

    labels = [config_label(r) for r in results]
    values = [r[metric] for r in results]

    plt.figure(figsize=(max(10, len(labels) * 1.2), 5))
    plt.bar(range(len(values)), values)
    plt.xticks(range(len(labels)), labels, rotation=45, ha="right")
    plt.ylabel(ylabel)
    plt.title(ylabel + " by swarm experiment")
    plt.tight_layout()
    out = plot_dir / filename
    plt.savefig(out, dpi=160)
    plt.close()
    print(f"Saved plot -> {out}")

def make_plots(results: list[dict]) -> None:
    if not results:
        return

    plot_metric(results, "goals_reached", "Goals reached", "goals_reached.png")
    plot_metric(results, "wall_collisions", "Wall collisions", "wall_collisions.png")
    plot_metric(results, "car_car_collisions", "Car-car collisions", "car_car_collisions.png")
    plot_metric(results, "explored_cells_total", "Explored cells", "explored_cells.png")
    plot_metric(results, "avg_pose_error", "Average pose error", "avg_pose_error.png")
    plot_metric(results, "recovery_robot_steps", "Recovery robot-steps", "recovery_robot_steps.png")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-steps", type=int, default=5000)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--max-robots", type=int, default=5, choices=[1, 2, 4, 5])
    args = parser.parse_args()

    genomes = available_genomes()
    if not genomes:
        raise FileNotFoundError(
            "No genome files found. Expected best_genome_staged.npy inside evo_alg/."
        )

    plan = make_experiment_plan(genomes, args.max_robots)

    print("=" * 70)
    print("Swarm experiment runner")
    print(f"Genomes: {[g.name for g in genomes]}")
    print(f"Configs: {len(plan)}  Trials/config: {args.trials}  Max steps: {args.max_steps}")
    print("=" * 70)

    results = []
    trial_counter = 0

    for cfg in plan:
        for trial in range(args.trials):
            trial_counter += 1
            print(
                f"[{trial_counter}] "
                f"{cfg['n_robots']} robots | {len(cfg['goals'])} goals | "
                f"shared={cfg['shared_map']} | avoid={cfg['car_avoidance']} | "
                f"genome={cfg['genome'].name}"
            )

            result = run_single_trial(
                n_robots=cfg["n_robots"],
                goals=cfg["goals"],
                shared_map=cfg["shared_map"],
                car_avoidance=cfg["car_avoidance"],
                genome_path=cfg["genome"],
                max_steps=args.max_steps,
                trial_id=trial,
            )
            results.append(result)

            print(
                f"    goals={result['goals_reached']}/{result['n_goals']} "
                f"steps={result['steps']} "
                f"wall={result['wall_collisions']} "
                f"car={result['car_car_collisions']} "
                f"pose_err={result['avg_pose_error']:.1f}"
            )

    save_results(results)
    make_plots(results)

    print("\nDone. Use the CSV/JSON and plots in your swarm experiment report.")


if __name__ == "__main__":
    main()
