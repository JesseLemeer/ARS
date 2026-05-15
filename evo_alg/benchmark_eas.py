import json
import sys
import time
from pathlib import Path

import numpy as np

BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

from ea_tools import EA, EA_goal, EA_new

def rastrigin(x, A=10.0):
    x = np.asarray(x, dtype=float)
    return float(A * len(x) + np.sum(x ** 2 - A * np.cos(2.0 * np.pi * x)))


def rosenbrock(x):
    x = np.asarray(x, dtype=float)
    return float(np.sum(100.0 * (x[1:] - x[:-1] ** 2) ** 2
                        + (1.0 - x[:-1]) ** 2))

class EAAdapter:
    def __init__(self, ea, kind):
        self.ea = ea
        self.kind = kind
    @property
    def population(self):
        return self.ea.population

    def step(self, fitnesses):
        if self.kind in ("EA", "EA_goal"):
            return self.ea.evolve(fitnesses)
        # EA_new takes (objective, combined). Without novelty, we pass objective twice.
        return self.ea.step(fitnesses, fitnesses)


def make_ea(name, genome_size, init_fn,
            pop_size, mutation_rate, mutation_scale,
            elite_count, crossover_rate, tournament_k):
    if name == "EA":
        ea = EA(pop_size=pop_size, genome_size=genome_size,
                mutation_rate=mutation_rate, mutation_scale=mutation_scale,
                elite_count=elite_count, crossover_rate=crossover_rate,
                tournament_k=tournament_k, init_genome_fn=init_fn)
        return EAAdapter(ea, "EA")

    if name == "EA_goal":
        ea = EA_goal(pop_size=pop_size, genome_size=genome_size,
                     mutation_rate=mutation_rate, mutation_scale=mutation_scale,
                     elite_count=elite_count, crossover_rate=crossover_rate,
                     tournament_k=tournament_k, init_fn=init_fn)
        return EAAdapter(ea, "EA_goal")

    if name in ("EA_new_tournament", "EA_new_roulette"):
        selection = "tournament" if name.endswith("tournament") else "roulette"
        ea = EA_new(pop_size, genome_size, init_fn,
                    mutation_rate=mutation_rate, mutation_scale=mutation_scale,
                    elite_count=elite_count, crossover_rate=crossover_rate,
                    tournament_k=tournament_k, selection=selection)
        return EAAdapter(ea, "EA_new")



def run_one(test_fn, ea_name, dim, bounds, n_gens, n_seeds, hp):
    lo, hi = bounds
    history = np.zeros((n_seeds, n_gens + 1))

    for seed in range(n_seeds):
        np.random.seed(seed)
        init_fn = lambda: np.random.uniform(lo, hi, dim)
        adapter = make_ea(ea_name, dim, init_fn, **hp)

        pop = adapter.population
        values = np.array([test_fn(g) for g in pop])
        best = values.min()
        history[seed, 0] = best

        for gen in range(n_gens):
            adapter.step((-values).tolist())
            pop = adapter.population
            values = np.array([test_fn(g) for g in pop])
            best = min(best, values.min())
            history[seed, gen + 1] = best
    return history


def main():
    OUT = BASE_DIR / "benchmark_results"
    OUT.mkdir(exist_ok=True)
    DIM  = 10
    N_GENS  = 100
    N_SEEDS  = 10
    #assign common hyperparameters to each
    COMMON_HP = dict(
        pop_size       = 50,
        mutation_rate  = 0.10,
        mutation_scale = 0.25,
        elite_count    = 2,
        crossover_rate = 0.70,
        tournament_k   = 3,
    )
    EAS = ["EA", "EA_new_tournament", "EA_new_roulette"]

    FUNCS = {
        "Rastrigin":  (rastrigin,  (-5.12, 5.12)),
        "Rosenbrock": (rosenbrock, (-5.0,  10.0)),
    }

    print(f"Benchmarking {len(EAS)} EAs × {len(FUNCS)} functions × "
          f"{N_SEEDS} seeds × {N_GENS} gens (dim={DIM}, pop={COMMON_HP['pop_size']})")
    print("-" * 75)

    results, summary = {}, {}
    t0 = time.time()

    for fname, (fn, bounds) in FUNCS.items():
        results[fname], summary[fname] = {}, {}
        for ea_name in EAS:
            t_start = time.time()
            hist    = run_one(fn, ea_name, DIM, bounds, N_GENS, N_SEEDS, COMMON_HP)
            results[fname][ea_name] = hist
            np.save(OUT / f"{fname}_{ea_name}.npy", hist)
            final = hist[:, -1]
            summary[fname][ea_name] = {
                "mean_final": float(final.mean()),
                "std_final":  float(final.std()),
                "min_final":  float(final.min()),
                "max_final":  float(final.max()),
            }
            print(f"  {fname:12s} {ea_name:25s}  "
                  f"final = {final.mean():9.3f} ± {final.std():7.3f}   "
                  f"min seed = {final.min():9.3f}   "
                  f"({time.time() - t_start:.1f}s)")

    print(f"\nTotal: {time.time() - t0:.1f}s")

    with open(OUT / "summary.json", "w") as fh:
        json.dump(summary, fh, indent=2)

    # --- Convergence plot -------------------------------------------------
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed — raw data saved, skipping plot.")
        return

    fig, axes = plt.subplots(1, len(FUNCS), figsize=(6 * len(FUNCS), 5))
    if len(FUNCS) == 1:
        axes = [axes]

    for ax, (fname, by_ea) in zip(axes, results.items()):
        for ea_name in EAS:
            hist = by_ea[ea_name]
            mean = hist.mean(axis=0)
            std  = hist.std(axis=0)
            xs   = np.arange(len(mean))
            ax.plot(xs, mean, label=ea_name, linewidth=1.8)
            ax.fill_between(xs,np.maximum(mean - std, 1e-3),mean + std, alpha=0.18)
        ax.set_title(f"{fname}  (dim={DIM}, {N_SEEDS} seeds)")
        ax.set_xlabel("Generation")
        ax.set_ylabel("Best objective so far  (lower is better)")
        ax.set_yscale("log")
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=9)

    plt.tight_layout()
    plt.savefig(OUT / "convergence.png", dpi=130, bbox_inches="tight")
    print(f"\nSaved → {OUT/'convergence.png'}")
    print(f"Raw histories: {OUT}/<func>_<ea>.npy")
    print(f"Summary:       {OUT}/summary.json")


if __name__ == "__main__":
    main()
