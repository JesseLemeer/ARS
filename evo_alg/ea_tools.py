import math
import numpy as np



class NeuralController:
    def __init__(self, n_inputs: int = 15, n_hidden: int = 10, n_outputs: int = 2):
        self.n_inputs = n_inputs
        self.n_hidden = n_hidden
        self.n_outputs = n_outputs

        # genome segment sizes (+1 for bias neuron in each layer)
        self._w1_size = (n_inputs + 1) * n_hidden
        self._w2_size = (n_hidden + 1) * n_outputs
        self.genome_size = self._w1_size + self._w2_size

    def forward(self, sensor_inputs: np.ndarray, genome: np.ndarray) -> np.ndarray:
        w1 = genome[: self._w1_size].reshape(self.n_hidden, self.n_inputs + 1)
        w2 = genome[self._w1_size :].reshape(self.n_outputs, self.n_hidden + 1)

        # Layer 1
        x1 = np.append(sensor_inputs, 1.0)
        h  = np.tanh(w1 @ x1)

        # Layer 2
        x2  = np.append(h, 1.0)
        out = np.tanh(w2 @ x2)

        return out

    def random_genome(self, scale: float = 0.5) -> np.ndarray:
        return np.random.randn(self.genome_size) * scale


class EA:
    def __init__(
        self,
        pop_size: int   = 20,
        genome_size:int   = 182,
        mutation_rate: float = 0.10,# probability a gene is perturbed
        mutation_scale: float = 0.30,# std-dev of Gaussian perturbation
        crossover_rate: float = 0.70, # probability of crossover vs. clone
        elite_count:int   = 2, # number of best individuals kept unchanged
        tournament_k:int   = 3, # tournament size for selection
    ):
        self.pop_size       = pop_size
        self.genome_size    = genome_size
        self.mutation_rate  = mutation_rate
        self.mutation_scale = mutation_scale
        self.crossover_rate = crossover_rate
        self.elite_count    = elite_count
        self.tournament_k   = tournament_k

        self.generation     = 0
        self.best_genome    = None
        self.best_fitness   = -np.inf
        self.fitness_history: list[dict] = []

        self.population: list[np.ndarray] = [
            np.random.randn(genome_size) * 0.5 for _ in range(pop_size)
        ]
        self.fitnesses = np.zeros(pop_size)

    def _tournament_select(self) -> np.ndarray:
        idx  = np.random.choice(self.pop_size, self.tournament_k, replace=False)
        best = idx[np.argmax(self.fitnesses[idx])]
        return self.population[best].copy()

    def _crossover(self, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
        if np.random.random() < self.crossover_rate:
            pt    = np.random.randint(1, self.genome_size)
            child = np.concatenate([p1[:pt], p2[pt:]])
        else:
            child = p1.copy()
        return child

    def _mutate(self, genome: np.ndarray) -> np.ndarray:
        genome = genome.copy()
        mask   = np.random.random(self.genome_size) < self.mutation_rate
        genome[mask] += np.random.randn(mask.sum()) * self.mutation_scale
        return genome

    def evolve(self, fitnesses: list[float]) -> dict:
        self.fitnesses = np.array(fitnesses, dtype=float)

        best_idx = int(np.argmax(self.fitnesses))
        if self.fitnesses[best_idx] > self.best_fitness:
            self.best_fitness = float(self.fitnesses[best_idx])
            self.best_genome  = self.population[best_idx].copy()

        stats = {
            "gen":  self.generation,
            "best": float(self.fitnesses[best_idx]),
            "avg":  float(np.mean(self.fitnesses)),
            "worst":float(np.min(self.fitnesses)),
        }
        self.fitness_history.append(stats)

        sorted_idx   = np.argsort(self.fitnesses)[::-1]
        new_population = []

        for i in range(min(self.elite_count, self.pop_size)):
            new_population.append(self.population[sorted_idx[i]].copy())

        while len(new_population) < self.pop_size:
            p1    = self._tournament_select()
            p2    = self._tournament_select()
            child = self._crossover(p1, p2)
            child = self._mutate(child)
            new_population.append(child)

        self.population = new_population
        self.generation += 1

        return stats
