import collections
import math
import numpy as np

MAX_V = 100.0
MAX_OMEGA = 5.0

WALLFOLLOW_TRIGGER = 2
WALLFOLLOW_WINDOW = 25
WALLFOLLOW_STEPS = 100
STUCK_DIST_THR = 20.0   # units — robot must travel this far in STUCK_STEPS or be deemed stuck
STUCK_STEPS = 50     # steps to measure displacement over

NOVELTY_K = 15 #how many closest neighbours are considered when quantifying novelty
ARCHIVE_CAP = 300 #number of behaviours stored to be compared against
ARCHIVE_PROB = 0.05

MUTATION_RATE = 0.12
MUTATION_SCALE = 0.20
ELITE_COUNT = 4
CROSSOVER_RATE = 0.70
TOURNAMENT_K = 3


#two-hidden-layer feedforward network; original controller used in early navigation experiments
class NeuralController:
    def __init__(self, n_inputs=15, n_hidden1=20, n_hidden2=12, n_outputs=2):
        self.n_inputs = n_inputs
        self.n_hidden1 = n_hidden1
        self.n_hidden2 = n_hidden2
        self.n_outputs = n_outputs

        self._w1_size = (n_inputs + 1) * n_hidden1
        self._w2_size = (n_hidden1 + 1) * n_hidden2
        self._w3_size = (n_hidden2 + 1) * n_outputs
        self.genome_size = self._w1_size + self._w2_size + self._w3_size

    def reset(self):
        pass

    def forward(self, sensor_inputs, genome):
        i = 0
        w1 = genome[i : i + self._w1_size].reshape(self.n_hidden1, self.n_inputs + 1)
        i += self._w1_size
        w2 = genome[i : i + self._w2_size].reshape(self.n_hidden2, self.n_hidden1 + 1)
        i += self._w2_size
        w3 = genome[i : i + self._w3_size].reshape(self.n_outputs, self.n_hidden2 + 1)

        h1 = np.tanh(w1 @ np.append(sensor_inputs, 1.0))
        h2 = np.tanh(w2 @ np.append(h1, 1.0))
        return np.tanh(w3 @ np.append(h2, 1.0))

    def random_genome(self, scale=0.5):
        return np.random.randn(self.genome_size) * scale

#Elman recurrent network; hidden state lets the robot integrate sensor history across steps
class RecurrentController:
    def __init__(self, n_inputs=15, n_hidden=20, n_outputs=2):
        self.n_inputs = n_inputs
        self.n_hidden = n_hidden
        self.n_outputs = n_outputs

        self._w_xh = n_inputs * n_hidden
        self._w_hh = n_hidden * n_hidden
        self._b_h = n_hidden
        self._w_hy = n_hidden * n_outputs
        self._b_y = n_outputs

        self.genome_size = (
            self._w_xh + self._w_hh + self._b_h + self._w_hy + self._b_y
        )
        self.hidden = np.zeros(n_hidden)

    def reset(self):
        self.hidden = np.zeros(self.n_hidden)

    def forward(self, sensor_inputs, genome):
        i = 0
        W_xh = genome[i : i + self._w_xh].reshape(self.n_hidden, self.n_inputs)
        i += self._w_xh
        W_hh = genome[i : i + self._w_hh].reshape(self.n_hidden, self.n_hidden)
        i += self._w_hh
        b_h = genome[i : i + self._b_h]
        i += self._b_h
        W_hy = genome[i : i + self._w_hy].reshape(self.n_outputs, self.n_hidden)
        i += self._w_hy
        b_y = genome[i : i + self._b_y]

        self.hidden = np.tanh(W_xh @ sensor_inputs + W_hh @ self.hidden + b_h)
        return np.tanh(W_hy @ self.hidden + b_y)

    def random_genome(self, scale=0.5):
        #small scale to keep it bounded
        g = np.random.randn(self.genome_size) * scale
        recurrent_scale = 0.3 / math.sqrt(self.n_hidden)
        g[self._w_xh : self._w_xh + self._w_hh] = (
            np.random.randn(self._w_hh) * recurrent_scale
        )
        return g


#single-hidden-layer feedforward network used in all goal-reaching experiments
class FeedforwardController:
    def __init__(self, n_in, n_hidden, n_out):
        self.n_in, self.n_hidden, self.n_out = n_in, n_hidden, n_out
        self._s1 = n_in * n_hidden
        self._b1 = n_hidden
        self._s2 = n_hidden * n_out
        self._b2 = n_out
        self.genome_size = self._s1 + self._b1 + self._s2 + self._b2

    def reset(self):
        pass

    def forward(self, x, genome):
        i = 0
        W1 = genome[i:i + self._s1].reshape(self.n_hidden, self.n_in); 
        i += self._s1
        b1 = genome[i:i + self._b1];
        i += self._b1
        W2 = genome[i:i + self._s2].reshape(self.n_out, self.n_hidden); 
        i += self._s2
        b2 = genome[i:i + self._b2]
        h = np.tanh(W1 @ x + b1)
        return np.tanh(W2 @ h + b2)

    def random_genome(self):
        return np.random.randn(self.genome_size) * 2


#failsafe recovery behaviour that overrides the controller when the robot is stuck or crashing against a wall
class WallFollowRecovery:
    TARGET_RIGHT = 0.55
    KP = 2.5
    STUCK_THRESHOLD = 3 # consecutive steps with no movement before reversing

    def __init__(self):
        self._buf = collections.deque(maxlen=WALLFOLLOW_WINDOW)
        self._pos_buf = collections.deque(maxlen=STUCK_STEPS)
        self._steps_left = 0
        self._stuck_count = 0
        self._prev_pos = None

    @property
    def active(self):
        return self._steps_left > 0

    def tick(self, hit, x=None, y=None):
        self._buf.append(int(hit))
        if x is not None:
            self._pos_buf.append((x, y))

        if self.active:
            # Track whether the robot is actually moving during recovery
            if x is not None:
                if self._prev_pos is not None:
                    moved = math.hypot(x - self._prev_pos[0], y - self._prev_pos[1])
                    self._stuck_count = self._stuck_count + 1 if moved < 0.1 else 0
                self._prev_pos = (x, y)
            return

        # Reset stuck tracking when wall-follow is inactive
        self._stuck_count = 0
        self._prev_pos    = None

        if sum(self._buf) >= WALLFOLLOW_TRIGGER:
            self._steps_left = WALLFOLLOW_STEPS
            self._buf.clear()
            self._pos_buf.clear()
            return
        # Position-based stuck detection: activate if we haven't moved far enough
        if x is not None and len(self._pos_buf) == STUCK_STEPS:
            dx = x - self._pos_buf[0][0]
            dy = y - self._pos_buf[0][1]
            if math.hypot(dx, dy) < STUCK_DIST_THR:
                self._steps_left = WALLFOLLOW_STEPS
                self._buf.clear()
                self._pos_buf.clear()

    def command(self, raw_acts):
        if not self.active:
            return None
        self._steps_left -= 1
        front = float(raw_acts[0])
        right = float(np.mean(raw_acts[9:11]))

        # Wedged and not moving at all → reverse to break free
        if self._stuck_count >= self.STUCK_THRESHOLD:
            return -MAX_V * 0.3, -MAX_OMEGA * 0.7

        # Wall directly ahead → back up while turning rather than pushing into it
        if front > 0.78:
            return -MAX_V * 0.2, MAX_OMEGA * 0.85

        omega = self.KP * (right - self.TARGET_RIGHT)
        v = MAX_V * (0.55 - 0.25 * front)
        return (float(np.clip(v, 5.0, MAX_V)),
                float(np.clip(omega, -MAX_OMEGA, MAX_OMEGA)))


#novelty search archive; promotes exploration by rewarding behaviourally distinct individuals
class NoveltyArchive:
    def __init__(self, k=NOVELTY_K, cap=ARCHIVE_CAP, p_add=ARCHIVE_PROB):
        self.k = k
        self.cap = cap
        self.p_add = p_add
        self.buf = collections.deque(maxlen=cap)

    def maybe_add(self, b):
        if np.random.random() < self.p_add:
            self.buf.append(np.asarray(b, dtype=float))

    def novelty(self, b, pop_behaviours):
        pool = list(self.buf) + [np.asarray(bb, dtype=float) for bb in pop_behaviours]
        if len(pool) <= 1:
            return 0.0
        b_arr = np.asarray(b, dtype=float)
        pool_arr = np.asarray(pool)
        d = np.linalg.norm(pool_arr - b_arr, axis=1)
        d_sorted = np.sort(d)
        # Drop the self-match (distance ≈ 0) if present
        if d_sorted[0] < 1e-6:
            d_sorted = d_sorted[1:]
        if len(d_sorted) == 0:
            return 0.0
        return float(np.mean(d_sorted[:self.k]))


#gradually grows the active training goal pool as the population masters the current set
class Curriculum:
    def __init__(self, all_goals, init_size, grow_every, master_thr):
        self.all_goals  = list(all_goals)
        self.pool = list(all_goals[:init_size])
        self.grow_every = grow_every
        self.master_thr = master_thr
        self._gen_since_growth = 0

    def slate(self, k):
        #samples K goals from the active pool. Without replacement when possible, with when necessary
        replace = k > len(self.pool)
        idx = np.random.choice(len(self.pool), size=k, replace=replace)
        return [self.pool[i] for i in idx]
    #grow pool when requirements are met
    def maybe_grow(self, best_gen_objective):
        self._gen_since_growth += 1
        if self._gen_since_growth < self.grow_every:
            return False
        if best_gen_objective < self.master_thr:
            return False
        if len(self.pool) >= len(self.all_goals):
            return False
        self.pool.append(self.all_goals[len(self.pool)])
        self._gen_since_growth = 0
        return True

#first EA implemented, uses NeuralController (largest genome); more difficult to evolve due to size
class EA:
    def __init__(
        self,
        pop_size=40,
        genome_size=762,
        mutation_rate=0.10,
        mutation_scale=0.30,
        crossover_rate=0.70,
        elite_count=2,
        tournament_k=3,
        init_genome_fn=None,
    ):
        self.pop_size = pop_size
        self.genome_size = genome_size
        self.mutation_rate = mutation_rate
        self.mutation_scale = mutation_scale
        self.crossover_rate = crossover_rate
        self.elite_count = elite_count
        self.tournament_k = tournament_k

        self.generation = 0
        self.best_genome = None
        self.best_fitness = -np.inf
        self.fitness_history: list[dict] = []

        if init_genome_fn is not None:
            self.population = [init_genome_fn() for _ in range(pop_size)]
        else:
            self.population = [
                np.random.randn(genome_size) * 0.5 for _ in range(pop_size)
            ]
        self.fitnesses = np.zeros(pop_size)

    def _tournament_select(self):
        idx = np.random.choice(self.pop_size, self.tournament_k, replace=False)
        best = idx[np.argmax(self.fitnesses[idx])]
        return self.population[best].copy()

    def _crossover(self, p1, p2):
        if np.random.random() < self.crossover_rate:
            pt = np.random.randint(1, self.genome_size)
            return np.concatenate([p1[:pt], p2[pt:]])
        return p1.copy()

    def _mutate(self, genome):
        genome = genome.copy()
        mask = np.random.random(self.genome_size) < self.mutation_rate
        genome[mask] += np.random.randn(mask.sum()) * self.mutation_scale
        return genome

    def evolve(self, fitnesses):
        self.fitnesses = np.array(fitnesses, dtype=float)

        best_idx = int(np.argmax(self.fitnesses))
        if self.fitnesses[best_idx] > self.best_fitness:
            self.best_fitness = float(self.fitnesses[best_idx])
            self.best_genome = self.population[best_idx].copy()

        stats = {
            "gen": self.generation,
            "best": float(self.fitnesses[best_idx]),
            "avg": float(np.mean(self.fitnesses)),
            "worst": float(np.min(self.fitnesses)),
        }
        self.fitness_history.append(stats)

        sorted_idx = np.argsort(self.fitnesses)[::-1]
        new_population = []
        for i in range(min(self.elite_count, self.pop_size)):
            new_population.append(self.population[sorted_idx[i]].copy())

        while len(new_population) < self.pop_size:
            p1 = self._tournament_select()
            p2 = self._tournament_select()
            child = self._crossover(p1, p2)
            child = self._mutate(child)
            new_population.append(child)

        self.population = new_population
        self.generation += 1
        return stats


#updated EA with separate objective/novelty fitness tracking and configurable selection strategy (tournament or roulette)
class EA_new:
    def __init__(self, pop_size, genome_size, init_fn,
                 mutation_rate=MUTATION_RATE, mutation_scale=MUTATION_SCALE,
                 elite_count=ELITE_COUNT, crossover_rate=CROSSOVER_RATE,
                 tournament_k=TOURNAMENT_K, selection='tournament'):
        self.pop_size = pop_size
        self.genome_size  = genome_size
        self.mutation_rate = mutation_rate
        self.mutation_scale = mutation_scale
        self.elite_count = elite_count
        self.crossover_rate = crossover_rate
        self.tournament_k = tournament_k
        self.selection = selection

        self.generation = 0
        self.population = [init_fn() for _ in range(pop_size)]
        self.combined_fit = np.zeros(pop_size)

        self.best_obj = -np.inf
        self.best_genome = None
        self.history = []

    def _tournament(self):
        idx = np.random.choice(self.pop_size, self.tournament_k, replace=False)
        return self.population[idx[np.argmax(self.combined_fit[idx])]].copy()

    def _roulette(self):
        order = np.argsort(self.combined_fit)
        ranks = np.arange(1, self.pop_size + 1, dtype=float)
        probs = ranks / ranks.sum()
        idx = np.random.choice(self.pop_size, p=probs)
        return self.population[order[idx]].copy()

    def _select(self):
        return self._roulette() if self.selection == 'roulette' else self._tournament()

    def _crossover(self, p1, p2):
        if np.random.random() < self.crossover_rate:
            mask = np.random.random(self.genome_size) < 0.5
            return np.where(mask, p1, p2)
        return p1.copy()

    def _mutate(self, g):
        g  = g.copy()
        mask = np.random.random(self.genome_size) < self.mutation_rate
        g[mask] += np.random.randn(mask.sum()) * self.mutation_scale
        return g

    def step(self, objectives, combined):
        self.combined_fit = np.array(combined, dtype=float)
        obj_arr = np.array(objectives, dtype=float)

        b = int(np.argmax(obj_arr))
        if obj_arr[b] > self.best_obj:
            self.best_obj    = float(obj_arr[b])
            self.best_genome = self.population[b].copy()

        stats = {
            "gen":      self.generation,
            "best_obj": self.best_obj,
            "gen_obj":  float(obj_arr[b]),
            "avg_obj":  float(np.mean(obj_arr)),
            "gen_comb": float(np.max(self.combined_fit)),
            "avg_comb": float(np.mean(self.combined_fit)),
        }
        self.history.append(stats)

        order = np.argsort(self.combined_fit)[::-1]
        new_pop = [self.population[i].copy() for i in order[:self.elite_count]]

        while len(new_pop) < self.pop_size:
            p1 = self._select()
            p2 = self._select()
            c  = self._crossover(p1, p2)
            c  = self._mutate(c)
            new_pop.append(c)

        self.population = new_pop
        self.generation += 1
        return stats
#EA variant used by ea_goal.py with curriculum goal rotation; simpler objective tracking without novelty
class EA_goal:
    def __init__(
        self, pop_size, genome_size, mutation_rate, mutation_scale,
        elite_count, crossover_rate=0.70, tournament_k=3, init_fn=None,
    ):
        self.pop_size = pop_size
        self.genome_size = genome_size
        self.mutation_rate = mutation_rate
        self.mutation_scale = mutation_scale
        self.crossover_rate = crossover_rate
        self.elite_count = elite_count
        self.tournament_k = tournament_k
        self.generation = 0
        self.best_fitness = -np.inf
        self.best_genome = None
        self.fitness_history: list = []

        make = init_fn if init_fn is not None else (
            lambda: np.random.randn(genome_size) * 0.4
        )
        self.population = [make() for _ in range(pop_size)]
        self.fitnesses  = np.zeros(pop_size)

    def _tournament_select(self):
        idx  = np.random.choice(self.pop_size, self.tournament_k, replace=False)
        best = idx[np.argmax(self.fitnesses[idx])]
        return self.population[best].copy()

    def _crossover(self, p1, p2):
        if np.random.random() < self.crossover_rate:
            mask = np.random.random(self.genome_size) < 0.5
            return np.where(mask, p1, p2)
        return p1.copy()

    def _mutate(self, genome):
        genome = genome.copy()
        mask   = np.random.random(self.genome_size) < self.mutation_rate
        genome[mask] += np.random.randn(mask.sum()) * self.mutation_scale
        return genome

    def evolve(self, fitnesses):
        self.fitnesses = np.array(fitnesses, dtype=float)

        best_idx = int(np.argmax(self.fitnesses))
        if self.fitnesses[best_idx] > self.best_fitness:
            self.best_fitness = float(self.fitnesses[best_idx])
            self.best_genome  = self.population[best_idx].copy()

        stats = {
            "gen":          self.generation,
            "best_alltime": float(self.best_fitness),
            "best_gen":     float(self.fitnesses[best_idx]),
            "avg":          float(np.mean(self.fitnesses)),
            "worst":        float(np.min(self.fitnesses)),
        }
        self.fitness_history.append(stats)

        sorted_idx = np.argsort(self.fitnesses)[::-1]
        new_pop = [self.population[i].copy() for i in sorted_idx[:self.elite_count]]

        while len(new_pop) < self.pop_size:
            p1    = self._tournament_select()
            p2    = self._tournament_select()
            child = self._crossover(p1, p2)
            child = self._mutate(child)
            new_pop.append(child)

        self.population = new_pop
        self.generation += 1
        return stats
