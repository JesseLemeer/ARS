import pygame
import sys
import math
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import motionmodel as mm
import map as mp
import filter as kf
from ellipse import draw_covariance_ellipse
from occupancygrid import OccupancyGrid

# Parameters
WHITE   = (255, 255, 255)
BLACK   = (  0,   0,   0)
ORANGE  = (255, 127,   0)
GREEN   = (  0, 200,   0)
RED     = (200,   0,   0)
BLUE    = ( 30,  80, 200)
YELLOW  = (220, 200,   0)
CYAN    = (  0, 200, 200)
PATROL_COL = (180,  60, 200)  # purple (patrol robot)

HUD_BG    = ( 10,  10,  25, 210)
HUD_TEXT  = (220, 220, 255)
WARN_COL  = (255, 200,  50)
TITLE_COL = (150, 200, 255)

SCREEN_W, SCREEN_H = 960, 700
TRAIL_LEN          = 350

CAR_LENGTH = 24
CAR_WIDTH  = 14
OMEGA      = 5.0
VELOCITY   = 100.0
FIXED_DT   = 1.0 / 60.0

START_X, START_Y, START_THETA = -300.0, 50.0, 0.0


# Set MODE = "RECORD" to drive and save a path (press S to save), then switch to "REPLAY".
MODE            = "REPLAY"
TRAJECTORY_FILE = "trajectory.npy"
RECORD_PATH: list = []
REPLAY_INDEX: int  = 0

SUMMARY_DIR = "phase3_summary_plots"


# KF DEFAULTS  (same as phase 2)
DEFAULT_KF = dict(
    sigma_sq_x      = 25.0,
    sigma_sq_y      = 25.0,
    sigma_sq_theta  = math.radians(10.0) ** 2,
    sigma_sq_Rx     = 2.0,
    sigma_sq_Ry     = 2.0,
    sigma_sq_Rtheta = math.radians(2.0) ** 2,
    sigma_sq_Qx     = 16.0,
    sigma_sq_Qy     = 16.0,
    sigma_sq_Qtheta = math.radians(8.0) ** 2,
)

# Default occupancy-grid world bounds (matches original map)
DEFAULT_GRID = dict(
    x_min=-600.0, x_max=450.0,
    y_min=-400.0, y_max=400.0,
    cell_size=10.0,
    p_occ=0.70, p_free=0.30, p_prior=0.50,
    l_max=5.0,  l_min=-5.0,
)


#  DYNAMIC OBSTACLE HELPERS

#A second robot that moves along a list of waypoints in a loop.
class PatrolRobot:

    def __init__(self, waypoints, speed: float = 70.0,
                 width: float = 18.0, height: float = 24.0):
        self.waypoints = waypoints
        self.speed     = speed
        self.w         = width
        self.h         = height
        self.wp_idx    = 0
        self.x, self.y = float(waypoints[0][0]), float(waypoints[0][1])
        self.theta      = 0.0

    def update(self, dt: float):
        tx, ty = self.waypoints[self.wp_idx]
        dx, dy = tx - self.x, ty - self.y
        dist   = math.hypot(dx, dy)
        if dist < 6.0:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
        else:
            self.theta = math.atan2(dy, dx)
            self.x += (dx / dist) * self.speed * dt
            self.y += (dy / dist) * self.speed * dt

    def get_wall_segments(self) -> list:
        """Return the four edges of the robot as wall-like line segments."""
        hw, hh = self.w / 2, self.h / 2
        cos_t, sin_t = math.cos(self.theta), math.sin(self.theta)
        # Local corners (front-right, front-left, rear-left, rear-right)
        local = [( hh,  hw), ( hh, -hw), (-hh, -hw), (-hh,  hw)]
        world_corners = [
            (self.x + cos_t * lx - sin_t * ly,
             self.y + sin_t * lx + cos_t * ly)
            for lx, ly in local
        ]
        n = len(world_corners)
        return [(world_corners[i], world_corners[(i + 1) % n]) for i in range(n)]

    def draw(self, surface, world_to_screen_fn):
        segs = self.get_wall_segments()
        pts  = [world_to_screen_fn(x, y, SCREEN_W, SCREEN_H)
                for (x, y), _ in segs]
        pts.append(pts[0])
        for i in range(len(pts) - 1):
            pygame.draw.line(surface, PATROL_COL, pts[i], pts[i + 1], 3)
        # direction arrow
        fx = self.x + (self.h / 2) * math.cos(self.theta)
        fy = self.y + (self.h / 2) * math.sin(self.theta)
        cx_s = world_to_screen_fn(self.x, self.y,  SCREEN_W, SCREEN_H)
        fx_s = world_to_screen_fn(fx,     fy,       SCREEN_W, SCREEN_H)
        pygame.draw.line(surface, WHITE, cx_s, fx_s, 2)




# EXPERIMENT DEFINITIONS

EXPERIMENTS = [
    {
        "id":   "A1",
        "name": "1 – Baseline (cell=10)",
        "desc": [
            "Default 10 × 10 unit cells.",
            "Standard map extent.",
            "Reference for all resolution comparisons.",
            "Expect: balanced accuracy vs speed.",
        ],
        "grid_overrides": {},
        "dynamic": None,
        "decay": 1.0,
    },
    {
        "id":   "A2",
        "name": "2 – Coarse Grid (cell=30)",
        "desc": [
            "Cells are 3× larger (30 units).",
            "Map uses ~11% the memory of A1.",
            "Walls appear as thick blobs.",
            "Expect: blocky map, fast update.",
        ],
        "grid_overrides": {"cell_size": 30.0},
        "dynamic": None,
        "decay": 1.0,
    },
    {
        "id":   "A3",
        "name": "3 – Fine Grid (cell=5)",
        "desc": [
            "Cells are 2× smaller (5 units).",
            "Map uses 4× the memory of A1.",
            "Sharp wall edges, slower update.",
            "Expect: crisp map, higher CPU cost.",
        ],
        "grid_overrides": {"cell_size": 5.0},
        "dynamic": None,
        "decay": 1.0,
    },
    
    {
        "id":   "4",
        "name": "4 – Moving Robot (Obstacle)",
        "desc": [
            "A patrol robot orbits a fixed path.",
            "Sensor sees it as a solid obstacle.",
            "No log-odds decay → ghost persists.",
            "Expect: smeared trail in the grid.",
        ],
        "grid_overrides": {},
        "dynamic": "patrol",
        "decay": 1.0,
    },
    
    {
        "id":   "5",
        "name": "5 – Softer Sensor Model (p_occ=0.6, p_free=0.4)",
        "desc": [
            "Same setup as baseline (A1).",
            "Lower confidence in sensor updates.",
            "Log-odds changes more gradually.",
            "Expect: smoother map, slower convergence.",
        ],
        "grid_overrides": {
            "p_occ": 0.60,
            "p_free": 0.40,
        },
        "dynamic": None,
        "decay": 1.0,
    },
    {
        "id":   "6",
        "name": "6 – Map Memory Decay (log-odds fading)",
        "desc": [
            "Same setup as baseline (A1).",
            "Applies exponential log-odds decay each frame.",
            "Old observations gradually fade from the map.",
            "Reduces ghosting from moving objects.",
            "Expect: more adaptive but less stable map representation.",
        ],
        "grid_overrides": {},
        "dynamic": None,
        "decay": 0.998,
},
    
]



# Patrol waypoints 
PATROL_WAYPOINTS = [
    ( 0,  -190), ( -50, -260), ( 0, -315),
    ( 50,-260), (  0,-190),
]





# FACTORY HELPERS

def make_grid(exp_index: int) -> OccupancyGrid:
    cfg = {**DEFAULT_GRID, **EXPERIMENTS[exp_index]["grid_overrides"]}
    return OccupancyGrid(**cfg)


def make_dynamics(exp_index: int):
    if EXPERIMENTS[exp_index]["dynamic"]== "patrol":
        return PatrolRobot(PATROL_WAYPOINTS, speed =70.0)
    return None
   


def reset_state(exp_index: int):
    """Reset robot, KF, grid and dynamic objects for a fresh run."""
    global REPLAY_INDEX

    REPLAY_INDEX = 0

    if RECORD_PATH:
        mm.x, mm.y, mm.theta, mm.v, mm.omega, _ = RECORD_PATH[0]
    else:
        mm.x, mm.y, mm.theta = START_X, START_Y, START_THETA
        mm.v = mm.omega = 0.0

    mm.dt = 0.0

    grid = make_grid(exp_index)
    patrol = make_dynamics(exp_index)

    return (
        mm.x, mm.y, mm.theta,         # kf estimate
        25.0, 25.0,                    # kf_sxx, kf_syy
        math.radians(10.0) ** 2,       # kf_stt
        0.0,                           # kf_sxy
        [], [],                        # true_trail, kf_trail
        grid, patrol, 
        [],                            # error_history
        [],                            # explored_history
    )



# DRAWING HELPERS

def draw_solid_trail(surface, trail, color, width=2):
    if len(trail) < 2:
        return
    pts = [mm.world_to_screen(px, py, SCREEN_W, SCREEN_H) for px, py in trail]
    pygame.draw.lines(surface, color, False, pts, width)


def draw_dashed_trail(surface, trail, color, gap=4, width=2):
    if len(trail) < 2:
        return
    pts = [mm.world_to_screen(px, py, SCREEN_W, SCREEN_H) for px, py in trail]
    for i in range(0, len(pts) - 1, gap * 2):
        j = min(i + gap, len(pts) - 1)
        pygame.draw.line(surface, color, pts[i], pts[j], width)


def draw_hud(surface, exp_idx, visible_lm,
             true_pos, kf_pos, kf_sigma,
             error_hist, explored_hist,
             grid: OccupancyGrid, decay,
             font_sm, font_md):
    """Right-side HUD panel."""
    pw = 290
    px = SCREEN_W - pw

    panel = pygame.Surface((pw, SCREEN_H), pygame.SRCALPHA)
    panel.fill(HUD_BG)
    surface.blit(panel, (px, 0))
    pygame.draw.line(surface, (80, 80, 120), (px, 0), (px, SCREEN_H), 2)

    y = [10]
    row = 19

    def txt(text, color=HUD_TEXT, bold=False):
        fnt = font_md if bold else font_sm
        surface.blit(fnt.render(text, True, color), (px + 7, y[0]))
        y[0] += row

    def sep():
        pygame.draw.line(surface, (70, 70, 100),
                         (px + 4, y[0]), (SCREEN_W - 4, y[0]), 1)
        y[0] += 5

    # ── experiment info ──
    exp = EXPERIMENTS[exp_idx]
    txt("EXPERIMENT", color=TITLE_COL, bold=True)
    txt(exp["id"], color=WARN_COL, bold=True)
    for part in exp["name"].split("–")[1:]:
        txt(part.strip(), color=WARN_COL)
    y[0] += 3
    for line in exp["desc"]:
        txt(line[:36], color=(190, 190, 210))
        y[0] -= 3
    y[0] += 6

    sep()

    # ── grid info ──
    txt("GRID CONFIG", color=TITLE_COL, bold=True)
    txt(f"cell size : {grid.cell_size:.0f} units")
    txt(f"dims      : {grid.cols} × {grid.rows}")
    txt(f"total cells: {grid.cols * grid.rows:,}")
    if decay < 1.0:
        txt(f"decay/frame: {decay:.3f}", color=WARN_COL)
    else:
        txt("decay     : none")
    y[0] += 4

    sep()

    # ── live stats ──
    txt("LIVE STATS", color=TITLE_COL, bold=True)
    err = math.hypot(true_pos[0] - kf_pos[0], true_pos[1] - kf_pos[1])
    mean_err = (sum(error_hist[-200:]) / max(len(error_hist[-200:]), 1))

    occ   = int(np.sum(grid.log_odds >  0.1))
    free  = int(np.sum(grid.log_odds < -0.1))
    total = grid.rows * grid.cols
    expl  = 100.0 * (occ + free) / max(total, 1)

    for s in [
        f"LM visible : {visible_lm}",
        f"KF error   : {err:.1f}",
        f"Mean err   : {mean_err:.1f}",
        f"σx²        : {kf_sigma[0]:.1f}",
        f"σy²        : {kf_sigma[1]:.1f}",
        f"Occ cells  : {occ:,}",
        f"Free cells : {free:,}",
        f"Explored % : {expl:.1f}%",
    ]:
        txt(s)

    y[0] += 4
    sep()

    # ── controls ──
    txt("CONTROLS", color=TITLE_COL, bold=True)
    for c in [
        "Arrows : drive",
        "E      : next experiment",
        "R      : reset",
        "D      : toggle map",
        "S      : save trajectory",
        "Q      : quit + save plots",
    ]:
        txt(c, color=(155, 155, 175))


# OFFLINE SIMULATION (for summary plots)

def run_offline_experiment(exp_index: int,
                            walls, landmarks, landmark_groups,
                            seed: int = 42):
    """
    Headless replay of the recorded trajectory with a given experiment config.
    Returns a dict with trajectory, errors, explored %, and grid snapshots.
    """
    if not RECORD_PATH:
        return None

    np.random.seed(seed + exp_index)

    saved = (mm.x, mm.y, mm.theta, mm.v, mm.omega, mm.dt)

    grid   = make_grid(exp_index)
    patrol = make_dynamics(exp_index)
    decay  = EXPERIMENTS[exp_index]["decay"]

    kf_x, kf_y, kf_t = START_X, START_Y, START_THETA
    sxx, syy, stt, sxy = 25.0, 25.0, math.radians(10.0) ** 2, 0.0
    p = DEFAULT_KF

    result = {
        "name": EXPERIMENTS[exp_index]["name"],
        "id":   EXPERIMENTS[exp_index]["id"],
        "true_traj":   [],
        "kf_traj":     [],
        "errors":      [],
        "explored_pct":[],
        "occ_cells":   [],
        "cell_size":   grid.cell_size,
        "decay":       decay,
    }


    for x, y, theta, v, omega, dt in RECORD_PATH:
        mm.x, mm.y, mm.theta = x, y, theta
        mm.v, mm.omega, mm.dt = v, omega, dt

        # Update dynamic objects
        dynamic_segs = []
        if patrol:
            patrol.update(dt)
            dynamic_segs.extend(patrol.get_wall_segments())
        

        # Sensor readings (walls + dynamic obstacle)
        sensor_walls = walls + dynamic_segs
        wall_readings = mm.get_sensor_readings(sensor_walls)
        lm_readings   = mm.get_landmark_measurements(landmark_groups)
        visible       = [r for r in lm_readings
                         if r["distance"] < mm.LANDMARK_SENSOR_RANGE]

        # KF
        mu, sigma_mat = kf.kalman_filter(
            kf_x, kf_y, kf_t, sxx, syy, stt,
            p["sigma_sq_Rx"], p["sigma_sq_Ry"], p["sigma_sq_Rtheta"],
            p["sigma_sq_Qx"], p["sigma_sq_Qy"], p["sigma_sq_Qtheta"],
            v, omega, dt, visible,
        )
        kf_x = float(mu[0, 0]); kf_y = float(mu[1, 0]); kf_t = float(mu[2, 0])
        sxx  = float(sigma_mat[0, 0])
        syy  = float(sigma_mat[1, 1])
        stt  = float(sigma_mat[2, 2])
        sxy  = float(sigma_mat[0, 1])

        # Grid update
        grid.update(robot_x=kf_x, robot_y=kf_y,
                    sensor_readings=wall_readings,
                    max_range=mm.SENSOR_MAX_RANGE)

        # Log-odds decay for dynamic experiments
        if decay < 1.0:
            grid.log_odds *= decay

        # Logging
        occ   = int(np.sum(grid.log_odds >  0.1))
        free  = int(np.sum(grid.log_odds < -0.1))
        total = grid.rows * grid.cols
        expl  = 100.0 * (occ + free) / max(total, 1)

        result["true_traj"].append((x, y))
        result["kf_traj"].append((kf_x, kf_y))
        result["errors"].append(math.hypot(x - kf_x, y - kf_y))
        result["explored_pct"].append(expl)
        result["occ_cells"].append(occ)
        result["final_grid"] = grid 

    mm.x, mm.y, mm.theta, mm.v, mm.omega, mm.dt = saved
    return result


# SUMMARY PLOT SAVERS

def ensure_dir():
    os.makedirs(SUMMARY_DIR, exist_ok=True)


def save_explored_comparison(results_A):
    """Bar chart: % of map explored at end of run, per resolution."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    labels = [r["id"] for r in results_A]
    cell_sizes = [r["cell_size"] for r in results_A]
    final_expl = [r["explored_pct"][-1] if r["explored_pct"] else 0.0
                  for r in results_A]
    final_occ  = [r["occ_cells"][-1]  if r["occ_cells"] else 0
                  for r in results_A]

    axes[0].bar(labels, final_expl, color="steelblue")
    axes[0].set_title("Explored % at end of run (Resolution group)")
    axes[0].set_ylabel("Explored %")
    axes[0].set_xlabel("Experiment")
    axes[0].set_ylim(0, 100)
    for i, v in enumerate(final_expl):
        axes[0].text(i, v + 1, f"{v:.1f}%", ha="center", fontsize=9)

    axes[1].bar(labels, cell_sizes, color="coral")
    axes[1].set_title("Cell size per experiment")
    axes[1].set_ylabel("Cell size (world units)")
    axes[1].set_xlabel("Experiment")
    for i, v in enumerate(cell_sizes):
        axes[1].text(i, v + 0.5, f"{v:.0f}", ha="center", fontsize=9)

    fig.tight_layout()
    path = os.path.join(SUMMARY_DIR, "A_resolution_comparison.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Saved] {path}")


def save_explored_over_time(all_results):
    fig, ax = plt.subplots(figsize=(11, 5))
    for r in all_results:
        if r and r["explored_pct"]:
            ax.plot(r["explored_pct"], lw=1.6, label=r["name"])
    ax.set_title("Explored % of Grid Over Time – All Experiments")
    ax.set_xlabel("Replay step")
    ax.set_ylabel("Explored %")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    path = os.path.join(SUMMARY_DIR, "all_explored_over_time.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Saved] {path}")


def save_error_comparison(all_results):
    fig, ax = plt.subplots(figsize=(11, 5))
    for r in all_results:
        if r and r["errors"]:
            ax.plot(r["errors"], lw=1.4, label=r["name"])
    ax.set_title("KF Position Error Over Time – All Experiments")
    ax.set_xlabel("Replay step")
    ax.set_ylabel("Euclidean error (world units)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    path = os.path.join(SUMMARY_DIR, "all_position_error.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Saved] {path}")


def save_grid_snapshot(grid: OccupancyGrid, title: str, filename: str):
    """Save a top-down matplotlib render of a finished occupancy grid."""
    prob = 1.0 - 1.0 / (1.0 + np.exp(grid.log_odds.astype(np.float64)))
    grey = (1.0 - prob)   # high occ → dark

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.imshow(grey, cmap="gray", vmin=0.0, vmax=1.0,
              origin="lower",
              extent=[grid.x_min, grid.x_max, grid.y_min, grid.y_max])
    ax.set_title(title)
    ax.set_xlabel("x (world units)")
    ax.set_ylabel("y (world units)")
    path = os.path.join(SUMMARY_DIR, filename)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Saved] {path}")

def save_occ_over_time(all_results):
    fig, ax = plt.subplots(figsize=(11, 5))

    for r in all_results:
        ax.plot(r["occ_cells"], label=r["name"])

    ax.set_title("Occupied Cells Over Time – All Experiments")
    ax.set_xlabel("Replay step")
    ax.set_ylabel("Occupied cells")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    path = os.path.join(SUMMARY_DIR, "all_occ_over_time.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"[Saved] {path}")


def print_stats_table(all_results):
    print("PHASE 3 SUMMARY STATISTICS")
    hdr = f"{'ID':<5} {'Experiment':<35} {'Mean err':>9} {'P95 err':>9} {'Expl %':>8} {'Cell':>5}"
    print(hdr)
    print("-" * 85)
    for r in all_results:
        if r is None:
            continue
        errs   = r["errors"]
        expl   = r["explored_pct"]
        me     = np.mean(errs)    if errs else float("nan")
        p95 = np.percentile(errs, 95)
        ex_f   = expl[-1]         if expl else float("nan")
        csz    = r["cell_size"]
        print(f"{r['id']:<5} {r['name'][:34]:<35} {me:>9.2f} {p95:>9.2f} {ex_f:>8.1f} {csz:>5.0f}")
  


def run_and_save_all_summaries(walls, landmarks, landmark_groups):
    ensure_dir()
    print("\nRunning all offline experiments")
    all_results = []

    for i in range(len(EXPERIMENTS)):
        print(f"  [{i+1}/{len(EXPERIMENTS)}]  {EXPERIMENTS[i]['name']}")
        r = run_offline_experiment(i, walls, landmarks, landmark_groups)
        all_results.append(r)

    # Save map 
        if r:
            save_grid_snapshot(
                r["final_grid"],
                title=f"{r['name']} – Final Map",
                filename=f"{r['id']}_map.png"
            )

    # Group A plots
    results_A = [r for r in all_results if r and r["id"].startswith("A")]
    if results_A:
        save_explored_comparison(results_A)
    
   

    save_explored_over_time(all_results)
    save_error_comparison(all_results)
    print_stats_table(all_results)
    print(f"\n[Done] All plots written to ./{SUMMARY_DIR}/")


#MAIN LOOP
def main():
    global RECORD_PATH, REPLAY_INDEX, MODE

    pygame.init()
    screen  = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("ARS Group 21 – Phase 3 Experiments")
    clock   = pygame.time.Clock()
    font_sm = pygame.font.SysFont("monospace", 14)
    font_md = pygame.font.SysFont("monospace", 16, bold=True)

    # Load map
    _res = mp.create_map()
    if len(_res) == 3:
        walls, landmarks, landmark_groups = _res
    else:
        walls, landmarks = _res
        landmark_groups  = []
    obstacles_static = walls + landmarks

    # Load / init trajectory
    if MODE == "REPLAY":
        if os.path.exists(TRAJECTORY_FILE):
            RECORD_PATH = np.load(TRAJECTORY_FILE, allow_pickle=True).tolist()
            print(f"[Loaded] {len(RECORD_PATH)} trajectory points")
        else:
            print("[Warning] No trajectory found – switching to RECORD mode")
            MODE = "RECORD"

    # Initial experiment
    current_exp = 0
    (kf_x, kf_y, kf_t,
     sxx, syy, stt, sxy,
     true_trail, kf_trail,
     grid, patrol,
     error_hist, explored_hist) = reset_state(current_exp)

    show_map        = True
    save_plots_now  = False
    collisions      = False

    running = True
    while running:

        # EVENTS 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                k = event.key

                if k == pygame.K_q:
                    running = False

                elif k == pygame.K_d:
                    show_map = not show_map

                elif k == pygame.K_p:
                    save_plots_now = True

                elif k == pygame.K_s and MODE == "RECORD":
                    np.save(TRAJECTORY_FILE,
                            np.array(RECORD_PATH, dtype=object))
                    print(f"[Saved] trajectory ({len(RECORD_PATH)} pts)")

                elif k == pygame.K_r:
                    (kf_x, kf_y, kf_t,
                     sxx, syy, stt, sxy,
                     true_trail, kf_trail,
                     grid, patrol,
                     error_hist, explored_hist) = reset_state(current_exp)
                    print(f"[Reset] {EXPERIMENTS[current_exp]['name']}")

                elif k == pygame.K_e:
                    current_exp = (current_exp + 1) % len(EXPERIMENTS)
                    (kf_x, kf_y, kf_t,
                     sxx, syy, stt, sxy,
                     true_trail, kf_trail,
                     grid, patrol,
                     error_hist, explored_hist) = reset_state(current_exp)
                    print(f"[Switch] {EXPERIMENTS[current_exp]['name']}")


        # ROBOT MOTION 
        if MODE == "REPLAY":
            if REPLAY_INDEX < len(RECORD_PATH):
                x, y, theta, v, omega, dt = RECORD_PATH[REPLAY_INDEX]
                mm.x, mm.y, mm.theta = x, y, theta
                mm.v, mm.omega, mm.dt = v, omega, dt
                REPLAY_INDEX += 1
            clock.tick(60)

        else:  # RECORD
            keys = pygame.key.get_pressed()
            mm.omega = (OMEGA if keys[pygame.K_LEFT]
                        else -OMEGA if keys[pygame.K_RIGHT] else 0.0)
            mm.v     = (VELOCITY if keys[pygame.K_UP]
                        else -VELOCITY if keys[pygame.K_DOWN] else 0.0)
            clock.tick(60)
            mm.dt      = FIXED_DT
            collisions = mm.update(obstacles_static, CAR_LENGTH, CAR_WIDTH)
            RECORD_PATH.append((mm.x, mm.y, mm.theta,
                                 mm.v, mm.omega, mm.dt))

        dt = mm.dt if mm.dt > 0 else FIXED_DT

        # DYNAMIC OBJECTS 
        dynamic_segs: list = []
        if patrol:
            patrol.update(dt)
            dynamic_segs.extend(patrol.get_wall_segments())
        

        #SENSOR READINGS
        sensor_walls  = walls + dynamic_segs
        wall_readings = mm.get_sensor_readings(sensor_walls)
        lm_readings   = mm.get_landmark_measurements(landmark_groups)
        visible       = [r for r in lm_readings
                         if r["distance"] < mm.LANDMARK_SENSOR_RANGE]

        #KALMAN FILTER 
        p = DEFAULT_KF
        kf_mu, kf_sigma_mat = kf.kalman_filter(
            kf_x, kf_y, kf_t, sxx, syy, stt,
            p["sigma_sq_Rx"], p["sigma_sq_Ry"], p["sigma_sq_Rtheta"],
            p["sigma_sq_Qx"], p["sigma_sq_Qy"], p["sigma_sq_Qtheta"],
            mm.v, mm.omega, mm.dt, visible,
        )
        kf_x = float(kf_mu[0, 0])
        kf_y = float(kf_mu[1, 0])
        kf_t = float(kf_mu[2, 0])
        sxx  = float(kf_sigma_mat[0, 0])
        syy  = float(kf_sigma_mat[1, 1])
        stt  = float(kf_sigma_mat[2, 2])
        sxy  = float(kf_sigma_mat[0, 1])

        # OCCUPANCY GRID
        grid.update(robot_x=kf_x, robot_y=kf_y,
                    sensor_readings=wall_readings,
                    max_range=mm.SENSOR_MAX_RANGE)

        decay = EXPERIMENTS[current_exp]["decay"]
        if decay < 1.0:
            grid.log_odds *= decay          # per-frame forgetting

        #TRAILS & HISTORY
        true_trail.append((mm.x, mm.y))
        kf_trail.append((kf_x, kf_y))
        if len(true_trail) > TRAIL_LEN: true_trail.pop(0)
        if len(kf_trail)   > TRAIL_LEN: kf_trail.pop(0)

        err = math.hypot(mm.x - kf_x, mm.y - kf_y)
        error_hist.append(err)

        occ  = int(np.sum(grid.log_odds >  0.1))
        free = int(np.sum(grid.log_odds < -0.1))
        expl = 100.0 * (occ + free) / max(grid.rows * grid.cols, 1)
        explored_hist.append(expl)

        # DRAW 
        screen.fill(WHITE)

        # Occupancy grid
        if show_map:
            grid.draw(
                surface=screen,
                world_to_screen_fn=mm.world_to_screen,
                screen_width=SCREEN_W,
                screen_height=SCREEN_H,
                robot_x=mm.x,
                robot_y=mm.y,
            )

        # Static walls & landmarks
        for seg in obstacles_static:
            s = mm.world_to_screen(seg[0][0], seg[0][1], SCREEN_W, SCREEN_H)
            e = mm.world_to_screen(seg[1][0], seg[1][1], SCREEN_W, SCREEN_H)
            pygame.draw.line(screen, BLACK, s, e, 2)


        # Patrol robot
        if patrol:
            patrol.draw(screen, mm.world_to_screen)

        # Sensor rays
        sx, sy = mm.world_to_screen(mm.x, mm.y, SCREEN_W, SCREEN_H)
        for rd in wall_readings:
            hx, hy = rd["hit_point"]
            hsx, hsy = mm.world_to_screen(hx, hy, SCREEN_W, SCREEN_H)
            pygame.draw.line(screen, (200, 200, 225), (sx, sy), (hsx, hsy), 1)

        # Landmark rays
        for rd in lm_readings:
            lx, ly = rd["landmark_center"]
            lsx, lsy = mm.world_to_screen(lx, ly, SCREEN_W, SCREEN_H)
            used = any(v["landmark_id"] == rd["landmark_id"] for v in visible)
            col  = GREEN if used else (200, 200, 0)
            pygame.draw.line(screen, col, (sx, sy), (lsx, lsy), 1)
            pygame.draw.circle(screen, col, (lsx, lsy), 4)

        # Trails
        draw_solid_trail(screen, true_trail, BLUE, 2)
        draw_dashed_trail(screen, kf_trail, ORANGE, gap=3, width=2)

        # True robot body
        robot_col   = RED if collisions else GREEN
        corners_w   = mm.get_robot_corners(CAR_LENGTH, CAR_WIDTH)
        corners_s   = [mm.world_to_screen(px, py, SCREEN_W, SCREEN_H)
                       for px, py in corners_w]
        pygame.draw.polygon(screen, robot_col, corners_s)
        pygame.draw.polygon(screen, BLACK, corners_s, 2)
        fx  = mm.x + (CAR_LENGTH / 2) * math.cos(mm.theta)
        fy  = mm.y + (CAR_LENGTH / 2) * math.sin(mm.theta)
        fxs, fys = mm.world_to_screen(fx, fy, SCREEN_W, SCREEN_H)
        pygame.draw.line(screen, BLACK, (sx, sy), (fxs, fys), 2)

        # KF estimate robot (orange outline)
        kf_sx, kf_sy = mm.world_to_screen(kf_x, kf_y, SCREEN_W, SCREEN_H)
        kf_corners_w = mm.get_robot_corners_at(kf_x, kf_y, kf_t,
                                                CAR_LENGTH, CAR_WIDTH)
        kf_corners_s = [mm.world_to_screen(px, py, SCREEN_W, SCREEN_H)
                        for px, py in kf_corners_w]
        pygame.draw.polygon(screen, ORANGE, kf_corners_s, 2)
        kfx = kf_x + (CAR_LENGTH / 2) * math.cos(kf_t)
        kfy = kf_y + (CAR_LENGTH / 2) * math.sin(kf_t)
        kfxs, kfys = mm.world_to_screen(kfx, kfy, SCREEN_W, SCREEN_H)
        pygame.draw.line(screen, ORANGE, (kf_sx, kf_sy), (kfxs, kfys), 2)

        # Covariance ellipse
        draw_covariance_ellipse(
            surface=screen,
            kf_est_x=kf_x, kf_est_y=kf_y,
            sigma_xx=sxx, sigma_yy=syy, sigma_xy=sxy,
            world_to_screen=mm.world_to_screen,
            screen_width=SCREEN_W, screen_height=SCREEN_H,
            color=ORANGE, n_std=2, n_points=64, thickness=2,
        )

        # Legend (bottom-left)
        legend = [
            (BLUE,       "── True path"),
            (ORANGE,     "--- KF estimate"),
            (GREEN,      "●  Landmark (used)"),
            (YELLOW,     "●  Landmark (suppressed)"),
            (ORANGE,     "○  Cov. ellipse 2σ"),
            (PATROL_COL, "■  Patrol robot"),
        ]
        lx0 = 8
        ly0 = SCREEN_H - len(legend) * 17 - 6
        for col, label in legend:
            surface = font_sm.render(label, True, col)
            screen.blit(surface, (lx0, ly0))
            ly0 += 17

        # HUD panel
        draw_hud(screen, current_exp, len(visible),
                 (mm.x, mm.y), (kf_x, kf_y), (sxx, syy, stt),
                 error_hist, explored_hist, grid, decay,
                 font_sm, font_md)

        pygame.display.flip()

        if save_plots_now:
            save_plots_now = False
            run_and_save_all_summaries(walls, landmarks, landmark_groups)

    #QUIT 
    if RECORD_PATH:
        run_and_save_all_summaries(walls, landmarks, landmark_groups)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()