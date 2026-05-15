"""
experiments.py  –  ARS Group 21
================================
Record a trajectory once (MODE = "RECORD", drive around, press S to save),
then replay it through all nine experiments and save comparison plots + table.

Experiment groups
-----------------
  Group A – Baseline        (same trajectory, no perturbations)
    1 – KF baseline
    2 – EKF baseline
    3 – EKF-SLAM baseline

  Group B – Sensor Noise    (distance + bearing bias/noise added to measurements)
    4 – KF   + sensor noise
    5 – EKF  + sensor noise
    6 – SLAM + sensor noise

  Group C – Moving Obstacle (a second robot follows a fixed loop)
    7 – KF   + moving obstacle
    8 – EKF  + moving obstacle
    9 – SLAM + moving obstacle

Controls
--------
  Arrow keys  – drive (RECORD mode only)
  S           – save recorded trajectory
  E           – next experiment
  1-9         – jump to experiment
  R           – reset current experiment
  P           – generate & save all summary plots now
  Q           – quit (auto-saves plots on exit)
"""

import pygame
import sys
import math
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse as MplEllipse

import motionmodel as mm
import map as mp
import filter as kf
from ellipse import draw_covariance_ellipse, get_ellipse_axes
from occupancygrid import OccupancyGrid

# ── colours ───────────────────────────────────────────────────────────────────
ORANGE  = (255, 127,   0)
BLACK   = (  0,   0,   0)
BLUE    = ( 70, 130, 180)
RED     = (200,   0,   0)
GREEN   = (  0, 200,   0)
WHITE   = (255, 255, 255)
PURPLE  = (160,  32, 240)
HUD_COLOR  = (220, 220, 255)
WARN_COLOR = (255, 200,  50)

# ── simulation constants ──────────────────────────────────────────────────────
OMEGA      = 5.0
VELOCITY   = 100.0
FIXED_DT   = 1.0 / 60.0
CAR_LENGTH = 24
CAR_WIDTH  = 14

SCREEN_WIDTH  = 900
SCREEN_HEIGHT = 680
TRAIL_LEN     = 400

START_X, START_Y, START_THETA = 0.0, 0.0, 0.0

# ── noise parameters (mirrors main.py defaults) ───────────────────────────────
KF_SIGMA_SQ_X      = 25.0
KF_SIGMA_SQ_Y      = 25.0
KF_SIGMA_SQ_THETA  = math.radians(10.0) ** 2

KF_SIGMA_SQ_RX     = 2.0
KF_SIGMA_SQ_RY     = 2.0
KF_SIGMA_SQ_RTHETA = math.radians(2.0) ** 2

KF_SIGMA_SQ_QX     = 16.0
KF_SIGMA_SQ_QY     = 16.0
KF_SIGMA_SQ_QTHETA = math.radians(8.0) ** 2

EKF_SIGMA_SQ_R   = 4.0
EKF_SIGMA_SQ_PHI = math.radians(3.0) ** 2

# ── occupancy grid parameters (mirrors main.py) ──────────────────────────────
GRID_PARAMS = dict(
    x_min=-600.0, x_max=450.0,
    y_min=-400.0, y_max=400.0,
    cell_size=10.0,
    p_occ=0.70, p_free=0.30, p_prior=0.50,
    l_max=5.0,  l_min=-5.0,
)

# ── sensor noise parameters for Group B (mirrors phase-2 exp 3) ──────────────
NOISE_DIST_BIAS      =  2.0
NOISE_DIST_STD       =  4.0
NOISE_ANGLE_BIAS_DEG =  3.0
NOISE_ANGLE_STD_DEG  =  2.0

# ── patrol robot waypoints for Group C (mirrors phase-3) ────────────────────
PATROL_WAYPOINTS = [
    (  0.0, -190.0),
    (-50.0, -260.0),
    (  0.0, -315.0),
    ( 50.0, -260.0),
]
PATROL_SPEED = 70.0   # world-units per second (matches phase-3)

PATROL_COLOR = (180, 60, 200)   # purple, same as phase-3 PATROL_COL

# ── file paths ────────────────────────────────────────────────────────────────
TRAJECTORY_FILE = os.path.join(os.path.dirname(__file__), "trajectory.npy")
SUMMARY_DIR     = os.path.join(os.path.dirname(__file__), "summary_plots4")

# ── mode ──────────────────────────────────────────────────────────────────────
MODE = "REPLAY"   # "RECORD" or "REPLAY"


# =============================================================================
# PATROL ROBOT  (moving obstacle – mirrors phase-3 PatrolRobot exactly)
# =============================================================================

class PatrolRobot:
    """
    A second robot that follows a loop of waypoints at a fixed speed.
    Its four oriented wall segments are passed to get_sensor_readings so
    the occupancy-grid sensor sees it as a solid obstacle and leaves a
    smeared trail in the map.  Collisions are not enforced.
    Landmark measurements use get_landmark_measurements which is geometry-
    agnostic, so KF / EKF / SLAM are unaffected – exactly as in phase-3
    where S4 == S1.
    """
    def __init__(self, waypoints=None, speed: float = PATROL_SPEED,
                 width: float = 18.0, height: float = 24.0):
        self.waypoints = waypoints or PATROL_WAYPOINTS
        self.speed  = speed
        self.w      = width
        self.h      = height
        self.wp_idx = 0
        self.x, self.y = float(self.waypoints[0][0]), float(self.waypoints[0][1])
        self.theta  = 0.0

    def update(self, dt: float):
        tx, ty = self.waypoints[self.wp_idx]
        dx, dy = tx - self.x, ty - self.y
        dist   = math.hypot(dx, dy)
        if dist < 6.0:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
        else:
            self.theta  = math.atan2(dy, dx)
            self.x += (dx / dist) * self.speed * dt
            self.y += (dy / dist) * self.speed * dt

    def as_segments(self) -> list:
        """Four oriented edges of the robot body as wall-format segments."""
        hw, hh  = self.w / 2, self.h / 2
        cos_t, sin_t = math.cos(self.theta), math.sin(self.theta)
        local   = [( hh,  hw), ( hh, -hw), (-hh, -hw), (-hh,  hw)]
        corners = [
            (self.x + cos_t * lx - sin_t * ly,
             self.y + sin_t * lx + cos_t * ly)
            for lx, ly in local
        ]
        n = len(corners)
        return [(corners[i], corners[(i + 1) % n]) for i in range(n)]

    def draw(self, surface, world_to_screen_fn,
             screen_width: int, screen_height: int):
        """Draw the patrol robot body and a heading arrow."""
        segs = self.as_segments()
        pts  = [world_to_screen_fn(p[0], p[1], screen_width, screen_height)
                for p, _ in segs]
        pts.append(pts[0])
        for i in range(len(pts) - 1):
            pygame.draw.line(surface, PATROL_COLOR, pts[i], pts[i + 1], 3)
        # heading arrow
        fx  = self.x + (self.h / 2) * math.cos(self.theta)
        fy  = self.y + (self.h / 2) * math.sin(self.theta)
        cx_s = world_to_screen_fn(self.x, self.y, screen_width, screen_height)
        fx_s = world_to_screen_fn(fx,     fy,     screen_width, screen_height)
        pygame.draw.line(surface, WHITE, cx_s, fx_s, 2)


# =============================================================================
# EXPERIMENT DEFINITIONS
# =============================================================================

EXPERIMENTS = [
    # ── Group A: Baseline ─────────────────────────────────────────────────────
    {
        "id": "A_KF",   "group": "A", "filter": "kf",
        "name": "1 – KF Baseline",
        "color": ORANGE,
        "sensor_noise": False, "moving_obstacle": False,
        "desc": [
            "Standard Kalman Filter.",
            "Triangulation correction from",
            "landmark pairs. No perturbations.",
        ],
    },
    {
        "id": "A_EKF",  "group": "A", "filter": "ekf",
        "name": "2 – EKF Baseline",
        "color": BLUE,
        "sensor_noise": False, "moving_obstacle": False,
        "desc": [
            "Extended Kalman Filter.",
            "Range+bearing per landmark.",
            "No perturbations.",
        ],
    },
    {
        "id": "A_SLAM", "group": "A", "filter": "slam",
        "name": "3 – EKF-SLAM Baseline",
        "color": PURPLE,
        "sensor_noise": False, "moving_obstacle": False,
        "desc": [
            "EKF-SLAM. Jointly estimates robot",
            "pose and landmark map.",
            "No perturbations.",
        ],
    },

    # ── Group B: Sensor Noise ─────────────────────────────────────────────────
    {
        "id": "B_KF",   "group": "B", "filter": "kf",
        "name": "4 – KF + Sensor Noise",
        "color": ORANGE,
        "sensor_noise": True, "moving_obstacle": False,
        "desc": [
            "KF with biased+noisy measurements.",
            f"dist: bias={NOISE_DIST_BIAS}, std={NOISE_DIST_STD}",
            f"bearing: bias={NOISE_ANGLE_BIAS_DEG}deg, std={NOISE_ANGLE_STD_DEG}deg",
        ],
    },
    {
        "id": "B_EKF",  "group": "B", "filter": "ekf",
        "name": "5 – EKF + Sensor Noise",
        "color": BLUE,
        "sensor_noise": True, "moving_obstacle": False,
        "desc": [
            "EKF with biased+noisy measurements.",
            f"dist: bias={NOISE_DIST_BIAS}, std={NOISE_DIST_STD}",
            f"bearing: bias={NOISE_ANGLE_BIAS_DEG}deg, std={NOISE_ANGLE_STD_DEG}deg",
        ],
    },
    {
        "id": "B_SLAM", "group": "B", "filter": "slam",
        "name": "6 – SLAM + Sensor Noise",
        "color": PURPLE,
        "sensor_noise": True, "moving_obstacle": False,
        "desc": [
            "EKF-SLAM with biased+noisy measurements.",
            f"dist: bias={NOISE_DIST_BIAS}, std={NOISE_DIST_STD}",
            f"bearing: bias={NOISE_ANGLE_BIAS_DEG}deg, std={NOISE_ANGLE_STD_DEG}deg",
        ],
    },

    # ── Group C: Moving Obstacle ──────────────────────────────────────────────
    {
        "id": "C_KF",   "group": "C", "filter": "kf",
        "name": "7 – KF + Moving Obstacle",
        "color": ORANGE,
        "sensor_noise": False, "moving_obstacle": True,
        "desc": [
            "KF with a moving obstacle present.",
            "Obstacle segments enter sensor range.",
            "No known correspondence: bad corrections.",
        ],
    },
    
    {
        "id": "C_EKF",  "group": "C", "filter": "ekf",
        "name": "8 – EKF + Moving Obstacle",
        "color": BLUE,
        "sensor_noise": False, "moving_obstacle": True,
        "desc": [
            "EKF with a moving obstacle present.",
            "Obstacle visible but not a known landmark.",
            "Correction relies only on true landmarks.",
        ],
    },
    {
        "id": "C_SLAM", "group": "C", "filter": "slam",
        "name": "9 – SLAM + Moving Obstacle",
        "color": PURPLE,
        "sensor_noise": False, "moving_obstacle": True,
        "desc": [
            "EKF-SLAM with moving obstacle.",
            "No known correspondence: ignored.",
            "Expected: identical to SLAM baseline.",
        ],
    },
]

# pygame key → experiment index (K_1 … K_9)
_KEY_MAP = {pygame.K_1 + i: i for i in range(len(EXPERIMENTS))}


# =============================================================================
# SENSOR NOISE HELPER
# =============================================================================

def corrupt_measurements(readings: list) -> list:
    corrupted = []
    for r in readings:
        new_r = dict(r)
        new_r["distance"] = max(
            0.0,
            r["distance"]
            + NOISE_DIST_BIAS
            + np.random.normal(0.0, NOISE_DIST_STD),
        )
        new_r["bearing_rad"] = (
            r["bearing_rad"]
            + math.radians(NOISE_ANGLE_BIAS_DEG)
            + np.random.normal(0.0, math.radians(NOISE_ANGLE_STD_DEG))
        )
        corrupted.append(new_r)
    return corrupted


# =============================================================================
# STATE RESET
# =============================================================================

def reset_state(exp_index: int, record_path: list) -> dict:
    if record_path:
        mm.x, mm.y, mm.theta, mm.v, mm.omega, mm.dt = record_path[0]
    else:
        mm.x, mm.y, mm.theta = START_X, START_Y, START_THETA
        mm.v = mm.omega = mm.dt = 0.0

    return {
        "x":     mm.x,
        "y":     mm.y,
        "theta": mm.theta,
        "sigma": np.diag([KF_SIGMA_SQ_X, KF_SIGMA_SQ_Y, KF_SIGMA_SQ_THETA]),
        # EKF-SLAM extras
        "slam_mu":     np.array([mm.x, mm.y, mm.theta]),
        "slam_sigma":  np.diag([KF_SIGMA_SQ_X, KF_SIGMA_SQ_Y, KF_SIGMA_SQ_THETA]),
        "slam_lm_idx": {},
        # shared noise matrices
        "sigma_R": np.diag([KF_SIGMA_SQ_RX, KF_SIGMA_SQ_RY, KF_SIGMA_SQ_RTHETA]),
        "sigma_Q": np.diag([EKF_SIGMA_SQ_R, EKF_SIGMA_SQ_PHI]),
        # display trails
        "true_trail": [],
        "est_trail":  [],
        # live occupancy grid (reset fresh each experiment)
        "grid": OccupancyGrid(**GRID_PARAMS),
    }


# =============================================================================
# FILTER STEP
# =============================================================================

def filter_step(exp_index: int, state: dict,
                measurements: list, v: float, omega: float, dt: float):
    """One predict+correct tick. Returns (ex, ey, et, sxx, syy, sxy)."""
    ftype = EXPERIMENTS[exp_index]["filter"]

    if ftype == "kf":
        mu, sigma = kf.kalman_filter(
            state["x"], state["y"], state["theta"],
            state["sigma"],
            KF_SIGMA_SQ_RX, KF_SIGMA_SQ_RY, KF_SIGMA_SQ_RTHETA,
            KF_SIGMA_SQ_QX, KF_SIGMA_SQ_QY, KF_SIGMA_SQ_QTHETA,
            v, omega, dt,
            measurements,
        )
        state["x"]     = float(mu[0, 0])
        state["y"]     = float(mu[1, 0])
        state["theta"] = float(mu[2, 0])
        state["sigma"] = sigma
        return (state["x"], state["y"], state["theta"],
                float(sigma[0, 0]), float(sigma[1, 1]), float(sigma[0, 1]))

    elif ftype == "ekf":
        mu, sigma = kf.ekf_filter(
            state["x"], state["y"], state["theta"],
            state["sigma"], state["sigma_R"], state["sigma_Q"],
            v, omega, dt, measurements,
        )
        state["x"]     = float(mu[0, 0])
        state["y"]     = float(mu[1, 0])
        state["theta"] = float(mu[2, 0])
        state["sigma"] = sigma
        return (state["x"], state["y"], state["theta"],
                float(sigma[0, 0]), float(sigma[1, 1]), float(sigma[0, 1]))

    elif ftype == "slam":
        slam_mu, slam_sigma, slam_lm_idx = kf.ekf_slam(
            state["slam_mu"], state["slam_sigma"],
            state["sigma_R"], state["sigma_Q"],
            v, omega, dt, measurements, state["slam_lm_idx"],
        )
        state["slam_mu"]     = slam_mu
        state["slam_sigma"]  = slam_sigma
        state["slam_lm_idx"] = slam_lm_idx
        return (float(slam_mu[0]), float(slam_mu[1]), float(slam_mu[2]),
                float(slam_sigma[0, 0]), float(slam_sigma[1, 1]),
                float(slam_sigma[0, 1]))

    raise ValueError(f"Unknown filter type: {ftype}")


# =============================================================================
# OFFLINE REPLAY (headless, for plot generation)
# =============================================================================

def run_offline_experiment(exp_index: int, landmark_groups: list,
                           walls: list, record_path: list,
                           seed: int = 42) -> dict:
    filter_seed = {"kf": 0, "ekf": 1, "slam": 2}
    np.random.seed(seed + filter_seed[EXPERIMENTS[exp_index]["filter"]])

    saved = (mm.x, mm.y, mm.theta, mm.v, mm.omega, mm.dt)
    state = reset_state(exp_index, record_path)

    exp          = EXPERIMENTS[exp_index]
    use_noise    = exp["sensor_noise"]
    use_obstacle = exp["moving_obstacle"]
    obstacle     = PatrolRobot() if use_obstacle else None

    # one occupancy grid per experiment, updated with the filter's estimated pose
    grid = OccupancyGrid(**GRID_PARAMS)

    result = {
        "name":      exp["name"],
        "id":        exp["id"],
        "group":     exp["group"],
        "filter":    exp["filter"],
        "color":     exp["color"],
        "true_traj": [],
        "est_traj":  [],
        "errors":    [],
        "covs":      [],
        "lm_counts": [],
        "grid":      grid,   # kept for map plotting after the loop
    }

    for frame in record_path:
        x, y, theta, v, omega, dt = frame
        mm.x, mm.y, mm.theta = x, y, theta
        mm.v, mm.omega, mm.dt = v, omega, dt

        if obstacle is not None:
            obstacle.update(dt)

        # Patrol robot segments are added to wall sensor so the occupancy
        # grid sees the obstacle as solid — mirroring phase-3 exactly.
        # Landmark measurements remain geometry-agnostic, so KF/EKF/SLAM
        # are unaffected (S4==S1 behaviour from phase-3 is preserved).
        dynamic_segs = obstacle.as_segments() if obstacle is not None else []
        sensor_walls = walls + dynamic_segs
        measurements = mm.get_landmark_measurements(landmark_groups)

        if use_noise:
            measurements = corrupt_measurements(measurements)

        ex, ey, et, sxx, syy, sxy = filter_step(
            exp_index, state, measurements, v, omega, dt
        )

        # wall sensor readings — includes patrol robot segments for Group C
        wall_readings = mm.get_sensor_readings(sensor_walls)

        # update the grid using the *filter's estimated pose* so map quality
        # directly reflects localisation accuracy
        grid.update(
            robot_x=ex,
            robot_y=ey,
            robot_theta=et,
            sensor_readings=wall_readings,
            max_range=mm.SENSOR_MAX_RANGE,
        )

        result["true_traj"].append((x, y))
        result["est_traj"].append((ex, ey))
        result["errors"].append(math.hypot(x - ex, y - ey))
        result["covs"].append((sxx, syy, sxy))
        result["lm_counts"].append(len(measurements))

    mm.x, mm.y, mm.theta, mm.v, mm.omega, mm.dt = saved
    return result


# =============================================================================
# PLOTS
# =============================================================================

def ensure_summary_dir():
    os.makedirs(SUMMARY_DIR, exist_ok=True)


def _rgb(color_tuple):
    return tuple(c / 255 for c in color_tuple)


def save_all_plots(all_results: list):
    ensure_summary_dir()
    print("\n[Generating plots…]")

    for result in all_results:
        _save_single_trajectory(result)
        _save_occupancy_map(result)

    for group_id in ("A", "B", "C"):
        group_results = [r for r in all_results if r["group"] == group_id]
        if group_results:
            _save_group_trajectory(group_id, group_results)
            _save_group_occupancy_maps(group_id, group_results)

    _save_error_by_filter(all_results)
    _save_error_all(all_results)
    _save_covariance_all(all_results)
    _save_table(all_results)

    print(f"[All plots saved to {SUMMARY_DIR}/]")


def _save_single_trajectory(result: dict):
    fig, ax = plt.subplots(figsize=(8, 7))
    tx  = [p[0] for p in result["true_traj"]]
    ty  = [p[1] for p in result["true_traj"]]
    ex  = [p[0] for p in result["est_traj"]]
    ey  = [p[1] for p in result["est_traj"]]
    rgb = _rgb(result["color"])

    ax.plot(tx, ty, lw=2, color="steelblue", label="True path", zorder=2)
    ax.plot(ex, ey, lw=2, ls="--", color=rgb, label="Estimate", zorder=3, alpha=0.85)
    ax.plot(tx[0], ty[0], "go", ms=8, label="Start")
    ax.plot(tx[-1], ty[-1], "rs", ms=8, label="End")

    # Compute axis span to use as max sensible ellipse size
    span_x = max(tx) - min(tx) if tx else 600
    span_y = max(ty) - min(ty) if ty else 600
    max_ellipse = max(span_x, span_y) * 0.4   # skip anything > 40% of map span

    ellipse_every = max(1, len(result["covs"]) // 20)
    for j in range(0, len(result["covs"]), ellipse_every):
        cx, cy = result["est_traj"][j]
        sxx, syy, sxy = result["covs"][j]
        w, h, ang = get_ellipse_axes(sxx, syy, sxy, n_std=2)
        if w > max_ellipse or h > max_ellipse:
            continue   # skip degenerate/diverged ellipses
        patch = MplEllipse(
            (cx, cy), width=w, height=h, angle=ang,
            fill=False, edgecolor=rgb, lw=0.8, alpha=0.6,
            clip_on=True,
        )
        ax.add_patch(patch)

    ax.set_title(f"{result['name']}  (mean err = {np.mean(result['errors']):.2f})")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Clip axes to true-path extent + 20% margin so diverged ellipses
    # don't compress the view (relevant for SLAM under noise)
    if result["true_traj"]:
        tx = [p[0] for p in result["true_traj"]]
        ty = [p[1] for p in result["true_traj"]]
        pad_x = (max(tx) - min(tx)) * 0.15 + 30
        pad_y = (max(ty) - min(ty)) * 0.15 + 30
        ax.set_xlim(min(tx) - pad_x, max(tx) + pad_x)
        ax.set_ylim(min(ty) - pad_y, max(ty) + pad_y)

    fname = result["id"] + "_trajectory.png"
    fig.savefig(os.path.join(SUMMARY_DIR, fname), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [Saved] {fname}")


def _render_log_odds_to_ax(ax, grid, result, show_path: bool = True):
    """
    Render a grid's log_odds array onto a matplotlib Axes.
    Black = occupied, white = free, grey = unknown.
    Optionally overlays true and estimated trajectories.
    """
    # Convert log-odds to probability and map to greyscale image
    prob = 1.0 - 1.0 / (1.0 + np.exp(grid.log_odds))
    # prob > 0.5 → occupied (dark), < 0.5 → free (bright), ~0.5 → unknown (mid-grey)
    img = np.ones_like(prob) * 0.5          # start everything at mid-grey
    img[prob > 0.6] = 0.0                   # occupied → black
    img[prob < 0.4] = 1.0                   # free     → white

    # Extent: map grid row/col indices to world coordinates
    extent = [grid.x_min, grid.x_max, grid.y_min, grid.y_max]
    ax.imshow(
        img,
        origin="lower",
        extent=extent,
        cmap="gray",
        vmin=0.0, vmax=1.0,
        interpolation="nearest",
        aspect="equal",
    )

    if show_path and result["true_traj"]:
        tx = [p[0] for p in result["true_traj"]]
        ty = [p[1] for p in result["true_traj"]]
        ex = [p[0] for p in result["est_traj"]]
        ey = [p[1] for p in result["est_traj"]]
        ax.plot(tx, ty, color="steelblue", lw=1.5, label="True path")
        ax.plot(ex, ey, color=_rgb(result["color"]),
                lw=1.2, ls="--", label="Estimate")
        ax.plot(tx[0], ty[0], "go", ms=6)
        ax.plot(tx[-1], ty[-1], "rs", ms=6)

    # compute explored %
    total    = grid.rows * grid.cols
    occupied = int(np.sum(grid.log_odds >  0.1))
    free_    = int(np.sum(grid.log_odds < -0.1))
    expl_pct = 100.0 * (occupied + free_) / total if total > 0 else 0.0

    ax.set_title(
        f"{result['name']}\n"
        f"explored={expl_pct:.1f}%  "
        f"mean err={np.mean(result['errors']):.2f}"
    )
    ax.set_xlabel("x")
    ax.set_ylabel("y")


def _save_occupancy_map(result: dict):
    """Individual occupancy map for one experiment."""
    grid = result.get("grid")
    if grid is None:
        return

    fig, ax = plt.subplots(figsize=(8, 7))
    _render_log_odds_to_ax(ax, grid, result, show_path=True)
    ax.legend(fontsize=8)

    fname = result["id"] + "_occupancy_map.png"
    fig.savefig(os.path.join(SUMMARY_DIR, fname), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [Saved] {fname}")


def _save_group_occupancy_maps(group_id: str, group_results: list):
    """Side-by-side occupancy maps for all three filters in a group."""
    group_labels = {"A": "Baseline", "B": "Sensor Noise", "C": "Moving Obstacle"}
    n   = len(group_results)
    fig, axes = plt.subplots(1, n, figsize=(7 * n, 6), squeeze=False)

    for col, result in enumerate(group_results):
        grid = result.get("grid")
        if grid is None:
            continue
        ax = axes[0][col]
        _render_log_odds_to_ax(ax, grid, result, show_path=True)
        ax.legend(fontsize=7)

    fig.suptitle(
        f"Group {group_id} – {group_labels.get(group_id, '')} – Occupancy Maps",
        fontsize=13, fontweight="bold",
    )
    fig.tight_layout()
    fname = f"group_{group_id}_occupancy_maps.png"
    fig.savefig(os.path.join(SUMMARY_DIR, fname), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [Saved] {fname}")


def _save_group_trajectory(group_id: str, group_results: list):
    """Side-by-side subplots for all three filters in a group."""
    group_labels = {"A": "Baseline", "B": "Sensor Noise", "C": "Moving Obstacle"}
    n   = len(group_results)
    fig, axes = plt.subplots(1, n, figsize=(7 * n, 6), squeeze=False)

    # Compute shared axis limits from the true path (same for all filters)
    all_tx = [p[0] for r in group_results for p in r["true_traj"]]
    all_ty = [p[1] for r in group_results for p in r["true_traj"]]
    pad_x  = (max(all_tx) - min(all_tx)) * 0.12 + 30
    pad_y  = (max(all_ty) - min(all_ty)) * 0.12 + 30
    shared_xlim = (min(all_tx) - pad_x, max(all_tx) + pad_x)
    shared_ylim = (min(all_ty) - pad_y, max(all_ty) + pad_y)
    map_span    = max(max(all_tx) - min(all_tx), max(all_ty) - min(all_ty))
    max_ellipse_g = map_span * 0.25   # skip ellipses larger than 25% of map span

    for col, result in enumerate(group_results):
        ax  = axes[0][col]
        tx  = [p[0] for p in result["true_traj"]]
        ty  = [p[1] for p in result["true_traj"]]
        ex  = [p[0] for p in result["est_traj"]]
        ey  = [p[1] for p in result["est_traj"]]
        rgb = _rgb(result["color"])

        ax.plot(tx, ty, lw=2, color="steelblue", label="True path", zorder=2)
        ax.plot(ex, ey, lw=2, ls="--", color=rgb, label=result["filter"].upper(),
                zorder=3, alpha=0.85)
        ax.plot(tx[0], ty[0], "go", ms=7)
        ax.plot(tx[-1], ty[-1], "rs", ms=7)

        # Draw a modest number of ellipses; skip oversized ones
        ellipse_every = max(1, len(result["covs"]) // 6)
        for j in range(0, len(result["covs"]), ellipse_every):
            cx, cy = result["est_traj"][j]
            sxx, syy, sxy = result["covs"][j]
            w, h, ang = get_ellipse_axes(sxx, syy, sxy, n_std=2)
            if w > max_ellipse_g or h > max_ellipse_g:
                continue
            patch = MplEllipse(
                (cx, cy), width=w, height=h, angle=ang,
                fill=False, edgecolor=rgb, lw=0.7, alpha=0.5,
                clip_on=True,
            )
            ax.add_patch(patch)

        mean_err = np.mean(result["errors"])
        ax.set_title(f"{result['filter'].upper()}  (mean={mean_err:.2f})")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_xlim(*shared_xlim)
        ax.set_ylim(*shared_ylim)

    fig.suptitle(f"Group {group_id} – {group_labels.get(group_id, '')}",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    fname = f"group_{group_id}_combined.png"
    fig.savefig(os.path.join(SUMMARY_DIR, fname), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [Saved] {fname}")


def _save_error_by_filter(all_results: list):
    """
    Three subplots (one per filter: KF / EKF / SLAM).
    Each overlays all three groups (A=baseline, B=noise, C=obstacle).
    """
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharey=False)
    filter_order  = ["kf", "ekf", "slam"]
    filter_labels = {"kf": "KF", "ekf": "EKF", "slam": "EKF-SLAM"}
    group_styles  = {"A": ("-",  1.8), "B": ("--", 1.6), "C": (":",  2.0)}
    group_labels  = {"A": "Baseline", "B": "Sensor noise", "C": "Moving obstacle"}
    group_colors  = {"A": "steelblue", "B": "tomato", "C": "seagreen"}

    for col, ftype in enumerate(filter_order):
        ax = axes[col]
        for result in all_results:
            if result["filter"] != ftype:
                continue
            g  = result["group"]
            ls, lw = group_styles[g]
            ax.plot(result["errors"], ls=ls, lw=lw,
                    color=group_colors[g], label=group_labels[g])
        ax.set_title(filter_labels[ftype], fontsize=12)
        ax.set_xlabel("Step")
        ax.set_ylabel("Position error")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    fig.suptitle("Error over time – KF vs EKF vs SLAM across conditions",
                 fontsize=12, fontweight="bold")
    fig.tight_layout()
    fname = "error_by_filter.png"
    fig.savefig(os.path.join(SUMMARY_DIR, fname), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [Saved] {fname}")


def _save_error_all(all_results: list):
    fig, ax = plt.subplots(figsize=(12, 5))
    ls_map = {"A": "-", "B": "--", "C": ":"}
    for result in all_results:
        ax.plot(result["errors"], lw=1.4,
                ls=ls_map[result["group"]],
                color=_rgb(result["color"]),
                label=result["name"])
    ax.set_title("Position Error – All 9 Experiments")
    ax.set_xlabel("Step")
    ax.set_ylabel("Euclidean position error")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=3)
    fig.tight_layout()
    fname = "all_error_over_time.png"
    fig.savefig(os.path.join(SUMMARY_DIR, fname), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [Saved] {fname}")


def _save_covariance_all(all_results: list):
    fig, ax = plt.subplots(figsize=(12, 5))
    ls_map = {"A": "-", "B": "--", "C": ":"}
    for result in all_results:
        trace = [sxx + syy for sxx, syy, _ in result["covs"]]
        ax.plot(trace, lw=1.4,
                ls=ls_map[result["group"]],
                color=_rgb(result["color"]),
                label=result["name"])
    ax.set_title("Covariance Trace (sigma_x^2 + sigma_y^2) – All 9 Experiments")
    ax.set_xlabel("Step")
    ax.set_ylabel("sigma_x^2 + sigma_y^2")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=3)
    fig.tight_layout()
    fname = "all_covariance_trace.png"
    fig.savefig(os.path.join(SUMMARY_DIR, fname), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [Saved] {fname}")


def _save_table(all_results: list):
    col_w  = 30
    num_w  = 11
    header = (f"{'Experiment':<{col_w}} {'Mean':>{num_w}} "
              f"{'Median':>{num_w}} {'P95':>{num_w}} "
              f"{'Max':>{num_w}} {'Final':>{num_w}}")
    sep    = "=" * len(header)
    thin   = "-" * len(header)

    group_title = {
        "A": "Group A – Baseline (no perturbations)",
        "B": "Group B – Sensor Noise/Bias",
        "C": "Group C – Moving Obstacle",
    }

    lines = [sep, "COMPARISON TABLE – KF / EKF / EKF-SLAM", sep, header, thin]
    current_group = None

    for result in all_results:
        if result["group"] != current_group:
            current_group = result["group"]
            lines.append(f"  {group_title[current_group]}")

        errs = result["errors"]
        if not errs:
            continue
        lines.append(
            f"  {result['name']:<{col_w - 2}} "
            f"{np.mean(errs):>{num_w}.2f} "
            f"{np.median(errs):>{num_w}.2f} "
            f"{np.percentile(errs, 95):>{num_w}.2f} "
            f"{np.max(errs):>{num_w}.2f} "
            f"{errs[-1]:>{num_w}.2f}"
        )

    lines.append(sep)

    print("\n" + "\n".join(lines) + "\n")

    tbl_path = os.path.join(SUMMARY_DIR, "comparison_table.txt")
    with open(tbl_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  [Saved] comparison_table.txt")


# =============================================================================
# HUD
# =============================================================================

def draw_hud(surface, exp_idx, visible_count,
             true_pos, est_pos, est_sxx, est_syy,
             error_history, font_sm, font_md):
    panel_w = 310
    panel_x = SCREEN_WIDTH - panel_w

    panel = pygame.Surface((panel_w, SCREEN_HEIGHT), pygame.SRCALPHA)
    panel.fill((10, 10, 25, 215))
    surface.blit(panel, (panel_x, 0))
    pygame.draw.line(surface, (80, 80, 120),
                     (panel_x, 0), (panel_x, SCREEN_HEIGHT), 2)

    y_off = [10]
    row   = 18

    def txt(text, color=HUD_COLOR, bold=False):
        f = font_md if bold else font_sm
        surface.blit(f.render(text, True, color), (panel_x + 8, y_off[0]))
        y_off[0] += row

    def sep():
        pygame.draw.line(surface, (70, 70, 100),
                         (panel_x + 4, y_off[0]),
                         (SCREEN_WIDTH - 4, y_off[0]), 1)
        y_off[0] += 6

    exp = EXPERIMENTS[exp_idx]

    txt("EXPERIMENT", color=(150, 200, 255), bold=True)
    for part in exp["name"].split("–"):
        txt(part.strip(), color=WARN_COLOR, bold=True)

    group_labels = {
        "A": "Group A – Baseline",
        "B": "Group B – Sensor Noise",
        "C": "Group C – Moving Obstacle",
    }
    txt(group_labels[exp["group"]], color=(170, 210, 170))
    y_off[0] += 2

    for line in exp["desc"]:
        txt("  " + line, color=(200, 200, 200))
    y_off[0] += 4

    sep()
    txt("LIVE STATS", color=(150, 200, 255), bold=True)

    err      = math.hypot(true_pos[0] - est_pos[0], true_pos[1] - est_pos[1])
    mean_err = np.mean(error_history[-200:]) if error_history else 0.0

    flag_on = WARN_COLOR
    for s in [
        f"Landmarks vis : {visible_count}",
        f"Position err  : {err:.1f}",
        f"Mean err(200) : {mean_err:.1f}",
        f"sigma_x^2     : {est_sxx:.2f}",
        f"sigma_y^2     : {est_syy:.2f}",
        f"Trace (x+y)   : {est_sxx + est_syy:.2f}",
        f"Filter        : {exp['filter'].upper()}",
        f"Sensor noise  : {'ON' if exp['sensor_noise'] else 'off'}",
        f"Patrol robot  : {'ON' if exp['moving_obstacle'] else 'off'}",
    ]:
        color = flag_on if "ON" in s else HUD_COLOR
        txt(s, color=color)

    y_off[0] += 4
    sep()
    txt("CONTROLS", color=(150, 200, 255), bold=True)
    for c in [
        "Arrows : drive  (RECORD mode)",
        "S      : save trajectory",
        "E      : next experiment",
        "1-9    : jump to experiment",
        "R      : reset state",
        "M      : toggle map overlay",
        "P      : save summary plots",
        "Q      : quit + save plots",
    ]:
        txt(c, color=(160, 160, 180))


# =============================================================================
# MAIN
# =============================================================================

def main():
    global MODE

    pygame.init()
    screen  = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ARS Group 21 – Filter Experiments (9)")
    clock   = pygame.time.Clock()
    font_sm = pygame.font.SysFont("monospace", 14)
    font_md = pygame.font.SysFont("monospace", 15, bold=True)

    # map
    _map = mp.create_map()
    if len(_map) == 3:
        walls, landmarks, landmark_groups = _map
    else:
        walls, landmarks = _map
        landmark_groups  = []
    obstacles = walls + landmarks

    # trajectory
    RECORD_PATH  = []
    REPLAY_INDEX = [0]

    if MODE == "REPLAY":
        if os.path.exists(TRAJECTORY_FILE):
            RECORD_PATH = np.load(TRAJECTORY_FILE, allow_pickle=True).tolist()
            print(f"[Loaded trajectory: {len(RECORD_PATH)} frames]")
        else:
            print("[No trajectory file found – switching to RECORD mode]")
            MODE = "RECORD"

    # initial state
    current_exp   = 0
    state         = reset_state(current_exp, RECORD_PATH)
    error_history = []
    save_now      = False
    collisions    = False
    show_map      = True   # toggle with M
    live_obstacle = PatrolRobot()

    running = True
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False

                elif event.key == pygame.K_r:
                    state         = reset_state(current_exp, RECORD_PATH)
                    error_history = []
                    REPLAY_INDEX[0] = 0
                    live_obstacle = PatrolRobot()
                    print(f"[Reset] {EXPERIMENTS[current_exp]['name']}")

                elif event.key == pygame.K_e:
                    current_exp   = (current_exp + 1) % len(EXPERIMENTS)
                    state         = reset_state(current_exp, RECORD_PATH)
                    error_history = []
                    REPLAY_INDEX[0] = 0
                    live_obstacle = PatrolRobot()
                    print(f"[Next] {EXPERIMENTS[current_exp]['name']}")

                elif event.key in _KEY_MAP:
                    current_exp   = _KEY_MAP[event.key]
                    state         = reset_state(current_exp, RECORD_PATH)
                    error_history = []
                    REPLAY_INDEX[0] = 0
                    live_obstacle = PatrolRobot()
                    print(f"[Jump] {EXPERIMENTS[current_exp]['name']}")

                elif event.key == pygame.K_m:
                    show_map = not show_map

                elif event.key == pygame.K_p:
                    save_now = True

                elif event.key == pygame.K_s:
                    np.save(TRAJECTORY_FILE, np.array(RECORD_PATH))
                    print(f"[Saved {len(RECORD_PATH)} frames -> {TRAJECTORY_FILE}]")

        # motion step
        if MODE == "REPLAY":
            idx = REPLAY_INDEX[0]
            if idx < len(RECORD_PATH):
                x, y, theta, v, omega, dt = RECORD_PATH[idx]
                mm.x, mm.y, mm.theta = x, y, theta
                mm.v, mm.omega, mm.dt = v, omega, dt
                REPLAY_INDEX[0] += 1
            clock.tick(60)
        else:
            keys = pygame.key.get_pressed()
            mm.omega = (OMEGA  if keys[pygame.K_LEFT]  else
                        -OMEGA if keys[pygame.K_RIGHT] else 0.0)
            mm.v     = (VELOCITY  if keys[pygame.K_UP]   else
                        -VELOCITY if keys[pygame.K_DOWN] else 0.0)
            clock.tick(60)
            mm.dt      = FIXED_DT
            collisions = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)
            RECORD_PATH.append((mm.x, mm.y, mm.theta, mm.v, mm.omega, mm.dt))

        exp = EXPERIMENTS[current_exp]

        # update moving obstacle (live view)
        if exp["moving_obstacle"]:
            live_obstacle.update(mm.dt if mm.dt > 0 else FIXED_DT)

        # Sensor readings: patrol segments added to wall sensor for Group C.
        # Landmark sensor stays geometry-agnostic (S4==S1 from phase-3).
        dynamic_segs = live_obstacle.as_segments() if exp["moving_obstacle"] else []
        sensor_walls = walls + dynamic_segs
        measurements = mm.get_landmark_measurements(landmark_groups)
        if exp["sensor_noise"]:
            measurements = corrupt_measurements(measurements)

        # filter step
        ex, ey, et, sxx, syy, sxy = filter_step(
            current_exp, state, measurements, mm.v, mm.omega, mm.dt
        )

        # update occupancy grid with estimated pose + wall sensor
        wall_readings_live = mm.get_sensor_readings(sensor_walls)
        state["grid"].update(
            robot_x=ex,
            robot_y=ey,
            robot_theta=et,
            sensor_readings=wall_readings_live,
            max_range=mm.SENSOR_MAX_RANGE,
        )

        # trails
        state["true_trail"].append((mm.x, mm.y))
        state["est_trail"].append((ex, ey))
        if len(state["true_trail"]) > TRAIL_LEN:
            state["true_trail"].pop(0)
        if len(state["est_trail"]) > TRAIL_LEN:
            state["est_trail"].pop(0)

        error_history.append(math.hypot(mm.x - ex, mm.y - ey))

        # ── drawing ───────────────────────────────────────────────────────────
        screen.fill(WHITE)
        est_col = exp["color"]

        # occupancy grid (drawn first so walls/robot render on top)
        if show_map:
            state["grid"].draw(
                surface=screen,
                world_to_screen_fn=mm.world_to_screen,
                screen_width=SCREEN_WIDTH,
                screen_height=SCREEN_HEIGHT,
                robot_x=mm.x,
                robot_y=mm.y,
            )

        # static map
        for obstacle in obstacles:
            s = mm.world_to_screen(obstacle[0][0], obstacle[0][1],
                                   SCREEN_WIDTH, SCREEN_HEIGHT)
            e = mm.world_to_screen(obstacle[1][0], obstacle[1][1],
                                   SCREEN_WIDTH, SCREEN_HEIGHT)
            pygame.draw.line(screen, BLACK, s, e, 3)

        # patrol robot (Group C) — drawn with heading arrow, same as phase-3
        if exp["moving_obstacle"]:
            live_obstacle.draw(screen, mm.world_to_screen,
                               SCREEN_WIDTH, SCREEN_HEIGHT)

        # true trail
        if len(state["true_trail"]) >= 2:
            pts = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                   for px, py in state["true_trail"]]
            pygame.draw.lines(screen, BLUE, False, pts, 2)

        # estimate trail (dashed)
        if len(state["est_trail"]) >= 2:
            pts = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                   for px, py in state["est_trail"]]
            for i in range(0, len(pts) - 1, 6):
                pygame.draw.line(screen, est_col,
                                 pts[i], pts[min(i + 3, len(pts) - 1)], 2)

        # sensor rays
        sx0, sy0 = mm.world_to_screen(mm.x, mm.y, SCREEN_WIDTH, SCREEN_HEIGHT)
        for reading in measurements:
            lx, ly = reading["landmark_center"]
            lsx, lsy = mm.world_to_screen(lx, ly, SCREEN_WIDTH, SCREEN_HEIGHT)
            pygame.draw.line(screen, GREEN, (sx0, sy0), (lsx, lsy), 1)
            pygame.draw.circle(screen, GREEN, (lsx, lsy), 5, 2)

        # true robot
        robot_col = RED if collisions else GREEN
        corners_w = mm.get_robot_corners(CAR_LENGTH, CAR_WIDTH)
        corners_s = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                     for px, py in corners_w]
        pygame.draw.polygon(screen, robot_col, corners_s)
        pygame.draw.polygon(screen, BLACK, corners_s, 2)

        fwx = mm.x + (CAR_LENGTH / 2) * math.cos(mm.theta)
        fwy = mm.y + (CAR_LENGTH / 2) * math.sin(mm.theta)
        fws = mm.world_to_screen(fwx, fwy, SCREEN_WIDTH, SCREEN_HEIGHT)
        pygame.draw.line(screen, BLACK, (sx0, sy0), fws, 2)

        # estimate robot
        esx, esy      = mm.world_to_screen(ex, ey, SCREEN_WIDTH, SCREEN_HEIGHT)
        est_corners_w = mm.get_robot_corners_at(ex, ey, et, CAR_LENGTH, CAR_WIDTH)
        est_corners_s = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                         for px, py in est_corners_w]
        pygame.draw.polygon(screen, est_col, est_corners_s, 2)

        efwx = ex + (CAR_LENGTH / 2) * math.cos(et)
        efwy = ey + (CAR_LENGTH / 2) * math.sin(et)
        efws = mm.world_to_screen(efwx, efwy, SCREEN_WIDTH, SCREEN_HEIGHT)
        pygame.draw.line(screen, est_col, (esx, esy), efws, 2)

        # covariance ellipse
        draw_covariance_ellipse(
            surface=screen, kf_est_x=ex, kf_est_y=ey,
            sigma_xx=sxx, sigma_yy=syy, sigma_xy=sxy,
            world_to_screen=mm.world_to_screen,
            screen_width=SCREEN_WIDTH, screen_height=SCREEN_HEIGHT,
            color=est_col, n_std=2, n_points=64, thickness=2,
        )

        # EKF-SLAM: estimated landmark positions
        if exp["filter"] == "slam":
            for lid, idx in state["slam_lm_idx"].items():
                lx_est = float(state["slam_mu"][idx])
                ly_est = float(state["slam_mu"][idx + 1])
                lsx, lsy = mm.world_to_screen(lx_est, ly_est,
                                              SCREEN_WIDTH, SCREEN_HEIGHT)
                pygame.draw.circle(screen, PURPLE, (lsx, lsy), 6, 2)
                screen.blit(font_sm.render(f"L{lid}", True, PURPLE),
                            (lsx + 7, lsy - 7))

        # legend
        legend = [
            (BLUE,    "-- True path"),
            (est_col, "-- Estimate"),
            (GREEN,   "  Landmark (in range)"),
            (est_col, "  2-sigma covariance ellipse"),
        ]
        if exp["filter"] == "slam":
            legend.append((PURPLE, "  SLAM landmark estimate"))
        if exp["moving_obstacle"]:
            legend.append((PATROL_COLOR, "  Patrol robot (moving obstacle)"))

        ly0 = SCREEN_HEIGHT - len(legend) * 18 - 6
        for col, label in legend:
            screen.blit(font_sm.render(label, True, col), (8, ly0))
            ly0 += 18

        draw_hud(screen, current_exp, len(measurements),
                 (mm.x, mm.y), (ex, ey), sxx, syy,
                 error_history, font_sm, font_md)

        pygame.display.flip()

        # deferred plot saving
        if save_now:
            save_now = False
            if RECORD_PATH:
                print("\n[Running all 9 offline experiments for plots...]")
                all_results = []
                for i in range(len(EXPERIMENTS)):
                    print(f"  [{i+1}/9] {EXPERIMENTS[i]['name']}")
                    all_results.append(
                        run_offline_experiment(i, landmark_groups, walls, RECORD_PATH)
                    )
                save_all_plots(all_results)
            else:
                print("[No trajectory recorded yet]")

    # on quit: auto-save
    pygame.quit()
    if RECORD_PATH:
        print("\n[Auto-saving summary plots on exit...]")
        all_results = []
        for i in range(len(EXPERIMENTS)):
            print(f"  [{i+1}/9] {EXPERIMENTS[i]['name']}")
            all_results.append(
                run_offline_experiment(i, landmark_groups, walls, RECORD_PATH)
            )
        save_all_plots(all_results)

    sys.exit()


if __name__ == "__main__":
    main()