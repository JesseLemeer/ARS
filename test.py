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

ORANGE = (255, 127,  0)
BLACK  = (  0,   0,  0)
BLUE   = ( 70, 130, 180)
RED    = (200,   0,  0)
GREEN  = (  0, 200,  0)
WHITE  = (255, 255, 255)
DARK_BG    = ( 20,  20,  30)
PANEL_BG   = ( 35,  35,  50, 200)   # semi-transparent (used with Surface)
HUD_COLOR  = (220, 220, 255)
WARN_COLOR = (255, 200,  50)

OMEGA      = 5.0
VELOCITY   = 100.0
CAR_LENGTH = 24
CAR_WIDTH  = 14

SCREEN_WIDTH,  SCREEN_HEIGHT  = 900, 680
WORLD_WIDTH,   WORLD_HEIGHT   = 1600, 1600

TRAIL_LEN = 400


START_X, START_Y, START_THETA = -300.0, 50.0, 0.0
#change MODE to "RECORD" to record a path 
#drive around in the simulation
#PRESS S TO SAVE THE TRAJECTORY 
#change to "REPLAY" to see the robot driving around in different experiments with the saved trajectory
#if MODE = "REPLAY", and there is no trajectory file, it will change mode to RECORD, save with S
MODE = "REPLAY"   
TRAJECTORY_FILE = "trajectory.npy"
RECORD_PATH = []
REPLAY_INDEX = 0

SUMMARY_DIR = "summary_plots"

# Used by summary plots only
LANDMARK_CENTRES = [
    [-300, 0], [100, 100], [-312, 145], [278, -203],
    [-87, 91], [341, 217], [-156, -74], [203, -189],
    [-367, 112], [94, 261], [-241, -238], [318, 43],
]

# Default KF noise parameter, given in main.py
DEFAULT_KF = dict(
    sigma_sq_x      = 25.0,
    sigma_sq_y      = 25.0,
    sigma_sq_theta  = math.radians(10.0) ** 2,
    # R – motion model / process noise
    sigma_sq_Rx     = 2.0,
    sigma_sq_Ry     = 2.0,
    sigma_sq_Rtheta = math.radians(2.0) ** 2,
    # Q – sensor / measurement noise
    sigma_sq_Qx     = 16.0,
    sigma_sq_Qy     = 16.0,
    sigma_sq_Qtheta = math.radians(8.0) ** 2,
)

# Experiment Setup
EXPERIMENTS = [
    {
        "name": "1 – Baseline (Normal)",
        "desc": [
            "Default Q and R matrices.",
            "All landmarks in sensor range are visible.",
            "No artificial sensor or control corruption.",
            "Expect: KF tracks true path closely.",
        ],
        "overrides": {},
        "flags": {},
    },
    {
        "name": "2 – Dead Reckoning (No Landmarks)",
        "desc": [
            "Landmark measurements are hidden from KF.",
            "Filter relies only on prediction step.",
            "Expect: drift grows over time.",
            "Ellipse grows continuously.",
        ],
        "overrides": {},
        "flags": {"no_landmarks": True},
    },
    {
        "name": "3 – Sensor Noise",
        "desc": [
            "Actual landmark measurements are corrupted.",
            "Distance and bearing include noise/bias.",
            "KF receives bad measurements.",
            "Expect: worse corrections and noisier estimate.",
        ],
        "overrides": {},
        "flags": {
            "sensor_dist_bias": 2.0,
            "sensor_dist_std": 4.0,
            "sensor_angle_bias_deg": 3.0,
            "sensor_angle_std_deg": 2.0,
        },
    },
    {
        "name": "4 – Motion Control Noise",
        "desc": [
            "KF prediction uses corrupted controls.",
            "Velocity and angular velocity include noise/bias.",
            "Real robot stays unchanged; odometry is wrong.",
            "Expect: drift between true and estimated path.",
        ],
        "overrides": {},
        "flags": {
            "control_v_scale": 1.05,
            "control_omega_scale": 1.0,
            "control_v_bias": 2.0,
            "control_omega_bias": math.radians(2.0),
            "control_v_std": 3.0,
            "control_omega_std": math.radians(1.0),
        },
    },
    {
        "name": "5 – Few Landmarks (max 3 visible)",
        "desc": [
            "Only up to 3 landmarks are fed to KF per step.",
            "Triangulation sometimes becomes fragile.",
            "Expect: patchy corrections and higher variance.",
            "Compare ellipse size vs baseline.",
        ],
        "overrides": {},
        "flags": {"max_landmarks": 3},
    },
        {
    "name": "6 – Low Q, High R",
    "desc": [
        "Higher trust in sensor measurements.",
        "Lower trust in motion model.",
        "Expected: more reactive but slightly noisier tracking."
    ],
    "q_scale": 0.5,
    "r_scale": 2.0,
    "flags": {},
},
    {
    "name": "7 – High Q, Low R",
    "desc": [
        "Lower trust in sensor measurements.",
        "Higher trust in motion model.",
        "Expected: smoother but more drift."
    ],
    "q_scale": 2.0,
    "r_scale": 0.5,
    "flags": {},
},
    {
    "name": "8 – Q = R (Balanced)",
    "desc": [
        "Equal trust in motion and sensor updates.",
        "Expected: best trade-off between stability and responsiveness."
    ],
    "q_scale": 1.0,
    "r_scale": 1.0,
    "flags": {},
},
]

# Helper classes
def build_kf_params(exp_index: int) -> dict:
    p = dict(DEFAULT_KF)
    exp = EXPERIMENTS[exp_index]

    q_scale = exp.get("q_scale", 1.0)
    r_scale = exp.get("r_scale", 1.0)

    # Sensor noise (Q)
    p["sigma_sq_Qx"] *= q_scale
    p["sigma_sq_Qy"] *= q_scale
    p["sigma_sq_Qtheta"] *= q_scale

    # Motion noise (R)
    p["sigma_sq_Rx"] *= r_scale
    p["sigma_sq_Ry"] *= r_scale
    p["sigma_sq_Rtheta"] *= r_scale

    return p
'''
def reset_state(exp_index: int):
    mm.x     = START_X
    mm.y     = START_Y
    mm.theta = START_THETA
    mm.v = mm.omega = mm.dt = 0.0

    p = build_kf_params(exp_index)
    return (
        START_X, START_Y, START_THETA,          # kf_est_x/y/theta
        p["sigma_sq_x"],                        # kf_sxx
        p["sigma_sq_y"],                        # kf_syy
        p["sigma_sq_theta"],                    # kf_stt
        0.0,                                    # kf_sxy  (off-diagonal)
        [], [],                                 # true_trail, kf_trail
    )
'''
def reset_state(exp_index: int):
    global REPLAY_INDEX

    REPLAY_INDEX = 0

    p = build_kf_params(exp_index)

    # fallback if no trajectory loaded
    if len(RECORD_PATH) == 0:
        mm.x, mm.y, mm.theta = START_X, START_Y, START_THETA
    else:
        mm.x, mm.y, mm.theta, mm.v, mm.omega, _ = RECORD_PATH[0]

    return (
        mm.x, mm.y, mm.theta,
        p["sigma_sq_x"],
        p["sigma_sq_y"],
        p["sigma_sq_theta"],
        0.0,
        [],
        [],
    )

def apply_landmark_flags(readings: list, flags: dict) -> list:
    if flags.get("no_landmarks", False):
        return []
    cap = flags.get("max_landmarks", None)
    if cap is not None:
        return readings[:cap]
    return readings

# Sensor noise
def corrupt_landmark_readings(readings: list, flags: dict) -> list:
    dist_bias = flags.get("sensor_dist_bias", 0.0)
    dist_std = flags.get("sensor_dist_std", 0.0)
    angle_bias_deg = flags.get("sensor_angle_bias_deg", 0.0)
    angle_std_deg = flags.get("sensor_angle_std_deg", 0.0)

    corrupted = []
    for r in readings:
        new_r = dict(r)

        new_distance = r["distance"] + dist_bias + np.random.normal(0.0, dist_std)
        new_angle_deg = r["angle_deg"] + angle_bias_deg + np.random.normal(0.0, angle_std_deg)

        new_r["distance"] = max(0.0, new_distance)
        new_r["angle_deg"] = new_angle_deg

        corrupted.append(new_r)

    return corrupted

# Motion noise
def get_kf_controls(v: float, omega: float, flags: dict):
    v_scale = flags.get("control_v_scale", 1.0)
    omega_scale = flags.get("control_omega_scale", 1.0)
    v_bias = flags.get("control_v_bias", 0.0)
    omega_bias = flags.get("control_omega_bias", 0.0)
    v_std = flags.get("control_v_std", 0.0)
    omega_std = flags.get("control_omega_std", 0.0)

    v_kf = v * v_scale + v_bias + np.random.normal(0.0, v_std)
    omega_kf = omega * omega_scale + omega_bias + np.random.normal(0.0, omega_std)

    return v_kf, omega_kf

def draw_solid_trail(surface, trail, color, width=2):
    """Draw a solid trajectory trail."""
    if len(trail) < 2:
        return
    pts = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT) for px, py in trail]
    pygame.draw.lines(surface, color, False, pts, width)

def draw_dotted_trail(surface, trail, color, gap=4, width=2):
    """Draw a trail as dotted/dashed line (every other *gap* segments drawn)."""
    if len(trail) < 2:
        return
    pts = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT) for px, py in trail]
    for i in range(0, len(pts) - 1, gap * 2):
        j = min(i + gap, len(pts) - 1)
        pygame.draw.line(surface, color, pts[i], pts[j], width)

def draw_hud(surface, exp_idx, visible_count,
             true_pos, kf_pos, kf_sigma, error_hist,
             font_sm, font_md):
    panel_w = 300
    panel_x = SCREEN_WIDTH - panel_w

    panel_surf = pygame.Surface((panel_w, SCREEN_HEIGHT), pygame.SRCALPHA)
    panel_surf.fill((10, 10, 25, 210))
    surface.blit(panel_surf, (panel_x, 0))
    pygame.draw.line(surface, (80, 80, 120), (panel_x, 0), (panel_x, SCREEN_HEIGHT), 2)

    y_off = [12]
    row = 20

    def txt(text, color=HUD_COLOR, bold=False):
        font = font_md if bold else font_sm
        surface.blit(font.render(text, True, color), (panel_x + 8, y_off[0]))
        y_off[0] += row

    def sep():
        pygame.draw.line(surface, (70, 70, 100),
                         (panel_x + 4, y_off[0]), (SCREEN_WIDTH - 4, y_off[0]), 1)
        y_off[0] += 6

    txt("EXPERIMENT", color=(150, 200, 255), bold=True)
    y_off[0] += 2
    for part in EXPERIMENTS[exp_idx]["name"].split("–"):
        txt(part.strip(), color=WARN_COLOR, bold=True)
    y_off[0] += 4

    txt("About:", color=(160, 160, 200))
    for line in EXPERIMENTS[exp_idx]["desc"]:
        while line:
            txt(line[:36], color=(200, 200, 200))
            y_off[0] -= 3
            line = line[36:]
    y_off[0] += 8

    sep()
    txt("LIVE STATS", color=(150, 200, 255), bold=True)

    err = math.hypot(true_pos[0] - kf_pos[0], true_pos[1] - kf_pos[1])
    mean_err = sum(error_hist[-200:]) / max(len(error_hist[-200:]), 1)

    stats = [
        f"Landmarks vis : {visible_count}",
        f"Position err  : {err:.1f}",
        f"Mean err(200) : {mean_err:.1f}",
        f"σx²           : {kf_sigma[0]:.2f}",
        f"σy²           : {kf_sigma[1]:.2f}",
        f"Trace σ(x+y)  : {kf_sigma[0] + kf_sigma[1]:.2f}",
    ]
    for s in stats:
        txt(s)

    y_off[0] += 4
    flags = EXPERIMENTS[exp_idx]["flags"]
    if flags.get("no_landmarks"):
        txt("⚠ No landmarks!", color=(255, 80, 80), bold=True)
    if "max_landmarks" in flags:
        txt(f"⚠ Max {flags['max_landmarks']} lm", color=WARN_COLOR, bold=True)

    sep()
    txt("CONTROLS", color=(150, 200, 255), bold=True)
    for c in [
        "Arrows : drive",
        "E      : next experiment",
        "1-5    : jump to exp",
        "R      : reset state",
        "P      : save summary plots",
        "Q      : quit + save plots",
    ]:
        txt(c, color=(160, 160, 180))

# Summary Simulation
'''
_MOTION_SEQ = [
    (80,  0.0, 120),
    (70,  5.0,  90),
    (80,  0.0, 120),
    (70, -5.0,  90),
    (80,  0.0, 120),
    (70,  5.0,  90),
    (60,  0.0, 100),
]
_TOTAL_STEPS = sum(n for _, _, n in _MOTION_SEQ)
_DT = 0.05
'''

def run_offline_experiment(exp_index: int, walls, landmarks, seed: int = 123):
    """
    Runs the SAME experiment logic as live mode, but headless and on a fixed motion path.
    """
    np.random.seed(seed + exp_index)

    saved_state = (mm.x, mm.y, mm.theta, mm.v, mm.omega, mm.dt)
    mm.x, mm.y, mm.theta = START_X, START_Y, START_THETA
    mm.v = mm.omega = mm.dt = 0.0

    obstacles = walls + landmarks
    flags = EXPERIMENTS[exp_index]["flags"]
    p = build_kf_params(exp_index)

    kf_x, kf_y, kf_theta = START_X, START_Y, START_THETA
    sx, sy, st = p["sigma_sq_x"], p["sigma_sq_y"], p["sigma_sq_theta"]

    result = {
        "true_traj": [],
        "kf_traj": [],
        "errors": [],
        "covs": [],
        "lm_counts": [],
        "name": EXPERIMENTS[exp_index]["name"],
    }

    for x, y, theta, v, omega, dt in RECORD_PATH:

        #ground truth replay (motion model state)
        mm.x, mm.y, mm.theta = x, y, theta
        mm.v, mm.omega, mm.dt = v, omega, dt

        #sensor simulation from replayed state
        all_landmark_readings = mm.get_sensor_readings(landmarks)
        raw_visible = [
            r for r in all_landmark_readings
            if r["distance"] < mm.SENSOR_MAX_RANGE
        ]

        visible = apply_landmark_flags(raw_visible, flags)
        visible = corrupt_landmark_readings(visible, flags)

        #KF prediction input (use recorded controls)
        v_kf, omega_kf = get_kf_controls(v, omega, flags)

        mu, sigma = kf.kalman_filter(
            kf_x, kf_y, kf_theta,
            sx, sy, st,
            p["sigma_sq_Rx"], p["sigma_sq_Ry"], p["sigma_sq_Rtheta"],
            p["sigma_sq_Qx"], p["sigma_sq_Qy"], p["sigma_sq_Qtheta"],
            v_kf, omega_kf, dt,
            visible,
        )

        kf_x, kf_y, kf_theta = float(mu[0, 0]), float(mu[1, 0]), float(mu[2, 0])
        sx = float(sigma[0, 0])
        sy = float(sigma[1, 1])
        st = float(sigma[2, 2])
        sxy = float(sigma[0, 1])

        #logging
        result["true_traj"].append((x, y))   # use recorded trajectory
        result["kf_traj"].append((kf_x, kf_y))
        result["errors"].append(math.hypot(x - kf_x, y - kf_y))
        result["covs"].append((sx, sy, sxy))
        result["lm_counts"].append(len(visible))
          

    # restore state
    mm.x, mm.y, mm.theta, mm.v, mm.omega, mm.dt = saved_state

    return result

def ensure_summary_dir():
    os.makedirs(SUMMARY_DIR, exist_ok=True)

def save_trajectory_plot(result):
    fig, ax = plt.subplots(figsize=(8, 7))

    tx = [p[0] for p in result["true_traj"]]
    ty = [p[1] for p in result["true_traj"]]
    kx = [p[0] for p in result["kf_traj"]]
    ky = [p[1] for p in result["kf_traj"]]

    ax.plot(tx, ty, lw=1.8, label="True path")
    ax.plot(kx, ky, lw=1.5, ls="--", label="KF estimate")
    ax.plot(tx[0], ty[0], "go", ms=7, label="Start")
    ax.plot(tx[-1], ty[-1], "rs", ms=7, label="End")

    lx = [c[0] for c in LANDMARK_CENTRES]
    ly = [c[1] for c in LANDMARK_CENTRES]
    ax.scatter(lx, ly, c="black", s=22, marker="^", label="Landmarks", zorder=5)

    ellipse_every = 55
    for i in range(0, len(result["covs"]), ellipse_every):
        cx, cy = result["kf_traj"][i]
        sx, sy, sxy = result["covs"][i]
        w, h, ang = get_ellipse_axes(sx, sy, sxy, n_std=2)
        ax.add_patch(MplEllipse(
            (cx, cy), width=w, height=h, angle=ang,
            fill=False, lw=1.0, alpha=0.7
        ))

    ax.set_title(f"Trajectory – {result['name']}")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    out_path = os.path.join(SUMMARY_DIR, f"{result['name']}_trajectory.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Saved] {out_path}")

def save_error_plot(all_results):
    fig, ax = plt.subplots(figsize=(10, 5))
    steps = list(range(len(all_results[0]["errors"])))
    

    for result in all_results:
        ax.plot(steps, result["errors"], lw=1.5, label=result["name"])

    ax.set_title("Position Error Over Time")
    ax.set_xlabel("Simulation step")
    ax.set_ylabel("Euclidean position error")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_xlim(0, len(steps))

    out_path = os.path.join(SUMMARY_DIR, "all_experiments_error.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Saved] {out_path}")

def save_covariance_plot(all_results):
    fig, ax = plt.subplots(figsize=(10, 5))

    max_len = max(len(r["covs"]) for r in all_results)
    steps = range(max_len)

    for result in all_results:
        trace = [sx + sy for sx, sy, _ in result["covs"]]
        ax.plot(range(len(trace)), trace, lw=1.5, label=result["name"])

    ax.set_title("KF Covariance Trace Over Time")
    ax.set_xlabel("Simulation step")
    ax.set_ylabel("σx² + σy²")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    out_path = os.path.join(SUMMARY_DIR, "all_experiments_covariance_trace.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Saved] {out_path}")

def save_landmark_plot(all_results):
    fig, ax = plt.subplots(figsize=(10, 5))

    max_len = max(len(r["lm_counts"]) for r in all_results)

    for result in all_results:
        ax.plot(
            range(len(result["lm_counts"])),
            result["lm_counts"],
            lw=1.5,
            label=result["name"]
        )

    ax.set_title("Visible Landmarks Over Time")
    ax.set_xlabel("Simulation step")
    ax.set_ylabel("Visible landmarks")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    out_path = os.path.join(SUMMARY_DIR, "all_experiments_landmark_visibility.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Saved] {out_path}")

def run_and_save_all_summaries(walls, landmarks):
    ensure_summary_dir()
    print("\n[Summary] Running offline experiments...")

    all_results = []

    for exp_index in range(len(EXPERIMENTS)):
        print(f"  Running {EXPERIMENTS[exp_index]['name']}")

        result = run_offline_experiment(exp_index, walls, landmarks)
        all_results.append(result)

        save_trajectory_plot(result)

    save_error_plot(all_results)
    save_covariance_plot(all_results)
    save_landmark_plot(all_results)

    print("\nDone generating all summary plots.")

    print("\n" + "=" * 75)
    print("SUMMARY STATISTICS")
    print("=" * 75)
    print(f"{'Experiment':<42} {'Mean err':>10} {'Max err':>10} {'Final err':>10}")
    print("-" * 75)
    for result in all_results:
        errs = result["errors"]
        print(f"{result['name']:<42} {np.mean(errs):>10.2f} {np.max(errs):>10.2f} {errs[-1]:>10.2f}")
    print("=" * 75)

# Initiating Pygame
pygame.init()
screen  = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("ARS Group 21 – KF Experiments")
clock   = pygame.time.Clock()
font_sm = pygame.font.SysFont("monospace", 15)
font_md = pygame.font.SysFont("monospace", 17, bold=True)
# font_lg = pygame.font.SysFont("monospace", 20, bold=True)

_map_result = mp.create_map()
if len(_map_result) == 3:
    walls, landmarks, _ = _map_result
else:
    walls, landmarks = _map_result
obstacles = walls + landmarks

# Initial experiment state
current_exp = 0
(kf_est_x, kf_est_y, kf_est_theta,
 kf_sxx, kf_syy, kf_stt, kf_sxy,
 true_trail, kf_trail) = reset_state(current_exp)

# running error accumulator
error_history = []
save_plots_now = False

if MODE == "REPLAY":
    if os.path.exists(TRAJECTORY_FILE):
        RECORD_PATH = np.load(TRAJECTORY_FILE, allow_pickle=True).tolist()
        print(f"[Loaded trajectory with {len(RECORD_PATH)} points]")
    else:
        print("Cannot REPLAY: no trajectory recorded")
        MODE = "RECORD"
    
# Main Loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False

            elif event.key == pygame.K_r:
                (
                    kf_est_x, kf_est_y, kf_est_theta,
                    kf_sxx, kf_syy, kf_stt, kf_sxy,
                    true_trail, kf_trail
                ) = reset_state(current_exp)
                error_history.clear()
                print(f"[Reset] Exp {current_exp + 1}: {EXPERIMENTS[current_exp]['name']}")

            elif event.key == pygame.K_e:
                current_exp = (current_exp + 1) % len(EXPERIMENTS)
                (
                    kf_est_x, kf_est_y, kf_est_theta,
                    kf_sxx, kf_syy, kf_stt, kf_sxy,
                    true_trail, kf_trail
                ) = reset_state(current_exp)
                error_history.clear()
                print(f"[Switch] Exp {current_exp + 1}: {EXPERIMENTS[current_exp]['name']}")

            elif event.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4, pygame.K_5):
                idx = event.key - pygame.K_1
                if idx < len(EXPERIMENTS):
                    current_exp = idx
                    (
                        kf_est_x, kf_est_y, kf_est_theta,
                        kf_sxx, kf_syy, kf_stt, kf_sxy,
                        true_trail, kf_trail
                    ) = reset_state(current_exp)
                    error_history.clear()
                    print(f"[Jump] Exp {current_exp + 1}: {EXPERIMENTS[current_exp]['name']}")

            elif event.key == pygame.K_p:
                save_plots_now = True
            
            elif event.key == pygame.K_s:
                np.save(TRAJECTORY_FILE, np.array(RECORD_PATH))
                print("[Saved trajectory]")
                
    # Robot movement
    if MODE == "REPLAY":
        if REPLAY_INDEX < len(RECORD_PATH):
            x, y, theta, v, omega, dt = RECORD_PATH[REPLAY_INDEX]

            mm.x, mm.y, mm.theta = x, y, theta
            mm.v, mm.omega = v, omega
            mm.dt = dt

            REPLAY_INDEX += 1
    else:
        # normal live driving
        keys = pygame.key.get_pressed()
        mm.omega = OMEGA if keys[pygame.K_LEFT] else -OMEGA if keys[pygame.K_RIGHT] else 0.0
        mm.v = VELOCITY if keys[pygame.K_UP] else -VELOCITY if keys[pygame.K_DOWN] else 0.0

        mm.dt = clock.tick(60) / 1000.0
        #mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)

        RECORD_PATH.append((mm.x, mm.y, mm.theta, mm.v, mm.omega, mm.dt))

    

    mm.dt = clock.tick(60) / 1000.0
    collisions = mm.update(obstacles, CAR_LENGTH, CAR_WIDTH)

    # Sensor readings
    all_landmark_readings = mm.get_sensor_readings(landmarks)
    raw_visible = [r for r in all_landmark_readings
                   if r["distance"] < mm.SENSOR_MAX_RANGE]

    # Apply experiment filter (no_landmarks / max_landmarks)
    flags = EXPERIMENTS[current_exp]["flags"]
    visible = apply_landmark_flags(raw_visible, flags)
    visible = corrupt_landmark_readings(visible, flags)

    # Kalman Filter step
    p = build_kf_params(current_exp)

    v_kf, omega_kf = get_kf_controls(mm.v, mm.omega, flags)
    kf_mu, kf_sigma_mat = kf.kalman_filter(
        kf_est_x, kf_est_y, kf_est_theta,
        kf_sxx, kf_syy, kf_stt,
        p["sigma_sq_Rx"], p["sigma_sq_Ry"], p["sigma_sq_Rtheta"],
        p["sigma_sq_Qx"], p["sigma_sq_Qy"], p["sigma_sq_Qtheta"],
        v_kf, omega_kf, mm.dt,
        visible,
    )

    kf_est_x     = float(kf_mu[0, 0])
    kf_est_y     = float(kf_mu[1, 0])
    kf_est_theta = float(kf_mu[2, 0])
    kf_sxx = float(kf_sigma_mat[0, 0])
    kf_syy = float(kf_sigma_mat[1, 1])
    kf_stt = float(kf_sigma_mat[2, 2])
    kf_sxy = float(kf_sigma_mat[0, 1])

    # Update
    true_trail.append((mm.x, mm.y))
    kf_trail.append((kf_est_x, kf_est_y))
    if len(true_trail) > TRAIL_LEN:  true_trail.pop(0)
    if len(kf_trail)   > TRAIL_LEN:  kf_trail.pop(0)

    # Track error history
    error_history.append(math.hypot(mm.x - kf_est_x, mm.y - kf_est_y))

    screen.fill(WHITE)

    # Walls and landmarks
    for obstacle in obstacles:
        s = mm.world_to_screen(obstacle[0][0], obstacle[0][1], SCREEN_WIDTH, SCREEN_HEIGHT)
        e = mm.world_to_screen(obstacle[1][0], obstacle[1][1], SCREEN_WIDTH, SCREEN_HEIGHT)
        pygame.draw.line(screen, BLACK, s, e, 3)

    # Trajectory trails
    draw_solid_trail(screen, true_trail, (30, 80, 200), width=2)      # blue solid = actual
    draw_dotted_trail(screen, kf_trail,  ORANGE, gap=3, width=2)      # orange dotted = KF

    # Sensor rays to landmarks
    screen_x, screen_y = mm.world_to_screen(mm.x, mm.y, SCREEN_WIDTH, SCREEN_HEIGHT)

    for reading in all_landmark_readings:
        hit_x, hit_y   = reading["hit_point"]
        hit_sx, hit_sy = mm.world_to_screen(hit_x, hit_y, SCREEN_WIDTH, SCREEN_HEIGHT)
        in_range       = reading["distance"] < mm.SENSOR_MAX_RANGE
        # Check whether this reading is passed to KF (respects experiment flags)
        is_used = in_range and any(
            r["angle_deg"] == reading["angle_deg"] for r in visible
        )

        if in_range:
            ray_col = GREEN if is_used else (200, 200, 0)   # yellow = visible but suppressed
            pygame.draw.line(screen, ray_col, (screen_x, screen_y), (hit_sx, hit_sy), 1)
            pygame.draw.circle(screen, ray_col, (hit_sx, hit_sy), 4)
        else:
            pygame.draw.line(screen, (210, 210, 255), (screen_x, screen_y), (hit_sx, hit_sy), 1)
            pygame.draw.circle(screen, (180, 180, 220), (hit_sx, hit_sy), 3)

    # True robot (green/red on collision)
    robot_color    = RED if collisions else GREEN
    corners_world  = mm.get_robot_corners(CAR_LENGTH, CAR_WIDTH)
    corners_screen = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                      for px, py in corners_world]
    pygame.draw.polygon(screen, robot_color, corners_screen)
    pygame.draw.polygon(screen, BLACK, corners_screen, 2)

    # Direction arrow (true)
    fx = mm.x + (CAR_LENGTH / 2) * math.cos(mm.theta)
    fy = mm.y + (CAR_LENGTH / 2) * math.sin(mm.theta)
    fxs, fys = mm.world_to_screen(fx, fy, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.draw.line(screen, BLACK, (screen_x, screen_y), (fxs, fys), 2)

    # KF estimate robot (orange outline)
    kf_sx, kf_sy      = mm.world_to_screen(kf_est_x, kf_est_y, SCREEN_WIDTH, SCREEN_HEIGHT)
    kf_corners_world  = mm.get_robot_corners_at(kf_est_x, kf_est_y, kf_est_theta,
                                                CAR_LENGTH, CAR_WIDTH)
    kf_corners_screen = [mm.world_to_screen(px, py, SCREEN_WIDTH, SCREEN_HEIGHT)
                         for px, py in kf_corners_world]
    pygame.draw.polygon(screen, ORANGE, kf_corners_screen, 2)

    kf_fx = kf_est_x + (CAR_LENGTH / 2) * math.cos(kf_est_theta)
    kf_fy = kf_est_y + (CAR_LENGTH / 2) * math.sin(kf_est_theta)
    kfxs, kfys = mm.world_to_screen(kf_fx, kf_fy, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.draw.line(screen, ORANGE, (kf_sx, kf_sy), (kfxs, kfys), 2)

    # COVARIANCE ELLIPSE
    draw_covariance_ellipse(
        surface         = screen,
        kf_est_x        = kf_est_x,
        kf_est_y        = kf_est_y,
        sigma_xx        = kf_sxx,
        sigma_yy        = kf_syy,
        sigma_xy        = kf_sxy,
        world_to_screen = mm.world_to_screen,
        screen_width    = SCREEN_WIDTH,
        screen_height   = SCREEN_HEIGHT,
        color           = ORANGE,
        n_std           = 2,
        n_points        = 64,
        thickness       = 2,
    )

    legend = [
        ((30,  80, 200), "── Actual path"),
        (ORANGE,         "--- KF estimate path"),
        (GREEN,          "●  Landmark (in range, used by KF)"),
        ((200, 200, 0),  "●  Landmark (in range, NOT used)"),
        (ORANGE,         "○  Covariance ellipse (2σ)"),
    ]
    lx_start = 8
    ly_start = SCREEN_HEIGHT - len(legend) * 18 - 6
    for col, label in legend:
        s = font_sm.render(label, True, col)
        screen.blit(s, (lx_start, ly_start))
        ly_start += 18

    draw_hud(
        screen,
        current_exp,
        len(visible),
        (mm.x, mm.y),
        (kf_est_x, kf_est_y),
        (kf_sxx, kf_syy, kf_stt),
        error_history,
        font_sm,
        font_md
    )

    pygame.display.flip()

    if save_plots_now:
        save_plots_now = False
        run_and_save_all_summaries(walls, landmarks)

pygame.quit()
run_and_save_all_summaries(walls, landmarks)
sys.exit()