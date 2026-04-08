# experiment.py
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import motionmodel as mm_module

FIXED_DT = 1 / 60
VELOCITIES = [50, 100, 150, 200, 250, 300]
OMEGAS = [1.0, 2.0, 3.0, 4.0, 5.0, 7.5, 10.0]


def reset_robot():
    mm_module.x = 0.0
    mm_module.y = 0.0
    mm_module.theta = 0.0
    mm_module.v = 0.0
    mm_module.omega = 0.0
    mm_module.dt = FIXED_DT


def measure_linear_resolution(velocity: float) -> float:
    """
    Minimum linear step = displacement from exactly one frame of motion.
    The robot cannot move less than this with binary on/off control.
    """
    reset_robot()
    mm_module.v = velocity
    mm_module.omega = 0.0

    x_before = mm_module.x
    y_before = mm_module.y

    # One frame
    dx, dy, _ = mm_module.velocity_step()
    mm_module.x += dx
    mm_module.y += dy

    return math.hypot(mm_module.x - x_before, mm_module.y - y_before)


def measure_angular_resolution(omega: float) -> float:
    """
    Minimum angular step = rotation from exactly one frame of motion.
    The robot cannot rotate less than this with binary on/off control.
    """
    reset_robot()
    mm_module.v = 0.0
    mm_module.omega = omega

    theta_before = mm_module.theta
    _, _, dtheta = mm_module.velocity_step()
    mm_module.theta += dtheta

    return abs(mm_module.theta - theta_before)


def run_experiment():
    print("=== Linear Resolution ===")
    linear_results = {}
    for v in VELOCITIES:
        res_px = measure_linear_resolution(v)
        res_theoretical = v * FIXED_DT
        linear_results[v] = res_px
        print(f"  v={v:>5} px/s | min step: {res_px:.4f} px "
              f"(theoretical: {res_theoretical:.4f} px)")

    print("\n=== Angular Resolution ===")
    angular_results = {}
    for w in OMEGAS:
        res_rad = measure_angular_resolution(w)
        res_deg = math.degrees(res_rad)
        res_theoretical = w * FIXED_DT
        angular_results[w] = res_rad
        print(f"  ω={w:>5} rad/s | min step: {res_rad:.5f} rad "
              f"({res_deg:.3f}°) (theoretical: {res_theoretical:.5f} rad)")

    return linear_results, angular_results


def plot_results(linear_results: dict, angular_results: dict):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(
        "Minimum Controllable Resolution Under Binary On/Off Control\n"
        f"(Fixed dt = 1/60 s ≈ {1/60*1000:.1f} ms)",
        fontsize=13
    )

    # --- Left: linear resolution ---
    ax = axes[0]
    velocities = list(linear_results.keys())
    steps_px = list(linear_results.values())
    theoretical_lin = [v * FIXED_DT for v in velocities]

    ax.bar(
        [str(v) for v in velocities], steps_px,
        color="steelblue", alpha=0.7, label="Simulated min step"
    )
    ax.plot(
        [str(v) for v in velocities], theoretical_lin,
        "ko--", linewidth=1.5, label="Theoretical (v · dt)"
    )
    ax.set_xlabel("Velocity (px/s)")
    ax.set_ylabel("Minimum step size (px)")
    ax.set_title("Linear Resolution vs Velocity")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)

    # Annotate bars
    for i, (v, s) in enumerate(zip(velocities, steps_px)):
        ax.text(i, s + 0.02, f"{s:.2f}px", ha="center", va="bottom", fontsize=8)

    # --- Right: angular resolution ---
    ax2 = axes[1]
    omegas = list(angular_results.keys())
    steps_rad = list(angular_results.values())
    steps_deg = [math.degrees(r) for r in steps_rad]
    theoretical_ang_deg = [math.degrees(w * FIXED_DT) for w in omegas]

    ax2.bar(
        [str(w) for w in omegas], steps_deg,
        color="darkorange", alpha=0.7, label="Simulated min step"
    )
    ax2.plot(
        [str(w) for w in omegas], theoretical_ang_deg,
        "ko--", linewidth=1.5, label="Theoretical (ω · dt)"
    )
    ax2.set_xlabel("Angular velocity (rad/s)")
    ax2.set_ylabel("Minimum step size (degrees)")
    ax2.set_title("Angular Resolution vs Angular Velocity")
    ax2.legend()
    ax2.grid(True, axis="y", alpha=0.3)

    for i, (w, d) in enumerate(zip(omegas, steps_deg)):
        ax2.text(i, d + 0.002, f"{d:.3f}°", ha="center", va="bottom", fontsize=8)

    plt.tight_layout()
    plt.savefig("precision_experiment.png", dpi=150)
    print("\nPlot saved to precision_experiment.png")


print(f"dt = {FIXED_DT:.6f} s (locked 60 fps)\n")
linear_results, angular_results = run_experiment()
plot_results(linear_results, angular_results)