"""
MMM 5162 - Modelling and Simulation - Mini Project 1
Track MR - Two-Link Robot Pick-Cycle Kinematic Simulation
Student parameter S = 19

Runs:
  1. Baseline simulation with the assigned parameters.
  2. Parameter study: cycle times 0.8T, T, 1.2T.
  3. Plots: end-effector path, joint angles/velocities vs time,
     comparison plot across cycle times.
  4. Performance-measure table.
  5. Automatic model/code check demonstration.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from mr_model import two_link_pick_cycle, path_length

# ----------------------------------------------------------------------
# Student-specific parameters, S = 19
# ----------------------------------------------------------------------
S = 19
L1 = 0.380 + 0.005 * S          # m
L2 = 0.300 + 0.004 * S          # m
q1_start = 20 + S               # deg
q1_end = 75 - 0.5 * S            # deg
q2_start = -55 + 0.8 * S         # deg
q2_end = 15 + 0.5 * S            # deg
T_nom = 2.40 + 0.04 * S          # s

print("Student-specific parameters (S = 19)")
print(f"  L1        = {L1:.4f} m")
print(f"  L2        = {L2:.4f} m")
print(f"  q1_start  = {q1_start:.2f} deg")
print(f"  q1_end    = {q1_end:.2f} deg")
print(f"  q2_start  = {q2_start:.2f} deg")
print(f"  q2_end    = {q2_end:.2f} deg")
print(f"  T_nom     = {T_nom:.4f} s")
print()

# ----------------------------------------------------------------------
# 1. Baseline simulation
# ----------------------------------------------------------------------
N = 1000
t_base = np.linspace(0, T_nom, N)
base = two_link_pick_cycle(t_base, T_nom, L1, L2,
                            q1_start, q1_end, q2_start, q2_end)

# ----------------------------------------------------------------------
# 2. Parameter study: cycle time factors
# ----------------------------------------------------------------------
factors = {"0.8T": 0.8, "1.0T (nominal)": 1.0, "1.2T": 1.2}
results = {}
for label, f in factors.items():
    T_case = f * T_nom
    t_case = np.linspace(0, T_case, N)
    sim = two_link_pick_cycle(t_case, T_case, L1, L2,
                               q1_start, q1_end, q2_start, q2_end)
    results[label] = dict(T=T_case, **sim)

# ----------------------------------------------------------------------
# 3. Performance measures
# ----------------------------------------------------------------------
print(f"{'Case':<16}{'T [s]':>8}{'max|q1dot|':>14}{'max|q2dot|':>14}"
      f"{'PathLen [m]':>14}{'x_end [m]':>12}{'y_end [m]':>12}")
table_rows = []
for label, r in results.items():
    max_q1dot = np.max(np.abs(r["q1dot"]))
    max_q2dot = np.max(np.abs(r["q2dot"]))
    pl = path_length(r["x"], r["y"])
    x_end, y_end = r["x"][-1], r["y"][-1]
    print(f"{label:<16}{r['T']:>8.3f}{max_q1dot:>14.4f}{max_q2dot:>14.4f}"
          f"{pl:>14.4f}{x_end:>12.4f}{y_end:>12.4f}")
    table_rows.append((label, r["T"], max_q1dot, max_q2dot, pl, x_end, y_end))

# Save table to CSV for the report
with open("performance_table.csv", "w") as f:
    f.write("Case,T_s,max_abs_q1dot_rad_s,max_abs_q2dot_rad_s,"
            "path_length_m,x_end_m,y_end_m\n")
    for row in table_rows:
        f.write(",".join(str(v) for v in row) + "\n")

# ----------------------------------------------------------------------
# 4. Automatic model/code check demonstration
# ----------------------------------------------------------------------
print("\nAutomatic model/code check demonstration:")
try:
    two_link_pick_cycle(np.array([-0.5, 1.0]), T_nom, L1, L2,
                         q1_start, q1_end, q2_start, q2_end)
except ValueError as e:
    print(f"  Caught expected error for out-of-range time: {e}")

try:
    two_link_pick_cycle(t_base, -1.0, L1, L2,
                         q1_start, q1_end, q2_start, q2_end)
except ValueError as e:
    print(f"  Caught expected error for non-physical T: {e}")

# ----------------------------------------------------------------------
# 5. Plots
# ----------------------------------------------------------------------
plt.rcParams.update({"font.size": 10})

# --- Principal response plot 1: end-effector path (baseline) ---
fig1, ax1 = plt.subplots(figsize=(5, 4.2))
ax1.plot(base["x"], base["y"], lw=2, color="#1f5fa8")
ax1.plot(base["x"][0], base["y"][0], "go", label="Start")
ax1.plot(base["x"][-1], base["y"][-1], "rs", label="End")
ax1.set_xlabel("x [m]")
ax1.set_ylabel("y [m]")
ax1.set_title(f"End-effector path (baseline, T = {T_nom:.2f} s)")
ax1.axis("equal")
ax1.grid(True, alpha=0.3)
ax1.legend()
fig1.tight_layout()
fig1.savefig("plot1_ee_path.png", dpi=150)

# --- Principal response plot 2: joint angles & velocities vs time ---
fig2, axs = plt.subplots(2, 1, figsize=(6, 6), sharex=True)
axs[0].plot(base["t"], np.degrees(base["q1"]), label="q1(t)")
axs[0].plot(base["t"], np.degrees(base["q2"]), label="q2(t)")
axs[0].set_ylabel("Joint angle [deg]")
axs[0].set_title("Baseline joint motion")
axs[0].legend()
axs[0].grid(True, alpha=0.3)

axs[1].plot(base["t"], base["q1dot"], label=r"$\dot{q}_1$")
axs[1].plot(base["t"], base["q2dot"], label=r"$\dot{q}_2$")
axs[1].set_xlabel("t [s]")
axs[1].set_ylabel("Joint velocity [rad/s]")
axs[1].legend()
axs[1].grid(True, alpha=0.3)
fig2.tight_layout()
fig2.savefig("plot2_joint_motion.png", dpi=150)

# --- Comparison plot: joint speed profiles for the 3 cycle times ---
# Plotted against actual elapsed time t (not normalised u = t/T) so the
# plot also shows the real difference in cycle duration between cases,
# not just the change in peak speed.
fig3, axs2 = plt.subplots(1, 2, figsize=(9, 4))
for label, r in results.items():
    axs2[0].plot(r["t"], r["q1dot"], label=f"{label} (T={r['T']:.2f} s)")
    axs2[1].plot(r["t"], r["q2dot"], label=f"{label} (T={r['T']:.2f} s)")
axs2[0].set_xlabel("t [s]")
axs2[0].set_ylabel(r"$\dot{q}_1$ [rad/s]")
axs2[0].set_title(r"Joint-1 speed vs time")
axs2[0].grid(True, alpha=0.3)
axs2[0].legend()

axs2[1].set_xlabel("t [s]")
axs2[1].set_ylabel(r"$\dot{q}_2$ [rad/s]")
axs2[1].set_title(r"Joint-2 speed vs time")
axs2[1].grid(True, alpha=0.3)
axs2[1].legend()
fig3.tight_layout()
fig3.savefig("plot3_speed_comparison.png", dpi=150)

print("\nPlots saved: plot1_ee_path.png, plot2_joint_motion.png, "
      "plot3_speed_comparison.png")
print("Performance table saved: performance_table.csv")
