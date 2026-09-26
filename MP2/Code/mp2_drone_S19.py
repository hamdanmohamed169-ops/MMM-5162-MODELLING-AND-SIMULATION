"""
MMM 5162 - Modelling and Simulation, Fall 2026 (Term 481)
Mini Project 2 - Track MR: Vertical Drone Motion with Thrust-Actuator Lag
Student number S = 19

State model:
    zdot = v
    vdot = (T - m*g - c*v*|v|) / m
    Tdot = (Tcmd(t) - T) / tau_T

Tcmd(t) = r*m*g  for  1 s <= t <= 1 + pulse_duration
        = m*g    otherwise
Initial conditions: z(0)=0, v(0)=0, T(0)=m*g
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------
# 1. Student-specific parameters (S = 19)
# ----------------------------------------------------------------------
S = 19
g = 9.81                       # m/s^2
m = 1.60 + 0.03 * S             # kg
c = 0.120 + 0.005 * S           # quadratic drag coefficient, N/(m/s)^2
tau_T = 0.160 + 0.005 * S       # thrust actuator time constant, s
r = 1.180 + 0.005 * S           # pulse thrust ratio
pulse_start = 1.0               # s
pulse_duration = 1.40 + 0.03 * S  # s
pulse_end = pulse_start + pulse_duration
t_span = (0.0, 8.0)             # simulation interval, s

print("=== Student-specific parameters (S = %d) ===" % S)
print(f"m      = {m:.4f} kg")
print(f"c      = {c:.4f} N/(m/s)^2")
print(f"tau_T  = {tau_T:.4f} s")
print(f"r      = {r:.4f}")
print(f"pulse_duration = {pulse_duration:.4f} s  (pulse active {pulse_start:.2f}-{pulse_end:.2f} s)")
print(f"Hover thrust m*g = {m*g:.4f} N,  Pulse thrust r*m*g = {r*m*g:.4f} N")


# ----------------------------------------------------------------------
# 2. Model function (reusable) -- thrust command and state derivative
# ----------------------------------------------------------------------
def Tcmd(t):
    """Commanded thrust: hover thrust except during the climb pulse."""
    if pulse_start <= t <= pulse_end:
        return r * m * g
    return m * g


def drone_ode(t, x):
    """First-order state model: x = [z, v, T]."""
    z, v, T = x
    zdot = v
    vdot = (T - m * g - c * v * abs(v)) / m
    Tdot = (Tcmd(t) - T) / tau_T
    return [zdot, vdot, Tdot]


x0 = [0.0, 0.0, m * g]  # z0, v0, T0


# ----------------------------------------------------------------------
# 3. Standard ODE solver (RK45, tight tolerance -> reference solution)
# ----------------------------------------------------------------------
t_eval = np.linspace(t_span[0], t_span[1], 4001)  # 2 ms reporting grid
sol = solve_ivp(drone_ode, t_span, x0, method="RK45",
                 t_eval=t_eval, rtol=1e-10, atol=1e-12, max_step=0.01)

z_ref, v_ref, T_ref = sol.y
t_ref = sol.t

v_max_ref = np.max(v_ref)
t_vmax_ref = t_ref[np.argmax(v_ref)]
z_end_ref = z_ref[-1]

print("\n=== Reference solution (RK45, tight tolerance) ===")
print(f"Max climb velocity   v_max = {v_max_ref:.5f} m/s  at t = {t_vmax_ref:.3f} s")
print(f"Altitude at t = 8 s  z(8)  = {z_end_ref:.5f} m")


# ----------------------------------------------------------------------
# 4. Forward Euler implementation (same state model)
# ----------------------------------------------------------------------
def forward_euler(dt):
    n = int(round((t_span[1] - t_span[0]) / dt)) + 1
    t = np.linspace(t_span[0], t_span[0] + (n - 1) * dt, n)
    x = np.zeros((n, 3))
    x[0] = x0
    for k in range(n - 1):
        dx = drone_ode(t[k], x[k])
        x[k + 1] = x[k] + dt * np.array(dx)
    return t, x[:, 0], x[:, 1], x[:, 2]


euler_steps = [0.02, 0.005, 0.001]   # three step sizes -> convergence study
euler_results = {}
for dt in euler_steps:
    t_e, z_e, v_e, T_e = forward_euler(dt)
    v_max_e = np.max(v_e)
    t_vmax_e = t_e[np.argmax(v_e)]
    z_end_e = z_e[-1]
    euler_results[dt] = dict(t=t_e, z=z_e, v=v_e, T=T_e,
                              v_max=v_max_e, t_vmax=t_vmax_e, z_end=z_end_e)
    print(f"\n=== Forward Euler, dt = {dt} s ===")
    print(f"Max climb velocity   v_max = {v_max_e:.5f} m/s  at t = {t_vmax_e:.3f} s "
          f"(error vs RK45: {v_max_e - v_max_ref:+.5f} m/s)")
    print(f"Altitude at t = 8 s  z(8)  = {z_end_e:.5f} m "
          f"(error vs RK45: {z_end_e - z_end_ref:+.5f} m)")


# ----------------------------------------------------------------------
# 5. Numerical verification table
# ----------------------------------------------------------------------
rows = []
rows.append(("RK45 (rtol=1e-10; ref.)", "-", v_max_ref, t_vmax_ref, z_end_ref, 0.0, 0.0))
for dt in euler_steps:
    r_ = euler_results[dt]
    rows.append((f"Forward Euler", f"{dt}", r_["v_max"], r_["t_vmax"], r_["z_end"],
                 r_["v_max"] - v_max_ref, r_["z_end"] - z_end_ref))

print("\n=== Verification table ===")
print(f"{'Method':22s} {'dt (s)':8s} {'v_max (m/s)':12s} {'t_vmax (s)':11s} "
      f"{'z(8) (m)':10s} {'v_max err':10s} {'z(8) err':10s}")
for row in rows:
    method, dt_s, vmax, tvmax, zend, ev, ez = row
    print(f"{method:22s} {dt_s:8s} {vmax:12.5f} {tvmax:11.3f} {zend:10.5f} {ev:+10.5f} {ez:+10.5f}")


# ----------------------------------------------------------------------
# 6. Automatic sanity check (time domain / physical bound)
# ----------------------------------------------------------------------
assert t_ref.min() >= t_span[0] - 1e-9 and t_ref.max() <= t_span[1] + 1e-9, \
    "Reported time outside simulation interval!"
assert np.all(T_ref >= 0), "Non-physical negative thrust encountered!"
print("\nAutomatic check passed: time within [0,8] s and thrust remains non-negative.")


# ----------------------------------------------------------------------
# 7. Plots
# ----------------------------------------------------------------------
# --- Figure 1: main transient response (z, v, T) from RK45 solution ---
fig1, axs = plt.subplots(3, 1, figsize=(7, 8), sharex=True)

axs[0].plot(t_ref, z_ref, color="tab:blue", lw=1.8)
axs[0].set_ylabel("Altitude z (m)")
axs[0].set_title(f"Drone Vertical Response (S={S}): Altitude, Velocity, Thrust vs Time")
axs[0].grid(True, alpha=0.3)

axs[1].plot(t_ref, v_ref, color="tab:orange", lw=1.8)
axs[1].axhline(v_max_ref, color="gray", ls="--", lw=1,
                label=f"max v = {v_max_ref:.3f} m/s")
axs[1].set_ylabel("Velocity v (m/s)")
axs[1].legend(loc="upper right", fontsize=8)
axs[1].grid(True, alpha=0.3)

axs[2].plot(t_ref, T_ref, color="tab:green", lw=1.8, label="T(t)")
axs[2].axhline(m * g, color="k", ls=":", lw=1, label="hover thrust m·g")
axs[2].axhline(r * m * g, color="r", ls=":", lw=1, label="pulse thrust r·m·g")
axs[2].set_ylabel("Thrust T (N)")
axs[2].set_xlabel("Time t (s)")
axs[2].legend(loc="upper right", fontsize=8)
axs[2].grid(True, alpha=0.3)

fig1.tight_layout()
fig1.savefig("fig1_transient_response.png", dpi=200)
plt.close(fig1)

# --- Figure 2: Standard solver vs Forward Euler comparison (velocity) ---
fig2, ax2 = plt.subplots(figsize=(7.5, 4.5))
ax2.plot(t_ref, v_ref, color="k", lw=2.2, label="RK45 (reference)")
colors = ["tab:red", "tab:orange", "tab:purple"]
for dt, col in zip(euler_steps, colors):
    r_ = euler_results[dt]
    ax2.plot(r_["t"], r_["v"], lw=1.1, color=col, label=f"Forward Euler, dt={dt} s")
ax2.set_xlabel("Time t (s)")
ax2.set_ylabel("Velocity v (m/s)")
ax2.set_title(f"Standard Solver (RK45) vs Forward Euler: Vertical Velocity (S={S})")
ax2.legend(loc="upper right", fontsize=8)
ax2.grid(True, alpha=0.3)
fig2.tight_layout()
fig2.savefig("fig2_euler_vs_rk45.png", dpi=200)
plt.close(fig2)

# --- Figure 3: Euler step-size convergence of v_max and z(8) ---
fig3, ax3a = plt.subplots(figsize=(7, 4.5))
dts = euler_steps
vmax_errs = [abs(euler_results[dt]["v_max"] - v_max_ref) for dt in dts]
zend_errs = [abs(euler_results[dt]["z_end"] - z_end_ref) for dt in dts]

ax3a.loglog(dts, vmax_errs, "o-", color="tab:red", label="|error| in v_max")
ax3a.loglog(dts, zend_errs, "s-", color="tab:blue", label="|error| in z(8)")
ax3a.set_xlabel("Euler time step dt (s)")
ax3a.set_ylabel("Absolute error vs RK45 reference")
ax3a.set_title(f"Forward Euler Convergence (S={S})")
ax3a.legend(loc="upper left", fontsize=9)
ax3a.grid(True, which="both", alpha=0.3)
fig3.tight_layout()
fig3.savefig("fig3_euler_convergence.png", dpi=200)
plt.close(fig3)

print("\nPlots saved: fig1_transient_response.png, fig2_euler_vs_rk45.png, fig3_euler_convergence.png")

# ----------------------------------------------------------------------
# 8. Save verification table to CSV for the report
# ----------------------------------------------------------------------
import csv
with open("verification_table.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Method", "dt (s)", "v_max (m/s)", "t_vmax (s)", "z(8) (m)",
                      "v_max error (m/s)", "z(8) error (m)"])
    for row in rows:
        method, dt_s, vmax, tvmax, zend, ev, ez = row
        writer.writerow([method, dt_s, f"{vmax:.5f}", f"{tvmax:.3f}", f"{zend:.5f}",
                          f"{ev:+.5f}", f"{ez:+.5f}"])

print("Verification table saved: verification_table.csv")
