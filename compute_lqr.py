#!/usr/bin/env python3
"""Compute LQR gains for inverted pendulum on wheels.

State: [x, xdot, theta, thetadot]
Input: u (wheel torque, Nm)

Linearized dynamics for inverted pendulum on wheels:
  M_total * x_ddot = u/r - friction
  (I + M_body*L^2) * theta_ddot = M_body*g*L*theta - u*L/r  (approx)

But actually for a wheeled inverted pendulum, the correct coupled dynamics are:

Let:
  M = total mass (body + wheels)
  m = body mass
  L = distance from wheel axis to body COM
  I_b = body moment of inertia about COM
  I_w = wheel moment of inertia
  r = wheel radius
  g = gravitational acceleration

The linearized equations (small angle):
  (M*r^2 + I_w) * x_ddot - m*r*L * theta_ddot = u
  -m*r*L * x_ddot + (I_b + m*L^2) * theta_ddot = m*g*L*theta

This gives us the A,B matrices for state-space form.
"""

import numpy as np
from scipy import linalg

# Physical parameters (from MuJoCo model / config)
# These are estimates - we'll validate with sim behavior
m_body = 0.43       # body mass (kg) 
m_wheel = 0.111     # mass per wheel (kg)
n_wheels = 2
M = m_body + n_wheels * m_wheel  # total mass
m = m_body          # body mass (for pendulum dynamics)
L = 0.036           # COM height above wheel axis (m) - from config
r = 0.08547         # wheel radius (m) - from MuJoCo
g = 9.81            # gravity

# Inertias
I_b = m * L * L     # body moment of inertia about COM (thin rod approx)
I_w = 0.5 * m_wheel * r * r * n_wheels  # wheel moment of inertia (cylinder)

print(f"Parameters:")
print(f"  M_total = {M:.3f} kg")
print(f"  m_body  = {m:.3f} kg")
print(f"  L (COM) = {L:.4f} m")
print(f"  r       = {r:.5f} m")
print(f"  I_b     = {I_b:.6f} kg⋅m²")
print(f"  I_w     = {I_w:.6f} kg⋅m²")
print()

# Mass matrix for linearized system:
# [M*r^2 + I_w,  -m*r*L ] [x_ddot    ]   [u       ]   [0        ] [x   ]
# [-m*r*L,    I_b+m*L^2 ] [theta_ddot] = [0       ] + [m*g*L    ] [theta]

M11 = M * r**2 + I_w
M12 = -m * r * L
M22 = I_b + m * L**2

M_mat = np.array([[M11, M12],
                   [M12, M22]])

# Force/stiffness
B_raw = np.array([[1.0],    # torque input to x_ddot equation
                   [0.0]])   # no direct torque in theta equation

K_grav = np.array([[0.0, 0.0],
                    [0.0, m * g * L]])  # gravity destabilizing term

# Solve: M_mat * [x_ddot; theta_ddot] = B_raw * u + K_grav * [x; theta]
M_inv = np.linalg.inv(M_mat)

# Acceleration = M_inv * K_grav * [x, theta] + M_inv * B_raw * u
A_accel = M_inv @ K_grav  # 2x2
B_accel = M_inv @ B_raw   # 2x1

print(f"M_inv @ K_grav (acceleration from state):")
print(f"  x_ddot  = {A_accel[0,0]:.4f}*x + {A_accel[0,1]:.4f}*theta")
print(f"  th_ddot = {A_accel[1,0]:.4f}*x + {A_accel[1,1]:.4f}*theta")
print(f"B_accel (acceleration from torque):")
print(f"  x_ddot/u  = {B_accel[0,0]:.4f}")
print(f"  th_ddot/u = {B_accel[1,0]:.4f}")
print()

# State-space: x = [pos, vel, theta, thetaDot]
A = np.zeros((4, 4))
A[0, 1] = 1.0  # pos_dot = vel
A[1, :] = [A_accel[0, 0], 0, A_accel[0, 1], 0]  # vel_dot
A[2, 3] = 1.0  # theta_dot = thetaDot
A[3, :] = [A_accel[1, 0], 0, A_accel[1, 1], 0]  # thetaDot_dot

B = np.zeros((4, 1))
B[1, 0] = B_accel[0, 0]
B[3, 0] = B_accel[1, 0]

print("A matrix:")
print(A)
print("\nB matrix:")
print(B)

# Open-loop eigenvalues
eigs = np.linalg.eigvals(A)
print(f"\nOpen-loop eigenvalues: {eigs}")
print(f"Unstable pole: {max(eigs.real):.4f} rad/s")
print()

# LQR design with different tunings
# u_limit = 5 Nm, so we want gains that don't exceed this
# Key insight: R must be large enough to penalize torque use

def solve_lqr(A, B, Q, R):
    """Solve continuous-time LQR."""
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K, P

print("=" * 60)
print("LQR Gain Sweep")
print("=" * 60)

# Try various Q/R combinations
configs = [
    # (name, Q_diag, R_scalar)
    ("Original (saturates)", [10, 1, 1000, 100], 0.005),
    ("R=0.1 (moderate)", [10, 1, 1000, 100], 0.1),
    ("R=1.0 (conservative)", [10, 1, 1000, 100], 1.0),
    ("R=5.0 (very conservative)", [10, 1, 1000, 100], 5.0),
    ("R=10 (ultra conservative)", [10, 1, 1000, 100], 10.0),
    ("Vel-focused R=1", [1, 10, 100, 10], 1.0),
    ("Vel-focused R=5", [1, 10, 100, 10], 5.0),
    ("Balanced R=2", [5, 5, 500, 50], 2.0),
    ("Pos+Vel R=2", [10, 10, 200, 20], 2.0),
    ("Pos+Vel R=5", [10, 10, 200, 20], 5.0),
]

u_limit = 5.0

for name, q_diag, r_val in configs:
    Q = np.diag(q_diag)
    R = np.array([[r_val]])
    K, P = solve_lqr(A, B, Q, R)
    K_flat = K[0]
    
    # Check saturation at typical disturbance
    # theta=0.01 rad (0.57°), thetaDot=0.1 rad/s, x=0.5m, v=1 m/s
    test_state = np.array([0.5, 1.0, 0.01, 0.1])
    u_test = K_flat @ test_state
    
    # Max theta before saturation (with other states zero)
    max_theta = u_limit / abs(K_flat[2]) if abs(K_flat[2]) > 1e-6 else float('inf')
    max_vel = u_limit / abs(K_flat[1]) if abs(K_flat[1]) > 1e-6 else float('inf')
    
    # Closed-loop eigenvalues
    A_cl = A - B @ K
    cl_eigs = np.linalg.eigvals(A_cl)
    
    print(f"\n{name}:")
    print(f"  K = [{K_flat[0]:.2f}, {K_flat[1]:.2f}, {K_flat[2]:.2f}, {K_flat[3]:.2f}]")
    print(f"  u at test state = {u_test:.2f} Nm (limit={u_limit})")
    print(f"  Max theta (alone) = {np.degrees(max_theta):.1f}° before sat")
    print(f"  Max vel (alone) = {max_vel:.1f} m/s before sat")
    print(f"  CL eigenvalues: {np.sort(cl_eigs.real)}")
    print(f"  Stable: {all(cl_eigs.real < 0)}")
