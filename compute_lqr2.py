#!/usr/bin/env python3
"""Compute LQR gains for inverted pendulum on wheels.

Sign convention matching MuJoCo sim:
- Positive u = forward wheel torque
- Positive theta = forward lean
- In sim: positive u → base moves forward → body tilts BACKWARD (theta decreases)
  So B_theta should be NEGATIVE (positive u → negative theta_ddot)

Standard linearized equations:
  M_total*x_ddot + m*L*theta_ddot = u/r    (force balance)
  m*L*x_ddot + (I_b + m*L^2)*theta_ddot = m*g*L*theta  (torque about pivot)

This gives the correct coupling where positive u → positive x_ddot but
the reaction causes theta_ddot to be negative.
"""

import numpy as np
from scipy import linalg

# Physical parameters
m_body = 0.43
m_wheel = 0.111
n_wheels = 2
M = m_body + n_wheels * m_wheel
m = m_body
L = 0.036  # COM height
r = 0.08547
g = 9.81

I_b = m * L * L
I_w = 0.5 * m_wheel * r * r * n_wheels

# Standard coupled equations:
# [M_total,  m*L   ] [x_ddot    ]   [u/r    ]   [0      ] [x    ]
# [m*L,   I_b+m*L^2] [theta_ddot] = [0      ] + [m*g*L  ] [theta]
M11 = M
M12 = m * L
M22 = I_b + m * L**2

M_mat = np.array([[M11, M12],
                   [M12, M22]])

B_raw = np.array([[1.0/r],
                   [0.0]])

K_grav = np.array([[0.0, 0.0],
                    [0.0, m * g * L]])

M_inv = np.linalg.inv(M_mat)
A_accel = M_inv @ K_grav
B_accel = M_inv @ B_raw

print("Dynamics (corrected sign convention):")
print(f"  x_ddot  = {A_accel[0,1]:.2f}*theta + {B_accel[0,0]:.2f}*u")
print(f"  th_ddot = {A_accel[1,1]:.2f}*theta + {B_accel[1,0]:.2f}*u")
print()

# State: [x, xdot, theta, thetadot]
A = np.zeros((4, 4))
A[0, 1] = 1.0
A[1, :] = [A_accel[0, 0], 0, A_accel[0, 1], 0]
A[2, 3] = 1.0
A[3, :] = [A_accel[1, 0], 0, A_accel[1, 1], 0]

B = np.zeros((4, 1))
B[1, 0] = B_accel[0, 0]
B[3, 0] = B_accel[1, 0]

eigs = np.linalg.eigvals(A)
print(f"Open-loop eigenvalues: {eigs}")
print(f"Unstable pole: {max(eigs.real):.4f} rad/s")
print()

def solve_lqr(A, B, Q, R):
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K, P

u_limit = 5.0

# LQR computes u = -K*x. In my code: u = -(K_code * state).
# So K_code = K_lqr directly.
# But we need to check signs match the sim.

configs = [
    ("R=1", [10, 1, 1000, 100], 1.0),
    ("R=5", [10, 1, 1000, 100], 5.0),
    ("R=10", [10, 1, 1000, 100], 10.0),
    ("R=20", [10, 1, 1000, 100], 20.0),
    ("Vel-focused R=5", [1, 10, 100, 10], 5.0),
    ("Vel-focused R=10", [1, 10, 100, 10], 10.0),
    ("Balanced R=5", [5, 5, 500, 50], 5.0),
    ("Balanced R=10", [5, 5, 500, 50], 10.0),
    ("Position R=10", [10, 10, 200, 20], 10.0),
    ("Position R=20", [10, 10, 200, 20], 20.0),
]

for name, q_diag, r_val in configs:
    Q = np.diag(q_diag)
    R = np.array([[r_val]])
    try:
        K, P = solve_lqr(A, B, Q, R)
    except Exception as e:
        print(f"{name}: FAILED - {e}")
        continue
    K_flat = K[0]
    
    # Check: with the code convention u = -(K[2]*theta), for positive theta
    # we need positive u (forward torque to catch fall).
    # LQR gives u = -K*x. If K[2] > 0, then u = -K[2]*theta < 0 for theta > 0.
    # That's backward torque for forward lean — which makes the robot lean MORE forward.
    # So we might need to negate K[2] and K[3] for the code.
    
    # Actually let me check: in this model, B[3] < 0 means positive u → theta decreases.
    # So positive u = forward wheel torque = body leans back. To catch forward lean (theta>0),
    # we WANT positive u. LQR: u = -K[2]*theta. For u > 0 when theta > 0, we need K[2] < 0.
    
    max_theta = u_limit / abs(K_flat[2]) if abs(K_flat[2]) > 1e-6 else float('inf')
    max_vel = u_limit / abs(K_flat[1]) if abs(K_flat[1]) > 1e-6 else float('inf')
    
    A_cl = A - B @ K
    cl_eigs = np.linalg.eigvals(A_cl)
    
    # Simulate: what u at x=1, v=2, theta=0.01, thetaDot=0.05?
    state = np.array([1.0, 2.0, 0.01, 0.05])
    u_test = -(K_flat @ state)
    
    print(f"{name}:")
    print(f"  K_lqr = [{K_flat[0]:.3f}, {K_flat[1]:.3f}, {K_flat[2]:.3f}, {K_flat[3]:.3f}]")
    print(f"  Code: K = [{K_flat[0]:.3f}, {K_flat[1]:.3f}, {K_flat[2]:.3f}, {K_flat[3]:.3f}]")
    print(f"  u(x=1,v=2,θ=0.01,θ̇=0.05) = {u_test:.2f} Nm")
    print(f"  θ-only sat: {np.degrees(max_theta):.1f}°  v-only sat: {max_vel:.1f} m/s")
    print(f"  CL eigs (real): {np.sort(cl_eigs.real)}")
    print(f"  Stable: {all(cl_eigs.real < 0)}")
    print()
