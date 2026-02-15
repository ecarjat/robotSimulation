#!/usr/bin/env python3
"""
Derive LQR gains from first principles using inverted pendulum physics.

Key physics:
- Inverted pendulum: θ̈ = (g/l_eff)*θ + (1/l_eff)*a_base
- Wheel acceleration: a_base = u_wheel / m_eff
- Natural frequency: ω_n = sqrt(g/l_eff)
- For good response: closed-loop poles at ω_n * damping_factor

Robot parameters from MuJoCo:
- Upper body mass: m_upper = 6.79 kg
- Wheel mass: m_wheel = 0.43 kg each
- COM height above wheel contact: ~0.4-0.5 m (depends on hip angle)
- Wheel radius: r = 0.08547 m
"""

import numpy as np
import csv
from pathlib import Path

# Robot parameters (from compute_com)
m_upper = 6.7877  # kg, upper body mass
m_wheel = 0.4280  # kg, per wheel
m_total = m_upper + 2 * m_wheel
r_wheel = 0.08547  # m
g = 9.81  # m/s^2

# Read equilibrium data to get COM geometry at each hip angle
eq_data = {}
for f in Path("linearize_out").glob("eq_hip_*.csv"):
    try:
        with open(f) as csvfile:
            lines = csvfile.readlines()
            if len(lines) < 2:
                continue
            parts = lines[1].strip().split(',')
            hip = float(parts[0])
            theta_eq = float(parts[1])
            eq_data[hip] = theta_eq
    except:
        pass

# Read COM measurements to get l_eff (COM height)
# From measured_com_data.h
com_measurements = [
    (-0.2700, 0.048, 0.256),  # hip, com_rel_x, com_rel_z
    (-0.1625, 0.044, 0.321),
    (-0.0550, 0.039, 0.377),
    ( 0.0525, 0.034, 0.418),
    ( 0.1600, 0.029, 0.457),
    ( 0.2675, 0.022, 0.494),
    ( 0.3750, 0.010, 0.530),
]

print("=" * 80)
print("PHYSICS-BASED GAIN DERIVATION")
print("=" * 80)
print(f"\nRobot Parameters:")
print(f"  Upper body mass: {m_upper:.2f} kg")
print(f"  Wheel mass (each): {m_wheel:.2f} kg")
print(f"  Total mass: {m_total:.2f} kg")
print(f"  Wheel radius: {r_wheel:.4f} m")
print(f"  Gravity: {g:.2f} m/s²")

# Physics derivation for each hip angle
results = []

print(f"\n{'Hip (rad)':>10} | {'l_eff(m)':>9} | {'ω_n(rad/s)':>11} | {'f_n(Hz)':>8} | {'K2_min':>7} | {'K2_phys':>9}")
print("-" * 80)

for hip, com_x, com_z in com_measurements:
    # Effective pendulum length (COM height above wheel contact)
    l_eff = com_z
    
    # Natural frequency of inverted pendulum
    omega_n = np.sqrt(g / l_eff)
    f_n = omega_n / (2 * np.pi)
    
    # Minimum K2 for stability (simplified): u = K2*theta must counter gravity
    # Gravitational torque: τ_g = m*g*l_eff*sin(θ) ≈ m*g*l_eff*θ
    # Wheel torque creates base accel: a = u/m_eff
    # For inverted pendulum: θ̈ = (g/l_eff)*θ - (1/l_eff)*a
    # Substituting a = -K2*θ / m_eff:
    # θ̈ = (g/l_eff - K2/(m_eff*l_eff))*θ
    # For stability: K2 > m_eff * g
    
    # Effective mass for wheel base (includes rotational inertia)
    # Simplified: m_eff ≈ m_total + I_wheel/r²
    # For cylinder: I = 0.5*m*r², so I/r² = 0.5*m
    m_eff = m_total + 0.5 * 2 * m_wheel  # Both wheels
    
    K2_min = m_eff * g
    
    # For good damping (ζ=0.7 critical damping), place poles at:
    # s = -ζ*ω_n ± j*ω_n*sqrt(1-ζ²)
    # This requires K2 approximately:
    zeta = 0.7  # Critical damping ratio
    K2_physics = m_eff * g * (1 + 2*zeta)  # Heuristic formula
    
    results.append((hip, l_eff, omega_n, f_n, K2_min, K2_physics))
    print(f"{hip:10.4f} | {l_eff:9.3f} | {omega_n:11.2f} | {f_n:8.2f} | {K2_min:7.1f} | {K2_physics:9.1f}")

print("\n" + "=" * 80)
print("ANALYSIS")
print("=" * 80)

print(f"\nEffective mass: {m_eff:.2f} kg")
print(f"Minimum K2 for stability: {K2_min:.1f} N·m/rad")
print(f"\nPhysics-based K2 range: {min(r[5] for r in results):.0f} to {max(r[5] for r in results):.0f}")

# Compare to current gains
print(f"\nCurrent gains from lqr_lut_final.csv:")
try:
    with open("lqr_lut_final.csv") as f:
        reader = csv.DictReader(f)
        for row in reader:
            hip = float(row['hip'])
            k2 = float(row['K2'])
            print(f"  Hip {hip:+.4f}: K2 = {k2:.1f} (needs ~{[r[5] for r in results if abs(r[0]-hip)<0.01][0]:.0f})")
except:
    print("  (Could not read lqr_lut_final.csv)")

print(f"\n⚠️  TORQUE LIMIT: 3.7 N·m peak")
print(f"At 5° error (0.087 rad):")
for hip, l_eff, omega_n, f_n, K2_min, K2_phys in results:
    torque_5deg = K2_phys * 0.087
    status = "✓" if torque_5deg < 3.7 else "❌"
    print(f"  Hip {hip:+.4f}: K2={K2_phys:5.0f} → u={torque_5deg:4.1f} Nm {status}")

print("\n" + "=" * 80)
print("CONCLUSION")
print("=" * 80)
print("""
Physics predicts K2 ~ 180-190 N·m/rad for good damping.
This exceeds 3.7 Nm torque limit at >2° error!

The robot is fundamentally UNDER-ACTUATED for the available motor torque.

Options:
1. Accept saturation (LQR stays stable under saturation usually)
2. Lower K2 (slower response, might still be stable if K2 > K2_min ~ 75)
3. Add position feedback (K0) to help wheel motors
4. Reduce performance expectations
""")

