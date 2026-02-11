# Robot2Wheel Simulation Pipeline

This repo uses a multi-step pipeline to go from CAD → MJCF → linearization → LQR gains → simulation. This README documents each tool, what it expects, what it outputs, and the units.

## Overview (pipeline)

1) **Export MJCF from Onshape**
   `onshape-to-robot myRobot`
   → `myRobot/scene.xml`, `myRobot/robot.xml`, meshes

2) **Linearize across hip angles**
   `build/lqr_harness/linearize_hip --model myRobot/scene.xml --hip hip_L --hip-r hip_R --min -0.27 --max 0.375 --equilibrium-wheels`
   → `linearize_out/A_hip_*.csv`, `B_hip_*.csv`
   → reduced `Ared_hip_*.csv`, `Bred_hip_*.csv`
   → equilibrium `eq_hip_*.csv`, `ctrl_hip_*.csv`

3) **System identification (MuJoCo model validation)**
   `build/sysid_fast myRobot/scene.xml`
   → Measures actual a, b, c coefficients from simulation

4) **Find equilibrium pitch per hip angle**
   `build/find_eq myRobot/scene.xml`
   → `theta_eq` values where zero torque needed

5) **Tune gains (empirical or LQR sweep)**
   Either: `python3 tools/lqr_sweep.py --diag --include-eq`
   Or: Manual tuning via `test_balance` iterations

6) **Generate LUT files**
   Edit `lqr_lut.csv` → run `python3 tools/lqr_lut_to_header.py lqr_lut.csv --out simulate_app/lqr_lut_data.h`

7) **Run simulation with MotionController**
   `build/test_balance myRobot/scene.xml 120 5.0`

```
Onshape
    |
    v
onshape-to-robot myRobot
    |  (myRobot/scene.xml + meshes)
    v
build/lqr_harness/linearize_hip
    |  (A/B + Ared/Bred + eq_hip + ctrl_hip)
    v
sysid_fast (validate model)
    |  (actual a, b, c coefficients)
    v
find_eq (equilibrium pitch)
    |  (theta_eq per hip)
    v
lqr_sweep.py OR manual tuning
    |  (K0..K3 gains)
    v
lqr_lut.csv + lqr_lut_to_header.py
    |  (lqr_lut_data.h)
    v
test_balance / simulate_mc
```

---

## 1) onshape-to-robot myRobot

**What it does**
Exports MJCF + meshes from your Onshape assembly into `myRobot/`.

**Command**
```
onshape-to-robot myRobot
```

**Output**
- `myRobot/scene.xml` (root file to load in MuJoCo)
- `myRobot/robot.xml` (robot-only MJCF)
- meshes under `myRobot/meshes/`

---

## 2) lqr_harness/linearize_hip

**What it does**
Linearizes the model across the hip range and exports full + reduced A/B and equilibrium info.

**Command**
```
./build/lqr_harness/linearize_hip --model myRobot/scene.xml \
  --hip hip_L --hip-r hip_R --min -0.27 --max 0.375 --equilibrium-wheels
```

**Outputs**
- `linearize_out/A_hip_*.csv`, `B_hip_*.csv`
- `linearize_out/Ared_hip_*.csv`, `Bred_hip_*.csv`
- `linearize_out/eq_hip_*.csv`, `ctrl_hip_*.csv`

---

## 3) tools/lqr_sweep.py

**What it does**
Sweeps Q/R scales and emits the LQR LUT (with optional `theta_eq`, `u_eq`).

**Command**
```
python3 tools/lqr_sweep.py --diag --include-eq --eq-dir linearize_out --eq-tol 1e-5
```

**Outputs**
- `lqr_sweep_results.csv`
- `lqr_lut.csv`

---

## 4) lqr_harness/linearize_hip (Detailed)

**What it does**
Sweeps hip angle, sets both hips, optionally solves equilibrium wheel torque, then linearizes the model using `mjd_transitionFD`.

**Command**
```
./build/lqr_harness/linearize_hip --equilibrium-wheels \
  --model myRobot/scene.xml \
  --hip hip_L --hip-r hip_R \
  --min -0.27 --max 0.375
```

**Outputs**
- `linearize_out/A_hip_*.csv`, `B_hip_*.csv`
  Full MuJoCo discrete linearization with state `x=[qvel,qpos,act]`, input `u=ctrl`
- `linearize_out/Ared_hip_*.csv`, `Bred_hip_*.csv`
  Reduced 4-state model for MotionController:
  - state = `[x, xdot, theta, thetadot]`
  - input = `u_sum` **torque (N·m)**
- `linearize_out/ctrl_hip_*.csv`
  Equilibrium control values (hip targets + wheel torques) at each hip

**Reduced model assumptions**
- `x = qpos[0]`
- `xdot = qvel[0]`
- `theta ≈ 2*qpos[qy]` (small angle, `qy` is free joint quaternion Y)
- `thetadot = qvel[4]` (base angular velocity around Y)
- Input uses wheel actuators summed and **converted to torque units** (`Bred` is scaled by gear).

**Units**
- `Ared`: unitless discrete-time transition
- `Bred`: per **N·m** input
- `ctrl_hip_*.csv`: `act_*` in ctrl units, `u_sum` in ctrl units (before gear)

---

---

## LUT Generation Process (Full Workflow)

### Step 1: System Identification

Run `sysid_fast` to measure actual system dynamics from MuJoCo:

```bash
cd simulate_app/build
cmake -DMUJOCO_DIR=/path/to/mujoco-3.4.0 .. && make -j$(nproc) sysid_fast
./sysid_fast ../../myRobot/scene.xml
```

**What it measures:**
- `a` — gravity term (rad/s² per rad pitch): θ̈ ∝ a·θ
- `b` — torque→pitch coupling (rad/s² per Nm): θ̈ ∝ b·u
- `c` — torque→velocity coupling ((m/s²)/Nm): ẍ ∝ c·u

**Example results (nominal hip):**
- a ≈ 113 rad/s² per rad (open-loop pole ≈ 10.6 rad/s)
- b ≈ 2.95 rad/s² per Nm
- c ≈ -1.57 (m/s²)/Nm

### Step 2: Find Equilibrium Pitch

The robot's COG is NOT aligned with the wheel axis. Find `theta_eq` where zero torque balances:

```bash
make -j$(nproc) find_eq
./find_eq ../../myRobot/scene.xml
```

**Output example:**

| Hip (rad) | theta_eq (rad) | theta_eq (°) |
|-----------|----------------|--------------|
| -0.270    | -0.145         | -8.3°        |
| -0.1625   | -0.115         | -6.6°        |
| -0.055    | -0.070         | -4.0°        |
| **0.0525**| **-0.095**     | **-5.4°**    |
| 0.160     | +0.025         | +1.4°        |
| 0.2675    | +0.010         | +0.6°        |
| 0.375     | -0.100         | -5.7°        |

### Step 3: Tune LQR Gains

Either use `lqr_sweep.py` with corrected model, or tune empirically via `test_balance`:

```bash
make -j$(nproc) test_balance
./test_balance ../../myRobot/scene.xml 120 5.0
```

**Control law:** `u = -(K[0]*x_err + K[1]*v_err + K[2]*theta_err + K[3]*thetaDot)`

**Tuned gains (2026-02-11):**
```
K[0] = -5.0    (position → torque)
K[1] = -7.5    (velocity → torque)
K[2] = -400.0  (pitch → torque)
K[3] = -24.0   (pitch rate → torque)
```

### Step 4: Create lqr_lut.csv

Format: `hip,K0,K1,K2,K3,theta_eq,u_eq`

```csv
hip,K0,K1,K2,K3,theta_eq,u_eq
-0.270000,-5.0,-7.5,-400.0,-24.0,-0.145,0.0
-0.162500,-5.0,-7.5,-400.0,-24.0,-0.115,0.0
-0.055000,-5.0,-7.5,-400.0,-24.0,-0.070,0.0
0.052500,-5.0,-7.5,-400.0,-24.0,-0.095,0.0
0.160000,-5.0,-7.5,-400.0,-24.0,0.025,0.0
0.267500,-5.0,-7.5,-400.0,-24.0,0.010,0.0
0.375000,-5.0,-7.5,-400.0,-24.0,-0.100,0.0
```

### Step 5: Generate Header

```bash
python3 tools/lqr_lut_to_header.py lqr_lut.csv --out simulate_app/lqr_lut_data.h
```

Or for firmware:
```bash
python3 tools/lqr_lut_to_header.py lqr_lut.csv --out ../stm32Controller/firmware/app/control/lqr_lut_data.h
```

### Step 6: Validate

```bash
./test_balance ../../myRobot/scene.xml 120 5.0
```

Pass criteria (120s test):
- Position drift < 200m
- Velocity < 3 m/s
- Pitch < 10°

---

## Key Findings (2026-02-11)

### 1. Velocity Gain Sign (Critical!)

**K[1] must be NEGATIVE** (same sign as K[2]).

**Why:** To slow down, the robot must first lean BACKWARD. This is a non-minimum phase system:
- Positive wheel torque → body tilts backward
- Backward tilt → gravity decelerates forward motion

Previous attempts with K[1] > 0 caused: forward lean → acceleration → divergence.

### 2. Model Mismatch

Analytical linearization predicted K[2] ≈ -15, but empirical tuning needed K[2] = -400 (25× larger).

**Root cause:** MuJoCo model dynamics differ significantly from analytical inverted pendulum assumptions.

**Solution:** Always validate with `sysid_fast` before trusting analytical gains.

### 3. Equilibrium Pitch Offset

The robot needs non-zero pitch to balance at different hip angles.

**Critical fix:** `theta_ref_limit` must be > max |theta_eq|
- Old: 0.05 rad (3°) — too small, clamped theta_eq values
- New: 0.20 rad (11.5°) — allows full range

### 4. Performance Summary

| Config | x_max (120s) | v_max | Improvement |
|--------|--------------|-------|-------------|
| Baseline (no theta_eq, weak gains) | 2323m | 25.8 m/s | — |
| Tuned gains only | 615m | 5.9 m/s | 3.8× |
| **Tuned + theta_eq + theta_ref_limit** | **101m** | **1.3 m/s** | **23×** |

---

## Notes / Gotchas

- Use `env -u PYTHONHOME -u PYTHONPATH` when running Python tools to avoid Fusion's Python interfering.
- `mjd_transitionFD` requires Euler integrator; the tool switches automatically.
- For symmetric COM, use `--mirror-left-to-right` to generate right side from left.
- If LQR sweep finds no stable Q/R, try smaller R or check B scaling.
- **Always check `theta_ref_limit`** — if theta_eq values are being clamped, the controller can't converge.
- **Test at 120s minimum** — 60s tests can pass with unstable configs that diverge later.
