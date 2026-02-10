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

3) **Sweep Q/R and generate LUT**  
   `python3 tools/lqr_sweep.py --diag --include-eq --eq-dir linearize_out --eq-tol 1e-5`  
   → `lqr_sweep_results.csv`, `lqr_lut.csv`

4) **Run simulation with MotionController**  
   `simulate_mc`

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
tools/lqr_sweep.py
    |  (Q/R sweep + lqr_lut.csv)
    v
simulate_mc (MotionController)
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

## 4) simulate_mc

**What it does**  
Runs the simulation with MotionController + the LUT.

**Command**
```
simulate_mc
```


---

## 4) lqr_harness/linearize_hip

**What it does**  
Sweeps hip angle, sets both hips, optionally solves equilibrium wheel torque, then linearizes the model using `mjd_transitionFD`.

**Command**
```
./build/lqr_harness/linearize_hip --equilibrium-wheels
```

**With myRobot export**
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
  Reduced 4‑state model for MotionController:
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

## 5) tools/lqr_sweep.py

**What it does**  
Sweeps Q/R to find gains that stabilize **all hip angles** and outputs a LUT.

**Command**
```
env -u PYTHONHOME -u PYTHONPATH /Library/Developer/CommandLineTools/usr/bin/python3 \
  tools/lqr_sweep.py --diag
```

**Outputs**
- `lqr_sweep_results.csv`: all Q/R candidates with stability metrics
- `lqr_lut.csv`: hip → K0..K3 for best candidate

**Diagnostics (`--diag`)**
- Open-loop spectral radius `rho(A)`
- Input norm `||B||`
- Controllability rank

**Units**
- State: `[x, xdot, theta, thetadot]` in m, m/s, rad, rad/s
- Input: `u_sum` in **N·m**

---

## Notes / Gotchas

- Use `env -u PYTHONHOME -u PYTHONPATH` when running Python tools to avoid Fusion’s Python interfering.
- `mjd_transitionFD` requires Euler integrator; the tool switches automatically.
- For symmetric COM, use `--mirror-left-to-right` to generate right side from left.
- If LQR sweep finds no stable Q/R, try smaller R or check B scaling.
