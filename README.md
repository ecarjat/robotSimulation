# Robot2Wheel Simulation Pipeline

This repo uses a multi-step pipeline to go from CAD → MJCF → linearization → LQR gains → simulation. This README documents each tool, what it expects, what it outputs, and the units.

## Overview (pipeline)

1) **Export MJCF from Onshape (if geometry changed)**
   `onshape-to-robot myRobot`
   → `myRobot/scene.xml`, `myRobot/robot.xml`, meshes

2) **Run the full LQR pipeline (recommended)**
   `python3 tools/run_lqr_pipeline.py`
   → linearizes with equilibrium wheel solve and writes keyframes  
   → runs `tools/lqr_sweep.py` (3-state LUT, eq included, no sign flip)  
   → runs adaptive per-hip `K0` search with `tools/k0_sweep.py` (ground-truth by default)  
   → updates `lqr_lut.csv` `K0` column  
   → regenerates `../stm32Controller/firmware/app/control/lqr_lut_data.h`

3) **Validate**
   `build/simulate_app/test_balance myRobot/scene.xml 240 --key eq_hip_p0525`
   (and/or run all keyframes)
   See [`simulate_app/README.md`](simulate_app/README.md) for `simulate_mc`/`test_balance`
   usage, key controls, and current estimator workaround.

### Recommended one-command workflow

```bash
python3 tools/run_lqr_pipeline.py
```

### Manual equivalent workflow

```bash
build/lqr_harness/linearize_hip \
  --model myRobot/scene.xml \
  --equilibrium-wheels \
  --write-keyframes myRobot/keyframes_eq.xml \
  --out linearize_out \
  --reduced-no-x

python3 tools/lqr_sweep.py \
  --dir linearize_out \
  --state-dim 3 \
  --include-eq \
  --no-sign-flip \
  --low-hip-min -0.270 \
  --low-hip-max -0.055 \
  --low-hip-k2-boost 2.0 \
  --low-hip-k3-boost 1.5 \
  --lut-out lqr_lut.csv

# Per-hip K0 sweep (run once per keyframe, or use run_lqr_pipeline.py to automate this).
python3 tools/k0_sweep.py --duration 2 --k0-values=-0.25 --keys eq_hip_p0525 --run-csv k0_sweep_runs.csv

python3 tools/lqr_lut_to_header.py \
  --out ../stm32Controller/firmware/app/control/lqr_lut_data.h \
  lqr_lut.csv
```

### Tool docs (detailed)

- `linearize_hip`: [`Linearize Hip Guide`](#linearize-hip-guide)
- `lqr_sweep.py`: [`tools/README.md#lqr_sweeppy`](tools/README.md#lqr_sweeppy)
- `k0_sweep.py`: [`tools/README.md#k0_sweeppy`](tools/README.md#k0_sweeppy)
- `run_lqr_pipeline.py`: [`docs/run_lqr_pipeline.md`](docs/run_lqr_pipeline.md)
- `simulate_mc` and `test_balance`: [`simulate_app/README.md`](simulate_app/README.md)

```
Onshape
    |
    v
onshape-to-robot myRobot
    |  (myRobot/scene.xml + meshes)
    v
tools/run_lqr_pipeline.py
    |  (linearize_hip + lqr_sweep + adaptive k0_sweep)
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

## Linearize Hip Guide

This section explains what `lqr_harness/linearize_hip.cpp` does and how it builds the linear models used by simulation and firmware tooling.

### Purpose

`linearize_hip` generates linear state-space models of the MuJoCo robot at multiple hip configurations.

For each sampled hip angle, it can produce:

- full linearization matrices from MuJoCo: `A_hip_*.csv`, `B_hip_*.csv`
- reduced models used by the controller: `Ared_hip_*.csv`, `Bred_hip_*.csv`
- optional reduced 3-state models: `Ared3_hip_*.csv`, `Bred3_hip_*.csv`
- optional equilibrium metadata: `ctrl_hip_*.csv`, `eq_hip_*.csv`

### What state it linearizes

The full MuJoCo transition linearization uses:

- state dimension: `2*nv + na`
- input dimension: `nu`

The reduced 4-state controller model is:

- `x` (forward position)
- `xdot` (forward velocity)
- `theta` (pitch)
- `thetadot` (pitch rate)

In code this is extracted from:

- free-joint translation-x tangent index
- free-joint rotation-y tangent index
- corresponding velocity indices

### Typical usage

```bash
build/lqr_harness/linearize_hip \
  --model myRobot/scene.xml \
  --key eq_hip_p0525 \
  --equilibrium-wheels \
  --min -0.27 \
  --max 0.375 \
  --steps 7 \
  --out linearize_out
```

Key options:

- `--model`: MuJoCo XML (typically `myRobot/scene.xml`)
- `--key`: keyframe seed for operating point initialization
- `--equilibrium-wheels`: solve wheel controls to hold pose via inverse dynamics
- `--write-keyframes <path>`: write settled equilibrium keyframes (`<mujocoinclude>`) for the sampled hip range (requires `--equilibrium-wheels`)
- `--reduced-fd`: compute reduced A/B by finite differences directly in reduced coordinates
- `--reduced-no-x`: also output 3-state `[v, theta, thetadot]` models

### Execution flow

#### 1) Parse options and load model

The executable parses CLI flags into `Options`, loads XML with `mj_loadXML`, allocates `mjData`, and forces the MuJoCo integrator to Euler for `mjd_transitionFD`.

#### 2) Choose initial seed state

It resolves `--key` via `mj_name2id(..., mjOBJ_KEY, ...)`.

- if found: `mj_resetDataKeyframe(m, d, key_id)`
- if missing: warning then `mj_resetData(m, d)`

Important: keyframe selection seeds the main simulation state used for linearization. Geometric `theta_eq` computation and `--write-keyframes` pose generation use a canonical reset state, so they are designed to be invariant to `--key`.

#### 3) Sweep hip range

For `i in [0, steps-1]`, hip is sampled linearly in `[hip_min, hip_max]`.

At each sample:

- set `hip_L = hip`
- if present, set `hip_R = hip`
- run `mj_forward`

#### 4) Optional equilibrium solve (`--equilibrium-wheels`)

If enabled, it builds an operating point in two stages.

1. Compute geometric `theta_eq` (`compute_geometric_theta_eq`):
- reset to canonical model state (`mj_resetData`)
- apply hip servo targets (`ctrl = hip_angle * gear`)
- settle constraints with base fixed
- compute upper-body COM (excluding wheels)
- derive pitch so COM aligns above wheel contact: `theta_eq = -atan2(com_rel_x, com_rel_z)`
2. Solve wheel controls for static equilibrium (`set_equilibrium_wheel_ctrl`):
- set hip controls
- zero `qvel/qacc/external forces`
- run `mj_inverse`
- map required wheel-joint torques to actuator controls (`ctrl = tau / gear`)

Then it writes:

- `ctrl_hip_*.csv`: actuator controls at the operating point
- `eq_hip_*.csv`: `theta_eq`, `u_eq`, and per-wheel torques

If `--write-keyframes` is set, it also generates one settled `qpos` keyframe per sampled hip:

- key names: `eq_hip_mXXXX` / `eq_hip_pXXXX`
- base pose adjusted so wheel contacts sit on the ground
- passive linkage/contact settled with hips held and wheels unforced
- generation starts from canonical reset state for deterministic output

#### 5) Full linearization from MuJoCo

It calls `mjd_transitionFD(m, d, eps, 1, A, B, nullptr, nullptr)` and writes:

- `A_hip_*.csv`
- `B_hip_*.csv`

These are discrete one-step Jacobians around the current state/control.

#### 6) Build reduced 4-state model

By default (`--no-reduced` not set), it creates:

- `Ared = C * A * S`
- `Bred = C * B * U`

where:

- `C` maps full state to reduced `[x, xdot, theta, thetadot]`
- `S` maps reduced perturbations back to full-state tangent coordinates
- `U` is implicit in code by summing `wheel_L` and `wheel_R` columns into one `u_sum`

Outputs:

- `Ared_hip_*.csv` (4x4)
- `Bred_hip_*.csv` (4x1)

#### 7) Optional reduced finite-difference mode (`--reduced-fd`)

Instead of projecting full `A/B`, it perturbs reduced coordinates directly and simulates one step:

- perturbs `x`, `xdot`, `theta`, `thetadot`
- perturbs `u_sum` by splitting delta equally across both wheels
- estimates derivatives numerically from next-step reduced state

This is useful to validate projection-based reduced matrices.

#### 8) Optional 3-state model (`--reduced-no-x`)

If enabled, it also builds `[v, theta, thetadot]` models:

- projection method by default
- FD method if `--reduced-fd` is set

Outputs:

- `Ared3_hip_*.csv` (3x3)
- `Bred3_hip_*.csv` (3x1)

### Units and conventions

- full `B` from `mjd_transitionFD` is in actuator control units
- reduced `Bred` in projection mode is also kept in control units
- with `--equilibrium-wheels`, inverse-dynamics torque is converted to control using actuator gear

### Common pitfalls

- missing `--key` seed: if keyframe does not exist, tool falls back to default reset state and prints a warning
- model convention mismatch: reduced-state extraction assumes forward along free-joint X and pitch about free-joint Y
- wheel-input reduction: default `u_sum` sums left/right wheel columns, so asymmetry can appear in reduced input channel

### Generated file naming

For sample `hip = 0.2675`:

- `A_hip_0.267500.csv`
- `B_hip_0.267500.csv`
- `Ared_hip_0.267500.csv`
- `Bred_hip_0.267500.csv`
- optional `Ared3_hip_0.267500.csv`, `Bred3_hip_0.267500.csv`
- optional `ctrl_hip_0.267500.csv`, `eq_hip_0.267500.csv`

---

## LUT Generation Process (Full Workflow)

This section describes the **current recommended process** to generate `lqr_lut.csv` and firmware header data.

### Step 1: Run the orchestrator

```bash
python3 tools/run_lqr_pipeline.py
```

Default orchestrator behavior:

- runs `build/lqr_harness/linearize_hip` with:
  - `--equilibrium-wheels`
  - `--write-keyframes myRobot/keyframes_eq.xml`
  - `--out linearize_out`
  - `--reduced-no-x`
- runs `python3 tools/lqr_sweep.py` with:
  - `--dir linearize_out`
  - `--state-dim 3`
  - `--include-eq`
  - `--no-sign-flip`
  - `--low-hip-min -0.270`
  - `--low-hip-max -0.055`
  - `--low-hip-k2-boost 2.0`
  - `--low-hip-k3-boost 1.5`
  - `--lut-out lqr_lut.csv`
- runs per-hip `python3 tools/k0_sweep.py` search:
  - default seeds: `-100,-50,-1`
  - adaptive refinement rounds around current best value
  - default mode: `SIM_USE_EKF=0` (ground-truth tuning)
- updates `K0` in `lqr_lut.csv` per hip
- regenerates:
  - `../stm32Controller/firmware/app/control/lqr_lut_data.h`

### Step 2: (Optional) control K0 search aggressiveness

Example with wider initial range and more refinement:

```bash
python3 tools/run_lqr_pipeline.py \
  --k0-values=-200,-100,-50,-10,-1,-0.1 \
  --k0-refine-rounds 3 \
  --k0-max-new-per-round 4 \
  --k0-duration 60 \
  --k0-print-dt 20
```

Notes:

- use longer `--k0-duration` for more reliable ranking
- keep `--k0-sim-use-ekf 0` while identifying control gains
- use `--k0-sim-use-ekf 1` only for estimator-path sensitivity checks

### Step 3: Validate

Run keyframe-aware balance tests after LUT/header update:

```bash
build/simulate_app/test_balance myRobot/scene.xml 240 --key eq_hip_p0525 --print-dt 240
```

For disturbance recovery torque measurement (with high wheel/LQR limit for sim):

```bash
SIM_USE_EKF=0 build/simulate_app/test_balance myRobot/scene.xml \
  --duration 8 --print-dt 0.5 --key eq_hip_p0525 \
  --theta-disturb-deg 5 --disturb-time 0.5 --wheel-limit-nm 35
```

Look for `RECOVERY_SUMMARY` in output (`max_tau`, `max_tau_to_settle`, `settle_delay`).

For speed-target torque measurement (torque needed to reach a commanded speed):

```bash
SIM_USE_EKF=0 build/simulate_app/test_balance myRobot/scene.xml \
  --duration 8 --print-dt 0.5 --key eq_hip_p0525 \
  --target-speed-mps 0.5 --speed-start-time 0.5 --wheel-limit-nm 35
```

Look for `SPEED_SUMMARY` in output (`max_tau`, `max_tau_to_settle`, `settle_delay`).

For higher-speed commands (e.g. `2.8 m/s`), increase LQR velocity-reference clamp and use a command ramp:

```bash
SIM_USE_EKF=0 build/simulate_app/test_balance myRobot/scene.xml \
  --duration 10 --print-dt 0.5 --key eq_hip_p0525 \
  --target-speed-mps 2.8 --speed-start-time 0.5 \
  --speed-ramp-mps2 0.8 --lqr-v-ref-limit 3.0 --wheel-limit-nm 35
```

To sweep all LUT hip poses and report global max torque:

```bash
python3 tools/torque_sweep.py \
  --duration 8 \
  --theta-disturb-deg 5 \
  --disturb-time 0.5 \
  --wheel-limit-nm 35 \
  --csv-out linearize_out/torque_sweep_results.csv
```

To sweep hip poses and find the one that stabilizes at the highest commanded speed:

```bash
python3 tools/speed_pose_sweep.py \
  --speed-min 0.5 \
  --speed-max 3.0 \
  --speed-step 0.25 \
  --duration 10 \
  --speed-ramp-mps2 0.8 \
  --post-settle-hold 2.0 \
  --wheel-limit-nm 35 \
  --lqr-v-ref-limit 3.2 \
  --break-on-first-unstable \
  --run-csv linearize_out/speed_pose_sweep_runs.csv \
  --key-csv linearize_out/speed_pose_sweep_key_summary.csv
```

Look for `BEST_POSE` in output and `max_stable_target_speed_mps` in the per-key CSV.

For interactive `simulate_mc`, environment flags, and the current StateEstimator
workaround, see [`simulate_app/README.md`](simulate_app/README.md).

Repeat for all `eq_hip_*` keyframes you care about.

### Outputs produced

- `linearize_out/A*_hip_*.csv`, `B*_hip_*.csv`, `eq_hip_*.csv`, `ctrl_hip_*.csv`
- `myRobot/keyframes_eq.xml`
- `lqr_lut.csv` (with per-hip adapted `K0`)
- `linearize_out/k0_sweeps/*.csv` (including `k0_selected.csv`)
- `../stm32Controller/firmware/app/control/lqr_lut_data.h`

### Detailed references

- linearization details: [`Linearize Hip Guide`](#linearize-hip-guide)
- LQR sweep details: [`tools/README.md#lqr_sweeppy`](tools/README.md#lqr_sweeppy)
- K0 sweep details: [`tools/README.md#k0_sweeppy`](tools/README.md#k0_sweeppy)
- orchestrator details: [`docs/run_lqr_pipeline.md`](docs/run_lqr_pipeline.md)

---

## Key Findings (Current Workflow)

### 1. Tune `K0` on ground-truth first

Use `SIM_USE_EKF=0` during `K0` identification (`tools/k0_sweep.py` and `tools/run_lqr_pipeline.py` default).

Why:

- `linearize_hip` and `lqr_sweep` are plant/model based.
- `K0` should be selected against plant behavior first, then re-checked with EKF enabled.
- This avoids estimator-path bias contaminating gain search.

### 2. EKF accel tilt can destabilize during dynamic motion

When EKF is enabled, accel-based pitch is biased by longitudinal acceleration (`a_x/g` effect).  
This can inject wrong `theta` into control and cause drift/divergence.

Practical implication:

- If EKF-on behavior diverges while EKF-off is stable, this is typically an estimator integration/gating issue, not a pure LQR gain issue.
- Keep accel gating/variance inflation in estimator path during dynamic phases.

### 3. `linearize_hip` is the source of truth for operating points

`linearize_hip` now handles:

- equilibrium wheel solve
- grounded equilibrium keyframe generation
- reduced model export used by `lqr_sweep`

This replaces older split workflows where equilibrium came from separate tools.

### 4. Sign consistency checks matter for LUT continuity

Use `lqr_sweep.py --no-sign-flip` (or equivalent via orchestrator) to reject gain sets that change sign between neighboring hip operating points.

This avoids abrupt controller behavior across LUT interpolation regions.

### 5. `K0` should be hip-dependent and searched adaptively

The current pipeline supports per-hip adaptive `K0` search (coarse seeds + refinement rounds), then writes selected `K0` values back into `lqr_lut.csv`.

This is preferred over one global fixed `K0` for all hip postures.

---

## Notes / Gotchas

- Prefer the orchestrator (`python3 tools/run_lqr_pipeline.py`) over manual steps to avoid file/version drift between tools.
- `k0_sweep.py` and `run_lqr_pipeline.py` tune `K0` in ground-truth mode by default (`SIM_USE_EKF=0`); treat EKF-enabled runs as a validation phase.
- When passing negative comma-separated K0 lists, use the `=` form:
  `--k0-values=-100,-50,-1`
- `mjd_transitionFD` requires Euler integrator; `linearize_hip` enforces/switches this automatically.
- If `lqr_sweep.py` reports no stable Q/R, widen `--q-scale` / `--r-scale` ranges and verify `Ared/Bred` were regenerated from the current model.
- Re-run the full pipeline whenever geometry, contact model, actuator setup, or keyframes change.
- Validate over long horizons (for example 120-240s); short tests can hide slow drift modes.
