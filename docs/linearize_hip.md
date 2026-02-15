# `linearize_hip` Guide

This document explains what `lqr_harness/linearize_hip.cpp` does and how the executable builds the linear models used by the simulation/firmware tooling.

## Purpose

`linearize_hip` generates linear state-space models of the MuJoCo robot at multiple hip configurations.

For each sampled hip angle, it can produce:

- Full linearization matrices from MuJoCo:
  - `A_hip_*.csv`
  - `B_hip_*.csv`
- Reduced models used by the controller:
  - `Ared_hip_*.csv`
  - `Bred_hip_*.csv`
- Optional reduced 3-state models:
  - `Ared3_hip_*.csv`
  - `Bred3_hip_*.csv`
- Optional equilibrium metadata:
  - `ctrl_hip_*.csv`
  - `eq_hip_*.csv`

## What State It Linearizes

The full MuJoCo transition linearization uses:

- State dimension: `2*nv + na`
- Input dimension: `nu`

The reduced 4-state controller model is:

- `x` (forward position)
- `xdot` (forward velocity)
- `theta` (pitch)
- `thetadot` (pitch rate)

In code this is extracted from:

- free-joint translation-x tangent index
- free-joint rotation-y tangent index
- corresponding velocity indices

## Typical Usage

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
- `--equilibrium-wheels`: solve wheel controls to hold the pose via inverse dynamics
- `--write-keyframes <path>`: write settled equilibrium keyframes (`<mujocoinclude>`) for the sampled hip range (requires `--equilibrium-wheels`)
- `--reduced-fd`: compute reduced A/B by finite differences directly in reduced coordinates
- `--reduced-no-x`: also output 3-state `[v, theta, thetadot]` models

## Execution Flow

## 1) Parse options and load model

The executable parses CLI flags into `Options`, loads the XML with `mj_loadXML`, allocates `mjData`, and forces MuJoCo integrator to Euler for `mjd_transitionFD`.

## 2) Choose initial seed state

It resolves `--key` via `mj_name2id(..., mjOBJ_KEY, ...)`.

- If found: `mj_resetDataKeyframe(m, d, key_id)`
- If missing: warns and falls back to `mj_resetData(m, d)`

Important: this key selection seeds the main simulation state used for linearization.
Geometric `theta_eq` computation and `--write-keyframes` pose generation use a canonical reset state, so they are designed to be invariant to `--key`.

## 3) Sweep hip range

For `i in [0, steps-1]`, hip is sampled linearly in `[hip_min, hip_max]`.

At each sample:

- Set `hip_L = hip`
- If present, set `hip_R = hip`
- `mj_forward`

## 4) Optional equilibrium solve (`--equilibrium-wheels`)

If enabled, it builds an operating point in two stages:

1. Compute geometric `theta_eq` (`compute_geometric_theta_eq`):
   - reset to canonical model state (`mj_resetData`)
   - apply hip servo targets (`ctrl = hip_angle * gear`)
   - settle constraints with base fixed
   - compute upper-body COM (excluding wheels)
   - derive pitch so COM aligns above wheel contact:
     - `theta_eq = -atan2(com_rel_x, com_rel_z)`

2. Solve wheel controls for static equilibrium (`set_equilibrium_wheel_ctrl`):
   - set hip controls
   - zero `qvel/qacc/external forces`
   - run `mj_inverse`
   - map required wheel joint torques to actuator controls (`ctrl = tau / gear`)

Then it writes:

- `ctrl_hip_*.csv`: actuator controls at the operating point
- `eq_hip_*.csv`: `theta_eq`, `u_eq`, and per-wheel torques

If `--write-keyframes` is set, it also generates one settled `qpos` keyframe per sampled hip:

- key names: `eq_hip_mXXXX` / `eq_hip_pXXXX`
- base pose adjusted so wheel contacts sit on the ground
- passive linkage/contact settled with hips held and wheels unforced
- generation starts from canonical reset state for deterministic output

## 5) Full linearization from MuJoCo

It calls:

- `mjd_transitionFD(m, d, eps, 1, A, B, nullptr, nullptr)`

and writes:

- `A_hip_*.csv`
- `B_hip_*.csv`

These are in MuJoCo transition form (discrete one-step Jacobians around the current state/control).

## 6) Build reduced 4-state model

By default (`--no-reduced` not set), it creates:

- `Ared = C * A * S`
- `Bred = C * B * U`

where:

- `C` maps full state to reduced `[x, xdot, theta, thetadot]`
- `S` maps reduced perturbations back to full state tangent coordinates
- `U` is implicit in code by summing wheel-L and wheel-R columns into a single `u_sum`

Output:

- `Ared_hip_*.csv` (4x4)
- `Bred_hip_*.csv` (4x1)

## 7) Optional reduced finite-difference mode (`--reduced-fd`)

Instead of projecting full `A/B`, it perturbs reduced coordinates directly and simulates one step:

- perturbs `x`, `xdot`, `theta`, `thetadot`
- perturbs `u_sum` by splitting delta equally across both wheels
- estimates derivatives numerically from next-step reduced state

This can be useful to validate projection-based reduced matrices.

## 8) Optional 3-state model (`--reduced-no-x`)

If enabled, it also builds `[v, theta, thetadot]` models:

- projection method by default
- FD method if `--reduced-fd` is set

Outputs:

- `Ared3_hip_*.csv` (3x3)
- `Bred3_hip_*.csv` (3x1)

## Units and Conventions

- Full `B` from `mjd_transitionFD` is in actuator control units.
- Reduced `Bred` in projection mode is also kept in control units (no gear conversion applied there).
- With `--equilibrium-wheels`, inverse-dynamics torque is converted to control using actuator gear.

## Common Pitfalls

- Missing `--key` seed: if the requested keyframe does not exist, tool falls back to default reset state and prints a warning.
- Model convention mismatch: reduced-state extraction assumes forward along free-joint X and pitch about free-joint Y.
- Wheel-input reduction: by default `u_sum` is formed by summing left/right wheel input columns, so any left/right asymmetry at the operating point appears in the reduced input channel.

## Generated File Naming

For sample `hip = 0.2675`:

- `A_hip_0.267500.csv`
- `B_hip_0.267500.csv`
- `Ared_hip_0.267500.csv`
- `Bred_hip_0.267500.csv`
- optional `Ared3_hip_0.267500.csv`, `Bred3_hip_0.267500.csv`
- optional `ctrl_hip_0.267500.csv`, `eq_hip_0.267500.csv`
