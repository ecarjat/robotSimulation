# `simulate_app` (`simulate_mc` + `test_balance`)

This directory contains the MuJoCo simulator binaries used in the LQR workflow:

- `simulate_mc`: interactive simulator with keyboard controls
- `test_balance`: headless balance test used by sweeps/pipeline

## Build

From repository root:

```bash
cmake -S . -B build
cmake --build build --target simulate_mc test_balance
```

## `simulate_mc` (interactive)

Run:

```bash
build/simulate_app/simulate_mc myRobot/scene.xml --key eq_hip_p0525
```

Usage:

```bash
build/simulate_app/simulate_mc [model_path] [--key name_or_index]
```

Useful runtime keys:

- `m`: toggle MotionController on/off
- `w`: toggle wheel control on/off
- `f`: toggle static measurement mode
- `i`: move hips to next LUT hip setpoint
- `k`: move hips to previous LUT hip setpoint

## `test_balance` (headless)

Run:

```bash
build/simulate_app/test_balance myRobot/scene.xml --duration 240 --print-dt 10 --key eq_hip_p0525
```

Usage:

```bash
build/simulate_app/test_balance [model_path] [duration_s] [--diag] \
  [--duration seconds] [--print-dt seconds] [--key name_or_index]
```

Notes:

- Pass/fail is keyframe-aware and uses initial lean as reference.
- `--diag` adds IMU/encoder diagnostic summaries at the end.

## Estimator/Control Environment Flags

Main flags:

- `SIM_USE_EKF`:
  - `0` = no EKF (ground-truth state path)
  - `1` = EKF-enabled path
- `SIM_USE_STATE_ESTIMATOR` (when `SIM_USE_EKF=1`):
  - `0` = direct `BalancerEKF` path
  - `1` = `StateEstimator` path
- `SIM_VENC_SCALE`:
  - encoder velocity scaling in the bridge (default `1.0`)

Debug/diagnostic flags:

- `SIM_MC_TRACE=1`: print bridge trace (`MC_TRACE ...`)
- `SIM_MC_TRACE_DT=<seconds>`: trace period (default `0.02`)
- `SIM_MC_DEBUG=1`: additional debug prints

Temporary estimator isolation flags:

- `SIM_EST_THETA_ONLY=1`:
  - use estimator for `theta/thetaDot`
  - use MuJoCo ground truth for `x/xDot`
- `SIM_EST_XDOT_GT=1`:
  - keep estimator `x`
  - force MuJoCo ground-truth `xDot`

## Current Fix (StateEstimator path)

Current known issue:

- full `StateEstimator` longitudinal states (`x`, `xDot`) in sim can cause instability/drift in `simulate_mc` and `test_balance`.

Current workaround:

- run EKF + StateEstimator with `SIM_EST_THETA_ONLY=1`.

Example:

```bash
SIM_USE_EKF=1 SIM_USE_STATE_ESTIMATOR=1 SIM_EST_THETA_ONLY=1 \
build/simulate_app/test_balance myRobot/scene.xml --duration 60 --key eq_hip_p0525
```

Equivalent interactive run:

```bash
SIM_USE_EKF=1 SIM_USE_STATE_ESTIMATOR=1 SIM_EST_THETA_ONLY=1 \
build/simulate_app/simulate_mc myRobot/scene.xml --key eq_hip_p0525
```

Status:

- This is a temporary simulation-side workaround while longitudinal estimator consistency is being debugged.
