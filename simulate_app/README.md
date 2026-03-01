# `simulate_app` (`simulate_mc` + `test_balance`)

This directory contains the MuJoCo simulator binaries used in the LQR workflow:

- `simulate_mc`: interactive simulator with keyboard controls
- `test_balance`: headless balance test used by sweeps/pipeline

## Build

From repository root:

```bash
cmake -S . -B build
cmake --build build --target simulate_mc test_balance test_motion_controller_bridge
```

## Automated Tests

`simulate_app` now has headless `ctest` coverage for:

- bridge hip-target LUT stepping and gear-scaled actuator targets
- short `test_balance` smoke run (ground-truth path)
- short `test_balance` smoke run (`StateEstimator` path with `SIM_EST_THETA_ONLY=1`)
- keyframe selection by name and index

Run:

```bash
ctest --test-dir build/simulate_app/tests --output-on-failure
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
- `↑` / `↓`: teleop forward/backward (press-and-hold ramps command; release returns to zero)
- `←` / `→`: teleop turn left/right while running (press-and-hold ramps; release returns to zero)
- `←` / `→` when paused: legacy step backward/forward
- `i`: move hips to next LUT hip setpoint
- `k`: move hips to previous LUT hip setpoint
- `v`: toggle MotionController HUD (`v_ref`, `vx`, wheel torques, running max `|tau|`)

## `test_balance` (headless)

Run:

```bash
build/simulate_app/test_balance myRobot/scene.xml --duration 240 --print-dt 10 --key eq_hip_p0525
```

Usage:

```bash
build/simulate_app/test_balance [model_path] [duration_s] [--diag] \
  [--duration seconds] [--print-dt seconds] [--key name_or_index] \
  [--turn left|right] [--turn-for seconds] \
  [--theta-disturb-deg deg] [--disturb-time s] [--wheel-limit-nm nm] \
  [--settle-theta-err-deg deg] [--settle-theta-dot rad_s] [--settle-hold s] \
  [--target-speed-mps mps] [--speed-start-time s] \
  [--speed-ramp-mps2 mps2] [--lqr-v-ref-limit mps] \
  [--speed-settle-err-mps mps] [--speed-settle-hold s]
```

Notes:

- Pass/fail is keyframe-aware and uses initial lean as reference.
- `--diag` adds IMU/encoder diagnostic summaries at the end.
- `--theta-disturb-deg` applies an instantaneous pitch disturbance and reports recovery torque.
- `--wheel-limit-nm` overrides wheel actuator `ctrlrange` and LQR `u_limit` for the test run.
- `RECOVERY_SUMMARY` is printed when disturbance mode is enabled.
- `--target-speed-mps` applies a direct speed target and reports torque-to-speed metrics.
- `--speed-ramp-mps2` sets speed-command slew rate in m/s² (`0` = step command).
- `--lqr-v-ref-limit` overrides LQR velocity-reference clamp in m/s (`0` disables clamp, default from firmware is `2.0`).
- `SPEED_SUMMARY` is printed when speed mode is enabled.
- Disturbance mode and speed mode are mutually exclusive in one run.
- In speed mode, PASS/FAIL is based on divergence plus speed stabilization (not theta-error bound).

### Disturbance recovery torque example

Run:

```bash
SIM_USE_EKF=0 build/simulate_app/test_balance myRobot/scene.xml \
  --duration 8 --print-dt 0.5 --key eq_hip_p0525 \
  --theta-disturb-deg 5 --disturb-time 0.5 --wheel-limit-nm 35
```

Key output lines:

```text
Applied disturbance at t=0.500 s: Δtheta=+5.000° (...)
Recovery torque peak |tau|: L=2.0831 Nm  R=2.0832 Nm  max=2.0832 Nm
Stabilized at t=1.340 s (0.840 s after disturbance), max |tau| until stabilize=2.0832 Nm
RECOVERY_SUMMARY disturb_deg=5.000000 disturb_time=0.500000 stabilized=1 ...
```

`RECOVERY_SUMMARY` fields:

- `stabilized=1|0`: whether settle criteria were met before test end
- `settle_time`: absolute simulation time when stabilized (`-1` if not stabilized)
- `settle_delay`: delay since disturbance (`-1` if not stabilized)
- `max_tau_l`, `max_tau_r`, `max_tau`: peak absolute wheel torques during recovery
- `max_tau_to_settle`: peak absolute wheel torque from disturbance until stabilization

### Speed target torque example

Run:

```bash
SIM_USE_EKF=0 build/simulate_app/test_balance myRobot/scene.xml \
  --duration 8 --print-dt 0.5 --key eq_hip_p0525 \
  --target-speed-mps 0.5 --speed-start-time 0.5 --wheel-limit-nm 35
```

Key output lines:

```text
Applied speed command at t=0.500 s: v_ref=+0.500 m/s (ramp 1.000 m/s² from -0.087 m/s)
Speed-mode torque peak |tau|: ... max=0.8764 Nm
Speed stabilized at t=2.134 s (1.634 s after command), max |tau| until stabilize=0.8764 Nm
SPEED_SUMMARY target_speed_mps=0.500000 ... speed_ramp_mps2=1.000000 ... max_tau=0.876357
```

`SPEED_SUMMARY` fields:

- `target_speed_mps`, `command_time`
- `stabilized`, `settle_time`, `settle_delay`
- `max_speed_mps`, `final_speed_mps`
- `speed_ramp_mps2`, `commanded_speed_mps`, `lqr_v_ref_limit_mps` (`-1` means default firmware clamp)
- `max_tau_l`, `max_tau_r`, `max_tau`, `max_tau_to_settle`

### High speed command example (2.8 m/s)

Use a raised LQR `v_ref` clamp plus command ramp:

```bash
SIM_USE_EKF=0 build/simulate_app/test_balance myRobot/scene.xml \
  --duration 10 --print-dt 0.5 --key eq_hip_p0525 \
  --target-speed-mps 2.8 --speed-start-time 0.5 \
  --speed-ramp-mps2 0.8 --lqr-v-ref-limit 3.0 --wheel-limit-nm 35
```

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

LQR speed scheduler flags (rest/cruise blending):

- `SIM_LQR_SPEED_SCHED_ENABLE=0|1`: disable/enable speed scheduling (default enabled)
- `SIM_LQR_CRUISE_ENTER_CMD_MPS`, `SIM_LQR_CRUISE_EXIT_CMD_MPS`
- `SIM_LQR_CRUISE_ENTER_MEAS_MPS`, `SIM_LQR_CRUISE_EXIT_MEAS_MPS`
- `SIM_LQR_CRUISE_BLEND_TAU_S`
- `SIM_LQR_VREF_MARGIN_MPS`
- `SIM_LQR_THETA_LIMIT_REST_CAP_RAD`, `SIM_LQR_THETA_LIMIT_CRUISE_CAP_RAD`
- `SIM_LQR_YAW_DAMP_CRUISE_MULT`
- Existing clamp override still applies: `SIM_LQR_V_REF_LIMIT`

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
