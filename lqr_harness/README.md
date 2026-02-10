# MuJoCo LQR Harness

This harness links the STM32 `MotionController` LQR code into a MuJoCo sim loop.

## Build

Set `MUJOCO_DIR` (or `MUJOCO_ROOT`) to your MuJoCo install path, then:

```bash
cd lqr_harness
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
cd lqr_harness/build
./lqr_harness --model ../../robot_planar.xml --key r50 --time 5
```

Useful options:
- `--no-hold-hips` disables hip hold torque.
- `--hip-target-key` holds hips at keyframe angles (default is mid-range of joint limits).
- `--no-ekf` disables EKF and uses direct torso kinematics for `theta/xDot`.
- `--headless` runs without a viewer window.
- `--fast` disables real-time sync (runs as fast as possible).
- `--hip-kp`, `--hip-kd` tune hip hold stiffness.
- `--u-scale` scales LQR output before sending to wheel actuators.
- `--print 0` disables periodic logging.

Notes:
- The harness uses `StateEstimate` fields: `theta`, `thetaDot`, `xDot` from the MuJoCo torso.
- `u_sum` is applied symmetrically to `act_wheel_L` and `act_wheel_R`.
- If wheel actuator `ctrlrange` is narrow (e.g., `[-1,1]`), use `--u-scale` or widen `ctrlrange` in the XML.
