# `linearize_hip` Test Suite

This folder contains regression tests for `/Users/emmanuelcarjat/git/robot2Wheel/simulation/lqr_harness/linearize_hip.cpp`.

## What Is Tested

The CMake target file `/Users/emmanuelcarjat/git/robot2Wheel/simulation/lqr_harness/tests/CMakeLists.txt` defines five active tests:

1. `test_equilibrium_solver`
2. `test_linearization_output`
3. `test_position_actuators`
4. `test_seed_invariance`
5. `test_grounded_keyframes`

These tests cover equilibrium solving, linearization quality, actuator semantics, seed-keyframe invariance, and keyframe grounding.

## Key Model/Control Assumption

For hip position actuators with non-unit gear, control is in actuator transmission coordinates.

- Transmission relation: `actuator_length = joint_angle * gear`
- Position servo target is `ctrl = actuator_length`
- Therefore, to command joint target `q_target`, use `ctrl = q_target * gear`.

The current tests and implementation assume this behavior.

## Test Details

### `test_equilibrium_solver.cpp`

Checks equilibrium solve correctness and actuator handling:

- `theta_eq` varies monotonically over the hip sweep.
- `theta_eq` range is non-trivial across the sampled hip values.
- Wheel inverse-dynamics torques are near zero at computed equilibrium.
- Gear-scaled position control drives hips to requested joint angles.

### `test_linearization_output.cpp`

Checks reduced linearization matrix sanity:

- Finite `A` and `B` values from `mjd_transitionFD`.
- Expected reduced `B` structure and magnitudes for wheel torque input.
- Expected reduced `A` couplings (for example, position-from-velocity and angle-from-angular-velocity terms).
- Determinism across repeated linearization calls from the same state.

### `test_position_actuators.cpp`

Validates MuJoCo position-actuator behavior used by this project:

- Gear-scaled control reaches target hip angles.
- Hip joints respect limits when commanded beyond range.
- Both hips can traverse full joint range (including bounds/interior points).
- Behavior remains consistent with the configured gear/transmission semantics.

### `test_seed_invariance.cpp`

Runs `linearize_hip` twice (default seed and explicit `--key eq_hip_p0525`) and verifies all generated `theta_eq` values match exactly for each hip row. This guards against seed-dependent equilibrium drift.

### `test_grounded_keyframes.cpp`

Runs `linearize_hip --write-keyframes ...`, parses generated keyframes, loads each into MuJoCo, and verifies both wheel contact points are on ground (within tolerance). This guards against floating/sunken equilibrium keyframes.

## Helper Program

`/Users/emmanuelcarjat/git/robot2Wheel/simulation/lqr_harness/tests/inspect_model.cpp` is a standalone inspection utility (not registered as a `ctest` test). It prints hip actuator gear/ctrlrange and hip joint ranges.

## Run

From repo root:

```bash
cmake -S /Users/emmanuelcarjat/git/robot2Wheel/simulation -B /Users/emmanuelcarjat/git/robot2Wheel/simulation/build
cmake --build /Users/emmanuelcarjat/git/robot2Wheel/simulation/build --target test_equilibrium_solver test_linearization_output test_position_actuators test_seed_invariance test_grounded_keyframes
ctest --test-dir /Users/emmanuelcarjat/git/robot2Wheel/simulation/build/lqr_harness/tests --output-on-failure
```

## Maintenance Rules

- If `linearize_hip` behavior changes intentionally, update the affected test expectations in the same change.
- Keep assertions deterministic (fixed seeds, fixed tolerances, no time-dependent randomness).
- Prefer adding regression coverage for each bug fix before changing algorithmic behavior.
