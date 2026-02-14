# Linearization Test Suite

This directory contains regression tests for `linearize_hip` to ensure the linearization pipeline remains correct.

## Critical Finding: MuJoCo Position Actuator Gear Semantics

### The Bug
Original code multiplied hip control by gear ratio (`ctrl = hip_angle * 5.0`), which was considered a "bug" and removed. However, tests revealed this was actually CORRECT!

### MuJoCo Position Actuator Behavior with `gear` parameter

For position actuators in MuJoCo:
```xml
<position name="hip_L" joint="hip_L" gear="5" />
```

The relationship is:
- **Control signal:** `ctrl` (in transmission space)
- **Joint position:** `qpos` (in joint space)
- **Relationship:** `qpos = ctrl / gear`

To command a joint to angle θ:
```cpp
d->ctrl[act_id] = θ * gear;  // Results in qpos = θ
```

### Why theta_eq Varied Without Gear Multiplication

When we removed the `* 5.0` multiplication:
- Command: `ctrl = -0.27` (thinking it sets hip to -0.27 rad)
- Actual joint: `hip ≈ -0.27 / 5 = -0.054` rad

The equilibrium solver computed theta_eq for the *actual* hip angle (-0.054), not the intended one (-0.27). Since the robot settled at different actual hip angles for different ctrl values, theta_eq varied - but the eq files were WRONG, labeling data with incorrect hip angles!

### The Fix

Updated `linearize_hip.cpp` to detect gear ratio and apply it correctly:
```cpp
double gear = m->actuator_gear[6 * act_id];
d->ctrl[act_id] = target_angle * gear;  // Now joint reaches target_angle
```

## Test Files

### test_equilibrium_solver.cpp
**Purpose:** Regression test for the critical equilibrium solver bug.

**Key Tests:**
1. `theta_eq varies with hip angle` - Core regression test
   - Ensures theta_eq changes monotonically across hip range
   - Minimum 0.57° variation required
   - Catches gear multiplication bugs

2. `Near-zero equilibrium torques`
   - Validates computed equilibrium states
   - Wheel torques should be < 1 mNm at equilibrium

3. `Position actuator gear handling`
   - Verifies correct use of gear ratio
   - Joint should reach commanded angle (not angle/gear)

### test_linearization_output.cpp
**Purpose:** Validates linearized state-space matrices.

**Key Tests:**
1. `Finite A, B matrices` - Basic sanity check
2. `Bred structure` - Validates control authority vector
   - B[3] (θ̇) > B[1] (v) > B[0] (x), B[2] (θ)
   - Ensures controllability
3. `Ared structure` - Validates dynamics matrix
   - Not identity (linearization worked)
   - Correct coupling (position←velocity, angle←angular velocity)
4. `Linearization consistency` - Deterministic output

### test_position_actuators.cpp
**Purpose:** Documents and validates MuJoCo actuator semantics.

**Key Tests:**
1. `Gear-aware position control` - Primary validation
   - With `gear=5`: `ctrl = 5*θ` results in `joint = θ`
2. `Joint limits` - Actuators respect URDF limits
3. `Full range of motion` - Hip actuators can reach all angles
4. `Motor vs position actuators` - Different gear semantics

## Running Tests

```bash
cd /Users/emmanuelcarjat/git/robot2Wheel/simulation/lqr_harness/build
cmake ..
make
cd tests
ctest --output-on-failure
```

## Expected Results

All tests should PASS. Failures indicate:
- **test_equilibrium_solver fails:** Gear multiplication bug reintroduced
- **test_linearization_output fails:** Matrix structure changed (investigate if intentional)
- **test_position_actuators fails:** Actuator definitions in robot.xml changed

## Adding New Tests

When modifying `linearize_hip.cpp`:
1. Add regression test if fixing a bug
2. Update test expectations if changing behavior intentionally
3. Document rationale in test comments
4. Ensure tests are deterministic (no random state)

## References

- MuJoCo Documentation: https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator
- Position actuator: `actuation = gain  ⋅ (length(ctrl) - length(qpos))`
- For `gear ≠ 0`: transmission ratio in generalized coordinates
