# Test Update Required

The test files currently test the WRONG behavior (setting ctrl without gear multiplication).

They need to be updated to test the CORRECT behavior that linearize_hip now uses.

## Summary for User

We discovered that MuJoCo position actuators with `gear=G` require:
```cpp
d->ctrl[act] = target_angle * G;  // To reach target_angle
```

The linearization now correctly multiplies by gear ratio. The tests need updating to:
1. Multiply control signals by gear ratio
2. Verify joints reach commanded angles
3. Document this as the correct behavior

## Next Steps

The user should review:
1. Is `gear=5` in robot.xml correct for their hardware?
2. Should we change robot.xml to use `gear=1` to simplify?  
3. Or update all test files to use gear-aware control?

For now, linear ize_hip.cpp is CORRECT. Tests need updating to match.
