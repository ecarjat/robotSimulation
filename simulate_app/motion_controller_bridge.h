#ifndef MOTION_CONTROLLER_BRIDGE_H_
#define MOTION_CONTROLLER_BRIDGE_H_

#include <mujoco/mujoco.h>

// Reset controller state for a newly loaded model.
void MotionControllerReset(const mjModel* m, mjData* d);

// MuJoCo control callback. Assign to mjcb_control.
void MotionControllerCallback(const mjModel* m, mjData* d);

// Toggle controller enable state; returns new state.
bool MotionControllerToggle();

// Returns whether the controller is currently enabled.
bool MotionControllerIsEnabled();

// Toggle wheel control enable state; returns new state.
bool WheelControlToggle();

// Toggle static measurement mode; returns new state.
// In static mode, the robot base is fixed in place and wheels are disabled.
bool StaticModeToggle();

// Step hip targets to the next LUT angle. Positive direction moves up, negative moves down.
// Returns true if the target changed.
bool MotionControllerStepHipTarget(int direction);

#endif  // MOTION_CONTROLLER_BRIDGE_H_
