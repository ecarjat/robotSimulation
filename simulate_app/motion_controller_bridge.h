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

#endif  // MOTION_CONTROLLER_BRIDGE_H_
