#ifndef MOTION_CONTROLLER_BRIDGE_H_
#define MOTION_CONTROLLER_BRIDGE_H_

#include <mujoco/mujoco.h>

struct MotionControllerHudTelemetry {
  bool valid = false;
  bool controller_enabled = false;
  bool wheel_control_enabled = false;
  bool static_mode = false;
  double sim_time_s = 0.0;
  float v_ref_mps = 0.0f;
  float vx_mps = 0.0f;
  float tau_l_nm = 0.0f;
  float tau_r_nm = 0.0f;
  float tau_abs_peak_nm = 0.0f;
  int requested_mode = 0;  // 0=PID, 1=LQR
  int active_mode = 0;     // 0=PID, 1=LQR
  bool stop_mode_active = false;
};

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

// Arrow-key teleop wrapper over MotionController::setTeleopCommands.
// `key` expects MuJoCo arrow key constants (`mjKEY_UP/DOWN/LEFT/RIGHT`).
// While pressed, command ramps up over time; releasing drives the corresponding axis to zero.
// Returns true if key was handled.
bool MotionControllerHandleArrowKey(int key, bool pressed);

// Override controller target velocity (m/s) directly.
// When enabled, this takes precedence over forward teleop command each control step.
void MotionControllerSetTargetVelocityMps(double velocity_mps);

// Disable direct target velocity override and resume normal teleop-derived target velocity.
void MotionControllerClearTargetVelocityOverride();

// Read live controller telemetry for HUD display.
bool MotionControllerGetHudTelemetry(MotionControllerHudTelemetry* out);

#endif  // MOTION_CONTROLLER_BRIDGE_H_
