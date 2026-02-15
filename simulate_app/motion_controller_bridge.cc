#include "motion_controller_bridge.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>

#include "MotionController.h"
#include "StateEstimate.h"
#include "StateEstimator.h"
#include "config_control.h"
#include "ekf/BalancerEKF.h"
#include "lqr_lut.h"
#include "lqr_lut_data.h"

namespace {
constexpr float kGravity = 9.81f;

struct ControllerState {
  MotionController controller;
  BalancerEKF ekf;
  StateEstimator state_estimator;
  bool ekf_initialized = false;
  bool state_estimator_initialized = false;
  bool model_ready = false;
  const mjModel* model = nullptr;

  int torso_id = -1;
  int act_wheel_L = -1;
  int act_wheel_R = -1;
  int act_hip_L = -1;
  int act_hip_R = -1;

  int jnt_hip_L = -1;
  int jnt_hip_R = -1;
  int jnt_wheel_L = -1;
  int jnt_wheel_R = -1;

  int sensor_gyro = -1;
  int sensor_acc = -1;

  int geom_wheel_L = -1;
  int site_wL = -1;
  int site_wR = -1;

  // Mesh-derived wheel radius from myRobot/assets/merged/wheel_geom*_collision.stl
  double wheel_radius = 0.08547;
  double wheel_base = 0.31;

  double hip_L_target = -0.1;  // Default to keyframe value
  double hip_R_target = -0.1;  // Default to keyframe value

  double last_time = -1.0;
  double last_lqr_lut_time = -1.0;

  std::atomic<bool> enabled{false};
  std::atomic<bool> user_override{false};
  std::atomic<bool> wheel_control_enabled{true};
  std::atomic<bool> static_mode{false};

  // Saved freejoint state for static mode
  double saved_freejoint_qpos[7] = {0, 0, 0, 1, 0, 0, 0};
  double saved_freejoint_qvel[6] = {0, 0, 0, 0, 0, 0};
  bool freejoint_saved = false;

  double hip_kp = 50.0;
  double hip_kd = 2.0;
  double wheel_torque_scale = 1.0;
  double v_enc_scale = 1.0;
  bool hold_hips = true;
  bool use_ekf = true;
  bool use_state_estimator = true;
  bool estimator_theta_only = false;
  bool estimator_xdot_gt = false;
  bool hip_target_mid = false;  // Use keyframe hip value, not joint range midpoint
  double lqr_lut_period = 0.02;
  bool k0_override_enabled = false;
  float k0_override = 0.0f;
  bool debug_verbose = false;
  bool trace_enabled = false;
  double trace_period = 0.02;
  double last_trace_time = -1.0;
  bool pitch_rate_initialized = false;
  double prev_pitch = 0.0;
  float ekf_theta_r_mult = 1.0f;

  float accel_vib_samples[IMU_VIB_WINDOW_SAMPLES] = {};
  unsigned accel_vib_index = 0;
  unsigned accel_vib_count = 0;
  float accel_vib_sum_sq = 0.0f;
  float accel_vib_rms = 0.0f;
  bool accel_gate = false;
  int accel_grace_steps_cfg = 0;
  int accel_grace_steps_remaining = 0;

  lqr_params_t lqr_params{};
  bool lqr_params_valid = false;
  float lqr_theta_eq = 0.0f;
  float lqr_u_eq = 0.0f;
  bool estimator_warned_invalid = false;

  ControllerState() : controller(RobotParams{}) {}
};

ControllerState g_state;

int find_id(const mjModel* m, int type, const char* name) {
  return mj_name2id(m, type, name);
}

bool env_var_false(const char* name) {
  const char* v = std::getenv(name);
  if (!v) {
    return false;
  }
  return !std::strcmp(v, "0") || !std::strcmp(v, "false") ||
         !std::strcmp(v, "FALSE") || !std::strcmp(v, "off") ||
         !std::strcmp(v, "OFF");
}

bool env_var_true(const char* name) {
  const char* v = std::getenv(name);
  if (!v) {
    return false;
  }
  return !std::strcmp(v, "1") || !std::strcmp(v, "true") ||
         !std::strcmp(v, "TRUE") || !std::strcmp(v, "on") ||
         !std::strcmp(v, "ON") || !std::strcmp(v, "yes") ||
         !std::strcmp(v, "YES");
}

double env_var_double(const char* name, double fallback) {
  const char* v = std::getenv(name);
  if (!v || !*v) {
    return fallback;
  }
  char* end = nullptr;
  const double parsed = std::strtod(v, &end);
  if (end == v) {
    return fallback;
  }
  return parsed;
}

bool env_var_parse_double(const char* name, double* out) {
  if (!out) {
    return false;
  }
  const char* v = std::getenv(name);
  if (!v || !*v) {
    return false;
  }
  char* end = nullptr;
  const double parsed = std::strtod(v, &end);
  if (end == v || (end && *end != '\0')) {
    return false;
  }
  if (!std::isfinite(parsed)) {
    return false;
  }
  *out = parsed;
  return true;
}

int env_var_int(const char* name, int fallback) {
  const char* v = std::getenv(name);
  if (!v || !*v) {
    return fallback;
  }
  char* end = nullptr;
  const long parsed = std::strtol(v, &end, 10);
  if (end == v || (end && *end != '\0')) {
    return fallback;
  }
  if (parsed < 0) {
    return 0;
  }
  return static_cast<int>(parsed);
}

void reset_accel_measurement_gate_state() {
  for (unsigned i = 0; i < IMU_VIB_WINDOW_SAMPLES; ++i) {
    g_state.accel_vib_samples[i] = 0.0f;
  }
  g_state.accel_vib_index = 0;
  g_state.accel_vib_count = 0;
  g_state.accel_vib_sum_sq = 0.0f;
  g_state.accel_vib_rms = 0.0f;
  g_state.accel_gate = false;
  g_state.accel_grace_steps_remaining = g_state.accel_grace_steps_cfg;
}

void update_accel_measurement_gate(float accel_norm_g) {
  if (!std::isfinite(accel_norm_g)) {
    return;
  }
  const float delta = accel_norm_g - 1.0f;
  float old = 0.0f;
  if (g_state.accel_vib_count >= IMU_VIB_WINDOW_SAMPLES) {
    old = g_state.accel_vib_samples[g_state.accel_vib_index];
  } else {
    g_state.accel_vib_count++;
  }
  g_state.accel_vib_samples[g_state.accel_vib_index] = delta;
  g_state.accel_vib_index = (g_state.accel_vib_index + 1U) % IMU_VIB_WINDOW_SAMPLES;
  g_state.accel_vib_sum_sq += (delta * delta) - (old * old);
  if (g_state.accel_vib_sum_sq < 0.0f) {
    g_state.accel_vib_sum_sq = 0.0f;
  }
  if (g_state.accel_vib_count > 0U) {
    g_state.accel_vib_rms =
        std::sqrt(g_state.accel_vib_sum_sq / static_cast<float>(g_state.accel_vib_count));
  } else {
    g_state.accel_vib_rms = 0.0f;
  }

  const bool prev_gate = g_state.accel_gate;
  if (!g_state.accel_gate && g_state.accel_vib_rms > IMU_VIB_ON_G) {
    g_state.accel_gate = true;
  } else if (g_state.accel_gate && g_state.accel_vib_rms < IMU_VIB_OFF_G) {
    g_state.accel_gate = false;
  }

  if (g_state.debug_verbose && prev_gate != g_state.accel_gate) {
    std::printf("MotionController: EKF accel gate %s (vib_rms=%.4f g)\n",
                g_state.accel_gate ? "ON" : "OFF",
                static_cast<double>(g_state.accel_vib_rms));
  }
}

double clamp_ctrl(const mjModel* m, int act_id, double u) {
  if (act_id < 0) {
    return 0.0;
  }
  if (m->actuator_ctrllimited[act_id]) {
    const double lo = m->actuator_ctrlrange[2 * act_id + 0];
    const double hi = m->actuator_ctrlrange[2 * act_id + 1];
    if (u < lo) return lo;
    if (u > hi) return hi;
  }
  return u;
}

bool read_sensor_vec3(const mjModel* m, const mjData* d, int sensor_id, mjtNum out[3]) {
  if (sensor_id < 0) {
    return false;
  }
  int adr = m->sensor_adr[sensor_id];
  int dim = m->sensor_dim[sensor_id];
  if (dim < 3) {
    return false;
  }
  out[0] = d->sensordata[adr + 0];
  out[1] = d->sensordata[adr + 1];
  out[2] = d->sensordata[adr + 2];
  return true;
}

double hip_target_from_range(const mjModel* m, int jnt_id, double fallback) {
  if (jnt_id < 0) {
    return fallback;
  }
  if (m->jnt_limited[jnt_id]) {
    double lo = m->jnt_range[2 * jnt_id + 0];
    double hi = m->jnt_range[2 * jnt_id + 1];
    return 0.5 * (lo + hi);
  }
  return fallback;
}

bool is_servo_actuator(const mjModel* m, int act_id) {
  if (act_id < 0) {
    return false;
  }
  return m->actuator_biastype[act_id] != mjBIAS_NONE;
}

double scaled_position_target(const mjModel* m, int act_id, double joint_target) {
  if (act_id < 0) {
    return joint_target;
  }
  // For hinge position actuators, actuator_length ≈ gear[0] * joint_qpos.
  // Convert desired joint target (rad) to actuator target units.
  const double gear = m->actuator_gear[6 * act_id + 0];
  if (std::abs(gear) < 1e-9) {
    return joint_target;
  }
  return gear * joint_target;
}

int nearest_hip_lut_index(double hip_rad) {
  int best = 0;
  double best_err = std::fabs(hip_rad - static_cast<double>(kHipLut[0]));
  for (int i = 1; i < LQR_LUT_SIZE; ++i) {
    const double err = std::fabs(hip_rad - static_cast<double>(kHipLut[i]));
    if (err < best_err) {
      best = i;
      best_err = err;
    }
  }
  return best;
}

void update_wheel_geometry(const mjModel* m, const mjData* d) {
  if (g_state.geom_wheel_L >= 0) {
    double r = m->geom_size[3 * g_state.geom_wheel_L + 0];
    if (r > 0.0) {
      g_state.wheel_radius = r;
    }
  }

  if (g_state.site_wL >= 0 && g_state.site_wR >= 0) {
    const mjtNum* wL = d->site_xpos + 3 * g_state.site_wL;
    const mjtNum* wR = d->site_xpos + 3 * g_state.site_wR;
    g_state.wheel_base = std::fabs(static_cast<double>(wL[1] - wR[1]));
  }
  if (g_state.wheel_base <= 1e-6) {
    g_state.wheel_base = 0.31;
  }
}

void reset_ekf(const mjModel* m, const mjData* d) {
  if (!g_state.use_ekf || g_state.torso_id < 0) {
    g_state.ekf_initialized = false;
    return;
  }
  if (g_state.sensor_gyro < 0 || g_state.sensor_acc < 0) {
    g_state.ekf_initialized = false;
    std::printf("MotionController: EKF disabled (gyro/acc sensor missing)\n");
    return;
  }

  g_state.ekf.begin();
  reset_accel_measurement_gate_state();
  // Use Euler pitch for theta
  double theta0 = std::asin(2.0 * (d->qpos[3] * d->qpos[5] - d->qpos[6] * d->qpos[4]));
  double x0 = d->qpos[0];
  g_state.ekf.reset(static_cast<float>(theta0), static_cast<float>(x0));
  g_state.ekf_initialized = true;
}

void reset_state_estimator(const mjModel* m, const mjData* d) {
  g_state.state_estimator_initialized = false;
  if (!g_state.use_ekf || !g_state.use_state_estimator || g_state.torso_id < 0) {
    return;
  }
  if (g_state.sensor_gyro < 0 || g_state.sensor_acc < 0) {
    std::printf("MotionController: StateEstimator disabled (gyro/acc sensor missing)\n");
    return;
  }

  RobotParams params{};
  params.wheelRadius = static_cast<float>(g_state.wheel_radius);
  params.wheelBase = static_cast<float>(g_state.wheel_base);
  g_state.state_estimator.begin(params);
  g_state.state_estimator.setControlDt(static_cast<float>(m->opt.timestep));
  g_state.state_estimator.resetTiming();
  g_state.state_estimator_initialized = true;

  // Prime estimator at startup so non-zero keyframe pitch does not produce
  // a large transient torque burst from zero-initialized EKF state.
  if (d) {
    // Prime using static gravity from current pose (not raw accel), so startup
    // is deterministic and independent of any reset/contact transient.
    const mjtNum qw = d->qpos[3];
    const mjtNum qx = d->qpos[4];
    const mjtNum qy = d->qpos[5];
    const mjtNum qz = d->qpos[6];
    const double theta0 = std::asin(2.0 * (qw * qy - qz * qx));
    // Construct a gravity-only accel sample consistent with the bridge tilt
    // extraction convention: theta = atan2(-ax, sqrt(ay^2+az^2)).
    const float g_body_x = static_cast<float>(-kGravity * std::sin(theta0));
    const float g_body_y = 0.0f;
    const float g_body_z = static_cast<float>(-kGravity * std::cos(theta0));

    const double dt_ms_real = 1000.0 * m->opt.timestep;
    uint32_t dt_ms = static_cast<uint32_t>(std::llround(dt_ms_real));
    if (dt_ms == 0U) {
      dt_ms = 1U;
    }
    uint32_t now_ms = static_cast<uint32_t>(std::llround(d->time * 1000.0));
    ImuReading primary{};
    primary.valid = true;
    primary.gyro_x = 0.0f;
    primary.gyro_y = 0.0f;
    primary.gyro_z = 0.0f;
    primary.accel_x = g_body_x;
    primary.accel_y = g_body_y;
    primary.accel_z = g_body_z;
    ImuReading secondary{};
    secondary.valid = false;
    for (int i = 0; i < 80; ++i) {
      now_ms += dt_ms;
      primary.timestamp_ms = now_ms;
      secondary.timestamp_ms = now_ms;
      g_state.state_estimator.update(primary, secondary, now_ms, 0.0f, 0.0f);
    }
    if (g_state.debug_verbose || g_state.trace_enabled) {
      const StateEstimate est0 = g_state.state_estimator.getEstimate();
      std::printf("MotionController: StateEstimator prime theta0=%.5f theta_est=%.5f valid=%d\n",
                  theta0, static_cast<double>(est0.theta), est0.valid ? 1 : 0);
    }
    g_state.state_estimator.resetTiming();
  }
}

void compute_kinematics(const mjModel* m, const mjData* d,
                        double* theta, double* theta_dot,
                        double* x, double* x_dot_world) {
  if (g_state.torso_id < 0) {
    *theta = 0.0;
    *theta_dot = 0.0;
    *x = 0.0;
    *x_dot_world = 0.0;
    return;
  }

  // Use proper Euler pitch angle: pitch = asin(2*(qw*qy - qz*qx))
  // This was empirically verified to produce correct sign for balance control.
  // Pitch rate is computed via finite difference with reset-safe history.
  const mjtNum qw = d->qpos[3];
  const mjtNum qx = d->qpos[4];
  const mjtNum qy = d->qpos[5];
  const mjtNum qz = d->qpos[6];
  double pitch = std::asin(2.0 * (qw * qy - qz * qx));
  
  // Finite-difference pitch rate (more reliable than qvel[1]) with
  // initialization guard to avoid spikes after reset/rewind.
  if (g_state.pitch_rate_initialized && m->opt.timestep > 0.0) {
    *theta_dot = (pitch - g_state.prev_pitch) / m->opt.timestep;
  } else {
    *theta_dot = d->qvel[4];
    g_state.pitch_rate_initialized = true;
  }
  g_state.prev_pitch = pitch;
  
  *theta = pitch;
  *x = d->qpos[0];
  *x_dot_world = d->qvel[0];  /* vx (linear), NOT qvel[3] which is wx (angular) */
}

void apply_wheel_control(const mjModel* m, mjData* d, const MotionController::Command& cmd) {
  // Check if wheel control is enabled
  if (!g_state.wheel_control_enabled.load()) {
    // Stop wheels by setting torque to zero
    if (g_state.act_wheel_L >= 0) {
      d->ctrl[g_state.act_wheel_L] = 0.0;
    }
    if (g_state.act_wheel_R >= 0) {
      d->ctrl[g_state.act_wheel_R] = 0.0;
    }
    return;
  }

  // The current reduced model and LUT assume same-sign wheel controls
  // for left/right wheel actuators.
  if (g_state.act_wheel_L >= 0) {
    double torque_nm = static_cast<double>(cmd.torque.torqueLeftNm) *
                       g_state.wheel_torque_scale;
    d->ctrl[g_state.act_wheel_L] = clamp_ctrl(m, g_state.act_wheel_L, torque_nm);
  }
  if (g_state.act_wheel_R >= 0) {
    double torque_nm = static_cast<double>(cmd.torque.torqueRightNm) *
                       g_state.wheel_torque_scale;
    // Same ctrl to both wheels.
    d->ctrl[g_state.act_wheel_R] = clamp_ctrl(m, g_state.act_wheel_R, torque_nm);
  }
}

void apply_hip_hold(const mjModel* m, mjData* d) {
  if (!g_state.hold_hips || g_state.act_hip_L < 0 || g_state.act_hip_R < 0 ||
      g_state.jnt_hip_L < 0 || g_state.jnt_hip_R < 0) {
    return;
  }

  if (is_servo_actuator(m, g_state.act_hip_L) &&
      is_servo_actuator(m, g_state.act_hip_R)) {
    const double hip_L_ctrl = scaled_position_target(m, g_state.act_hip_L, g_state.hip_L_target);
    const double hip_R_ctrl = scaled_position_target(m, g_state.act_hip_R, g_state.hip_R_target);
    d->ctrl[g_state.act_hip_L] = clamp_ctrl(m, g_state.act_hip_L, hip_L_ctrl);
    d->ctrl[g_state.act_hip_R] = clamp_ctrl(m, g_state.act_hip_R, hip_R_ctrl);
    return;
  }

  const int qpos_L = m->jnt_qposadr[g_state.jnt_hip_L];
  const int qvel_L = m->jnt_dofadr[g_state.jnt_hip_L];
  const int qpos_R = m->jnt_qposadr[g_state.jnt_hip_R];
  const int qvel_R = m->jnt_dofadr[g_state.jnt_hip_R];

  double err_L = g_state.hip_L_target - d->qpos[qpos_L];
  double err_R = g_state.hip_R_target - d->qpos[qpos_R];
  double derr_L = -d->qvel[qvel_L];
  double derr_R = -d->qvel[qvel_R];

  double u_L = g_state.hip_kp * err_L + g_state.hip_kd * derr_L;
  double u_R = g_state.hip_kp * err_R + g_state.hip_kd * derr_R;

  d->ctrl[g_state.act_hip_L] = clamp_ctrl(m, g_state.act_hip_L, u_L);
  d->ctrl[g_state.act_hip_R] = clamp_ctrl(m, g_state.act_hip_R, u_R);
}

void init_lqr_params_defaults(lqr_params_t* out) {
  if (!out) {
    return;
  }
  out->K[0] = LQR_K0_X;
  out->K[1] = LQR_K1_V;
  out->K[2] = LQR_K2_THETA;
  out->K[3] = LQR_K3_THETADOT;
  out->u_limit = LQR_U_LIMIT;
  out->du_limit = 0.0f;  // Disable rate limiting in simulation
  out->theta_ref_limit = 0.20f;  // 0.20 rad ≈ 11.5° — allow full theta_eq range from LUT
  out->v_ref_limit = LQR_V_REF_LIMIT;
  out->engage_ramp_ms = 0;   // Skip PID→LQR ramp in simulation
  out->disengage_ramp_ms = 0;
  out->default_mode = 1;     // Start directly in LQR mode
}

void update_lqr_from_hip(const mjModel* m, mjData* d, bool force) {
  if (!m || !d || g_state.jnt_hip_L < 0 || g_state.jnt_hip_R < 0) {
    return;
  }
  if (!force && g_state.last_lqr_lut_time >= 0.0 &&
      (d->time - g_state.last_lqr_lut_time) < g_state.lqr_lut_period) {
    return;
  }

  const int qpos_L = m->jnt_qposadr[g_state.jnt_hip_L];
  const int qpos_R = m->jnt_qposadr[g_state.jnt_hip_R];
  const float hip_L = static_cast<float>(d->qpos[qpos_L]);
  const float hip_R = static_cast<float>(d->qpos[qpos_R]);
  const float hip_avg = 0.5f * (hip_L + hip_R);

  float K[4] = {0};
  float theta_eq = 0.0f;
  float u_eq = 0.0f;
  if (!lqr_lut_eval_full(hip_avg, K, &theta_eq, &u_eq)) {
    return;
  }

  if (g_state.k0_override_enabled) {
    K[0] = g_state.k0_override;
  }

  if (!g_state.lqr_params_valid) {
    init_lqr_params_defaults(&g_state.lqr_params);
    g_state.lqr_params_valid = true;
  }
  /* LUT gains regenerated from sysid 2026-02-11 */
  for (int i = 0; i < 4; ++i) {
    g_state.lqr_params.K[i] = K[i];
  }
  g_state.controller.setLqrParams(g_state.lqr_params);
  g_state.controller.setLqrEquilibrium(theta_eq, u_eq);
  g_state.lqr_theta_eq = theta_eq;
  g_state.lqr_u_eq = u_eq;
  g_state.last_lqr_lut_time = d->time;
}

}  // namespace

void MotionControllerReset(const mjModel* m, mjData* d) {
  if (!m || !d) {
    return;
  }

  g_state.model = m;
  g_state.model_ready = false;
  g_state.last_time = d->time;
  g_state.use_ekf = !env_var_false("SIM_USE_EKF");
  g_state.use_state_estimator = !env_var_false("SIM_USE_STATE_ESTIMATOR");
  g_state.estimator_theta_only = env_var_true("SIM_EST_THETA_ONLY");
  g_state.estimator_xdot_gt = env_var_true("SIM_EST_XDOT_GT");
  g_state.v_enc_scale = env_var_double("SIM_VENC_SCALE", 1.0);
  g_state.debug_verbose = env_var_true("SIM_MC_DEBUG");
  g_state.trace_enabled = env_var_true("SIM_MC_TRACE");
  g_state.trace_period = env_var_double("SIM_MC_TRACE_DT", 0.02);
  if (!std::isfinite(g_state.trace_period) || g_state.trace_period <= 0.0) {
    g_state.trace_period = 0.02;
  }
  g_state.last_trace_time = -1.0;
  g_state.pitch_rate_initialized = false;
  g_state.prev_pitch = 0.0;
  g_state.estimator_warned_invalid = false;
  g_state.ekf_theta_r_mult = static_cast<float>(
      env_var_double("SIM_EKF_THETA_R_MULT", 1.0));
  if (!std::isfinite(g_state.ekf_theta_r_mult) || g_state.ekf_theta_r_mult <= 0.0f) {
    g_state.ekf_theta_r_mult = 1.0f;
  }
  g_state.k0_override_enabled = false;
  g_state.k0_override = 0.0f;
  {
    int grace_default = 0;
    if (m->opt.timestep > 0.0) {
      grace_default =
          static_cast<int>(std::ceil((0.001 * static_cast<double>(EKF_GRACE_MS)) /
                                     m->opt.timestep));
    }
    g_state.accel_grace_steps_cfg = env_var_int("SIM_EKF_ACCEL_GRACE_STEPS", grace_default);
    if (g_state.accel_grace_steps_cfg < 0) {
      g_state.accel_grace_steps_cfg = 0;
    }
  }
  reset_accel_measurement_gate_state();
  {
    double parsed = 0.0;
    const char* raw = std::getenv("SIM_K0_OVERRIDE");
    if (raw && *raw) {
      if (env_var_parse_double("SIM_K0_OVERRIDE", &parsed)) {
        g_state.k0_override_enabled = true;
        g_state.k0_override = static_cast<float>(parsed);
      } else {
        std::printf("MotionController: ignoring invalid SIM_K0_OVERRIDE='%s'\n", raw);
      }
    }
  }

  g_state.torso_id = find_id(m, mjOBJ_BODY, "torso");
  g_state.act_wheel_L = find_id(m, mjOBJ_ACTUATOR, "wheel_L");
  g_state.act_wheel_R = find_id(m, mjOBJ_ACTUATOR, "wheel_R");
  g_state.act_hip_L = find_id(m, mjOBJ_ACTUATOR, "hip_L");
  g_state.act_hip_R = find_id(m, mjOBJ_ACTUATOR, "hip_R");

  g_state.jnt_hip_L = find_id(m, mjOBJ_JOINT, "hip_L");
  g_state.jnt_hip_R = find_id(m, mjOBJ_JOINT, "hip_R");
  g_state.jnt_wheel_L = find_id(m, mjOBJ_JOINT, "wheel_L");
  g_state.jnt_wheel_R = find_id(m, mjOBJ_JOINT, "wheel_R");

  g_state.sensor_gyro = find_id(m, mjOBJ_SENSOR, "gyro");
  g_state.sensor_acc = find_id(m, mjOBJ_SENSOR, "acc");

  g_state.geom_wheel_L = find_id(m, mjOBJ_GEOM, "wheel_geom_L");
  g_state.site_wL = find_id(m, mjOBJ_SITE, "W_L");
  g_state.site_wR = find_id(m, mjOBJ_SITE, "W_R");

  const bool ready = (g_state.torso_id >= 0 && g_state.act_wheel_L >= 0 &&
                      g_state.act_wheel_R >= 0 && g_state.jnt_wheel_L >= 0 &&
                      g_state.jnt_wheel_R >= 0);
  g_state.model_ready = ready;

  if (!g_state.model_ready) {
    std::printf("MotionController: model not supported (missing joints/actuators)\n");
    if (!g_state.user_override.load()) {
      g_state.enabled.store(false);
    }
    return;
  }

  // Ensure kinematics/sensors are current for this reset state (keyframe/load).
  mj_forward(m, d);

  update_wheel_geometry(m, d);

  if (g_state.hold_hips && g_state.jnt_hip_L >= 0 && g_state.jnt_hip_R >= 0) {
    int qpos_L_idx = m->jnt_qposadr[g_state.jnt_hip_L];
    int qpos_R_idx = m->jnt_qposadr[g_state.jnt_hip_R];
    double hip_L_key = d->qpos[qpos_L_idx];
    double hip_R_key = d->qpos[qpos_R_idx];
    std::printf("MotionController: Reading keyframe qpos[%d]=%.4f (hip_L), qpos[%d]=%.4f (hip_R)\n",
                qpos_L_idx, hip_L_key, qpos_R_idx, hip_R_key);
    if (g_state.hip_target_mid) {
      g_state.hip_L_target = hip_target_from_range(m, g_state.jnt_hip_L, hip_L_key);
      g_state.hip_R_target = hip_target_from_range(m, g_state.jnt_hip_R, hip_R_key);
    } else {
      g_state.hip_L_target = hip_L_key;
      g_state.hip_R_target = hip_R_key;
    }
    std::printf("MotionController: Hip targets set to L=%.4f, R=%.4f rad\n",
                g_state.hip_L_target, g_state.hip_R_target);
  }

  g_state.controller.setControlDt(static_cast<float>(m->opt.timestep));
  balance_gains_t gains{};
  gains.Kp_theta = BALANCE_DEFAULT_KP_THETA;
  gains.Kd_theta = BALANCE_DEFAULT_KD_THETA;
  gains.Kp_v_to_theta = BALANCE_DEFAULT_KP_V_TO_THETA;
  gains.Ki_v_to_theta = BALANCE_DEFAULT_KI_V_TO_THETA;
  gains.max_tilt_ref = BALANCE_DEFAULT_MAX_TILT_REF;
  gains.Kv_damp = BALANCE_DEFAULT_KV_DAMP;
  gains.K_turn = BALANCE_DEFAULT_K_TURN;
  gains.K_yawDamp = BALANCE_DEFAULT_K_YAW_DAMP;
  gains.alpha_yaw = BALANCE_DEFAULT_ALPHA_YAW;
  gains.IqMax = BALANCE_DEFAULT_IQ_MAX;
  gains.thetaKill = BALANCE_DEFAULT_THETA_KILL;
  gains.iV_max = BALANCE_DEFAULT_IV_MAX;
  g_state.controller.setBalanceGains(gains);
  g_state.controller.setTargetVelocity(0.0f);

  // Initialize LQR params BEFORE requesting mode change
  init_lqr_params_defaults(&g_state.lqr_params);
  g_state.lqr_params_valid = true;
  update_lqr_from_hip(m, d, true);

  // Now request LQR mode (will use engage_ramp_ms=0 from params)
  g_state.controller.setRequestedMode(InnerLongMode::LQR);

  if (g_state.use_state_estimator) {
    reset_state_estimator(m, d);
    g_state.ekf_initialized = false;
  } else {
    reset_ekf(m, d);
    g_state.state_estimator_initialized = false;
  }
  if (!g_state.use_ekf) {
    std::printf("MotionController: EKF disabled (SIM_USE_EKF=0)\n");
  }
  if (g_state.use_ekf) {
    std::printf("MotionController: estimator backend = %s\n",
                g_state.use_state_estimator ? "StateEstimator" : "direct-EKF");
    if (g_state.estimator_theta_only) {
      std::printf("MotionController: SIM_EST_THETA_ONLY=1 (x/xdot from ground truth)\n");
    } else if (g_state.estimator_xdot_gt) {
      std::printf("MotionController: SIM_EST_XDOT_GT=1 (xDot from ground truth)\n");
    }
  }
  std::printf("MotionController: encoder velocity scale = %.3f\n", g_state.v_enc_scale);
  if (g_state.k0_override_enabled) {
    std::printf("MotionController: overriding LUT K0 with %.6f (SIM_K0_OVERRIDE)\n",
                static_cast<double>(g_state.k0_override));
  }
  if (g_state.use_ekf && !g_state.use_state_estimator) {
    std::printf("MotionController: EKF accel grace = %d steps (%.1f ms)\n",
                g_state.accel_grace_steps_cfg,
                1000.0 * static_cast<double>(g_state.accel_grace_steps_cfg) *
                    static_cast<double>(m->opt.timestep));
    std::printf("MotionController: EKF theta R multiplier = %.3f (SIM_EKF_THETA_R_MULT)\n",
                static_cast<double>(g_state.ekf_theta_r_mult));
  } else if (g_state.use_ekf && g_state.use_state_estimator && !g_state.state_estimator_initialized) {
    std::printf("MotionController: StateEstimator unavailable, falling back to ground truth\n");
  }

  if (!g_state.user_override.load()) {
    g_state.enabled.store(true);
  }
  std::printf("MotionController: %s\n", g_state.enabled.load() ? "enabled" : "disabled");
}

void MotionControllerCallback(const mjModel* m, mjData* d) {
  if (!m || !d) {
    return;
  }
  if (!g_state.enabled.load()) {
    return;
  }
  if (!g_state.model_ready || g_state.model != m) {
    return;
  }

  if (g_state.last_time > 0.0 && d->time + 1e-9 < g_state.last_time) {
    MotionControllerReset(m, d);
    return;
  }
  g_state.last_time = d->time;

  // Optional diagnostic info every 0.1s (before static mode check).
  static double last_print_time = -1.0;
  if (g_state.debug_verbose && (d->time - last_print_time > 0.1)) {
    last_print_time = d->time;

    // Get average hip joint angle directly from qpos.
    double hip_joint = NAN;
    if (g_state.jnt_hip_L >= 0 && g_state.jnt_hip_R >= 0) {
      const int qpos_L = m->jnt_qposadr[g_state.jnt_hip_L];
      const int qpos_R = m->jnt_qposadr[g_state.jnt_hip_R];
      hip_joint = 0.5 * (d->qpos[qpos_L] + d->qpos[qpos_R]);
    }

    // Get hip joint position
    double hip_pos[3] = {0, 0, 0};
    if (g_state.jnt_hip_L >= 0) {
      hip_pos[0] = d->xanchor[3 * g_state.jnt_hip_L + 0];
      hip_pos[1] = d->xanchor[3 * g_state.jnt_hip_L + 1];
      hip_pos[2] = d->xanchor[3 * g_state.jnt_hip_L + 2];
    }

    // Get wheel center position (body, not geom)
    int body_wheel_L = mj_name2id(m, mjOBJ_BODY, "wheel_geom");
    double wheel_pos[3] = {0, 0, 0};
    if (body_wheel_L >= 0) {
      wheel_pos[0] = d->xpos[3 * body_wheel_L + 0];
      wheel_pos[1] = d->xpos[3 * body_wheel_L + 1];
      wheel_pos[2] = d->xpos[3 * body_wheel_L + 2];
    }

    // Compute distance
    double dx = wheel_pos[0] - hip_pos[0];
    double dy = wheel_pos[1] - hip_pos[1];
    double dz = wheel_pos[2] - hip_pos[2];
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Compute upper body COM (excluding wheels)
    double com[3] = {0, 0, 0};
    double total_mass = 0.0;
    int body_wheel_R = mj_name2id(m, mjOBJ_BODY, "wheel_geom_2");
    for (int b = 0; b < m->nbody; b++) {
      if (b == body_wheel_L || b == body_wheel_R) continue;
      double body_mass = m->body_mass[b];
      if (body_mass > 0) {
        com[0] += d->xipos[3*b + 0] * body_mass;
        com[1] += d->xipos[3*b + 1] * body_mass;
        com[2] += d->xipos[3*b + 2] * body_mass;
        total_mass += body_mass;
      }
    }
    if (total_mass > 1e-9) {
      com[0] /= total_mass;
      com[1] /= total_mass;
      com[2] /= total_mass;
    } else {
      com[0] = wheel_pos[0];
      com[1] = wheel_pos[1];
      com[2] = wheel_pos[2];
    }

    // COM relative to wheel contact
    double wheel_contact_z = wheel_pos[2] - g_state.wheel_radius;
    double com_rel_x = com[0] - wheel_pos[0];
    double com_rel_z = com[2] - wheel_contact_z;
    double theta_eq = -std::atan2(com_rel_x, com_rel_z);

    printf("hip=%.4f  ||hip-wheel||=%.4f  COM_rel=(%.3f,%.3f)  θ_eq=%6.2f°%s\n",
           hip_joint, distance, com_rel_x, com_rel_z, theta_eq * 57.3,
           g_state.static_mode.load() ? "  [STATIC]" : "");
  }

  // Static measurement mode: fix freejoint to prevent robot motion
  if (g_state.static_mode.load()) {
    // Save freejoint state on first entry to static mode
    if (!g_state.freejoint_saved) {
      for (int i = 0; i < 7; i++) {
        g_state.saved_freejoint_qpos[i] = d->qpos[i];
      }
      for (int i = 0; i < 6; i++) {
        g_state.saved_freejoint_qvel[i] = d->qvel[i];
      }
      g_state.freejoint_saved = true;
    }

    // Restore freejoint state (fixes robot position in space)
    for (int i = 0; i < 7; i++) {
      d->qpos[i] = g_state.saved_freejoint_qpos[i];
    }
    for (int i = 0; i < 6; i++) {
      d->qvel[i] = 0.0;  // Zero all base velocities
    }

    // Zero wheel torques
    if (g_state.act_wheel_L >= 0) {
      d->ctrl[g_state.act_wheel_L] = 0.0;
    }
    if (g_state.act_wheel_R >= 0) {
      d->ctrl[g_state.act_wheel_R] = 0.0;
    }

    // Hip control is still active, so user can move sliders
    apply_hip_hold(m, d);
    return;  // Skip normal control logic
  }

  double theta = 0.0;
  double theta_dot = 0.0;
  double x = 0.0;
  double x_dot_world = 0.0;
  compute_kinematics(m, d, &theta, &theta_dot, &x, &x_dot_world);
  update_lqr_from_hip(m, d, false);

  StateEstimate est{};
  est.gyroBias = 0.0f;
  est.valid = true;

  mjtNum gyro_body[3] = {0, 0, 0};
  mjtNum accel_body[3] = {0, 0, 0};
  bool have_gyro = read_sensor_vec3(m, d, g_state.sensor_gyro, gyro_body);
  bool have_acc = read_sensor_vec3(m, d, g_state.sensor_acc, accel_body);
  float gyro_pitch = have_gyro ? static_cast<float>(gyro_body[1]) : 0.0f;
  float gyro_yaw = have_gyro ? static_cast<float>(gyro_body[2]) : 0.0f;

  double omega_L = 0.0;
  double omega_R = 0.0;
  if (g_state.jnt_wheel_L >= 0) {
    omega_L = d->qvel[m->jnt_dofadr[g_state.jnt_wheel_L]];
  }
  if (g_state.jnt_wheel_R >= 0) {
    omega_R = d->qvel[m->jnt_dofadr[g_state.jnt_wheel_R]];
  }

  float yaw_rate_enc = 0.0f;
  if (g_state.wheel_base > 1e-6) {
    yaw_rate_enc = static_cast<float>(g_state.wheel_radius * (omega_R - omega_L) /
                                      g_state.wheel_base);
  }
  // Wheel joint velocities are opposite sign to world forward x in this model.
  const float v_enc = static_cast<float>(
      -g_state.v_enc_scale * g_state.wheel_radius * 0.5 * (omega_L + omega_R));
  // Firmware StateEstimator convention is opposite to legacy bridge convention.
  const float v_enc_state_estimator = -v_enc;

  bool used_state_estimator = false;
  if (g_state.state_estimator_initialized) {
    const uint32_t now_ms = static_cast<uint32_t>(std::llround(d->time * 1000.0));
    ImuReading primary{};
    primary.timestamp_ms = now_ms;
    primary.valid = have_gyro && have_acc;
    if (primary.valid) {
      primary.gyro_x = static_cast<float>(gyro_body[0]);
      primary.gyro_y = static_cast<float>(gyro_body[1]);
      primary.gyro_z = static_cast<float>(gyro_body[2]);
      primary.accel_x = static_cast<float>(accel_body[0]);
      primary.accel_y = static_cast<float>(accel_body[1]);
      primary.accel_z = static_cast<float>(accel_body[2]);
    }
    ImuReading secondary{};
    secondary.timestamp_ms = now_ms;
    secondary.valid = false;

    g_state.state_estimator.update(primary, secondary, now_ms, v_enc_state_estimator, yaw_rate_enc);
    StateEstimate estimator_est = g_state.state_estimator.getEstimate();
    g_state.controller.setYawRates(g_state.state_estimator.getEstimatedYawRate(), yaw_rate_enc);
    est = estimator_est;
    used_state_estimator = true;
    if (!estimator_est.valid && !g_state.estimator_warned_invalid) {
      std::printf(
          "MotionController: StateEstimator invalid "
          "(have_gyro=%d have_acc=%d v_enc=%.4f v_enc_est=%.4f yaw_rate_enc=%.4f)\n",
          have_gyro ? 1 : 0, have_acc ? 1 : 0,
          static_cast<double>(v_enc),
          static_cast<double>(v_enc_state_estimator),
          static_cast<double>(yaw_rate_enc));
      if (have_gyro) {
        std::printf("  gyro_body=[%.4f %.4f %.4f] rad/s\n",
                    static_cast<double>(gyro_body[0]),
                    static_cast<double>(gyro_body[1]),
                    static_cast<double>(gyro_body[2]));
      }
      if (have_acc) {
        std::printf("  acc_body=[%.4f %.4f %.4f] m/s^2\n",
                    static_cast<double>(accel_body[0]),
                    static_cast<double>(accel_body[1]),
                    static_cast<double>(accel_body[2]));
      }
      EkfLogData ekf_log{};
      if (g_state.state_estimator.getEkfLogData(ekf_log)) {
        std::printf(
            "  ekf_log: valid=%d dt=%.4f theta=%.4f theta_acc=%.4f "
            "gyro_y=%.4f accel_norm_g=%.4f gate=%u "
            "innov=%.4f s=%.4f k0=%.4f k1=%.4f r_used=%.4f\n",
            ekf_log.valid ? 1 : 0, static_cast<double>(ekf_log.dt_s),
            static_cast<double>(ekf_log.theta),
            static_cast<double>(ekf_log.theta_acc),
            static_cast<double>(ekf_log.gyro_y),
            static_cast<double>(ekf_log.accel_norm_g),
            static_cast<unsigned>(ekf_log.gate),
            static_cast<double>(ekf_log.innov),
            static_cast<double>(ekf_log.s),
            static_cast<double>(ekf_log.k0),
            static_cast<double>(ekf_log.k1),
            static_cast<double>(ekf_log.r_used));
      } else {
        std::printf("  ekf_log: unavailable\n");
      }
      ImuHealthMetrics imu_health{};
      if (g_state.state_estimator.getImuHealthMetrics(imu_health)) {
        std::printf(
            "  imu_health: valid=%d active_sensor=%u vib_rms_g=%.4f gate_accel=%u "
            "gyro_diff_dps=%.3f gyro_pitch_diff_dps=%.3f acc_angle_diff_deg=%.3f\n",
            imu_health.valid ? 1 : 0, static_cast<unsigned>(imu_health.active_sensor),
            static_cast<double>(imu_health.vib_rms_g),
            static_cast<unsigned>(imu_health.gate_accel),
            static_cast<double>(imu_health.gyro_diff_dps),
            static_cast<double>(imu_health.gyro_pitch_diff_dps),
            static_cast<double>(imu_health.acc_angle_diff_deg));
      } else {
        std::printf("  imu_health: unavailable\n");
      }
      g_state.estimator_warned_invalid = true;
    } else if (estimator_est.valid && g_state.estimator_warned_invalid) {
      std::printf("MotionController: StateEstimator recovered\n");
      g_state.estimator_warned_invalid = false;
    }
  }

  if (!used_state_estimator && g_state.ekf_initialized) {
    g_state.controller.setYawRates(gyro_yaw, yaw_rate_enc);
    float theta_acc = NAN;
    float theta_var = g_state.ekf.getThetaMeasurementVarianceBase();
    theta_var *= g_state.ekf_theta_r_mult;
    if (have_acc) {
      const float ax = static_cast<float>(accel_body[0]);
      const float ay = static_cast<float>(accel_body[1]);
      const float az = static_cast<float>(accel_body[2]);
      const float accel_norm_g = std::sqrt(ax * ax + ay * ay + az * az) / kGravity;
      update_accel_measurement_gate(accel_norm_g);
      if (g_state.accel_grace_steps_remaining <= 0) {
        const float accel_yz = std::sqrt(ay * ay + az * az);
        if (accel_yz > 1e-6f) {
          theta_acc = std::atan2(-ax, accel_yz);
        }
      }
    }
    if (g_state.accel_grace_steps_remaining > 0) {
      g_state.accel_grace_steps_remaining--;
    }
    if (g_state.accel_gate) {
      theta_var *= EKF_TUNE_R_MULT;
    }

    float pos_enc = NAN;
    float dt = static_cast<float>(m->opt.timestep);
    bool ok = g_state.ekf.step(theta_acc, v_enc, pos_enc,
                               gyro_pitch, gyro_yaw, yaw_rate_enc,
                               dt, theta_var);
    BalancerState st = g_state.ekf.getState();
    est.theta = st.theta;
    est.thetaDot = st.thetaDot;
    est.x = st.x;
    est.xDot = st.xDot;
    est.gyroBias = st.gyroBias;
    est.valid = ok && g_state.ekf.isStateValid();
  }

  if (!used_state_estimator && !g_state.ekf_initialized) {
    g_state.controller.setYawRates(gyro_yaw, yaw_rate_enc);
    est.theta = static_cast<float>(theta);
    est.thetaDot = static_cast<float>(theta_dot);
    est.x = static_cast<float>(x);
    est.xDot = static_cast<float>(x_dot_world);
  }

  if ((used_state_estimator || g_state.ekf_initialized) &&
      g_state.estimator_theta_only) {
    // Debug mode: isolate tilt estimation by using MuJoCo ground-truth
    // longitudinal states for control.
    est.x = static_cast<float>(x);
    est.xDot = static_cast<float>(x_dot_world);
  } else if ((used_state_estimator || g_state.ekf_initialized) &&
             g_state.estimator_xdot_gt) {
    // Debug mode: keep estimator position x, but force ground-truth xDot.
    est.xDot = static_cast<float>(x_dot_world);
  } else {
    // Match firmware control path: longitudinal velocity term uses encoder
    // velocity directly at control time, not EKF state xDot.
    if (used_state_estimator) {
      est.xDot = v_enc_state_estimator;
    } else if (g_state.ekf_initialized) {
      est.xDot = v_enc;
    }
  }

  const float dt = static_cast<float>(m->opt.timestep);
  MotionController::Command cmd = g_state.controller.computeControl(est, dt);

  if (g_state.trace_enabled &&
      (g_state.last_trace_time < 0.0 ||
       (d->time - g_state.last_trace_time) >= g_state.trace_period)) {
    const char* backend = "GT";
    if (used_state_estimator) {
      backend = "SE";
    } else if (g_state.ekf_initialized) {
      backend = "EKF";
    }
    std::printf(
        "MC_TRACE t=%.3f backend=%s valid=%d "
        "theta_gt=%.5f theta_est=%.5f thetaDot_gt=%.5f thetaDot_est=%.5f "
        "x_gt=%.5f x_est=%.5f xdot_gt=%.5f v_enc=%.5f v_enc_est=%.5f xdot_est=%.5f "
        "uL=%.5f uR=%.5f\n",
        d->time, backend, est.valid ? 1 : 0,
        theta, static_cast<double>(est.theta),
        theta_dot, static_cast<double>(est.thetaDot),
        x, static_cast<double>(est.x),
        x_dot_world, static_cast<double>(v_enc),
        static_cast<double>(v_enc_state_estimator),
        static_cast<double>(est.xDot),
        static_cast<double>(cmd.torque.torqueLeftNm),
        static_cast<double>(cmd.torque.torqueRightNm));
    g_state.last_trace_time = d->time;
  }

  apply_wheel_control(m, d, cmd);
  apply_hip_hold(m, d);
}

bool MotionControllerToggle() {
  const bool new_state = !g_state.enabled.load();
  g_state.enabled.store(new_state);
  g_state.user_override.store(true);
  std::printf("MotionController: %s\n", new_state ? "enabled" : "disabled");
  return new_state;
}

bool MotionControllerIsEnabled() {
  return g_state.enabled.load();
}

bool WheelControlToggle() {
  const bool new_state = !g_state.wheel_control_enabled.load();
  g_state.wheel_control_enabled.store(new_state);
  std::printf("Wheel control: %s\n", new_state ? "enabled" : "disabled");
  return new_state;
}

bool StaticModeToggle() {
  const bool new_state = !g_state.static_mode.load();
  g_state.static_mode.store(new_state);
  g_state.freejoint_saved = false;  // Reset saved state when toggling
  std::printf("Static measurement mode: %s\n", new_state ? "ENABLED (robot fixed)" : "DISABLED");
  return new_state;
}

bool MotionControllerStepHipTarget(int direction) {
  if (direction == 0 || !g_state.hold_hips || g_state.jnt_hip_L < 0 ||
      g_state.jnt_hip_R < 0) {
    return false;
  }

  const int step = (direction > 0) ? 1 : -1;
  const double hip_target = 0.5 * (g_state.hip_L_target + g_state.hip_R_target);
  const int index = nearest_hip_lut_index(hip_target);
  int next = index + step;
  if (next < 0) {
    next = 0;
  }
  if (next >= LQR_LUT_SIZE) {
    next = LQR_LUT_SIZE - 1;
  }
  if (next == index) {
    return false;
  }

  const double next_target = static_cast<double>(kHipLut[next]);
  g_state.hip_L_target = next_target;
  g_state.hip_R_target = next_target;
  std::printf("MotionController: hip target -> LUT %d/%d (%.4f rad)\n",
              next + 1, LQR_LUT_SIZE, next_target);
  return true;
}
