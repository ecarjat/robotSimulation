#include "motion_controller_bridge.h"

#include <atomic>
#include <cmath>
#include <cstdio>

#include "MotionController.h"
#include "StateEstimate.h"
#include "ekf/BalancerEKF.h"
#include "lqr_lut.h"

namespace {
struct ControllerState {
  MotionController controller;
  BalancerEKF ekf;
  bool ekf_initialized = false;
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

  double hip_L_target = 0.0;
  double hip_R_target = 0.0;

  double last_time = -1.0;
  double last_lqr_lut_time = -1.0;

  std::atomic<bool> enabled{false};
  std::atomic<bool> user_override{false};

  double hip_kp = 50.0;
  double hip_kd = 2.0;
  double wheel_torque_scale = 1.0;
  bool hold_hips = true;
  bool use_ekf = true;
  bool hip_target_mid = true;
  double lqr_lut_period = 0.02;

  lqr_params_t lqr_params{};
  bool lqr_params_valid = false;
  float lqr_theta_eq = 0.0f;
  float lqr_u_eq = 0.0f;

  ControllerState() : controller(RobotParams{}) {}
};

ControllerState g_state;

int find_id(const mjModel* m, int type, const char* name) {
  return mj_name2id(m, type, name);
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
  // Use Euler pitch for theta
  double theta0 = std::asin(2.0 * (d->qpos[3] * d->qpos[5] - d->qpos[6] * d->qpos[4]));
  double x0 = d->qpos[0];
  g_state.ekf.reset(static_cast<float>(theta0), static_cast<float>(x0));
  g_state.ekf_initialized = true;
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
  // Pitch rate is computed via finite difference (stored in bridge state).
  const mjtNum qw = d->qpos[3];
  const mjtNum qx = d->qpos[4];
  const mjtNum qy = d->qpos[5];
  const mjtNum qz = d->qpos[6];
  double pitch = std::asin(2.0 * (qw * qy - qz * qx));
  
  // Finite-difference pitch rate (more reliable than qvel[1])
  static double prev_pitch = pitch;
  double dt = m->opt.timestep;
  *theta_dot = (pitch - prev_pitch) / dt;
  prev_pitch = pitch;
  
  *theta = pitch;
  *x = d->qpos[0];
  *x_dot_world = d->qvel[3];
}

void apply_wheel_control(const mjModel* m, mjData* d, const MotionController::Command& cmd) {
  // The LQR gains are computed assuming both wheels have the same effect direction.
  // But the myRobot model has opposite wheel body orientations, so their joint axes
  // point in different directions in world frame. We negate wheel_R to compensate.
  if (g_state.act_wheel_L >= 0) {
    double torque_nm = static_cast<double>(cmd.torque.torqueLeftNm) *
                       g_state.wheel_torque_scale;
    d->ctrl[g_state.act_wheel_L] = clamp_ctrl(m, g_state.act_wheel_L, torque_nm);
  }
  if (g_state.act_wheel_R >= 0) {
    double torque_nm = static_cast<double>(cmd.torque.torqueRightNm) *
                       g_state.wheel_torque_scale;
    // Same ctrl to both wheels — empirically verified both wheels
    // contribute the same direction of pitch when given same ctrl sign
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
    d->ctrl[g_state.act_hip_L] = clamp_ctrl(m, g_state.act_hip_L, g_state.hip_L_target);
    d->ctrl[g_state.act_hip_R] = clamp_ctrl(m, g_state.act_hip_R, g_state.hip_R_target);
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
  out->theta_ref_limit = 0.05f;  // 0.05 rad ≈ 3° — tighter than default to prevent divergence
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

  if (!g_state.lqr_params_valid) {
    init_lqr_params_defaults(&g_state.lqr_params);
    g_state.lqr_params_valid = true;
  }
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

  update_wheel_geometry(m, d);

  if (g_state.hold_hips && g_state.jnt_hip_L >= 0 && g_state.jnt_hip_R >= 0) {
    double hip_L_key = d->qpos[m->jnt_qposadr[g_state.jnt_hip_L]];
    double hip_R_key = d->qpos[m->jnt_qposadr[g_state.jnt_hip_R]];
    if (g_state.hip_target_mid) {
      g_state.hip_L_target = hip_target_from_range(m, g_state.jnt_hip_L, hip_L_key);
      g_state.hip_R_target = hip_target_from_range(m, g_state.jnt_hip_R, hip_R_key);
    } else {
      g_state.hip_L_target = hip_L_key;
      g_state.hip_R_target = hip_R_key;
    }
  }

  g_state.controller.setControlDt(static_cast<float>(m->opt.timestep));
  g_state.controller.setTargetVelocity(0.0f);
  g_state.controller.setRequestedMode(InnerLongMode::LQR);
  init_lqr_params_defaults(&g_state.lqr_params);
  g_state.lqr_params_valid = true;

  reset_ekf(m, d);
  update_lqr_from_hip(m, d, true);

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

  double theta = 0.0;
  double theta_dot = 0.0;
  double x = 0.0;
  double x_dot_world = 0.0;
  compute_kinematics(m, d, &theta, &theta_dot, &x, &x_dot_world);
  update_lqr_from_hip(m, d, false);

  StateEstimate est{};
  est.gyroBias = 0.0f;
  est.valid = true;

  if (g_state.ekf_initialized) {
    mjtNum gyro_body[3] = {0, 0, 0};
    mjtNum accel_body[3] = {0, 0, 0};
    bool have_gyro = read_sensor_vec3(m, d, g_state.sensor_gyro, gyro_body);
    bool have_acc = read_sensor_vec3(m, d, g_state.sensor_acc, accel_body);

    float theta_acc = NAN;
    if (have_acc) {
      float ay = static_cast<float>(accel_body[1]);
      float az = static_cast<float>(accel_body[2]);
      float accel_yz = std::sqrt(ay * ay + az * az);
      if (accel_yz > 1e-6f) {
        theta_acc = std::atan2(-static_cast<float>(accel_body[0]), accel_yz);
      }
    }

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

    float v_enc = static_cast<float>(g_state.wheel_radius * 0.5 * (omega_L + omega_R));
    float yaw_rate_enc = 0.0f;
    if (g_state.wheel_base > 1e-6) {
      yaw_rate_enc = static_cast<float>(g_state.wheel_radius * (omega_R - omega_L) /
                                        g_state.wheel_base);
    }

    float pos_enc = NAN;
    float dt = static_cast<float>(m->opt.timestep);
    bool ok = g_state.ekf.step(theta_acc, v_enc, pos_enc,
                               gyro_pitch, gyro_yaw, yaw_rate_enc,
                               dt, NAN);
    BalancerState st = g_state.ekf.getState();
    est.theta = st.theta;
    est.thetaDot = st.thetaDot;
    est.x = st.x;
    est.xDot = st.xDot;
    est.gyroBias = st.gyroBias;
    est.valid = ok && g_state.ekf.isStateValid();
  } else {
    est.theta = static_cast<float>(theta);
    est.thetaDot = static_cast<float>(theta_dot);
    est.x = static_cast<float>(x);
    est.xDot = static_cast<float>(x_dot_world);
  }

  const float dt = static_cast<float>(m->opt.timestep);
  MotionController::Command cmd = g_state.controller.computeControl(est, dt);

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
