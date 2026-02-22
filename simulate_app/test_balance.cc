// Headless balance test for robot control verification
#include <mujoco/mujoco.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "motion_controller_bridge.h"

namespace {

constexpr double kBridgeDefaultWheelRadius = 0.08547;

double GetPitch(const mjData* d) {
  const double qw = d->qpos[3];
  const double qx = d->qpos[4];
  const double qy = d->qpos[5];
  const double qz = d->qpos[6];
  return std::asin(2.0 * (qw * qy - qz * qx));
}

double GetYaw(const mjData* d) {
  const double qw = d->qpos[3];
  const double qx = d->qpos[4];
  const double qy = d->qpos[5];
  const double qz = d->qpos[6];
  return std::atan2(2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz));
}

struct BalanceStats {
  double theta_ref = 0.0;
  double max_theta = 0.0;
  double max_theta_err = 0.0;
  double max_theta_dot = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  double max_xdot = 0.0;
  double max_ydot = 0.0;
  double rms_theta = 0.0;
  double rms_theta_err = 0.0;
  double rms_x = 0.0;
  double rms_y = 0.0;
  double final_x = 0.0;
  double final_y = 0.0;
  double final_xdot = 0.0;
  double final_ydot = 0.0;
  double final_theta = 0.0;
  double final_theta_err = 0.0;
  double final_theta_dot = 0.0;
  int samples = 0;
  bool diverged = false;
};

struct DiagContext {
  int sensor_gyro = -1;
  int sensor_acc = -1;
  int jnt_wheel_L = -1;
  int jnt_wheel_R = -1;
  int geom_wheel_L = -1;
  int body_torso = -1;
  double wheel_radius = kBridgeDefaultWheelRadius;
  double venc_scale = 1.0;
};

struct DiagStats {
  int theta_acc_samples = 0;
  int gyro_pitch_samples = 0;
  int v_enc_samples = 0;
  int v_sign_samples = 0;
  int v_sign_agree = 0;
  double sum_sq_theta_acc_err = 0.0;
  double max_abs_theta_acc_err = 0.0;
  double sum_sq_gyro_pitch_err = 0.0;
  double max_abs_gyro_pitch_err = 0.0;
  double sum_sq_v_enc_err = 0.0;
  double max_abs_v_enc_err = 0.0;
  double sum_venc_times_xdot = 0.0;
  double sum_venc_sq = 0.0;
  int accel_norm_samples = 0;
  double accel_norm_sum = 0.0;
  double accel_norm_min = 1e9;
  double accel_norm_max = 0.0;
  int theta_grav_samples = 0;
  double sum_sq_theta_grav_err = 0.0;
  double max_abs_theta_grav_err = 0.0;
  int theta_acc_vs_grav_samples = 0;
  double sum_sq_theta_acc_vs_grav = 0.0;
  double max_abs_theta_acc_vs_grav = 0.0;
  int linacc_samples = 0;
  double sum_sq_linacc = 0.0;
  double max_linacc = 0.0;
  int theta_acc_xacc_samples = 0;
  double sum_theta_acc_err_xacc = 0.0;
  double sum_xacc = 0.0;
  double sum_sq_xacc = 0.0;
  int tilt_candidate_samples = 0;
  double sum_sq_tilt_err_ax_neg = 0.0;
  double sum_sq_tilt_err_ax_pos = 0.0;
  double sum_sq_tilt_err_ay_neg = 0.0;
  double sum_sq_tilt_err_ay_pos = 0.0;
  bool theta_fd_initialized = false;
  double theta_prev = 0.0;
};

bool ReadSensorVec3(const mjModel* m, const mjData* d, int sensor_id, mjtNum out[3]) {
  if (sensor_id < 0) {
    return false;
  }
  const int adr = m->sensor_adr[sensor_id];
  const int dim = m->sensor_dim[sensor_id];
  if (dim < 3) {
    return false;
  }
  out[0] = d->sensordata[adr + 0];
  out[1] = d->sensordata[adr + 1];
  out[2] = d->sensordata[adr + 2];
  return true;
}

double EnvVarDouble(const char* name, double fallback) {
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

void InitDiagContext(const mjModel* m, DiagContext& ctx) {
  ctx.sensor_gyro = mj_name2id(m, mjOBJ_SENSOR, "gyro");
  ctx.sensor_acc = mj_name2id(m, mjOBJ_SENSOR, "acc");
  ctx.jnt_wheel_L = mj_name2id(m, mjOBJ_JOINT, "wheel_L");
  ctx.jnt_wheel_R = mj_name2id(m, mjOBJ_JOINT, "wheel_R");
  ctx.geom_wheel_L = mj_name2id(m, mjOBJ_GEOM, "wheel_geom_L");
  ctx.body_torso = mj_name2id(m, mjOBJ_BODY, "torso");
  ctx.wheel_radius = kBridgeDefaultWheelRadius;
  ctx.venc_scale = EnvVarDouble("SIM_VENC_SCALE", 1.0);
  if (ctx.geom_wheel_L >= 0) {
    const double r = m->geom_size[3 * ctx.geom_wheel_L + 0];
    if (r > 0.0) {
      ctx.wheel_radius = r;
    }
  }
}

void UpdateDiag(const mjModel* m, const mjData* d, const DiagContext& ctx, DiagStats& stats,
                bool print_line) {
  const double theta = GetPitch(d);
  const double xdot = d->qvel[0];

  if (!stats.theta_fd_initialized) {
    stats.theta_prev = theta;
    stats.theta_fd_initialized = true;
  }
  const double theta_dot_fd = (theta - stats.theta_prev) / m->opt.timestep;
  stats.theta_prev = theta;

  double omegaL = 0.0;
  double omegaR = 0.0;
  if (ctx.jnt_wheel_L >= 0) {
    omegaL = d->qvel[m->jnt_dofadr[ctx.jnt_wheel_L]];
  }
  if (ctx.jnt_wheel_R >= 0) {
    omegaR = d->qvel[m->jnt_dofadr[ctx.jnt_wheel_R]];
  }
  // Mirror motion_controller_bridge encoder velocity convention.
  const double v_enc = -ctx.venc_scale * ctx.wheel_radius * 0.5 * (omegaL + omegaR);
  const double v_err = v_enc - xdot;
  stats.v_enc_samples++;
  stats.sum_sq_v_enc_err += v_err * v_err;
  stats.max_abs_v_enc_err = std::max(stats.max_abs_v_enc_err, std::abs(v_err));
  stats.sum_venc_times_xdot += v_enc * xdot;
  stats.sum_venc_sq += v_enc * v_enc;
  if (std::abs(v_enc) > 0.02 && std::abs(xdot) > 0.02) {
    stats.v_sign_samples++;
    if ((v_enc > 0.0) == (xdot > 0.0)) {
      stats.v_sign_agree++;
    }
  }

  mjtNum gyro_body[3] = {0, 0, 0};
  mjtNum accel_body[3] = {0, 0, 0};
  const bool have_gyro = ReadSensorVec3(m, d, ctx.sensor_gyro, gyro_body);
  const bool have_acc = ReadSensorVec3(m, d, ctx.sensor_acc, accel_body);

  double theta_acc = NAN;
  double theta_acc_err = NAN;
  double theta_grav = NAN;
  double theta_grav_err = NAN;
  double theta_acc_vs_grav = NAN;
  double linacc_norm = NAN;
  if (have_acc) {
    const double ax = accel_body[0];
    const double ay = accel_body[1];
    const double az = accel_body[2];
    const double ayz = std::sqrt(ay * ay + az * az);
    const double anorm = std::sqrt(ax * ax + ay * ay + az * az);
    stats.accel_norm_samples++;
    stats.accel_norm_sum += anorm;
    stats.accel_norm_min = std::min(stats.accel_norm_min, anorm);
    stats.accel_norm_max = std::max(stats.accel_norm_max, anorm);
    if (ayz > 1e-9) {
      theta_acc = std::atan2(-ax, ayz);
      theta_acc_err = theta_acc - theta;
      stats.theta_acc_samples++;
      stats.sum_sq_theta_acc_err += theta_acc_err * theta_acc_err;
      stats.max_abs_theta_acc_err = std::max(stats.max_abs_theta_acc_err,
                                             std::abs(theta_acc_err));
      const double xacc = d->qacc[0];
      if (std::isfinite(xacc)) {
        stats.theta_acc_xacc_samples++;
        stats.sum_theta_acc_err_xacc += theta_acc_err * xacc;
        stats.sum_sq_theta_acc_err += theta_acc_err * theta_acc_err;
        stats.sum_xacc += xacc;
        stats.sum_sq_xacc += xacc * xacc;
      }
    }
    const double axz = std::sqrt(ax * ax + az * az);
    if (ayz > 1e-9 && axz > 1e-9) {
      const double t_ax_neg = std::atan2(-ax, ayz);
      const double t_ax_pos = std::atan2(+ax, ayz);
      const double t_ay_neg = std::atan2(-ay, axz);
      const double t_ay_pos = std::atan2(+ay, axz);
      stats.tilt_candidate_samples++;
      stats.sum_sq_tilt_err_ax_neg += (t_ax_neg - theta) * (t_ax_neg - theta);
      stats.sum_sq_tilt_err_ax_pos += (t_ax_pos - theta) * (t_ax_pos - theta);
      stats.sum_sq_tilt_err_ay_neg += (t_ay_neg - theta) * (t_ay_neg - theta);
      stats.sum_sq_tilt_err_ay_pos += (t_ay_pos - theta) * (t_ay_pos - theta);
    }
    if (ctx.body_torso >= 0) {
      // Compute gravity in body frame directly from freejoint quaternion.
      // qpos quaternion stores body orientation in world (body->world).
      const double qw = d->qpos[3];
      const double qx = d->qpos[4];
      const double qy = d->qpos[5];
      const double qz = d->qpos[6];
      const double r20 = 2.0 * (qx * qz - qy * qw);
      const double r21 = 2.0 * (qy * qz + qx * qw);
      const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
      const double g_body_x = -9.81 * r20;
      const double g_body_y = -9.81 * r21;
      const double g_body_z = -9.81 * r22;
      const double gyz = std::sqrt(g_body_y * g_body_y + g_body_z * g_body_z);
      if (gyz > 1e-9) {
        theta_grav = std::atan2(-g_body_x, gyz);
        theta_grav_err = theta_grav - theta;
        stats.theta_grav_samples++;
        stats.sum_sq_theta_grav_err += theta_grav_err * theta_grav_err;
        stats.max_abs_theta_grav_err =
            std::max(stats.max_abs_theta_grav_err, std::abs(theta_grav_err));
      }
      if (std::isfinite(theta_acc) && std::isfinite(theta_grav)) {
        theta_acc_vs_grav = theta_acc - theta_grav;
        stats.theta_acc_vs_grav_samples++;
        stats.sum_sq_theta_acc_vs_grav += theta_acc_vs_grav * theta_acc_vs_grav;
        stats.max_abs_theta_acc_vs_grav =
            std::max(stats.max_abs_theta_acc_vs_grav, std::abs(theta_acc_vs_grav));
      }
      const double lin_x = ax - g_body_x;
      const double lin_y = ay - g_body_y;
      const double lin_z = az - g_body_z;
      linacc_norm = std::sqrt(lin_x * lin_x + lin_y * lin_y + lin_z * lin_z);
      stats.linacc_samples++;
      stats.sum_sq_linacc += linacc_norm * linacc_norm;
      stats.max_linacc = std::max(stats.max_linacc, linacc_norm);
    }
  }

  double gyro_pitch = NAN;
  double gyro_pitch_err = NAN;
  if (have_gyro) {
    gyro_pitch = gyro_body[1];
    gyro_pitch_err = gyro_pitch - theta_dot_fd;
    stats.gyro_pitch_samples++;
    stats.sum_sq_gyro_pitch_err += gyro_pitch_err * gyro_pitch_err;
    stats.max_abs_gyro_pitch_err = std::max(stats.max_abs_gyro_pitch_err,
                                            std::abs(gyro_pitch_err));
  }

  if (print_line) {
    std::printf("diag θ_gt=% .4f  θ_acc=% .4f  eθ=% .4f  θ̇_fd=% .4f  gyro_y=% .4f  eω=% .4f  "
                "xdot=% .4f  v_enc=% .4f  ev=% .4f\n",
                theta, theta_acc, theta_acc_err, theta_dot_fd, gyro_pitch, gyro_pitch_err,
                xdot, v_enc, v_err);
  }
}

void PrintDiagSummary(const DiagStats& stats) {
  std::printf("\n=== Diagnostic Summary ===\n");
  if (stats.accel_norm_samples > 0) {
    const double mean_norm = stats.accel_norm_sum / stats.accel_norm_samples;
    std::printf("IMU accel norm: mean=%.6f m/s^2 (%.4f g)  min=%.6f  max=%.6f  samples=%d\n",
                mean_norm, mean_norm / 9.81, stats.accel_norm_min, stats.accel_norm_max,
                stats.accel_norm_samples);
  }
  if (stats.theta_acc_samples > 0) {
    const double rms = std::sqrt(stats.sum_sq_theta_acc_err / stats.theta_acc_samples);
    std::printf("IMU pitch (acc) error: RMS=%.6f rad  max=%.6f rad  samples=%d\n",
                rms, stats.max_abs_theta_acc_err, stats.theta_acc_samples);
  } else {
    std::printf("IMU pitch (acc) error: no valid samples\n");
  }
  if (stats.theta_grav_samples > 0) {
    const double rms = std::sqrt(stats.sum_sq_theta_grav_err / stats.theta_grav_samples);
    std::printf("Pitch from gravity-frame quaternion error: RMS=%.6f rad  max=%.6f rad  samples=%d\n",
                rms, stats.max_abs_theta_grav_err, stats.theta_grav_samples);
  }
  if (stats.theta_acc_vs_grav_samples > 0) {
    const double rms = std::sqrt(stats.sum_sq_theta_acc_vs_grav / stats.theta_acc_vs_grav_samples);
    std::printf("IMU pitch(acc) vs gravity-only pitch: RMS=%.6f rad  max=%.6f rad  samples=%d\n",
                rms, stats.max_abs_theta_acc_vs_grav, stats.theta_acc_vs_grav_samples);
  }
  if (stats.linacc_samples > 0) {
    const double rms = std::sqrt(stats.sum_sq_linacc / stats.linacc_samples);
    std::printf("Estimated linear accel magnitude (acc - g_body): RMS=%.6f m/s^2  max=%.6f m/s^2\n",
                rms, stats.max_linacc);
  }
  if (stats.theta_acc_xacc_samples > 0 && stats.sum_sq_xacc > 1e-12) {
    const double slope = stats.sum_theta_acc_err_xacc / stats.sum_sq_xacc;
    const double mean_xacc = stats.sum_xacc / stats.theta_acc_xacc_samples;
    const double rms_xacc = std::sqrt(stats.sum_sq_xacc / stats.theta_acc_xacc_samples);
    std::printf("theta_acc_err vs xacc: slope=%.6f rad/(m/s^2)  mean_xacc=%.6f  rms_xacc=%.6f\n",
                slope, mean_xacc, rms_xacc);
  }
  if (stats.gyro_pitch_samples > 0) {
    const double rms = std::sqrt(stats.sum_sq_gyro_pitch_err / stats.gyro_pitch_samples);
    std::printf("IMU gyro_y vs theta_dot(fd): RMS=%.6f rad/s  max=%.6f rad/s  samples=%d\n",
                rms, stats.max_abs_gyro_pitch_err, stats.gyro_pitch_samples);
  } else {
    std::printf("IMU gyro_y vs theta_dot(fd): no valid samples\n");
  }
  if (stats.tilt_candidate_samples > 0) {
    const double rms_ax_neg =
        std::sqrt(stats.sum_sq_tilt_err_ax_neg / stats.tilt_candidate_samples);
    const double rms_ax_pos =
        std::sqrt(stats.sum_sq_tilt_err_ax_pos / stats.tilt_candidate_samples);
    const double rms_ay_neg =
        std::sqrt(stats.sum_sq_tilt_err_ay_neg / stats.tilt_candidate_samples);
    const double rms_ay_pos =
        std::sqrt(stats.sum_sq_tilt_err_ay_pos / stats.tilt_candidate_samples);
    std::printf("Tilt formula RMS vs theta (rad):\n");
    std::printf("  atan2(-ax, sqrt(ay^2+az^2)) = %.6f  [current]\n", rms_ax_neg);
    std::printf("  atan2(+ax, sqrt(ay^2+az^2)) = %.6f\n", rms_ax_pos);
    std::printf("  atan2(-ay, sqrt(ax^2+az^2)) = %.6f\n", rms_ay_neg);
    std::printf("  atan2(+ay, sqrt(ax^2+az^2)) = %.6f\n", rms_ay_pos);
  }
  if (stats.v_enc_samples > 0) {
    const double rms = std::sqrt(stats.sum_sq_v_enc_err / stats.v_enc_samples);
    const double scale =
        (stats.sum_venc_sq > 1e-12) ? (stats.sum_venc_times_xdot / stats.sum_venc_sq) : 0.0;
    std::printf("Encoder velocity vs xdot: RMS=%.6f m/s  max=%.6f m/s  scale(xdot/v_enc)=%.4f\n",
                rms, stats.max_abs_v_enc_err, scale);
    if (stats.v_sign_samples > 0) {
      const double agree = 100.0 * static_cast<double>(stats.v_sign_agree) /
                           static_cast<double>(stats.v_sign_samples);
      std::printf("Encoder velocity sign agreement: %.1f%% (%d/%d)\n",
                  agree, stats.v_sign_agree, stats.v_sign_samples);
    }
  }
}

void UpdateStats(const mjData* d, BalanceStats& stats) {
  // Extract state from free joint (world body to torso)
  // qpos: [x, y, z, qw, qx, qy, qz]
  // qvel: [vx, vy, vz, wx, wy, wz]
  
  double x = d->qpos[0];
  double y = d->qpos[1];
  double xdot = d->qvel[0];
  double ydot = d->qvel[1];
  
  // Use the same pitch extraction as motion_controller_bridge.
  double theta = GetPitch(d);
  double theta_err = theta - stats.theta_ref;
  
  // Angular velocity around Y
  double theta_dot = d->qvel[4];
  
  stats.max_theta = std::max(stats.max_theta, std::abs(theta));
  stats.max_theta_err = std::max(stats.max_theta_err, std::abs(theta_err));
  stats.max_theta_dot = std::max(stats.max_theta_dot, std::abs(theta_dot));
  stats.max_x = std::max(stats.max_x, std::abs(x));
  stats.max_y = std::max(stats.max_y, std::abs(y));
  stats.max_xdot = std::max(stats.max_xdot, std::abs(xdot));
  stats.max_ydot = std::max(stats.max_ydot, std::abs(ydot));
  
  stats.rms_theta += theta * theta;
  stats.rms_theta_err += theta_err * theta_err;
  stats.rms_x += x * x;
  stats.rms_y += y * y;
  stats.final_x = x;
  stats.final_y = y;
  stats.final_xdot = xdot;
  stats.final_ydot = ydot;
  stats.final_theta = theta;
  stats.final_theta_err = theta_err;
  stats.final_theta_dot = theta_dot;
  stats.samples++;
  
  // Divergence check relative to starting keyframe lean, plus rate guard.
  if (std::abs(theta_err) > 0.52 || std::abs(theta_dot) > 10.0) {
    stats.diverged = true;
  }
}

void PrintState(double t, const mjModel* m, const mjData* d) {
  double x = d->qpos[0];
  double y = d->qpos[1];
  double xdot = d->qvel[0];
  double ydot = d->qvel[1];
  double theta = GetPitch(d);
  double yaw = GetYaw(d);
  double theta_dot = d->qvel[4];
  
  // Get wheel velocities
  int wheel_L_jnt = mj_name2id(m, mjOBJ_JOINT, "wheel_L");
  int wheel_R_jnt = mj_name2id(m, mjOBJ_JOINT, "wheel_R");
  
  double wheel_L_vel = 0.0;
  double wheel_R_vel = 0.0;
  if (wheel_L_jnt >= 0 && wheel_L_jnt < m->njnt) {
    int qvel_adr = m->jnt_dofadr[wheel_L_jnt];
    if (qvel_adr >= 0 && qvel_adr < m->nv) {
      wheel_L_vel = d->qvel[qvel_adr];
    }
  }
  if (wheel_R_jnt >= 0 && wheel_R_jnt < m->njnt) {
    int qvel_adr = m->jnt_dofadr[wheel_R_jnt];
    if (qvel_adr >= 0 && qvel_adr < m->nv) {
      wheel_R_vel = d->qvel[qvel_adr];
    }
  }
  
  std::printf("t=%.3f  x=% 7.4f  y=% 7.4f  vx=% 7.4f  vy=% 7.4f  ψ=% 7.2f°  θ=% 7.4f°  θ̇=% 7.4f  "
              "wL=% 7.3f  wR=% 7.3f\n",
              t, x, y, xdot, ydot, yaw * 57.3, theta * 57.3, theta_dot,
              wheel_L_vel, wheel_R_vel);
}

void PrintAvailableKeyframes(const mjModel* m) {
  std::printf("Available keyframes (%d):\n", m->nkey);
  for (int i = 0; i < m->nkey; ++i) {
    const char* name = mj_id2name(m, mjOBJ_KEY, i);
    std::printf("  [%d] %s\n", i, (name && name[0]) ? name : "(unnamed)");
  }
}

bool ResolveKeyframe(const mjModel* m, const char* key_spec, int* key_idx_out) {
  if (!key_spec || !key_spec[0]) {
    return false;
  }

  char* end = nullptr;
  long idx = std::strtol(key_spec, &end, 10);
  if (end && *end == '\0') {
    if (idx < 0 || idx >= m->nkey) {
      std::printf("ERROR: keyframe index out of range: %ld (valid [0, %d))\n", idx, m->nkey);
      return false;
    }
    *key_idx_out = static_cast<int>(idx);
    return true;
  }

  const int id = mj_name2id(m, mjOBJ_KEY, key_spec);
  if (id < 0) {
    std::printf("ERROR: keyframe name not found: '%s'\n", key_spec);
    return false;
  }

  *key_idx_out = id;
  return true;
}

int RunBalanceTest(const char* model_path, double duration, double print_dt, bool diag_enabled,
                   const char* key_spec, int turn_key, double turn_for_s) {
  char error[1024] = "";
  mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
  
  if (!m) {
    std::printf("ERROR: Failed to load model: %s\n", error);
    return 1;
  }
  
  mjData* d = mj_makeData(m);
  if (!d) {
    std::printf("ERROR: Failed to create data\n");
    mj_deleteModel(m);
    return 1;
  }
  
  // Reset to requested keyframe (or keyframe 0 by default) if available.
  if (m->nkey > 0) {
    int key_idx = 0;
    if (key_spec && key_spec[0]) {
      if (!ResolveKeyframe(m, key_spec, &key_idx)) {
        PrintAvailableKeyframes(m);
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
      }
    }
    mj_resetDataKeyframe(m, d, key_idx);
    const char* key_name = mj_id2name(m, mjOBJ_KEY, key_idx);
    std::printf("Reset to keyframe %d (%s)\n", key_idx,
                (key_name && key_name[0]) ? key_name : "(unnamed)");
  } else if (key_spec && key_spec[0]) {
    std::printf("ERROR: model has no keyframes, cannot use --key '%s'\n", key_spec);
    mj_deleteData(d);
    mj_deleteModel(m);
    return 1;
  }
  
  // Initialize controller
  mj_forward(m, d);
  BalanceStats stats;
  stats.theta_ref = GetPitch(d);
  MotionControllerReset(m, d);
  
  // Enable controller if not already enabled
  if (!MotionControllerIsEnabled()) {
    MotionControllerToggle();
  }
  
  std::printf("\n=== Balance Test ===\n");
  std::printf("Model: %s\n", model_path);
  std::printf("Duration: %.1f seconds\n", duration);
  std::printf("Timestep: %.4f seconds\n", m->opt.timestep);
  std::printf("Controller enabled: %s\n\n", MotionControllerIsEnabled() ? "YES" : "NO");
  
  std::printf("Legend: x/y=position(m)  vx/vy=velocity(m/s)  ψ=yaw(deg)  θ=pitch(deg)  θ̇=angular_vel(rad/s)  "
              "wL/wR=wheel_vel(rad/s)\n\n");
  
  DiagContext diag_ctx;
  DiagStats diag_stats;
  if (diag_enabled) {
    InitDiagContext(m, diag_ctx);
    std::printf("Diagnostics enabled (SIM_USE_EKF=%s)\n",
                std::getenv("SIM_USE_EKF") ? std::getenv("SIM_USE_EKF") : "unset");
    std::printf("IMU sensors: gyro=%d acc=%d\n", diag_ctx.sensor_gyro, diag_ctx.sensor_acc);
    std::printf("Wheel radius used for v_enc check: %.5f m (scale %.3f)\n\n",
                diag_ctx.wheel_radius, diag_ctx.venc_scale);
  }
  double next_print = 0.0;
  bool turn_pressed = false;
  if (turn_key != 0) {
    MotionControllerHandleArrowKey(turn_key, true);
    turn_pressed = true;
  }
  
  while (d->time < duration) {
    if (turn_pressed && turn_for_s > 0.0 && d->time >= turn_for_s) {
      MotionControllerHandleArrowKey(turn_key, false);
      turn_pressed = false;
    }
    mj_step(m, d);
    
    UpdateStats(d, stats);
    bool print_now = d->time >= next_print;
    if (diag_enabled) {
      UpdateDiag(m, d, diag_ctx, diag_stats, print_now);
    }
    
    if (print_now) {
      PrintState(d->time, m, d);
      next_print += print_dt;
    }
    
    if (stats.diverged) {
      std::printf("\n❌ DIVERGED at t=%.3f seconds\n", d->time);
      break;
    }
  }
  
  // Final statistics
  stats.rms_theta = std::sqrt(stats.rms_theta / stats.samples);
  stats.rms_theta_err = std::sqrt(stats.rms_theta_err / stats.samples);
  stats.rms_x = std::sqrt(stats.rms_x / stats.samples);
  stats.rms_y = std::sqrt(stats.rms_y / stats.samples);
  
  std::printf("\n=== Test Results ===\n");
  std::printf("Duration:      %.3f / %.3f seconds\n", d->time, duration);
  std::printf("Samples:       %d\n", stats.samples);
  std::printf("Theta ref:     %.4f° (%.6f rad)\n", stats.theta_ref * 57.3, stats.theta_ref);
  std::printf("Max |theta|:   %.4f° (%.6f rad)\n", stats.max_theta * 57.3, stats.max_theta);
  std::printf("Max |theta_err|: %.4f° (%.6f rad)\n",
              stats.max_theta_err * 57.3, stats.max_theta_err);
  std::printf("Max |theta_dot|: %.4f rad/s\n", stats.max_theta_dot);
  std::printf("RMS theta:     %.4f° (%.6f rad)\n", stats.rms_theta * 57.3, stats.rms_theta);
  std::printf("RMS theta_err: %.4f° (%.6f rad)\n",
              stats.rms_theta_err * 57.3, stats.rms_theta_err);
  std::printf("Max |x|:       %.4f m\n", stats.max_x);
  std::printf("Max |y|:       %.4f m\n", stats.max_y);
  std::printf("Max |xdot|:    %.4f m/s\n", stats.max_xdot);
  std::printf("Max |ydot|:    %.4f m/s\n", stats.max_ydot);
  std::printf("RMS x/y:       %.4f / %.4f m\n", stats.rms_x, stats.rms_y);
  std::printf("Final state:   x=% .4f  y=% .4f  vx=% .4f  vy=% .4f  θ=% .4f°  θ_err=% .4f°  θ̇=% .4f\n",
              stats.final_x, stats.final_y, stats.final_xdot, stats.final_ydot,
              stats.final_theta * 57.3, stats.final_theta_err * 57.3, stats.final_theta_dot);
  
  bool passed = !stats.diverged && stats.max_theta_err < 0.17; // < ~10 degrees from keyframe ref
  if (diag_enabled) {
    PrintDiagSummary(diag_stats);
  }
  
  if (passed) {
    std::printf("\n✅ PASS: Robot balanced successfully\n");
  } else {
    std::printf("\n❌ FAIL: Robot did not balance\n");
  }

  std::printf("SUMMARY max_theta_deg=%.6f max_theta_err_deg=%.6f max_x=%.6f max_y=%.6f "
              "max_xdot=%.6f max_ydot=%.6f final_x=%.6f final_y=%.6f final_vx=%.6f "
              "final_vy=%.6f final_theta_err_deg=%.6f diverged=%d passed=%d\n",
              stats.max_theta * 57.3, stats.max_theta_err * 57.3,
              stats.max_x, stats.max_y, stats.max_xdot, stats.max_ydot,
              stats.final_x, stats.final_y, stats.final_xdot, stats.final_ydot,
              stats.final_theta_err * 57.3,
              stats.diverged ? 1 : 0, passed ? 1 : 0);

  if (turn_pressed) {
    MotionControllerHandleArrowKey(turn_key, false);
  }
  
  mj_deleteData(d);
  mj_deleteModel(m);
  
  return passed ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
  const char* model_path = "myRobot/scene.xml";
  double duration = 5.0;  // 5 second test
  double print_dt = 0.2;   // Print every 0.2 seconds
  bool diag_enabled = false;
  const char* key_spec = nullptr;
  int turn_key = 0;
  double turn_for_s = 0.0;

  int positional = 0;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--diag")) {
      diag_enabled = true;
      continue;
    }
    if (!std::strcmp(argv[i], "--duration") && i + 1 < argc) {
      duration = std::atof(argv[++i]);
      continue;
    }
    if (!std::strcmp(argv[i], "--print-dt") && i + 1 < argc) {
      print_dt = std::atof(argv[++i]);
      continue;
    }
    if (!std::strcmp(argv[i], "--key") && i + 1 < argc) {
      key_spec = argv[++i];
      continue;
    }
    if (!std::strcmp(argv[i], "--turn") && i + 1 < argc) {
      const char* dir = argv[++i];
      if (!std::strcmp(dir, "left")) {
        turn_key = mjKEY_LEFT;
      } else if (!std::strcmp(dir, "right")) {
        turn_key = mjKEY_RIGHT;
      } else {
        std::printf("Unknown --turn value: %s (expected left|right)\n", dir);
        return 1;
      }
      continue;
    }
    if (!std::strcmp(argv[i], "--turn-for") && i + 1 < argc) {
      turn_for_s = std::atof(argv[++i]);
      continue;
    }
    if (!std::strcmp(argv[i], "--help")) {
      std::printf("Usage: test_balance [model_path] [duration_s] [--diag] "
                  "[--duration seconds] [--print-dt seconds] [--key name_or_index] "
                  "[--turn left|right] [--turn-for seconds]\n");
      return 0;
    }
    if (positional == 0) {
      model_path = argv[i];
      positional++;
    } else if (positional == 1) {
      duration = std::atof(argv[i]);
      positional++;
    } else if (positional == 2) {
      key_spec = argv[i];
      positional++;
    } else {
      std::printf("Unknown argument: %s\n", argv[i]);
      return 1;
    }
  }
  
  // Set control callback
  mjcb_control = MotionControllerCallback;
  
  if (turn_key != 0 && turn_for_s <= 0.0) {
    turn_for_s = duration;
  }

  return RunBalanceTest(model_path, duration, print_dt, diag_enabled, key_spec,
                        turn_key, turn_for_s);
}
