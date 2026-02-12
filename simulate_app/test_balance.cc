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

struct BalanceStats {
  double max_theta = 0.0;
  double max_theta_dot = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  double max_xdot = 0.0;
  double max_ydot = 0.0;
  double rms_theta = 0.0;
  double rms_x = 0.0;
  double rms_y = 0.0;
  double final_x = 0.0;
  double final_y = 0.0;
  double final_xdot = 0.0;
  double final_ydot = 0.0;
  double final_theta = 0.0;
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
  if (have_acc) {
    const double ay = accel_body[1];
    const double az = accel_body[2];
    const double ayz = std::sqrt(ay * ay + az * az);
    if (ayz > 1e-9) {
      theta_acc = std::atan2(-accel_body[0], ayz);
      theta_acc_err = theta_acc - theta;
      stats.theta_acc_samples++;
      stats.sum_sq_theta_acc_err += theta_acc_err * theta_acc_err;
      stats.max_abs_theta_acc_err = std::max(stats.max_abs_theta_acc_err,
                                             std::abs(theta_acc_err));
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
  if (stats.theta_acc_samples > 0) {
    const double rms = std::sqrt(stats.sum_sq_theta_acc_err / stats.theta_acc_samples);
    std::printf("IMU pitch (acc) error: RMS=%.6f rad  max=%.6f rad  samples=%d\n",
                rms, stats.max_abs_theta_acc_err, stats.theta_acc_samples);
  } else {
    std::printf("IMU pitch (acc) error: no valid samples\n");
  }
  if (stats.gyro_pitch_samples > 0) {
    const double rms = std::sqrt(stats.sum_sq_gyro_pitch_err / stats.gyro_pitch_samples);
    std::printf("IMU gyro_y vs theta_dot(fd): RMS=%.6f rad/s  max=%.6f rad/s  samples=%d\n",
                rms, stats.max_abs_gyro_pitch_err, stats.gyro_pitch_samples);
  } else {
    std::printf("IMU gyro_y vs theta_dot(fd): no valid samples\n");
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
  
  // Angular velocity around Y
  double theta_dot = d->qvel[4];
  
  stats.max_theta = std::max(stats.max_theta, std::abs(theta));
  stats.max_theta_dot = std::max(stats.max_theta_dot, std::abs(theta_dot));
  stats.max_x = std::max(stats.max_x, std::abs(x));
  stats.max_y = std::max(stats.max_y, std::abs(y));
  stats.max_xdot = std::max(stats.max_xdot, std::abs(xdot));
  stats.max_ydot = std::max(stats.max_ydot, std::abs(ydot));
  
  stats.rms_theta += theta * theta;
  stats.rms_x += x * x;
  stats.rms_y += y * y;
  stats.final_x = x;
  stats.final_y = y;
  stats.final_xdot = xdot;
  stats.final_ydot = ydot;
  stats.final_theta = theta;
  stats.final_theta_dot = theta_dot;
  stats.samples++;
  
  // Divergence check: fallen over (> 30 degrees) or excessive velocity
  if (std::abs(theta) > 0.52 || std::abs(theta_dot) > 10.0) {
    stats.diverged = true;
  }
}

void PrintState(double t, const mjModel* m, const mjData* d) {
  double x = d->qpos[0];
  double y = d->qpos[1];
  double xdot = d->qvel[0];
  double ydot = d->qvel[1];
  double theta = GetPitch(d);
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
  
  std::printf("t=%.3f  x=% 7.4f  y=% 7.4f  vx=% 7.4f  vy=% 7.4f  θ=% 7.4f°  θ̇=% 7.4f  "
              "wL=% 7.3f  wR=% 7.3f\n",
              t, x, y, xdot, ydot, theta * 57.3, theta_dot, wheel_L_vel, wheel_R_vel);
}

int RunBalanceTest(const char* model_path, double duration, double print_dt, bool diag_enabled) {
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
  
  // Reset to keyframe if available
  if (m->nkey > 0) {
    mj_resetDataKeyframe(m, d, 0);  // Use first keyframe
    std::printf("Reset to keyframe 0\n");
  }
  
  // Initialize controller
  mj_forward(m, d);
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
  
  std::printf("Legend: x/y=position(m)  vx/vy=velocity(m/s)  θ=pitch(deg)  θ̇=angular_vel(rad/s)  "
              "wL/wR=wheel_vel(rad/s)\n\n");
  
  BalanceStats stats;
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
  
  while (d->time < duration) {
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
  stats.rms_x = std::sqrt(stats.rms_x / stats.samples);
  stats.rms_y = std::sqrt(stats.rms_y / stats.samples);
  
  std::printf("\n=== Test Results ===\n");
  std::printf("Duration:      %.3f / %.3f seconds\n", d->time, duration);
  std::printf("Samples:       %d\n", stats.samples);
  std::printf("Max |theta|:   %.4f° (%.6f rad)\n", stats.max_theta * 57.3, stats.max_theta);
  std::printf("Max |theta_dot|: %.4f rad/s\n", stats.max_theta_dot);
  std::printf("RMS theta:     %.4f° (%.6f rad)\n", stats.rms_theta * 57.3, stats.rms_theta);
  std::printf("Max |x|:       %.4f m\n", stats.max_x);
  std::printf("Max |y|:       %.4f m\n", stats.max_y);
  std::printf("Max |xdot|:    %.4f m/s\n", stats.max_xdot);
  std::printf("Max |ydot|:    %.4f m/s\n", stats.max_ydot);
  std::printf("RMS x/y:       %.4f / %.4f m\n", stats.rms_x, stats.rms_y);
  std::printf("Final state:   x=% .4f  y=% .4f  vx=% .4f  vy=% .4f  θ=% .4f°  θ̇=% .4f\n",
              stats.final_x, stats.final_y, stats.final_xdot, stats.final_ydot,
              stats.final_theta * 57.3, stats.final_theta_dot);
  
  bool passed = !stats.diverged && stats.max_theta < 0.17; // < ~10 degrees
  if (diag_enabled) {
    PrintDiagSummary(diag_stats);
  }
  
  if (passed) {
    std::printf("\n✅ PASS: Robot balanced successfully\n");
  } else {
    std::printf("\n❌ FAIL: Robot did not balance\n");
  }

  std::printf("SUMMARY max_theta_deg=%.6f max_x=%.6f max_y=%.6f max_xdot=%.6f max_ydot=%.6f "
              "final_x=%.6f final_y=%.6f final_vx=%.6f final_vy=%.6f diverged=%d passed=%d\n",
              stats.max_theta * 57.3, stats.max_x, stats.max_y, stats.max_xdot, stats.max_ydot,
              stats.final_x, stats.final_y, stats.final_xdot, stats.final_ydot,
              stats.diverged ? 1 : 0, passed ? 1 : 0);
  
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
    if (!std::strcmp(argv[i], "--help")) {
      std::printf("Usage: test_balance [model_path] [duration_s] [--diag] "
                  "[--duration seconds] [--print-dt seconds]\n");
      return 0;
    }
    if (positional == 0) {
      model_path = argv[i];
      positional++;
    } else if (positional == 1) {
      duration = std::atof(argv[i]);
      positional++;
    } else {
      std::printf("Unknown argument: %s\n", argv[i]);
      return 1;
    }
  }
  
  // Set control callback
  mjcb_control = MotionControllerCallback;
  
  return RunBalanceTest(model_path, duration, print_dt, diag_enabled);
}
