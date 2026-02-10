// Headless balance test for robot control verification
#include <mujoco/mujoco.h>
#include <cmath>
#include <cstdio>
#include "motion_controller_bridge.h"

namespace {

struct BalanceStats {
  double max_theta = 0.0;
  double max_theta_dot = 0.0;
  double max_x = 0.0;
  double max_xdot = 0.0;
  double rms_theta = 0.0;
  int samples = 0;
  bool diverged = false;
};

void UpdateStats(const mjData* d, BalanceStats& stats) {
  // Extract state from free joint (world body to torso)
  // qpos: [x, y, z, qw, qx, qy, qz]
  // qvel: [vx, vy, vz, wx, wy, wz]
  
  double x = d->qpos[0];
  double xdot = d->qvel[0];
  
  // Small angle: theta ≈ 2*qy (quaternion to pitch)
  double qy = d->qpos[5];
  double theta = 2.0 * qy;
  
  // Angular velocity around Y
  double theta_dot = d->qvel[4];
  
  stats.max_theta = std::max(stats.max_theta, std::abs(theta));
  stats.max_theta_dot = std::max(stats.max_theta_dot, std::abs(theta_dot));
  stats.max_x = std::max(stats.max_x, std::abs(x));
  stats.max_xdot = std::max(stats.max_xdot, std::abs(xdot));
  
  stats.rms_theta += theta * theta;
  stats.samples++;
  
  // Divergence check: fallen over (> 30 degrees) or excessive velocity
  if (std::abs(theta) > 0.52 || std::abs(theta_dot) > 10.0) {
    stats.diverged = true;
  }
}

void PrintState(double t, const mjModel* m, const mjData* d) {
  double x = d->qpos[0];
  double xdot = d->qvel[0];
  double qy = d->qpos[5];
  double theta = 2.0 * qy;
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
  
  std::printf("t=%.3f  x=% 7.4f  v=% 7.4f  θ=% 7.4f°  θ̇=% 7.4f  wL=% 7.3f  wR=% 7.3f\n",
              t, x, xdot, theta * 57.3, theta_dot, wheel_L_vel, wheel_R_vel);
}

int RunBalanceTest(const char* model_path, double duration, double print_dt) {
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
  
  std::printf("Legend: x=position(m)  v=velocity(m/s)  θ=pitch(deg)  θ̇=angular_vel(rad/s)  wL/wR=wheel_vel(rad/s)\n\n");
  
  BalanceStats stats;
  double next_print = 0.0;
  
  while (d->time < duration) {
    mj_step(m, d);
    
    UpdateStats(d, stats);
    
    if (d->time >= next_print) {
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
  
  std::printf("\n=== Test Results ===\n");
  std::printf("Duration:      %.3f / %.3f seconds\n", d->time, duration);
  std::printf("Samples:       %d\n", stats.samples);
  std::printf("Max |theta|:   %.4f° (%.6f rad)\n", stats.max_theta * 57.3, stats.max_theta);
  std::printf("Max |theta_dot|: %.4f rad/s\n", stats.max_theta_dot);
  std::printf("RMS theta:     %.4f° (%.6f rad)\n", stats.rms_theta * 57.3, stats.rms_theta);
  std::printf("Max |x|:       %.4f m\n", stats.max_x);
  std::printf("Max |xdot|:    %.4f m/s\n", stats.max_xdot);
  
  bool passed = !stats.diverged && stats.max_theta < 0.17; // < ~10 degrees
  
  if (passed) {
    std::printf("\n✅ PASS: Robot balanced successfully\n");
  } else {
    std::printf("\n❌ FAIL: Robot did not balance\n");
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
  
  if (argc > 1) {
    model_path = argv[1];
  }
  if (argc > 2) {
    duration = std::atof(argv[2]);
  }
  
  // Set control callback
  mjcb_control = MotionControllerCallback;
  
  return RunBalanceTest(model_path, duration, print_dt);
}
