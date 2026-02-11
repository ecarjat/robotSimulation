// Simple system ID: measure a and b empirically
// Let robot settle, then apply torque pulse and measure response
// Model: theta_ddot = a*theta + b*u

#include <mujoco/mujoco.h>
#include <cstdio>
#include <cmath>
#include <vector>

double get_pitch(const mjData* d) {
    double qw = d->qpos[3], qx = d->qpos[4], qy = d->qpos[5], qz = d->qpos[6];
    return std::asin(2.0 * (qw * qy - qz * qx));
}

int main(int argc, char** argv) {
    const char* model_path = "myRobot/scene.xml";
    if (argc > 1) model_path = argv[1];

    char error[1024];
    mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m) { printf("Error: %s\n", error); return 1; }

    int act_wL = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_L");
    int act_wR = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_R");
    int act_hL = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");
    int act_hR = mj_name2id(m, mjOBJ_ACTUATOR, "hip_R");

    double dt = m->opt.timestep;
    printf("dt = %.4f s\n\n", dt);

    // ========== FIND EQUILIBRIUM ==========
    printf("=== Finding equilibrium (let robot settle) ===\n");
    mjData* d = mj_makeData(m);
    mj_resetDataKeyframe(m, d, 0);
    
    // Let it settle for 0.5s
    for (int i = 0; i < 250; i++) {
        d->ctrl[act_wL] = 0; d->ctrl[act_wR] = 0;
        d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
        mj_step(m, d);
    }
    
    double theta_eq = get_pitch(d);
    double thetaDot_eq = d->qvel[4]; // wy
    printf("Equilibrium: theta=%.4f rad (%.2f°), thetaDot=%.4f\n", 
           theta_eq, theta_eq*57.3, thetaDot_eq);
    
    // ========== TEST 1: Torque pulse response ==========
    printf("\n=== Test 1: Torque pulse (ctrl=0.5 for 10ms) ===\n");
    
    // Re-settle
    mj_resetDataKeyframe(m, d, 0);
    for (int i = 0; i < 250; i++) {
        d->ctrl[act_wL] = 0; d->ctrl[act_wR] = 0;
        d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
        mj_step(m, d);
    }
    
    double theta0 = get_pitch(d);
    double thetaDot0 = d->qvel[4];
    printf("Before pulse: theta=%.6f, thetaDot=%.4f\n", theta0, thetaDot0);
    
    // Apply pulse (10ms = 5 steps at 2ms)
    double ctrl_val = 0.5;  // ctrl value
    double u = -2.0 * ctrl_val;  // total torque (gear=-1, 2 wheels)
    printf("Applying ctrl=%.2f → torque=%.2f Nm for 10ms\n", ctrl_val, u);
    
    std::vector<double> theta_hist, thetaDot_hist, t_hist;
    
    for (int step = 0; step < 100; step++) {
        // Pulse for first 5 steps (10ms)
        double ctrl = (step < 5) ? ctrl_val : 0.0;
        d->ctrl[act_wL] = ctrl; d->ctrl[act_wR] = ctrl;
        d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
        
        mj_step(m, d);
        
        t_hist.push_back(step * dt);
        theta_hist.push_back(get_pitch(d));
        thetaDot_hist.push_back(d->qvel[4]);
    }
    
    // Measure immediate response
    double dTheta = theta_hist[5] - theta0;
    double dThetaDot = thetaDot_hist[5] - thetaDot0;
    double impulse = u * 0.010;  // torque × time
    printf("\nAfter 10ms pulse:\n");
    printf("  delta_theta = %.6f rad\n", dTheta);
    printf("  delta_thetaDot = %.4f rad/s\n", dThetaDot);
    printf("  Impulse = %.4f Nm·s\n", impulse);
    printf("  b_eff (dThetaDot/u/dt_pulse) ≈ %.2f rad/s² per Nm\n", dThetaDot / u / 0.010);
    
    // Print trajectory
    printf("\nTrajectory (every 10ms):\n");
    printf("%8s %10s %10s\n", "time", "theta", "thetaDot");
    for (size_t i = 0; i < theta_hist.size(); i += 5) {
        printf("%8.3f %10.6f %10.4f\n", t_hist[i], theta_hist[i], thetaDot_hist[i]);
    }
    
    // ========== TEST 2: Different torque levels ==========
    printf("\n=== Test 2: Multiple torque levels (measure b) ===\n");
    printf("%10s %10s %10s %10s\n", "ctrl", "u(Nm)", "dThetaDot", "b_est");
    
    double b_sum = 0;
    int b_count = 0;
    
    for (double test_ctrl : {0.1, 0.2, 0.3, 0.4, 0.5, -0.1, -0.2, -0.3}) {
        mj_resetDataKeyframe(m, d, 0);
        for (int i = 0; i < 250; i++) {
            d->ctrl[act_wL] = 0; d->ctrl[act_wR] = 0;
            d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
            mj_step(m, d);
        }
        
        double t0 = get_pitch(d);
        double td0 = d->qvel[4];
        
        // 10ms pulse
        for (int step = 0; step < 5; step++) {
            d->ctrl[act_wL] = test_ctrl; d->ctrl[act_wR] = test_ctrl;
            mj_step(m, d);
        }
        
        double td1 = d->qvel[4];
        double test_u = -2.0 * test_ctrl;
        double dtd = td1 - td0;
        double b_est = dtd / test_u / 0.010;  // rad/s² per Nm
        
        printf("%10.2f %10.2f %10.4f %10.2f\n", test_ctrl, test_u, dtd, b_est);
        b_sum += b_est;
        b_count++;
    }
    
    double b_avg = b_sum / b_count;
    printf("\nAverage b = %.2f rad/s² per Nm\n", b_avg);
    
    // ========== TEST 3: Free oscillation (measure a) ==========
    printf("\n=== Test 3: Free oscillation from offset (measure a) ===\n");
    
    mj_resetDataKeyframe(m, d, 0);
    for (int i = 0; i < 250; i++) {
        d->ctrl[act_wL] = 0; d->ctrl[act_wR] = 0;
        d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
        mj_step(m, d);
    }
    
    // Give it a small pitch offset via torque pulse
    for (int i = 0; i < 10; i++) {
        d->ctrl[act_wL] = 0.3; d->ctrl[act_wR] = 0.3;
        mj_step(m, d);
    }
    
    printf("After offset pulse: theta=%.4f rad\n", get_pitch(d));
    
    // Now let it evolve freely and measure theta_ddot vs theta
    printf("\nFree evolution (u=0):\n");
    printf("%8s %10s %10s %10s %10s\n", "time", "theta", "thetaDot", "thetaDDot", "a=ddot/th");
    
    double prev_thetaDot = d->qvel[4];
    double a_sum = 0;
    int a_count = 0;
    
    for (int step = 0; step < 150; step++) {
        d->ctrl[act_wL] = 0; d->ctrl[act_wR] = 0;
        d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
        mj_step(m, d);
        
        double theta = get_pitch(d);
        double thetaDot = d->qvel[4];
        double thetaDDot = (thetaDot - prev_thetaDot) / dt;
        prev_thetaDot = thetaDot;
        
        // Estimate a from theta_ddot = a * theta (when u=0)
        double a_est = (std::abs(theta) > 0.001) ? thetaDDot / theta : 0;
        
        if (step > 5 && step < 100 && std::abs(theta) > 0.005) {
            a_sum += a_est;
            a_count++;
        }
        
        if (step % 10 == 0) {
            printf("%8.3f %10.6f %10.4f %10.2f %10.2f\n", 
                   step*dt, theta, thetaDot, thetaDDot, a_est);
        }
        
        if (std::abs(theta) > 0.5) break;
    }
    
    double a_avg = (a_count > 0) ? a_sum / a_count : 0;
    printf("\nAverage a = %.2f rad/s² per rad\n", a_avg);
    
    // ========== SUMMARY ==========
    printf("\n========== SUMMARY ==========\n");
    printf("Model: theta_ddot = a * theta + b * u\n");
    printf("  a = %.2f rad/s² per rad\n", a_avg);
    printf("  b = %.2f rad/s² per Nm\n", b_avg);
    if (a_avg > 0) {
        printf("\nOpen-loop unstable pole: sqrt(%.2f) = %.2f rad/s (%.1f Hz)\n", 
               a_avg, std::sqrt(a_avg), std::sqrt(a_avg)/(2*M_PI));
    }
    printf("\nFor LQR with Q=diag([1,1,10,1]), R=[1]:\n");
    printf("  (Use these a,b values in compute_lqr2.py)\n");
    
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
