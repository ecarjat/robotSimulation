// Fast system ID: measure response in first 20-50ms before robot falls
// Start from keyframe (theta≈0), apply torque, measure immediate response

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

    // ========== BASELINE: No control ==========
    printf("=== Baseline: No control from keyframe ===\n");
    {
        mjData* d = mj_makeData(m);
        mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
        
        printf("t=0: theta=%.6f rad, thetaDot=%.4f\n", get_pitch(d), d->qvel[4]);
        
        double prev_thetaDot = d->qvel[4];
        printf("\n%8s %10s %10s %10s\n", "time_ms", "theta_deg", "thetaDot", "thetaDDot");
        
        for (int step = 0; step < 50; step++) {
            d->ctrl[act_wL] = 0; d->ctrl[act_wR] = 0;
            d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
            mj_step(m, d);
            
            double thetaDDot = (d->qvel[4] - prev_thetaDot) / dt;
            prev_thetaDot = d->qvel[4];
            
            if (step % 5 == 4) {
                printf("%8.1f %10.4f %10.4f %10.2f\n", 
                       (step+1)*dt*1000, get_pitch(d)*57.3, d->qvel[4], thetaDDot);
            }
        }
        mj_deleteData(d);
    }
    
    // ========== TEST: Multiple torque levels ==========
    printf("\n=== Torque step test (first 20ms) ===\n");
    printf("%8s %10s %10s %10s %10s %10s\n", 
           "ctrl", "u(Nm)", "th_20ms", "thDot_20ms", "avg_thDDot", "b=ddot/u");
    
    double b_estimates[20];
    int b_count = 0;
    
    for (double ctrl_val : {0.0, 0.1, 0.2, 0.3, 0.5, 1.0, -0.1, -0.2, -0.3, -0.5, -1.0}) {
        mjData* d = mj_makeData(m);
        mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
        
        double theta0 = get_pitch(d);
        double thetaDot0 = d->qvel[4];
        double u = -2.0 * ctrl_val;  // total torque
        
        // Collect thetaDDot samples
        double ddot_sum = 0;
        double prev_thetaDot = thetaDot0;
        
        for (int step = 0; step < 10; step++) {  // 20ms
            d->ctrl[act_wL] = ctrl_val; d->ctrl[act_wR] = ctrl_val;
            d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
            mj_step(m, d);
            
            double thetaDDot = (d->qvel[4] - prev_thetaDot) / dt;
            prev_thetaDot = d->qvel[4];
            
            if (step >= 2) {  // skip first 2 steps (transient)
                ddot_sum += thetaDDot;
            }
        }
        
        double theta_final = get_pitch(d);
        double thetaDot_final = d->qvel[4];
        double avg_ddot = ddot_sum / 8;  // 8 samples after transient
        
        // b estimate: difference from baseline
        // At theta≈0, thetaDDot ≈ b*u (gravity term small)
        double b_est = (std::abs(u) > 0.01) ? avg_ddot / u : 0;
        
        printf("%8.2f %10.2f %10.4f %10.4f %10.2f %10.2f\n",
               ctrl_val, u, theta_final*57.3, thetaDot_final, avg_ddot, b_est);
        
        if (std::abs(u) > 0.01) {
            b_estimates[b_count++] = b_est;
        }
        
        mj_deleteData(d);
    }
    
    // Compute b correcting for baseline drift
    // baseline (u=0) gives the natural acceleration due to initial offset
    // Subtract baseline from other measurements
    printf("\n=== Corrected b estimates (subtracting baseline) ===\n");
    
    // Get baseline ddot
    mjData* d = mj_makeData(m);
    mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);
    double ddot_baseline = 0;
    double prev_thetaDot = d->qvel[4];
    for (int step = 0; step < 10; step++) {
        d->ctrl[act_wL] = 0; d->ctrl[act_wR] = 0;
        d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
        mj_step(m, d);
        double thetaDDot = (d->qvel[4] - prev_thetaDot) / dt;
        prev_thetaDot = d->qvel[4];
        if (step >= 2) ddot_baseline += thetaDDot;
    }
    ddot_baseline /= 8;
    printf("Baseline avg_thetaDDot (u=0): %.2f rad/s²\n\n", ddot_baseline);
    mj_deleteData(d);
    
    printf("%8s %10s %10s %10s\n", "ctrl", "u(Nm)", "ddot_corr", "b_corr");
    
    double b_sum = 0;
    int b_valid = 0;
    
    for (double ctrl_val : {0.1, 0.2, 0.3, 0.5, 1.0, -0.1, -0.2, -0.3, -0.5, -1.0}) {
        d = mj_makeData(m);
        mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
        
        double u = -2.0 * ctrl_val;
        double ddot_sum = 0;
        prev_thetaDot = d->qvel[4];
        
        for (int step = 0; step < 10; step++) {
            d->ctrl[act_wL] = ctrl_val; d->ctrl[act_wR] = ctrl_val;
            d->ctrl[act_hL] = 0; d->ctrl[act_hR] = 0;
            mj_step(m, d);
            double thetaDDot = (d->qvel[4] - prev_thetaDot) / dt;
            prev_thetaDot = d->qvel[4];
            if (step >= 2) ddot_sum += thetaDDot;
        }
        
        double avg_ddot = ddot_sum / 8;
        double ddot_corrected = avg_ddot - ddot_baseline;  // remove gravity effect
        double b_corr = ddot_corrected / u;
        
        printf("%8.2f %10.2f %10.2f %10.2f\n", ctrl_val, u, ddot_corrected, b_corr);
        
        b_sum += b_corr;
        b_valid++;
        
        mj_deleteData(d);
    }
    
    double b_final = b_sum / b_valid;
    
    // ========== Estimate 'a' from baseline ==========
    // At t=0, theta≈0 but there's initial offset causing fall
    // After a few ms, theta grows, and thetaDDot = a*theta
    // But this is complicated by the initial conditions
    
    // Alternative: use baseline ddot and approximate theta to get 'a'
    // thetaDDot ≈ a * theta_avg
    d = mj_makeData(m);
    mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);
    
    double theta_sum = 0;
    double ddot_sum = 0;
    prev_thetaDot = d->qvel[4];
    
    for (int step = 0; step < 25; step++) {  // 50ms
        d->ctrl[act_wL] = 0; d->ctrl[act_wR] = 0;
        mj_step(m, d);
        double theta = get_pitch(d);
        double thetaDDot = (d->qvel[4] - prev_thetaDot) / dt;
        prev_thetaDot = d->qvel[4];
        
        if (step >= 5) {  // skip initial transient
            theta_sum += theta;
            ddot_sum += thetaDDot;
        }
    }
    
    double theta_avg = theta_sum / 20;
    double ddot_avg = ddot_sum / 20;
    double a_est = (std::abs(theta_avg) > 1e-4) ? ddot_avg / theta_avg : 0;
    
    printf("\n=== Gravity term 'a' estimate ===\n");
    printf("Avg theta (5-50ms): %.6f rad\n", theta_avg);
    printf("Avg thetaDDot: %.2f rad/s²\n", ddot_avg);
    printf("a ≈ thetaDDot / theta = %.2f rad/s² per rad\n", a_est);
    
    mj_deleteData(d);
    
    // ========== SUMMARY ==========
    printf("\n========== FINAL RESULTS ==========\n");
    printf("Model: theta_ddot = a * theta + b * u\n\n");
    printf("  a = %.2f rad/s² per rad\n", a_est);
    printf("  b = %.2f rad/s² per Nm\n", b_final);
    
    if (a_est > 0) {
        double pole = std::sqrt(a_est);
        printf("\nOpen-loop unstable pole: %.2f rad/s (%.2f Hz)\n", pole, pole/(2*M_PI));
        printf("Time constant: %.3f s\n", 1.0/pole);
    }
    
    printf("\n=== Python copy-paste ===\n");
    printf("a = %.4f  # rad/s² per rad\n", a_est);
    printf("b = %.4f  # rad/s² per Nm\n", b_final);
    
    mj_deleteModel(m);
    return 0;
}
