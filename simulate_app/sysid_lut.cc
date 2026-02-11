// System ID at multiple hip positions for LUT generation
// Outputs CSV with a, b coefficients for each hip angle

#include <mujoco/mujoco.h>
#include <cstdio>
#include <cmath>
#include <vector>

double get_pitch(const mjData* d) {
    double qw = d->qpos[3], qx = d->qpos[4], qy = d->qpos[5], qz = d->qpos[6];
    return std::asin(2.0 * (qw * qy - qz * qx));
}

struct SysidResult {
    double hip_rad;
    double a;  // gravity term (rad/s² per rad)
    double b;  // input gain (rad/s² per Nm)
    double theta_eq;  // equilibrium pitch at this hip angle
};

SysidResult run_sysid_at_hip(mjModel* m, double hip_target) {
    int act_wL = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_L");
    int act_wR = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_R");
    int act_hL = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");
    int act_hR = mj_name2id(m, mjOBJ_ACTUATOR, "hip_R");
    int jnt_hL = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int jnt_hR = mj_name2id(m, mjOBJ_JOINT, "hip_R");
    
    double dt = m->opt.timestep;
    SysidResult result = {hip_target, 0, 0, 0};
    
    // === Set hip position and let settle briefly ===
    mjData* d = mj_makeData(m);
    mj_resetDataKeyframe(m, d, 0);
    
    // Set hip joint positions directly
    int qpos_hL = m->jnt_qposadr[jnt_hL];
    int qpos_hR = m->jnt_qposadr[jnt_hR];
    d->qpos[qpos_hL] = hip_target;
    d->qpos[qpos_hR] = hip_target;
    
    mj_forward(m, d);
    
    // Hold hips with strong PD while letting body settle (10ms)
    double hip_kp = 100.0, hip_kd = 5.0;
    for (int i = 0; i < 5; i++) {
        double err_L = hip_target - d->qpos[qpos_hL];
        double err_R = hip_target - d->qpos[qpos_hR];
        int dof_hL = m->jnt_dofadr[jnt_hL];
        int dof_hR = m->jnt_dofadr[jnt_hR];
        d->ctrl[act_hL] = hip_kp * err_L - hip_kd * d->qvel[dof_hL];
        d->ctrl[act_hR] = hip_kp * err_R - hip_kd * d->qvel[dof_hR];
        d->ctrl[act_wL] = 0;
        d->ctrl[act_wR] = 0;
        mj_step(m, d);
    }
    
    // Record initial state
    double theta0 = get_pitch(d);
    result.theta_eq = theta0;  // This is approximate equilibrium pitch
    
    // === Measure 'b' (torque → pitch accel) ===
    // Apply different torque levels, measure response in first 20ms
    double b_sum = 0;
    int b_count = 0;
    
    // Get baseline (u=0)
    mj_resetDataKeyframe(m, d, 0);
    d->qpos[qpos_hL] = hip_target;
    d->qpos[qpos_hR] = hip_target;
    mj_forward(m, d);
    
    for (int i = 0; i < 5; i++) {
        double err_L = hip_target - d->qpos[qpos_hL];
        double err_R = hip_target - d->qpos[qpos_hR];
        int dof_hL = m->jnt_dofadr[jnt_hL];
        int dof_hR = m->jnt_dofadr[jnt_hR];
        d->ctrl[act_hL] = hip_kp * err_L - hip_kd * d->qvel[dof_hL];
        d->ctrl[act_hR] = hip_kp * err_R - hip_kd * d->qvel[dof_hR];
        d->ctrl[act_wL] = 0;
        d->ctrl[act_wR] = 0;
        mj_step(m, d);
    }
    
    double ddot_baseline = 0;
    double prev_thetaDot = d->qvel[4];
    for (int step = 0; step < 10; step++) {
        double err_L = hip_target - d->qpos[qpos_hL];
        double err_R = hip_target - d->qpos[qpos_hR];
        int dof_hL = m->jnt_dofadr[jnt_hL];
        int dof_hR = m->jnt_dofadr[jnt_hR];
        d->ctrl[act_hL] = hip_kp * err_L - hip_kd * d->qvel[dof_hL];
        d->ctrl[act_hR] = hip_kp * err_R - hip_kd * d->qvel[dof_hR];
        d->ctrl[act_wL] = 0;
        d->ctrl[act_wR] = 0;
        mj_step(m, d);
        
        double thetaDDot = (d->qvel[4] - prev_thetaDot) / dt;
        prev_thetaDot = d->qvel[4];
        if (step >= 2) ddot_baseline += thetaDDot;
    }
    ddot_baseline /= 8;
    
    // Test multiple torque levels
    for (double ctrl_val : {0.1, 0.2, 0.3, 0.5, -0.1, -0.2, -0.3, -0.5}) {
        mj_resetDataKeyframe(m, d, 0);
        d->qpos[qpos_hL] = hip_target;
        d->qpos[qpos_hR] = hip_target;
        mj_forward(m, d);
        
        // Brief settle with hip hold
        for (int i = 0; i < 5; i++) {
            double err_L = hip_target - d->qpos[qpos_hL];
            double err_R = hip_target - d->qpos[qpos_hR];
            int dof_hL = m->jnt_dofadr[jnt_hL];
            int dof_hR = m->jnt_dofadr[jnt_hR];
            d->ctrl[act_hL] = hip_kp * err_L - hip_kd * d->qvel[dof_hL];
            d->ctrl[act_hR] = hip_kp * err_R - hip_kd * d->qvel[dof_hR];
            d->ctrl[act_wL] = 0;
            d->ctrl[act_wR] = 0;
            mj_step(m, d);
        }
        
        double u = -2.0 * ctrl_val;  // total torque
        double ddot_sum = 0;
        prev_thetaDot = d->qvel[4];
        
        for (int step = 0; step < 10; step++) {
            double err_L = hip_target - d->qpos[qpos_hL];
            double err_R = hip_target - d->qpos[qpos_hR];
            int dof_hL = m->jnt_dofadr[jnt_hL];
            int dof_hR = m->jnt_dofadr[jnt_hR];
            d->ctrl[act_hL] = hip_kp * err_L - hip_kd * d->qvel[dof_hL];
            d->ctrl[act_hR] = hip_kp * err_R - hip_kd * d->qvel[dof_hR];
            d->ctrl[act_wL] = ctrl_val;
            d->ctrl[act_wR] = ctrl_val;
            mj_step(m, d);
            
            double thetaDDot = (d->qvel[4] - prev_thetaDot) / dt;
            prev_thetaDot = d->qvel[4];
            if (step >= 2) ddot_sum += thetaDDot;
        }
        
        double avg_ddot = ddot_sum / 8;
        double ddot_corrected = avg_ddot - ddot_baseline;
        double b_est = ddot_corrected / u;
        
        b_sum += b_est;
        b_count++;
    }
    
    result.b = b_sum / b_count;
    
    // === Measure 'a' (gravity term) ===
    // Let robot tip from small offset, measure theta_ddot / theta
    mj_resetDataKeyframe(m, d, 0);
    d->qpos[qpos_hL] = hip_target;
    d->qpos[qpos_hR] = hip_target;
    mj_forward(m, d);
    
    // Brief settle
    for (int i = 0; i < 5; i++) {
        double err_L = hip_target - d->qpos[qpos_hL];
        double err_R = hip_target - d->qpos[qpos_hR];
        int dof_hL = m->jnt_dofadr[jnt_hL];
        int dof_hR = m->jnt_dofadr[jnt_hR];
        d->ctrl[act_hL] = hip_kp * err_L - hip_kd * d->qvel[dof_hL];
        d->ctrl[act_hR] = hip_kp * err_R - hip_kd * d->qvel[dof_hR];
        d->ctrl[act_wL] = 0;
        d->ctrl[act_wR] = 0;
        mj_step(m, d);
    }
    
    // Let it evolve and measure a from theta range 0.5° to 5°
    double a_sum = 0;
    int a_count = 0;
    prev_thetaDot = d->qvel[4];
    
    for (int step = 0; step < 500; step++) {
        double err_L = hip_target - d->qpos[qpos_hL];
        double err_R = hip_target - d->qpos[qpos_hR];
        int dof_hL = m->jnt_dofadr[jnt_hL];
        int dof_hR = m->jnt_dofadr[jnt_hR];
        d->ctrl[act_hL] = hip_kp * err_L - hip_kd * d->qvel[dof_hL];
        d->ctrl[act_hR] = hip_kp * err_R - hip_kd * d->qvel[dof_hR];
        d->ctrl[act_wL] = 0;
        d->ctrl[act_wR] = 0;
        mj_step(m, d);
        
        double theta = get_pitch(d);
        double thetaDot = d->qvel[4];
        double thetaDDot = (thetaDot - prev_thetaDot) / dt;
        prev_thetaDot = thetaDot;
        
        double theta_deg = std::abs(theta) * 57.3;
        if (theta_deg > 0.5 && theta_deg < 5.0) {
            double a_est = thetaDDot / theta;
            a_sum += a_est;
            a_count++;
        }
        
        if (theta_deg > 10.0) break;  // Stop if tipping too far
    }
    
    result.a = (a_count > 0) ? a_sum / a_count : 100.0;  // Default if failed
    
    mj_deleteData(d);
    return result;
}

int main(int argc, char** argv) {
    const char* model_path = "myRobot/scene.xml";
    if (argc > 1) model_path = argv[1];

    char error[1024];
    mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m) { printf("Error: %s\n", error); return 1; }

    printf("Running sysid at 7 hip positions...\n\n");
    
    // Hip positions from the LUT
    double hip_angles[] = {-0.270, -0.1625, -0.055, 0.0525, 0.160, 0.2675, 0.375};
    int n_hips = 7;
    
    std::vector<SysidResult> results;
    
    printf("%10s %10s %10s %10s\n", "hip(rad)", "a", "b", "theta_eq");
    printf("%10s %10s %10s %10s\n", "--------", "--------", "--------", "--------");
    
    for (int i = 0; i < n_hips; i++) {
        SysidResult r = run_sysid_at_hip(m, hip_angles[i]);
        results.push_back(r);
        printf("%10.4f %10.2f %10.2f %10.6f\n", r.hip_rad, r.a, r.b, r.theta_eq);
    }
    
    // Output for Python LQR computation
    printf("\n=== Python arrays ===\n");
    printf("hip_angles = [");
    for (int i = 0; i < n_hips; i++) printf("%.4f%s", results[i].hip_rad, i < n_hips-1 ? ", " : "");
    printf("]\n");
    
    printf("a_values = [");
    for (int i = 0; i < n_hips; i++) printf("%.2f%s", results[i].a, i < n_hips-1 ? ", " : "");
    printf("]\n");
    
    printf("b_values = [");
    for (int i = 0; i < n_hips; i++) printf("%.2f%s", results[i].b, i < n_hips-1 ? ", " : "");
    printf("]\n");
    
    printf("theta_eq = [");
    for (int i = 0; i < n_hips; i++) printf("%.6f%s", results[i].theta_eq, i < n_hips-1 ? ", " : "");
    printf("]\n");
    
    mj_deleteModel(m);
    return 0;
}
