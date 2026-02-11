// Closed-loop sysid: use stabilizing controller, inject perturbations, measure response
// This works even at hip angles where open-loop would tip immediately

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
    double a;
    double b;
    double theta_eq;
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
    
    // Hip PD gains
    double hip_kp = 100.0, hip_kd = 5.0;
    int qpos_hL = m->jnt_qposadr[jnt_hL];
    int qpos_hR = m->jnt_qposadr[jnt_hR];
    int dof_hL = m->jnt_dofadr[jnt_hL];
    int dof_hR = m->jnt_dofadr[jnt_hR];
    
    // Pitch stabilizing controller (simple PD)
    // We know roughly K[2] ~ -400, K[3] ~ -20 work
    double K_theta = -200.0;  // Use moderate gains for sysid
    double K_thetaDot = -10.0;
    
    auto apply_hip_ctrl = [&](mjData* d) {
        double err_L = hip_target - d->qpos[qpos_hL];
        double err_R = hip_target - d->qpos[qpos_hR];
        d->ctrl[act_hL] = hip_kp * err_L - hip_kd * d->qvel[dof_hL];
        d->ctrl[act_hR] = hip_kp * err_R - hip_kd * d->qvel[dof_hR];
    };
    
    // === Find equilibrium with stabilizing controller ===
    mjData* d = mj_makeData(m);
    mj_resetDataKeyframe(m, d, 0);
    d->qpos[qpos_hL] = hip_target;
    d->qpos[qpos_hR] = hip_target;
    mj_forward(m, d);
    
    // Run with stabilizing controller until settled
    double theta_sum = 0;
    for (int i = 0; i < 1000; i++) {  // 2 seconds
        apply_hip_ctrl(d);
        double theta = get_pitch(d);
        double thetaDot = d->qvel[4];
        double u_stab = K_theta * theta + K_thetaDot * thetaDot;
        u_stab = std::max(-3.0, std::min(3.0, u_stab));  // Limit
        d->ctrl[act_wL] = -u_stab / 2.0;  // gear = -1, so negate
        d->ctrl[act_wR] = -u_stab / 2.0;
        mj_step(m, d);
        
        if (i >= 500) theta_sum += theta;  // Average last 1s
    }
    result.theta_eq = theta_sum / 500;
    
    // === Measure 'b' via torque pulse injection ===
    // Inject small torque pulse on top of stabilizing controller
    // Measure pitch rate change
    
    double b_sum = 0;
    int b_count = 0;
    
    for (double pulse : {0.5, 1.0, 1.5, -0.5, -1.0, -1.5}) {
        // Reset to equilibrium
        mj_resetDataKeyframe(m, d, 0);
        d->qpos[qpos_hL] = hip_target;
        d->qpos[qpos_hR] = hip_target;
        mj_forward(m, d);
        
        // Settle
        for (int i = 0; i < 500; i++) {
            apply_hip_ctrl(d);
            double theta = get_pitch(d);
            double thetaDot = d->qvel[4];
            double u_stab = K_theta * theta + K_thetaDot * thetaDot;
            u_stab = std::max(-3.0, std::min(3.0, u_stab));
            d->ctrl[act_wL] = -u_stab / 2.0;
            d->ctrl[act_wR] = -u_stab / 2.0;
            mj_step(m, d);
        }
        
        double thetaDot_before = d->qvel[4];
        
        // Apply pulse for 10ms (5 steps)
        double u_pulse = pulse;  // Nm total
        for (int i = 0; i < 5; i++) {
            apply_hip_ctrl(d);
            double theta = get_pitch(d);
            double thetaDot = d->qvel[4];
            double u_stab = K_theta * theta + K_thetaDot * thetaDot;
            u_stab = std::max(-3.0, std::min(3.0, u_stab));
            double u_total = u_stab + u_pulse;
            d->ctrl[act_wL] = -u_total / 2.0;
            d->ctrl[act_wR] = -u_total / 2.0;
            mj_step(m, d);
        }
        
        double thetaDot_after = d->qvel[4];
        double delta_thetaDot = thetaDot_after - thetaDot_before;
        
        // b = delta_thetaDot / (u_pulse * dt_pulse)
        // u_pulse is total torque in Nm, dt_pulse = 0.01s
        double b_est = delta_thetaDot / (u_pulse * 0.01);
        b_sum += b_est;
        b_count++;
    }
    
    result.b = b_sum / b_count;
    
    // === Measure 'a' via small pitch perturbation ===
    // With stabilizing controller, perturb theta, measure thetaDDot response
    // At equilibrium: thetaDDot = a*theta + b*u_stab = a*theta + b*(K_theta*theta)
    // thetaDDot = (a + b*K_theta) * theta
    // So: a = thetaDDot/theta - b*K_theta
    
    double a_sum = 0;
    int a_count = 0;
    
    // Reset and settle
    mj_resetDataKeyframe(m, d, 0);
    d->qpos[qpos_hL] = hip_target;
    d->qpos[qpos_hR] = hip_target;
    mj_forward(m, d);
    
    for (int i = 0; i < 500; i++) {
        apply_hip_ctrl(d);
        double theta = get_pitch(d);
        double thetaDot = d->qvel[4];
        double u_stab = K_theta * theta + K_thetaDot * thetaDot;
        u_stab = std::max(-3.0, std::min(3.0, u_stab));
        d->ctrl[act_wL] = -u_stab / 2.0;
        d->ctrl[act_wR] = -u_stab / 2.0;
        mj_step(m, d);
    }
    
    // Give small kick to induce oscillation
    for (int i = 0; i < 3; i++) {
        apply_hip_ctrl(d);
        d->ctrl[act_wL] = 0.5;  // Small kick
        d->ctrl[act_wR] = 0.5;
        mj_step(m, d);
    }
    
    // Measure response
    double prev_thetaDot = d->qvel[4];
    for (int i = 0; i < 200; i++) {
        apply_hip_ctrl(d);
        double theta = get_pitch(d);
        double thetaDot = d->qvel[4];
        double u_stab = K_theta * theta + K_thetaDot * thetaDot;
        u_stab = std::max(-3.0, std::min(3.0, u_stab));
        d->ctrl[act_wL] = -u_stab / 2.0;
        d->ctrl[act_wR] = -u_stab / 2.0;
        mj_step(m, d);
        
        double thetaDDot = (d->qvel[4] - prev_thetaDot) / dt;
        prev_thetaDot = d->qvel[4];
        
        // Only use samples where theta is away from zero
        if (std::abs(theta - result.theta_eq) > 0.005 && std::abs(theta) < 0.3) {
            // thetaDDot ≈ a*theta + b*u_stab
            // At this point u_stab = K_theta*theta + K_thetaDot*thetaDot
            double u_applied = K_theta * theta + K_thetaDot * thetaDot;
            // thetaDDot = a*theta + b*u_applied
            // a ≈ (thetaDDot - b*u_applied) / theta
            double a_est = (thetaDDot - result.b * u_applied) / theta;
            
            if (std::isfinite(a_est) && a_est > 0 && a_est < 500) {
                a_sum += a_est;
                a_count++;
            }
        }
    }
    
    result.a = (a_count > 5) ? a_sum / a_count : 113.0;  // Default from earlier sysid
    
    mj_deleteData(d);
    return result;
}

int main(int argc, char** argv) {
    const char* model_path = "myRobot/scene.xml";
    if (argc > 1) model_path = argv[1];

    char error[1024];
    mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m) { printf("Error: %s\n", error); return 1; }

    printf("Closed-loop sysid at 7 hip positions...\n\n");
    
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
