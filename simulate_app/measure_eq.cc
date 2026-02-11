// Measure equilibrium pitch angle at each hip position
// Run with stabilizing controller, let settle, measure average pitch

#include <mujoco/mujoco.h>
#include <cstdio>
#include <cmath>

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
    int jnt_hL = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int jnt_hR = mj_name2id(m, mjOBJ_JOINT, "hip_R");
    
    int qpos_hL = m->jnt_qposadr[jnt_hL];
    int qpos_hR = m->jnt_qposadr[jnt_hR];
    int dof_hL = m->jnt_dofadr[jnt_hL];
    int dof_hR = m->jnt_dofadr[jnt_hR];
    
    double dt = m->opt.timestep;
    
    // Strong stabilizing gains (from tuned values)
    double K0 = -5.0, K1 = -7.5, K2 = -400.0, K3 = -24.0;
    double hip_kp = 100.0, hip_kd = 5.0;
    double u_limit = 5.0;
    
    double hip_angles[] = {-0.270, -0.1625, -0.055, 0.0525, 0.160, 0.2675, 0.375};
    int n_hips = 7;
    
    printf("Measuring equilibrium pitch at each hip position...\n");
    printf("(Running 10s stabilization, averaging last 5s)\n\n");
    
    printf("%10s %12s %12s %12s\n", "hip(rad)", "theta_eq(rad)", "theta_eq(°)", "u_eq(Nm)");
    printf("%10s %12s %12s %12s\n", "--------", "------------", "-----------", "--------");
    
    for (int h = 0; h < n_hips; h++) {
        double hip_target = hip_angles[h];
        
        mjData* d = mj_makeData(m);
        mj_resetDataKeyframe(m, d, 0);
        d->qpos[qpos_hL] = hip_target;
        d->qpos[qpos_hR] = hip_target;
        mj_forward(m, d);
        
        double theta_sum = 0, u_sum = 0;
        int sample_count = 0;
        double x_ref = 0;  // Position reference
        
        // Run for 10 seconds
        for (int step = 0; step < 5000; step++) {
            double t = step * dt;
            
            // Hip hold
            double err_hL = hip_target - d->qpos[qpos_hL];
            double err_hR = hip_target - d->qpos[qpos_hR];
            d->ctrl[act_hL] = hip_kp * err_hL - hip_kd * d->qvel[dof_hL];
            d->ctrl[act_hR] = hip_kp * err_hR - hip_kd * d->qvel[dof_hR];
            
            // Balance controller (with theta_eq = 0 initially)
            double x = d->qpos[0];
            double v = d->qvel[0];
            double theta = get_pitch(d);
            double thetaDot = d->qvel[4];
            
            double x_err = x - x_ref;
            double v_err = v;
            
            // Position clamp
            double u_pos = -K0 * x_err;
            double pos_limit = u_limit * 0.4;
            u_pos = std::max(-pos_limit, std::min(pos_limit, u_pos));
            
            double u = u_pos - K1 * v_err - K2 * theta - K3 * thetaDot;
            u = std::max(-u_limit, std::min(u_limit, u));
            
            d->ctrl[act_wL] = -u / 2.0;  // gear = -1
            d->ctrl[act_wR] = -u / 2.0;
            
            mj_step(m, d);
            
            // Slowly move position reference to track robot (prevents drift accumulation)
            x_ref += 0.001 * (x - x_ref);
            
            // Collect samples from last 5 seconds
            if (t >= 5.0) {
                theta_sum += theta;
                u_sum += u;
                sample_count++;
            }
        }
        
        double theta_eq = theta_sum / sample_count;
        double u_eq = u_sum / sample_count;
        
        printf("%10.4f %12.6f %12.4f %12.4f\n", 
               hip_target, theta_eq, theta_eq * 57.3, u_eq);
        
        mj_deleteData(d);
    }
    
    printf("\n=== CSV for lqr_lut.csv (theta_eq, u_eq columns) ===\n");
    
    // Re-run to get clean output
    printf("hip,K0,K1,K2,K3,theta_eq,u_eq\n");
    for (int h = 0; h < n_hips; h++) {
        double hip_target = hip_angles[h];
        
        mjData* d = mj_makeData(m);
        mj_resetDataKeyframe(m, d, 0);
        d->qpos[qpos_hL] = hip_target;
        d->qpos[qpos_hR] = hip_target;
        mj_forward(m, d);
        
        double theta_sum = 0, u_sum = 0;
        int sample_count = 0;
        double x_ref = 0;
        
        for (int step = 0; step < 5000; step++) {
            double t = step * dt;
            
            double err_hL = hip_target - d->qpos[qpos_hL];
            double err_hR = hip_target - d->qpos[qpos_hR];
            d->ctrl[act_hL] = hip_kp * err_hL - hip_kd * d->qvel[dof_hL];
            d->ctrl[act_hR] = hip_kp * err_hR - hip_kd * d->qvel[dof_hR];
            
            double x = d->qpos[0];
            double v = d->qvel[0];
            double theta = get_pitch(d);
            double thetaDot = d->qvel[4];
            
            double x_err = x - x_ref;
            double u_pos = -K0 * x_err;
            double pos_limit = u_limit * 0.4;
            u_pos = std::max(-pos_limit, std::min(pos_limit, u_pos));
            
            double u = u_pos - K1 * v - K2 * theta - K3 * thetaDot;
            u = std::max(-u_limit, std::min(u_limit, u));
            
            d->ctrl[act_wL] = -u / 2.0;
            d->ctrl[act_wR] = -u / 2.0;
            
            mj_step(m, d);
            x_ref += 0.001 * (x - x_ref);
            
            if (t >= 5.0) {
                theta_sum += theta;
                u_sum += u;
                sample_count++;
            }
        }
        
        double theta_eq = theta_sum / sample_count;
        double u_eq = u_sum / sample_count;
        
        printf("%.6f,%.1f,%.1f,%.1f,%.1f,%.6f,%.6f\n",
               hip_target, K0, K1, K2, K3, theta_eq, u_eq);
        
        mj_deleteData(d);
    }
    
    mj_deleteModel(m);
    return 0;
}
