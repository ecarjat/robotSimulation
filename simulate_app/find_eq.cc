// Find equilibrium: measure u needed to hold theta=0, and theta where u=0
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

    double hip_kp = 100.0, hip_kd = 5.0;
    double hip_angles[] = {-0.270, -0.1625, -0.055, 0.0525, 0.160, 0.2675, 0.375};
    int n_hips = 7;
    
    printf("Finding equilibrium for each hip position...\n\n");
    printf("Method: Use very high pitch gain to hold theta near target,\n");
    printf("        measure average torque needed, then find theta where u→0\n\n");
    
    printf("%10s %12s %12s\n", "hip(rad)", "theta_eq(rad)", "theta_eq(°)");
    printf("%10s %12s %12s\n", "--------", "------------", "-----------");
    
    double theta_eq_values[7];
    
    for (int h = 0; h < n_hips; h++) {
        double hip_target = hip_angles[h];
        
        // Binary search for theta where average torque is zero
        double theta_lo = -0.15, theta_hi = 0.15;  // ±8.6°
        double theta_mid = 0;
        
        for (int iter = 0; iter < 20; iter++) {
            theta_mid = (theta_lo + theta_hi) / 2;
            
            mjData* d = mj_makeData(m);
            mj_resetDataKeyframe(m, d, 0);
            d->qpos[qpos_hL] = hip_target;
            d->qpos[qpos_hR] = hip_target;
            mj_forward(m, d);
            
            // Very strong pitch controller to hold at theta_mid
            double K_pitch = -2000.0;
            double K_rate = -100.0;
            
            double u_sum = 0;
            int samples = 0;
            
            for (int step = 0; step < 2000; step++) {  // 4 seconds
                // Hip hold
                double err_hL = hip_target - d->qpos[qpos_hL];
                double err_hR = hip_target - d->qpos[qpos_hR];
                d->ctrl[act_hL] = hip_kp * err_hL - hip_kd * d->qvel[dof_hL];
                d->ctrl[act_hR] = hip_kp * err_hR - hip_kd * d->qvel[dof_hR];
                
                // Strong pitch hold at theta_mid
                double theta = get_pitch(d);
                double thetaDot = d->qvel[4];
                double u = K_pitch * (theta - theta_mid) + K_rate * thetaDot;
                u = std::max(-10.0, std::min(10.0, u));  // Higher limit for measurement
                
                d->ctrl[act_wL] = -u / 2.0;
                d->ctrl[act_wR] = -u / 2.0;
                
                mj_step(m, d);
                
                // Collect after settling (last 2s)
                if (step >= 1000) {
                    u_sum += u;
                    samples++;
                }
            }
            
            double u_avg = u_sum / samples;
            
            // If u_avg > 0, we're pushing too hard forward, need more backward theta
            // If u_avg < 0, we're pushing backward, need more forward theta
            if (u_avg > 0.01) {
                theta_hi = theta_mid;  // Need smaller (more negative) theta
            } else if (u_avg < -0.01) {
                theta_lo = theta_mid;  // Need larger (more positive) theta
            } else {
                break;  // Found equilibrium
            }
            
            mj_deleteData(d);
        }
        
        theta_eq_values[h] = theta_mid;
        printf("%10.4f %12.6f %12.4f\n", hip_target, theta_mid, theta_mid * 57.3);
    }
    
    // Output CSV
    printf("\n=== Updated lqr_lut.csv ===\n");
    printf("hip,K0,K1,K2,K3,theta_eq,u_eq\n");
    for (int h = 0; h < n_hips; h++) {
        printf("%.6f,-5.0,-7.5,-400.0,-24.0,%.6f,0.0\n", 
               hip_angles[h], theta_eq_values[h]);
    }
    
    mj_deleteModel(m);
    return 0;
}
