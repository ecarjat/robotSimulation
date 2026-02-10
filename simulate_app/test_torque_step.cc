// Direct torque step test: apply constant ctrl to wheels and observe pitch response
#include <mujoco/mujoco.h>
#include <cstdio>
#include <cmath>
#include <cstdlib>

int main(int argc, char** argv) {
    const char* model_path = "myRobot/scene.xml";
    double ctrl_value = 0.5; // positive ctrl to both wheels
    
    if (argc > 1) model_path = argv[1];
    if (argc > 2) ctrl_value = std::atof(argv[2]);
    
    char error[1024];
    mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m) { printf("Error: %s\n", error); return 1; }
    mjData* d = mj_makeData(m);
    
    if (m->nkey > 0) mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);
    
    // Find actuators
    int act_wL = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_L");
    int act_wR = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_R");
    int act_hL = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");
    int act_hR = mj_name2id(m, mjOBJ_ACTUATOR, "hip_R");
    
    printf("Actuators: wheel_L=%d, wheel_R=%d, hip_L=%d, hip_R=%d\n",
           act_wL, act_wR, act_hL, act_hR);
    
    // Print gear ratios
    if (act_wL >= 0) printf("wheel_L gear=%.3f\n", m->actuator_gear[6*act_wL]);
    if (act_wR >= 0) printf("wheel_R gear=%.3f\n", m->actuator_gear[6*act_wR]);
    
    // Find joints for reading velocities
    int jnt_wL = mj_name2id(m, mjOBJ_JOINT, "wheel_L");
    int jnt_wR = mj_name2id(m, mjOBJ_JOINT, "wheel_R");
    
    printf("\n=== Torque Step Test ===\n");
    printf("Applying ctrl=%.3f to both wheels for 0.5s\n", ctrl_value);
    printf("With gear=-1: tau_joint = gear*ctrl = %.3f Nm\n\n", -1.0 * ctrl_value);
    
    printf("%8s %10s %10s %10s %10s %10s %10s %10s\n",
           "time", "theta_q", "theta_xm", "thetadot", "x", "xdot",
           "wL_vel", "wR_vel");
    
    int torso_id = mj_name2id(m, mjOBJ_BODY, "torso");
    
    for (int step = 0; step < 250; step++) { // 0.5s at dt=0.002
        // Apply constant ctrl to wheels
        if (act_wL >= 0) d->ctrl[act_wL] = ctrl_value;
        if (act_wR >= 0) d->ctrl[act_wR] = ctrl_value;
        // Hold hips at 0
        if (act_hL >= 0) d->ctrl[act_hL] = 0.0;
        if (act_hR >= 0) d->ctrl[act_hR] = 0.0;
        
        mj_step(m, d);
        
        if (step % 25 == 0 || step == 249) {
            // Pitch from quaternion (same as linearization)
            double qw = d->qpos[3], qy = d->qpos[5];
            double theta_q = 2.0 * std::atan2(qy, qw);
            
            // Pitch from xmat (same as bridge)
            double theta_xm = 0.0;
            if (torso_id >= 0) {
                const double* xmat = d->xmat + 9 * torso_id;
                theta_xm = std::atan2(xmat[2], xmat[0]);
            }
            
            // Pitch rate (qvel[1] = rotation about Y)
            double thetadot = d->qvel[1];
            
            // Position and velocity
            double x = d->qpos[0];
            double xdot = d->qvel[3];
            
            // Wheel velocities
            double wL = 0, wR = 0;
            if (jnt_wL >= 0) wL = d->qvel[m->jnt_dofadr[jnt_wL]];
            if (jnt_wR >= 0) wR = d->qvel[m->jnt_dofadr[jnt_wR]];
            
            printf("%8.3f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f\n",
                   d->time, theta_q * 57.3, theta_xm * 57.3, thetadot,
                   x, xdot, wL, wR);
        }
    }
    
    printf("\n=== Summary ===\n");
    double qw = d->qpos[3], qy = d->qpos[5];
    double theta_final = 2.0 * std::atan2(qy, qw) * 57.3;
    printf("ctrl=%.3f → theta=%.3f° after 0.5s\n", ctrl_value, theta_final);
    if (theta_final > 0) {
        printf("Robot pitched FORWARD (positive theta)\n");
        printf("→ Positive ctrl causes FORWARD pitch\n");
        printf("→ To correct forward pitch, need NEGATIVE ctrl\n");
    } else {
        printf("Robot pitched BACKWARD (negative theta)\n");
        printf("→ Positive ctrl causes BACKWARD pitch\n");
        printf("→ To correct forward pitch, need POSITIVE ctrl\n");
    }
    
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
