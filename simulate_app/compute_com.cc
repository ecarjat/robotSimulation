#include <mujoco/mujoco.h>
#include <cstdio>
#include <cmath>

// Global state for control callback
struct {
    double saved_qpos[7];
    double hip_target;
    int act_hip_L;
    int act_hip_R;
    int act_wheel_L;
    int act_wheel_R;
} g_ctrl;

void control_callback(const mjModel* m, mjData* d) {
    // Fix freejoint to prevent drift
    for (int i = 0; i < 7; i++) {
        d->qpos[i] = g_ctrl.saved_qpos[i];
    }
    for (int i = 0; i < 6; i++) {
        d->qvel[i] = 0.0;
    }

    // Set hip actuator controls
    if (g_ctrl.act_hip_L >= 0) d->ctrl[g_ctrl.act_hip_L] = g_ctrl.hip_target * 5.0;
    if (g_ctrl.act_hip_R >= 0) d->ctrl[g_ctrl.act_hip_R] = g_ctrl.hip_target * 5.0;

    // Zero wheel torques
    if (g_ctrl.act_wheel_L >= 0) d->ctrl[g_ctrl.act_wheel_L] = 0.0;
    if (g_ctrl.act_wheel_R >= 0) d->ctrl[g_ctrl.act_wheel_R] = 0.0;
}

int main(int argc, char** argv) {
    const char* model_path = (argc > 1) ? argv[1] : "myRobot/scene.xml";

    char error[1024];
    mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m) {
        printf("Error loading model: %s\n", error);
        return 1;
    }

    mjData* d = mj_makeData(m);

    int jnt_hip_L = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int jnt_hip_R = mj_name2id(m, mjOBJ_JOINT, "hip_R");
    int jnt_passive1 = mj_name2id(m, mjOBJ_JOINT, "passive1");
    int jnt_passive2 = mj_name2id(m, mjOBJ_JOINT, "passive2");
    int jnt_passive3 = mj_name2id(m, mjOBJ_JOINT, "passive3");
    int jnt_passive4 = mj_name2id(m, mjOBJ_JOINT, "passive4");
    int body_torso = mj_name2id(m, mjOBJ_BODY, "torso");
    int body_wheel_L = mj_name2id(m, mjOBJ_BODY, "wheel_geom");
    int body_wheel_R = mj_name2id(m, mjOBJ_BODY, "wheel_geom_2");

    if (jnt_hip_L < 0 || jnt_hip_R < 0) {
        printf("Error: hip joints not found\n");
        return 1;
    }

    int qpos_hip_L = m->jnt_qposadr[jnt_hip_L];
    int qpos_hip_R = m->jnt_qposadr[jnt_hip_R];

    double hip_angles[] = {-0.270, -0.1625, -0.055, 0.0525, 0.160, 0.2675, 0.375};
    int n_hips = 7;

    // Get total robot mass using MuJoCo function
    double total_robot_mass = mj_getTotalmass(m);

    // Get wheel masses
    double wheel_L_mass = (body_wheel_L >= 0) ? m->body_mass[body_wheel_L] : 0.0;
    double wheel_R_mass = (body_wheel_R >= 0) ? m->body_mass[body_wheel_R] : 0.0;
    double total_wheel_mass = wheel_L_mass + wheel_R_mass;
    double upper_body_mass = total_robot_mass - total_wheel_mass;

    printf("Computing COM position and expected equilibrium for each hip angle\n\n");
    printf("Mass Summary (using MuJoCo functions):\n");
    printf("  Total robot mass:     %.4f kg (mj_getTotalmass)\n", total_robot_mass);
    printf("  Wheel L mass:         %.4f kg (m->body_mass[wheel_geom])\n", wheel_L_mass);
    printf("  Wheel R mass:         %.4f kg (m->body_mass[wheel_geom_2])\n", wheel_R_mass);
    printf("  Upper body mass:      %.4f kg (total - wheels)\n\n", upper_body_mass);

    // Check joint positions at neutral hip angle to show joint layout
    mj_resetDataKeyframe(m, d, 0);
    d->qpos[qpos_hip_L] = 0.0;  // neutral hip
    d->qpos[qpos_hip_R] = 0.0;
    mj_forward(m, d);
    double hip_x_neutral = d->xanchor[3*jnt_hip_L + 0];
    double passive1_x_neutral = d->xanchor[3*jnt_passive1 + 0];
    printf("Joint Positions (at hip=0, using d->xanchor):\n");
    printf("  Hip_L x-position:     %.4f m\n", hip_x_neutral);
    printf("  Passive1 x-position:  %.4f m (knee joint)\n", passive1_x_neutral);
    printf("  Passive1 relative:    %.4f m (%s of hip)\n\n",
           passive1_x_neutral - hip_x_neutral,
           (passive1_x_neutral > hip_x_neutral) ? "AHEAD" : "BEHIND");

    printf("Hip (rad) | Hip (°) | ||hip-wheel|| | Passive1 (°) | Passive2 (°) | Constr viol | θ_eq (°) | Status\n");
    printf("----------|---------|--------------|--------------|--------------|-------------|----------|--------\n");

    // Get hip actuators
    g_ctrl.act_hip_L = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");
    g_ctrl.act_hip_R = mj_name2id(m, mjOBJ_ACTUATOR, "hip_R");
    g_ctrl.act_wheel_L = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_L");
    g_ctrl.act_wheel_R = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_R");

    // Keep gravity enabled for ground contact, but use control callback to fix base
    // Install control callback
    mjcb_control = control_callback;

    for (int i = 0; i < n_hips; i++) {
        double hip = hip_angles[i];

        // Reset to keyframe
        mj_resetDataKeyframe(m, d, 0);

        // Save initial freejoint state (upright position from keyframe)
        for (int j = 0; j < 7; j++) {
            g_ctrl.saved_qpos[j] = d->qpos[j];
        }

        // Set hip target
        g_ctrl.hip_target = hip;

        // Wait for system to settle (like pressing I/K and waiting in simulate_mc)
        // Check that hip joints reached target and velocities are near zero
        bool settled = false;
        int max_steps = 5000;
        for (int step = 0; step < max_steps && !settled; step++) {
            mj_step(m, d);

            // Check every 10 steps to avoid excessive checking
            if (step % 10 == 0 && step > 100) {
                // Check if hip joints are at target (within 0.01 rad = 0.57°)
                double hip_L_pos = d->qpos[qpos_hip_L];
                double hip_R_pos = d->qpos[qpos_hip_R];
                bool hips_at_target = (fabs(hip_L_pos - hip) < 0.01) &&
                                     (fabs(hip_R_pos - hip) < 0.01);

                // Check if system has low velocities (settled)
                // Exclude freejoint velocities since we're fixing them to zero
                double max_vel = 0.0;
                for (int v = 6; v < m->nv; v++) {  // Skip first 6 DOFs (freejoint)
                    if (fabs(d->qvel[v]) > max_vel) {
                        max_vel = fabs(d->qvel[v]);
                    }
                }
                bool velocities_low = max_vel < 0.05;  // Relaxed threshold (rad/s or m/s)

                if (hips_at_target && velocities_low) {
                    settled = true;
                    // printf("  Settled at step %d (max_vel=%.4f)\n", step, max_vel);
                }
            }
        }

        if (!settled) {
            printf("WARNING: System did not settle for hip=%.4f after %d steps\n", hip, max_steps);
        }

        // Check constraint violations (equality constraints are first in efc arrays)
        double max_constraint_violation = 0.0;
        int neq_constraints = (d->nefc < m->neq * 6) ? d->nefc : m->neq * 6;
        for (int c = 0; c < neq_constraints; c++) {
            double viol = fabs(d->efc_pos[c]);
            if (viol > max_constraint_violation) {
                max_constraint_violation = viol;
            }
        }

        // GEOMETRY VERIFICATION CHECKS
        // Expected ||hip-wheel|| distances from simulate GUI measurements
        const double expected_hip_wheel_dist[] = {0.2824, 0.3660, 0.4440, 0.5120, 0.5751, 0.6346, 0.6866};

        // Compute ||hip_L - wheel_L|| distance
        double hip_pos[3] = {
            d->xanchor[3*jnt_hip_L + 0],
            d->xanchor[3*jnt_hip_L + 1],
            d->xanchor[3*jnt_hip_L + 2]
        };
        int body_wheel_L_id = mj_name2id(m, mjOBJ_BODY, "wheel_geom");
        double wheel_center[3] = {
            d->xpos[3*body_wheel_L_id + 0],
            d->xpos[3*body_wheel_L_id + 1],
            d->xpos[3*body_wheel_L_id + 2]
        };
        double dx = wheel_center[0] - hip_pos[0];
        double dy = wheel_center[1] - hip_pos[1];
        double dz = wheel_center[2] - hip_pos[2];
        double hip_wheel_dist = sqrt(dx*dx + dy*dy + dz*dz);

        // Verify geometry
        bool geometry_ok = true;
        double dist_error = fabs(hip_wheel_dist - expected_hip_wheel_dist[i]);
        if (dist_error > 0.01) {  // More than 10mm error
            printf("WARNING: ||hip-wheel|| mismatch at hip=%.4f: got %.4f, expected %.4f (error: %.1fmm)\n",
                   hip, hip_wheel_dist, expected_hip_wheel_dist[i], dist_error * 1000);
            geometry_ok = false;
        }
        if (max_constraint_violation > 0.005) {  // More than 5mm constraint violation
            printf("WARNING: Large constraint violation at hip=%.4f: %.1fmm\n",
                   hip, max_constraint_violation * 1000);
            geometry_ok = false;
        }

        // Get upper body COM (excluding wheels)
        double com[3] = {0, 0, 0};
        double total_mass = 0.0;
        // Compute COM manually from body positions and masses, EXCLUDING wheels
        for (int b = 0; b < m->nbody; b++) {
            // Skip wheel bodies
            if (b == body_wheel_L || b == body_wheel_R) continue;

            double body_mass = m->body_mass[b];
            if (body_mass > 0) {
                com[0] += d->xipos[3*b + 0] * body_mass;
                com[1] += d->xipos[3*b + 1] * body_mass;
                com[2] += d->xipos[3*b + 2] * body_mass;
                total_mass += body_mass;
            }
        }
        com[0] /= total_mass;
        com[1] /= total_mass;
        com[2] /= total_mass;

        // Get hip joint position from MuJoCo's xanchor (average of left and right hip)
        double hip_x = (d->xanchor[3*jnt_hip_L + 0] + d->xanchor[3*jnt_hip_R + 0]) / 2.0;
        double hip_z = (d->xanchor[3*jnt_hip_L + 2] + d->xanchor[3*jnt_hip_R + 2]) / 2.0;

        // COM position relative to hip joint
        double com_rel_hip_x = com[0] - hip_x;

        // Get wheel contact point (wheel body position - radius)
        int body_wheel = mj_name2id(m, mjOBJ_BODY, "wheel_geom");
        double wheel_x = d->xpos[3*body_wheel + 0];
        double wheel_z = d->xpos[3*body_wheel + 2] - 0.08547;  // subtract radius

        // Wheel position relative to hip joint
        double wheel_rel_hip_x = wheel_x - hip_x;
        double wheel_rel_hip_z = wheel_z - hip_z;

        // COM position relative to wheel contact (INVERTED PENDULUM PIVOT)
        double com_rel_wheel_x = com[0] - wheel_x;
        double com_rel_wheel_z = com[2] - wheel_z;

        // Expected equilibrium angle from wheel contact reference frame
        // Positive com_rel_wheel_x (COM ahead of wheel) requires negative theta (lean back)
        double theta_eq_expected = -atan2(com_rel_wheel_x, com_rel_wheel_z);

        // Get passive joint angles
        double passive1_angle = d->qpos[m->jnt_qposadr[jnt_passive1]];
        double passive2_angle = d->qpos[m->jnt_qposadr[jnt_passive2]];

        printf("%9.4f | %7.2f | %12.4f | %12.2f | %12.2f | %11.1f | %8.2f | %s\n",
               hip, hip * 57.3, hip_wheel_dist,
               passive1_angle * 57.3, passive2_angle * 57.3,
               max_constraint_violation * 1000, theta_eq_expected * 57.3,
               geometry_ok ? "OK" : "WARN");
    }

    printf("\nGeometry Verification:\n");
    printf("- ||hip-wheel|| distances are compared against known good values from simulate GUI\n");
    printf("- Constraint violations should be < 5mm for correct linkage geometry\n");
    printf("- Status 'OK' means geometry matches expected values (< 10mm error)\n");
    printf("- Status 'WARN' indicates potential geometry mismatch or solver issue\n");

    printf("\nNotes:\n");
    printf("- COM is computed for UPPER BODY ONLY (excludes wheels) using MuJoCo's d->xipos\n");
    printf("- θ_eq = -atan2(COM_rel_wheel_x, dZ_wheel) - equilibrium from inverted pendulum pivot\n");

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
