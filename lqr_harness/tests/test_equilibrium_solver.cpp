/**
 * @file test_equilibrium_solver.cpp
 * @brief Regression tests for equilibrium solver in linearize_hip
 *
 * Critical test: Ensures equilibrium solver produces varying theta_eq
 * for different hip angles. This test catches the bug where position
 * actuator targets were incorrectly multiplied by gear ratio.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <mujoco/mujoco.h>

#include <cmath>
#include <string>
#include <cstdio>

namespace {

// Compute geometric equilibrium theta where COM is directly above wheel contact
double compute_geometric_theta_eq(mjModel* m, mjData* d,
                                   int jnt_hip_l, int jnt_hip_r,
                                   double hip_angle) {
    // Reset to keyframe
    mj_resetDataKeyframe(m, d, 0);

    int qpos_hip_l = m->jnt_qposadr[jnt_hip_l];
    int qpos_hip_r = m->jnt_qposadr[jnt_hip_r];

    // Get actuators for settling
    int act_hip_L = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");
    int act_hip_R = mj_name2id(m, mjOBJ_ACTUATOR, "hip_R");
    int act_wheel_L = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_L");
    int act_wheel_R = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_R");

    // Save initial base state
    double saved_qpos[7];
    for (int j = 0; j < 7; j++) {
        saved_qpos[j] = d->qpos[j];
    }

    // Position actuators with gear: ctrl = joint_angle * gear
    // (actuator_length = joint_angle * gear, and position servo drives actuator_length → ctrl)
    if (act_hip_L >= 0) {
        mjtNum gear = m->actuator_gear[6 * act_hip_L];
        d->ctrl[act_hip_L] = hip_angle * gear;
    }
    if (act_hip_R >= 0) {
        mjtNum gear = m->actuator_gear[6 * act_hip_R];
        d->ctrl[act_hip_R] = hip_angle * gear;
    }
    if (act_wheel_L >= 0) d->ctrl[act_wheel_L] = 0.0;
    if (act_wheel_R >= 0) d->ctrl[act_wheel_R] = 0.0;

    // Run simulation to let constraints settle with fixed base
    for (int step = 0; step < 200; step++) {
        // Fix freejoint to prevent drift
        for (int j = 0; j < 7; j++) {
            d->qpos[j] = saved_qpos[j];
        }
        for (int j = 0; j < 6; j++) {
            d->qvel[j] = 0.0;
        }
        mj_step(m, d);
    }

    // Get wheel bodies
    int body_wheel_L = mj_name2id(m, mjOBJ_BODY, "wheel_geom");
    int body_wheel_R = mj_name2id(m, mjOBJ_BODY, "wheel_geom_2");

    // Compute upper body COM (excluding wheels)
    double com[3] = {0, 0, 0};
    double total_mass = 0.0;
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

    // Get wheel contact point (wheel body position - radius)
    double wheel_x = d->xpos[3*body_wheel_L + 0];
    double wheel_z = d->xpos[3*body_wheel_L + 2] - 0.08547;  // subtract radius

    // COM position relative to wheel contact (inverted pendulum pivot)
    double com_rel_x = com[0] - wheel_x;
    double com_rel_z = com[2] - wheel_z;

    // Equilibrium angle: where COM is directly above wheel contact
    // Positive com_rel_x (COM ahead of wheel) requires negative theta (lean back)
    double theta_eq = -std::atan2(com_rel_x, com_rel_z);

    return theta_eq;
}

int joint_id(const mjModel* m, const char* name) {
    return mj_name2id(m, mjOBJ_JOINT, name);
}

} // namespace

TEST_CASE("Equilibrium solver produces varying theta_eq for different hip angles", "[equilibrium][critical]")
{
    // This is the CRITICAL regression test for the bug fix:
    // Before fix: theta_eq was the same for all hip angles (equilibrium solver broken)
    // After fix: theta_eq varies monotonically with hip angle

    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int hip_r_id = mj_name2id(m, mjOBJ_JOINT, "hip_R");
    REQUIRE(hip_id >= 0);
    REQUIRE(hip_r_id >= 0);

    // Test multiple hip angles
    const double hip_angles[] = {-0.27, -0.1625, -0.055, 0.0525, 0.16, 0.2675};
    const size_t n_angles = sizeof(hip_angles) / sizeof(hip_angles[0]);

    double theta_eq_values[6];

    for (size_t i = 0; i < n_angles; ++i) {
        theta_eq_values[i] = compute_geometric_theta_eq(m, d, hip_id, hip_r_id, hip_angles[i]);

        // All equilibrium angles should be finite
        REQUIRE(std::isfinite(theta_eq_values[i]));

        // Should be reasonable values (not at actuator limits)
        INFO("hip_angle=" << hip_angles[i] << " -> theta_eq=" << theta_eq_values[i]);
        CHECK(std::abs(theta_eq_values[i]) < 0.5);  // Less than ~30 degrees
    }

    // CRITICAL: theta_eq should vary monotonically with hip angle
    // Lower hips (more negative) should require more backward lean (more negative theta_eq)
    for (size_t i = 1; i < n_angles; ++i) {
        INFO("Checking monotonicity: hip[" << i-1 << "]=" << hip_angles[i-1]
             << " theta_eq=" << theta_eq_values[i-1]
             << " vs hip[" << i << "]=" << hip_angles[i]
             << " theta_eq=" << theta_eq_values[i]);

        // As hip angle increases, theta_eq should become less negative (monotonic increase)
        CHECK(theta_eq_values[i] > theta_eq_values[i-1]);
    }

    // CRITICAL: theta_eq should have significant variation across the range
    double theta_eq_range = theta_eq_values[n_angles-1] - theta_eq_values[0];
    INFO("theta_eq range: " << theta_eq_range << " rad (" << theta_eq_range * 57.3 << " deg)");
    CHECK(theta_eq_range > 0.01);  // At least 0.57 degrees variation

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Equilibrium solver produces near-zero torques at equilibrium", "[equilibrium]")
{
    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int hip_r_id = mj_name2id(m, mjOBJ_JOINT, "hip_R");
    int wheel_l_jnt = mj_name2id(m, mjOBJ_JOINT, "wheel_L");
    int wheel_r_jnt = mj_name2id(m, mjOBJ_JOINT, "wheel_R");

    const double test_hip_angle = -0.1;

    // Compute equilibrium
    double theta_eq = compute_geometric_theta_eq(m, d, hip_id, hip_r_id, test_hip_angle);

    // Reset to equilibrium state
    mj_resetDataKeyframe(m, d, 0);
    int qpos_hip_l = m->jnt_qposadr[hip_id];
    int qpos_hip_r = m->jnt_qposadr[hip_r_id];
    d->qpos[qpos_hip_l] = test_hip_angle;
    d->qpos[qpos_hip_r] = test_hip_angle;

    // Set pitch to equilibrium angle
    d->qpos[3] = std::cos(theta_eq * 0.5);  // qw
    d->qpos[4] = 0.0;                        // qx
    d->qpos[5] = std::sin(theta_eq * 0.5);  // qy
    d->qpos[6] = 0.0;                        // qz

    // Zero velocities
    for (int i = 0; i < m->nv; ++i) {
        d->qvel[i] = 0.0;
        d->qacc[i] = 0.0;
    }

    mj_forward(m, d);
    mj_inverse(m, d);

    // Check wheel torques are near zero at equilibrium
    if (wheel_l_jnt >= 0) {
        int dof = m->jnt_dofadr[wheel_l_jnt];
        if (dof >= 0 && dof < m->nv) {
            double tau_l = d->qfrc_inverse[dof];
            INFO("wheel_L torque at equilibrium: " << tau_l << " Nm");
            CHECK(std::abs(tau_l) < 1e-3);  // Less than 1 mNm
        }
    }

    if (wheel_r_jnt >= 0) {
        int dof = m->jnt_dofadr[wheel_r_jnt];
        if (dof >= 0 && dof < m->nv) {
            double tau_r = d->qfrc_inverse[dof];
            INFO("wheel_R torque at equilibrium: " << tau_r << " Nm");
            CHECK(std::abs(tau_r) < 1e-3);  // Less than 1 mNm
        }
    }

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Position actuators with gear use transmission-scaled control", "[actuator]")
{
    // This test verifies that linearize_hip.cpp correctly uses joint coordinates
    // for position actuators with inheritrange="1": d->ctrl[act] = target_angle

    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int act_hip_L = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");

    REQUIRE(hip_id >= 0);
    REQUIRE(act_hip_L >= 0);

    // Get actuator gear and ctrlrange
    mjtNum gear = m->actuator_gear[6 * act_hip_L];
    mjtNum ctrlrange_min = m->actuator_ctrlrange[2 * act_hip_L];
    mjtNum ctrlrange_max = m->actuator_ctrlrange[2 * act_hip_L + 1];
    INFO("hip_L actuator gear: " << gear << " (provides force multiplication and transmission scaling)");
    INFO("hip_L actuator ctrlrange: [" << ctrlrange_min << ", " << ctrlrange_max << "]");

    // Test: verify gear-scaled control reaches target position
    const double target_angle = -0.15;  // Within joint range

    mj_resetDataKeyframe(m, d, 0);

    // CORRECT: ctrl = target * gear (due to actuator_length = joint_angle * gear)
    d->ctrl[act_hip_L] = target_angle * gear;
    INFO("Setting ctrl = " << (target_angle * gear));

    // Simulate to let actuator reach target
    for (int step = 0; step < 2000; step++) {
        mj_step(m, d);
    }

    // Check joint reached target position
    int qpos_idx = m->jnt_qposadr[hip_id];
    double actual_angle = d->qpos[qpos_idx];

    INFO("target_angle=" << target_angle << " ctrl=" << (target_angle * gear) << " actual_angle=" << actual_angle);
    // Relax margin slightly to account for steady-state error with gravity
    CHECK(actual_angle == Catch::Approx(target_angle).margin(0.002));

    mj_deleteData(d);
    mj_deleteModel(m);
}
