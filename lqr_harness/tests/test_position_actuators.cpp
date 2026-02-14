/**
 * @file test_position_actuators.cpp
 * @brief Tests for MuJoCo position actuator behavior with gear and inheritrange
 *
 * KEY INSIGHT: Position actuators with gear parameter scale transmission:
 * - actuator_length = joint_angle * gear
 * - Position servo drives: actuator_length → ctrl
 * - Therefore: ctrl = joint_angle * gear
 * - inheritrange should match gear to allow full joint ROM
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <mujoco/mujoco.h>

#include <cmath>
#include <string>

TEST_CASE("Position actuator with gear uses transmission-scaled control", "[actuator][critical]")
{
    // CRITICAL: For position actuators with gear:
    //   - actuator_length = joint_angle * gear (transmission scaling)
    //   - position servo drives: actuator_length → ctrl
    //   - Therefore: ctrl = joint_angle * gear
    //   - inheritrange should match gear for full joint ROM

    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int act_hip = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");

    REQUIRE(hip_id >= 0);
    REQUIRE(act_hip >= 0);

    // Verify actuator properties
    mjtNum gear = m->actuator_gear[6 * act_hip];
    INFO("Actuator gear: " << gear << " (provides force multiplication)");

    // Test multiple target positions within joint range
    const double target_angles[] = {-0.25, -0.15, -0.05, 0.05, 0.15, 0.25};

    for (double target : target_angles) {
        mj_resetDataKeyframe(m, d, 0);

        // CORRECT: ctrl = target * gear (due to transmission scaling)
        d->ctrl[act_hip] = target * gear;

        // Simulate to steady state
        for (int step = 0; step < 2000; step++) {
            mj_step(m, d);
        }

        // Read actual joint position
        int qpos_idx = m->jnt_qposadr[hip_id];
        double actual = d->qpos[qpos_idx];

        INFO("target=" << target << " ctrl=" << (target * gear) << " actual=" << actual);

        // Joint should reach target position (with small margin for steady-state error)
        CHECK(actual == Catch::Approx(target).margin(0.005));
    }

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Position actuator respects inherited joint limits", "[actuator]")
{
    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int act_hip = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");

    REQUIRE(hip_id >= 0);
    REQUIRE(act_hip >= 0);

    // Get joint limits
    int jnt_limited = m->jnt_limited[hip_id];
    double jnt_range[2] = {0, 0};
    if (jnt_limited) {
        jnt_range[0] = m->jnt_range[2*hip_id + 0];
        jnt_range[1] = m->jnt_range[2*hip_id + 1];
    }

    INFO("Joint limits: [" << jnt_range[0] << ", " << jnt_range[1] << "]");
    REQUIRE(jnt_limited);

    // Get actuator gear
    mjtNum gear = m->actuator_gear[6 * act_hip];

    // Test: command position beyond upper limit
    {
        mj_resetDataKeyframe(m, d, 0);
        double beyond_limit = jnt_range[1] + 0.5;
        d->ctrl[act_hip] = beyond_limit * gear;

        for (int step = 0; step < 2000; step++) {
            mj_step(m, d);
        }

        int qpos_idx = m->jnt_qposadr[hip_id];
        double actual = d->qpos[qpos_idx];

        INFO("Commanded beyond_limit=" << beyond_limit << " actual=" << actual);
        CHECK(actual <= jnt_range[1] + 0.01);  // Should be clamped
        CHECK(actual >= jnt_range[1] - 0.05);  // Should try to reach limit
    }

    // Test: command position beyond lower limit
    {
        mj_resetDataKeyframe(m, d, 0);
        double beyond_limit = jnt_range[0] - 0.5;
        d->ctrl[act_hip] = beyond_limit * gear;

        for (int step = 0; step < 2000; step++) {
            mj_step(m, d);
        }

        int qpos_idx = m->jnt_qposadr[hip_id];
        double actual = d->qpos[qpos_idx];

        INFO("Commanded beyond_limit=" << beyond_limit << " actual=" << actual);
        // Joint limits in MuJoCo are soft constraints, may overshoot slightly
        CHECK(actual >= jnt_range[0] - 0.02);  // Should be near lower limit
        CHECK(actual <= jnt_range[0] + 0.05);  // Should try to reach limit
    }

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Hip actuators can reach full range of motion", "[actuator][range]")
{
    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    // Test both hip actuators
    const char* hip_joints[] = {"hip_L", "hip_R"};
    const char* hip_actuators[] = {"hip_L", "hip_R"};

    for (int i = 0; i < 2; ++i) {
        int hip_id = mj_name2id(m, mjOBJ_JOINT, hip_joints[i]);
        int act_hip = mj_name2id(m, mjOBJ_ACTUATOR, hip_actuators[i]);

        REQUIRE(hip_id >= 0);
        REQUIRE(act_hip >= 0);

        // Get joint limits
        double jnt_range[2];
        jnt_range[0] = m->jnt_range[2*hip_id + 0];
        jnt_range[1] = m->jnt_range[2*hip_id + 1];

        INFO("Testing " << hip_joints[i] << " range [" << jnt_range[0] << ", " << jnt_range[1] << "]");

        // Get actuator gear
        mjtNum gear = m->actuator_gear[6 * act_hip];

        // Test lower limit
        {
            mj_resetDataKeyframe(m, d, 0);
            d->ctrl[act_hip] = jnt_range[0] * gear;

            for (int step = 0; step < 2000; step++) {
                mj_step(m, d);
            }

            int qpos_idx = m->jnt_qposadr[hip_id];
            double actual = d->qpos[qpos_idx];

            INFO(hip_joints[i] << " lower limit: target=" << jnt_range[0] << " actual=" << actual);
            CHECK(actual == Catch::Approx(jnt_range[0]).margin(0.01));
        }

        // Test upper limit
        {
            mj_resetDataKeyframe(m, d, 0);
            d->ctrl[act_hip] = jnt_range[1] * gear;

            for (int step = 0; step < 2000; step++) {
                mj_step(m, d);
            }

            int qpos_idx = m->jnt_qposadr[hip_id];
            double actual = d->qpos[qpos_idx];

            INFO(hip_joints[i] << " upper limit: target=" << jnt_range[1] << " actual=" << actual);
            CHECK(actual == Catch::Approx(jnt_range[1]).margin(0.05));
        }

        // Test mid-range
        {
            mj_resetDataKeyframe(m, d, 0);
            double mid = (jnt_range[0] + jnt_range[1]) * 0.5;
            d->ctrl[act_hip] = mid * gear;

            for (int step = 0; step < 2000; step++) {
                mj_step(m, d);
            }

            int qpos_idx = m->jnt_qposadr[hip_id];
            double actual = d->qpos[qpos_idx];

            INFO(hip_joints[i] << " mid-range: target=" << mid << " actual=" << actual);
            CHECK(actual == Catch::Approx(mid).margin(0.01));
        }
    }

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Gear parameter scales transmission and requires matching inheritrange", "[actuator][gear]")
{
    // This test documents the role of the gear parameter:
    // - gear=5 scales transmission: actuator_length = joint_angle * 5
    // - Provides force multiplication (needed to support torso weight)
    // - Requires ctrl = joint_angle * gear to reach target
    // - inheritrange should match gear for full joint ROM

    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int act_hip = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");

    REQUIRE(hip_id >= 0);
    REQUIRE(act_hip >= 0);

    mjtNum gear = m->actuator_gear[6 * act_hip];
    const double target = -0.15;

    INFO("Actuator gear: " << gear);
    INFO("Purpose: Transmission scaling AND force multiplication");

    // Verify: ctrl = target * gear (due to transmission scaling)
    {
        mj_resetDataKeyframe(m, d, 0);
        d->ctrl[act_hip] = target * gear;  // Scaled by gear

        for (int step = 0; step < 2000; step++) {
            mj_step(m, d);
        }

        int qpos_idx = m->jnt_qposadr[hip_id];
        double actual = d->qpos[qpos_idx];

        INFO("target=" << target << " ctrl=" << (target * gear) << " actual=" << actual);
        // Should reach target when ctrl is scaled by gear (with small margin for steady-state error)
        CHECK(actual == Catch::Approx(target).margin(0.005));
    }

    mj_deleteData(d);
    mj_deleteModel(m);
}
