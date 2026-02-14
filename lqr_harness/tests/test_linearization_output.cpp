/**
 * @file test_linearization_output.cpp
 * @brief Tests for linearization pipeline output validation
 *
 * Validates that linearization produces physically reasonable
 * state-space matrices with correct structure and magnitudes.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <mujoco/mujoco.h>

#include <cmath>
#include <vector>
#include <string>

namespace {

int joint_id(const mjModel* m, const char* name) {
    return mj_name2id(m, mjOBJ_JOINT, name);
}

int actuator_id(const mjModel* m, const char* name) {
    return mj_name2id(m, mjOBJ_ACTUATOR, name);
}

} // namespace

TEST_CASE("Linearization produces finite A and B matrices", "[linearization]")
{
    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    // Switch to Euler integrator for finite difference
    m->opt.integrator = mjINT_EULER;

    // Reset to keyframe
    mj_resetDataKeyframe(m, d, 0);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    REQUIRE(hip_id >= 0);

    // Set test hip angle
    d->qpos[m->jnt_qposadr[hip_id]] = -0.1;
    mj_forward(m, d);

    // Allocate A, B matrices
    const int state_dim = 2 * m->nv + m->na;
    const int a_rows = state_dim;
    const int a_cols = state_dim;
    const int b_rows = state_dim;
    const int b_cols = m->nu;

    std::vector<mjtNum> A(a_rows * a_cols);
    std::vector<mjtNum> B(b_rows * b_cols);

    // Compute linearization
    const double eps = 1e-6;
    mjd_transitionFD(m, d, eps, 1, A.data(), B.data(), nullptr, nullptr);

    // Check all elements are finite
    for (size_t i = 0; i < A.size(); ++i) {
        REQUIRE(std::isfinite(A[i]));
    }

    for (size_t i = 0; i < B.size(); ++i) {
        REQUIRE(std::isfinite(B[i]));
    }

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Reduced B matrix (Bred) has expected structure", "[linearization][bred]")
{
    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    m->opt.integrator = mjINT_EULER;
    mj_resetDataKeyframe(m, d, 0);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    d->qpos[m->jnt_qposadr[hip_id]] = -0.1;
    mj_forward(m, d);

    const int state_dim = 2 * m->nv + m->na;
    std::vector<mjtNum> A(state_dim * state_dim);
    std::vector<mjtNum> B(state_dim * m->nu);

    mjd_transitionFD(m, d, 1e-6, 1, A.data(), B.data(), nullptr, nullptr);

    // Build reduced B matrix: Bred = C * B
    // State indices for free joint
    const int free_id = joint_id(m, "torso_freejoint");
    const int base_dof = (free_id >= 0) ? m->jnt_dofadr[free_id] : 0;
    const int idx_trans_x = base_dof + 0;  // forward position
    const int idx_rot_y = base_dof + 4;    // pitch angle
    const int idx_x = idx_trans_x;
    const int idx_xdot = m->nv + idx_trans_x;
    const int idx_theta = idx_rot_y;
    const int idx_thetadot = m->nv + idx_rot_y;

    // Build C matrix (4 x state_dim): extract [x, xdot, theta, thetadot]
    std::vector<mjtNum> C(4 * state_dim, 0.0);
    C[0 * state_dim + idx_x] = 1.0;
    C[1 * state_dim + idx_xdot] = 1.0;
    C[2 * state_dim + idx_theta] = 1.0;
    C[3 * state_dim + idx_thetadot] = 1.0;

    // Compute Bred = C * B (sum wheel torques)
    int act_wheel_l = actuator_id(m, "wheel_L");
    int act_wheel_r = actuator_id(m, "wheel_R");
    REQUIRE(act_wheel_l >= 0);
    REQUIRE(act_wheel_r >= 0);

    std::vector<mjtNum> Bred(4, 0.0);
    for (int r = 0; r < 4; ++r) {
        double sum = 0.0;
        for (int k = 0; k < state_dim; ++k) {
            double ca = C[r * state_dim + k];
            if (ca != 0.0) {
                sum += ca * B[k * m->nu + act_wheel_l];
                sum += ca * B[k * m->nu + act_wheel_r];
            }
        }
        Bred[r] = sum;
    }

    // Validate Bred structure
    INFO("Bred = [" << Bred[0] << ", " << Bred[1] << ", " << Bred[2] << ", " << Bred[3] << "]");

    // All elements should be finite
    for (int i = 0; i < 4; ++i) {
        REQUIRE(std::isfinite(Bred[i]));
    }

    // Expected structure for inverted pendulum:
    // B[0] (x): small - position changes slowly with torque
    // B[1] (v): moderate - velocity changes with torque
    // B[2] (theta): small - angle changes slowly
    // B[3] (theta_dot): largest - angular acceleration directly affected

    // B[1] and B[3] should be significantly larger than B[0] and B[2]
    CHECK(std::abs(Bred[1]) > std::abs(Bred[0]) * 10.0);
    CHECK(std::abs(Bred[3]) > std::abs(Bred[2]) * 10.0);

    // B[3] (angular acceleration) should be the largest
    CHECK(std::abs(Bred[3]) > std::abs(Bred[1]));

    // All non-zero (robot should be controllable)
    CHECK(std::abs(Bred[0]) > 1e-9);
    CHECK(std::abs(Bred[1]) > 1e-6);
    CHECK(std::abs(Bred[2]) > 1e-9);
    CHECK(std::abs(Bred[3]) > 1e-5);

    // Reasonable magnitudes (not extreme)
    CHECK(std::abs(Bred[0]) < 1.0);
    CHECK(std::abs(Bred[1]) < 1.0);
    CHECK(std::abs(Bred[2]) < 1.0);
    CHECK(std::abs(Bred[3]) < 1.0);

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Reduced A matrix (Ared) has expected structure", "[linearization][ared]")
{
    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    m->opt.integrator = mjINT_EULER;
    mj_resetDataKeyframe(m, d, 0);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    d->qpos[m->jnt_qposadr[hip_id]] = -0.1;
    mj_forward(m, d);

    const int state_dim = 2 * m->nv + m->na;
    std::vector<mjtNum> A(state_dim * state_dim);
    std::vector<mjtNum> B(state_dim * m->nu);

    mjd_transitionFD(m, d, 1e-6, 1, A.data(), B.data(), nullptr, nullptr);

    // Build reduced A matrix
    const int free_id = joint_id(m, "torso_freejoint");
    const int base_dof = (free_id >= 0) ? m->jnt_dofadr[free_id] : 0;
    const int idx_trans_x = base_dof + 0;
    const int idx_rot_y = base_dof + 4;
    const int idx_x = idx_trans_x;
    const int idx_xdot = m->nv + idx_trans_x;
    const int idx_theta = idx_rot_y;
    const int idx_thetadot = m->nv + idx_rot_y;

    std::vector<mjtNum> C(4 * state_dim, 0.0);
    std::vector<mjtNum> S(state_dim * 4, 0.0);

    C[0 * state_dim + idx_x] = 1.0;
    S[idx_x * 4 + 0] = 1.0;
    C[1 * state_dim + idx_xdot] = 1.0;
    S[idx_xdot * 4 + 1] = 1.0;
    C[2 * state_dim + idx_theta] = 1.0;
    S[idx_theta * 4 + 2] = 1.0;
    C[3 * state_dim + idx_thetadot] = 1.0;
    S[idx_thetadot * 4 + 3] = 1.0;

    // Ared = C * A * S
    std::vector<mjtNum> Ared(4 * 4, 0.0);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double sum = 0.0;
            for (int k = 0; k < state_dim; ++k) {
                double ca = C[r * state_dim + k];
                if (ca == 0.0) continue;
                for (int j = 0; j < state_dim; ++j) {
                    double as = A[k * state_dim + j];
                    double s = S[j * 4 + c];
                    if (s == 0.0) continue;
                    sum += ca * as * s;
                }
            }
            Ared[r * 4 + c] = sum;
        }
    }

    // Validate all elements finite
    for (int i = 0; i < 16; ++i) {
        REQUIRE(std::isfinite(Ared[i]));
    }

    // Expected structure for continuous-time linearization of inverted pendulum:
    // Row 0 (x_next):    affected by x, v
    // Row 1 (v_next):    affected by x, v, theta, theta_dot
    // Row 2 (theta_next): affected by theta, theta_dot
    // Row 3 (theta_dot_next): affected by x, v, theta, theta_dot

    // A[0,1] (x affected by v) should be non-zero and positive
    INFO("Ared[0,1] (position from velocity) = " << Ared[0*4 + 1]);
    CHECK(Ared[0*4 + 1] > 0.0);

    // A[2,3] (theta affected by theta_dot) should be non-zero and positive
    INFO("Ared[2,3] (angle from angular velocity) = " << Ared[2*4 + 3]);
    CHECK(Ared[2*4 + 3] > 0.0);

    // A should not be identity matrix (would indicate linearization failed)
    bool is_identity = true;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double expected = (r == c) ? 1.0 : 0.0;
            if (std::abs(Ared[r*4 + c] - expected) > 0.01) {
                is_identity = false;
                break;
            }
        }
        if (!is_identity) break;
    }
    CHECK_FALSE(is_identity);

    mj_deleteData(d);
    mj_deleteModel(m);
}

TEST_CASE("Linearization is consistent across multiple calls", "[linearization][stability]")
{
    const std::string model_path = TEST_MODEL_PATH;

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    m->opt.integrator = mjINT_EULER;

    const int state_dim = 2 * m->nv + m->na;
    std::vector<mjtNum> A1(state_dim * state_dim);
    std::vector<mjtNum> B1(state_dim * m->nu);
    std::vector<mjtNum> A2(state_dim * state_dim);
    std::vector<mjtNum> B2(state_dim * m->nu);

    const double test_hip = -0.1;
    const double eps = 1e-6;

    // First linearization
    mj_resetDataKeyframe(m, d, 0);
    int hip_id = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    d->qpos[m->jnt_qposadr[hip_id]] = test_hip;
    mj_forward(m, d);
    mjd_transitionFD(m, d, eps, 1, A1.data(), B1.data(), nullptr, nullptr);

    // Second linearization (reset state first)
    mj_resetDataKeyframe(m, d, 0);
    d->qpos[m->jnt_qposadr[hip_id]] = test_hip;
    mj_forward(m, d);
    mjd_transitionFD(m, d, eps, 1, A2.data(), B2.data(), nullptr, nullptr);

    // Should produce identical results
    for (size_t i = 0; i < A1.size(); ++i) {
        CHECK(A1[i] == Catch::Approx(A2[i]).margin(1e-10));
    }

    for (size_t i = 0; i < B1.size(); ++i) {
        CHECK(B1[i] == Catch::Approx(B2[i]).margin(1e-10));
    }

    mj_deleteData(d);
    mj_deleteModel(m);
}
