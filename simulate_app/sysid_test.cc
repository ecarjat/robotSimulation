// System identification: measure actual a (gravity) and b (input) coefficients
// from MuJoCo simulation by analyzing pitch dynamics.
//
// Model: theta_ddot = a * theta + b * u
// where u is total wheel torque (Nm, both wheels combined)
//
// Test 1: Free fall from small initial pitch offset → extracts 'a'
// Test 2: Constant torque from equilibrium → extracts 'b'
#include <mujoco/mujoco.h>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>

struct Sample {
    double t, theta, thetaDot, thetaDDot, x, xDot;
};

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
    printf("dt = %.4f\n\n", dt);

    // Helper: get pitch from quaternion
    auto get_pitch = [](const mjData* d) -> double {
        double qw = d->qpos[3], qx = d->qpos[4], qy = d->qpos[5], qz = d->qpos[6];
        return std::asin(2.0 * (qw * qy - qz * qx));
    };

    // ========== TEST 1: Free fall from initial pitch offset ==========
    printf("=== TEST 1: Free-fall from initial pitch offset ===\n");
    {
        mjData* d = mj_makeData(m);
        if (m->nkey > 0) mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);

        // Apply small pitch offset by rotating the quaternion
        double pitch_offset = 0.02; // 0.02 rad ≈ 1.15°
        // Rotate quat by pitch_offset around Y axis
        double half = pitch_offset / 2.0;
        double cH = std::cos(half), sH = std::sin(half);
        // q_new = q_offset * q_orig
        double qw = d->qpos[3], qx = d->qpos[4], qy = d->qpos[5], qz = d->qpos[6];
        d->qpos[3] = cH * qw - sH * qy;  // w
        d->qpos[4] = cH * qx + sH * qz;  // x
        d->qpos[5] = sH * qw + cH * qy;  // y
        d->qpos[6] = cH * qz - sH * qx;  // z
        mj_forward(m, d);

        double initial_pitch = get_pitch(d);
        printf("Initial pitch offset: %.4f rad (%.2f°)\n", initial_pitch, initial_pitch * 57.3);

        std::vector<Sample> samples;
        double prev_thetaDot = 0.0;

        for (int step = 0; step < 150; step++) { // 0.3s
            // Zero all controls
            if (act_wL >= 0) d->ctrl[act_wL] = 0.0;
            if (act_wR >= 0) d->ctrl[act_wR] = 0.0;
            if (act_hL >= 0) d->ctrl[act_hL] = 0.0;
            if (act_hR >= 0) d->ctrl[act_hR] = 0.0;

            mj_step(m, d);

            double theta = get_pitch(d);
            double thetaDot = d->qvel[1]; // pitch rate from qvel
            double thetaDDot = (thetaDot - prev_thetaDot) / dt;
            prev_thetaDot = thetaDot;

            samples.push_back({d->time, theta, thetaDot, thetaDDot, d->qpos[0], d->qvel[0]});
        }

        // Print data
        printf("%8s %10s %10s %12s %10s\n", "time", "theta", "thetaDot", "thetaDDot", "a=ddot/th");
        for (size_t i = 2; i < samples.size(); i += 5) {
            auto& s = samples[i];
            double a_est = (std::abs(s.theta) > 1e-6) ? s.thetaDDot / s.theta : 0.0;
            printf("%8.3f %10.6f %10.6f %12.4f %10.2f\n",
                   s.t, s.theta, s.thetaDot, s.thetaDDot, a_est);
        }

        // Estimate 'a' from early samples (before nonlinear effects dominate)
        double a_sum = 0.0;
        int a_count = 0;
        for (size_t i = 5; i < 50 && i < samples.size(); i++) {
            if (std::abs(samples[i].theta) > 1e-5) {
                a_sum += samples[i].thetaDDot / samples[i].theta;
                a_count++;
            }
        }
        double a_est = (a_count > 0) ? a_sum / a_count : 0.0;
        printf("\nEstimated a = %.2f rad/s² per rad (from %d samples)\n", a_est, a_count);
        printf("(Compare: analytical a = m*g*L/(I_b+m*L^2) with L=0.036 → a=%.2f)\n",
               0.43 * 9.81 * 0.036 / (0.43 * 0.036 * 0.036 + 0.43 * 0.036 * 0.036));

        mj_deleteData(d);
    }

    // ========== TEST 2: Torque step from equilibrium ==========
    printf("\n=== TEST 2: Torque step (ctrl=0.1) from equilibrium ===\n");
    {
        mjData* d = mj_makeData(m);
        if (m->nkey > 0) mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);

        double ctrl_val = 0.1; // Small torque
        // With gear=-1, actual joint torque = -0.1 Nm per wheel = -0.2 Nm total
        double u_total = -2.0 * ctrl_val; // total torque (gear * ctrl * 2 wheels)
        printf("ctrl = %.3f → joint torque per wheel = %.3f Nm → total = %.3f Nm\n",
               ctrl_val, -ctrl_val, u_total);

        std::vector<Sample> samples;
        double prev_thetaDot = 0.0;

        for (int step = 0; step < 150; step++) {
            if (act_wL >= 0) d->ctrl[act_wL] = ctrl_val;
            if (act_wR >= 0) d->ctrl[act_wR] = ctrl_val;
            if (act_hL >= 0) d->ctrl[act_hL] = 0.0;
            if (act_hR >= 0) d->ctrl[act_hR] = 0.0;

            mj_step(m, d);

            double theta = get_pitch(d);
            double thetaDot = d->qvel[1];
            double thetaDDot = (thetaDot - prev_thetaDot) / dt;
            prev_thetaDot = thetaDot;

            samples.push_back({d->time, theta, thetaDot, thetaDDot, d->qpos[0], d->qvel[0]});
        }

        printf("%8s %10s %10s %12s %12s\n", "time", "theta", "thetaDot", "thetaDDot", "b=ddot/u");
        for (size_t i = 2; i < samples.size(); i += 5) {
            auto& s = samples[i];
            double b_est = s.thetaDDot / u_total;
            printf("%8.3f %10.6f %10.6f %12.4f %12.2f\n",
                   s.t, s.theta, s.thetaDot, s.thetaDDot, b_est);
        }

        // Estimate 'b' from early samples (before theta grows enough for gravity)
        double b_sum = 0.0;
        int b_count = 0;
        for (size_t i = 2; i < 20 && i < samples.size(); i++) {
            // At early times, theta is small so thetaDDot ≈ b*u
            if (std::abs(u_total) > 1e-6) {
                b_sum += samples[i].thetaDDot / u_total;
                b_count++;
            }
        }
        double b_est = (b_count > 0) ? b_sum / b_count : 0.0;
        printf("\nEstimated b = %.2f rad/s² per Nm (from %d samples)\n", b_est, b_count);

        mj_deleteData(d);
    }

    // ========== TEST 3: Also measure with FD pitch (like the controller uses) ==========
    printf("\n=== TEST 3: Torque step with pitch from Euler quaternion ===\n");
    {
        mjData* d = mj_makeData(m);
        if (m->nkey > 0) mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);

        double ctrl_val = 0.2;
        double u_total = -2.0 * ctrl_val;
        printf("ctrl = %.3f → total torque = %.3f Nm\n", ctrl_val, u_total);

        double prev_theta = get_pitch(d);
        double prev_thetaDot = 0.0;

        printf("%8s %10s %10s %12s %10s %10s\n",
               "time", "theta_deg", "thetaDot", "thetaDDot", "x", "xDot");

        for (int step = 0; step < 100; step++) {
            if (act_wL >= 0) d->ctrl[act_wL] = ctrl_val;
            if (act_wR >= 0) d->ctrl[act_wR] = ctrl_val;
            if (act_hL >= 0) d->ctrl[act_hL] = 0.0;
            if (act_hR >= 0) d->ctrl[act_hR] = 0.0;

            mj_step(m, d);

            double theta = get_pitch(d);
            double thetaDot = (theta - prev_theta) / dt;
            double thetaDDot = (thetaDot - prev_thetaDot) / dt;
            prev_theta = theta;
            prev_thetaDot = thetaDot;

            if (step % 10 == 9) {
                printf("%8.3f %10.4f %10.4f %12.2f %10.6f %10.4f\n",
                       d->time, theta * 57.3, thetaDot, thetaDDot,
                       d->qpos[0], d->qvel[0]);
            }
        }
        mj_deleteData(d);
    }

    mj_deleteModel(m);
    return 0;
}
