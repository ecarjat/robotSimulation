// Clean system identification using MuJoCo's qacc
// Focus on 2x2 pitch subsystem: [theta, thetaDot]
// Use qacc directly instead of finite differencing

#include <mujoco/mujoco.h>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <functional>

struct Sample {
    double theta, thetaDot, thetaDDot;
    double x, xDot, xDDot;
    double u;
};

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

    double dt = m->opt.timestep;
    printf("MuJoCo timestep: %.4f s\n", dt);
    printf("qvel layout: [0]=vx, [1]=wy (pitch rate), [2]=vz, [3]=wx, [4]=...\n\n");

    std::vector<Sample> samples;
    
    auto collect = [&](const char* name, double duration, 
                       std::function<double(double)> ctrl_fn) {
        printf("Collecting: %s\n", name);
        
        mjData* d = mj_makeData(m);
        if (m->nkey > 0) mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
        
        int steps = (int)(duration / dt);
        for (int step = 0; step < steps; step++) {
            double t = step * dt;
            double ctrl = ctrl_fn(t);
            
            if (act_wL >= 0) d->ctrl[act_wL] = ctrl;
            if (act_wR >= 0) d->ctrl[act_wR] = ctrl;
            if (act_hL >= 0) d->ctrl[act_hL] = 0.0;
            if (act_hR >= 0) d->ctrl[act_hR] = 0.0;
            
            // Forward to compute qacc
            mj_forward(m, d);
            
            double theta = get_pitch(d);
            double thetaDot = d->qvel[1];  // pitch rate from MuJoCo
            double thetaDDot = d->qacc[1]; // pitch accel from MuJoCo
            double x = d->qpos[0];
            double xDot = d->qvel[0];
            double xDDot = d->qacc[0];
            double u = -2.0 * ctrl;  // total torque (gear=-1, 2 wheels)
            
            // Only collect when pitch is small (linear region)
            if (step > 5 && std::abs(theta) < 0.2) {
                samples.push_back({theta, thetaDot, thetaDDot, x, xDot, xDDot, u});
            }
            
            mj_step(m, d);
            
            if (std::abs(get_pitch(d)) > 0.5) break;
        }
        mj_deleteData(d);
    };
    
    // Various excitation signals
    collect("zero input", 0.3, [](double t) { return 0.0; });
    collect("small positive", 0.4, [](double t) { return 0.1; });
    collect("small negative", 0.4, [](double t) { return -0.1; });
    collect("medium positive", 0.3, [](double t) { return 0.25; });
    collect("medium negative", 0.3, [](double t) { return -0.25; });
    collect("sine 5Hz", 0.5, [](double t) { return 0.2 * std::sin(2*M_PI*5*t); });
    collect("sine 10Hz", 0.5, [](double t) { return 0.2 * std::sin(2*M_PI*10*t); });
    collect("sine 20Hz", 0.3, [](double t) { return 0.15 * std::sin(2*M_PI*20*t); });
    
    printf("\nCollected %zu samples\n\n", samples.size());
    
    // ========== 2x2 PITCH SUBSYSTEM ==========
    // Model: thetaDDot = a * theta + b * u + c * thetaDot
    // (including damping term c)
    //
    // Least squares: [theta, u, thetaDot] * [a; b; c] = thetaDDot
    
    int n = samples.size();
    double XTX[3][3] = {0}, XTy[3] = {0};
    
    for (const auto& s : samples) {
        double X[3] = {s.theta, s.u, s.thetaDot};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                XTX[i][j] += X[i] * X[j];
            }
            XTy[i] += X[i] * s.thetaDDot;
        }
    }
    
    // Regularization
    for (int i = 0; i < 3; i++) XTX[i][i] += 1e-8;
    
    // Solve 3x3 system
    double aug[3][4];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) aug[i][j] = XTX[i][j];
        aug[i][3] = XTy[i];
    }
    
    for (int i = 0; i < 3; i++) {
        int maxRow = i;
        for (int k = i+1; k < 3; k++)
            if (std::abs(aug[k][i]) > std::abs(aug[maxRow][i])) maxRow = k;
        for (int k = 0; k < 4; k++) std::swap(aug[i][k], aug[maxRow][k]);
        for (int k = i+1; k < 3; k++) {
            double c = aug[k][i] / aug[i][i];
            for (int j = i; j < 4; j++) aug[k][j] -= c * aug[i][j];
        }
    }
    double coef[3];
    for (int i = 2; i >= 0; i--) {
        coef[i] = aug[i][3];
        for (int j = i+1; j < 3; j++) coef[i] -= aug[i][j] * coef[j];
        coef[i] /= aug[i][i];
    }
    
    double a = coef[0], b = coef[1], c = coef[2];
    
    printf("========== PITCH SUBSYSTEM (using qacc) ==========\n");
    printf("Model: theta_ddot = a*theta + b*u + c*theta_dot\n\n");
    printf("  a = %10.4f  (gravity/pendulum term)\n", a);
    printf("  b = %10.4f  (input gain, rad/s² per Nm)\n", b);
    printf("  c = %10.4f  (damping)\n", c);
    
    if (a > 0) {
        printf("\nOpen-loop unstable pole: sqrt(%.2f) = %.2f rad/s\n", a, std::sqrt(a));
    } else {
        printf("\nWARNING: a <= 0 means system appears stable (unexpected!)\n");
    }
    
    // R² for pitch model
    double ss_tot = 0, ss_res = 0, mean_ddot = 0;
    for (const auto& s : samples) mean_ddot += s.thetaDDot;
    mean_ddot /= n;
    for (const auto& s : samples) {
        double pred = a * s.theta + b * s.u + c * s.thetaDot;
        ss_res += (s.thetaDDot - pred) * (s.thetaDDot - pred);
        ss_tot += (s.thetaDDot - mean_ddot) * (s.thetaDDot - mean_ddot);
    }
    printf("Fit R² = %.4f\n", 1.0 - ss_res / (ss_tot + 1e-10));
    
    // ========== POSITION DYNAMICS ==========
    // Model: xDDot = d*theta + e*u + f*xDot
    printf("\n========== POSITION DYNAMICS ==========\n");
    
    double XTX2[3][3] = {0}, XTy2[3] = {0};
    for (const auto& s : samples) {
        double X[3] = {s.theta, s.u, s.xDot};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) XTX2[i][j] += X[i] * X[j];
            XTy2[i] += X[i] * s.xDDot;
        }
    }
    for (int i = 0; i < 3; i++) XTX2[i][i] += 1e-8;
    
    double aug2[3][4];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) aug2[i][j] = XTX2[i][j];
        aug2[i][3] = XTy2[i];
    }
    for (int i = 0; i < 3; i++) {
        int maxRow = i;
        for (int k = i+1; k < 3; k++)
            if (std::abs(aug2[k][i]) > std::abs(aug2[maxRow][i])) maxRow = k;
        for (int k = 0; k < 4; k++) std::swap(aug2[i][k], aug2[maxRow][k]);
        for (int k = i+1; k < 3; k++) {
            double c = aug2[k][i] / aug2[i][i];
            for (int j = i; j < 4; j++) aug2[k][j] -= c * aug2[i][j];
        }
    }
    double coef2[3];
    for (int i = 2; i >= 0; i--) {
        coef2[i] = aug2[i][3];
        for (int j = i+1; j < 3; j++) coef2[i] -= aug2[i][j] * coef2[j];
        coef2[i] /= aug2[i][i];
    }
    
    printf("Model: x_ddot = d*theta + e*u + f*x_dot\n\n");
    printf("  d = %10.4f  (pitch→accel coupling)\n", coef2[0]);
    printf("  e = %10.4f  (torque→accel)\n", coef2[1]);
    printf("  f = %10.4f  (velocity damping)\n", coef2[2]);
    
    ss_tot = 0; ss_res = 0; double mean_xddot = 0;
    for (const auto& s : samples) mean_xddot += s.xDDot;
    mean_xddot /= n;
    for (const auto& s : samples) {
        double pred = coef2[0] * s.theta + coef2[1] * s.u + coef2[2] * s.xDot;
        ss_res += (s.xDDot - pred) * (s.xDDot - pred);
        ss_tot += (s.xDDot - mean_xddot) * (s.xDDot - mean_xddot);
    }
    printf("Fit R² = %.4f\n", 1.0 - ss_res / (ss_tot + 1e-10));
    
    // ========== COMBINED 4-STATE MODEL ==========
    printf("\n========== COMBINED 4-STATE MODEL ==========\n");
    printf("State: x = [pos, vel, theta, thetaDot]\n");
    printf("x_dot = A*x + B*u\n\n");
    
    printf("A = np.array([\n");
    printf("    [%12.6f, %12.6f, %12.6f, %12.6f],  # pos_dot\n", 
           0.0, 1.0, 0.0, 0.0);
    printf("    [%12.6f, %12.6f, %12.6f, %12.6f],  # vel_dot\n", 
           0.0, coef2[2], coef2[0], 0.0);
    printf("    [%12.6f, %12.6f, %12.6f, %12.6f],  # theta_dot\n", 
           0.0, 0.0, 0.0, 1.0);
    printf("    [%12.6f, %12.6f, %12.6f, %12.6f],  # thetaDot_dot\n", 
           0.0, 0.0, a, c);
    printf("])\n\n");
    
    printf("B = np.array([[%12.6f], [%12.6f], [%12.6f], [%12.6f]])\n",
           0.0, coef2[1], 0.0, b);
    
    // Print some sample data for debugging
    printf("\n========== SAMPLE DATA (first 10) ==========\n");
    printf("%10s %10s %10s %10s %10s %10s %10s\n", 
           "theta", "thetaDot", "thetaDDot", "x", "xDot", "xDDot", "u");
    for (int i = 0; i < 10 && i < (int)samples.size(); i++) {
        const auto& s = samples[i];
        printf("%10.6f %10.4f %10.2f %10.6f %10.4f %10.2f %10.4f\n",
               s.theta, s.thetaDot, s.thetaDDot, s.x, s.xDot, s.xDDot, s.u);
    }
    
    mj_deleteModel(m);
    return 0;
}
