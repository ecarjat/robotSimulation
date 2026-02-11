// Full state-space system identification
// Estimates 4x4 A matrix and 4x1 B matrix for:
//   x = [position, velocity, pitch, pitch_rate]
//   x_dot = A*x + B*u
//
// Uses least-squares fitting from simulation data

#include <mujoco/mujoco.h>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <array>
#include <functional>

struct Sample {
    double t;
    std::array<double, 4> x;      // state: [pos, vel, theta, thetaDot]
    std::array<double, 4> x_dot;  // state derivative
    double u;                      // control input (total wheel torque)
};

// Get pitch from quaternion
double get_pitch(const mjData* d) {
    double qw = d->qpos[3], qx = d->qpos[4], qy = d->qpos[5], qz = d->qpos[6];
    return std::asin(2.0 * (qw * qy - qz * qx));
}

// Simple 5x5 matrix operations for least squares
void print_matrix(const char* name, const double* M, int rows, int cols) {
    printf("%s:\n", name);
    for (int i = 0; i < rows; i++) {
        printf("  [");
        for (int j = 0; j < cols; j++) {
            printf("%10.4f", M[i * cols + j]);
        }
        printf(" ]\n");
    }
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
    printf("MuJoCo timestep: %.4f s\n\n", dt);

    std::vector<Sample> samples;
    
    // ========== DATA COLLECTION ==========
    // Run multiple trajectories with different inputs
    
    auto collect_trajectory = [&](const char* name, double duration, 
                                   std::function<double(double)> ctrl_fn) {
        printf("Collecting: %s (%.1fs)\n", name, duration);
        
        mjData* d = mj_makeData(m);
        if (m->nkey > 0) mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
        
        std::array<double, 4> prev_x = {d->qpos[0], d->qvel[0], get_pitch(d), d->qvel[1]};
        
        int steps = (int)(duration / dt);
        for (int step = 0; step < steps; step++) {
            double t = step * dt;
            double ctrl = ctrl_fn(t);
            
            // Apply control (gear = -1, so torque = -ctrl)
            if (act_wL >= 0) d->ctrl[act_wL] = ctrl;
            if (act_wR >= 0) d->ctrl[act_wR] = ctrl;
            if (act_hL >= 0) d->ctrl[act_hL] = 0.0;
            if (act_hR >= 0) d->ctrl[act_hR] = 0.0;
            
            mj_step(m, d);
            
            // Get current state
            std::array<double, 4> x = {d->qpos[0], d->qvel[0], get_pitch(d), d->qvel[1]};
            
            // Compute derivative (finite difference)
            std::array<double, 4> x_dot;
            for (int i = 0; i < 4; i++) {
                x_dot[i] = (x[i] - prev_x[i]) / dt;
            }
            
            // Total wheel torque (gear = -1, two wheels)
            double u = -2.0 * ctrl;
            
            // Skip first few samples (transient) and when pitch too large
            if (step > 10 && std::abs(x[2]) < 0.3) {  // pitch < 17°
                samples.push_back({t, prev_x, x_dot, u});
            }
            
            prev_x = x;
            
            // Stop if falling
            if (std::abs(x[2]) > 0.5) break;
        }
        
        mj_deleteData(d);
    };
    
    // Trajectory 1: Small positive torque steps
    collect_trajectory("positive torque step", 0.5, [](double t) { return 0.1; });
    
    // Trajectory 2: Small negative torque steps  
    collect_trajectory("negative torque step", 0.5, [](double t) { return -0.1; });
    
    // Trajectory 3: Larger positive torque
    collect_trajectory("larger positive", 0.3, [](double t) { return 0.3; });
    
    // Trajectory 4: Larger negative torque
    collect_trajectory("larger negative", 0.3, [](double t) { return -0.3; });
    
    // Trajectory 5: Alternating torque (excites dynamics)
    collect_trajectory("alternating", 1.0, [](double t) { 
        return 0.2 * std::sin(10.0 * t); 
    });
    
    // Trajectory 6: Chirp (frequency sweep)
    collect_trajectory("chirp", 1.0, [](double t) { 
        double freq = 1.0 + 20.0 * t;  // 1-21 Hz sweep
        return 0.15 * std::sin(2.0 * M_PI * freq * t); 
    });
    
    // Trajectory 7: Zero input from small perturbation
    collect_trajectory("free response", 0.5, [](double t) { return 0.0; });
    
    printf("\nCollected %zu samples\n\n", samples.size());
    
    if (samples.size() < 20) {
        printf("Not enough samples!\n");
        mj_deleteModel(m);
        return 1;
    }
    
    // ========== LEAST SQUARES FIT ==========
    // We want to find A (4x4) and B (4x1) such that:
    //   x_dot = A*x + B*u
    //
    // Rewrite as: x_dot = [A | B] * [x; u] = W * z
    // where W is 4x5, z is 5x1
    //
    // Stack all samples: X_dot = W * Z
    // Solve: W = X_dot * Z^T * (Z * Z^T)^(-1)
    // Or equivalently: W^T = (Z * Z^T)^(-1) * Z * X_dot^T
    
    int n = samples.size();
    
    // Compute Z * Z^T (5x5) and Z * X_dot^T (5x4)
    double ZZT[5][5] = {0};
    double ZXdotT[5][4] = {0};
    
    for (int i = 0; i < n; i++) {
        const auto& s = samples[i];
        double z[5] = {s.x[0], s.x[1], s.x[2], s.x[3], s.u};
        
        for (int j = 0; j < 5; j++) {
            for (int k = 0; k < 5; k++) {
                ZZT[j][k] += z[j] * z[k];
            }
            for (int k = 0; k < 4; k++) {
                ZXdotT[j][k] += z[j] * s.x_dot[k];
            }
        }
    }
    
    // Add small regularization for numerical stability
    for (int i = 0; i < 5; i++) {
        ZZT[i][i] += 1e-6;
    }
    
    // Solve (Z*Z^T) * W^T = Z * X_dot^T using Gaussian elimination
    // Augmented matrix: [ZZT | ZXdotT]
    double aug[5][9];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) aug[i][j] = ZZT[i][j];
        for (int j = 0; j < 4; j++) aug[i][5+j] = ZXdotT[i][j];
    }
    
    // Forward elimination
    for (int i = 0; i < 5; i++) {
        // Find pivot
        int maxRow = i;
        for (int k = i+1; k < 5; k++) {
            if (std::abs(aug[k][i]) > std::abs(aug[maxRow][i])) maxRow = k;
        }
        // Swap
        for (int k = 0; k < 9; k++) std::swap(aug[i][k], aug[maxRow][k]);
        
        // Eliminate
        for (int k = i+1; k < 5; k++) {
            double c = aug[k][i] / aug[i][i];
            for (int j = i; j < 9; j++) {
                aug[k][j] -= c * aug[i][j];
            }
        }
    }
    
    // Back substitution
    double WT[5][4];  // W^T: each row is a column of W
    for (int col = 0; col < 4; col++) {
        for (int i = 4; i >= 0; i--) {
            double sum = aug[i][5 + col];
            for (int j = i+1; j < 5; j++) {
                sum -= aug[i][j] * WT[j][col];
            }
            WT[i][col] = sum / aug[i][i];
        }
    }
    
    // Extract A (4x4) and B (4x1) from W = [A | B]
    // W is 4x5, so W[i][j] = WT[j][i]
    double A[4][4], B[4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            A[i][j] = WT[j][i];
        }
        B[i] = WT[4][i];
    }
    
    // ========== RESULTS ==========
    printf("========== IDENTIFIED STATE-SPACE MODEL ==========\n");
    printf("State: x = [position, velocity, pitch, pitch_rate]\n");
    printf("Input: u = total wheel torque (Nm)\n\n");
    
    printf("A matrix (4x4):\n");
    printf("          pos       vel     pitch   pitch_rate\n");
    const char* labels[] = {"pos_dot", "vel_dot", "pitch_dot", "prate_dot"};
    for (int i = 0; i < 4; i++) {
        printf("%10s [%9.4f %9.4f %9.4f %9.4f ]\n", 
               labels[i], A[i][0], A[i][1], A[i][2], A[i][3]);
    }
    
    printf("\nB matrix (4x1):\n");
    for (int i = 0; i < 4; i++) {
        printf("%10s [%9.4f ]\n", labels[i], B[i]);
    }
    
    // Compute eigenvalues of A (characteristic polynomial)
    // For a rough estimate, just check A[2][2] and A[3][2] (pitch dynamics)
    printf("\n========== ANALYSIS ==========\n");
    printf("Key entries for pitch dynamics:\n");
    printf("  A[pitch_dot][pitch] = %.4f (should be ~0 if pitch_dot = pitch_rate)\n", A[2][2]);
    printf("  A[pitch_dot][pitch_rate] = %.4f (should be ~1)\n", A[2][3]);
    printf("  A[prate_dot][pitch] = %.4f (gravity term 'a')\n", A[3][2]);
    printf("  A[prate_dot][pitch_rate] = %.4f (damping)\n", A[3][3]);
    printf("  B[prate_dot] = %.4f (input gain 'b')\n", B[3]);
    
    // Approximate open-loop pole from pitch dynamics
    double a_eff = A[3][2];
    if (a_eff > 0) {
        printf("\nOpen-loop unstable pole estimate: sqrt(%.2f) = %.2f rad/s\n", 
               a_eff, std::sqrt(a_eff));
    }
    
    // Compute fit quality (R²)
    double ss_tot[4] = {0}, ss_res[4] = {0};
    double mean_xdot[4] = {0};
    for (const auto& s : samples) {
        for (int i = 0; i < 4; i++) mean_xdot[i] += s.x_dot[i];
    }
    for (int i = 0; i < 4; i++) mean_xdot[i] /= n;
    
    for (const auto& s : samples) {
        double pred[4];
        for (int i = 0; i < 4; i++) {
            pred[i] = B[i] * s.u;
            for (int j = 0; j < 4; j++) {
                pred[i] += A[i][j] * s.x[j];
            }
            ss_res[i] += (s.x_dot[i] - pred[i]) * (s.x_dot[i] - pred[i]);
            ss_tot[i] += (s.x_dot[i] - mean_xdot[i]) * (s.x_dot[i] - mean_xdot[i]);
        }
    }
    
    printf("\nFit quality (R²):\n");
    for (int i = 0; i < 4; i++) {
        double r2 = 1.0 - ss_res[i] / (ss_tot[i] + 1e-10);
        printf("  %10s: R² = %.4f\n", labels[i], r2);
    }
    
    // Output for Python/LQR
    printf("\n========== PYTHON COPY-PASTE ==========\n");
    printf("A = np.array([\n");
    for (int i = 0; i < 4; i++) {
        printf("    [%12.6f, %12.6f, %12.6f, %12.6f],\n", 
               A[i][0], A[i][1], A[i][2], A[i][3]);
    }
    printf("])\n\n");
    printf("B = np.array([[%12.6f], [%12.6f], [%12.6f], [%12.6f]])\n",
           B[0], B[1], B[2], B[3]);
    
    mj_deleteModel(m);
    return 0;
}
