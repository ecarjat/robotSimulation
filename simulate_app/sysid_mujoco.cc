// Use MuJoCo's finite-difference Jacobians for linearization
// This handles the full complex dynamics (5-bar linkage, etc.)

#include <mujoco/mujoco.h>
#include <cstdio>
#include <cmath>
#include <cstring>

double get_pitch(const mjData* d) {
    double qw = d->qpos[3], qx = d->qpos[4], qy = d->qpos[5], qz = d->qpos[6];
    return std::asin(2.0 * (qw * qy - qz * qx));
}

void print_matrix(const char* name, const double* M, int rows, int cols, 
                  const char** row_labels = nullptr, const char** col_labels = nullptr) {
    printf("%s (%dx%d):\n", name, rows, cols);
    if (col_labels) {
        printf("         ");
        for (int j = 0; j < cols; j++) printf("%10s", col_labels[j]);
        printf("\n");
    }
    for (int i = 0; i < rows; i++) {
        if (row_labels) printf("%8s ", row_labels[i]);
        else printf("  ");
        for (int j = 0; j < cols; j++) {
            printf("%10.4f", M[i * cols + j]);
        }
        printf("\n");
    }
}

int main(int argc, char** argv) {
    const char* model_path = "myRobot/scene.xml";
    if (argc > 1) model_path = argv[1];

    char error[1024];
    mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m) { printf("Error: %s\n", error); return 1; }

    mjData* d = mj_makeData(m);
    
    printf("Model info:\n");
    printf("  nq (pos dim)   = %d\n", m->nq);
    printf("  nv (vel dim)   = %d\n", m->nv);
    printf("  nu (ctrl dim)  = %d\n", m->nu);
    printf("  timestep       = %.4f\n\n", m->opt.timestep);
    
    // Print joint names
    printf("Joints (nv=%d):\n", m->nv);
    for (int i = 0; i < m->njnt; i++) {
        const char* name = mj_id2name(m, mjOBJ_JOINT, i);
        printf("  [%d] %s\n", i, name ? name : "(unnamed)");
    }
    
    // Print actuator names
    printf("\nActuators (nu=%d):\n", m->nu);
    for (int i = 0; i < m->nu; i++) {
        const char* name = mj_id2name(m, mjOBJ_ACTUATOR, i);
        printf("  [%d] %s\n", i, name ? name : "(unnamed)");
    }
    
    // Reset to keyframe (standing position)
    if (m->nkey > 0) {
        mj_resetDataKeyframe(m, d, 0);
        printf("\nReset to keyframe 0\n");
    }
    mj_forward(m, d);
    
    printf("\nEquilibrium state:\n");
    printf("  pitch = %.4f rad (%.2f°)\n", get_pitch(d), get_pitch(d) * 57.3);
    printf("  qpos[0] (x) = %.4f\n", d->qpos[0]);
    
    // Allocate Jacobians
    // State: [qpos; qvel] → dim = nq + nv
    // For linearization: x_next = A*x + B*u
    // MuJoCo gives us: dx_next/dx (state Jacobian) and dx_next/du (control Jacobian)
    
    int nstate = m->nq + m->nv;
    double* A = new double[nstate * nstate];
    double* B = new double[nstate * m->nu];
    double* C = new double[nstate * nstate];  // sensor Jacobian (unused)
    double* D = new double[nstate * m->nu];   // sensor Jacobian (unused)
    
    // Use finite differences to compute Jacobians
    // mjd_transitionFD computes discrete-time A, B at current state
    double eps = 1e-6;
    mjtByte* flg_actuation = new mjtByte[m->nu];
    for (int i = 0; i < m->nu; i++) flg_actuation[i] = 1;
    
    // Centered differences for better accuracy
    mjd_transitionFD(m, d, eps, /*centered=*/1, A, B, C, D);
    
    printf("\n========== MUJOCO LINEARIZATION ==========\n");
    printf("Discrete-time model: x[k+1] = A*x[k] + B*u[k]\n");
    printf("State dim: %d (nq=%d, nv=%d), Control dim: %d\n\n", nstate, m->nq, m->nv, m->nu);
    
    // The full matrices are large. Let's extract the relevant parts.
    // We care about: body x position (qvel[0]), body pitch (qvel[1]), 
    // and wheel actuators.
    
    // Find relevant indices
    // qpos: [x, y, z, qw, qx, qy, qz, hip_L, passive1, wheel_L, passive2, hip_R, ...]
    // qvel: [vx, vy, vz, wx, wy, wz, hip_L_dot, ...]
    // For a freejoint body: first 3 qvel are linear vel, next 3 are angular vel
    
    // Extract 4x4 submatrix for [x, vx, pitch_approx, pitch_rate]
    // This is tricky because pitch is encoded in quaternion...
    
    // Let's look at the velocity-level dynamics
    // The nv x nv block of A (lower-right) gives d(qvel_next)/d(qvel)
    // The nv x nu block of B (lower half) gives d(qvel_next)/d(ctrl)
    
    printf("--- Velocity dynamics (lower-right block of A) ---\n");
    printf("Indices: [0]=vx, [1]=vy, [2]=vz, [3]=wx, [4]=wy (pitch_rate), [5]=wz\n\n");
    
    // Print 6x6 corner of A (velocity to velocity)
    printf("A[nq:nq+6, nq:nq+6] (d(vel_next)/d(vel)):\n");
    printf("          vx        vy        vz        wx        wy        wz\n");
    const char* vel_labels[] = {"vx", "vy", "vz", "wx", "wy", "wz"};
    for (int i = 0; i < 6; i++) {
        printf("%4s ", vel_labels[i]);
        for (int j = 0; j < 6; j++) {
            printf("%10.4f", A[(m->nq + i) * nstate + (m->nq + j)]);
        }
        printf("\n");
    }
    
    // Print effect of position on velocity (coupling)
    printf("\nA[nq:nq+6, 0:7] (d(vel_next)/d(pos)) - quat indices 3-6:\n");
    printf("           x         y         z        qw        qx        qy        qz\n");
    for (int i = 0; i < 6; i++) {
        printf("%4s ", vel_labels[i]);
        for (int j = 0; j < 7; j++) {
            printf("%10.4f", A[(m->nq + i) * nstate + j]);
        }
        printf("\n");
    }
    
    // Print B matrix (control effect on velocity)
    printf("\n--- Control Jacobian (d(vel_next)/d(ctrl)) ---\n");
    printf("B[nq:nq+6, :]:\n");
    printf("       ");
    for (int j = 0; j < m->nu; j++) {
        const char* name = mj_id2name(m, mjOBJ_ACTUATOR, j);
        printf("%10s", name ? name : "?");
    }
    printf("\n");
    for (int i = 0; i < 6; i++) {
        printf("%4s ", vel_labels[i]);
        for (int j = 0; j < m->nu; j++) {
            printf("%10.6f", B[(m->nq + i) * m->nu + j]);
        }
        printf("\n");
    }
    
    // Extract effective 4-state model for [x, vx, theta, thetaDot]
    // Approximate theta ≈ qy (for small angles, sin(theta/2) ≈ theta/2)
    // So d(theta)/d(qy) ≈ 2
    
    printf("\n========== EXTRACTED 4-STATE MODEL ==========\n");
    printf("Approximation: theta ≈ 2*qy for small angles\n");
    printf("State: [x, vx, theta, thetaDot] ≈ [qpos[0], qvel[0], 2*qpos[5], qvel[4]]\n\n");
    
    // Indices in full state:
    // x -> qpos[0] -> idx 0
    // vx -> qvel[0] -> idx nq+0
    // qy -> qpos[5] -> idx 5  (theta ≈ 2*qy)
    // wy -> qvel[4] -> idx nq+4
    
    // We want the 4x4 dynamics matrix for these states
    int idx[4] = {0, (int)m->nq + 0, 5, (int)m->nq + 4};
    double A4[4][4];
    
    printf("A (4x4 extracted):\n");
    printf("          x        vx     theta  thetaDot\n");
    const char* state4[] = {"x", "vx", "theta", "thDot"};
    for (int i = 0; i < 4; i++) {
        printf("%6s ", state4[i]);
        for (int j = 0; j < 4; j++) {
            double val = A[idx[i] * nstate + idx[j]];
            // Scale qy columns by 2 (theta = 2*qy)
            if (j == 2) val *= 2.0;
            // Scale qy rows by 0.5 (d(theta)/dt = 2*d(qy)/dt, but we measure thetaDot directly)
            // Actually for row 2 (theta), we need: d(theta_next)/d(state)
            // theta_next ≈ 2*qy_next, so d(theta_next)/d(x) = 2*d(qy_next)/d(x)
            if (i == 2) val *= 2.0;
            A4[i][j] = val;
            printf("%10.4f", val);
        }
        printf("\n");
    }
    
    // Extract B for wheel actuators
    // Find wheel actuator indices
    int wL = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_L");
    int wR = mj_name2id(m, mjOBJ_ACTUATOR, "wheel_R");
    printf("\nWheel actuator indices: L=%d, R=%d\n", wL, wR);
    
    printf("\nB (4x2 for wheels):\n");
    printf("        wheel_L   wheel_R\n");
    for (int i = 0; i < 4; i++) {
        printf("%6s ", state4[i]);
        double bL = B[idx[i] * m->nu + wL];
        double bR = B[idx[i] * m->nu + wR];
        if (i == 2) { bL *= 2.0; bR *= 2.0; }  // theta scaling
        printf("%10.6f%10.6f\n", bL, bR);
    }
    
    // Combined wheel input (u = ctrl_L + ctrl_R gives torque = -2*u due to gear=-1)
    printf("\nFor combined wheel input u (total torque = -2*u):\n");
    printf("B (4x1):\n");
    for (int i = 0; i < 4; i++) {
        double bL = B[idx[i] * m->nu + wL];
        double bR = B[idx[i] * m->nu + wR];
        if (i == 2) { bL *= 2.0; bR *= 2.0; }
        printf("%6s  %10.6f\n", state4[i], bL + bR);
    }
    
    // Convert to continuous time (approximate)
    // A_continuous ≈ (A_discrete - I) / dt
    double dt = m->opt.timestep;
    printf("\n========== CONTINUOUS-TIME (approx) ==========\n");
    printf("A_c ≈ (A_d - I) / dt\n\n");
    
    printf("A_c = np.array([\n");
    for (int i = 0; i < 4; i++) {
        printf("    [");
        for (int j = 0; j < 4; j++) {
            double val = (A4[i][j] - (i == j ? 1.0 : 0.0)) / dt;
            printf("%12.4f,", val);
        }
        printf("],\n");
    }
    printf("])\n");
    
    printf("\nB_c (combined wheels):\n");
    printf("B_c = np.array([");
    for (int i = 0; i < 4; i++) {
        double bL = B[idx[i] * m->nu + wL];
        double bR = B[idx[i] * m->nu + wR];
        if (i == 2) { bL *= 2.0; bR *= 2.0; }
        double bc = (bL + bR) / dt;
        printf("[%12.6f], ", bc);
    }
    printf("]).T\n");
    
    delete[] A;
    delete[] B;
    delete[] C;
    delete[] D;
    delete[] flg_actuation;
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
