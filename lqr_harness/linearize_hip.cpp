#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string model_path = "../../myRobot/scene.xml";
    std::string keyframe = "r0";
    std::string hip_joint = "hip_L";
    std::string hip_joint_r = "hip_R";
    double hip_min = -0.27;
    double hip_max = 0.375;
    int steps = 7;
    double eps = 1e-6;
    std::string out_dir = "linearize_out";
    bool equilibrium_wheels = false;
    bool reduced_out = true;
    bool reduced_fd = false;
    bool reduced_no_x = false;
};

void usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  --model <path>     MuJoCo XML model (default: ../../myRobot/scene.xml)\n"
        "  --key <name>       Keyframe name (default: r0)\n"
        "  --hip <name>       Hip joint name (default: hip_L)\n"
        "  --hip-r <name>     Right hip joint name (default: hip_R)\n"
        "  --min <rad>        Hip min angle (default: -0.27)\n"
        "  --max <rad>        Hip max angle (default: 0.375)\n"
        "  --steps <int>      Number of samples (default: 7)\n"
        "  --eps <val>        Finite difference epsilon (default: 1e-6)\n"
        "  --out <dir>        Output directory (default: linearize_out)\n"
        "  --equilibrium-wheels  Solve wheel ctrl to hold pose (inverse dynamics)\n"
        "  --no-reduced       Do not write reduced A/B for MotionController\n"
        "  --reduced-fd       Build reduced A/B via finite difference on reduced state\n"
        "  --reduced-no-x     Also write reduced A/B without x state (v,theta,thetadot)\n"
        "  --help             Show this help\n",
        prog);
}

bool parse_args(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if (!std::strcmp(arg, "--help")) {
            usage(argv[0]);
            return false;
        }
        if (!std::strcmp(arg, "--model") && i + 1 < argc) {
            opt.model_path = argv[++i];
            continue;
        }
        if (!std::strcmp(arg, "--key") && i + 1 < argc) {
            opt.keyframe = argv[++i];
            continue;
        }
        if (!std::strcmp(arg, "--hip") && i + 1 < argc) {
            opt.hip_joint = argv[++i];
            continue;
        }
        if (!std::strcmp(arg, "--hip-r") && i + 1 < argc) {
            opt.hip_joint_r = argv[++i];
            continue;
        }
        if (!std::strcmp(arg, "--min") && i + 1 < argc) {
            opt.hip_min = std::atof(argv[++i]);
            continue;
        }
        if (!std::strcmp(arg, "--max") && i + 1 < argc) {
            opt.hip_max = std::atof(argv[++i]);
            continue;
        }
        if (!std::strcmp(arg, "--steps") && i + 1 < argc) {
            opt.steps = std::atoi(argv[++i]);
            continue;
        }
        if (!std::strcmp(arg, "--eps") && i + 1 < argc) {
            opt.eps = std::atof(argv[++i]);
            continue;
        }
        if (!std::strcmp(arg, "--out") && i + 1 < argc) {
            opt.out_dir = argv[++i];
            continue;
        }
        if (!std::strcmp(arg, "--equilibrium-wheels")) {
            opt.equilibrium_wheels = true;
            continue;
        }
        if (!std::strcmp(arg, "--no-reduced")) {
            opt.reduced_out = false;
            continue;
        }
        if (!std::strcmp(arg, "--reduced-fd")) {
            opt.reduced_fd = true;
            continue;
        }
        if (!std::strcmp(arg, "--reduced-no-x")) {
            opt.reduced_no_x = true;
            continue;
        }
        std::fprintf(stderr, "Unknown arg: %s\n", arg);
        usage(argv[0]);
        return false;
    }
    return true;
}

void write_matrix_csv(const std::string& path, const mjtNum* data, int rows, int cols) {
    std::ofstream out(path);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            out << data[r * cols + c];
            if (c + 1 < cols) out << ",";
        }
        out << "\n";
    }
}

void write_ctrl_csv(const std::string& path,
                    double hip,
                    const mjModel* m,
                    const mjData* d,
                    int act_hip_l, int act_wheel_l,
                    int act_hip_r, int act_wheel_r) {
    std::ofstream out(path);
    out << "hip,hip_L,wheel_L,hip_R,wheel_R,u_sum\n";
    mjtNum hip_l = (act_hip_l >= 0) ? d->ctrl[act_hip_l] : 0;
    mjtNum wheel_l = (act_wheel_l >= 0) ? d->ctrl[act_wheel_l] : 0;
    mjtNum hip_r = (act_hip_r >= 0) ? d->ctrl[act_hip_r] : 0;
    mjtNum wheel_r = (act_wheel_r >= 0) ? d->ctrl[act_wheel_r] : 0;
    mjtNum u_sum = wheel_l + wheel_r;
    out << hip << "," << hip_l << "," << wheel_l << "," << hip_r << "," << wheel_r << "," << u_sum << "\n";
}

void write_eq_csv(const std::string& path,
                  double hip,
                  double theta_eq,
                  double u_eq,
                  double tau_l,
                  double tau_r) {
    std::ofstream out(path);
    out << "hip,theta_eq,u_eq,tau_L,tau_R\n";
    out << hip << "," << theta_eq << "," << u_eq << "," << tau_l << "," << tau_r << "\n";
}

int actuator_id(const mjModel* m, const char* name) {
    return mj_name2id(m, mjOBJ_ACTUATOR, name);
}

int joint_id(const mjModel* m, const char* name) {
    return mj_name2id(m, mjOBJ_JOINT, name);
}

// Compute geometric equilibrium theta where COM is directly above wheel contact
// This is the angle where gravitational torque is minimized
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

    // Set hip actuator targets
    // For position actuators with gear: actuator_length = joint_angle * gear
    // Position servo drives: actuator_length = ctrl
    // Therefore: ctrl = joint_angle * gear
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

bool set_equilibrium_wheel_ctrl(mjModel* m, mjData* d,
                                int hip_id, int hip_r_id,
                                int act_hip_l, int act_hip_r,
                                int act_wheel_l, int act_wheel_r,
                                double hip_angle) {
    // Set hip position targets to hold angle
    // ctrl = joint_angle * gear (due to transmission scaling)
    if (act_hip_l >= 0) {
        mjtNum gear = m->actuator_gear[6 * act_hip_l];
        d->ctrl[act_hip_l] = hip_angle * gear;
    }
    if (act_hip_r >= 0) {
        mjtNum gear = m->actuator_gear[6 * act_hip_r];
        d->ctrl[act_hip_r] = hip_angle * gear;
    }

    // Zero velocities and accelerations for equilibrium.
    mju_zero(d->qvel, m->nv);
    mju_zero(d->qacc, m->nv);
    mju_zero(d->qfrc_applied, m->nv);
    mju_zero(d->xfrc_applied, 6 * m->nbody);

    // Inverse dynamics gives required generalized forces for qacc=0.
    mj_inverse(m, d);

    auto solve_wheel = [&](int act_id, int jnt_id) {
        if (act_id < 0 || jnt_id < 0) return;
        int dof = m->jnt_dofadr[jnt_id];
        if (dof < 0 || dof >= m->nv) return;
        // Desired joint torque.
        mjtNum tau = d->qfrc_inverse[dof];
        // Motor gain is stored in actuator_gear (first component).
        mjtNum gear = m->actuator_gear[6 * act_id];
        if (gear == 0) return;
        // For joint motors, actuator moment is 1, so ctrl = tau / gear.
        d->ctrl[act_id] = tau / gear;
    };

    int wheel_l_jnt = joint_id(m, "wheel_L");
    int wheel_r_jnt = joint_id(m, "wheel_R");
    solve_wheel(act_wheel_l, wheel_l_jnt);
    solve_wheel(act_wheel_r, wheel_r_jnt);
    return true;
}

static inline mjtNum pitch_from_state(const mjData* d) {
    const mjtNum qw = d->qpos[3];
    const mjtNum qy = d->qpos[5];
    return (mjtNum)(2.0 * std::atan2(qy, qw));
}

static inline void wheel_torques(const mjModel* m, const mjData* d,
                                 mjtNum* tau_l, mjtNum* tau_r) {
    if (tau_l) *tau_l = 0;
    if (tau_r) *tau_r = 0;
    int wheel_l_jnt = joint_id(m, "wheel_L");
    int wheel_r_jnt = joint_id(m, "wheel_R");
    if (wheel_l_jnt >= 0 && tau_l) {
        int dof = m->jnt_dofadr[wheel_l_jnt];
        if (dof >= 0 && dof < m->nv) *tau_l = d->qfrc_inverse[dof];
    }
    if (wheel_r_jnt >= 0 && tau_r) {
        int dof = m->jnt_dofadr[wheel_r_jnt];
        if (dof >= 0 && dof < m->nv) *tau_r = d->qfrc_inverse[dof];
    }
}

static inline void reduced_state(const mjData* d, mjtNum y[4]) {
    // x, xdot
    y[0] = d->qpos[0];
    // free joint qvel order: [ang x,y,z, lin x,y,z]
    y[1] = d->qvel[3];
    // pitch from quaternion (qw,qy)
    const mjtNum qw = d->qpos[3];
    const mjtNum qy = d->qpos[5];
    y[2] = (mjtNum)(2.0 * std::atan2(qy, qw));
    // pitch rate around Y
    y[3] = d->qvel[1];
}

static inline void reduced_state_no_x(const mjData* d, mjtNum y[3]) {
    // v, theta, thetadot
    y[0] = d->qvel[3];
    const mjtNum qw = d->qpos[3];
    const mjtNum qy = d->qpos[5];
    y[1] = (mjtNum)(2.0 * std::atan2(qy, qw));
    y[2] = d->qvel[1];
}

static inline void set_pitch_in_qpos(mjtNum* qpos, mjtNum theta) {
    // overwrite quaternion with pure pitch about Y (qw,qy)
    qpos[3] = std::cos(theta * 0.5);
    qpos[4] = 0.0;
    qpos[5] = std::sin(theta * 0.5);
    qpos[6] = 0.0;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt{};
    if (!parse_args(argc, argv, opt)) {
        return 1;
    }

    char error[1024] = {0};
    mjModel* m = mj_loadXML(opt.model_path.c_str(), nullptr, error, sizeof(error));
    if (!m) {
        std::fprintf(stderr, "Failed to load model: %s\n", error);
        return 1;
    }
    mjData* d = mj_makeData(m);
    if (!d) {
        std::fprintf(stderr, "Failed to allocate mjData\n");
        mj_deleteModel(m);
        return 1;
    }
    mjData* dtmp = nullptr;
    if (opt.reduced_out && opt.reduced_fd) {
        dtmp = mj_makeData(m);
        if (!dtmp) {
            std::fprintf(stderr, "Failed to allocate mjData for reduced-fd\n");
            mj_deleteData(d);
            mj_deleteModel(m);
            return 1;
        }
    }

    if (m->opt.integrator != mjINT_EULER) {
        std::printf("Switching integrator to Euler for mjd_transitionFD\n");
        m->opt.integrator = mjINT_EULER;
    }

    int key_id = mj_name2id(m, mjOBJ_KEY, opt.keyframe.c_str());
    if (key_id >= 0) {
        mj_resetDataKeyframe(m, d, key_id);
    } else {
        mj_resetData(m, d);
    }
    mj_forward(m, d);

    int hip_id = mj_name2id(m, mjOBJ_JOINT, opt.hip_joint.c_str());
    int hip_r_id = mj_name2id(m, mjOBJ_JOINT, opt.hip_joint_r.c_str());
    if (hip_id < 0) {
        std::fprintf(stderr, "Hip joint '%s' not found\n", opt.hip_joint.c_str());
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }

    std::filesystem::create_directories(opt.out_dir);

    const int state_dim = 2 * m->nv + m->na;
    const int a_rows = state_dim;
    const int a_cols = state_dim;
    const int b_rows = state_dim;
    const int b_cols = m->nu;

    std::vector<mjtNum> A(a_rows * a_cols);
    std::vector<mjtNum> B(b_rows * b_cols);

    std::printf("Linearizing model '%s'\n", opt.model_path.c_str());
    std::printf("State dim: %d (2*nv + na), Control dim: %d\n", state_dim, m->nu);
    std::printf("Hip joint: %s, Right hip: %s\n",
                opt.hip_joint.c_str(), opt.hip_joint_r.c_str());
    std::printf("Output directory: %s\n", opt.out_dir.c_str());
    if (opt.equilibrium_wheels) {
        std::printf("Equilibrium wheel ctrl enabled (inverse dynamics)\n");
    }

    for (int i = 0; i < opt.steps; ++i) {
        double t = (opt.steps <= 1) ? 0.0 : (double)i / (double)(opt.steps - 1);
        double hip = opt.hip_min + t * (opt.hip_max - opt.hip_min);

        d->qpos[m->jnt_qposadr[hip_id]] = hip;
        if (hip_r_id >= 0) {
            d->qpos[m->jnt_qposadr[hip_r_id]] = hip;
        }
        mj_forward(m, d);

        if (opt.equilibrium_wheels) {
            int act_hip_l = actuator_id(m, "hip_L");
            int act_hip_r = actuator_id(m, "hip_R");
            int act_wheel_l = actuator_id(m, "wheel_L");
            int act_wheel_r = actuator_id(m, "wheel_R");

            // Compute COM-aligned equilibrium theta (geometric calculation)
            double theta_eq = compute_geometric_theta_eq(m, d, hip_id, hip_r_id, hip);

            std::printf("  hip=%.4f  theta_eq=%.6f (%.2f°)\n", hip, theta_eq, theta_eq * 57.3);

            // Reset to equilibrium state
            int qpos_hip_l = m->jnt_qposadr[hip_id];
            int qpos_hip_r = m->jnt_qposadr[hip_r_id];
            d->qpos[qpos_hip_l] = hip;
            d->qpos[qpos_hip_r] = hip;
            set_pitch_in_qpos(d->qpos, theta_eq);
            mj_forward(m, d);

            set_equilibrium_wheel_ctrl(m, d,
                                       hip_id, hip_r_id,
                                       act_hip_l, act_hip_r,
                                       act_wheel_l, act_wheel_r,
                                       hip);
            mj_forward(m, d);

            // Compute equilibrium torques at this pose.
            mj_inverse(m, d);
            mjtNum tau_l = 0.0;
            mjtNum tau_r = 0.0;
            wheel_torques(m, d, &tau_l, &tau_r);
            mjtNum u_eq = tau_l + tau_r;

            std::ostringstream ctag;
            ctag.setf(std::ios::fixed);
            ctag.precision(6);
            ctag << "hip_" << hip;
            std::string c_path = opt.out_dir + "/ctrl_" + ctag.str() + ".csv";
            write_ctrl_csv(c_path, hip, m, d, act_hip_l, act_wheel_l, act_hip_r, act_wheel_r);

            std::string e_path = opt.out_dir + "/eq_" + ctag.str() + ".csv";
            write_eq_csv(e_path, hip, theta_eq, u_eq, tau_l, tau_r);
        }

        mjd_transitionFD(m, d, opt.eps, 1, A.data(), B.data(), nullptr, nullptr);

        std::ostringstream tag;
        tag.setf(std::ios::fixed);
        tag.precision(6);
        tag << "hip_" << hip;

        std::string a_path = opt.out_dir + "/A_" + tag.str() + ".csv";
        std::string b_path = opt.out_dir + "/B_" + tag.str() + ".csv";
        write_matrix_csv(a_path, A.data(), a_rows, a_cols);
        write_matrix_csv(b_path, B.data(), b_rows, b_cols);

        if (opt.reduced_out) {
            // Reduced state for MotionController: [x, xdot, theta, thetadot]
            // In mjd_transitionFD, the state is [qpos_tangent (nv), qvel (nv), act].
            // For a free joint, mj_differentiatePos tangent order is:
            //   [trans_x, trans_y, trans_z, rot_x, rot_y, rot_z]
            // (translations FIRST, then rotations)
            // qvel order matches: [trans_x, trans_y, trans_z, rot_x, rot_y, rot_z]
            const int free_id = joint_id(m, "torso_freejoint");
            const int base_dof = (free_id >= 0) ? m->jnt_dofadr[free_id] : 0;
            const int idx_trans_x = base_dof + 0;  // forward position
            const int idx_rot_y  = base_dof + 4;   // pitch angle (Y rotation)
            const int idx_x = idx_trans_x;
            const int idx_xdot = m->nv + idx_trans_x;
            const int idx_theta = idx_rot_y;
            const int idx_thetadot = m->nv + idx_rot_y;

            // Build C and S mappings for small-angle approximation.
            // x_red = C * x_full, x_full ≈ S * x_red.
            std::vector<mjtNum> C(4 * state_dim, 0.0);
            std::vector<mjtNum> S(state_dim * 4, 0.0);

            // x
            C[0 * state_dim + idx_x] = 1.0;
            S[idx_x * 4 + 0] = 1.0;
            // xdot
            C[1 * state_dim + idx_xdot] = 1.0;
            S[idx_xdot * 4 + 1] = 1.0;
            // theta (pitch about Y — tangent[4] matches Euler pitch directly)
            C[2 * state_dim + idx_theta] = 1.0;
            S[idx_theta * 4 + 2] = 1.0;
            // thetadot
            C[3 * state_dim + idx_thetadot] = 1.0;
            S[idx_thetadot * 4 + 3] = 1.0;

            // Abar = C * A * S
            std::vector<mjtNum> Abar(4 * 4, 0.0);
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
                    Abar[r * 4 + c] = (mjtNum)sum;
                }
            }

            // Bbar = C * B * U, where U maps u_sum to wheel actuators.
            // Both wheels receive same ctrl value (empirically verified they
            // produce the same direction of pitch when given same ctrl sign).
            std::vector<mjtNum> Bbar(4, 0.0);
            int act_wheel_l = actuator_id(m, "wheel_L");
            int act_wheel_r = actuator_id(m, "wheel_R");
            for (int r = 0; r < 4; ++r) {
                double sum = 0.0;
                for (int k = 0; k < state_dim; ++k) {
                    double ca = C[r * state_dim + k];
                    if (ca == 0.0) continue;
                    if (act_wheel_l >= 0) sum += ca * B[k * m->nu + act_wheel_l];
                    if (act_wheel_r >= 0) sum += ca * B[k * m->nu + act_wheel_r];
                }
                Bbar[r] = (mjtNum)sum;
            }
            // NOTE: Bbar stays in ctrl-units (not converted to torque).
            // The controller outputs ctrl directly (d->ctrl = u_sum),
            // so LQR gains must be in ctrl-space, matching the MuJoCo B matrix.

            std::string ar_path = opt.out_dir + "/Ared_" + tag.str() + ".csv";
            std::string br_path = opt.out_dir + "/Bred_" + tag.str() + ".csv";
            if (opt.reduced_fd) {
                if (!dtmp) {
                    std::fprintf(stderr, "reduced-fd requested but dtmp is null\n");
                } else {
                    std::vector<mjtNum> qpos0(m->nq);
                    std::vector<mjtNum> qvel0(m->nv);
                    std::vector<mjtNum> act0(m->na);
                    std::vector<mjtNum> ctrl0(m->nu);
                    mju_copy(qpos0.data(), d->qpos, m->nq);
                    mju_copy(qvel0.data(), d->qvel, m->nv);
                    if (m->na) mju_copy(act0.data(), d->act, m->na);
                    if (m->nu) mju_copy(ctrl0.data(), d->ctrl, m->nu);

                    auto step_with = [&](auto perturb_fn, mjtNum yout[4]) {
                        mju_copy(dtmp->qpos, qpos0.data(), m->nq);
                        mju_copy(dtmp->qvel, qvel0.data(), m->nv);
                        if (m->na) mju_copy(dtmp->act, act0.data(), m->na);
                        if (m->nu) mju_copy(dtmp->ctrl, ctrl0.data(), m->nu);
                        dtmp->time = d->time;
                        perturb_fn();
                        mj_forward(m, dtmp);
                        mj_step(m, dtmp);
                        reduced_state(dtmp, yout);
                    };

                    // Baseline next-step reduced state (unperturbed)
                    mjtNum y_next0[4] = {0};
                    step_with([&]() {}, y_next0);

                    // Ared via finite differences on reduced state
                    for (int j = 0; j < 4; ++j) {
                        mjtNum y1[4] = {0};
                        step_with([&]() {
                            if (j == 0) {
                                dtmp->qpos[0] += opt.eps;
                            } else if (j == 1) {
                                dtmp->qvel[3] += opt.eps;
                            } else if (j == 2) {
                                // current pitch from baseline state
                                mjtNum qw = qpos0[3];
                                mjtNum qy = qpos0[5];
                                mjtNum theta0 = (mjtNum)(2.0 * std::atan2(qy, qw));
                                set_pitch_in_qpos(dtmp->qpos, theta0 + opt.eps);
                            } else if (j == 3) {
                                dtmp->qvel[1] += opt.eps;
                            }
                        }, y1);
                        for (int r = 0; r < 4; ++r) {
                            Abar[r * 4 + j] = (y1[r] - y_next0[r]) / opt.eps;
                        }
                    }

                    // Bred via finite difference on u_sum torque
                    int act_wheel_l = actuator_id(m, "wheel_L");
                    int act_wheel_r = actuator_id(m, "wheel_R");
                    mjtNum gear_l = (act_wheel_l >= 0) ? m->actuator_gear[6 * act_wheel_l] : 0.0;
                    mjtNum gear_r = (act_wheel_r >= 0) ? m->actuator_gear[6 * act_wheel_r] : 0.0;
                    mjtNum y1[4] = {0};
                    step_with([&]() {
                        // Apply delta torque equally to both wheels.
                        mjtNum delta = opt.eps;
                        if (act_wheel_l >= 0 && gear_l != 0) {
                            dtmp->ctrl[act_wheel_l] += (delta * 0.5) / gear_l;
                        }
                        if (act_wheel_r >= 0 && gear_r != 0) {
                            dtmp->ctrl[act_wheel_r] += (delta * 0.5) / gear_r;
                        }
                    }, y1);
                    for (int r = 0; r < 4; ++r) {
                        Bbar[r] = (y1[r] - y_next0[r]) / opt.eps;
                    }

                    write_matrix_csv(ar_path, Abar.data(), 4, 4);
                    write_matrix_csv(br_path, Bbar.data(), 4, 1);
                }
            } else {
                write_matrix_csv(ar_path, Abar.data(), 4, 4);
                write_matrix_csv(br_path, Bbar.data(), 4, 1);
            }

            if (opt.reduced_no_x) {
                std::string ar3_path = opt.out_dir + "/Ared3_" + tag.str() + ".csv";
                std::string br3_path = opt.out_dir + "/Bred3_" + tag.str() + ".csv";

                if (opt.reduced_fd) {
                    if (!dtmp) {
                        std::fprintf(stderr, "reduced-fd requested but dtmp is null\n");
                    } else {
                        std::vector<mjtNum> qpos0(m->nq);
                        std::vector<mjtNum> qvel0(m->nv);
                        std::vector<mjtNum> act0(m->na);
                        std::vector<mjtNum> ctrl0(m->nu);
                        mju_copy(qpos0.data(), d->qpos, m->nq);
                        mju_copy(qvel0.data(), d->qvel, m->nv);
                        if (m->na) mju_copy(act0.data(), d->act, m->na);
                        if (m->nu) mju_copy(ctrl0.data(), d->ctrl, m->nu);

                        auto step_with = [&](auto perturb_fn, mjtNum yout[3]) {
                            mju_copy(dtmp->qpos, qpos0.data(), m->nq);
                            mju_copy(dtmp->qvel, qvel0.data(), m->nv);
                            if (m->na) mju_copy(dtmp->act, act0.data(), m->na);
                            if (m->nu) mju_copy(dtmp->ctrl, ctrl0.data(), m->nu);
                            dtmp->time = d->time;
                            perturb_fn();
                            mj_forward(m, dtmp);
                            mj_step(m, dtmp);
                            reduced_state_no_x(dtmp, yout);
                        };

                        mjtNum y_next0[3] = {0};
                        step_with([&]() {}, y_next0);

                        // Ared3 via finite differences on [v, theta, thetadot]
                        std::vector<mjtNum> A3(3 * 3, 0.0);
                        for (int j = 0; j < 3; ++j) {
                            mjtNum y1[3] = {0};
                            step_with([&]() {
                                if (j == 0) {
                                    dtmp->qvel[3] += opt.eps;
                                } else if (j == 1) {
                                    mjtNum qw = qpos0[3];
                                    mjtNum qy = qpos0[5];
                                    mjtNum theta0 = (mjtNum)(2.0 * std::atan2(qy, qw));
                                    set_pitch_in_qpos(dtmp->qpos, theta0 + opt.eps);
                                } else if (j == 2) {
                                    dtmp->qvel[1] += opt.eps;
                                }
                            }, y1);
                            for (int r = 0; r < 3; ++r) {
                                A3[r * 3 + j] = (y1[r] - y_next0[r]) / opt.eps;
                            }
                        }

                        // Bred3 via finite difference on u_sum torque
                        int act_wheel_l = actuator_id(m, "wheel_L");
                        int act_wheel_r = actuator_id(m, "wheel_R");
                        mjtNum gear_l = (act_wheel_l >= 0) ? m->actuator_gear[6 * act_wheel_l] : 0.0;
                        mjtNum gear_r = (act_wheel_r >= 0) ? m->actuator_gear[6 * act_wheel_r] : 0.0;
                        mjtNum y1[3] = {0};
                        step_with([&]() {
                            mjtNum delta = opt.eps;
                            if (act_wheel_l >= 0 && gear_l != 0) {
                                dtmp->ctrl[act_wheel_l] += (delta * 0.5) / gear_l;
                            }
                            if (act_wheel_r >= 0 && gear_r != 0) {
                                dtmp->ctrl[act_wheel_r] += (delta * 0.5) / gear_r;
                            }
                        }, y1);
                        std::vector<mjtNum> B3(3, 0.0);
                        for (int r = 0; r < 3; ++r) {
                            B3[r] = (y1[r] - y_next0[r]) / opt.eps;
                        }

                        write_matrix_csv(ar3_path, A3.data(), 3, 3);
                        write_matrix_csv(br3_path, B3.data(), 3, 1);
                    }
                } else {
                    // Build linear mapping for 3-state: [v, theta, thetadot]
                    const int free_id = joint_id(m, "torso_freejoint");
                    const int base_dof = (free_id >= 0) ? m->jnt_dofadr[free_id] : 0;
                    const int idx_trans_x = base_dof + 0;
                    const int idx_rot_y  = base_dof + 4;  // pitch
                    const int idx_xdot = m->nv + idx_trans_x;
                    const int idx_theta = idx_rot_y;
                    const int idx_thetadot = m->nv + idx_rot_y;

                    std::vector<mjtNum> C3(3 * state_dim, 0.0);
                    std::vector<mjtNum> S3(state_dim * 3, 0.0);

                    // v (xdot)
                    C3[0 * state_dim + idx_xdot] = 1.0;
                    S3[idx_xdot * 3 + 0] = 1.0;

                    // theta (pitch — tangent[4] matches Euler pitch)
                    C3[1 * state_dim + idx_theta] = 1.0;
                    S3[idx_theta * 3 + 1] = 1.0;

                    // thetadot
                    C3[2 * state_dim + idx_thetadot] = 1.0;
                    S3[idx_thetadot * 3 + 2] = 1.0;

                    std::vector<mjtNum> A3(3 * 3, 0.0);
                    for (int r = 0; r < 3; ++r) {
                        for (int c = 0; c < 3; ++c) {
                            double sum = 0.0;
                            for (int k = 0; k < state_dim; ++k) {
                                double ca = C3[r * state_dim + k];
                                if (ca == 0.0) continue;
                                for (int j = 0; j < state_dim; ++j) {
                                    double as = A[k * state_dim + j];
                                    double s = S3[j * 3 + c];
                                    if (s == 0.0) continue;
                                    sum += ca * as * s;
                                }
                            }
                            A3[r * 3 + c] = (mjtNum)sum;
                        }
                    }

                    std::vector<mjtNum> B3(3, 0.0);
                    int act_wheel_l = actuator_id(m, "wheel_L");
                    int act_wheel_r = actuator_id(m, "wheel_R");
                    for (int r = 0; r < 3; ++r) {
                        double sum = 0.0;
                        for (int k = 0; k < state_dim; ++k) {
                            double ca = C3[r * state_dim + k];
                            if (ca == 0.0) continue;
                            if (act_wheel_l >= 0) sum += ca * B[k * m->nu + act_wheel_l];
                            if (act_wheel_r >= 0) sum += ca * B[k * m->nu + act_wheel_r];
                        }
                        B3[r] = (mjtNum)sum;
                    }
                    // NOTE: mjd_transitionFD B matrix is already in ctrl-space,
                    // so NO gear division needed here.

                    write_matrix_csv(ar3_path, A3.data(), 3, 3);
                    write_matrix_csv(br3_path, B3.data(), 3, 1);
                }
            }
        }
    }

    if (dtmp) {
        mj_deleteData(dtmp);
    }
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
