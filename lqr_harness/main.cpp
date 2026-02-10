#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <mujoco/mujoco.h>

#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "MotionController.h"
#include "StateEstimate.h"
#include "ekf/BalancerEKF.h"

namespace {

struct Options {
    std::string model_path = "../../robot_planar.xml";
    std::string keyframe = "r50";
    double sim_time = 5.0;
    double print_dt = 0.1;
    bool hold_hips = true;
    bool hip_target_mid = true;
    bool use_ekf = true;
    bool headless = false;
    bool realtime = true;
    double hip_kp = 50.0;
    double hip_kd = 2.0;
    double u_scale = 1.0;
};

void usage(const char* prog)
{
    std::printf(
        "Usage: %s [options]\n"
        "  --model <path>       MuJoCo XML model (default: ../../robot_planar.xml)\n"
        "  --key <name>         Keyframe name (default: r50)\n"
        "  --time <sec>         Simulation duration (default: 5.0)\n"
        "  --print <sec>        Print period, 0 to disable (default: 0.1)\n"
        "  --no-hold-hips       Disable hip hold (PD) torque\n"
        "  --hip-target-key     Hold hips at keyframe angles (default: mid-range)\n"
        "  --no-ekf             Disable EKF (use direct theta/xDot)\n"
        "  --headless           Disable viewer (no rendering)\n"
        "  --fast               Run as fast as possible (no real-time sync)\n"
        "  --hip-kp <val>       Hip hold Kp (default: 50)\n"
        "  --hip-kd <val>       Hip hold Kd (default: 2)\n"
        "  --u-scale <val>      Scale LQR current to actuator ctrl (default: 1.0)\n"
        "  --help               Show this help\n",
        prog);
}

bool parse_args(int argc, char** argv, Options& opt)
{
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
        if (!std::strcmp(arg, "--time") && i + 1 < argc) {
            opt.sim_time = std::atof(argv[++i]);
            continue;
        }
        if (!std::strcmp(arg, "--print") && i + 1 < argc) {
            opt.print_dt = std::atof(argv[++i]);
            continue;
        }
        if (!std::strcmp(arg, "--no-hold-hips")) {
            opt.hold_hips = false;
            continue;
        }
        if (!std::strcmp(arg, "--hip-target-key")) {
            opt.hip_target_mid = false;
            continue;
        }
        if (!std::strcmp(arg, "--no-ekf")) {
            opt.use_ekf = false;
            continue;
        }
        if (!std::strcmp(arg, "--headless")) {
            opt.headless = true;
            continue;
        }
        if (!std::strcmp(arg, "--fast")) {
            opt.realtime = false;
            continue;
        }
        if (!std::strcmp(arg, "--hip-kp") && i + 1 < argc) {
            opt.hip_kp = std::atof(argv[++i]);
            continue;
        }
        if (!std::strcmp(arg, "--hip-kd") && i + 1 < argc) {
            opt.hip_kd = std::atof(argv[++i]);
            continue;
        }
        if (!std::strcmp(arg, "--u-scale") && i + 1 < argc) {
            opt.u_scale = std::atof(argv[++i]);
            continue;
        }

        std::fprintf(stderr, "Unknown arg: %s\n", arg);
        usage(argv[0]);
        return false;
    }
    return true;
}

int find_id(const mjModel* m, int type, const char* name)
{
    return mj_name2id(m, type, name);
}

double clamp_ctrl(const mjModel* m, int act_id, double u)
{
    if (act_id < 0) {
        return 0.0;
    }
    if (m->actuator_ctrllimited[act_id]) {
        const double lo = m->actuator_ctrlrange[2 * act_id + 0];
        const double hi = m->actuator_ctrlrange[2 * act_id + 1];
        if (u < lo) return lo;
        if (u > hi) return hi;
    }
    return u;
}

void warn_ctrlrange(const mjModel* m, int act_id, const char* name)
{
    if (act_id < 0) {
        std::fprintf(stderr, "Warning: actuator '%s' not found\n", name);
        return;
    }
    if (m->actuator_ctrllimited[act_id]) {
        double lo = m->actuator_ctrlrange[2 * act_id + 0];
        double hi = m->actuator_ctrlrange[2 * act_id + 1];
        if (hi - lo < 2.0) {
            std::fprintf(stderr,
                         "Warning: %s ctrlrange [%.3f, %.3f] is narrow; "
                         "use --u-scale or widen ctrlrange in XML.\n",
                         name, lo, hi);
        }
    }
}

bool read_sensor_vec3(const mjModel* m, const mjData* d, int sensor_id, mjtNum out[3])
{
    if (sensor_id < 0) {
        return false;
    }
    int adr = m->sensor_adr[sensor_id];
    int dim = m->sensor_dim[sensor_id];
    if (dim < 3) {
        return false;
    }
    out[0] = d->sensordata[adr + 0];
    out[1] = d->sensordata[adr + 1];
    out[2] = d->sensordata[adr + 2];
    return true;
}

double hip_target_from_range(const mjModel* m, int jnt_id, double fallback)
{
    if (jnt_id < 0) {
        return fallback;
    }
    if (m->jnt_limited[jnt_id]) {
        double lo = m->jnt_range[2 * jnt_id + 0];
        double hi = m->jnt_range[2 * jnt_id + 1];
        return 0.5 * (lo + hi);
    }
    return fallback;
}

bool is_servo_actuator(const mjModel* m, int act_id)
{
    if (act_id < 0) {
        return false;
    }
    return m->actuator_biastype[act_id] != mjBIAS_NONE;
}

}  // namespace

int main(int argc, char** argv)
{
    Options opt{};
    if (!parse_args(argc, argv, opt)) {
        return 1;
    }

    GLFWwindow* window = nullptr;
    mjvCamera cam;
    mjvOption vopt;
    mjvScene scn;
    mjrContext con;
    bool viewer_ready = false;

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

    int key_id = find_id(m, mjOBJ_KEY, opt.keyframe.c_str());
    if (key_id >= 0) {
        mj_resetDataKeyframe(m, d, key_id);
    } else {
        std::fprintf(stderr, "Keyframe '%s' not found, using default reset\n",
                     opt.keyframe.c_str());
        mj_resetData(m, d);
    }
    mj_forward(m, d);

    if (!opt.headless) {
        if (!glfwInit()) {
            std::fprintf(stderr, "Failed to initialize GLFW\n");
            mj_deleteData(d);
            mj_deleteModel(m);
            return 1;
        }
        window = glfwCreateWindow(1200, 900, "MuJoCo LQR Harness", nullptr, nullptr);
        if (!window) {
            std::fprintf(stderr, "Failed to create GLFW window\n");
            glfwTerminate();
            mj_deleteData(d);
            mj_deleteModel(m);
            return 1;
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        mjv_defaultCamera(&cam);
        mjv_defaultOption(&vopt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);
        mjv_makeScene(m, &scn, 1000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);
        viewer_ready = true;
    }

    const int torso_id = find_id(m, mjOBJ_BODY, "torso");
    if (torso_id < 0) {
        std::fprintf(stderr, "Body 'torso' not found\n");
        if (viewer_ready) {
            mjr_freeContext(&con);
            mjv_freeScene(&scn);
            glfwTerminate();
        }
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }

    const int act_wheel_L = find_id(m, mjOBJ_ACTUATOR, "act_wheel_L");
    const int act_wheel_R = find_id(m, mjOBJ_ACTUATOR, "act_wheel_R");
    const int act_hip_L = find_id(m, mjOBJ_ACTUATOR, "act_hip_L");
    const int act_hip_R = find_id(m, mjOBJ_ACTUATOR, "act_hip_R");

    warn_ctrlrange(m, act_wheel_L, "act_wheel_L");
    warn_ctrlrange(m, act_wheel_R, "act_wheel_R");

    const int jnt_hip_L = find_id(m, mjOBJ_JOINT, "hip_L");
    const int jnt_hip_R = find_id(m, mjOBJ_JOINT, "hip_R");
    const int jnt_wheel_L = find_id(m, mjOBJ_JOINT, "wheel_L");
    const int jnt_wheel_R = find_id(m, mjOBJ_JOINT, "wheel_R");

    const int sensor_gyro = find_id(m, mjOBJ_SENSOR, "gyro");
    const int sensor_acc = find_id(m, mjOBJ_SENSOR, "acc");

    const int geom_wheel_L = find_id(m, mjOBJ_GEOM, "wheel_geom_L");
    double wheel_radius = (geom_wheel_L >= 0) ? m->geom_size[3 * geom_wheel_L + 0] : 0.085;
    if (wheel_radius <= 0.0) {
        wheel_radius = 0.085;
    }

    const int site_wL = find_id(m, mjOBJ_SITE, "W_L");
    const int site_wR = find_id(m, mjOBJ_SITE, "W_R");
    const int jnt_base = find_id(m, mjOBJ_JOINT, "base");
    double wheel_base = 0.31;
    if (site_wL >= 0 && site_wR >= 0) {
        const mjtNum* wL = d->site_xpos + 3 * site_wL;
        const mjtNum* wR = d->site_xpos + 3 * site_wR;
        wheel_base = std::fabs(static_cast<double>(wL[1] - wR[1]));
    }

    if (site_wL >= 0 && site_wR >= 0 && jnt_base >= 0) {
        const mjtNum* wL = d->site_xpos + 3 * site_wL;
        const mjtNum* wR = d->site_xpos + 3 * site_wR;
        double min_z = static_cast<double>(wL[2]);
        if (wR[2] < min_z) {
            min_z = static_cast<double>(wR[2]);
        }
        double dz = wheel_radius - min_z;
        if (std::fabs(dz) > 1e-6) {
            int base_qpos = m->jnt_qposadr[jnt_base];
            d->qpos[base_qpos + 2] += dz;
            mj_forward(m, d);
        }
    }

    if (viewer_ready) {
        cam.type = mjCAMERA_TRACKING;
        cam.trackbodyid = torso_id;
        cam.distance = 2.0;
        cam.azimuth = 90.0;
        cam.elevation = -20.0;
    }

    double hip_L_target = 0.0;
    double hip_R_target = 0.0;
    if (opt.hold_hips && jnt_hip_L >= 0 && jnt_hip_R >= 0) {
        double hip_L_key = d->qpos[m->jnt_qposadr[jnt_hip_L]];
        double hip_R_key = d->qpos[m->jnt_qposadr[jnt_hip_R]];
        if (opt.hip_target_mid) {
            hip_L_target = hip_target_from_range(m, jnt_hip_L, hip_L_key);
            hip_R_target = hip_target_from_range(m, jnt_hip_R, hip_R_key);
        } else {
            hip_L_target = hip_L_key;
            hip_R_target = hip_R_key;
        }
    }

    MotionController controller(RobotParams{});
    controller.setControlDt(static_cast<float>(m->opt.timestep));
    controller.setTargetVelocity(0.0f);
    controller.setRequestedMode(InnerLongMode::LQR);

    BalancerEKF ekf;
    if (opt.use_ekf) {
        ekf.begin();
        const mjtNum* quat0 = d->xquat + 4 * torso_id;
        mjtNum R0[9];
        mju_quat2Mat(R0, quat0);
        const mjtNum x_body_world0[3] = {R0[0], R0[3], R0[6]};
        double theta0 = std::atan2(x_body_world0[2], x_body_world0[0]);
        double x0 = d->xpos[3 * torso_id + 0];
        ekf.reset(static_cast<float>(theta0), static_cast<float>(x0));
    }

    double next_print = (opt.print_dt > 0.0) ? opt.print_dt : opt.sim_time + 1.0;
    double sim_start = d->time;
    auto wall_start = std::chrono::steady_clock::now();

    while (d->time < opt.sim_time) {
        if (viewer_ready && glfwWindowShouldClose(window)) {
            break;
        }
        const mjtNum* quat = d->xquat + 4 * torso_id;
        mjtNum R[9];
        mju_quat2Mat(R, quat);

        const mjtNum x_body_world[3] = {R[0], R[3], R[6]};
        double theta = std::atan2(x_body_world[2], x_body_world[0]);

        mjtNum vel_world[6] = {0};
        mjtNum vel_body[6] = {0};
        mj_objectVelocity(m, d, mjOBJ_BODY, torso_id, vel_world, 0);
        mj_objectVelocity(m, d, mjOBJ_BODY, torso_id, vel_body, 1);
        double theta_dot = vel_body[1];
        double x_dot_world = vel_world[3];

        StateEstimate est{};
        est.gyroBias = 0.0f;
        est.valid = true;

        if (opt.use_ekf) {
            mjtNum gyro_body[3] = {0, 0, 0};
            mjtNum accel_body[3] = {0, 0, 0};
            bool have_gyro = read_sensor_vec3(m, d, sensor_gyro, gyro_body);
            bool have_acc = read_sensor_vec3(m, d, sensor_acc, accel_body);

            float theta_acc = NAN;
            if (have_acc) {
                float ay = static_cast<float>(accel_body[1]);
                float az = static_cast<float>(accel_body[2]);
                float accel_yz = std::sqrt(ay * ay + az * az);
                if (accel_yz > 1e-6f) {
                    theta_acc = std::atan2(-static_cast<float>(accel_body[0]), accel_yz);
                }
            }

            float gyro_pitch = have_gyro ? static_cast<float>(gyro_body[1]) : 0.0f;
            float gyro_yaw = have_gyro ? static_cast<float>(gyro_body[2]) : 0.0f;

            double omega_L = 0.0;
            double omega_R = 0.0;
            if (jnt_wheel_L >= 0) {
                omega_L = d->qvel[m->jnt_dofadr[jnt_wheel_L]];
            }
            if (jnt_wheel_R >= 0) {
                omega_R = d->qvel[m->jnt_dofadr[jnt_wheel_R]];
            }

            float v_enc = static_cast<float>(wheel_radius * 0.5 * (omega_L + omega_R));
            float yaw_rate_enc = 0.0f;
            if (wheel_base > 1e-6) {
                yaw_rate_enc = static_cast<float>(wheel_radius * (omega_R - omega_L) / wheel_base);
            }

            float pos_enc = NAN;
            float dt = static_cast<float>(m->opt.timestep);
            bool ok = ekf.step(theta_acc, v_enc, pos_enc,
                               gyro_pitch, gyro_yaw, yaw_rate_enc,
                               dt, NAN);
            BalancerState st = ekf.getState();
            est.theta = st.theta;
            est.thetaDot = st.thetaDot;
            est.x = st.x;
            est.xDot = st.xDot;
            est.gyroBias = st.gyroBias;
            est.valid = ok && ekf.isStateValid();
        } else {
            est.theta = static_cast<float>(theta);
            est.thetaDot = static_cast<float>(theta_dot);
            est.x = static_cast<float>(d->xpos[3 * torso_id + 0]);
            est.xDot = static_cast<float>(x_dot_world);
        }

        const float dt = static_cast<float>(m->opt.timestep);
        MotionController::Command cmd = controller.computeControl(est, dt);

        if (act_wheel_L >= 0) {
            double u = static_cast<double>(cmd.torque.torqueLeftNm) * opt.u_scale;
            d->ctrl[act_wheel_L] = clamp_ctrl(m, act_wheel_L, u);
        }
        if (act_wheel_R >= 0) {
            double u = static_cast<double>(cmd.torque.torqueRightNm) * opt.u_scale;
            d->ctrl[act_wheel_R] = clamp_ctrl(m, act_wheel_R, u);
        }

        if (opt.hold_hips && act_hip_L >= 0 && act_hip_R >= 0 &&
            jnt_hip_L >= 0 && jnt_hip_R >= 0) {
            if (is_servo_actuator(m, act_hip_L) && is_servo_actuator(m, act_hip_R)) {
                d->ctrl[act_hip_L] = clamp_ctrl(m, act_hip_L, hip_L_target);
                d->ctrl[act_hip_R] = clamp_ctrl(m, act_hip_R, hip_R_target);
            } else {
                const int qpos_L = m->jnt_qposadr[jnt_hip_L];
                const int qvel_L = m->jnt_dofadr[jnt_hip_L];
                const int qpos_R = m->jnt_qposadr[jnt_hip_R];
                const int qvel_R = m->jnt_dofadr[jnt_hip_R];

                double err_L = hip_L_target - d->qpos[qpos_L];
                double err_R = hip_R_target - d->qpos[qpos_R];
                double derr_L = -d->qvel[qvel_L];
                double derr_R = -d->qvel[qvel_R];

                double u_L = opt.hip_kp * err_L + opt.hip_kd * derr_L;
                double u_R = opt.hip_kp * err_R + opt.hip_kd * derr_R;

                d->ctrl[act_hip_L] = clamp_ctrl(m, act_hip_L, u_L);
                d->ctrl[act_hip_R] = clamp_ctrl(m, act_hip_R, u_R);
            }
        }

        mj_step(m, d);

        if (viewer_ready) {
            mjrRect viewport{0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            mjv_updateScene(m, d, &vopt, nullptr, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);
            glfwSwapBuffers(window);
            glfwPollEvents();
        }

        if (opt.realtime) {
            auto now = std::chrono::steady_clock::now();
            double wall_elapsed =
                std::chrono::duration<double>(now - wall_start).count();
            double sim_elapsed = d->time - sim_start;
            if (sim_elapsed > wall_elapsed) {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(sim_elapsed - wall_elapsed));
            }
        }

        if (opt.print_dt > 0.0 && d->time >= next_print) {
            std::printf("t=%.3f theta=%.4f thetaDot=%.4f xDot=%.4f uL=%.3f uR=%.3f\n",
                        d->time, theta, theta_dot, est.xDot,
                        static_cast<double>(cmd.torque.torqueLeftNm),
                        static_cast<double>(cmd.torque.torqueRightNm));
            next_print += opt.print_dt;
        }
    }

    if (viewer_ready) {
        mjr_freeContext(&con);
        mjv_freeScene(&scn);
        glfwTerminate();
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
