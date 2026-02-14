#include <mujoco/mujoco.h>
#include <cstdio>

int main() {
    const char* model_path = "../../myRobot/robot.xml";

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));

    if (!m) {
        printf("Error loading model: %s\n", error);
        return 1;
    }

    // Find hip actuators
    int act_hip_L = mj_name2id(m, mjOBJ_ACTUATOR, "hip_L");
    int act_hip_R = mj_name2id(m, mjOBJ_ACTUATOR, "hip_R");

    printf("=== Hip Actuator Properties ===\n");

    if (act_hip_L >= 0) {
        printf("\nhip_L actuator (index %d):\n", act_hip_L);
        printf("  gear[0] = %f\n", m->actuator_gear[6 * act_hip_L]);
        printf("  ctrlrange = [%f, %f]\n",
               m->actuator_ctrlrange[2 * act_hip_L],
               m->actuator_ctrlrange[2 * act_hip_L + 1]);
        printf("  ctrllimited = %d\n", m->actuator_ctrllimited[act_hip_L]);
    }

    if (act_hip_R >= 0) {
        printf("\nhip_R actuator (index %d):\n", act_hip_R);
        printf("  gear[0] = %f\n", m->actuator_gear[6 * act_hip_R]);
        printf("  ctrlrange = [%f, %f]\n",
               m->actuator_ctrlrange[2 * act_hip_R],
               m->actuator_ctrlrange[2 * act_hip_R + 1]);
        printf("  ctrllimited = %d\n", m->actuator_ctrllimited[act_hip_R]);
    }

    // Check joint ranges
    int jnt_hip_L = mj_name2id(m, mjOBJ_JOINT, "hip_L");
    int jnt_hip_R = mj_name2id(m, mjOBJ_JOINT, "hip_R");

    printf("\n=== Hip Joint Properties ===\n");

    if (jnt_hip_L >= 0) {
        printf("\nhip_L joint (index %d):\n", jnt_hip_L);
        printf("  range = [%f, %f]\n",
               m->jnt_range[2 * jnt_hip_L],
               m->jnt_range[2 * jnt_hip_L + 1]);
        printf("  limited = %d\n", m->jnt_limited[jnt_hip_L]);
    }

    if (jnt_hip_R >= 0) {
        printf("\nhip_R joint (index %d):\n", jnt_hip_R);
        printf("  range = [%f, %f]\n",
               m->jnt_range[2 * jnt_hip_R],
               m->jnt_range[2 * jnt_hip_R + 1]);
        printf("  limited = %d\n", m->jnt_limited[jnt_hip_R]);
    }

    mj_deleteModel(m);
    return 0;
}
