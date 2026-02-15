// Generate equilibrium keyframes for given hip/theta pairs.
#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace {

struct Target {
  double hip;
  double theta_eq;
};

struct Pose {
  std::string name;
  std::vector<double> qpos;
  double hip;
  double theta_target;
  double theta_actual;
  double wheel_contact_z_l;
  double wheel_contact_z_r;
};

constexpr Target kTargets[] = {
    {-0.2700, -0.184172},
    {-0.1625, -0.133335},
    {-0.0550, -0.100863},
    {0.0525, -0.080519},
    {0.1600, -0.064520},
    {0.2675, -0.045325},
    {0.3750, -0.031514},
};

double GetPitch(const mjData* d) {
  const double qw = d->qpos[3];
  const double qx = d->qpos[4];
  const double qy = d->qpos[5];
  const double qz = d->qpos[6];
  return std::asin(2.0 * (qw * qy - qz * qx));
}

double ClampCtrl(const mjModel* m, int act_id, double u) {
  if (act_id < 0) {
    return 0.0;
  }
  if (m->actuator_ctrllimited[act_id]) {
    const double lo = m->actuator_ctrlrange[2 * act_id + 0];
    const double hi = m->actuator_ctrlrange[2 * act_id + 1];
    if (u < lo) {
      return lo;
    }
    if (u > hi) {
      return hi;
    }
  }
  return u;
}

double PositionCtrlForJointTarget(const mjModel* m, int act_id, double joint_target) {
  if (act_id < 0) {
    return joint_target;
  }
  const double gear = m->actuator_gear[6 * act_id + 0];
  if (std::abs(gear) < 1e-9) {
    return joint_target;
  }
  return joint_target * gear;
}

void SetFreejointPose(mjData* d, int qpos_adr, double x, double y, double z, double pitch) {
  d->qpos[qpos_adr + 0] = x;
  d->qpos[qpos_adr + 1] = y;
  d->qpos[qpos_adr + 2] = z;
  d->qpos[qpos_adr + 3] = std::cos(0.5 * pitch);
  d->qpos[qpos_adr + 4] = 0.0;
  d->qpos[qpos_adr + 5] = std::sin(0.5 * pitch);
  d->qpos[qpos_adr + 6] = 0.0;
}

std::string KeyNameFromHip(double hip) {
  const char sign = (hip < 0.0) ? 'm' : 'p';
  const int mag = static_cast<int>(std::llround(std::abs(hip) * 10000.0));
  char buf[32];
  std::snprintf(buf, sizeof(buf), "eq_hip_%c%04d", sign, mag);
  return std::string(buf);
}

bool FindIdOrFail(const mjModel* m, int type, const char* name, int* out) {
  const int id = mj_name2id(m, type, name);
  if (id < 0) {
    std::printf("ERROR: missing %s id for '%s'\n",
                (type == mjOBJ_JOINT) ? "joint"
                : (type == mjOBJ_ACTUATOR) ? "actuator"
                : (type == mjOBJ_BODY) ? "body"
                : (type == mjOBJ_GEOM) ? "geom"
                : "object",
                name);
    return false;
  }
  *out = id;
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  const char* model_path = "myRobot/scene.xml";
  const char* out_path = "myRobot/keyframes_eq.xml";
  if (argc > 1) {
    model_path = argv[1];
  }
  if (argc > 2) {
    out_path = argv[2];
  }

  char error[1024] = {0};
  mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
  if (!m) {
    std::printf("ERROR: failed to load model '%s': %s\n", model_path, error);
    return 1;
  }

  int jnt_free = -1;
  int jnt_hip_L = -1;
  int jnt_hip_R = -1;
  int act_hip_L = -1;
  int act_hip_R = -1;
  int act_wheel_L = -1;
  int act_wheel_R = -1;
  int body_wheel_L = -1;
  int body_wheel_R = -1;

  bool ok = true;
  ok = ok && FindIdOrFail(m, mjOBJ_JOINT, "torso_freejoint", &jnt_free);
  ok = ok && FindIdOrFail(m, mjOBJ_JOINT, "hip_L", &jnt_hip_L);
  ok = ok && FindIdOrFail(m, mjOBJ_JOINT, "hip_R", &jnt_hip_R);
  ok = ok && FindIdOrFail(m, mjOBJ_ACTUATOR, "hip_L", &act_hip_L);
  ok = ok && FindIdOrFail(m, mjOBJ_ACTUATOR, "hip_R", &act_hip_R);
  ok = ok && FindIdOrFail(m, mjOBJ_ACTUATOR, "wheel_L", &act_wheel_L);
  ok = ok && FindIdOrFail(m, mjOBJ_ACTUATOR, "wheel_R", &act_wheel_R);
  ok = ok && FindIdOrFail(m, mjOBJ_BODY, "wheel_geom", &body_wheel_L);
  ok = ok && FindIdOrFail(m, mjOBJ_BODY, "wheel_geom_2", &body_wheel_R);
  if (!ok) {
    mj_deleteModel(m);
    return 1;
  }

  const int free_qpos_adr = m->jnt_qposadr[jnt_free];
  const int free_dof_adr = m->jnt_dofadr[jnt_free];
  const int hipL_qpos_adr = m->jnt_qposadr[jnt_hip_L];
  const int hipR_qpos_adr = m->jnt_qposadr[jnt_hip_R];

  double wheel_radius = 0.08547;
  const int geom_wheel_contact_L = mj_name2id(m, mjOBJ_GEOM, "wheel_contact_L");
  if (geom_wheel_contact_L >= 0) {
    const double r = m->geom_size[3 * geom_wheel_contact_L + 0];
    if (r > 0.0) {
      wheel_radius = r;
    }
  }

  mjData* d = mj_makeData(m);
  if (!d) {
    std::printf("ERROR: failed to allocate mjData\n");
    mj_deleteModel(m);
    return 1;
  }

  const int key0 = (m->nkey > 0) ? 0 : -1;
  if (key0 >= 0) {
    mj_resetDataKeyframe(m, d, key0);
  } else {
    mj_resetData(m, d);
  }
  mj_forward(m, d);

  const double base_x = d->qpos[free_qpos_adr + 0];
  const double base_y = d->qpos[free_qpos_adr + 1];
  const double base_z_nominal = d->qpos[free_qpos_adr + 2];

  std::vector<Pose> poses;
  poses.reserve(sizeof(kTargets) / sizeof(kTargets[0]));

  for (const Target& t : kTargets) {
    if (key0 >= 0) {
      mj_resetDataKeyframe(m, d, key0);
    } else {
      mj_resetData(m, d);
    }

    d->qpos[hipL_qpos_adr] = t.hip;
    d->qpos[hipR_qpos_adr] = t.hip;
    SetFreejointPose(d, free_qpos_adr, base_x, base_y, base_z_nominal, t.theta_eq);
    mj_forward(m, d);

    const double zL = d->xpos[3 * body_wheel_L + 2];
    const double zR = d->xpos[3 * body_wheel_R + 2];
    const double wheel_z_avg = 0.5 * (zL + zR);
    double base_z = base_z_nominal + (wheel_radius - wheel_z_avg);
    SetFreejointPose(d, free_qpos_adr, base_x, base_y, base_z, t.theta_eq);
    mj_forward(m, d);

    // Let passive linkage and contacts settle while clamping base pose.
    const double hip_ctrl_L = PositionCtrlForJointTarget(m, act_hip_L, t.hip);
    const double hip_ctrl_R = PositionCtrlForJointTarget(m, act_hip_R, t.hip);
    for (int step = 0; step < 600; ++step) {
      SetFreejointPose(d, free_qpos_adr, base_x, base_y, base_z, t.theta_eq);
      for (int i = 0; i < 6; ++i) {
        d->qvel[free_dof_adr + i] = 0.0;
      }
      d->ctrl[act_hip_L] = ClampCtrl(m, act_hip_L, hip_ctrl_L);
      d->ctrl[act_hip_R] = ClampCtrl(m, act_hip_R, hip_ctrl_R);
      d->ctrl[act_wheel_L] = 0.0;
      d->ctrl[act_wheel_R] = 0.0;
      mj_step(m, d);
    }

    // Correct base height after settling because passive joints/contacts can
    // move wheel centers relative to the initial estimate.
    const double settled_zL = d->xpos[3 * body_wheel_L + 2];
    const double settled_zR = d->xpos[3 * body_wheel_R + 2];
    const double settled_avg = 0.5 * (settled_zL + settled_zR);
    base_z += (wheel_radius - settled_avg);

    SetFreejointPose(d, free_qpos_adr, base_x, base_y, base_z, t.theta_eq);
    for (int i = 0; i < m->nv; ++i) {
      d->qvel[i] = 0.0;
    }
    d->ctrl[act_hip_L] = ClampCtrl(m, act_hip_L, hip_ctrl_L);
    d->ctrl[act_hip_R] = ClampCtrl(m, act_hip_R, hip_ctrl_R);
    d->ctrl[act_wheel_L] = 0.0;
    d->ctrl[act_wheel_R] = 0.0;
    mj_forward(m, d);

    Pose p;
    p.name = KeyNameFromHip(t.hip);
    p.qpos.assign(d->qpos, d->qpos + m->nq);
    p.hip = t.hip;
    p.theta_target = t.theta_eq;
    p.theta_actual = GetPitch(d);
    p.wheel_contact_z_l = d->xpos[3 * body_wheel_L + 2] - wheel_radius;
    p.wheel_contact_z_r = d->xpos[3 * body_wheel_R + 2] - wheel_radius;
    poses.push_back(std::move(p));
  }

  std::ofstream out(out_path, std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    std::printf("ERROR: failed to open output '%s'\n", out_path);
    mj_deleteData(d);
    mj_deleteModel(m);
    return 1;
  }

  out << "<mujocoinclude>\n";
  out.setf(std::ios::fixed);
  out.precision(6);
  for (const Pose& p : poses) {
    out << "  <key name=\"" << p.name << "\" qpos=\"";
    for (int i = 0; i < static_cast<int>(p.qpos.size()); ++i) {
      out << p.qpos[i];
      if (i + 1 < static_cast<int>(p.qpos.size())) {
        out << " ";
      }
    }
    out << "\"/>\n";
  }
  out << "</mujocoinclude>\n";
  out.close();

  std::printf("Wrote %zu keyframes to %s\n", poses.size(), out_path);
  std::printf("%12s %10s %14s %14s %12s %12s\n",
              "key", "hip(rad)", "theta_tgt(rad)", "theta_out(rad)",
              "wheelL_z", "wheelR_z");
  for (const Pose& p : poses) {
    std::printf("%12s %10.6f %14.6f %14.6f %12.6f %12.6f\n",
                p.name.c_str(), p.hip, p.theta_target, p.theta_actual,
                p.wheel_contact_z_l, p.wheel_contact_z_r);
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
