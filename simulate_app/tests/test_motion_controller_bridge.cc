#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "lqr_lut_data.h"
#include "motion_controller_bridge.h"

#ifndef SIM_TEST_MODEL_PATH
#define SIM_TEST_MODEL_PATH "myRobot/scene.xml"
#endif

#ifndef TEST_BALANCE_BIN
#define TEST_BALANCE_BIN "build/simulate_app/test_balance"
#endif

namespace fs = std::filesystem;

namespace {

struct ModelData {
  mjModel* m = nullptr;
  mjData* d = nullptr;

  ~ModelData() {
    if (d) {
      mj_deleteData(d);
    }
    if (m) {
      mj_deleteModel(m);
    }
  }
};

struct CommandResult {
  int code = -1;
  std::string output;
};

class ScopedEnv {
 public:
  explicit ScopedEnv(
      const std::vector<std::pair<std::string, std::string>>& entries) {
    for (const auto& [key, value] : entries) {
      SaveAndSet(key, value);
    }
  }

  ~ScopedEnv() {
    for (const auto& s : saved_) {
      if (s.had_value) {
        setenv(s.key.c_str(), s.value.c_str(), 1);
      } else {
        unsetenv(s.key.c_str());
      }
    }
  }

 private:
  struct Saved {
    std::string key;
    bool had_value = false;
    std::string value;
  };

  void SaveAndSet(const std::string& key, const std::string& value) {
    Saved s{};
    s.key = key;
    const char* prior = std::getenv(key.c_str());
    if (prior) {
      s.had_value = true;
      s.value = prior;
    }
    saved_.push_back(s);
    setenv(key.c_str(), value.c_str(), 1);
  }

  std::vector<Saved> saved_;
};

std::string ShellQuote(const std::string& s) {
  std::string out = "'";
  for (const char ch : s) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out.push_back('\'');
  return out;
}

ModelData LoadModel() {
  ModelData md;
  char error[1024] = {0};
  md.m = mj_loadXML(SIM_TEST_MODEL_PATH, nullptr, error, sizeof(error));
  REQUIRE(md.m != nullptr);
  md.d = mj_makeData(md.m);
  REQUIRE(md.d != nullptr);
  return md;
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

int NearestHipLutIndex(double hip) {
  int best = 0;
  double best_err = std::fabs(hip - static_cast<double>(kHipLut[0]));
  for (int i = 1; i < LQR_LUT_SIZE; ++i) {
    const double err = std::fabs(hip - static_cast<double>(kHipLut[i]));
    if (err < best_err) {
      best = i;
      best_err = err;
    }
  }
  return best;
}

void CheckHipCtrlTargets(const mjModel* m, const mjData* d, int act_hip_L, int act_hip_R,
                         double hip_target, double tol) {
  const double gear_L = m->actuator_gear[6 * act_hip_L + 0];
  const double gear_R = m->actuator_gear[6 * act_hip_R + 0];
  const double expected_L = ClampCtrl(m, act_hip_L, hip_target * gear_L);
  const double expected_R = ClampCtrl(m, act_hip_R, hip_target * gear_R);
  CHECK(d->ctrl[act_hip_L] == Catch::Approx(expected_L).margin(tol));
  CHECK(d->ctrl[act_hip_R] == Catch::Approx(expected_R).margin(tol));
}

CommandResult RunTestBalance(const std::vector<std::string>& args) {
  const auto now =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  const fs::path out_path =
      fs::temp_directory_path() / ("simulate_test_balance_" + std::to_string(now) + ".log");

  std::ostringstream cmd;
  cmd << ShellQuote(TEST_BALANCE_BIN) << " " << ShellQuote(SIM_TEST_MODEL_PATH);
  for (const auto& arg : args) {
    cmd << " " << ShellQuote(arg);
  }
  cmd << " > " << ShellQuote(out_path.string()) << " 2>&1";

  CommandResult result{};
  result.code = std::system(cmd.str().c_str());

  std::ifstream in(out_path);
  std::ostringstream out;
  out << in.rdbuf();
  result.output = out.str();
  fs::remove(out_path);
  return result;
}

void RequireSummaryPass(const CommandResult& run) {
  INFO(run.output);
  CHECK(run.code == 0);
  REQUIRE(run.output.find("SUMMARY ") != std::string::npos);
  REQUIRE(run.output.find("passed=1") != std::string::npos);
}

}  // namespace

TEST_CASE("bridge hip target LUT stepping keeps gear-scaled actuator ctrl", "[simulate][bridge]")
{
  ModelData md = LoadModel();
  const int key_id = mj_name2id(md.m, mjOBJ_KEY, "eq_hip_m0550");
  REQUIRE(key_id >= 0);
  mj_resetDataKeyframe(md.m, md.d, key_id);
  mj_forward(md.m, md.d);

  const int act_hip_L = mj_name2id(md.m, mjOBJ_ACTUATOR, "hip_L");
  const int act_hip_R = mj_name2id(md.m, mjOBJ_ACTUATOR, "hip_R");
  const int jnt_hip_L = mj_name2id(md.m, mjOBJ_JOINT, "hip_L");
  const int jnt_hip_R = mj_name2id(md.m, mjOBJ_JOINT, "hip_R");
  REQUIRE(act_hip_L >= 0);
  REQUIRE(act_hip_R >= 0);
  REQUIRE(jnt_hip_L >= 0);
  REQUIRE(jnt_hip_R >= 0);

  const int qpos_L = md.m->jnt_qposadr[jnt_hip_L];
  const int qpos_R = md.m->jnt_qposadr[jnt_hip_R];
  const double hip_initial = 0.5 * (md.d->qpos[qpos_L] + md.d->qpos[qpos_R]);
  const int idx_initial = NearestHipLutIndex(hip_initial);
  const int idx_up = std::min(idx_initial + 1, LQR_LUT_SIZE - 1);
  const int idx_down = std::max(idx_up - 1, 0);

  mjcb_control = MotionControllerCallback;
  MotionControllerReset(md.m, md.d);
  REQUIRE(MotionControllerIsEnabled());

  MotionControllerCallback(md.m, md.d);
  CheckHipCtrlTargets(md.m, md.d, act_hip_L, act_hip_R, hip_initial, 2e-3);

  const bool changed_up = MotionControllerStepHipTarget(+1);
  CHECK(changed_up == (idx_up != idx_initial));
  MotionControllerCallback(md.m, md.d);
  CheckHipCtrlTargets(md.m, md.d, act_hip_L, act_hip_R, static_cast<double>(kHipLut[idx_up]),
                      2e-3);

  const bool changed_down = MotionControllerStepHipTarget(-1);
  CHECK(changed_down == (idx_down != idx_up));
  MotionControllerCallback(md.m, md.d);
  CheckHipCtrlTargets(md.m, md.d, act_hip_L, act_hip_R,
                      static_cast<double>(kHipLut[idx_down]), 2e-3);

  for (int i = 0; i < LQR_LUT_SIZE + 2; ++i) {
    MotionControllerStepHipTarget(-1);
    MotionControllerCallback(md.m, md.d);
  }
  CheckHipCtrlTargets(md.m, md.d, act_hip_L, act_hip_R, static_cast<double>(kHipLut[0]), 2e-3);
  CHECK_FALSE(MotionControllerStepHipTarget(-1));
}

TEST_CASE("test_balance smoke passes in ground-truth mode", "[simulate][balance]")
{
  ScopedEnv env({
      {"SIM_USE_EKF", "0"},
      {"SIM_USE_STATE_ESTIMATOR", "0"},
      {"SIM_EST_THETA_ONLY", "0"},
  });
  const CommandResult run =
      RunTestBalance({"--duration", "2", "--print-dt", "2", "--key", "eq_hip_p0525"});
  RequireSummaryPass(run);
}

TEST_CASE("test_balance smoke passes in StateEstimator theta-only mode",
          "[simulate][balance][estimator]")
{
  ScopedEnv env({
      {"SIM_USE_EKF", "1"},
      {"SIM_USE_STATE_ESTIMATOR", "1"},
      {"SIM_EST_THETA_ONLY", "1"},
  });
  const CommandResult run =
      RunTestBalance({"--duration", "2", "--print-dt", "2", "--key", "eq_hip_p0525"});
  RequireSummaryPass(run);
}

TEST_CASE("test_balance accepts keyframe name", "[simulate][keyframe]")
{
  const CommandResult run =
      RunTestBalance({"--duration", "0.5", "--print-dt", "0.5", "--key", "eq_hip_m0550"});
  INFO(run.output);
  CHECK(run.code == 0);
  REQUIRE(run.output.find("Reset to keyframe ") != std::string::npos);
  REQUIRE(run.output.find("(eq_hip_m0550)") != std::string::npos);
}

TEST_CASE("test_balance accepts keyframe index", "[simulate][keyframe]")
{
  const CommandResult run =
      RunTestBalance({"--duration", "0.5", "--print-dt", "0.5", "--key", "2"});
  INFO(run.output);
  CHECK(run.code == 0);
  REQUIRE(run.output.find("Reset to keyframe 2 (eq_hip_m0550)") != std::string::npos);
}
