#include <catch2/catch_test_macros.hpp>

#include <mujoco/mujoco.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::string quote(const fs::path& p) {
    return "\"" + p.string() + "\"";
}

struct ParsedKeyframe {
    std::string name;
    std::vector<double> qpos;
};

std::vector<double> parse_qpos_values(const std::string& s) {
    std::vector<double> vals;
    std::istringstream iss(s);
    double v = 0.0;
    while (iss >> v) {
        vals.push_back(v);
    }
    return vals;
}

std::vector<ParsedKeyframe> parse_generated_keyframes(const fs::path& keyframes_xml_path) {
    std::ifstream in(keyframes_xml_path);
    REQUIRE(in.is_open());

    std::vector<ParsedKeyframe> out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.find("<key ") == std::string::npos) continue;

        const auto name_key = line.find("name=\"");
        const auto qpos_key = line.find("qpos=\"");
        if (name_key == std::string::npos || qpos_key == std::string::npos) continue;

        const auto name_start = name_key + 6;
        const auto name_end = line.find('"', name_start);
        if (name_end == std::string::npos) continue;

        const auto qpos_start = qpos_key + 6;
        const auto qpos_end = line.find('"', qpos_start);
        if (qpos_end == std::string::npos) continue;

        ParsedKeyframe kf;
        kf.name = line.substr(name_start, name_end - name_start);
        kf.qpos = parse_qpos_values(line.substr(qpos_start, qpos_end - qpos_start));
        out.push_back(std::move(kf));
    }
    return out;
}

}  // namespace

TEST_CASE("generated equilibrium keyframes keep both wheels on ground", "[equilibrium][keyframe][ground]")
{
    const fs::path robot_xml = fs::path(TEST_MODEL_PATH);
    const fs::path scene_xml = robot_xml.parent_path() / "scene.xml";
    REQUIRE(fs::exists(scene_xml));

    const auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    const fs::path tmp_root =
        fs::temp_directory_path() / ("linearize_grounded_keyframes_" + std::to_string(now));
    const fs::path out_dir = tmp_root / "out";
    const fs::path kf_path = tmp_root / "generated_keyframes.xml";
    fs::create_directories(out_dir);

    std::ostringstream cmd;
    cmd << quote(fs::path(LINEARIZE_HIP_BIN))
        << " --model " << quote(scene_xml)
        << " --equilibrium-wheels"
        << " --write-keyframes " << quote(kf_path)
        << " --out " << quote(out_dir);
    const int rc = std::system(cmd.str().c_str());
    REQUIRE(rc == 0);
    REQUIRE(fs::exists(kf_path));

    auto keyframes = parse_generated_keyframes(kf_path);
    REQUIRE(keyframes.size() == 7);

    char error[1024] = {0};
    mjModel* m = mj_loadXML(scene_xml.string().c_str(), nullptr, error, sizeof(error));
    REQUIRE(m != nullptr);

    mjData* d = mj_makeData(m);
    REQUIRE(d != nullptr);

    const int body_wheel_l = mj_name2id(m, mjOBJ_BODY, "wheel_geom");
    const int body_wheel_r = mj_name2id(m, mjOBJ_BODY, "wheel_geom_2");
    REQUIRE(body_wheel_l >= 0);
    REQUIRE(body_wheel_r >= 0);

    double wheel_radius = 0.08547;
    const int geom_wheel_contact_l = mj_name2id(m, mjOBJ_GEOM, "wheel_contact_L");
    if (geom_wheel_contact_l >= 0) {
        const double r = m->geom_size[3 * geom_wheel_contact_l + 0];
        if (r > 0.0) wheel_radius = r;
    }

    // Ground contact z residual tolerance (meters). We expect around sub-mm.
    const double z_tol = 2e-3;

    for (const auto& kf : keyframes) {
        INFO("keyframe=" << kf.name);
        REQUIRE(kf.qpos.size() == static_cast<size_t>(m->nq));

        mju_copy(d->qpos, kf.qpos.data(), m->nq);
        mju_zero(d->qvel, m->nv);
        if (m->na) mju_zero(d->act, m->na);
        if (m->nu) mju_zero(d->ctrl, m->nu);
        mj_forward(m, d);

        const double wheel_l_contact_z = d->xpos[3 * body_wheel_l + 2] - wheel_radius;
        const double wheel_r_contact_z = d->xpos[3 * body_wheel_r + 2] - wheel_radius;

        INFO("wheel_l_contact_z=" << wheel_l_contact_z
             << " wheel_r_contact_z=" << wheel_r_contact_z);
        CHECK(std::abs(wheel_l_contact_z) < z_tol);
        CHECK(std::abs(wheel_r_contact_z) < z_tol);
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    fs::remove_all(tmp_root);
}
