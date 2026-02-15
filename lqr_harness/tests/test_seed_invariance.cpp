#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::string quote(const fs::path& p) {
    return "\"" + p.string() + "\"";
}

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        cols.push_back(cell);
    }
    return cols;
}

std::map<double, double> read_theta_eq_map(const fs::path& out_dir) {
    std::map<double, double> theta_by_hip;
    for (const auto& ent : fs::directory_iterator(out_dir)) {
        if (!ent.is_regular_file()) continue;
        const std::string name = ent.path().filename().string();
        if (name.rfind("eq_hip_", 0) != 0) continue;
        if (ent.path().extension() != ".csv") continue;

        std::ifstream in(ent.path());
        if (!in.is_open()) continue;
        std::string header;
        std::string row;
        if (!std::getline(in, header)) continue;
        if (!std::getline(in, row)) continue;
        const auto cols = split_csv_line(row);
        if (cols.size() < 2) continue;
        const double hip = std::stod(cols[0]);
        const double theta_eq = std::stod(cols[1]);
        theta_by_hip[hip] = theta_eq;
    }
    return theta_by_hip;
}

std::map<double, double> run_linearize_and_read(const fs::path& model_path,
                                                 const fs::path& out_dir,
                                                 const fs::path& keyframes_out_path,
                                                 const std::optional<std::string>& key) {
    std::ostringstream cmd;
    cmd << quote(fs::path(LINEARIZE_HIP_BIN))
        << " --model " << quote(model_path)
        << " --equilibrium-wheels"
        << " --write-keyframes " << quote(keyframes_out_path)
        << " --out " << quote(out_dir);
    if (key.has_value()) {
        cmd << " --key " << key.value();
    }

    const int rc = std::system(cmd.str().c_str());
    REQUIRE(rc == 0);

    const auto theta_map = read_theta_eq_map(out_dir);
    REQUIRE(theta_map.size() == 7);
    return theta_map;
}

}  // namespace

TEST_CASE("theta_eq is invariant to seed keyframe selection", "[equilibrium][seed]")
{
    const fs::path robot_xml = fs::path(TEST_MODEL_PATH);
    const fs::path scene_xml = robot_xml.parent_path() / "scene.xml";
    REQUIRE(fs::exists(scene_xml));

    const auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    const fs::path tmp_root =
        fs::temp_directory_path() / ("linearize_seed_invariance_" + std::to_string(now));
    const fs::path out_default = tmp_root / "default";
    const fs::path out_seeded = tmp_root / "seeded";
    const fs::path kf_default = tmp_root / "default_keyframes.xml";
    const fs::path kf_seeded = tmp_root / "seeded_keyframes.xml";
    fs::create_directories(out_default);
    fs::create_directories(out_seeded);

    const auto theta_default = run_linearize_and_read(scene_xml, out_default, kf_default, std::nullopt);
    const auto theta_seeded = run_linearize_and_read(scene_xml, out_seeded, kf_seeded, std::string("eq_hip_p0525"));

    REQUIRE(theta_default.size() == theta_seeded.size());

    for (const auto& [hip, theta0] : theta_default) {
        const auto it = theta_seeded.find(hip);
        REQUIRE(it != theta_seeded.end());
        const double theta1 = it->second;
        INFO("hip=" << hip << " theta_default=" << theta0 << " theta_seeded=" << theta1);
        CHECK(std::abs(theta0 - theta1) < 1e-9);
    }

    fs::remove_all(tmp_root);
}
