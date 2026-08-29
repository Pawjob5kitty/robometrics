// Tests for the JSON report -- the tool's public structured-data format.
//
// Two layers: unit tests serialise a hand-built JsonReport and check the bytes
// with a real JSON parser (nlohmann/json, a test-only dependency), and
// end-to-end tests run the CLI with --json and confirm the JSON carries the
// same numbers as the CSV from the same run. If the two ever disagree, the
// round-trip test below is where it shows.

#include <doctest/doctest.h>

#include <nlohmann/json.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "robometrics/cli.hpp"
#include "robometrics/report.hpp"
#include "robometrics/report_json.hpp"

namespace {

using nlohmann::json;
using robometrics::EfficiencyStatus;
using robometrics::JsonReport;
using robometrics::JsonRollout;
using robometrics::ProfileMode;
using robometrics::RolloutReport;

std::string fixture(const char* f) { return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + f; }

std::string serialize(const JsonReport& report) {
  std::ostringstream os;
  robometrics::writeJsonReport(os, report);
  return os.str();
}

// A rollout entry with a profile and a dexterity margin, for the shape tests.
JsonRollout sampleRollout(const std::string& file, EfficiencyStatus status) {
  JsonRollout roll;
  roll.file = file;
  roll.task = file;
  roll.steps = 3;
  roll.success = true;
  roll.status = status;
  roll.report.dexterityProfile = {0.18, 0.05, 0.20};
  roll.report.dexterityMargin = 0.05;
  roll.report.worstIndex = 1;
  roll.report.pathEfficiency =
      status == EfficiencyStatus::Available ? std::optional<double>(0.97) : std::nullopt;
  return roll;
}

// ---- CLI harness, mirroring test_cli.cpp ----------------------------------

class TempDir {
public:
  TempDir() : path_(make()) { std::filesystem::create_directories(path_); }
  ~TempDir() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;

  std::string str() const { return path_.string(); }
  std::string file(const std::string& name) const { return (path_ / name).string(); }
  std::string write(const std::string& name, const std::string& content) const {
    const std::string full = file(name);
    std::ofstream f(full);
    f << content;
    return full;
  }

private:
  static std::filesystem::path make() {
    static std::atomic<unsigned> counter{0};
    static const unsigned long long tag = std::random_device{}();
    std::ostringstream unique;
    unique << "robometrics_json_" << tag << '_' << counter.fetch_add(1);
    return std::filesystem::temp_directory_path() / unique.str();
  }
  std::filesystem::path path_;
};

/// A rollout file: `dofs` joints, `q1..` driven by the samples, joint 0 held.
std::string rolloutText(const std::vector<std::vector<double>>& rows, int dofs, int success) {
  std::ostringstream s;
  s << "# robometrics rollout v1\n# dofs: " << dofs << "\n# success: " << success << "\nt";
  for (int j = 0; j < dofs; ++j) {
    s << ",q" << j;
  }
  s << "\n";
  for (std::size_t i = 0; i < rows.size(); ++i) {
    s << 0.05 * static_cast<double>(i);
    for (double v : rows[i]) {
      s << ',' << v;
    }
    s << "\n";
  }
  return s.str();
}

std::string readFile(const std::string& path) {
  std::ifstream in(path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::vector<std::string> splitCsvRow(const std::string& row) {
  std::vector<std::string> out;
  std::string field;
  std::istringstream in(row);
  while (std::getline(in, field, ',')) {
    out.push_back(field);
  }
  return out;
}

int runCli(const std::vector<std::string>& args) {
  std::ostringstream out;
  std::ostringstream err;
  return robometrics::runCli(args, out, err);
}

}  // namespace

// ---------------------------------------------------------------------------
// Serialisation shape
// ---------------------------------------------------------------------------

TEST_CASE("the report is valid JSON with the documented top-level shape") {
  JsonReport report;
  report.urdf = "panda_arm.urdf";
  report.numDofs = 7;
  report.characteristicLength = 1.31926;
  report.thresholdNormalized = 0.0379;
  report.tip = "panda_hand";
  report.summary.numRollouts = 1;
  report.summary.dexterityMedian = 0.1066;
  report.rollouts.push_back(sampleRollout("demo_0", EfficiencyStatus::Available));

  const json j = json::parse(serialize(report));  // throws if invalid

  CHECK(j.at("format_version") == 1);
  CHECK(j.at("tool") == "robometrics");
  CHECK(j.at("robot").at("num_dofs") == 7);
  CHECK(j.at("robot").at("characteristic_length").get<double>() == doctest::Approx(1.31926));
  CHECK(j.at("robot").at("urdf") == "panda_arm.urdf");
  CHECK(j.at("params").at("tip") == "panda_hand");
  CHECK(j.at("summary").at("num_rollouts") == 1);
  CHECK(j.at("rollouts").is_array());
  CHECK(j.at("rollouts").size() == 1);
  CHECK(j.at("rollouts")[0].at("file") == "demo_0");
  CHECK(j.at("rollouts")[0].at("profile").is_array());
}

TEST_CASE("an auto-detected tip and an available robot serialise as nulls") {
  JsonReport report;
  report.numDofs = 4;
  report.characteristicLength = 0.6;
  report.robotStatus = EfficiencyStatus::Available;
  // tip left unset

  const json j = json::parse(serialize(report));
  CHECK(j.at("params").at("tip").is_null());
  CHECK(j.at("summary").at("efficiency_na_reason").is_null());
  CHECK(j.at("rollouts").is_array());
  CHECK(j.at("rollouts").empty());
}

TEST_CASE("efficiency_status carries the reason, per rollout and in the summary") {
  JsonReport report;
  report.robotStatus = EfficiencyStatus::MixedJointTypes;
  report.rollouts.push_back(sampleRollout("a", EfficiencyStatus::Available));
  report.rollouts.push_back(sampleRollout("b", EfficiencyStatus::NotRedundant));
  report.rollouts.push_back(sampleRollout("c", EfficiencyStatus::MixedJointTypes));

  const json j = json::parse(serialize(report));
  CHECK(j.at("rollouts")[0].at("efficiency_status") == "available");
  CHECK(j.at("rollouts")[1].at("efficiency_status") == "not_redundant");
  CHECK(j.at("rollouts")[2].at("efficiency_status") == "mixed_joint_types");
  // available rollout has a value; the others are null
  CHECK(j.at("rollouts")[0].at("efficiency").get<double>() == doctest::Approx(0.97));
  CHECK(j.at("rollouts")[1].at("efficiency").is_null());
  CHECK(j.at("summary").at("efficiency_na_reason") == "mixed_joint_types");
}

TEST_CASE("empty and populated span lists both serialise correctly") {
  JsonReport report;
  JsonRollout clean = sampleRollout("clean", EfficiencyStatus::Available);
  JsonRollout flagged = sampleRollout("flagged", EfficiencyStatus::Available);
  flagged.report.lowDexteritySpans = {{10, 14, 0.03}, {40, 41, 0.01}};
  report.rollouts = {clean, flagged};

  const json j = json::parse(serialize(report));
  CHECK(j.at("rollouts")[0].at("spans").is_array());
  CHECK(j.at("rollouts")[0].at("spans").empty());

  const json& spans = j.at("rollouts")[1].at("spans");
  REQUIRE(spans.size() == 2);
  CHECK(spans[0].at("begin") == 10);
  CHECK(spans[0].at("end") == 14);  // half-open, matching Span in report.hpp
  CHECK(spans[0].at("worst").get<double>() == doctest::Approx(0.03));
  CHECK(spans[1].at("begin") == 40);
}

TEST_CASE("profiles: embedded gives an array, none gives null, empty gives []") {
  JsonReport report;
  JsonRollout roll = sampleRollout("r", EfficiencyStatus::Available);
  JsonRollout emptyTraj = sampleRollout("empty", EfficiencyStatus::Available);
  emptyTraj.steps = 0;
  emptyTraj.report = RolloutReport{};  // no profile, no margin
  report.rollouts = {roll, emptyTraj};

  SUBCASE("embedded") {
    report.profiles = ProfileMode::Embedded;
    const json j = json::parse(serialize(report));
    CHECK(j.at("rollouts")[0].at("profile").is_array());
    CHECK(j.at("rollouts")[0].at("profile").size() == 3);
    CHECK(j.at("rollouts")[0].at("profile")[0].get<double>() == doctest::Approx(0.18));
    // an empty trajectory is an empty array, NOT null: it has zero steps, not
    // "profiles suppressed".
    CHECK(j.at("rollouts")[1].at("profile").is_array());
    CHECK(j.at("rollouts")[1].at("profile").empty());
    CHECK(j.at("rollouts")[1].at("dexterity_min").is_null());
    CHECK(j.at("rollouts")[1].at("dexterity_worst_at").is_null());
  }
  SUBCASE("none") {
    report.profiles = ProfileMode::None;
    const json j = json::parse(serialize(report));
    CHECK(j.at("rollouts")[0].at("profile").is_null());
    CHECK(j.at("rollouts")[1].at("profile").is_null());
    // the scalars are still present in 'none' mode
    CHECK(j.at("rollouts")[0].at("dexterity_min").get<double>() == doctest::Approx(0.05));
  }
}

TEST_CASE("success is a tri-state: true, false, or null") {
  JsonReport report;
  JsonRollout yes = sampleRollout("yes", EfficiencyStatus::Available);
  JsonRollout no = sampleRollout("no", EfficiencyStatus::Available);
  JsonRollout unknown = sampleRollout("unknown", EfficiencyStatus::Available);
  no.success = false;
  unknown.success = std::nullopt;
  report.rollouts = {yes, no, unknown};

  const json j = json::parse(serialize(report));
  CHECK(j.at("rollouts")[0].at("success") == true);
  CHECK(j.at("rollouts")[1].at("success") == false);
  CHECK(j.at("rollouts")[2].at("success").is_null());
}

TEST_CASE("strings with quotes and backslashes stay valid and round-trip") {
  JsonReport report;
  JsonRollout roll = sampleRollout("odd\"name\\path", EfficiencyStatus::Available);
  roll.task = "with\ttab and \"quote\"";
  report.rollouts.push_back(roll);
  report.urdf = "C:\\robots\\panda.urdf";

  const json j = json::parse(serialize(report));  // must not throw
  CHECK(j.at("robot").at("urdf") == "C:\\robots\\panda.urdf");
  CHECK(j.at("rollouts")[0].at("file") == "odd\"name\\path");
  CHECK(j.at("rollouts")[0].at("task") == "with\ttab and \"quote\"");
}

// ---------------------------------------------------------------------------
// End to end: the CLI writes JSON that agrees with the CSV
// ---------------------------------------------------------------------------

TEST_CASE("--json and --out carry the same numbers from one run") {
  const TempDir dir;
  // planar_arm (non-redundant): efficiency is N/A, which exercises the null path
  // on both sides. One rollout dips through the singularity to make a span.
  std::vector<std::vector<double>> clean;
  for (int i = 0; i < 20; ++i) {
    clean.push_back({0.0, 1.2 + 0.01 * i});
  }
  std::vector<std::vector<double>> dip;
  for (int i = 0; i < 41; ++i) {
    const double u = static_cast<double>(i) / 40.0;
    dip.push_back({0.0, 1.2 * std::fabs(2.0 * u - 1.0)});
  }
  const std::string a = dir.write("task_alpha_demo_3.csv", rolloutText(clean, 2, 1));
  const std::string b = dir.write("task_alpha_demo_4.csv", rolloutText(dip, 2, 0));
  const std::string csvPath = dir.file("report.csv");
  const std::string jsonPath = dir.file("report.json");

  const int code = runCli({"analyze", "--urdf", fixture("planar_arm.urdf"), "--out", csvPath,
                           "--json", jsonPath, a, b});
  REQUIRE(code == 0);

  const json j = json::parse(readFile(jsonPath));
  REQUIRE(j.at("rollouts").size() == 2);

  // Index the JSON by file stem so the comparison does not assume an order.
  std::vector<std::string> csvLines;
  {
    std::istringstream in(readFile(csvPath));
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
      csvLines.push_back(line);
    }
  }
  REQUIRE(csvLines.size() == 2);

  for (const std::string& line : csvLines) {
    const std::vector<std::string> f = splitCsvRow(line);
    REQUIRE(f.size() == 8);
    const std::string stem = f[0];  // no directory in these names, no extension logic needed
    // Find the JSON rollout whose file matches this CSV row's stem.
    const json* match = nullptr;
    for (const json& r : j.at("rollouts")) {
      if (r.at("file") == std::filesystem::path(stem).stem().string()) {
        match = &r;
      }
    }
    REQUIRE(match != nullptr);

    // dexterity_norm (CSV col 5) == dexterity_min (JSON)
    CHECK(std::stod(f[4]) == doctest::Approx(match->at("dexterity_min").get<double>()));
    // path_efficiency (col 6): empty in CSV <=> null in JSON
    CHECK(f[5].empty() == match->at("efficiency").is_null());
    // low_dex_spans (col 7) == number of spans in JSON
    CHECK(std::stoul(f[6]) == match->at("spans").size());
    // worst_at (col 8) == dexterity_worst_at
    if (!f[7].empty()) {
      CHECK(std::stoul(f[7]) == match->at("dexterity_worst_at").get<std::size_t>());
    }
    // task is the stem minus _demo_N
    CHECK(match->at("task") == "task_alpha");
  }

  // Summary agrees with the run.
  CHECK(j.at("summary").at("num_rollouts") == 2);
  CHECK(j.at("summary").at("rollouts_with_span") == 1);
  CHECK(j.at("summary").at("efficiency_available") == 0);
  CHECK(j.at("summary").at("efficiency_na_reason") == "not_redundant");
}

TEST_CASE("a redundant robot reports an efficiency value and status available") {
  const TempDir dir;
  std::vector<std::vector<double>> rows;
  for (int i = 0; i < 15; ++i) {
    const double a = 0.3 + 0.02 * i;
    rows.push_back({a, -a, a, -a});  // planar_4r, 4 dofs, stays away from singularity
  }
  const std::string in = dir.write("m_demo_0.csv", rolloutText(rows, 4, 1));
  const std::string jsonPath = dir.file("r.json");

  const int code = runCli({"analyze", "--urdf", fixture("planar_4r.urdf"), "--json", jsonPath, in});
  REQUIRE(code == 0);

  const json j = json::parse(readFile(jsonPath));
  REQUIRE(j.at("rollouts").size() == 1);
  CHECK(j.at("rollouts")[0].at("efficiency_status") == "available");
  CHECK(j.at("rollouts")[0].at("efficiency").is_number());
  CHECK(j.at("summary").at("efficiency_available") == 1);
  CHECK(j.at("summary").at("efficiency_na_reason").is_null());
}

TEST_CASE("--json does not change the CSV output") {
  const TempDir dir;
  std::vector<std::vector<double>> rows;
  for (int i = 0; i < 20; ++i) {
    rows.push_back({0.0, 1.2 + 0.01 * i});
  }
  const std::string in = dir.write("clean_demo_1.csv", rolloutText(rows, 2, 1));

  std::ostringstream withoutOut;
  std::ostringstream withoutErr;
  robometrics::runCli({"analyze", "--urdf", fixture("planar_arm.urdf"), in}, withoutOut,
                      withoutErr);

  const std::string jsonPath = dir.file("r.json");
  std::ostringstream withOut;
  std::ostringstream withErr;
  robometrics::runCli({"analyze", "--urdf", fixture("planar_arm.urdf"), "--json", jsonPath, in},
                      withOut, withErr);

  CHECK(withOut.str() == withoutOut.str());  // the CSV on stdout is byte-identical
}

TEST_CASE("profiles none omits the arrays but keeps the scalars, on a real run") {
  const TempDir dir;
  std::vector<std::vector<double>> rows;
  for (int i = 0; i < 50; ++i) {
    rows.push_back({0.0, 1.2 + 0.005 * i});
  }
  const std::string in = dir.write("big_demo_0.csv", rolloutText(rows, 2, 1));
  const std::string embedded = dir.file("emb.json");
  const std::string none = dir.file("none.json");

  runCli({"analyze", "--urdf", fixture("planar_arm.urdf"), "--json", embedded, in});
  runCli(
      {"analyze", "--urdf", fixture("planar_arm.urdf"), "--json", none, "--profiles", "none", in});

  const json je = json::parse(readFile(embedded));
  const json jn = json::parse(readFile(none));
  CHECK(je.at("rollouts")[0].at("profile").size() == 50);
  CHECK(jn.at("rollouts")[0].at("profile").is_null());
  // scalars unchanged between the two modes
  CHECK(je.at("rollouts")[0].at("dexterity_min") == jn.at("rollouts")[0].at("dexterity_min"));
}

TEST_CASE("--profiles rejects an unknown mode") {
  const TempDir dir;
  const std::string in = dir.write("x_demo_0.csv", rolloutText({{0.0, 1.2}, {0.0, 1.3}}, 2, 1));
  const int code =
      runCli({"analyze", "--urdf", fixture("planar_arm.urdf"), "--profiles", "sometimes", in});
  CHECK(code == 2);
}
