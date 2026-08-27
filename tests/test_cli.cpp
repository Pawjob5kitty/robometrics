// End-to-end tests for the command line.
//
// These drive runCli() directly rather than spawning the built binary. The
// files are real and the CSV is really produced; what is skipped is subprocess
// plumbing, whose failures would be indistinguishable from failures of the code
// under test. See cli.hpp.

#include <doctest/doctest.h>

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "robometrics/cli.hpp"

namespace {

std::string fixture(const char* f) { return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + f; }

/// A scratch directory removed when the test finishes, pass or fail.
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

  /// Writes a rollout file and returns its path.
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
    unique << "robometrics_cli_" << tag << '_' << counter.fetch_add(1);
    return std::filesystem::temp_directory_path() / unique.str();
  }

  std::filesystem::path path_;
};

/// A 2-DOF rollout whose second joint follows the given samples. planar_arm is
/// singular at q1 == 0, so the caller controls where the dips are.
std::string rolloutText(const std::vector<double>& q1, int success) {
  std::ostringstream s;
  s << "# robometrics rollout v1\n# dofs: 2\n# success: " << success << "\nt,q0,q1\n";
  for (std::size_t i = 0; i < q1.size(); ++i) {
    s << 0.05 * static_cast<double>(i) << ",0," << q1[i] << "\n";
  }
  return s.str();
}

std::vector<double> ramp(int n, double from, double to) {
  std::vector<double> v;
  for (int i = 0; i < n; ++i) {
    v.push_back(from + (to - from) * i / (n - 1));
  }
  return v;
}

/// Splits captured output into lines, dropping a trailing empty one.
std::vector<std::string> lines(const std::string& text) {
  std::vector<std::string> out;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    out.push_back(line);
  }
  return out;
}

struct Run {
  int code;
  std::string out;
  std::string err;
};

Run run(const std::vector<std::string>& args) {
  std::ostringstream out;
  std::ostringstream err;
  const int code = robometrics::runCli(args, out, err);
  return {code, out.str(), err.str()};
}

}  // namespace

// ---------------------------------------------------------------------------
// The happy path, end to end
// ---------------------------------------------------------------------------

TEST_CASE("one clean rollout produces one CSV row and exit code 0") {
  const TempDir dir;
  const std::string input = dir.write("clean.csv", rolloutText(ramp(30, 1.2, 1.4), 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), input});

  CHECK(r.code == 0);
  const std::vector<std::string> csv = lines(r.out);
  REQUIRE(csv.size() == 2);  // header plus one row
  CHECK(csv[0] ==
        "file,dofs,steps,success,dexterity_margin,path_efficiency,low_dex_spans,worst_at");
  CHECK(csv[1].find("clean.csv") != std::string::npos);

  // Column count must match the header, or every downstream reader misaligns.
  const auto commas = [](const std::string& s) { return std::count(s.begin(), s.end(), ','); };
  CHECK(commas(csv[1]) == commas(csv[0]));

  // A clean rollout: no spans, and the summary says so.
  CHECK(csv[1].find(",0,") != std::string::npos);
  CHECK(r.err.find("1 rollout, 1 ok, 0 failed") != std::string::npos);
}

TEST_CASE("metadata reaches the CSV") {
  const TempDir dir;
  const std::string input = dir.write("m.csv", rolloutText(ramp(20, 1.2, 1.3), 0));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), input});
  const std::vector<std::string> csv = lines(r.out);
  REQUIRE(csv.size() == 2);

  // dofs, steps and success come from the file, not from the robot.
  CHECK(csv[1].find(",2,20,0,") != std::string::npos);
}

TEST_CASE("a rollout through a singularity is flagged and localised on stderr") {
  // The whole point of the layer: not that the number is bad, but where.
  const TempDir dir;
  std::vector<double> dip;
  for (int i = 0; i < 41; ++i) {
    const double u = static_cast<double>(i) / 40.0;
    dip.push_back(1.2 * std::fabs(2.0 * u - 1.0));  // through zero at the middle
  }
  const std::string input = dir.write("dip.csv", rolloutText(dip, 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), input});
  CHECK(r.code == 0);

  CHECK(r.err.find("dip.csv") != std::string::npos);
  CHECK(r.err.find("low-dexterity span") != std::string::npos);
  CHECK(r.err.find("worst") != std::string::npos);
  CHECK(r.err.find("1 rollout (100%) have at least one low-dexterity span") != std::string::npos);
}

TEST_CASE("stderr reports spans with INCLUSIVE bounds") {
  // The struct is half-open, the printed range is inclusive, and the
  // conversion happens exactly once. Getting it wrong would name a step the
  // robot was fine at.
  //
  // The rollout below is singular from the start and recovers, so the span
  // begins at 0 and its printed end must be one less than the first good step.
  const TempDir dir;
  std::vector<double> q1;
  for (int i = 0; i < 20; ++i) {
    q1.push_back(1.2 * i / 19.0);  // starts stretched, opens up
  }
  const std::string input = dir.write("start.csv", rolloutText(q1, 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), input});
  CHECK(r.err.find("[0..") != std::string::npos);

  // Cross-check against the profile the tool itself writes: the last index
  // named in the span must still be below the threshold, and the next one must
  // not be. Reading it back from --profile-out means the two outputs have to
  // agree with each other, not merely each with itself.
  const TempDir profiles;
  run({"analyze", "--urdf", fixture("planar_arm.urdf"), "--profile-out", profiles.str(), input});

  std::ifstream prof(profiles.file("start_profile.csv"));
  REQUIRE(prof.good());
  std::string line;
  std::getline(prof, line);  // header
  std::vector<double> values;
  while (std::getline(prof, line)) {
    const std::size_t lastComma = line.rfind(',');
    REQUIRE(lastComma != std::string::npos);
    values.push_back(std::stod(line.substr(lastComma + 1)));
  }
  REQUIRE(values.size() == 20);

  // Parse "[0..N" out of stderr and check the boundary against the profile.
  const std::size_t open = r.err.find("[0..");
  REQUIRE(open != std::string::npos);
  const std::size_t numStart = open + 4;
  const std::size_t numEnd = r.err.find(' ', numStart);
  REQUIRE(numEnd != std::string::npos);
  const std::size_t lastBad = std::stoul(r.err.substr(numStart, numEnd - numStart));

  CHECK(values[lastBad] < 0.05);
  REQUIRE(lastBad + 1 < values.size());
  CHECK(values[lastBad + 1] >= 0.05);
}

// ---------------------------------------------------------------------------
// Robustness across a batch
// ---------------------------------------------------------------------------

TEST_CASE("one broken file among good ones is skipped, not fatal") {
  // The rule the exit code encodes: a batch of forty with one corrupt file is
  // a successful batch with a warning.
  const TempDir dir;
  const std::string a = dir.write("a.csv", rolloutText(ramp(20, 1.2, 1.3), 1));
  const std::string bad = dir.write("bad.csv", "# dofs: 2\nt,q0,q1\n0,1,2\n0.05,3\n");
  const std::string c = dir.write("c.csv", rolloutText(ramp(25, 1.1, 1.4), 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), a, bad, c});

  CHECK(r.code == 0);  // two succeeded

  const std::vector<std::string> csv = lines(r.out);
  REQUIRE(csv.size() == 3);  // header plus the two good rollouts
  CHECK(csv[1].find("a.csv") != std::string::npos);
  CHECK(csv[2].find("c.csv") != std::string::npos);

  // The skip is reported, and carries the parser's line number with it.
  CHECK(r.err.find("skipping") != std::string::npos);
  CHECK(r.err.find("bad.csv:4") != std::string::npos);
  CHECK(r.err.find("3 rollouts, 2 ok, 1 failed to parse") != std::string::npos);
}

TEST_CASE("the LAST rollout in the batch is not dropped") {
  // A loop that stops one short would leave the summary and the CSV
  // consistent with each other and simply lose a file. Three inputs, three
  // rows, and the last one named explicitly.
  const TempDir dir;
  const std::string a = dir.write("first.csv", rolloutText(ramp(15, 1.2, 1.3), 1));
  const std::string b = dir.write("second.csv", rolloutText(ramp(15, 1.1, 1.2), 1));
  const std::string c = dir.write("third.csv", rolloutText(ramp(15, 1.0, 1.1), 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), a, b, c});

  const std::vector<std::string> csv = lines(r.out);
  REQUIRE(csv.size() == 4);
  CHECK(csv[1].find("first.csv") != std::string::npos);
  CHECK(csv[2].find("second.csv") != std::string::npos);
  CHECK(csv[3].find("third.csv") != std::string::npos);
  CHECK(r.err.find("3 rollouts, 3 ok, 0 failed") != std::string::npos);
}

TEST_CASE("a batch where everything fails returns non-zero") {
  const TempDir dir;
  const std::string bad1 = dir.write("b1.csv", "not a rollout at all\n");
  const std::string bad2 = dir.write("b2.csv", "# dofs: 2\n");

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), bad1, bad2});

  CHECK(r.code != 0);
  CHECK(r.err.find("2 rollouts, 0 ok, 2 failed to parse") != std::string::npos);
  // Only the header row survives.
  CHECK(lines(r.out).size() == 1);
}

TEST_CASE("a rollout whose width disagrees with the URDF is skipped, not fatal") {
  const TempDir dir;
  const std::string wrong =
      dir.write("wide.csv", "# dofs: 7\nt,q0,q1,q2,q3,q4,q5,q6\n0,0,0,0,0,0,0,0\n");
  const std::string good = dir.write("good.csv", rolloutText(ramp(15, 1.2, 1.3), 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), wrong, good});

  CHECK(r.code == 0);
  CHECK(r.err.find("skipping") != std::string::npos);
  CHECK(lines(r.out).size() == 2);
}

// ---------------------------------------------------------------------------
// Output routing and options
// ---------------------------------------------------------------------------

TEST_CASE("--out writes the CSV to a file and leaves stdout empty") {
  const TempDir dir;
  const std::string input = dir.write("x.csv", rolloutText(ramp(20, 1.2, 1.3), 1));
  const std::string outPath = dir.file("report.csv");

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), "--out", outPath, input});
  CHECK(r.code == 0);
  CHECK(r.out.empty());

  std::ifstream produced(outPath);
  REQUIRE(produced.good());
  std::ostringstream contents;
  contents << produced.rdbuf();
  const std::vector<std::string> csv = lines(contents.str());
  REQUIRE(csv.size() == 2);
  CHECK(csv[0].rfind("file,dofs,steps", 0) == 0);

  // The summary still reaches stderr, so a person watching the run sees it
  // even though the CSV went to a file.
  CHECK(r.err.find("1 ok") != std::string::npos);
}

TEST_CASE("--profile-out writes one profile per rollout") {
  const TempDir dir;
  const TempDir profiles;
  const std::string a = dir.write("pa.csv", rolloutText(ramp(12, 1.2, 1.3), 1));
  const std::string b = dir.write("pb.csv", rolloutText(ramp(9, 1.1, 1.2), 1));

  const Run r =
      run({"analyze", "--urdf", fixture("planar_arm.urdf"), "--profile-out", profiles.str(), a, b});
  CHECK(r.code == 0);

  CHECK(std::filesystem::exists(profiles.file("pa_profile.csv")));
  CHECK(std::filesystem::exists(profiles.file("pb_profile.csv")));

  std::ifstream f(profiles.file("pb_profile.csv"));
  std::ostringstream contents;
  contents << f.rdbuf();
  const std::vector<std::string> rows = lines(contents.str());
  CHECK(rows[0] == "step,t,dexterity");
  CHECK(rows.size() == 10);  // header plus nine steps
  CHECK(rows[1].rfind("0,", 0) == 0);
}

TEST_CASE("--threshold changes how much is flagged") {
  const TempDir dir;
  const std::string input = dir.write("t.csv", rolloutText(ramp(30, 1.2, 1.4), 1));

  const Run strict = run({"analyze", "--urdf", fixture("planar_arm.urdf"), input});
  const Run loose =
      run({"analyze", "--urdf", fixture("planar_arm.urdf"), "--threshold", "10", input});

  CHECK(strict.err.find("0 rollouts (0%)") != std::string::npos);
  CHECK(loose.err.find("1 rollout (100%)") != std::string::npos);
}

// ---------------------------------------------------------------------------
// Argument errors
// ---------------------------------------------------------------------------

TEST_CASE("missing or malformed arguments are rejected with usage, exit 2") {
  const TempDir dir;
  const std::string input = dir.write("i.csv", rolloutText(ramp(10, 1.2, 1.3), 1));

  SUBCASE("no --urdf") {
    const Run r = run({"analyze", input});
    CHECK(r.code == 2);
    CHECK(r.err.find("--urdf is required") != std::string::npos);
    CHECK(r.err.find("usage:") != std::string::npos);
  }
  SUBCASE("no inputs") {
    const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf")});
    CHECK(r.code == 2);
    CHECK(r.err.find("no rollout files") != std::string::npos);
  }
  SUBCASE("unknown subcommand") {
    const Run r = run({"summarise", "--urdf", fixture("planar_arm.urdf"), input});
    CHECK(r.code == 2);
    CHECK(r.err.find("unknown subcommand") != std::string::npos);
  }
  SUBCASE("a mistyped flag is not treated as a filename") {
    // Otherwise --thresold becomes a rollout path and is reported as a missing
    // file, sending the reader looking in entirely the wrong place.
    const Run r =
        run({"analyze", "--urdf", fixture("planar_arm.urdf"), "--thresold", "0.1", input});
    CHECK(r.code == 2);
    CHECK(r.err.find("unknown option '--thresold'") != std::string::npos);
  }
  SUBCASE("a flag with no value") {
    const Run r = run({"analyze", "--urdf"});
    CHECK(r.code == 2);
    CHECK(r.err.find("needs a value") != std::string::npos);
  }
  SUBCASE("a non-numeric threshold") {
    const Run r =
        run({"analyze", "--urdf", fixture("planar_arm.urdf"), "--threshold", "low", input});
    CHECK(r.code == 2);
    CHECK(r.err.find("--threshold") != std::string::npos);
  }
  SUBCASE("a URDF that does not exist") {
    const Run r = run({"analyze", "--urdf", dir.file("nope.urdf"), input});
    CHECK(r.code == 2);
    CHECK(r.err.find("cannot load URDF") != std::string::npos);
  }
}

TEST_CASE("--help prints usage and succeeds") {
  const Run r = run({"--help"});
  CHECK(r.code == 0);
  CHECK(r.err.find("usage:") != std::string::npos);
  CHECK(r.out.empty());
}

// ---------------------------------------------------------------------------
// Summary statistics
// ---------------------------------------------------------------------------

TEST_CASE("summary percentiles are values that actually occurred") {
  // Nearest-rank, no interpolation: every number printed is a value some
  // rollout produced, so "which file was that?" always has an answer.
  const TempDir dir;
  std::vector<std::string> inputs;
  for (int k = 0; k < 5; ++k) {
    inputs.push_back(dir.write("r" + std::to_string(k) + ".csv",
                               rolloutText(ramp(20, 0.4 + 0.2 * k, 0.5 + 0.2 * k), 1)));
  }

  std::vector<std::string> args{"analyze", "--urdf", fixture("planar_arm.urdf")};
  args.insert(args.end(), inputs.begin(), inputs.end());
  const Run r = run(args);
  REQUIRE(r.code == 0);

  // Collect the margins the CSV reported, then check the printed min is one of
  // them -- and specifically the smallest.
  std::vector<double> margins;
  const std::vector<std::string> csv = lines(r.out);
  REQUIRE(csv.size() == 6);
  for (std::size_t i = 1; i < csv.size(); ++i) {
    std::vector<std::string> fields;
    std::istringstream row(csv[i]);
    std::string field;
    while (std::getline(row, field, ',')) {
      fields.push_back(field);
    }
    REQUIRE(fields.size() == 8);
    margins.push_back(std::stod(fields[4]));
  }
  std::sort(margins.begin(), margins.end());

  std::ostringstream expectedMin;
  expectedMin << "min " << [&] {
    std::ostringstream s;
    s.precision(6);
    s << margins.front();
    return s.str();
  }();
  CHECK(r.err.find(expectedMin.str()) != std::string::npos);
}

TEST_CASE("a rollout with no computable metrics leaves empty fields, not zeros") {
  // A single-step rollout has no path efficiency. Writing 0 would make it the
  // worst row in the file; empty is what a dataframe reads back as missing.
  const TempDir dir;
  const std::string input = dir.write("one.csv", "# dofs: 2\n# success: 1\nt,q0,q1\n0,0,1.2\n");

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), input});
  CHECK(r.code == 0);

  const std::vector<std::string> csv = lines(r.out);
  REQUIRE(csv.size() == 2);

  std::vector<std::string> fields;
  std::istringstream row(csv[1]);
  std::string field;
  while (std::getline(row, field, ',')) {
    fields.push_back(field);
  }
  REQUIRE(fields.size() == 8);
  CHECK_FALSE(fields[4].empty());  // dexterity margin exists for one step
  CHECK(fields[5].empty());        // path efficiency does not
}

// ---------------------------------------------------------------------------
// Noise floor, dropped column, redundancy warning
// ---------------------------------------------------------------------------

TEST_CASE("numerical noise near a singularity prints as a clean zero") {
  // The noise appears just OFF the singularity, not at it: at q1 == 0 exactly
  // the SVD returns a clean 0.0 on its own, so that configuration would pass
  // with the clamp removed and prove nothing. At q1 = 1e-12 sigma_min comes out
  // around 1.3e-13 from cancellation -- a precision the number does not have,
  // and eleven orders below anything that could matter.
  const TempDir dir;
  const std::string input = dir.write("sing.csv", rolloutText(std::vector<double>(12, 1e-12), 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), input});
  REQUIRE(r.code == 0);

  const std::vector<std::string> csv = lines(r.out);
  REQUIRE(csv.size() == 2);
  std::vector<std::string> fields;
  std::istringstream row(csv[1]);
  std::string field;
  while (std::getline(row, field, ',')) {
    fields.push_back(field);
  }
  REQUIRE(fields.size() == 8);
  CHECK(fields[4] == "0");  // not 1.34176e-13

  // Not just the CSV: the summary and the span line have to agree with it, so
  // the clamp cannot be applied in one output and forgotten in another.
  CHECK(r.err.find("e-1") == std::string::npos);
  CHECK(r.err.find("worst 0]") != std::string::npos);
  CHECK(r.err.find("min 0\n") != std::string::npos);
}

TEST_CASE("a real small value is not flattened") {
  // The clamp must be a noise floor, not a rounding of everything small. 1e-12
  // is eleven orders below the default threshold, so any value that could
  // matter survives.
  const TempDir dir;
  const std::string input = dir.write("small.csv", rolloutText(ramp(15, 0.02, 0.05), 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), input});
  const std::vector<std::string> csv = lines(r.out);
  REQUIRE(csv.size() == 2);
  std::vector<std::string> fields;
  std::istringstream row(csv[1]);
  std::string field;
  while (std::getline(row, field, ',')) {
    fields.push_back(field);
  }
  REQUIRE(fields.size() == 8);
  CHECK(fields[4] != "0");
  CHECK(std::stod(fields[4]) > 0.0);
}

TEST_CASE("worst_dex is gone and worst_at is still the last column") {
  // The dropped column, pinned so it cannot come back by accident. worst_dex
  // was identical to dexterity_margin by construction.
  const TempDir dir;
  const std::string input = dir.write("cols.csv", rolloutText(ramp(20, 1.2, 1.3), 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), input});
  const std::vector<std::string> csv = lines(r.out);
  REQUIRE(csv.size() == 2);
  CHECK(csv[0].find("worst_dex") == std::string::npos);
  CHECK(csv[0].substr(csv[0].rfind(',') + 1) == "worst_at");
  CHECK(std::count(csv[0].begin(), csv[0].end(), ',') == 7);
  CHECK(std::count(csv[1].begin(), csv[1].end(), ',') == 7);
}

TEST_CASE("a non-redundant robot is warned about, once") {
  // The warning exists because a reader who does not know this draws exactly
  // the wrong conclusion: a column of 1.000 looks like a flawless policy and
  // is in fact a statement about the mechanism.
  const TempDir dir;
  const std::string a = dir.write("w1.csv", rolloutText(ramp(15, 1.2, 1.3), 1));
  const std::string b = dir.write("w2.csv", rolloutText(ramp(15, 1.1, 1.2), 1));

  const Run r = run({"analyze", "--urdf", fixture("planar_arm.urdf"), a, b});
  REQUIRE(r.code == 0);

  const std::string needle = "is not redundant";
  const std::size_t first = r.err.find(needle);
  CHECK(first != std::string::npos);
  // Once, not once per rollout: two inputs must not produce two warnings.
  CHECK(r.err.find(needle, first + 1) == std::string::npos);

  CHECK(r.err.find("2 dofs") != std::string::npos);
  CHECK(r.err.find("rank 2") != std::string::npos);
  CHECK(r.err.find("path_efficiency") != std::string::npos);
}

TEST_CASE("a redundant robot is not warned about") {
  // planar_4r has four joints and rank 3, so the null space is real and
  // path_efficiency means something. The warning must stay quiet.
  const TempDir dir;
  std::ostringstream text;
  text << "# dofs: 4\nt,q0,q1,q2,q3\n";
  for (int i = 0; i < 20; ++i) {
    text << 0.05 * i << "," << 0.8 * i / 19.0 << ",0.7,-0.4,0.5\n";
  }
  const std::string input = dir.write("r4.csv", text.str());

  const Run r = run({"analyze", "--urdf", fixture("planar_4r.urdf"), input});
  REQUIRE(r.code == 0);
  CHECK(r.err.find("is not redundant") == std::string::npos);
}

TEST_CASE("the redundancy probe takes the maximum rank, not the rank at one pose") {
  // planar_3r is NOT redundant (three joints, rank 3), but it starts this
  // rollout fully stretched, where the rank drops to 2. A probe that looked at
  // only the first configuration would see 2 < 3 and stay silent -- exactly
  // the case the warning exists for.
  const TempDir dir;
  std::ostringstream text;
  text << "# dofs: 3\nt,q0,q1,q2\n";
  for (int i = 0; i < 20; ++i) {
    // q1 starts at 0 (stretched, rank-deficient) and opens up.
    text << 0.05 * i << ",0," << 1.2 * i / 19.0 << ",0.3\n";
  }
  const std::string input = dir.write("r3.csv", text.str());

  const Run r = run({"analyze", "--urdf", fixture("planar_3r.urdf"), input});
  REQUIRE(r.code == 0);
  CHECK(r.err.find("is not redundant") != std::string::npos);
  CHECK(r.err.find("3 dofs") != std::string::npos);
  CHECK(r.err.find("rank 3") != std::string::npos);
}
