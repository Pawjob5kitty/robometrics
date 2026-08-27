// Tests for the rollout text format.
//
// Error cases are stated inline as strings rather than as fixture files. That
// is the opposite choice from test_urdf.cpp, and deliberate: a malformed
// rollout is three lines long and reads better next to the assertion about it,
// whereas a malformed URDF needs its whole tree to be malformed in context.
// One real file on disk covers the loadRollout() path so the string-based
// tests are not testing a road that nothing drives on.

#include <doctest/doctest.h>

#include <Eigen/Core>
#include <atomic>
#include <filesystem>
#include <random>
#include <sstream>
#include <string>

#include "robometrics/rollout.hpp"

namespace {

using robometrics::Rollout;
using robometrics::RolloutError;

std::string fixture(const char* f) { return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + f; }

/// Runs the parser expecting failure, and hands the error to the caller so the
/// test can say what the message must contain. Asserting on the message is not
/// gold-plating here: the line number IS the feature, and a test that only
/// checks "it threw" would pass on a parser that reports every fault as line 0.
template <typename Fn>
void expectError(const char* text, Fn&& inspect) {
  try {
    robometrics::parseRollout(text, "demo.csv");
    FAIL("expected RolloutError");
  } catch (const RolloutError& e) {
    inspect(e);
  }
}

/// Unique scratch path inside the system temp directory, removed on scope exit.
class TempFile {
public:
  explicit TempFile(const char* name) : path_(makePath(name)) {}
  ~TempFile() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }
  TempFile(const TempFile&) = delete;
  TempFile& operator=(const TempFile&) = delete;

  std::string str() const { return path_.string(); }

private:
  // Deliberately not getpid(): that is POSIX only and this project builds on
  // Windows too. A random 64-bit tag plus a counter makes collisions between
  // concurrent test runs a non-issue without a platform #ifdef.
  static std::filesystem::path makePath(const char* name) {
    static std::atomic<unsigned> counter{0};
    static const unsigned long long tag = std::random_device{}();
    std::ostringstream unique;
    unique << "robometrics_test_" << tag << '_' << counter.fetch_add(1) << '_' << name;
    return std::filesystem::temp_directory_path() / unique.str();
  }

  std::filesystem::path path_;
};

}  // namespace

// ---------------------------------------------------------------------------
// Happy path
// ---------------------------------------------------------------------------

TEST_CASE("a valid rollout file loads with its metadata") {
  const Rollout r = robometrics::loadRollout(fixture("rollouts/valid_2dof.csv"));

  CHECK(r.dofs == 2);
  CHECK(r.size() == 4);
  REQUIRE(r.t.size() == r.q.size());

  CHECK(r.t[0] == doctest::Approx(0.0));
  CHECK(r.t[3] == doctest::Approx(0.15));
  CHECK(r.q[0](0) == doctest::Approx(0.0));
  CHECK(r.q[0](1) == doctest::Approx(1.5707963267948966));
  CHECK(r.q[3](0) == doctest::Approx(0.3));

  // Unknown keys are kept rather than rejected, so a converter can record
  // provenance without this parser having to learn about it.
  CHECK(r.meta.at("robot") == "planar_arm.urdf");
  CHECK(r.meta.at("success") == "1");
  CHECK(r.meta.at("task") == "reach_left");
}

TEST_CASE("a rollout with a header and no data rows is legal") {
  // Zero steps, not an error. Recordings do get cut short, and the metrics
  // already return nullopt for an empty trajectory -- rejecting it here would
  // turn a recoverable situation into a crash in a batch run.
  const Rollout r = robometrics::parseRollout("# dofs: 3\nt,q0,q1,q2\n", "demo.csv");
  CHECK(r.dofs == 3);
  CHECK(r.size() == 0);
  CHECK(r.q.empty());
}

TEST_CASE("blank lines and comment-only banners are ignored") {
  const Rollout r = robometrics::parseRollout(
      "# robometrics rollout v1\n"
      "\n"
      "# dofs: 1\n"
      "\n"
      "t,q0\n"
      "0,1\n"
      "\n"
      "1,2\n",
      "demo.csv");
  CHECK(r.size() == 2);
  CHECK(r.q[1](0) == doctest::Approx(2.0));
}

TEST_CASE("the decimal point does not depend on the locale") {
  // from_chars has no locale to get wrong. strtod-family parsing on a machine
  // with a Czech or German locale reads "0.05" as 0, and the file would load
  // without error into a trajectory that never moves -- a silent wrong answer,
  // which is the worst kind. This test does not switch locales; it pins the
  // value so a future switch back to stod would be caught.
  const Rollout r = robometrics::parseRollout("# dofs: 1\nt,q0\n0,0.05\n", "demo.csv");
  REQUIRE(r.size() == 1);
  CHECK(r.q[0](0) == doctest::Approx(0.05));
  CHECK(r.q[0](0) > 0.0);
}

TEST_CASE("CRLF line endings parse the same as LF") {
  // A file authored on Windows leaves '\r' on the end of every line. Without
  // stripping it, the last numeric field of every row fails to parse.
  const Rollout r =
      robometrics::parseRollout("# dofs: 2\r\nt,q0,q1\r\n0,1,2\r\n0.1,3,4\r\n", "demo.csv");
  REQUIRE(r.size() == 2);
  CHECK(r.q[1](1) == doctest::Approx(4.0));
}

TEST_CASE("whitespace around fields is tolerated") {
  const Rollout r =
      robometrics::parseRollout("#  dofs : 2 \nt, q0 , q1\n 0 , 1 , 2 \n", "demo.csv");
  REQUIRE(r.size() == 1);
  CHECK(r.dofs == 2);
  CHECK(r.q[0](1) == doctest::Approx(2.0));
}

// ---------------------------------------------------------------------------
// Round trip
// ---------------------------------------------------------------------------

TEST_CASE("write then read reproduces the rollout exactly") {
  // Exactly, not approximately. formatRollout writes the shortest decimal that
  // round-trips, so this is a bit-for-bit claim -- if it degrades to 15 digits
  // the == below starts failing rather than quietly losing precision.
  Rollout original;
  original.dofs = 3;
  original.meta["robot"] = "planar_3r.urdf";
  original.meta["success"] = "0";
  for (int i = 0; i < 5; ++i) {
    original.t.push_back(0.05 * i);
    Eigen::VectorXd q(3);
    q << 0.1 * i, -0.7853981633974483, 1.0 / 3.0;
    original.q.push_back(q);
  }

  const Rollout back = robometrics::parseRollout(robometrics::formatRollout(original), "roundtrip");

  REQUIRE(back.dofs == original.dofs);
  REQUIRE(back.size() == original.size());
  for (std::size_t i = 0; i < original.size(); ++i) {
    CHECK(back.t[i] == original.t[i]);
    for (Eigen::Index j = 0; j < original.q[i].size(); ++j) {
      CHECK(back.q[i](j) == original.q[i](j));
    }
  }
  CHECK(back.meta.at("robot") == "planar_3r.urdf");
  CHECK(back.meta.at("success") == "0");
  CHECK(back.meta.at("dofs") == "3");
}

TEST_CASE("saveRollout and loadRollout agree with the in-memory versions") {
  const Rollout original = robometrics::loadRollout(fixture("rollouts/valid_2dof.csv"));

  const TempFile tmp("roundtrip.csv");
  robometrics::saveRollout(tmp.str(), original);
  const Rollout back = robometrics::loadRollout(tmp.str());

  REQUIRE(back.size() == original.size());
  CHECK(back.dofs == original.dofs);
  for (std::size_t i = 0; i < original.size(); ++i) {
    CHECK(back.t[i] == original.t[i]);
    CHECK(back.q[i](0) == original.q[i](0));
    CHECK(back.q[i](1) == original.q[i](1));
  }
}

TEST_CASE("written files are byte-identical across runs") {
  // Deterministic output is what makes these files diffable, which was half
  // the reason for choosing text. std::map's sorted iteration is what
  // guarantees it; an unordered_map would have broken this silently.
  Rollout r;
  r.dofs = 1;
  r.meta["zebra"] = "1";
  r.meta["alpha"] = "2";
  r.t.push_back(0.0);
  Eigen::VectorXd q(1);
  q << 0.5;
  r.q.push_back(q);

  CHECK(robometrics::formatRollout(r) == robometrics::formatRollout(r));
  const std::string text = robometrics::formatRollout(r);
  CHECK(text.find("# alpha: 2\n# zebra: 1\n") != std::string::npos);
}

// ---------------------------------------------------------------------------
// Errors, one kind at a time
// ---------------------------------------------------------------------------

TEST_CASE("a row with the wrong number of columns names the line and both counts") {
  expectError(
      "# dofs: 2\n"
      "t,q0,q1\n"
      "0,0.1,0.2\n"
      "0.05,0.3\n",
      [](const RolloutError& e) {
        CHECK(e.line() == 4);  // the short row, not the header, not the file
        CHECK(std::string(e.what()).find("demo.csv:4") != std::string::npos);
        CHECK(e.detail().find("expected 3") != std::string::npos);
        CHECK(e.detail().find("found 2") != std::string::npos);
      });
}

TEST_CASE("missing dofs is a whole-file error, not a line error") {
  expectError(
      "# robot: panda.urdf\n"
      "t,q0,q1\n"
      "0,1,2\n",
      [](const RolloutError& e) {
        CHECK(e.line() == 0);  // no single line is to blame
        CHECK(e.detail().find("dofs") != std::string::npos);
        // The message should show what a correct line looks like, because the
        // reader is most likely hand-writing the file when they hit this.
        CHECK(e.detail().find("# dofs:") != std::string::npos);
      });
}

TEST_CASE("a non-numeric value names the line and the column") {
  expectError(
      "# dofs: 2\n"
      "t,q0,q1\n"
      "0,0.1,banana\n",
      [](const RolloutError& e) {
        CHECK(e.line() == 3);
        CHECK(e.detail().find("q1") != std::string::npos);
        CHECK(e.detail().find("banana") != std::string::npos);
      });
}

TEST_CASE("trailing junk after a number is rejected, not truncated") {
  // from_chars stops at the junk and reports where it stopped; accepting
  // "0.05abc" as 0.05 would be a silent data corruption.
  expectError("# dofs: 1\nt,q0\n0,0.05abc\n", [](const RolloutError& e) {
    CHECK(e.line() == 3);
    CHECK(e.detail().find("0.05abc") != std::string::npos);
  });
}

TEST_CASE("an empty file is an error, and says why") {
  expectError("", [](const RolloutError& e) {
    CHECK(e.line() == 0);
    CHECK(e.detail().find("empty") != std::string::npos);
  });
}

TEST_CASE("a file of nothing but comments is an error too") {
  expectError("# robometrics rollout v1\n# dofs: 2\n", [](const RolloutError& e) {
    CHECK(e.line() == 0);
    CHECK(e.detail().find("column header") != std::string::npos);
  });
}

TEST_CASE("a column header disagreeing with dofs is caught at the header") {
  // The important one. Without this check the file would parse -- every row
  // has six fields and six fields is what the rows have -- and surface later
  // as a robot whose configuration vector is the wrong width.
  expectError(
      "# dofs: 7\n"
      "t,q0,q1,q2,q3,q4\n"
      "0,1,2,3,4,5\n",
      [](const RolloutError& e) {
        CHECK(e.line() == 2);  // the header line, where the disagreement is
        CHECK(e.detail().find("dofs=7") != std::string::npos);
        CHECK(e.detail().find("6") != std::string::npos);
      });
}

TEST_CASE("a misnamed column is caught") {
  expectError("# dofs: 2\nt,q0,q2\n0,1,2\n", [](const RolloutError& e) {
    CHECK(e.line() == 2);
    CHECK(e.detail().find("q1") != std::string::npos);
  });
}

TEST_CASE("a header not starting with t is caught") {
  expectError("# dofs: 1\ntime,q0\n0,1\n", [](const RolloutError& e) {
    CHECK(e.line() == 2);
    CHECK(e.detail().find("'t'") != std::string::npos);
  });
}

TEST_CASE("dofs must be a positive integer") {
  expectError("# dofs: 0\nt\n", [](const RolloutError& e) {
    CHECK(e.detail().find("positive integer") != std::string::npos);
  });
  expectError("# dofs: two\nt,q0\n",
              [](const RolloutError& e) { CHECK(e.detail().find("two") != std::string::npos); });
}

TEST_CASE("a comment among the data rows is rejected, not skipped") {
  // More likely a half-deleted row than an intentional comment. Skipping it
  // would silently drop a step from the trajectory.
  expectError(
      "# dofs: 1\n"
      "t,q0\n"
      "0,1\n"
      "# 0.05,2\n"
      "0.1,3\n",
      [](const RolloutError& e) {
        CHECK(e.line() == 4);
        CHECK(e.detail().find("header") != std::string::npos);
      });
}

TEST_CASE("a missing file is reported as a missing file") {
  try {
    robometrics::loadRollout(fixture("rollouts/does_not_exist.csv"));
    FAIL("expected RolloutError");
  } catch (const RolloutError& e) {
    CHECK(std::string(e.what()).find("could not be opened") != std::string::npos);
    CHECK(e.source().find("does_not_exist.csv") != std::string::npos);
  }
}

// ---------------------------------------------------------------------------
// Non-finite values and time ordering
// ---------------------------------------------------------------------------

TEST_CASE("nan and inf are rejected with the row and column named") {
  // The subtle one. std::from_chars parses "nan" and "inf" happily -- they are
  // valid floating-point text -- so this is a separate check, not something
  // the parse step gives for free. A NaN that slips through reappears as a NaN
  // metric several layers away, by which point nothing points back at the row.
  expectError("# dofs: 2\nt,q0,q1\n0,0.1,nan\n", [](const RolloutError& e) {
    CHECK(e.line() == 3);
    CHECK(e.detail().find("q1") != std::string::npos);
    CHECK(e.detail().find("finite") != std::string::npos);
    CHECK(e.detail().find("nan") != std::string::npos);
  });

  expectError("# dofs: 1\nt,q0\n0,inf\n", [](const RolloutError& e) {
    CHECK(e.line() == 3);
    CHECK(e.detail().find("finite") != std::string::npos);
  });

  expectError("# dofs: 1\nt,q0\n0,-infinity\n", [](const RolloutError& e) {
    CHECK(e.line() == 3);
    CHECK(e.detail().find("finite") != std::string::npos);
  });

  // Not only in q: a non-finite timestamp is caught in column 1 too.
  expectError("# dofs: 1\nt,q0\n0,1\nnan,2\n", [](const RolloutError& e) {
    CHECK(e.line() == 4);
    CHECK(e.detail().find("(t)") != std::string::npos);
    CHECK(e.detail().find("finite") != std::string::npos);
  });
}

TEST_CASE("a value that merely looks like nan is still a number") {
  // Guards the finiteness check against being implemented as a text match on
  // "nan"/"inf", which would reject legitimate values. from_chars rejects
  // these as trailing junk, so they fail as non-numeric, not as non-finite --
  // and the distinction is what the messages say.
  expectError("# dofs: 1\nt,q0\n0,nano\n", [](const RolloutError& e) {
    CHECK(e.detail().find("not a number") != std::string::npos);
  });
  expectError("# dofs: 1\nt,q0\n0,information\n", [](const RolloutError& e) {
    CHECK(e.detail().find("not a number") != std::string::npos);
  });
}

TEST_CASE("t going backwards is an error naming both timestamps") {
  // Out-of-order rows would silently reverse a segment of the trajectory, and
  // every metric would then be computed on a path the robot never took.
  expectError(
      "# dofs: 1\n"
      "t,q0\n"
      "0,1\n"
      "0.1,2\n"
      "0.05,3\n",
      [](const RolloutError& e) {
        CHECK(e.line() == 5);  // the row that goes back, not the one before it
        CHECK(e.detail().find("0.05") != std::string::npos);
        CHECK(e.detail().find("0.1") != std::string::npos);
        CHECK(e.detail().find("decrease") != std::string::npos);
      });
}

TEST_CASE("equal consecutive timestamps are allowed") {
  // A deliberate line, not an oversight. A duplicate instant is ambiguous --
  // it can be a duplicated row, but just as often it is a clock too coarse to
  // separate two samples, which real converters produce. Since no metric in
  // this library differentiates by time, an equal timestamp costs nothing,
  // while rejecting it would throw away an otherwise sound rollout.
  const Rollout r = robometrics::parseRollout(
      "# dofs: 1\n"
      "t,q0\n"
      "0,1\n"
      "0,2\n"
      "0.1,3\n",
      "demo.csv");
  REQUIRE(r.size() == 3);
  CHECK(r.t[0] == r.t[1]);
  CHECK(r.q[1](0) == doctest::Approx(2.0));
}

TEST_CASE("a single decreasing step is caught even far into a long file") {
  // The check is per-step against the previous row, so it cannot be satisfied
  // by the file being mostly in order.
  std::string text = "# dofs: 1\nt,q0\n";
  for (int i = 0; i < 200; ++i) {
    text += std::to_string(i) + "," + std::to_string(i) + "\n";
  }
  text += "150,999\n";  // jumps backwards after 200 good rows
  expectError(text.c_str(), [](const RolloutError& e) {
    CHECK(e.line() == 203);
    CHECK(e.detail().find("decrease") != std::string::npos);
  });
}
