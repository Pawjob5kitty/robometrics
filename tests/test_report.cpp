// Tests for analyze(), mostly for span localisation -- the part where a
// one-off index error is invisible in the aggregate numbers. Most of what
// follows is about boundaries: a span starting at the first step, one still
// open at the last, one exactly at the threshold.
//
// Profile shapes come from synthetic rollouts on planar_arm, whose singularity
// sits at a known configuration, so assertions about span arithmetic stay
// disentangled from assertions about kinematics.

#include <doctest/doctest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "robometrics/report.hpp"

namespace {

using robometrics::analyze;
using robometrics::Robot;
using robometrics::Rollout;
using robometrics::RolloutReport;

std::string fixture(const char* f) { return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + f; }

const double kHalfPi = std::acos(0.0);

/// Builds a 2-DOF rollout on planar_arm from a per-step callback.
///
/// planar_arm is singular when q1 == 0 (the arm fully stretched), so a
/// callback that drives q1 through zero produces a profile with a known dip in
/// a known place.
template <typename Fn>
Rollout makeRollout(int steps, Fn&& qAt) {
  Rollout r;
  r.dofs = 2;
  for (int i = 0; i < steps; ++i) {
    Eigen::VectorXd q(2);
    q << 0.0, qAt(i);
    r.t.push_back(0.05 * i);
    r.q.push_back(q);
  }
  return r;
}

Robot planarArm() { return Robot::fromUrdfFile(fixture("planar_arm.urdf")); }

}  // namespace

// ---------------------------------------------------------------------------
// The basic shape of a report
// ---------------------------------------------------------------------------

TEST_CASE("a clean rollout has a profile, a margin, and no spans") {
  const Robot robot = planarArm();
  // q1 stays near pi/2 throughout: well away from the stretched singularity.
  const Rollout r = makeRollout(30, [](int i) { return kHalfPi + 0.01 * i; });

  const RolloutReport report = analyze(robot, r);

  CHECK(report.dexterityProfile.size() == r.size());
  REQUIRE(report.dexterityMargin.has_value());
  CHECK(*report.dexterityMargin > 0.1);
  REQUIRE(report.worstIndex.has_value());
  CHECK(*report.worstIndex < r.size());
  CHECK(report.lowDexteritySpans.empty());
}

TEST_CASE("the margin and the worst index describe the same step") {
  // Two ways of reading the same profile must not disagree. They are computed
  // from one min_element precisely so they cannot, and this pins that.
  const Robot robot = planarArm();
  const Rollout r = makeRollout(40, [](int i) { return 1.4 - 0.035 * i; });

  const RolloutReport report = analyze(robot, r);
  REQUIRE(report.dexterityMargin.has_value());
  REQUIRE(report.worstIndex.has_value());
  CHECK(report.dexterityProfile[*report.worstIndex] == *report.dexterityMargin);
}

TEST_CASE("an empty rollout yields empty everything, not zeros") {
  Rollout empty;
  empty.dofs = 2;

  const RolloutReport report = analyze(planarArm(), empty);
  CHECK(report.dexterityProfile.empty());
  CHECK_FALSE(report.dexterityMargin.has_value());
  CHECK_FALSE(report.worstIndex.has_value());
  CHECK_FALSE(report.pathEfficiency.has_value());
  CHECK(report.lowDexteritySpans.empty());
}

TEST_CASE("a rollout whose width disagrees with the robot is rejected") {
  // Pairing the wrong URDF with a rollout would otherwise produce a complete
  // set of plausible numbers describing a robot that never ran.
  Rollout r;
  r.dofs = 7;
  r.t.push_back(0.0);
  Eigen::VectorXd q(7);
  q.setZero();
  r.q.push_back(q);

  CHECK_THROWS_AS(analyze(planarArm(), r), std::invalid_argument);
  try {
    analyze(planarArm(), r);
  } catch (const std::invalid_argument& e) {
    const std::string message = e.what();
    CHECK(message.find('7') != std::string::npos);
    CHECK(message.find('2') != std::string::npos);
    CHECK(message.find("planar_arm") != std::string::npos);
  }
}

// ---------------------------------------------------------------------------
// Span localisation
// ---------------------------------------------------------------------------

TEST_CASE("passing through a singularity produces exactly one span, in the right place") {
  // The headline case. q1 sweeps from +1.2 down through 0 (stretched, singular)
  // and back to +1.2, so the profile has one dip in the middle.
  const Robot robot = planarArm();
  const int steps = 41;
  const Rollout r = makeRollout(steps, [steps](int i) {
    const double u = static_cast<double>(i) / (steps - 1);  // 0 .. 1
    return 1.2 * std::fabs(2.0 * u - 1.0);                  // 1.2 -> 0 -> 1.2
  });

  const RolloutReport report = analyze(robot, r, 0.05);

  REQUIRE(report.lowDexteritySpans.size() == 1);
  const auto& span = report.lowDexteritySpans[0];

  // The dip is centred on the middle step, and the span must bracket it.
  CHECK(span.begin < steps / 2u);
  CHECK(span.end > steps / 2u);
  CHECK(span.length() > 0);

  // The span's worst value is the minimum of the profile INSIDE it, and here
  // that is also the global minimum.
  CHECK(span.worst < 0.05);
  REQUIRE(report.dexterityMargin.has_value());
  CHECK(span.worst == *report.dexterityMargin);
  REQUIRE(report.worstIndex.has_value());
  CHECK(*report.worstIndex >= span.begin);
  CHECK(*report.worstIndex < span.end);
}

TEST_CASE("span bounds are half-open and match the profile exactly") {
  // The off-by-one test. Every step inside [begin, end) must be below the
  // threshold, and the steps immediately outside must not be. Stated as a
  // property of the profile rather than as expected indices, so it stays true
  // if the fixture geometry is ever tweaked.
  const Robot robot = planarArm();
  const int steps = 41;
  const Rollout r = makeRollout(steps, [steps](int i) {
    const double u = static_cast<double>(i) / (steps - 1);
    return 1.2 * std::fabs(2.0 * u - 1.0);
  });
  const double threshold = 0.05;

  const RolloutReport report = analyze(robot, r, threshold);
  REQUIRE(report.lowDexteritySpans.size() == 1);
  const auto& span = report.lowDexteritySpans[0];

  for (std::size_t i = span.begin; i < span.end; ++i) {
    CHECK(report.dexterityProfile[i] < threshold);
  }
  if (span.begin > 0) {
    CHECK(report.dexterityProfile[span.begin - 1] >= threshold);
  }
  if (span.end < report.dexterityProfile.size()) {
    CHECK(report.dexterityProfile[span.end] >= threshold);
  }
}

TEST_CASE("two separate dips produce two spans, not one") {
  // Guards against a scan that never closes a span, or one that merges runs
  // separated by good steps.
  const Robot robot = planarArm();
  const int steps = 81;
  const Rollout r = makeRollout(steps, [steps](int i) {
    // Two V-shaped dips down to zero, at u = 0.25 and u = 0.75.
    const double u = static_cast<double>(i) / (steps - 1);
    const double d = std::min(std::fabs(u - 0.25), std::fabs(u - 0.75));
    return 4.8 * d;  // 0 at each dip, comfortably large between them
  });

  const RolloutReport report = analyze(robot, r, 0.05);
  REQUIRE(report.lowDexteritySpans.size() == 2);

  const auto& first = report.lowDexteritySpans[0];
  const auto& second = report.lowDexteritySpans[1];
  CHECK(first.end < second.begin);  // disjoint AND non-adjacent
  CHECK(first.begin < first.end);
  CHECK(second.begin < second.end);
}

TEST_CASE("a span still open at the last step is closed at the end") {
  // The classic omission: a run that never sees a good step after it. This is
  // not an exotic case -- it is what a rollout that failed at the last moment
  // looks like, which is the most interesting rollout in the batch.
  const Robot robot = planarArm();
  const int steps = 20;
  const Rollout r = makeRollout(steps, [steps](int i) {
    // Starts fine, ends stretched and stays there.
    const double u = static_cast<double>(i) / (steps - 1);
    return std::max(0.0, 1.2 * (1.0 - 2.0 * u));
  });

  const RolloutReport report = analyze(robot, r, 0.05);
  REQUIRE(report.lowDexteritySpans.size() == 1);
  CHECK(report.lowDexteritySpans[0].end == report.dexterityProfile.size());
  CHECK(report.lowDexteritySpans[0].begin > 0);
}

TEST_CASE("a span open from the very first step starts at zero") {
  // The mirror case, and a different code path: the span opens on the first
  // iteration rather than on a transition.
  const Robot robot = planarArm();
  const int steps = 20;
  const Rollout r = makeRollout(steps, [steps](int i) {
    const double u = static_cast<double>(i) / (steps - 1);
    return 1.2 * u;  // starts stretched, opens up
  });

  const RolloutReport report = analyze(robot, r, 0.05);
  REQUIRE(report.lowDexteritySpans.size() == 1);
  CHECK(report.lowDexteritySpans[0].begin == 0);
  CHECK(report.lowDexteritySpans[0].end < report.dexterityProfile.size());
}

TEST_CASE("an entirely singular rollout is one span covering everything") {
  const Robot robot = planarArm();
  const Rollout r = makeRollout(12, [](int) { return 0.0; });  // always stretched

  const RolloutReport report = analyze(robot, r, 0.05);
  REQUIRE(report.lowDexteritySpans.size() == 1);
  CHECK(report.lowDexteritySpans[0].begin == 0);
  CHECK(report.lowDexteritySpans[0].end == r.size());
  CHECK(report.lowDexteritySpans[0].length() == r.size());
}

TEST_CASE("the threshold comparison is strict") {
  // The boundary case the spec asks for. A step sitting EXACTLY on the
  // threshold is acceptable -- the threshold is the last good value, not the
  // first bad one. Invisible on real data, glaring here.
  //
  // Rather than hunting for a configuration whose sigma_min is exactly some
  // round number, the profile is measured first and the threshold is then set
  // to a value the profile actually attains. That makes the equality exact
  // rather than approximate.
  const Robot robot = planarArm();
  const Rollout r = makeRollout(15, [](int i) { return 0.2 + 0.05 * i; });

  const RolloutReport measured = analyze(robot, r, 0.0);
  REQUIRE(measured.dexterityProfile.size() == r.size());
  const double exactValue = measured.dexterityProfile[7];

  // Threshold exactly equal to one profile value: that step is NOT low.
  const RolloutReport atValue = analyze(robot, r, exactValue);
  CHECK(std::none_of(atValue.lowDexteritySpans.begin(), atValue.lowDexteritySpans.end(),
                     [](const RolloutReport::Span& s) { return s.begin <= 7 && 7 < s.end; }));

  // Nudge the threshold just above that value and the step becomes low.
  const RolloutReport justAbove = analyze(robot, r, std::nextafter(exactValue, 1e9));
  CHECK(std::any_of(justAbove.lowDexteritySpans.begin(), justAbove.lowDexteritySpans.end(),
                    [](const RolloutReport::Span& s) { return s.begin <= 7 && 7 < s.end; }));
}

TEST_CASE("a higher threshold can only grow the flagged region") {
  // Monotonicity in the threshold. Cheap, and it catches a scan whose
  // comparison drifted in a way the single-threshold tests happen to miss.
  const Robot robot = planarArm();
  const int steps = 41;
  const Rollout r = makeRollout(steps, [steps](int i) {
    const double u = static_cast<double>(i) / (steps - 1);
    return 1.2 * std::fabs(2.0 * u - 1.0);
  });

  auto flaggedCount = [&](double threshold) {
    std::size_t total = 0;
    for (const auto& span : analyze(robot, r, threshold).lowDexteritySpans) {
      total += span.length();
    }
    return total;
  };

  CHECK(flaggedCount(0.0) == 0);
  CHECK(flaggedCount(0.02) <= flaggedCount(0.05));
  CHECK(flaggedCount(0.05) <= flaggedCount(0.2));
  CHECK(flaggedCount(1e9) == r.size());
}

TEST_CASE("a rollout that recovers on the very last step ends its span there") {
  // The shape that catches a scan loop stopping one element short. Every other
  // test here survives that, because the "close whatever is still open" step
  // uses profile.size() rather than the loop index, so a span running to the
  // end still comes out with the right bounds.
  //
  // Only this shape shows it: bad to the second-to-last step, GOOD on the last.
  // The correct span stops before that final step; a scan that never looks at
  // it leaves the span open and swallows a good step into the flagged region.
  const Robot robot = planarArm();
  const int steps = 12;
  const Rollout r = makeRollout(steps, [steps](int i) {
    return (i == steps - 1) ? 1.2 : 0.0;  // stretched throughout, recovers at the end
  });
  const double threshold = 0.05;

  const RolloutReport report = analyze(robot, r, threshold);
  REQUIRE(report.dexterityProfile.size() == static_cast<std::size_t>(steps));
  // Premise: the last step really is good and the one before it really is not.
  REQUIRE(report.dexterityProfile.back() >= threshold);
  REQUIRE(report.dexterityProfile[static_cast<std::size_t>(steps) - 2] < threshold);

  REQUIRE(report.lowDexteritySpans.size() == 1);
  CHECK(report.lowDexteritySpans[0].begin == 0);
  CHECK(report.lowDexteritySpans[0].end == static_cast<std::size_t>(steps) - 1);
  CHECK(report.lowDexteritySpans[0].end < report.dexterityProfile.size());
}

TEST_CASE("every flagged step is below the threshold and every other one is not") {
  // The complete statement, as a sweep over the whole profile rather than over
  // span boundaries: membership in a span must agree with the comparison at
  // every single index. Any index arithmetic that is off anywhere fails here.
  const Robot robot = planarArm();
  const int steps = 61;
  const Rollout r = makeRollout(steps, [steps](int i) {
    const double u = static_cast<double>(i) / (steps - 1);
    const double d = std::min(std::fabs(u - 0.3), std::fabs(u - 0.8));
    return 4.0 * d;
  });
  const double threshold = 0.05;

  const RolloutReport report = analyze(robot, r, threshold);
  REQUIRE(report.lowDexteritySpans.size() >= 2);

  for (std::size_t i = 0; i < report.dexterityProfile.size(); ++i) {
    const bool inSomeSpan =
        std::any_of(report.lowDexteritySpans.begin(), report.lowDexteritySpans.end(),
                    [i](const RolloutReport::Span& s) { return s.begin <= i && i < s.end; });
    CHECK(inSomeSpan == (report.dexterityProfile[i] < threshold));
  }
}
