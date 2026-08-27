// Tests for the kinematic quality metrics.
//
// The fixture throughout is planar_arm: a 2R arm with two parallel z axes and
// links of 0.3 + 0.3. It is the textbook singularity -- with the arm fully
// stretched the tip lies on the line through both joints and cannot move
// radially at all. Everything here is checked against that one pose, because
// it is the case where the right answer is known without computing anything.

#include <doctest/doctest.h>

// JacobiSVD lives in Eigen/SVD, not Eigen/Core -- the same trap as .cross() in
// test_se3.cpp, except this one fails at compile time rather than at link time.
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "robometrics/jacobian.hpp"
#include "robometrics/metrics.hpp"

namespace {

/// Quarter turn, at full double precision.
const double kHalfPi = std::acos(0.0);

std::string fixture(const char* f) { return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + f; }

}  // namespace

TEST_CASE("a stretched arm is singular, a bent one has margin") {
  // With q2 == 0 both joints push the tip along the same direction, so J_v
  // loses rank and sigma_min collapses to zero. With q2 == pi/2 the two
  // contributions are perpendicular and the margin is wide open.
  //
  // This is the test that says sigmaMinTranslation measures the right thing at
  // all. If it fails, nothing else in this file means anything.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  Eigen::VectorXd stretched(robot.numDofs());
  stretched << 0.0, 0.0;

  Eigen::VectorXd bent(robot.numDofs());
  bent << 0.0, kHalfPi;

  const double sStretched =
      robometrics::sigmaMinTranslation(robometrics::jacobian(robot, stretched));
  const double sBent = robometrics::sigmaMinTranslation(robometrics::jacobian(robot, bent));

  CHECK(sStretched < 1e-9);
  CHECK(sBent > 0.1);
}

TEST_CASE("the full 6xN Jacobian would hide the singularity") {
  // The reason sigmaMinTranslation takes topRows(3), pinned as an executable
  // fact rather than a claim in a comment.
  //
  // At the stretched pose the arm is unambiguously singular: the tip cannot
  // move radially. Yet the smallest singular value of the FULL Jacobian is
  // about 0.19 -- a number no threshold would flag. Stacking rows can only
  // raise sigma_min (||M*x||^2 == ||A*x||^2 + ||B*x||^2), and the rotational
  // rows fill in exactly the direction the translational block lost.
  //
  // If someone ever "simplifies" the implementation to use the whole matrix,
  // this test names the consequence instead of just going red.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  Eigen::VectorXd stretched(robot.numDofs());
  stretched << 0.0, 0.0;

  const Eigen::MatrixXd J = robometrics::jacobian(robot, stretched);
  const Eigen::VectorXd sFull = Eigen::JacobiSVD<Eigen::MatrixXd>(J).singularValues();

  // Translational part: genuinely zero.
  CHECK(robometrics::sigmaMinTranslation(J) < 1e-9);
  // Full matrix: comfortably nonzero, and nowhere near any sensible threshold.
  CHECK(sFull(sFull.size() - 1) > 0.15);
}

TEST_CASE("sigma_min is the shortest semi-axis of the velocity ellipsoid") {
  // The geometric claim in the header, checked directly. Sweep unit joint
  // velocities around the circle and record the slowest tip response; that
  // minimum must be sigma_min.
  //
  // Only valid because this arm has exactly two joints, so unit qdot is a
  // circle that can be swept exhaustively.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  Eigen::VectorXd bent(robot.numDofs());
  bent << 0.2, 1.0;
  const Eigen::MatrixXd Jv = robometrics::jacobian(robot, bent).topRows(3);

  double slowest = 1e30;
  const int steps = 2000;
  for (int i = 0; i < steps; ++i) {
    const double angle = 4.0 * kHalfPi * static_cast<double>(i) / steps;
    Eigen::VectorXd qdot(2);
    qdot << std::cos(angle), std::sin(angle);  // unit length by construction
    slowest = std::min(slowest, (Jv * qdot).norm());
  }

  // The sweep is discrete, so it can only overshoot the true minimum slightly.
  CHECK(slowest ==
        doctest::Approx(robometrics::sigmaMinTranslation(robometrics::jacobian(robot, bent)))
            .epsilon(1e-3));
}

TEST_CASE("a 2-DOF arm does not report a constant zero") {
  // A 2-joint arm cannot span R^3, so it is worth pinning that the metric is
  // still a live signal for it rather than a permanent zero -- that is the
  // failure mode a rank-deficient mechanism invites.
  //
  // What this does NOT test, despite the tempting story: it does not
  // distinguish s(k - 1) from s.tail(1). Eigen's singularValues() returns
  // min(rows, cols) == 2 entries for a 3 x 2 Jacobian, so those two
  // expressions read the same element and no test could tell them apart. The
  // min(rows, cols) in the implementation guards against formulations that DO
  // pad (eigenvalues of Jv * Jv^T, fixed-size storage); none of them is in use
  // here, so there is nothing to assert about it.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));
  REQUIRE(robot.numDofs() == 2);

  const Eigen::MatrixXd Jv = robometrics::jacobian(robot, Eigen::VectorXd::Zero(2)).topRows(3);
  REQUIRE(Eigen::JacobiSVD<Eigen::MatrixXd>(Jv).singularValues().size() == 2);

  Eigen::VectorXd bent(robot.numDofs());
  bent << 0.0, kHalfPi;
  CHECK(robometrics::sigmaMinTranslation(robometrics::jacobian(robot, bent)) > 0.1);
}

TEST_CASE("the profile dips to zero when passing through a singularity") {
  // A trajectory from bent, through fully stretched, back to bent. The middle
  // must collapse; the ends must not.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  std::vector<Eigen::VectorXd> traj;
  const int n = 21;
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / (n - 1);  // 0 .. 1
    Eigen::VectorXd q(2);
    q << 0.0, 2.0 * kHalfPi * std::abs(t - 0.5);
    traj.push_back(q);
  }

  const std::vector<double> profile = robometrics::sigmaMinProfile(robot, traj);

  REQUIRE(profile.size() == traj.size());
  CHECK(profile.front() > 0.1);
  CHECK(profile.back() > 0.1);
  CHECK(*std::min_element(profile.begin(), profile.end()) < 1e-6);
}

TEST_CASE("dexterityMargin returns the minimum of the profile") {
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  // All bent configurations -> a wide margin throughout.
  std::vector<Eigen::VectorXd> safe;
  for (int i = 0; i < 5; ++i) {
    Eigen::VectorXd q(2);
    q << 0.1 * i, kHalfPi;
    safe.push_back(q);
  }
  const std::optional<double> safeMargin = robometrics::dexterityMargin(robot, safe);
  REQUIRE(safeMargin.has_value());
  CHECK(*safeMargin > 0.1);

  // One singular point is enough to make the whole trajectory fragile. This is
  // the test that distinguishes the minimum from the mean: adding a single bad
  // frame to five good ones must drag the result to zero, not shift it by a
  // sixth.
  std::vector<Eigen::VectorXd> risky = safe;
  Eigen::VectorXd sing(2);
  sing << 0.0, 0.0;
  risky.push_back(sing);
  const std::optional<double> riskyMargin = robometrics::dexterityMargin(robot, risky);
  REQUIRE(riskyMargin.has_value());
  CHECK(*riskyMargin < 1e-9);
}

TEST_CASE("an empty trajectory has no margin at all") {
  // std::nullopt, not 0.0. The distinction is the whole point: 0.0 is the value
  // a FULLY SINGULAR rollout produces, so returning it for empty input would
  // make the two most opposite outcomes this metric can report indistinguishable.
  //
  // The second CHECK is the one that matters. A caller aggregating over many
  // rollouts would, under the old behaviour, have counted every empty one as a
  // catastrophic failure and dragged the aggregate to zero.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  const std::optional<double> empty = robometrics::dexterityMargin(robot, {});
  CHECK_FALSE(empty.has_value());
  CHECK(empty == std::nullopt);

  // A genuinely singular trajectory DOES have a value, and it is zero. These
  // two cases must never compare equal.
  std::vector<Eigen::VectorXd> singular;
  Eigen::VectorXd sing(2);
  sing << 0.0, 0.0;
  singular.push_back(sing);
  const std::optional<double> atSingularity = robometrics::dexterityMargin(robot, singular);
  REQUIRE(atSingularity.has_value());
  CHECK(*atSingularity < 1e-9);
  CHECK(empty != atSingularity);
}

TEST_CASE("dexterityMargin agrees with the profile it summarises") {
  // The two functions must not drift apart. Cheap to state, and it means a
  // future change to either one has to keep them consistent.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  std::vector<Eigen::VectorXd> traj;
  for (int i = 0; i < 7; ++i) {
    Eigen::VectorXd q(2);
    q << 0.2 * i, 0.3 + 0.2 * i;
    traj.push_back(q);
  }

  const std::vector<double> profile = robometrics::sigmaMinProfile(robot, traj);
  const double expected = *std::min_element(profile.begin(), profile.end());

  const std::optional<double> margin = robometrics::dexterityMargin(robot, traj);
  REQUIRE(margin.has_value());
  CHECK(*margin == doctest::Approx(expected));
}
