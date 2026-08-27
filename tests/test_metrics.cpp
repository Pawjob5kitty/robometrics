// Tests for the kinematic quality metrics.
//
// The singularity tests use planar_arm: a 2R arm with two parallel z axes and
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
#include "robometrics/kinematics.hpp"
#include "robometrics/metrics.hpp"
#include "robometrics/se3.hpp"

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

// ---------------------------------------------------------------------------
// pathEfficiency
// ---------------------------------------------------------------------------
//
// Two fixtures are involved and the difference between them is the whole
// point. A planar arm can produce only three twist components (vx, vy,
// omega_z), so rank(J) <= 3 whichever way it is built:
//
//   planar_3r  3 joints, rank 3  ->  no null space, E == 1 always
//   planar_4r  4 joints, rank 3  ->  null space of dimension 1, E < 1 possible
//
// Verified directly in the first test below, because the whole interpretation
// of every number that follows rests on it.

namespace {

/// Builds a trajectory of n samples from a callback u in [0, 1] -> q.
template <typename Fn>
std::vector<Eigen::VectorXd> sampleTrajectory(int n, Fn&& atFraction) {
  std::vector<Eigen::VectorXd> traj;
  traj.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    traj.push_back(atFraction(static_cast<double>(i) / (n - 1)));
  }
  return traj;
}

Eigen::VectorXd joints4(double a, double b, double c, double d) {
  Eigen::VectorXd q(4);
  q << a, b, c, d;
  return q;
}

}  // namespace

TEST_CASE("redundancy is about rank(J), not about joint count") {
  // The premise the rest of this section depends on. A planar arm drives only
  // three of the six twist components, so adding a third joint does NOT make
  // it redundant -- it makes it exactly determined.
  const robometrics::Robot r3 = robometrics::Robot::fromUrdfFile(fixture("planar_3r.urdf"));
  const robometrics::Robot r4 = robometrics::Robot::fromUrdfFile(fixture("planar_4r.urdf"));

  Eigen::VectorXd q3(3);
  q3 << 0.3, 0.7, -0.4;
  Eigen::VectorXd q4(4);
  q4 << 0.3, 0.7, -0.4, 0.5;

  auto rankOf = [](const Eigen::MatrixXd& J) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    svd.setThreshold(1e-9);
    return svd.rank();
  };

  CHECK(rankOf(robometrics::jacobian(r3, q3)) == 3);
  CHECK(r3.numDofs() == 3);  // rank == columns -> null space is empty

  CHECK(rankOf(robometrics::jacobian(r4, q4)) == 3);
  CHECK(r4.numDofs() == 4);  // one spare direction: the null space
}

TEST_CASE("efficiency never exceeds 1 on a redundant arm") {
  // The optimum is by definition the smallest joint motion producing the
  // observed tip motion, so the ratio cannot exceed 1. If it does, either the
  // pseudoinverse is not returning the minimum-norm solution, or the step is
  // so coarse that the linearisation has broken down.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_4r.urdf"));

  const std::vector<Eigen::VectorXd> traj = sampleTrajectory(400, [](double u) {
    return joints4(1.1 * u, 0.9 - 0.7 * u + 0.3 * std::sin(5.0 * u), -0.3 + 1.0 * u,
                   0.4 * std::cos(4.0 * u));
  });

  const std::optional<double> e = robometrics::pathEfficiency(robot, traj);
  REQUIRE(e.has_value());
  CHECK(*e > 0.0);
  CHECK(*e <= 1.0);
}

TEST_CASE("a non-redundant arm always scores exactly 1") {
  // The limitation from the header, as an executable fact. Both of these arms
  // have an empty null space, so the recorded motion IS the minimum-norm one
  // and no trajectory can score below 1 -- however clumsy the policy was.
  //
  // Read as: a 1.0 from a robot like this is a statement about the mechanism,
  // not about the policy.
  //
  // The residual deviation is discretisation, not a defect: dx is a finite
  // difference between two poses while J is the derivative at the start of the
  // step, so the two agree only to first order. The second half of this test
  // pins that down by showing the deviation SHRINKS when the same path is
  // sampled more finely. A real error in the formula would not do that.
  const robometrics::Robot r3 = robometrics::Robot::fromUrdfFile(fixture("planar_3r.urdf"));
  const robometrics::Robot r2 = robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  auto path3 = [](double u) {
    Eigen::VectorXd q(3);
    q << 0.9 * u, 0.8 - 0.6 * u, -0.2 + 0.7 * u;
    return q;
  };
  auto path2 = [](double u) {
    Eigen::VectorXd q(2);
    q << 0.7 * u, 0.9 - 0.5 * u;
    return q;
  };

  const std::optional<double> e3 = robometrics::pathEfficiency(r3, sampleTrajectory(2000, path3));
  const std::optional<double> e2 = robometrics::pathEfficiency(r2, sampleTrajectory(2000, path2));
  REQUIRE(e3.has_value());
  REQUIRE(e2.has_value());
  CHECK(*e3 == doctest::Approx(1.0).epsilon(1e-3));
  CHECK(*e2 == doctest::Approx(1.0).epsilon(1e-3));

  // Refining the sampling must move the answer closer to 1.
  const double coarse =
      std::fabs(*robometrics::pathEfficiency(r3, sampleTrajectory(250, path3)) - 1.0);
  const double fine =
      std::fabs(*robometrics::pathEfficiency(r3, sampleTrajectory(1000, path3)) - 1.0);
  CHECK(fine < coarse);
}

TEST_CASE("wasted joint motion lowers the efficiency") {
  // Two trajectories with the SAME first joint sweep. The second additionally
  // drives j2 and j3 against each other, which on a redundant arm is largely
  // null-space motion: joints turn, the tip barely notices, and the metric
  // charges for it.
  //
  // The clean run does not score 1 either, and that is worth understanding
  // rather than treating as noise: moving only j1 is not the cheapest way to
  // produce the tip motion it produces. A 4R arm could reach the same tip path
  // with less total joint travel by sharing it across joints. So even the
  // "clean" policy leaves something on the table -- which is exactly the kind
  // of thing this metric exists to notice.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_4r.urdf"));

  const std::vector<Eigen::VectorXd> clean =
      sampleTrajectory(400, [](double u) { return joints4(0.8 * u, 0.7, -0.4, 0.5); });

  const std::vector<Eigen::VectorXd> wasteful = sampleTrajectory(400, [](double u) {
    const double osc = 0.4 * std::sin(8.0 * u);
    return joints4(0.8 * u, 0.7 + osc, -0.4 - osc, 0.5);
  });

  const std::optional<double> eClean = robometrics::pathEfficiency(robot, clean);
  const std::optional<double> eWasteful = robometrics::pathEfficiency(robot, wasteful);
  REQUIRE(eClean.has_value());
  REQUIRE(eWasteful.has_value());

  CHECK(*eWasteful < *eClean);
  // Not merely lower but substantially so, so that the test cannot pass on a
  // rounding difference if the extra motion ever stopped being counted.
  CHECK(*eClean - *eWasteful > 0.05);
}

TEST_CASE("degenerate trajectories have no efficiency") {
  // Fewer than two points means there is no step to measure, and a motionless
  // trajectory makes the denominator zero. Neither has an efficiency, and 1.0
  // would be the worst possible answer for both -- it reads as "perfect".
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_4r.urdf"));

  CHECK_FALSE(robometrics::pathEfficiency(robot, {}).has_value());
  CHECK_FALSE(robometrics::pathEfficiency(robot, {joints4(0.1, 0.2, 0.3, 0.4)}).has_value());

  const std::vector<Eigen::VectorXd> still(5, joints4(0.1, 0.2, 0.3, 0.4));
  CHECK_FALSE(robometrics::pathEfficiency(robot, still).has_value());

  // Two distinct points is the smallest trajectory that does have one.
  const std::vector<Eigen::VectorXd> twoPoints{joints4(0.1, 0.2, 0.3, 0.4),
                                               joints4(0.11, 0.2, 0.3, 0.4)};
  CHECK(robometrics::pathEfficiency(robot, twoPoints).has_value());
}

TEST_CASE("the Jacobian is taken at the midpoint of the step, not at an endpoint") {
  // dx is a finite difference over the step; J is a derivative at a point.
  // Which point decides the order of the error, and the midpoint is the one
  // that makes the first-order terms cancel.
  //
  // Pinned directly rather than through a tolerance: one big step, where the
  // three candidate configurations give genuinely different answers, all three
  // computed explicitly. The value of the test is the CONTRAST -- it does not
  // re-derive the formula independently, it asserts which of three concrete
  // numbers comes out.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_4r.urdf"));

  // Chosen so that all three candidate sampling points give clearly different
  // answers (0.234 / 0.667 / 0.175). With a step where two of them happen to
  // coincide, the test would pass without discriminating anything.
  const Eigen::VectorXd q0 = joints4(-0.5, 1.1, 0.4, -0.9);
  const Eigen::VectorXd q1 = joints4(1.2, -0.9, -1.3, 1.1);
  const Eigen::VectorXd qMid = 0.5 * (q0 + q1);

  // The step cost with the Jacobian AND the frame conversion sampled at one
  // chosen configuration. Both move together; see the header for why splitting
  // them is worse than either consistent choice.
  auto costSampledAt = [&](const Eigen::VectorXd& qEval) {
    const robometrics::SE3 t0 = robometrics::forwardKinematics(robot, q0);
    const robometrics::SE3 t1 = robometrics::forwardKinematics(robot, q1);
    const robometrics::SE3 tEval = robometrics::forwardKinematics(robot, qEval);
    const robometrics::Vec6 dxLocal = robometrics::log(t0.inverse() * t1);
    const robometrics::SE3 rotOnly(tEval.rotation(), robometrics::Vec3::Zero());
    const robometrics::Vec6 dx = robometrics::adjoint(rotOnly) * dxLocal;

    const Eigen::MatrixXd J = robometrics::jacobian(robot, qEval);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd rhs = dx;
    return svd.solve(rhs).norm() / (q1 - q0).norm();
  };

  const double atStart = costSampledAt(q0);
  const double atMid = costSampledAt(qMid);
  const double atEnd = costSampledAt(q1);

  // If any two coincided the test would prove nothing, so say so.
  REQUIRE(std::fabs(atMid - atStart) > 0.01);
  REQUIRE(std::fabs(atMid - atEnd) > 0.01);

  const std::optional<double> actual = robometrics::pathEfficiency(robot, {q0, q1});
  REQUIRE(actual.has_value());
  CHECK(*actual == doctest::Approx(atMid).epsilon(1e-12));
  CHECK(*actual != doctest::Approx(atStart).epsilon(1e-6));
  CHECK(*actual != doctest::Approx(atEnd).epsilon(1e-6));
}

TEST_CASE("the midpoint rule converges faster than sampling at the step start") {
  // The claim the previous test cannot make: not merely a different number,
  // but one that improves faster as the trajectory is refined.
  //
  // planar_3r is non-redundant, so E is exactly 1 for every trajectory and any
  // deviation is purely quadrature error. Halving the step quarters that error
  // for a second-order rule and only halves it for a first-order one.
  const robometrics::Robot robot = robometrics::Robot::fromUrdfFile(fixture("planar_3r.urdf"));

  auto path = [](double u) {
    Eigen::VectorXd q(3);
    q << 0.9 * u, 0.8 - 0.6 * u, -0.2 + 0.7 * u;
    return q;
  };

  // Start-of-step sampling, implemented here so the two schemes can be
  // compared. This is what the library did before the midpoint change.
  auto efficiencyAtStart = [&](int n) {
    const std::vector<Eigen::VectorXd> traj = sampleTrajectory(n, path);
    double opt = 0.0;
    double act = 0.0;
    for (std::size_t i = 1; i < traj.size(); ++i) {
      const Eigen::VectorXd& a = traj[i - 1];
      const Eigen::VectorXd& b = traj[i];
      act += (b - a).norm();
      const robometrics::SE3 ta = robometrics::forwardKinematics(robot, a);
      const robometrics::SE3 tb = robometrics::forwardKinematics(robot, b);
      const robometrics::SE3 rotOnly(ta.rotation(), robometrics::Vec3::Zero());
      const robometrics::Vec6 dx =
          robometrics::adjoint(rotOnly) * robometrics::log(ta.inverse() * tb);
      const Eigen::MatrixXd J = robometrics::jacobian(robot, a);
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
      const Eigen::VectorXd rhs = dx;
      opt += svd.solve(rhs).norm();
    }
    return opt / act;
  };

  auto midpointError = [&](int n) {
    return std::fabs(*robometrics::pathEfficiency(robot, sampleTrajectory(n, path)) - 1.0);
  };
  auto startError = [&](int n) { return std::fabs(efficiencyAtStart(n) - 1.0); };

  const double midCoarse = midpointError(100);
  const double midFine = midpointError(200);
  const double startCoarse = startError(100);
  const double startFine = startError(200);

  // Both schemes converge at all.
  CHECK(midFine < midCoarse);
  CHECK(startFine < startCoarse);

  // First order halves the error, second order quarters it. Checked with room
  // on both sides: this is about the ORDER, not about hitting 2.00 and 4.00.
  const double startRatio = startCoarse / startFine;
  const double midRatio = midCoarse / midFine;
  CHECK(startRatio > 1.7);
  CHECK(startRatio < 2.5);
  CHECK(midRatio > 3.4);

  // And it is not merely faster-converging but far more accurate at the same
  // sampling: orders of magnitude, not percent.
  CHECK(midCoarse < startCoarse / 100.0);
}
