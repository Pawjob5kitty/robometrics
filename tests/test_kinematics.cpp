// Tests for forward kinematics.
//
// The structure mirrors the SE(3) tests: first hand-computed cases (which say
// whether the formula is right), then invariants (which catch regressions that
// specific numbers would miss).

#include <doctest/doctest.h>

#include <Eigen/Core>
#include <cmath>
#include <string>
#include <vector>

#include "robometrics/kinematics.hpp"
#include "robometrics/urdf.hpp"

namespace {

using robometrics::forwardKinematics;
using robometrics::forwardKinematicsAll;
using robometrics::Mat3;
using robometrics::Robot;
using robometrics::SE3;
using robometrics::Vec3;

/// Looser than kDefaultTol, for the same reason as in test_urdf.cpp: rotations
/// go through rodrigues(), so cos(pi/2) comes out at 6.1e-17 instead of zero.
/// The error is of order 1e-16, so 1e-12 has four orders of headroom.
constexpr double kTol = 1e-12;

const double kHalfPi = std::acos(0.0);

std::string fixture(const char* fileName) {
  return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + fileName;
}

Eigen::VectorXd zeros(int n) { return Eigen::VectorXd::Zero(n); }

}  // namespace

// ---------------------------------------------------------------------------
// q = 0
// ---------------------------------------------------------------------------

TEST_CASE("at q = 0 the result is the composition of the originTransforms") {
  // At zero configuration motion(0) is identity, so jointTransform leaves just
  // the bare originTransform. The result MUST therefore be their product along
  // the chain -- which can be computed independently, without FK.
  //
  // If the product order were confused anywhere (local * parent instead of
  // parent * local), this test catches it at the first joint with a nonzero
  // origin, because translations compose through the parent's rotation.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  const SE3 actual = forwardKinematics(robot, zeros(robot.numDofs()));

  // By hand: joint1 (0,0,0.1) * joint2 (0.5,0,0) * tool offset (0,0,0.05).
  // All rotations are identity, so the translations simply add up.
  SE3 expected = SE3::identity();
  for (int i = 0; i < robot.numJoints(); ++i) {
    expected = expected * robot.joint(i).originTransform;
  }
  expected = expected * robot.link(robot.tipLinkIndex()).offset;

  CHECK(actual.isApprox(expected, kTol));
  CHECK(actual.translation().isApprox(Vec3(0.5, 0.0, 0.15)));
  CHECK(actual.rotation().isApprox(Mat3::Identity()));
}

TEST_CASE("at q = 0 the root is at identity") {
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  const std::vector<SE3> all = forwardKinematicsAll(robot, zeros(robot.numDofs()));
  CHECK(all[static_cast<std::size_t>(robot.rootLinkIndex())].isApprox(SE3::identity(), kTol));
}

TEST_CASE("the root is at identity for any q") {
  // A stronger claim than the previous test: the root must not move even when
  // the joints do. If FK accidentally applied the first joint to the base too,
  // this fails, while the q = 0 test would pass.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  Eigen::VectorXd q(2);
  q << 0.7, -1.3;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);
  CHECK(all[static_cast<std::size_t>(robot.rootLinkIndex())].isApprox(SE3::identity(), kTol));
}

// ---------------------------------------------------------------------------
// Single-joint robot against a hand computation
// ---------------------------------------------------------------------------

TEST_CASE("one joint, rotation about z by pi/2") {
  // By hand. The joint sits at (0, 0, 0.1) and rotates about z. Beyond it is a
  // fixed 0.5 arm (0.5, 0, 0) to link 'tip'.
  //
  //   q = 0:      tip = (0.5, 0, 0.1),  R = I
  //
  //   q = pi/2:   rotation about z sends x to y, so the arm (0.5, 0, 0) maps to
  //               (0, 0.5, 0). Add the joint height:
  //               tip = (0, 0.5, 0.1)
  //
  //               R = Rz(pi/2) = | 0  -1   0 |
  //                              | 1   0   0 |
  //                              | 0   0   1 |
  //
  // The height 0.1 must NOT rotate -- the joint is above the base, not behind
  // the rotation. With the product order swapped the tip would come out at
  // z = 0 and y = 0.6 or similar.
  const Robot robot = Robot::fromUrdfFile(fixture("single_joint.urdf"));
  REQUIRE(robot.numDofs() == 1);

  Eigen::VectorXd q(1);

  q << 0.0;
  const SE3 atZero = forwardKinematics(robot, q);
  CHECK(atZero.translation().isApprox(Vec3(0.5, 0.0, 0.1)));
  CHECK(atZero.rotation().isApprox(Mat3::Identity()));

  q << kHalfPi;
  const SE3 atQuarter = forwardKinematics(robot, q);
  CHECK((atQuarter.translation() - Vec3(0.0, 0.5, 0.1)).cwiseAbs().maxCoeff() < kTol);

  Mat3 expectedR;
  // clang-format off
  expectedR << 0.0, -1.0, 0.0,
               1.0,  0.0, 0.0,
               0.0,  0.0, 1.0;
  // clang-format on
  CHECK((atQuarter.rotation() - expectedR).cwiseAbs().maxCoeff() < kTol);
}

TEST_CASE("rotation by pi puts the arm on the opposite side") {
  // A second hand case, now without the matrix: at q = pi the arm must be
  // exactly mirrored about the axis. Catches a sign error in the angle that
  // pi/2 could in principle miss, if the axis flipped along with the sign.
  const Robot robot = Robot::fromUrdfFile(fixture("single_joint.urdf"));
  Eigen::VectorXd q(1);
  q << 2.0 * kHalfPi;
  const SE3 T = forwardKinematics(robot, q);
  CHECK((T.translation() - Vec3(-0.5, 0.0, 0.1)).cwiseAbs().maxCoeff() < kTol);
}

TEST_CASE("the sign of the angle follows the right-hand rule") {
  // A positive q about z must send the arm from +x toward +y, not -y. This is
  // the only test that distinguishes rodrigues(w) from rodrigues(-w) -- every
  // invariant below holds for both.
  const Robot robot = Robot::fromUrdfFile(fixture("single_joint.urdf"));
  Eigen::VectorXd q(1);
  q << 0.3;
  const SE3 T = forwardKinematics(robot, q);
  CHECK(T.translation().y() > 0.0);
  CHECK(T.translation().x() > 0.0);  // still closer to +x than to +y
}

// ---------------------------------------------------------------------------
// Two-joint chain at nonzero q -- product order
// ---------------------------------------------------------------------------
//
// This section was added after a mutation check. The original tests did NOT
// catch two basic order swaps:
//
//   (1) motion * originTransform instead of originTransform * motion
//   (2) local * parent instead of parent * local when composing the chain
//
// The reason was the same each time: the only test with an absolute pose at
// nonzero q was the single-joint robot with a z axis and origin (0, 0, 0.1).
// Rotation about z leaves the vector (0, 0, 0.1) in place, so both swapped
// versions gave the same thing. The remaining nonzero-q tests were either
// relative (the mutual pose of two links) or self-consistent (both sides went
// through the same swap).
//
// Lesson for the future: when verifying product order, the test case must have
// a translation NOT parallel to the axis of rotation. Otherwise the transforms
// commute and the test distinguishes nothing.

TEST_CASE("two joints at nonzero q, computed by hand") {
  // two_joint.urdf:
  //   joint1: origin (0, 0, 0.1), axis z
  //   joint2: origin (0.3, 0, 0), axis y      <- translation perpendicular to z
  //   tip = link2, no fixed offset
  //
  // Case q = (0, pi/2). By hand:
  //
  //   local1 = trans(0,0,0.1) * Rz(0)     = SE3(I,  (0, 0, 0.1))
  //   local2 = trans(0.3,0,0) * Ry(pi/2)  = SE3(Ry, (0.3, 0, 0))
  //
  //   X2 = local1 * local2 = SE3(Ry, I*(0.3,0,0) + (0,0,0.1))
  //                        = SE3(Ry, (0.3, 0, 0.1))
  //
  // If motion were applied on the left (error 1), local2 would equal
  // SE3(Ry, Ry*(0.3,0,0)) = SE3(Ry, (0, 0, -0.3)) and the tip would land at
  // (0, 0, -0.2).
  //
  // If the chain composed backwards (error 2), the result would be
  // local2 * local1 = SE3(Ry, Ry*(0,0,0.1) + (0.3,0,0)) = SE3(Ry, (0.4, 0, 0)).
  const Robot robot = Robot::fromUrdfFile(fixture("two_joint.urdf"));
  REQUIRE(robot.numDofs() == 2);

  Eigen::VectorXd q(2);
  q << 0.0, kHalfPi;
  const SE3 T = forwardKinematics(robot, q);

  CHECK((T.translation() - Vec3(0.3, 0.0, 0.1)).cwiseAbs().maxCoeff() < kTol);

  Mat3 expectedR;  // Ry(pi/2)
  // clang-format off
  expectedR <<  0.0, 0.0, 1.0,
                0.0, 1.0, 0.0,
               -1.0, 0.0, 0.0;
  // clang-format on
  CHECK((T.rotation() - expectedR).cwiseAbs().maxCoeff() < kTol);

  // And explicitly that it is not the two wrong variants -- if someone swaps the
  // order, this line names what happened.
  CHECK((T.translation() - Vec3(0.0, 0.0, -0.2)).cwiseAbs().maxCoeff() > 0.1);
  CHECK((T.translation() - Vec3(0.4, 0.0, 0.0)).cwiseAbs().maxCoeff() > 0.1);
}

TEST_CASE("two joints, both nonzero, computed by hand") {
  // The general case q = (pi/2, pi/2), where nothing commutes.
  //
  //   local1 = trans(0,0,0.1) * Rz(pi/2) = SE3(Rz, (0, 0, 0.1))
  //   local2 = trans(0.3,0,0) * Ry(pi/2) = SE3(Ry, (0.3, 0, 0))
  //
  //   translation: R1 * t2 + t1 = Rz(pi/2)*(0.3,0,0) + (0,0,0.1)
  //                             = (0, 0.3, 0) + (0, 0, 0.1)
  //                             = (0, 0.3, 0.1)
  //
  //   rotation:    Rz(pi/2) * Ry(pi/2)
  //     Rz = |0 -1  0|      Ry = | 0  0  1|
  //          |1  0  0|           | 0  1  0|
  //          |0  0  1|           |-1  0  0|
  //
  //     row 0 Rz = (0,-1,0) times cols of Ry (0,0,-1),(0,1,0),(1,0,0)
  //                                      -> ( 0, -1,  0)
  //     row 1 Rz = (1, 0,0)              -> ( 0,  0,  1)
  //     row 2 Rz = (0, 0,1)              -> (-1,  0,  0)
  const Robot robot = Robot::fromUrdfFile(fixture("two_joint.urdf"));

  Eigen::VectorXd q(2);
  q << kHalfPi, kHalfPi;
  const SE3 T = forwardKinematics(robot, q);

  CHECK((T.translation() - Vec3(0.0, 0.3, 0.1)).cwiseAbs().maxCoeff() < kTol);

  Mat3 expectedR;
  // clang-format off
  expectedR <<  0.0, -1.0, 0.0,
                0.0,  0.0, 1.0,
               -1.0,  0.0, 0.0;
  // clang-format on
  CHECK((T.rotation() - expectedR).cwiseAbs().maxCoeff() < kTol);
}

TEST_CASE("jointTransform applies motion from the right, not the left") {
  // The smallest possible test of that one decision, isolated from the rest of
  // FK. The axis is z and the origin's translation is along x -- PERPENDICULAR
  // to the axis, otherwise both variants would commute and the test would
  // distinguish nothing.
  //
  //   origin = trans(0.2, 0, 0),  axis z,  q = pi/2
  //
  //   correct  origin * motion = SE3(Rz, (0.2, 0, 0))
  //            the translation does not rotate, because the rotation comes after
  //
  //   wrong    motion * origin = SE3(Rz, Rz*(0.2,0,0)) = SE3(Rz, (0, 0.2, 0))
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"a\"/>"
      "    <origin xyz=\"0.2 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-3.2\" upper=\"3.2\" effort=\"1\" velocity=\"1\"/></joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);
  const SE3 T = robometrics::jointTransform(robot.joint(0), kHalfPi);

  CHECK((T.translation() - Vec3(0.2, 0.0, 0.0)).cwiseAbs().maxCoeff() < kTol);
  CHECK((T.translation() - Vec3(0.0, 0.2, 0.0)).cwiseAbs().maxCoeff() > 0.1);
}

// ---------------------------------------------------------------------------
// Relationship of forwardKinematics and forwardKinematicsAll
// ---------------------------------------------------------------------------

TEST_CASE("forwardKinematicsAll at the tip index equals forwardKinematics") {
  // Index through tipLinkIndex(), not .back(): on a tree with a gripper the last
  // link in the array is a random finger, not the end-effector. Link order is a
  // matter of the traversal; the tip is a matter of the request.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 0.4, -0.9;

  const std::vector<SE3> all = forwardKinematicsAll(robot, q);
  const SE3 tip = forwardKinematics(robot, q);

  REQUIRE(all.size() == static_cast<std::size_t>(robot.numLinks()));
  CHECK(all[static_cast<std::size_t>(robot.tipLinkIndex())].isApprox(tip, kTol));
}

TEST_CASE("a joint frame is the pose of its child link") {
  // The bridge to the Jacobian: it iterates over joints, not links. This checks
  // that the conversion between them really is just indexing through childLink,
  // as kinematics.hpp promises.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 0.2, 0.5;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);

  // Build the first joint's pose by hand and compare with its child link's pose.
  const robometrics::Joint& j0 = robot.joint(0);
  const SE3 expected = robometrics::jointTransform(j0, q(j0.dofIndex));
  CHECK(all[static_cast<std::size_t>(j0.childLink)].isApprox(expected, kTol));
}

// ---------------------------------------------------------------------------
// Fixed joints in FK
// ---------------------------------------------------------------------------

TEST_CASE("a trailing fixed joint shows up in the tip pose") {
  // The parser stores it as Link::offset; this test checks FK actually uses it.
  // Without it the tip would end up at link 'link2', 0.05 lower -- exactly the
  // error that Link's own offset exists to prevent.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 0.0, 0.0;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);

  const SE3 link2 = all[static_cast<std::size_t>(robot.findLink("link2"))];
  const SE3 tool = all[static_cast<std::size_t>(robot.findLink("tool"))];

  // The difference between them is exactly the fixed origin 0.05 along z of link2.
  const SE3 relative = link2.inverse() * tool;
  CHECK(relative.translation().isApprox(Vec3(0.0, 0.0, 0.05)));
  CHECK(relative.rotation().isApprox(Mat3::Identity()));
}

TEST_CASE("a fixed offset is rigid relative to its joint for any q") {
  // A stronger version of the previous: the mutual pose of link2 and tool must
  // not depend on q at all. If the fixed offset were applied on the wrong side
  // of the product, q = 0 would still come out right and this would not.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 1.1, -0.6;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);

  const SE3 relative = all[static_cast<std::size_t>(robot.findLink("link2"))].inverse() *
                       all[static_cast<std::size_t>(robot.findLink("tool"))];
  CHECK(relative.isApprox(SE3(Mat3::Identity(), Vec3(0.0, 0.0, 0.05)), kTol));
}

// ---------------------------------------------------------------------------
// Prismatic and mimic joints
// ---------------------------------------------------------------------------

TEST_CASE("a prismatic joint translates, the mimic moves against it") {
  // Hand computation for the gripper:
  //
  //   arm q = 0        => hand at (0, 0, 0.4)
  //   finger_left q = 0.03, origin (0, 0.02, 0.05), axis y
  //                    => left  = (0, 0.4) + (0, 0.02, 0.05) + (0, 0.03, 0)
  //                             = (0, 0.05, 0.45)
  //   finger_right mimic, multiplier -1 => value -0.03
  //                       origin (0, -0.02, 0.05), axis y
  //                    => right = (0, -0.02, 0.05) + (0, -0.03, 0), offset
  //                             = (0, -0.05, 0.45)
  //
  // The fingers come out symmetric about the axis. If the multiplier were
  // ignored, right would end at (0, 0.01, 0.45) and the symmetry would vanish.
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "hand");
  REQUIRE(robot.numDofs() == 2);
  REQUIRE(robot.numJoints() == 3);

  Eigen::VectorXd q(2);
  q << 0.0, 0.03;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);

  const Vec3 left = all[static_cast<std::size_t>(robot.findLink("left"))].translation();
  const Vec3 right = all[static_cast<std::size_t>(robot.findLink("right"))].translation();

  CHECK((left - Vec3(0.0, 0.05, 0.45)).cwiseAbs().maxCoeff() < kTol);
  CHECK((right - Vec3(0.0, -0.05, 0.45)).cwiseAbs().maxCoeff() < kTol);

  // Symmetry checked separately, because it says exactly what the mimic multiplier means.
  CHECK(left.y() == doctest::Approx(-right.y()));
}

TEST_CASE("the mimic offset is applied") {
  // The multiplier alone would pass even if the offset were discarded, because
  // it is zero in the fixture. This case has it nonzero.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/><link name=\"b\"/>"
      "  <joint name=\"driver\" type=\"prismatic\">"
      "    <parent link=\"base\"/><child link=\"a\"/><axis xyz=\"1 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>"
      "  <joint name=\"follower\" type=\"prismatic\">"
      "    <parent link=\"a\"/><child link=\"b\"/><axis xyz=\"1 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "    <mimic joint=\"driver\" multiplier=\"2\" offset=\"0.5\"/></joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);
  REQUIRE(robot.numDofs() == 1);

  Eigen::VectorXd q(1);
  q << 0.1;
  // driver   = 0.1
  // follower = 2 * 0.1 + 0.5 = 0.7
  // total along x: 0.1 + 0.7 = 0.8
  const SE3 tip = forwardKinematics(robot, q);
  CHECK(tip.translation().x() == doctest::Approx(0.8));
}

TEST_CASE("a mimic joint moves even when its q is zero") {
  // A consequence of a nonzero offset: at q = 0 the follower is not at zero but
  // at 0.5. The test exists because "everything at zero" is the most common
  // assumption, and it does not hold for a mimic joint with an offset.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/><link name=\"b\"/>"
      "  <joint name=\"driver\" type=\"prismatic\">"
      "    <parent link=\"base\"/><child link=\"a\"/><axis xyz=\"1 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>"
      "  <joint name=\"follower\" type=\"prismatic\">"
      "    <parent link=\"a\"/><child link=\"b\"/><axis xyz=\"1 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "    <mimic joint=\"driver\" multiplier=\"1\" offset=\"0.5\"/></joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);
  const SE3 tip = forwardKinematics(robot, zeros(1));
  CHECK(tip.translation().x() == doctest::Approx(0.5));
}

// ---------------------------------------------------------------------------
// Invariants
// ---------------------------------------------------------------------------

TEST_CASE("every resulting pose is a valid SE(3) transform") {
  // A structural check. Catches errors the specific numbers would miss -- for
  // instance a stray scale or reflection; the result would still be "somewhere",
  // just not a rigid transform.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 2.4, -1.7;
  for (const SE3& T : forwardKinematicsAll(robot, q)) {
    const Mat3 r = T.rotation();
    CHECK((r.transpose() * r - Mat3::Identity()).cwiseAbs().maxCoeff() < kTol);
  }
}

TEST_CASE("turning one joint out and back returns the original pose") {
  // Says FK is a function of q and remembers nothing between calls. Trivially
  // true for a pure function -- which is exactly why it is a good test for
  // whether state crept in (a cache, a static, a mutated Robot).
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd a(2);
  a << 0.3, 1.2;
  Eigen::VectorXd b(2);
  b << -2.0, 0.4;

  const SE3 first = forwardKinematics(robot, a);
  forwardKinematics(robot, b);
  const SE3 again = forwardKinematics(robot, a);
  CHECK(first.isApprox(again, kTol));
}

TEST_CASE("a continuous joint accepts an angle outside (-pi, pi)") {
  // A continuous joint has no limits, so q = 7 rad is a legitimate input. The
  // result must match that for 7 - 2*pi -- not because we wrap the angle, but
  // because a rotation by 2*pi is the identity.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/>"
      "  <joint name=\"wheel\" type=\"continuous\">"
      "    <parent link=\"base\"/><child link=\"a\"/>"
      "    <origin xyz=\"0.2 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/></joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);

  Eigen::VectorXd big(1);
  big << 7.0;
  Eigen::VectorXd wrapped(1);
  wrapped << 7.0 - 4.0 * kHalfPi;

  // Looser tolerance: 7 - 2*pi subtracts nearly equal numbers, so the angle
  // itself carries an error of order 1e-16 that passes straight into the
  // rotation.
  CHECK(forwardKinematics(robot, big).isApprox(forwardKinematics(robot, wrapped), 1e-14));
}

// ---------------------------------------------------------------------------
// Error cases
// ---------------------------------------------------------------------------

TEST_CASE("a wrong-size q is an exception") {
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  CHECK_THROWS_AS(forwardKinematics(robot, zeros(1)), std::invalid_argument);
  CHECK_THROWS_AS(forwardKinematics(robot, zeros(3)), std::invalid_argument);
  CHECK_THROWS_AS(forwardKinematicsAll(robot, zeros(0)), std::invalid_argument);
  CHECK_NOTHROW(forwardKinematics(robot, zeros(2)));
}

TEST_CASE("the wrong-size q message states both numbers") {
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  try {
    forwardKinematics(robot, zeros(5));
    FAIL("expected std::invalid_argument");
  } catch (const std::invalid_argument& e) {
    const std::string message = e.what();
    CHECK(message.find('5') != std::string::npos);
    CHECK(message.find('2') != std::string::npos);
    CHECK(message.find("fixed_chain") != std::string::npos);
  }
}

TEST_CASE("for a robot with mimic joints the message explains the count difference") {
  // numJoints() == 3, numDofs() == 2. Whoever uses numJoints() gets an
  // exception, and it must say why outright, or it looks like a library bug.
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "hand");
  try {
    forwardKinematics(robot, zeros(robot.numJoints()));
    FAIL("expected std::invalid_argument");
  } catch (const std::invalid_argument& e) {
    CHECK(std::string(e.what()).find("mimic") != std::string::npos);
  }
}
