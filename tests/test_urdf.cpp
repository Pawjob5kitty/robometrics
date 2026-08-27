// Tests for the URDF parser.
//
// Fixtures are files in tests/fixtures/, not inline strings, so the
// Robot::fromUrdfFile() path is exercised and a failing file can be opened and
// read. CMake passes their directory as the ROBOMETRICS_FIXTURE_DIR macro.
//
// The error tests deliberately assert not just "it threw" but WHAT the message
// says. The message is part of the API here: when someone gets a URDF from a
// third-party vendor, that one sentence is all they have to go on.

#include <doctest/doctest.h>

// determinant() lives in Eigen/LU, not Eigen/Core -- the same trap as .cross()
// in test_se3.cpp: it compiles and fails only at link time.
#include <Eigen/LU>
#include <string>
#include <vector>

#include "robometrics/urdf.hpp"

namespace {

using robometrics::JointType;
using robometrics::Mat3;
using robometrics::Robot;
using robometrics::SE3;
using robometrics::UrdfError;
using robometrics::Vec3;

/// Tolerance for comparing transforms. Looser than kDefaultTol (1e-9) because
/// rpy goes through rodrigues() -- cos(pi/2) comes out at 6.1e-17 instead of
/// zero, and similar tiny residuals appear after all three products. The error
/// is of order 1e-16, so 1e-12 has four orders of headroom and would still
/// catch any real convention or sign error.
constexpr double kTol = 1e-12;

std::string fixture(const char* fileName) {
  return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + fileName;
}

/// Builds a Mat3 row by row so the test reads like a matrix on paper. Eigen has
/// operator<<, but it reads poorly in a test because of the commas.
Mat3 rows(double a, double b, double c, double d, double e, double f, double g, double h,
          double i) {
  Mat3 m;
  // clang-format off
  m << a, b, c,
       d, e, f,
       g, h, i;
  // clang-format on
  return m;
}

}  // namespace

// ---------------------------------------------------------------------------
// A valid minimal URDF
// ---------------------------------------------------------------------------

TEST_CASE("a two-joint chain loads with the right values") {
  // The basic "does it load at all" test. If only this one fails, the fault is
  // low down (XML, attribute names), not in the topology.
  const Robot robot = Robot::fromUrdfFile(fixture("two_joint.urdf"));

  CHECK(robot.name() == "two_joint");
  CHECK(robot.numJoints() == 2);
  CHECK(robot.numDofs() == 2);  // no mimic => same as numJoints()
  CHECK(robot.numLinks() == 3);

  // Joint order must run from base to tip regardless of the file order.
  CHECK(robot.joint(0).name == "joint1");
  CHECK(robot.joint(1).name == "joint2");

  // parentJoint == -1 means "hangs directly off the root link". For joint2 it
  // must be index 0 -- this is the field forward kinematics walks.
  CHECK(robot.joint(0).parentJoint == -1);
  CHECK(robot.joint(1).parentJoint == 0);

  CHECK(robot.joint(0).type == JointType::Revolute);
  CHECK(robot.joint(0).axis.isApprox(Vec3::UnitZ()));
  CHECK(robot.joint(1).axis.isApprox(Vec3::UnitY()));

  CHECK(robot.joint(0).lowerLimit == doctest::Approx(-1.5));
  CHECK(robot.joint(0).upperLimit == doctest::Approx(1.5));
  CHECK(robot.joint(0).effortLimit == doctest::Approx(87.0));
  CHECK(robot.joint(0).velocityLimit == doctest::Approx(2.61));

  CHECK(robot.joint(0).originTransform.translation().isApprox(Vec3(0.0, 0.0, 0.1)));
  CHECK(robot.joint(1).originTransform.translation().isApprox(Vec3(0.3, 0.0, 0.0)));

  // The only leaf => tip auto-detection must find it, with no second argument.
  CHECK(robot.tipLinkIndex() == robot.findLink("link2"));
  CHECK(robot.rootLinkIndex() == robot.findLink("base"));
}

TEST_CASE("fromUrdfString gives the same result as fromUrdfFile") {
  // If the two paths diverged, the string-based tests would stop saying anything
  // about what happens with a real file.
  const Robot fromFile = Robot::fromUrdfFile(fixture("two_joint.urdf"));

  const std::string xml =
      "<robot name=\"two_joint\">"
      "  <link name=\"base\"/><link name=\"link1\"/><link name=\"link2\"/>"
      "  <joint name=\"joint1\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"link1\"/>"
      "    <origin xyz=\"0 0 0.1\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-1.5\" upper=\"1.5\" effort=\"87\" velocity=\"2.61\"/>"
      "  </joint>"
      "  <joint name=\"joint2\" type=\"revolute\">"
      "    <parent link=\"link1\"/><child link=\"link2\"/>"
      "    <origin xyz=\"0.3 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 1 0\"/>"
      "    <limit lower=\"-2\" upper=\"2\" effort=\"12\" velocity=\"2.61\"/>"
      "  </joint>"
      "</robot>";
  const Robot fromString = Robot::fromUrdfString(xml);

  REQUIRE(fromString.numJoints() == fromFile.numJoints());
  for (int i = 0; i < fromFile.numJoints(); ++i) {
    CHECK(fromString.joint(i).name == fromFile.joint(i).name);
    CHECK(fromString.joint(i).originTransform.isApprox(fromFile.joint(i).originTransform, kTol));
  }
}

// ---------------------------------------------------------------------------
// Folding fixed joints
// ---------------------------------------------------------------------------

TEST_CASE("a fixed joint mid-chain folds into the following joint") {
  // The most important test in the whole parser. A fixed joint must neither
  // vanish nor stay as a degree of freedom of its own -- it has to fold into the
  // originTransform of the next movable joint.
  //
  // Chain: base -[joint1]-> link1 -[mount_fixed 0.2]-> mount -[joint2 0.3]-> link2
  // So joint2 starts at 0.2 + 0.3 = 0.5 from link1, not at 0.3.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  CHECK(robot.numJoints() == 2);  // four joints in the file, two movable
  CHECK(robot.numLinks() == 5);   // but links are not discarded, all five stay

  CHECK(robot.joint(0).name == "joint1");
  CHECK(robot.joint(1).name == "joint2");

  // If mount_fixed were discarded this would be 0.3. If it stayed as a joint,
  // numJoints() would be 4 and this CHECK would never be reached.
  CHECK(robot.joint(1).originTransform.translation().isApprox(Vec3(0.5, 0.0, 0.0)));

  // parentJoint skips the fixed joint: joint2 hangs off joint1, not off nothing.
  CHECK(robot.joint(1).parentJoint == 0);

  // Link 'mount' still exists, supported by joint1 and offset by 0.2.
  const int mount = robot.findLink("mount");
  REQUIRE(mount >= 0);
  CHECK(robot.link(mount).supportingJoint == 0);
  CHECK(robot.link(mount).offset.translation().isApprox(Vec3(0.2, 0.0, 0.0)));
}

TEST_CASE("a trailing fixed joint survives as a link offset") {
  // Folding "forward into the next movable joint" would lose this case -- there
  // is no movable joint after tool_fixed. And this is exactly what the end of
  // every real manipulator looks like (hand -> flange -> grasp target), so the
  // end-effector would settle a few centimetres off, silently.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  const int tool = robot.findLink("tool");
  REQUIRE(tool >= 0);

  // Supported by the last movable joint, offset by its own fixed origin.
  CHECK(robot.link(tool).supportingJoint == 1);
  CHECK(robot.link(tool).offset.translation().isApprox(Vec3(0.0, 0.0, 0.05)));

  // And it is the auto-detected tip, because it is the only leaf.
  CHECK(robot.tipLinkIndex() == tool);
}

TEST_CASE("the root link has no supporting joint and sits at identity") {
  // The reference frame of the whole library. If the root got a nonzero offset,
  // everything would shift and no other test need catch it, because every pose
  // would shift by the same amount.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  const int root = robot.rootLinkIndex();

  CHECK(robot.link(root).name == "base");
  CHECK(robot.link(root).supportingJoint == -1);
  CHECK(robot.link(root).offset.isApprox(SE3::identity(), kTol));
}

// ---------------------------------------------------------------------------
// rpy convention against hand-computed numbers
// ---------------------------------------------------------------------------

TEST_CASE("rpy = (0, 0, pi/2) is a rotation about z") {
  // The simplest case, but on its own it says NOTHING about the product order:
  // with two of the three angles zero, Rz*Ry*Rx and Rx*Ry*Rz give the same
  // thing. It is here as a sign check, not a convention check.
  //
  //   Rz(pi/2) = | 0  -1   0 |
  //              | 1   0   0 |
  //              | 0   0   1 |
  const Robot robot = Robot::fromUrdfFile(fixture("rpy_cases.urdf"));
  const int idx = robot.findJoint("case_yaw");
  REQUIRE(idx >= 0);

  // clang-format off
  const Mat3 expected = rows( 0.0, -1.0,  0.0,
                              1.0,  0.0,  0.0,
                              0.0,  0.0,  1.0);
  // clang-format on
  const Mat3 actual = robot.joint(idx).originTransform.rotation();
  CHECK((actual - expected).cwiseAbs().maxCoeff() < kTol);
}

TEST_CASE("rpy = (pi/2, pi/2, 0) distinguishes Rz*Ry*Rx from Rx*Ry*Rz") {
  // This is the test that actually pins the convention. By hand:
  //
  //   Rx(pi/2) = | 1  0  0 |     Ry(pi/2) = |  0  0  1 |     Rz(0) = I
  //              | 0  0 -1 |                |  0  1  0 |
  //              | 0  1  0 |                | -1  0  0 |
  //
  //   Rz(0)*Ry(pi/2)*Rx(pi/2) = Ry*Rx:
  //     row 0 Ry = ( 0, 0, 1) times cols of Rx (1,0,0),(0,0,1),(0,-1,0)
  //                           -> ( 0,  1,  0)
  //     row 1 Ry = ( 0, 1, 0) -> ( 0,  0, -1)
  //     row 2 Ry = (-1, 0, 0) -> (-1,  0,  0)
  //
  //   correct (Rz*Ry*Rx)     wrong (Rx*Ry*Rz)
  //     |  0   1   0 |          | 0  0  1 |
  //     |  0   0  -1 |          | 1  0  0 |
  //     | -1   0   0 |          | 0  1  0 |
  //
  // With the convention reversed the second matrix would come out -- and that
  // happens to be the correct result for case_ry below. So the test would not
  // fail on "garbage" but on a specific, different rotation.
  const Robot robot = Robot::fromUrdfFile(fixture("rpy_cases.urdf"));
  const int idx = robot.findJoint("case_rp");
  REQUIRE(idx >= 0);

  // clang-format off
  const Mat3 expected   = rows( 0.0,  1.0,  0.0,
                                0.0,  0.0, -1.0,
                               -1.0,  0.0,  0.0);
  const Mat3 wrongOrder = rows( 0.0,  0.0,  1.0,
                                1.0,  0.0,  0.0,
                                0.0,  1.0,  0.0);
  // clang-format on
  const Mat3 actual = robot.joint(idx).originTransform.rotation();

  CHECK((actual - expected).cwiseAbs().maxCoeff() < kTol);
  // Explicitly that it is NOT the reversed order -- if someone later swaps
  // rz*ry*rx for rx*ry*rz, this line names exactly what happened rather than
  // just failing the number in the previous CHECK.
  CHECK((actual - wrongOrder).cwiseAbs().maxCoeff() > 0.5);

  // xyz applies independently of rpy, so the translation is checked separately.
  CHECK(robot.joint(idx).originTransform.translation().isApprox(Vec3(0.1, -0.2, 0.3)));
}

TEST_CASE("rpy = (pi/2, 0, pi/2) is a cyclic permutation of the axes") {
  // The third case, two nonzero angles with a zero between them. By hand:
  //
  //   Rz(pi/2)*Ry(0)*Rx(pi/2) = Rz*Rx:
  //     row 0 Rz = (0, -1, 0) times cols of Rx (1,0,0),(0,0,1),(0,-1,0)
  //                           -> (0, 0, 1)
  //     row 1 Rz = (1,  0, 0) -> (1, 0, 0)
  //     row 2 Rz = (0,  0, 1) -> (0, 1, 0)
  //
  //   | 0  0  1 |    x -> y, y -> z, z -> x
  //   | 1  0  0 |
  //   | 0  1  0 |
  //
  // An easy check without arithmetic: the matrix columns are the images of the
  // basis vectors, so the first column (0,1,0) says x maps to y. That matches.
  const Robot robot = Robot::fromUrdfFile(fixture("rpy_cases.urdf"));
  const int idx = robot.findJoint("case_ry");
  REQUIRE(idx >= 0);

  // clang-format off
  const Mat3 expected = rows( 0.0,  0.0,  1.0,
                              1.0,  0.0,  0.0,
                              0.0,  1.0,  0.0);
  // clang-format on
  const Mat3 actual = robot.joint(idx).originTransform.rotation();
  CHECK((actual - expected).cwiseAbs().maxCoeff() < kTol);

  // A check via the images of the basis vectors, independent of whether I wrote
  // the matrix out correctly by rows or by columns.
  CHECK((actual * Vec3::UnitX()).isApprox(Vec3::UnitY()));
  CHECK((actual * Vec3::UnitY()).isApprox(Vec3::UnitZ()));
  CHECK((actual * Vec3::UnitZ()).isApprox(Vec3::UnitX()));
}

TEST_CASE("the rpy result is always a proper rotation") {
  // A structural invariant that holds regardless of convention. It catches
  // errors the specific numbers above would miss -- for instance if someone
  // accidentally composed rotations with a scale or a reflection.
  const Robot robot = Robot::fromUrdfFile(fixture("rpy_cases.urdf"));
  for (int i = 0; i < robot.numJoints(); ++i) {
    const Mat3 r = robot.joint(i).originTransform.rotation();
    CHECK((r.transpose() * r - Mat3::Identity()).cwiseAbs().maxCoeff() < kTol);
    CHECK(r.determinant() == doctest::Approx(1.0));
  }
}

// ---------------------------------------------------------------------------
// Mimic joints
// ---------------------------------------------------------------------------

TEST_CASE("a mimic joint counts in numJoints but not in numDofs") {
  // This is why the two counts exist separately. If the mimic tag were ignored,
  // numDofs() would be 3 and a recorded rollout's q (length 2) would be indexed
  // off by one -- with no exception, just wrong numbers.
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "left");

  CHECK(robot.numJoints() == 3);
  CHECK(robot.numDofs() == 2);

  const int right = robot.findJoint("finger_right");
  const int left = robot.findJoint("finger_left");
  REQUIRE(right >= 0);
  REQUIRE(left >= 0);

  CHECK(robot.joint(right).isMimic());
  CHECK(robot.joint(right).mimicSource == left);
  CHECK(robot.joint(right).mimicMultiplier == doctest::Approx(-1.0));
  CHECK(robot.joint(right).mimicOffset == doctest::Approx(0.0));
  CHECK(robot.joint(right).dofIndex == -1);  // value is derived, not read from q

  CHECK_FALSE(robot.joint(left).isMimic());
  CHECK(robot.joint(left).dofIndex >= 0);
}

TEST_CASE("dofIndex is dense and has no holes") {
  // The invariant q indexing rests on: the independent joints get
  // 0, 1, ..., numDofs()-1, each exactly once. If a hole opened in the
  // numbering, FK would read q out of range or a joint would silently stay at
  // zero.
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "left");

  std::vector<bool> seen(static_cast<std::size_t>(robot.numDofs()), false);
  for (int i = 0; i < robot.numJoints(); ++i) {
    const int dof = robot.joint(i).dofIndex;
    if (robot.joint(i).isMimic()) {
      CHECK(dof == -1);
      continue;
    }
    REQUIRE(dof >= 0);
    REQUIRE(dof < robot.numDofs());
    CHECK_FALSE(seen[static_cast<std::size_t>(dof)]);
    seen[static_cast<std::size_t>(dof)] = true;
  }
  for (const bool used : seen) {
    CHECK(used);
  }
}

// ---------------------------------------------------------------------------
// Error cases
// ---------------------------------------------------------------------------

TEST_CASE("a missing <axis> on a revolute joint is an error with a clear message") {
  // The spec would default it to "1 0 0". This parser rejects it, because
  // quietly rotating about x is exactly the kind of error that produces a robot
  // that looks plausible and is wrong.
  try {
    Robot::fromUrdfFile(fixture("missing_axis.urdf"));
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    // The message must name the specific joint -- on a 3000-line URDF "missing
    // axis" without a name is useless.
    CHECK(message.find("joint2") != std::string::npos);
    CHECK(message.find("axis") != std::string::npos);
    // And it must say this is a deliberate choice, not that the file is invalid.
    CHECK(message.find("default") != std::string::npos);
    CHECK(e.where().find("joint2") != std::string::npos);
  }
}

TEST_CASE("an unknown joint type is an error that lists the supported types") {
  try {
    Robot::fromUrdfFile(fixture("unknown_type.urdf"));
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("joint1") != std::string::npos);
    CHECK(message.find("screw") != std::string::npos);     // what was written
    CHECK(message.find("revolute") != std::string::npos);  // what is allowed
  }
}

TEST_CASE("planar and floating are rejected as multi-dimensional") {
  // A message distinct from a fully unknown type: these two ARE valid URDF,
  // they just do not fit "one scalar per joint". That is our limitation, not a
  // fault in the file, and the message must say so.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"floating\">"
      "    <parent link=\"base\"/><child link=\"l\"/>"
      "  </joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("floating") != std::string::npos);
    CHECK(message.find("degree of freedom") != std::string::npos);
  }
}

TEST_CASE("a zero axis is an error, not a case to normalise") {
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "  </joint>"
      "</robot>";
  CHECK_THROWS_AS(Robot::fromUrdfString(xml), UrdfError);
}

TEST_CASE("the axis is normalised") {
  // An unnormalised axis in the file is not an error, but must not be used as
  // is: the joint would rotate by ||axis|| * q instead of q. A scale error on
  // every metric, and utterly invisible in a test with clean axes.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 5\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "  </joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);
  CHECK(robot.joint(0).axis.isApprox(Vec3::UnitZ()));
  CHECK(robot.joint(0).axis.norm() == doctest::Approx(1.0));
}

TEST_CASE("several leaves mean the tip cannot be guessed") {
  // Every robot with a gripper. Guessing among the fingers would put the
  // end-effector on a random one; the message lists the candidates so the fix is
  // clear from it.
  try {
    Robot::fromUrdfFile(fixture("mimic_gripper.urdf"));
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("left") != std::string::npos);
    CHECK(message.find("right") != std::string::npos);
  }
}

TEST_CASE("an explicitly given tip overrides auto-detection") {
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "hand");
  CHECK(robot.tipLinkIndex() == robot.findLink("hand"));
}

TEST_CASE("a nonexistent tip is an error") {
  CHECK_THROWS_AS(Robot::fromUrdfFile(fixture("two_joint.urdf"), "nonexistent"), UrdfError);
}

TEST_CASE("a link with two parents is not a tree") {
  // A closed loop (a parallel mechanism). FK would have no unique answer, so it
  // must be rejected rather than picking one branch.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/><link name=\"b\"/>"
      "  <joint name=\"j1\" type=\"fixed\"><parent link=\"base\"/><child link=\"b\"/></joint>"
      "  <joint name=\"j2\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/></joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("exactly one parent") != std::string::npos);
  }
}

TEST_CASE("two roots mean two disconnected trees") {
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"baseA\"/><link name=\"a\"/><link name=\"baseB\"/><link name=\"b\"/>"
      "  <joint name=\"j1\" type=\"fixed\"><parent link=\"baseA\"/><child link=\"a\"/></joint>"
      "  <joint name=\"j2\" type=\"fixed\"><parent link=\"baseB\"/><child link=\"b\"/></joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("baseA") != std::string::npos);
    CHECK(message.find("baseB") != std::string::npos);
  }
}

TEST_CASE("a reference to an undeclared link is an error that names it") {
  // A typical name typo. Without the name in the message it is hard to find.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/>"
      "  <joint name=\"j\" type=\"fixed\">"
      "    <parent link=\"base\"/><child link=\"typo_link\"/>"
      "  </joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("typo_link") != std::string::npos);
  }
}

TEST_CASE("a missing <limit> is an error for revolute but not continuous") {
  // The split is the spec's: revolute is bounded by definition, continuous is not.
  const std::string revolute =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 1\"/>"
      "  </joint>"
      "</robot>";
  CHECK_THROWS_AS(Robot::fromUrdfString(revolute), UrdfError);

  const std::string continuous =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"continuous\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 1\"/>"
      "  </joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(continuous);
  CHECK(robot.joint(0).type == JointType::Continuous);
  CHECK(robot.joint(0).lowerLimit == -robometrics::kUnbounded);
  CHECK(robot.joint(0).upperLimit == robometrics::kUnbounded);
}

TEST_CASE("a non-number in an attribute is an error that quotes what was there") {
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"nope\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "  </joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("lower") != std::string::npos);
    CHECK(message.find("nope") != std::string::npos);
  }
}

TEST_CASE("malformed XML reports the line number") {
  const std::string xml =
      "<robot name=\"r\">\n"
      "  <link name=\"base\"/>\n"
      "  <joint name=\"j\" type=\"fixed\">\n"
      "</robot>\n";
  try {
    Robot::fromUrdfString(xml);
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("line") != std::string::npos);
  }
}

TEST_CASE("a missing file is reported as a missing file") {
  // Not "parse error" -- otherwise a person hunts for a fault in a URDF that does not exist.
  try {
    Robot::fromUrdfFile(fixture("this_file_does_not_exist.urdf"));
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("not found") != std::string::npos);
  }
}

TEST_CASE("a mimic of a mimic is rejected with an explanation") {
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/><link name=\"b\"/><link name=\"c\"/>"
      "  <joint name=\"j1\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"a\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>"
      "  <joint name=\"j2\" type=\"revolute\">"
      "    <parent link=\"a\"/><child link=\"b\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "    <mimic joint=\"j1\"/></joint>"
      "  <joint name=\"j3\" type=\"revolute\">"
      "    <parent link=\"b\"/><child link=\"c\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "    <mimic joint=\"j2\"/></joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("expected UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("chained") != std::string::npos);
  }
}
