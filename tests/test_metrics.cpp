#include <doctest/doctest.h>

#include <Eigen/Core>
#include <string>

#include "robometrics/jacobian.hpp"
#include "robometrics/metrics.hpp"

namespace {
std::string fixture(const char* f) {
  return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + f;
}
}  // namespace

TEST_CASE("natazene rameno je singularita, ohnute ma rezervu") {
  // planar_arm: dva rovnobezne klouby kolem z, ramena 0.3 + 0.3.
  // Pri q2 = 0 lezi tip na primce oboma klouby -> tip se nemuze hnout
  // radialne a J_v ztrati hodnost. Pri q2 = pi/2 je rezerva plna.
  const robometrics::Robot robot =
      robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  Eigen::VectorXd stretched(robot.numDofs());
  stretched << 0.0, 0.0;

  Eigen::VectorXd bent(robot.numDofs());
  bent << 0.0, 1.5708;

  const double sStretched =
      robometrics::sigmaMinTranslation(robometrics::jacobian(robot, stretched));
  const double sBent =
      robometrics::sigmaMinTranslation(robometrics::jacobian(robot, bent));

  CHECK(sStretched < 1e-9);
  CHECK(sBent > 0.1);
}