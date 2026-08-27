#include <doctest/doctest.h>

#include <Eigen/Core>
#include <string>

#include "robometrics/jacobian.hpp"
#include "robometrics/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <vector>
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

TEST_CASE("profil klesne k nule pri pruchodu singularitou") {
  // Trajektorie od ohnuteho ramene pres natazene zpet do ohnuteho.
  // Uprostred musi profil spadnout k nule; na krajich ne.
  const robometrics::Robot robot =
      robometrics::Robot::fromUrdfFile(fixture("planar_arm.urdf"));

  std::vector<Eigen::VectorXd> traj;
  const int n = 21;
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / (n - 1);   // 0 .. 1
    Eigen::VectorXd q(2);
       q << 0.0, 1.5708 * 2.0 * std::abs(t - 0.5);
    traj.push_back(q);
  }

  const std::vector<double> profile = robometrics::sigmaMinProfile(robot, traj);

  REQUIRE(profile.size() == traj.size());
  CHECK(profile.front() > 0.1);
  CHECK(profile.back() > 0.1);
  CHECK(*std::min_element(profile.begin(), profile.end()) < 1e-6);
}