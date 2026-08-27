#include <doctest/doctest.h>

#include <Eigen/Core>
#include <string>

#include "robometrics/jacobian.hpp"

namespace {
std::string fixture(const char* f) {
  return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + f;
}
}  // namespace

TEST_CASE("jacobian ma rozmer 6 x numDofs") {
  const robometrics::Robot robot =
      robometrics::Robot::fromUrdfFile(fixture("two_joint.urdf"));
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.numDofs());

  const Eigen::MatrixXd J = robometrics::jacobian(robot, q);
  CHECK(J.rows() == 6);
  CHECK(J.cols() == robot.numDofs());
}

TEST_CASE("jacobian sedi s numerickou derivaci") {
  const robometrics::Robot robot =
      robometrics::Robot::fromUrdfFile(fixture("two_joint.urdf"));

  Eigen::VectorXd q(robot.numDofs());
  q << 0.4, -0.7;

  const Eigen::MatrixXd J = robometrics::jacobian(robot, q);

  const double h = 1e-6;
  for (int i = 0; i < robot.numDofs(); ++i) {
    Eigen::VectorXd qp = q, qm = q;
    qp(i) += h;
    qm(i) -= h;

    const robometrics::SE3 Tp = robometrics::forwardKinematics(robot, qp);
    const robometrics::SE3 Tm = robometrics::forwardKinematics(robot, qm);

    // twist mezi dvema blizkymi polohami, deleno krokem
    const robometrics::Vec6 numeric = robometrics::log(Tm.inverse() * Tp) / (2.0 * h);

    const robometrics::SE3 T = robometrics::forwardKinematics(robot, q);
    const robometrics::SE3 rotOnly(T.rotation(), robometrics::Vec3::Zero());
    const robometrics::Vec6 inBase = robometrics::adjoint(rotOnly) * numeric;
  
    CHECK((J.col(i) - inBase).cwiseAbs().maxCoeff() < 1e-6);
  }
}

TEST_CASE("jednokloubovy robot, sloupec spocitany rucne") {
  // Kloub kolem z v (0,0,0.1), rameno 0.5 podel x -> tip v (0.5, 0, 0.1).
  // Pri q=0 a otaceni rychlosti 1:
  //   omega = (0, 0, 1)
  //   v     = omega x (p_tip - p_joint) = (0,0,1) x (0.5,0,0) = (0, 0.5, 0)
  const robometrics::Robot robot =
      robometrics::Robot::fromUrdfFile(fixture("single_joint.urdf"));

  Eigen::VectorXd q(1);
  q << 0.0;

  const Eigen::MatrixXd J = robometrics::jacobian(robot, q);

  robometrics::Vec6 expected;
  expected << 0.0, 0.5, 0.0,   // v
              0.0, 0.0, 1.0;   // omega

  CHECK((J.col(0) - expected).cwiseAbs().maxCoeff() < 1e-12);
}

TEST_CASE("numericka derivace sedi i s mimic klouby") {
  const robometrics::Robot robot =
      robometrics::Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "hand");

  Eigen::VectorXd q(robot.numDofs());
  q << 0.3, 0.02;

  const Eigen::MatrixXd J = robometrics::jacobian(robot, q);
  const robometrics::SE3 T = robometrics::forwardKinematics(robot, q);
  const robometrics::SE3 rotOnly(T.rotation(), robometrics::Vec3::Zero());

  const double h = 1e-6;
  for (int i = 0; i < robot.numDofs(); ++i) {
    Eigen::VectorXd qp = q, qm = q;
    qp(i) += h;
    qm(i) -= h;

    const robometrics::Vec6 numeric =
        robometrics::log(robometrics::forwardKinematics(robot, qm).inverse() *
                         robometrics::forwardKinematics(robot, qp)) / (2.0 * h);
    const robometrics::Vec6 inBase = robometrics::adjoint(rotOnly) * numeric;

    CHECK((J.col(i) - inBase).cwiseAbs().maxCoeff() < 1e-6);
  }
}