#include "robometrics/jacobian.hpp"
#include <Eigen/Geometry>
#include <vector>
#include "robometrics/se3.hpp"

namespace robometrics {

Eigen::MatrixXd jacobian(const Robot& robot, const Eigen::VectorXd& q) {
  // 1. poloha vsech ramcu
  const std::vector<SE3> poses = forwardKinematicsAll(robot, q);

  // 2. pozice chapadla
  const Vec3 pTip = poses[static_cast<std::size_t>(robot.tipLinkIndex())].translation();

  // 3. prazdna matice 6 x numDofs, vynulovana
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot.numDofs());

  // 4. pro kazdy kloub: spocitat prispevek a pricist do spravneho sloupce
  for (int i = 0; i < robot.numJoints(); ++i) {
    const Joint& joint = robot.joint(i);
    const SE3& frame = poses[static_cast<std::size_t>(joint.childLink)];

    const Vec3 axisWorld = frame.rotation() * joint.axis;

    Vec3 v, w;
    if (joint.type == JointType::Prismatic) {
      w = Vec3::Zero();
      v = axisWorld;
    } else {
      w = axisWorld;
      v = axisWorld.cross(pTip - frame.translation());
    }

    const int col = joint.isMimic() ? robot.joint(joint.mimicSource).dofIndex : joint.dofIndex;
    const double scale = joint.isMimic() ? joint.mimicMultiplier : 1.0;

    J.block<3, 1>(0, col) += scale * v;
    J.block<3, 1>(3, col) += scale * w;
  }

  return J;
}

}  // namespace robometrics