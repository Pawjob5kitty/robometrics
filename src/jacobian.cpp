#include "robometrics/jacobian.hpp"
#include <Eigen/Geometry>
#include <vector>
#include "robometrics/se3.hpp"

namespace robometrics {

Eigen::MatrixXd jacobian(const Robot& robot, const Eigen::VectorXd& q) {
  // 1. pose of every frame
  const std::vector<SE3> poses = forwardKinematicsAll(robot, q);

  // 2. gripper position
  const Vec3 pTip = poses[static_cast<std::size_t>(robot.tipLinkIndex())].translation();

  // 3. empty 6 x numDofs matrix, zeroed
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot.numDofs());

  // 4. for each joint: compute the contribution and add it to the right column
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