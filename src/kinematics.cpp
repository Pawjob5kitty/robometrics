#include "robometrics/kinematics.hpp"

#include <sstream>
#include <stdexcept>

namespace robometrics {
namespace {

// A dimension mismatch is a caller bug -- the wrong vector was handed in --
// not a malformed input file, so it does not go through UrdfError. Everything
// UrdfError carries (which element, which attribute) would be empty here.
void requireMatchingSize(const Robot& robot, const Eigen::VectorXd& q) {
  if (q.size() == robot.numDofs()) {
    return;
  }
  std::ostringstream msg;
  msg << "configuration vector has " << q.size() << " entries but robot '" << robot.name()
      << "' has " << robot.numDofs() << " degrees of freedom";
  if (robot.numDofs() != robot.numJoints()) {
    // The single most likely cause of this error, so say it outright rather
    // than leaving the caller to discover mimic joints on their own.
    msg << " (" << robot.numJoints() << " movable joints, of which "
        << robot.numJoints() - robot.numDofs()
        << " are mimic joints whose values are derived, not supplied)";
  }
  throw std::invalid_argument(msg.str());
}

}  // namespace

double jointValue(const Robot& robot, const Joint& joint, const Eigen::VectorXd& q) {
  if (!joint.isMimic()) {
    return q(joint.dofIndex);
  }
  // Single-level only: the parser rejects a mimic whose source is itself a
  // mimic, so one lookup is enough and no recursion or cycle check is needed
  // here. That restriction is what keeps this function a straight line.
  const Joint& source = robot.joint(joint.mimicSource);
  return joint.mimicMultiplier * q(source.dofIndex) + joint.mimicOffset;
}

SE3 jointTransform(const Joint& joint, double value) {
  switch (joint.type) {
    case JointType::Revolute:
    case JointType::Continuous: {
      // The axis is a unit vector and `value` is the angle, so axis * value is
      // precisely the rotation vector rodrigues() wants. This is where the
      // parser's normalisation pays off: an axis of length 5 would silently
      // turn every commanded angle into five times that angle.
      const SE3 motion(rodrigues(joint.axis * value), Vec3::Zero());
      return joint.originTransform * motion;
    }
    case JointType::Prismatic: {
      const SE3 motion(Mat3::Identity(), joint.axis * value);
      return joint.originTransform * motion;
    }
    case JointType::Fixed:
      break;
  }
  // Unreachable: the parser folds fixed joints away and never emits one into
  // the joint array. Returning the origin rather than throwing keeps this
  // function total, which matters because it sits in the inner loop.
  return joint.originTransform;
}

std::vector<SE3> forwardKinematicsAll(const Robot& robot, const Eigen::VectorXd& q) {
  requireMatchingSize(robot, q);

  // Pass one: pose of each JOINT frame.
  //
  // Joints are stored in topological order, so joint i's parent is always at a
  // smaller index and its pose is already final by the time we reach i. That is
  // the entire reason the parser sorts them -- forward kinematics collapses to
  // a single forward sweep with no recursion, no stack, and no revisiting.
  //
  // A mimic joint needs its source's VALUE, not its pose, and jointValue()
  // reads that straight out of q. So mimic joints impose no ordering constraint
  // of their own; the source may sit at a higher index, or on a different
  // branch entirely.
  std::vector<SE3> jointPoses;
  jointPoses.reserve(static_cast<std::size_t>(robot.numJoints()));
  for (int i = 0; i < robot.numJoints(); ++i) {
    const Joint& joint = robot.joint(i);
    const SE3 local = jointTransform(joint, jointValue(robot, joint, q));

    if (joint.parentJoint < 0) {
      // Hangs directly off the root link, whose pose is the identity.
      jointPoses.push_back(local);
    } else {
      jointPoses.push_back(jointPoses[static_cast<std::size_t>(joint.parentJoint)] * local);
    }
  }

  // Pass two: pose of each LINK.
  //
  // A link is rigidly attached to its supporting joint, offset by the fixed
  // transforms the parser folded into Link::offset. Links with no supporting
  // joint are rigid with respect to the root, which covers the root itself
  // (offset == identity) and any fixed frame bolted to the base.
  std::vector<SE3> linkPoses;
  linkPoses.reserve(static_cast<std::size_t>(robot.numLinks()));
  for (int i = 0; i < robot.numLinks(); ++i) {
    const Link& link = robot.link(i);
    if (link.supportingJoint < 0) {
      linkPoses.push_back(link.offset);
    } else {
      linkPoses.push_back(jointPoses[static_cast<std::size_t>(link.supportingJoint)] * link.offset);
    }
  }
  return linkPoses;
}

SE3 forwardKinematics(const Robot& robot, const Eigen::VectorXd& q) {
  // Deliberately the same code path as forwardKinematicsAll rather than a
  // root-to-tip walk. Two implementations of the same formula drift apart, and
  // the test that they agree is only as good as the cases it covers.
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);
  return all[static_cast<std::size_t>(robot.tipLinkIndex())];
}

}  // namespace robometrics
