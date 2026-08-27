#pragma once

#include <Eigen/Core>
#include <vector>

#include "robometrics/se3.hpp"
#include "robometrics/urdf.hpp"

/// Forward kinematics: joint values in, frame poses out.
///
/// Each joint contributes a transform depending on its own value alone,
///
///     jointTransform(i, q_i) = originTransform_i * motion_i(q_i)
///
/// and a frame's pose is the product of those along the path from the root.
///
/// Every pose returned here is in the ROOT LINK frame, which is the identity by
/// definition. Nothing in this library knows about a world frame; if the base
/// moves, that is a transform the caller applies on the left.
namespace robometrics {

/// The value a joint actually takes for a given configuration.
///
/// For an independent joint this is q(joint.dofIndex); for a mimic joint it is
/// the affine function of its source that <mimic> declares:
///
///     value = mimicMultiplier * value(source) + mimicOffset
///
/// Pre:  q.size() == robot.numDofs(); joint belongs to robot.
///
/// Public because a caller checking a configuration against joint limits has to
/// compare the DERIVED value for a mimic joint. Comparing the driver's value
/// against the mimic's limits is a bug that produces plausible answers.
double jointValue(const Robot& robot, const Joint& joint, const Eigen::VectorXd& q);

/// The transform a joint contributes at a given value: its fixed origin
/// followed by its motion.
///
///   revolute, continuous:  motion(v) = rotation by v about `axis`
///   prismatic:             motion(v) = translation by v along `axis`
///
/// `axis` lives in the joint's OWN frame, so the motion multiplies from the
/// RIGHT of originTransform. Swapping the two rotates about an axis through the
/// parent's origin instead of the joint's -- the same thing whenever the origin
/// happens to be zero, and different for every real robot.
///
/// Pre:  joint.type is not JointType::Fixed (the parser guarantees this).
/// Post: jointTransform(joint, 0) == joint.originTransform.
SE3 jointTransform(const Joint& joint, double value);

/// Pose of every link, in the root link frame, indexed by LINK index.
///
/// `all[robot.tipLinkIndex()]` is the end-effector; `all[robot.rootLinkIndex()]`
/// is the identity.
///
/// The pose of JOINT i's own frame -- what a Jacobian iterates over -- is
/// `all[robot.joint(i).childLink]`: a movable joint's child link sits exactly
/// at the joint frame after the motion.
///
/// Pre:  q.size() == robot.numDofs(), else std::invalid_argument.
/// Post: result.size() == robot.numLinks();
///       result[robot.rootLinkIndex()] == SE3::identity();
///       result[r.tipLinkIndex()] == forwardKinematics(r, q).
std::vector<SE3> forwardKinematicsAll(const Robot& robot, const Eigen::VectorXd& q);

/// Pose of the end-effector, in the root link frame.
///
/// Deliberately the same code path as forwardKinematicsAll rather than a
/// root-to-tip walk: two implementations of one formula drift apart, and the
/// test that they agree is only as good as the cases it covers.
///
/// Pre:  q.size() == robot.numDofs(), else std::invalid_argument.
SE3 forwardKinematics(const Robot& robot, const Eigen::VectorXd& q);

}  // namespace robometrics
