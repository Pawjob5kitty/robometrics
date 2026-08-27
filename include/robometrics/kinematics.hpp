#pragma once

#include <Eigen/Core>
#include <vector>

#include "robometrics/se3.hpp"
#include "robometrics/urdf.hpp"

/// Forward kinematics: joint values in, frame poses out.
///
/// The whole module is one idea applied repeatedly. Each joint contributes a
/// transform that depends on its own value alone,
///
///     jointTransform(i, q_i) = originTransform_i * motion_i(q_i)
///
/// and the pose of a frame is the product of those transforms along the path
/// from the root to it. Everything below is bookkeeping around that product:
/// walking the tree in the right order, folding in the fixed offsets that the
/// parser already collapsed, and resolving the values of mimic joints.
///
/// FRAMES. Every pose returned here is expressed in the ROOT LINK frame, which
/// is the identity by definition. Nothing in this library knows about a world
/// frame; if the robot base moves, that is a transform the caller applies on
/// the left.
namespace robometrics {

/// The value a joint actually takes for a given configuration.
///
/// For an independent joint this is just q(joint.dofIndex). For a mimic joint
/// it is the affine function of its source that URDF's <mimic> declares:
///
///     value = mimicMultiplier * value(source) + mimicOffset
///
/// Pre:  q.size() == robot.numDofs(); joint belongs to robot.
/// Post: finite whenever q is finite.
///
/// Exposed rather than kept private because a Jacobian needs it too, and
/// because a caller checking a configuration against joint limits has to
/// compare the DERIVED value for a mimic joint -- comparing the driver's value
/// against the mimic's limits is a bug that produces plausible answers.
double jointValue(const Robot& robot, const Joint& joint, const Eigen::VectorXd& q);

/// The transform a joint contributes at a given value: its fixed origin
/// followed by its motion.
///
///   revolute, continuous:  motion(v) = rotation by v about `axis`
///   prismatic:             motion(v) = translation by v along `axis`
///
/// Note that `axis` lives in the joint's OWN frame, so the motion is applied on
/// the RIGHT of originTransform, not the left. Swapping the two would rotate
/// about an axis through the parent's origin instead of the joint's -- which is
/// the same thing whenever the origin happens to be zero, and different for
/// every real robot.
///
/// Pre:  joint.type is not JointType::Fixed (the parser guarantees this).
/// Post: jointTransform(joint, 0) == joint.originTransform.
///
/// Must hold:
///   - for a revolute joint, jointTransform(j, a) * jointTransform(j, b) is NOT
///     generally jointTransform(j, a + b) -- the origin sits in between. The
///     motions alone do compose that way; the full transforms do not.
SE3 jointTransform(const Joint& joint, double value);

/// Pose of every link, in the root link frame, indexed by LINK index.
///
/// The result has robot.numLinks() entries, so `all[robot.tipLinkIndex()]` is
/// the end-effector and `all[robot.rootLinkIndex()]` is the identity.
///
/// The pose of JOINT i's own frame -- what a Jacobian iterates over -- is
/// `all[robot.joint(i).childLink]`, since a movable joint's child link is by
/// construction placed exactly at the joint frame after the motion.
///
/// Pre:  q.size() == robot.numDofs(), else std::invalid_argument.
/// Post: result.size() == robot.numLinks();
///       result[robot.rootLinkIndex()] == SE3::identity().
///
/// Must hold:
///   - forwardKinematicsAll(r, VectorXd::Zero(n)) reproduces the composition of
///     the originTransform chain, with every joint's motion equal to identity
///   - result[r.tipLinkIndex()] == forwardKinematics(r, q)
std::vector<SE3> forwardKinematicsAll(const Robot& robot, const Eigen::VectorXd& q);

/// Pose of the end-effector, in the root link frame.
///
/// Equivalent to forwardKinematicsAll(robot, q)[robot.tipLinkIndex()], and
/// computed the same way -- there is no cheaper path. Walking only the ancestors
/// of the tip would skip other branches, but it would still have to visit every
/// joint between the root and the tip, and for the chains this library sees the
/// saving is not worth a second code path that can disagree with the first.
///
/// Pre:  q.size() == robot.numDofs(), else std::invalid_argument.
SE3 forwardKinematics(const Robot& robot, const Eigen::VectorXd& q);

}  // namespace robometrics
