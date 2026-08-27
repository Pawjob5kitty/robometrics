#pragma once

#include <Eigen/Core>
#include <optional>
#include <vector>

#include "robometrics/urdf.hpp"

/// Kinematic quality metrics over recorded rollouts.
///
/// Everything here answers one question: at a given configuration, how freely
/// can the end-effector move? Not whether it is in a good place, not whether
/// the task succeeded -- purely how much of its local velocity space is still
/// available. A configuration where some direction of tip motion costs
/// unbounded joint speed is a singularity, and these metrics measure the
/// distance to one.
namespace robometrics {

/// Smallest singular value of the TRANSLATIONAL rows of the Jacobian.
///
/// GEOMETRIC MEANING. Take every joint-velocity vector of unit length,
/// ||qdot|| == 1, and collect the tip velocities it produces, v = J_v * qdot.
/// That set is an ellipsoid in R^3 -- the translational manipulability
/// ellipsoid. Its semi-axes are exactly the singular values of J_v, each along
/// its own left singular vector.
///
/// So sigma_min is the SHORTEST semi-axis: the direction in which the tip
/// responds least to joint motion, and the number of metres per second of tip
/// speed you get for one unit of joint speed spent in the worst direction.
///
///   large sigma_min  the ellipsoid is round; the tip moves comfortably
///                    whichever way you ask
///   small sigma_min  the ellipsoid is a flattened disc; one direction costs
///                    enormous joint speed
///   sigma_min == 0   the ellipsoid is degenerate. That direction is LOCKED:
///                    no combination of joint velocities produces tip motion
///                    along it, at any speed. This is a singularity.
///
/// WHY ONLY topRows(3). Two reasons, and the second is the one that actually
/// bites.
///
/// First, units. The translational rows carry metres per radian; the
/// rotational rows of a revolute joint are its unit axis vector, which is
/// dimensionless. Singular values of a matrix whose rows are in different
/// units have no meaning -- switching from metres to millimetres would scale
/// the translational rows by 1000 and change the answer arbitrarily, while
/// leaving the rotational ones untouched.
///
/// Second, and worse: stacking rows can only ever INCREASE the smallest
/// singular value. For M = [A; B] we have ||M*x||^2 == ||A*x||^2 + ||B*x||^2,
/// so sigma_min(M) >= sigma_min(A). The rotational rows therefore fill in
/// precisely the directions where the translational block is deficient, and a
/// translational singularity stops being visible. The full 6 x n matrix is
/// singular only when translation AND rotation collapse in the SAME
/// joint-velocity direction, which is a much rarer event than the one being
/// measured here.
///
/// Concretely, on this project's own planar_arm fixture with the arm fully
/// stretched -- a textbook singularity, the tip demonstrably cannot move
/// radially:
///
///     sigma(J_v,   3 x 2) = 0.6708,  0.0000     <- singularity, plainly
///     sigma(J_full, 6 x 2) = 1.5533,  0.1931    <- looks perfectly healthy
///
/// A threshold on the full Jacobian would have called that pose fine.
///
/// WHY s(k - 1) AND NOT s.tail(1). Careful here, because the obvious story is
/// wrong: with Eigen's JacobiSVD the two are the SAME expression.
/// singularValues() already returns exactly min(rows, cols) entries, so for a
/// 3 x 2 Jacobian s.size() == 2 == k, and s(k - 1) is the last element. There
/// are no structural zeros to skip past, because Eigen never produces them.
/// Verified on planar_arm: s.size() == 2 and s(k - 1) == s.tail(1)(0) at both
/// a singular and a well-conditioned pose.
///
/// The explicit min(rows, cols) is therefore defensive rather than
/// load-bearing. It is kept because the same quantity is easy to reach for in
/// a formulation where the distinction DOES matter, and then the padding is
/// silent:
///
///   - eigenvalues of Jv * Jv^T, a 3 x 3 matrix, always yield three numbers,
///     the last of which is a structural zero for a 2-joint arm
///   - copying the spectrum into a fixed-size 3-vector pads with zeros
///   - a different decomposition backend may return a full-length vector
///
/// In any of those, reading the last entry reports zero for a 2-DOF arm in
/// EVERY pose, and the metric goes constant. Writing k out states which value
/// is meant rather than relying on the current backend's sizing.
///
/// Eigen returns the singular values in decreasing order, so index k - 1 is
/// the smallest.
///
/// WHAT IS TRUE ABOUT n < 3. A robot with fewer than three joints cannot span
/// R^3 at all, so its velocity ellipsoid is a flat disc (n == 2) or a segment
/// (n == 1) by construction, in every configuration. sigma_min is then the
/// shortest axis of that LOWER-DIMENSIONAL ellipsoid, not a distance to
/// singularity in R^3. The number is still a valid relative signal along one
/// robot's trajectory, which is what this library uses it for, but it is not
/// comparable across robots with different joint counts.
///
/// LIMITATION. For an arm mixing revolute and prismatic joints the columns of
/// J_v still carry different units (metres per radian against dimensionless
/// metres per metre), so the number is only strictly dimensionally clean for a
/// uniform-joint-type arm. It stays a usable relative indicator in the mixed
/// case, but comparing it across robots of different joint composition is not
/// meaningful.
///
/// Pre:  J has at least 3 rows and at least 1 column.
/// Post: result >= 0, and equals 0 exactly when J_v is rank deficient.
double sigmaMinTranslation(const Eigen::MatrixXd& J);

/// sigmaMinTranslation evaluated at every point of a trajectory.
///
/// Input:  a sequence of configurations, each of length robot.numDofs().
/// Output: a vector of the same length, one value per configuration.
///
/// This is the raw signal; the aggregate below is one summary of it. Keeping
/// the profile available separately matters because WHERE a rollout came close
/// to a singularity is usually more informative than how close it got -- a dip
/// at the approach means something different from a dip during the grasp.
///
/// Pre:  every entry of traj has length robot.numDofs().
/// Post: result.size() == traj.size().
std::vector<double> sigmaMinProfile(const Robot& robot, const std::vector<Eigen::VectorXd>& traj);

/// Worst dexterity along a whole trajectory: the minimum of sigmaMinProfile.
///
/// WHY THE MINIMUM AND NOT THE MEAN. A single pass through a singularity makes
/// the entire trajectory fragile. At that instant the controller needs
/// unbounded joint velocity to keep tracking, tiny errors in the commanded tip
/// motion blow up into large joint motions, and any inverse-kinematics step
/// there is ill-conditioned. The rollout does not recover from that by being
/// well-conditioned before and after.
///
/// A mean would hide exactly this. One bad frame in two hundred moves the
/// average by half a percent, so a trajectory that briefly locks up scores
/// almost identically to one that never does -- and the two are not remotely
/// equivalent to execute. The minimum is the honest aggregate because the
/// failure it describes is not cumulative; it is a single worst moment.
///
/// WHY THE NAME. This is dexterity, not safety and not mobility, and the
/// distinctions are worth stating because the wrong name would invite the
/// wrong conclusions:
///
///   NOT safety.   The metric knows nothing about contact forces, obstacles,
///                 payload, or what the gripper is holding. A robot can be in
///                 a maximally dexterous pose and still crush the glass it is
///                 picking up. A high value here licenses no claim about
///                 anything being safe.
///
///   NOT mobility. Mobility is a property of the MECHANISM -- how many joints
///                 it has, what they can reach. This is a property of one
///                 CONFIGURATION. A seven-joint arm is highly mobile and can
///                 still be completely inept in a specific pose; that is
///                 precisely the case this measures.
///
///   Dexterity is the standard term in the robotics literature for how well a
///   manipulator can move in all directions from a given configuration. The
///   manipulability ellipsoid this is read off comes from Yoshikawa (1985),
///   "Manipulability of Robotic Mechanisms", IJRR 4(2), whose own measure was
///   the ellipsoid's volume, sqrt(det(J * J^T)). Using the shortest semi-axis
///   instead -- sigma_min, as here -- is from the follow-up literature; see
///   Klein and Blaho (1987), "Dexterity Measures for the Design and Control of
///   Kinematically Redundant Manipulators", IJRR 6(2), which compares it
///   against the determinant and the condition number. sigma_min is the
///   pessimistic member of that family, which is what suits a worst-case
///   aggregate.
///
/// WHY std::optional. An empty trajectory has no worst moment, and there is no
/// double that says so. Returning 0.0 would make "the rollout was empty"
/// indistinguishable from "the rollout was fully singular" -- the two most
/// opposite conclusions this metric can support, collapsed onto one value. A
/// caller aggregating over many rollouts would silently count every empty one
/// as a catastrophic failure.
///
/// Note this is the opposite call from kUnbounded in urdf.hpp, deliberately.
/// There, infinity was right because every use of a joint limit is an
/// arithmetic comparison and infinity makes those comparisons come out
/// correctly with no branch. Here there is no comparison that behaves
/// correctly on a sentinel: any threshold test against 0.0 gives the wrong
/// answer for empty input. The rule is not "prefer sentinels" or "prefer
/// optional" -- it is whether the absent case has a value that behaves like
/// the real ones.
///
/// Pre:  every entry of traj has length robot.numDofs().
/// Post: std::nullopt when traj is empty; otherwise a value >= 0.
std::optional<double> dexterityMargin(const Robot& robot, const std::vector<Eigen::VectorXd>& traj);

}  // namespace robometrics
