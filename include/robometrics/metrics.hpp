#pragma once

#include <Eigen/Core>
#include <optional>
#include <vector>

#include "robometrics/urdf.hpp"

/// Kinematic quality metrics over recorded rollouts. Everything here answers
/// one question: at a given configuration, how freely can the end-effector
/// move? Not whether it is in a good place, not whether the task succeeded.
namespace robometrics {

/// Smallest singular value of the TRANSLATIONAL rows of the Jacobian.
///
/// Take every joint-velocity vector of unit length and collect the tip
/// velocities it produces. That set is an ellipsoid whose semi-axes are the
/// singular values of J_v, so sigma_min is its SHORTEST semi-axis: metres per
/// second of tip speed per unit of joint speed, in the worst direction.
/// sigma_min == 0 means that direction is locked -- no combination of joint
/// velocities produces tip motion along it, at any speed.
///
/// WHY ONLY topRows(3). Two reasons, and the second is the one that bites.
///
/// Units: the translational rows carry metres per radian, the rotational rows
/// of a revolute joint are a unit axis vector and are dimensionless. Singular
/// values of a matrix whose rows are in different units are meaningless --
/// switching metres to millimetres would change the answer arbitrarily.
///
/// Worse, stacking rows can only ever INCREASE the smallest singular value:
/// ||M*x||^2 == ||A*x||^2 + ||B*x||^2 for M = [A; B], so the rotational rows
/// fill in precisely the directions where the translational block is deficient
/// and a translational singularity stops being visible. On the planar_arm
/// fixture with the arm fully stretched -- a textbook singularity, the tip
/// demonstrably cannot move radially:
///
///     sigma(J_v,    3 x 2) = 0.6708,  0.0000     <- singular, plainly
///     sigma(J_full, 6 x 2) = 1.5533,  0.1931     <- looks perfectly healthy
///
/// A threshold on the full Jacobian would have called that pose fine.
///
/// WHY s(k - 1) AND NOT s.tail(1). With Eigen's JacobiSVD the two are the same
/// expression: singularValues() already returns exactly min(rows, cols)
/// entries and pads with nothing. The explicit min is defensive, naming which
/// value is meant so that a switch to eigenvalues of Jv * Jv^T, or to a backend
/// that returns a full-length vector, cannot silently start reading a
/// structural zero.
///
/// A robot with fewer than three joints cannot span R^3 at all, so its
/// ellipsoid is a flat disc or a segment in every configuration and sigma_min
/// is the shortest axis of a lower-dimensional ellipsoid, not a distance to
/// singularity in R^3. Still a valid relative signal along one robot's
/// trajectory; not comparable across robots.
///
/// LIMITATION: for an arm mixing revolute and prismatic joints the columns of
/// J_v carry different units, so the number is only dimensionally clean for a
/// uniform-joint-type arm.
///
/// Pre:  J has at least 3 rows and at least 1 column.
/// Post: result >= 0, and equals 0 exactly when J_v is rank deficient.
double sigmaMinTranslation(const Eigen::MatrixXd& J);

/// sigmaMinTranslation at every point of a trajectory.
///
/// The raw signal; the aggregate below is one summary of it. WHERE a rollout
/// came close to a singularity is usually more informative than how close it
/// got -- a dip at the approach means something different from one during the
/// grasp.
///
/// Pre:  every entry of traj has length robot.numDofs().
/// Post: result.size() == traj.size().
std::vector<double> sigmaMinProfile(const Robot& robot, const std::vector<Eigen::VectorXd>& traj);

/// Worst dexterity along a whole trajectory: the minimum of sigmaMinProfile.
///
/// The minimum and not the mean. A single pass through a singularity makes the
/// whole trajectory fragile -- the controller needs unbounded joint velocity to
/// keep tracking and any inverse-kinematics step there is ill-conditioned --
/// and the rollout does not recover from that by being well-conditioned before
/// and after. One bad frame in two hundred moves a mean by half a percent, so a
/// trajectory that briefly locks up would score almost identically to one that
/// never does.
///
/// DEXTERITY, not safety and not mobility. The metric knows nothing about
/// contact forces, obstacles or payload -- a robot can be maximally dexterous
/// and still crush the glass it is holding. And mobility is a property of the
/// MECHANISM, while this is a property of one CONFIGURATION: a seven-joint arm
/// is highly mobile and can still be completely inept in a specific pose.
///
/// The manipulability ellipsoid this is read off comes from Yoshikawa (1985),
/// "Manipulability of Robotic Mechanisms", IJRR 4(2), whose own measure was the
/// ellipsoid's volume. Taking the shortest semi-axis instead is from the
/// follow-up literature; see Klein and Blaho (1987), IJRR 6(2).
///
/// std::optional because an empty trajectory has no worst moment and there is
/// no double that says so. Returning 0.0 would make "the rollout was empty"
/// indistinguishable from "the rollout was fully singular" -- the two most
/// opposite conclusions this metric supports. Contrast kUnbounded in urdf.hpp,
/// where infinity IS right: the rule is whether the absent case has a value
/// that behaves like the real ones.
///
/// Pre:  every entry of traj has length robot.numDofs().
/// Post: std::nullopt when traj is empty; otherwise a value >= 0.
std::optional<double> dexterityMargin(const Robot& robot, const std::vector<Eigen::VectorXd>& traj);

/// Path efficiency: how much of the joint motion in a rollout actually moved
/// the end-effector.
///
///     E = optimal cost / actual cost,   in (0, 1]
///
/// summed over the trajectory before dividing, with
///
///     actual cost  = ||q[i] - q[i-1]||
///     optimal cost = ||J^+ * dx||
///
/// where dx is the twist the tip travelled and J^+ is the pseudoinverse of the
/// Jacobian. J^+ * dx is the smallest joint motion that would have produced the
/// same tip displacement, so E == 0.5 means half the joint motion cancelled
/// itself out and the tip never saw it. On a redundant arm that waste is
/// physical: the joints can move along the Jacobian's null space and the tip
/// does not move at all.
///
/// READ THIS BEFORE TRUSTING A 1.0.
///
/// (1) A NON-REDUNDANT ROBOT ALWAYS SCORES 1. The condition for this metric to
///     say anything is numDofs() > rank(J). Without a null space, J*dq = dx has
///     a unique solution, the minimum-norm solution IS the actual one, and the
///     ratio is 1 for every trajectory however clumsy. So it says something
///     about a 7-DOF Franka and NOTHING about a 6-DOF UR.
///
///     Count rank(J), not joints. rank(J) is bounded by the number of twist
///     components the mechanism can produce, which can be well below 6: a
///     PLANAR arm drives only vx, vy and omega_z, so rank(J) <= 3 however many
///     joints it has. That makes a planar 3R non-redundant and a planar 4R the
///     smallest planar arm this metric can say anything about.
///
/// (2) THE OPTIMUM IS RELATIVE TO THIS CARTESIAN PATH. J^+ * dx is the cheapest
///     way to follow the path the tip ACTUALLY took, not the cheapest way to get
///     from start to end. A policy that drags the tip in a wide arc where a
///     straight line would do can score a perfect 1.0. This measures
///     joint-space waste, not task-space waste.
///
/// (3) DIMENSIONAL TRAP, KNOWN AND NOT FIXED. ||q[i] - q[i-1]|| adds radians and
///     metres in quadrature whenever the robot mixes revolute and prismatic
///     joints. Numerator and denominator carry the same defect so the ratio is
///     less wrong than either half, but it is not right, and it is not
///     comparable between robots of different joint composition. The fix is a
///     weighting matrix W and norms ||W * dq||, which changes what the metric
///     MEANS rather than fixing a bug.
///
/// (4) dx is a FINITE DIFFERENCE between two recorded poses; J is a DERIVATIVE.
///     Pairing them is a quadrature rule, so the result carries a
///     discretisation error and E can exceed 1 slightly. Both the Jacobian and
///     the frame conversion are sampled at the MIDPOINT of the step, which is
///     the midpoint rule: first-order terms cancel and the residual is O(h^2).
///     Measured on planar_3r, where E must be exactly 1 so any deviation is
///     pure discretisation error:
///
///         steps      endpoint        midpoint
///            50      1.2e-02         1.0e-06
///           200      2.9e-03         6.2e-08
///           800      7.2e-04         3.9e-09
///         order         1.00            2.00
///
///     Both parts must move together. Putting only the Jacobian at the midpoint
///     and leaving the frame conversion at the start is WORSE than leaving both
///     at the start (8.4e-02 at 50 steps), because the two first-order terms no
///     longer partly cancel.
///
/// Pre:  every entry of traj has length robot.numDofs().
/// Post: std::nullopt when traj has fewer than two points, or when the robot
///       never moved; otherwise a value > 0.
///
/// "Never moved" is tested exactly, not against a threshold: any epsilon would
/// have to be in the mixed units of (3), so there is no defensible value.
std::optional<double> pathEfficiency(const Robot& robot, const std::vector<Eigen::VectorXd>& traj);

}  // namespace robometrics
