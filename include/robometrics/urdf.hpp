#pragma once

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "robometrics/se3.hpp"

/// URDF loading, reduced to what forward kinematics needs. Geometry and
/// topology only -- inertias, meshes, collision geometry, materials and gazebo
/// tags are skipped.
///
/// FRAME CONVENTIONS, relied on everywhere below.
///
/// A joint is an <origin> placing its frame relative to the parent link, an
/// <axis> expressed IN that joint frame, and a value q:
///
///     X_child = X_parent * origin * motion(q)
///
/// The axis is fixed in the joint frame and moves with the parent -- it is NOT
/// a world-frame axis, which is why motion multiplies from the right.
///
/// <origin rpy="r p y"> is fixed-axis roll-pitch-yaw:
///
///     R = Rz(yaw) * Ry(pitch) * Rx(roll)
///
/// Roll about the FIXED x first, then pitch about the FIXED y, then yaw about
/// the FIXED z. The same matrix comes out of the intrinsic ZYX convention,
/// which is why both descriptions circulate. Reversing it to Rx*Ry*Rz parses
/// every real URDF without error and describes a different robot.
namespace robometrics {

/// Joint types this library accepts.
///
/// URDF also defines `planar` and `floating`. Both are rejected at parse time:
/// they are multi-DOF and would break the one-scalar-per-joint contract the
/// whole q vector rests on. Silently dropping them would produce a robot that
/// parses fine and computes the wrong end-effector pose.
enum class JointType {
  Revolute,    ///< rotation about `axis`, bounded by [lowerLimit, upperLimit]
  Continuous,  ///< rotation about `axis`, unbounded (a wheel, a wrist roll)
  Prismatic,   ///< translation along `axis`, bounded
  Fixed,       ///< no DOF; see the folding note on Robot, these never reach the joint array
};

/// Human-readable name, for diagnostics and error messages.
///
/// Deliberately NOT called toString(): doctest treats an ADL-visible
/// `toString(T)` as its stringification hook and requires doctest::String back,
/// so that name would break every CHECK involving a JointType -- inside the
/// test framework's headers, not ours.
const char* jointTypeName(JointType type);

/// Sentinel for "this joint is not bounded in this direction".
///
/// Infinity rather than std::optional: every use of a limit is an arithmetic
/// comparison (`q < lowerLimit`), and infinity makes those come out right with
/// no branch. Compare pathEfficiency's optional in metrics.hpp -- the rule is
/// whether the absent case has a value that behaves like the real ones.
inline constexpr double kUnbounded = std::numeric_limits<double>::infinity();

/// One movable degree of freedom.
///
/// INVARIANT: `type` is never JointType::Fixed. Fixed joints exist in the URDF
/// but are folded away during parsing (see Robot).
struct Joint {
  std::string name;
  JointType type = JointType::Revolute;

  /// Transform from the frame of joint `parentJoint` to this joint's frame,
  /// with any intervening fixed joints already multiplied in -- so NOT the raw
  /// <origin> when fixed joints were collapsed. Measured from the root link
  /// frame when `parentJoint` is -1.
  SE3 originTransform;

  /// Motion axis in this joint's own frame, normalised to unit length.
  ///
  /// Normalising is a deliberate change to the input. Real files carry values
  /// like "0 0 1.0000001"; without it the joint rotates by ||axis|| * q instead
  /// of q -- a scale error on every downstream metric, invisible in any test
  /// that uses clean axes. A zero-length axis is a parse error.
  Vec3 axis = Vec3::UnitZ();

  /// Position limits in radians (revolute) or metres (prismatic).
  /// Both are +/-kUnbounded for Continuous joints.
  double lowerLimit = -kUnbounded;
  double upperLimit = kUnbounded;

  /// Velocity and effort ceilings, kUnbounded when the URDF omits them.
  /// Parsed and stored but never consulted by this library; they are here
  /// because rollout evaluation wants to report limit violations.
  double velocityLimit = kUnbounded;
  double effortLimit = kUnbounded;

  /// Index of the nearest preceding MOVABLE joint, or -1 if this joint hangs
  /// directly off the root link. This is what forward kinematics walks; joints
  /// are topologically sorted, so parentJoint is always a smaller index and one
  /// forward pass suffices.
  int parentJoint = -1;

  /// Position of this joint's value inside the configuration vector q, or -1
  /// when the joint is a mimic and its value is derived instead of supplied.
  ///
  /// For a robot with no mimic joints this is simply the joint's own index, and
  /// the distinction below can be ignored entirely.
  int dofIndex = -1;

  /// Index of the joint this one mimics, or -1 when independent.
  ///
  /// URDF's <mimic> makes a joint a rigid function of another:
  ///
  ///     q_this = mimicMultiplier * q_source + mimicOffset
  ///
  /// Grippers are full of them. They are modelled rather than rejected because
  /// both alternatives are wrong: rejecting means no real URDF with a gripper
  /// loads, and ignoring the tag makes q longer than the robot's actual DOF
  /// count, so recorded rollouts stop lining up index by index and every metric
  /// is silently computed for a different configuration. Nothing throws; the
  /// numbers are just wrong.
  ///
  /// Restriction: the source must itself be independent. A mimic of a mimic is
  /// rejected at parse time -- chains are legal URDF but turn a single
  /// resolution pass into a fixed-point iteration with a cycle check, and no
  /// robot in scope needs it. The error message says exactly that.
  int mimicSource = -1;
  double mimicMultiplier = 1.0;
  double mimicOffset = 0.0;

  /// Convenience predicate; equivalent to mimicSource >= 0.
  bool isMimic() const { return mimicSource >= 0; }

  /// Indices into Robot's link table, for diagnostics and for locating frames.
  /// `parentLink` is the link the URDF names as parent; note it may not be the
  /// child link of `parentJoint` when fixed joints were folded in between.
  int parentLink = -1;
  int childLink = -1;
};

/// A named frame on the robot. Every link is kept, including ones that only
/// exist to hang a fixed joint off, because tool tips, camera mounts and grasp
/// targets are all attached that way in real URDFs.
struct Link {
  std::string name;

  /// Nearest movable joint at or above this link, or -1 when the link is rigid
  /// with respect to the root.
  int supportingJoint = -1;

  /// This link's frame expressed in the frame of `supportingJoint`. Identity
  /// when the link IS that joint's child link; a fixed transform otherwise.
  ///
  /// This pair is what lets fixed joints be folded without losing anything. A
  /// trailing fixed chain -- hand -> flange -> grasp target, the usual shape at
  /// the end of a manipulator -- has no following movable joint to fold into,
  /// so folding forward alone would silently drop it and put the end-effector
  /// in the wrong place.
  SE3 offset;
};

/// Thrown for every malformed or unsupported URDF.
///
/// Carries the element separately from the explanation so that no message can
/// be written without naming a location. On a 3000-line URDF, "parse error" is
/// not a diagnostic.
///
/// Message format:  "joint 'panda_joint4' <axis>: attribute xyz is missing"
class UrdfError : public std::runtime_error {
public:
  UrdfError(std::string where, std::string what);

  /// The element that failed, e.g. "joint 'panda_joint4' <axis>".
  const std::string& where() const;

  /// The explanation alone, without the location prefix.
  const std::string& detail() const;

private:
  std::string where_;
  std::string detail_;
};

namespace detail {
/// Internal construction hook, befriended by Robot below.
///
/// It exists so that the parser can assemble a Robot from an already-validated
/// tree while Robot's own state stays private and pugixml stays out of this
/// header entirely. pugixml is a PRIVATE dependency of the library target, so a
/// caller who includes this file must not need it on their include path.
struct RobotAccess;
}  // namespace detail

/// A parsed kinematic tree with one designated end-effector.
///
/// WHAT PARSING DOES TO THE FILE, in order:
///
///   1. Read every <joint> and <link>. Reject planar/floating and unknown types.
///   2. Find the root -- the single link that never appears as a child. Zero
///      roots means a cycle, more than one means a forest; both are errors.
///   3. Topologically sort so every joint appears after its parent. A URDF may
///      list joints in any order, and real exporters do.
///   4. Fold fixed joints away: their transforms accumulate into the
///      `originTransform` of the next movable joint downstream, and into the
///      `offset` of every link they carry. Nothing is discarded.
///   5. Record, for each link, which movable joint supports it.
///   6. Resolve <mimic> references and assign configuration-vector indices.
///
/// Two counts exist and they differ exactly when the robot has mimic joints:
///
///   numJoints() -- entries in the joint array; every one of them moves
///   numDofs()   -- length of q; independent joints only
///
/// For a robot without mimic joints they are equal and joint(i).dofIndex == i.
///
/// This is a value type: copying it copies the tree. Robots are parsed once and
/// evaluated over thousands of rollout frames, so construction cost is
/// irrelevant and shared ownership would only add lifetime questions.
class Robot {
public:
  /// Parse a URDF file. The end-effector is auto-detected: if the tree has
  /// exactly one leaf link, that is the tip. If it has several -- which is the
  /// normal case for anything with a gripper -- this throws and the message
  /// lists the candidates, so the fix is obvious from the error alone.
  ///
  /// Pre:  path names a readable file containing a single <robot> element.
  /// Post: numJoints() >= 0; joints are topologically sorted from the root.
  static Robot fromUrdfFile(const std::string& path);

  /// Same, with the end-effector named explicitly. Use this whenever the robot
  /// has a gripper, or whenever the frame you want to track is a fixed frame
  /// hanging off the last link.
  ///
  /// Pre: tipLink names a link present in the file.
  static Robot fromUrdfFile(const std::string& path, const std::string& tipLink);

  /// In-memory variants, for tests and for URDFs assembled at runtime.
  static Robot fromUrdfString(const std::string& xml);
  static Robot fromUrdfString(const std::string& xml, const std::string& tipLink);

  /// Name from the <robot name="..."> attribute.
  const std::string& name() const;

  /// Number of MOVABLE joints. Fixed joints are not counted; they no longer
  /// exist as joints at all. Mimic joints ARE counted -- they move.
  int numJoints() const;

  /// Required length of the configuration vector q: independent joints only.
  /// Equals numJoints() unless the robot has mimic joints.
  int numDofs() const;

  /// Pre: 0 <= i < numJoints(). Not bounds-checked -- see the note below.
  const Joint& joint(int i) const;

  int numLinks() const;

  /// Pre: 0 <= i < numLinks().
  const Link& link(int i) const;

  /// Index of the root link. Its pose is the identity by definition; everything
  /// this library reports is relative to it.
  int rootLinkIndex() const;

  /// Index of the designated end-effector link.
  int tipLinkIndex() const;

  /// Name lookup. Returns -1 when the name is unknown, rather than throwing:
  /// asking whether a frame exists is a legitimate question, and the caller
  /// that wants an exception can compare against -1 and produce a better
  /// message than this class could.
  int findJoint(const std::string& jointName) const;
  int findLink(const std::string& linkName) const;

private:
  friend struct detail::RobotAccess;

  Robot() = default;

  std::string name_;
  std::vector<Joint> joints_;
  std::vector<Link> links_;
  int rootLink_ = -1;
  int tipLink_ = -1;
  int numDofs_ = 0;
};

}  // namespace robometrics
