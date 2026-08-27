#pragma once

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "robometrics/se3.hpp"

/// URDF loading, reduced to what forward kinematics actually needs.
///
/// This module reads geometry and topology only. Inertias, visual meshes,
/// collision geometry, materials, gazebo tags and transmissions are skipped
/// without complaint -- the library evaluates recorded rollouts, it never
/// simulates dynamics, so carrying that data would be dead weight.
///
/// FRAME CONVENTIONS, stated once and relied on everywhere below.
///
/// URDF defines every joint by three things: an <origin> that places the joint
/// frame relative to the parent link, an <axis> expressed IN that joint frame,
/// and a joint value q. The child link frame is then
///
///     X_child = X_parent * origin * motion(q)
///
/// where motion(q) is a rotation about the axis (revolute/continuous) or a
/// translation along it (prismatic). Note the axis is fixed in the joint frame,
/// so it moves with the parent -- it is NOT a world-frame axis.
///
/// <origin rpy="r p y"> is fixed-axis roll-pitch-yaw, which composes as
///
///     R = Rz(yaw) * Ry(pitch) * Rx(roll)
///
/// i.e. apply roll about the FIXED x first, then pitch about the FIXED y, then
/// yaw about the FIXED z. This is the same matrix as the intrinsic ZYX
/// convention, which is why both descriptions appear in the literature and why
/// it is worth writing down which one this file means: the numerically
/// identical product above. Reversing it to Rx*Ry*Rz compiles, parses every
/// real URDF without error, and silently produces a different robot.
namespace robometrics {

/// Joint types this library accepts.
///
/// URDF also defines `planar` and `floating`. Both are rejected at parse time
/// rather than approximated: they are multi-DOF, so they would break the
/// one-scalar-per-joint contract that the whole q vector rests on. Failing
/// loudly is the only honest option -- silently dropping them would produce a
/// robot that parses fine and computes the wrong end-effector pose.
enum class JointType {
  Revolute,    ///< rotation about `axis`, bounded by [lowerLimit, upperLimit]
  Continuous,  ///< rotation about `axis`, unbounded (a wheel, a wrist roll)
  Prismatic,   ///< translation along `axis`, bounded
  Fixed,       ///< no DOF; see the folding note on Robot, these never reach the joint array
};

/// Human-readable name, for diagnostics and error messages.
///
/// Deliberately NOT called toString(). doctest picks up an ADL-visible
/// `toString(T)` in the type's own namespace as its stringification hook and
/// requires it to return doctest::String; a `const char*` overload with that
/// name makes every CHECK involving a JointType fail to compile, in the test
/// framework's headers rather than ours. A specific name keeps the type free of
/// that entanglement.
const char* jointTypeName(JointType type);

/// Sentinel for "this joint is not bounded in this direction".
///
/// Infinity rather than std::optional on purpose: every downstream use of a
/// limit is an arithmetic comparison (`q < lowerLimit`), and infinity makes
/// those comparisons do the right thing with no branch. An optional would push
/// a `has_value()` check into every call site, and the one thing it buys --
/// distinguishing "absent" from "unbounded" -- is a distinction this library
/// never needs to make.
inline constexpr double kUnbounded = std::numeric_limits<double>::infinity();

/// One movable degree of freedom.
///
/// INVARIANT: `type` is never JointType::Fixed. Fixed joints exist in the URDF
/// but are folded away during parsing (see Robot).
struct Joint {
  std::string name;
  JointType type = JointType::Revolute;

  /// Transform from the frame of joint `parentJoint` to this joint's frame,
  /// with any intervening fixed joints already multiplied in.
  ///
  /// This is NOT the raw <origin> of the URDF element when fixed joints were
  /// collapsed -- it is the product of that origin with every fixed transform
  /// between this joint and the previous movable one. When `parentJoint` is -1
  /// it is measured from the root link frame.
  SE3 originTransform;

  /// Motion axis in this joint's own frame, normalised to unit length.
  ///
  /// Normalising is a deliberate change to the input: the URDF spec says the
  /// axis should already be a unit vector, but real files carry values like
  /// "0 0 1.0000001" or unnormalised integer triples. Without normalisation the
  /// joint would rotate by ||axis|| * q instead of q, which is a scale error on
  /// every downstream metric and is invisible in any test that uses clean axes.
  /// A zero-length axis is a parse error, not something to normalise.
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
  /// directly off the root link.
  ///
  /// This is the field forward kinematics actually walks. Because joints are
  /// stored in topological order, `parentJoint < ` own index always holds, so a
  /// single forward pass computes every pose with no recursion and no
  /// bookkeeping.
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
  /// Grippers are full of them -- a Robotiq 2F-85 has five, all driven by one
  /// actuator, and a Panda's two finger joints are frequently coupled this way.
  ///
  /// Why they are supported rather than rejected. A mimic joint is genuinely
  /// movable, so it cannot be folded away like a fixed joint, but it is not an
  /// independent input either. The two wrong answers are both tempting:
  ///   - Rejecting the file outright means no real URDF with a gripper loads,
  ///     even when the frame of interest is on the arm and the fingers are
  ///     irrelevant.
  ///   - Ignoring the <mimic> tag and treating the joint as independent makes
  ///     q longer than the robot's actual DOF count. Recorded rollouts then no
  ///     longer line up with q index by index, and every downstream metric is
  ///     silently computed for a different configuration than the one that was
  ///     recorded. Nothing throws; the numbers are just wrong.
  /// Modelling them explicitly costs one extra index and keeps q the same
  /// length as the actuator vector in the data.
  ///
  /// Restriction: the source must itself be independent. A mimic of a mimic is
  /// rejected at parse time. Chains are legal URDF and resolving them is not
  /// hard, but it turns a single resolution pass into a fixed-point iteration
  /// with a cycle check, and no robot in scope for this library needs it. The
  /// error message says exactly this, so the day one shows up the fix is a
  /// known quantity rather than a surprise.
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

/// A named frame on the robot.
///
/// Links matter here purely as frames to report poses for -- a link's mass and
/// meshes are irrelevant to kinematics. Keeping every link (including the ones
/// that only existed to hang a fixed joint off) is what makes it possible to
/// ask for the pose of a tool tip, a camera mount, or a grasp target, all of
/// which are attached by fixed joints in real URDFs.
struct Link {
  std::string name;

  /// Nearest movable joint at or above this link, or -1 when the link is rigid
  /// with respect to the root.
  int supportingJoint = -1;

  /// This link's frame expressed in the frame of `supportingJoint`. Identity
  /// when the link IS that joint's child link; a fixed transform otherwise.
  ///
  /// This pair (supportingJoint, offset) is what lets fixed joints be folded
  /// without losing anything. A trailing fixed chain -- hand -> tool flange ->
  /// grasp target, the usual shape at the end of a manipulator -- has no
  /// following movable joint to fold into, so folding forward alone would
  /// silently drop it and put the end-effector in the wrong place.
  SE3 offset;
};

/// Thrown for every malformed or unsupported URDF.
///
/// Carries the element it failed on separately from the explanation so that a
/// caller can act on it, and so the message is forced to name a location.
/// A bare `runtime_error("parse error")` on a 3000-line URDF is not a
/// diagnostic, it is a dare.
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
///   1. Read every <joint> and <link>. Reject planar/floating joints and any
///      unknown type string.
///   2. Build the tree and find the root -- the single link that never appears
///      as a child. Zero roots means a cycle, more than one means a forest;
///      both are errors.
///   3. Topologically sort so that every joint appears after its parent. A
///      URDF file may list joints in any order, and real exporters do.
///   4. Fold fixed joints away: their transforms are accumulated into the
///      `originTransform` of the next movable joint downstream, and into the
///      `offset` of every link they carry. Nothing is discarded.
///   5. Record, for each link, which movable joint supports it.
///   6. Resolve <mimic> references and assign configuration-vector indices to
///      the independent joints, in joint order.
///
/// After that, `numJoints()` counts movable joints only. Two counts exist and
/// they differ exactly when the robot has mimic joints:
///
///   numJoints() -- entries in the joint array; every one of them moves
///   numDofs()   -- length of q; independent joints only
///
/// For a robot without mimic joints they are equal and `joint(i).dofIndex == i`
/// throughout, which is the case worth keeping in mind while reading the rest
/// of this header.
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
