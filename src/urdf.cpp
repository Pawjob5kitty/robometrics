#include "robometrics/urdf.hpp"

#include <pugixml.hpp>

#include <cmath>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

// Parsing runs in two stages, and the split is the main structural idea in this
// file.
//
//   Stage one (RawJoint, parseJoint): a literal transcription of the XML, with
//   link and joint references still stored as names. Every attribute-level
//   error -- a missing axis, a non-numeric limit, an unknown joint type -- is
//   reported here, while the offending element is still in hand.
//
//   Stage two (buildRobot): names become indices, the tree is walked, and fixed
//   joints are folded. Every error here is topological -- two parents, a cycle,
//   an unreachable link.
//
// Keeping them apart is what makes the error messages specific. A single-pass
// parser has to report "something is wrong with this robot" because by the time
// it notices, it no longer knows which element it came from.

namespace robometrics {
namespace {

// ---------------------------------------------------------------------------
// Error reporting
// ---------------------------------------------------------------------------
// Every throw goes through fail(), which requires a location. That is a
// constraint on this file rather than a convenience: the failure mode being
// designed against is a 3000-line URDF and a message that says "parse error".

[[noreturn]] void fail(std::string where, std::string detail) {
  throw UrdfError(std::move(where), std::move(detail));
}

std::string jointWhere(const std::string& name) { return "joint '" + name + "'"; }

std::string jointWhere(const std::string& name, const char* element) {
  return "joint '" + name + "' <" + element + ">";
}

// ---------------------------------------------------------------------------
// Attribute parsing
// ---------------------------------------------------------------------------

// Reads exactly one double. std::stod would accept "1.0abc" and "0x10"; an
// istringstream plus an explicit end-of-input check rejects anything that is
// not precisely one number.
//
// The out-parameter rather than std::optional keeps the caller's error context
// where it belongs -- the caller knows which attribute of which element this
// was, and can say so.
bool parseDouble(const std::string& text, double& out) {
  std::istringstream in(text);
  in >> out;
  if (in.fail()) {
    return false;
  }
  in >> std::ws;  // skip trailing whitespace, then insist there is nothing left
  return in.eof();
}

// URDF writes vectors as three space-separated numbers: xyz="0 0 1".
bool parseVec3(const std::string& text, Vec3& out) {
  std::istringstream in(text);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  in >> x >> y >> z;
  if (in.fail()) {
    return false;
  }
  in >> std::ws;
  if (!in.eof()) {
    return false;  // a fourth number is a copy-paste accident, not an intent
  }
  out = Vec3(x, y, z);
  return true;
}

double requireDoubleAttr(const pugi::xml_node& node, const char* attrName,
                         const std::string& where) {
  const pugi::xml_attribute attr = node.attribute(attrName);
  if (!attr) {
    fail(where, std::string("required attribute '") + attrName + "' is missing");
  }
  double value = 0.0;
  if (!parseDouble(attr.value(), value)) {
    fail(where,
         std::string("attribute '") + attrName + "' is not a number: '" + attr.value() + "'");
  }
  if (!std::isfinite(value)) {
    fail(where, std::string("attribute '") + attrName + "' is not finite: '" + attr.value() + "'");
  }
  return value;
}

double optionalDoubleAttr(const pugi::xml_node& node, const char* attrName, double fallback,
                          const std::string& where) {
  if (!node.attribute(attrName)) {
    return fallback;
  }
  return requireDoubleAttr(node, attrName, where);
}

Vec3 requireVec3Attr(const pugi::xml_node& node, const char* attrName, const std::string& where) {
  const pugi::xml_attribute attr = node.attribute(attrName);
  if (!attr) {
    fail(where, std::string("required attribute '") + attrName + "' is missing");
  }
  Vec3 value = Vec3::Zero();
  if (!parseVec3(attr.value(), value)) {
    fail(where, std::string("attribute '") + attrName +
                    "' is not three numbers separated by spaces: '" + attr.value() + "'");
  }
  if (!value.allFinite()) {
    fail(where, std::string("attribute '") + attrName + "' has a non-finite component: '" +
                    attr.value() + "'");
  }
  return value;
}

// ---------------------------------------------------------------------------
// <origin>
// ---------------------------------------------------------------------------

// Fixed-axis roll-pitch-yaw, per the URDF specification:
//
//     R = Rz(yaw) * Ry(pitch) * Rx(roll)
//
// Read right to left as the order of application: roll about the FIXED x axis
// first, then pitch about the FIXED y, then yaw about the FIXED z. The same
// matrix comes out of the intrinsic ZYX convention (rotate about z, then about
// the NEW y, then the NEW x), which is why both descriptions of "rpy"
// circulate in the literature. Those two agree. What does not agree is
// Rx*Ry*Rz, which parses every real URDF without complaint and describes a
// different robot.
//
// Worked example, so the order can be checked by hand. For rpy = (pi/2, pi/2, 0):
//
//   Rx(pi/2) = | 1  0  0 |        Ry(pi/2) = |  0  0  1 |
//              | 0  0 -1 |                   |  0  1  0 |
//              | 0  1  0 |                   | -1  0  0 |
//
//   Rz(0)*Ry*Rx = |  0  1  0 |        but    Rx*Ry*Rz = | 0  0  1 |
//                 |  0  0 -1 |                          | 1  0  0 |
//                 | -1  0  0 |                          | 0  1  0 |
//
// Same three angles, different matrices. That pair is what the rpy tests pin
// down; a test using only rpy = (0, 0, something) cannot tell the two apart,
// because with two of the three angles zero the product order does not matter.
//
// Composing three rodrigues() calls instead of writing out nine entries keeps
// one source of truth for what a rotation is. If the handedness convention in
// rodrigues() were ever wrong, this function would be wrong in the same
// direction rather than silently disagreeing with the rest of the library.
Mat3 rpyToMatrix(const Vec3& rpy) {
  const Mat3 rx = rodrigues(Vec3::UnitX() * rpy.x());
  const Mat3 ry = rodrigues(Vec3::UnitY() * rpy.y());
  const Mat3 rz = rodrigues(Vec3::UnitZ() * rpy.z());
  return rz * ry * rx;
}

// <origin> is optional wherever it appears, and so is each of its attributes.
// Absent means identity in both cases; that default is the spec's.
SE3 parseOrigin(const pugi::xml_node& parent, const std::string& where) {
  const pugi::xml_node origin = parent.child("origin");
  if (!origin) {
    return SE3::identity();
  }
  const std::string originWhere = where + " <origin>";

  Vec3 xyz = Vec3::Zero();
  if (origin.attribute("xyz")) {
    xyz = requireVec3Attr(origin, "xyz", originWhere);
  }
  Vec3 rpy = Vec3::Zero();
  if (origin.attribute("rpy")) {
    rpy = requireVec3Attr(origin, "rpy", originWhere);
  }
  return SE3(rpyToMatrix(rpy), xyz);
}

// ---------------------------------------------------------------------------
// Stage one: the file as written
// ---------------------------------------------------------------------------

bool parseJointType(const std::string& text, JointType& out) {
  if (text == "revolute") {
    out = JointType::Revolute;
  } else if (text == "continuous") {
    out = JointType::Continuous;
  } else if (text == "prismatic") {
    out = JointType::Prismatic;
  } else if (text == "fixed") {
    out = JointType::Fixed;
  } else {
    return false;
  }
  return true;
}

bool isMovable(JointType type) { return type != JointType::Fixed; }

struct RawJoint {
  std::string name;
  JointType type = JointType::Fixed;
  std::string parentLink;
  std::string childLink;
  SE3 origin;
  Vec3 axis = Vec3::UnitZ();
  double lowerLimit = -kUnbounded;
  double upperLimit = kUnbounded;
  double velocityLimit = kUnbounded;
  double effortLimit = kUnbounded;
  std::string mimicSource;  // empty when the joint is independent
  double mimicMultiplier = 1.0;
  double mimicOffset = 0.0;
};

void parseAxis(const pugi::xml_node& node, RawJoint& raw) {
  const pugi::xml_node axis = node.child("axis");

  // The spec gives <axis> a default of "1 0 0" when omitted. This parser
  // deliberately requires it instead. A missing axis is almost always an
  // authoring mistake, and quietly rotating about x produces a robot that looks
  // plausible and is wrong -- the worst possible failure mode for a library
  // whose entire output is geometry. Fixed joints have no axis and never reach
  // this function.
  if (!axis) {
    fail(jointWhere(raw.name),
         "a " + std::string(jointTypeName(raw.type)) +
             " joint requires an <axis> element (this parser does not apply the URDF "
             "default of '1 0 0')");
  }

  const Vec3 rawAxis = requireVec3Attr(axis, "xyz", jointWhere(raw.name, "axis"));
  const double norm = rawAxis.norm();

  // A zero axis is not a case to normalise; it states that the joint moves
  // along no direction at all, which is meaningless for every joint type.
  if (norm < 1e-12) {
    fail(jointWhere(raw.name, "axis"), "axis has zero length; it must specify a direction");
  }
  raw.axis = rawAxis / norm;
}

void parseLimits(const pugi::xml_node& node, RawJoint& raw) {
  const pugi::xml_node limit = node.child("limit");
  const std::string where = jointWhere(raw.name, "limit");

  // Continuous joints are unbounded by definition, so <limit> is optional for
  // them; revolute and prismatic joints are bounded by definition, so it is
  // required. That split comes from the spec, it is not a house rule.
  if (!limit) {
    if (raw.type == JointType::Continuous) {
      return;
    }
    fail(jointWhere(raw.name), "a " + std::string(jointTypeName(raw.type)) +
                                   " joint requires a <limit> element with 'lower' and 'upper'");
  }

  if (raw.type == JointType::Continuous) {
    // Present but contradictory: a continuous joint carrying lower/upper is
    // arguing with itself. Rejecting the file would be harsh, since exporters
    // emit this, but the bounds must not be stored -- keeping them would make
    // an unbounded joint report as limited.
    raw.lowerLimit = -kUnbounded;
    raw.upperLimit = kUnbounded;
  } else {
    raw.lowerLimit = requireDoubleAttr(limit, "lower", where);
    raw.upperLimit = requireDoubleAttr(limit, "upper", where);
    if (raw.lowerLimit > raw.upperLimit) {
      std::ostringstream msg;
      msg << "lower limit " << raw.lowerLimit << " is greater than upper limit " << raw.upperLimit;
      fail(where, msg.str());
    }
  }

  // 'effort' and 'velocity' are required by the spec but irrelevant to
  // kinematics, and hand-written URDFs routinely omit them. Rejecting a file
  // over a number this library never reads would be pedantry.
  raw.velocityLimit = optionalDoubleAttr(limit, "velocity", kUnbounded, where);
  raw.effortLimit = optionalDoubleAttr(limit, "effort", kUnbounded, where);
}

void parseMimic(const pugi::xml_node& node, RawJoint& raw) {
  const pugi::xml_node mimic = node.child("mimic");
  if (!mimic) {
    return;
  }
  const std::string where = jointWhere(raw.name, "mimic");

  const pugi::xml_attribute source = mimic.attribute("joint");
  if (!source || std::string(source.value()).empty()) {
    fail(where, "required attribute 'joint' is missing or empty");
  }
  raw.mimicSource = source.value();
  if (raw.mimicSource == raw.name) {
    fail(where, "a joint cannot mimic itself");
  }

  raw.mimicMultiplier = optionalDoubleAttr(mimic, "multiplier", 1.0, where);
  raw.mimicOffset = optionalDoubleAttr(mimic, "offset", 0.0, where);
}

RawJoint parseJoint(const pugi::xml_node& node) {
  RawJoint raw;

  const pugi::xml_attribute nameAttr = node.attribute("name");
  if (!nameAttr || std::string(nameAttr.value()).empty()) {
    fail("<joint>", "required attribute 'name' is missing or empty");
  }
  raw.name = nameAttr.value();

  const pugi::xml_attribute typeAttr = node.attribute("type");
  if (!typeAttr) {
    fail(jointWhere(raw.name), "required attribute 'type' is missing");
  }
  if (!parseJointType(typeAttr.value(), raw.type)) {
    // 'planar' and 'floating' are valid URDF but multi-DOF, so they cannot be
    // represented by one scalar in q. Naming them turns a confusing rejection
    // into a stated limitation.
    const std::string type = typeAttr.value();
    if (type == "planar" || type == "floating") {
      fail(jointWhere(raw.name), "joint type '" + type +
                                     "' has more than one degree of freedom and is not "
                                     "supported; this library assumes one scalar per joint");
    }
    fail(jointWhere(raw.name),
         "unknown joint type '" + type + "'; expected revolute, continuous, prismatic or fixed");
  }

  const pugi::xml_node parent = node.child("parent");
  if (!parent || !parent.attribute("link")) {
    fail(jointWhere(raw.name), "missing <parent link=\"...\"/>");
  }
  raw.parentLink = parent.attribute("link").value();

  const pugi::xml_node child = node.child("child");
  if (!child || !child.attribute("link")) {
    fail(jointWhere(raw.name), "missing <child link=\"...\"/>");
  }
  raw.childLink = child.attribute("link").value();

  if (raw.parentLink == raw.childLink) {
    fail(jointWhere(raw.name), "parent and child are the same link '" + raw.parentLink + "'");
  }

  raw.origin = parseOrigin(node, jointWhere(raw.name));

  if (isMovable(raw.type)) {
    parseAxis(node, raw);
    parseLimits(node, raw);
    parseMimic(node, raw);
  }
  return raw;
}

}  // namespace

// ---------------------------------------------------------------------------
// UrdfError, jointTypeName
// ---------------------------------------------------------------------------

UrdfError::UrdfError(std::string where, std::string what)
    : std::runtime_error(where + ": " + what), where_(std::move(where)), detail_(std::move(what)) {}

const std::string& UrdfError::where() const { return where_; }

const std::string& UrdfError::detail() const { return detail_; }

const char* jointTypeName(JointType type) {
  switch (type) {
    case JointType::Revolute:
      return "revolute";
    case JointType::Continuous:
      return "continuous";
    case JointType::Prismatic:
      return "prismatic";
    case JointType::Fixed:
      return "fixed";
  }
  return "unknown";  // unreachable for a valid enumerator; -Wreturn-type wants it anyway
}

// ---------------------------------------------------------------------------
// Construction hook
// ---------------------------------------------------------------------------

namespace detail {
struct RobotAccess {
  static Robot make(std::string name, std::vector<Joint> joints, std::vector<Link> links,
                    int rootLink, int tipLink, int numDofs) {
    Robot robot;
    robot.name_ = std::move(name);
    robot.joints_ = std::move(joints);
    robot.links_ = std::move(links);
    robot.rootLink_ = rootLink;
    robot.tipLink_ = tipLink;
    robot.numDofs_ = numDofs;
    return robot;
  }
};
}  // namespace detail

namespace {

// ---------------------------------------------------------------------------
// Stage two: names -> indices, tree -> ordered joints, fixed -> folded
// ---------------------------------------------------------------------------

Robot buildRobot(const pugi::xml_node& robotNode, const std::string* requestedTip) {
  // --- Step 1a: collect links -------------------------------------------
  std::vector<std::string> linkNames;
  std::unordered_map<std::string, int> linkIndex;
  for (pugi::xml_node node = robotNode.child("link"); node; node = node.next_sibling("link")) {
    const pugi::xml_attribute nameAttr = node.attribute("name");
    if (!nameAttr || std::string(nameAttr.value()).empty()) {
      fail("<link>", "required attribute 'name' is missing or empty");
    }
    const std::string name = nameAttr.value();
    if (linkIndex.count(name) != 0) {
      fail("link '" + name + "'", "declared more than once");
    }
    linkIndex.emplace(name, static_cast<int>(linkNames.size()));
    linkNames.push_back(name);
  }
  if (linkNames.empty()) {
    fail("<robot>", "contains no <link> elements");
  }

  // --- Step 1b: collect joints ------------------------------------------
  std::vector<RawJoint> raws;
  std::unordered_map<std::string, int> rawIndex;
  for (pugi::xml_node node = robotNode.child("joint"); node; node = node.next_sibling("joint")) {
    RawJoint raw = parseJoint(node);
    if (rawIndex.count(raw.name) != 0) {
      fail(jointWhere(raw.name), "declared more than once");
    }
    rawIndex.emplace(raw.name, static_cast<int>(raws.size()));
    raws.push_back(std::move(raw));
  }

  const std::size_t numLinks = linkNames.size();
  const std::size_t numRaw = raws.size();

  auto linkOf = [&linkIndex](const std::string& name, const std::string& where) {
    const auto it = linkIndex.find(name);
    if (it == linkIndex.end()) {
      fail(where, "refers to link '" + name + "', which is not declared as a <link> in this file");
    }
    return it->second;
  };

  // --- Step 2: find the root --------------------------------------------
  // The root is the single link that is never anybody's child. Building the
  // incoming-joint table on the way also catches the "two parents" case, which
  // is a graph but not a tree and would make forward kinematics ambiguous.
  std::vector<int> incomingJoint(numLinks, -1);
  std::vector<std::vector<int>> outgoingJoints(numLinks);
  for (std::size_t j = 0; j < numRaw; ++j) {
    const RawJoint& raw = raws[j];
    const int parent = linkOf(raw.parentLink, jointWhere(raw.name, "parent"));
    const int child = linkOf(raw.childLink, jointWhere(raw.name, "child"));
    const std::size_t childIdx = static_cast<std::size_t>(child);

    if (incomingJoint[childIdx] != -1) {
      fail(jointWhere(raw.name), "link '" + raw.childLink + "' already has a parent joint '" +
                                     raws[static_cast<std::size_t>(incomingJoint[childIdx])].name +
                                     "'; in a kinematic tree every link has exactly one parent");
    }
    incomingJoint[childIdx] = static_cast<int>(j);
    outgoingJoints[static_cast<std::size_t>(parent)].push_back(static_cast<int>(j));
  }

  std::vector<int> roots;
  for (std::size_t i = 0; i < numLinks; ++i) {
    if (incomingJoint[i] == -1) {
      roots.push_back(static_cast<int>(i));
    }
  }
  if (roots.empty()) {
    // Every link has a parent, so the joints contain at least one cycle and
    // there is no base to measure anything from.
    fail("<robot>",
         "every link has a parent joint, so the joints form a cycle; a kinematic tree "
         "needs exactly one root link");
  }
  if (roots.size() > 1) {
    std::ostringstream msg;
    msg << "found " << roots.size() << " links with no parent joint (";
    for (std::size_t i = 0; i < roots.size(); ++i) {
      msg << (i == 0 ? "" : ", ") << "'" << linkNames[static_cast<std::size_t>(roots[i])] << "'";
    }
    msg << "); a URDF must describe a single connected tree";
    fail("<robot>", msg.str());
  }
  const int rootLink = roots[0];

  // --- Steps 3 to 5: one walk does sorting, folding and link placement ---
  // A depth-first walk from the root visits every parent before its children,
  // so movable joints come out in topological order for free -- no separate
  // sorting pass, and parentJoint is always an index that already exists.
  //
  // Two quantities are carried down the tree per link, exactly the pair
  // documented on Link: which movable joint supports it, and its offset inside
  // that joint's frame. A fixed joint EXTENDS the offset; a movable joint RESETS
  // it and opens a new joint frame. That is the whole folding rule, and it is
  // why a trailing fixed chain (hand -> flange -> grasp target) survives:
  // folding only forward into the next movable joint would have nowhere to put
  // it and would silently drop it.
  std::vector<Joint> joints;
  std::vector<Link> links(numLinks);
  for (std::size_t i = 0; i < numLinks; ++i) {
    links[i].name = linkNames[i];
  }
  links[static_cast<std::size_t>(rootLink)].supportingJoint = -1;
  links[static_cast<std::size_t>(rootLink)].offset = SE3::identity();

  std::vector<int> emittedFor(numRaw, -1);  // raw joint index -> emitted joint index
  std::vector<bool> visited(numLinks, false);
  std::vector<int> stack{rootLink};
  visited[static_cast<std::size_t>(rootLink)] = true;

  while (!stack.empty()) {
    const int linkIdx = stack.back();
    stack.pop_back();

    for (const int rawIdx : outgoingJoints[static_cast<std::size_t>(linkIdx)]) {
      const RawJoint& raw = raws[static_cast<std::size_t>(rawIdx)];
      const int childIdx = linkOf(raw.childLink, jointWhere(raw.name, "child"));
      const std::size_t child = static_cast<std::size_t>(childIdx);

      // Read the parent's carried state fresh each iteration: `links` is
      // written to inside this loop, so holding a reference across the write
      // would be asking for trouble even though the vector never reallocates.
      const int parentSupport = links[static_cast<std::size_t>(linkIdx)].supportingJoint;
      const SE3 parentOffset = links[static_cast<std::size_t>(linkIdx)].offset;

      if (isMovable(raw.type)) {
        Joint joint;
        joint.name = raw.name;
        joint.type = raw.type;
        // Accumulated fixed transform down to this link, then the joint's own
        // origin. With no fixed joints in between, parentOffset is identity and
        // this is just raw.origin.
        joint.originTransform = parentOffset * raw.origin;
        joint.axis = raw.axis;
        joint.lowerLimit = raw.lowerLimit;
        joint.upperLimit = raw.upperLimit;
        joint.velocityLimit = raw.velocityLimit;
        joint.effortLimit = raw.effortLimit;
        joint.parentJoint = parentSupport;
        joint.parentLink = linkIdx;
        joint.childLink = childIdx;

        const int emitted = static_cast<int>(joints.size());
        joints.push_back(std::move(joint));
        emittedFor[static_cast<std::size_t>(rawIdx)] = emitted;

        links[child].supportingJoint = emitted;
        links[child].offset = SE3::identity();
      } else {
        links[child].supportingJoint = parentSupport;
        links[child].offset = parentOffset * raw.origin;
      }

      visited[child] = true;
      stack.push_back(childIdx);
    }
  }

  // A link the walk never reached sits in a component disconnected from the
  // root. The single-root check above does not catch this when that component
  // contains a cycle of its own, since then none of its links is parentless.
  for (std::size_t i = 0; i < numLinks; ++i) {
    if (!visited[i]) {
      fail("link '" + linkNames[i] + "'",
           "is not connected to the root link '" + linkNames[static_cast<std::size_t>(rootLink)] +
               "'; the joints reaching it form a cycle or a separate component");
    }
  }

  // --- Step 6: mimic references and configuration indices ---------------
  // Independent joints are numbered in joint order, so for a robot with no
  // mimic joints dofIndex ends up equal to the joint index throughout.
  int numDofs = 0;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const int rawIdx = rawIndex.find(joints[i].name)->second;
    const RawJoint& raw = raws[static_cast<std::size_t>(rawIdx)];

    if (raw.mimicSource.empty()) {
      joints[i].dofIndex = numDofs++;
      continue;
    }

    const auto sourceIt = rawIndex.find(raw.mimicSource);
    if (sourceIt == rawIndex.end()) {
      fail(jointWhere(joints[i].name, "mimic"),
           "refers to joint '" + raw.mimicSource + "', which does not exist");
    }
    const RawJoint& sourceRaw = raws[static_cast<std::size_t>(sourceIt->second)];
    if (!isMovable(sourceRaw.type)) {
      fail(jointWhere(joints[i].name, "mimic"),
           "refers to fixed joint '" + raw.mimicSource + "', which has no value to mimic");
    }
    if (!sourceRaw.mimicSource.empty()) {
      fail(jointWhere(joints[i].name, "mimic"),
           "refers to joint '" + raw.mimicSource +
               "', which is itself a mimic joint; chained mimics are not supported");
    }
    joints[i].mimicSource = emittedFor[static_cast<std::size_t>(sourceIt->second)];
    joints[i].mimicMultiplier = raw.mimicMultiplier;
    joints[i].mimicOffset = raw.mimicOffset;
  }

  // --- End-effector -----------------------------------------------------
  int tipLink = -1;
  if (requestedTip != nullptr) {
    const auto it = linkIndex.find(*requestedTip);
    if (it == linkIndex.end()) {
      fail("<robot>", "requested tip link '" + *requestedTip + "' does not exist in this file");
    }
    tipLink = it->second;
  } else {
    // A leaf is a link with no outgoing joints. Exactly one leaf makes the tip
    // unambiguous. Anything with a gripper has several, and guessing among them
    // would put the end-effector on an arbitrary finger.
    std::vector<int> leaves;
    for (std::size_t i = 0; i < numLinks; ++i) {
      if (outgoingJoints[i].empty()) {
        leaves.push_back(static_cast<int>(i));
      }
    }
    if (leaves.size() != 1) {
      std::ostringstream msg;
      msg << "cannot pick an end-effector automatically: the robot has " << leaves.size()
          << " leaf links (";
      for (std::size_t i = 0; i < leaves.size(); ++i) {
        msg << (i == 0 ? "" : ", ") << "'" << linkNames[static_cast<std::size_t>(leaves[i])] << "'";
      }
      msg << "); name one explicitly";
      fail("<robot>", msg.str());
    }
    tipLink = leaves[0];
  }

  const std::string robotName =
      robotNode.attribute("name") ? robotNode.attribute("name").value() : "";
  return detail::RobotAccess::make(robotName, std::move(joints), std::move(links), rootLink,
                                   tipLink, numDofs);
}

// ---------------------------------------------------------------------------
// Document loading
// ---------------------------------------------------------------------------

// pugixml reports a byte offset; a line number is what a person can act on.
[[noreturn]] void failXml(const pugi::xml_parse_result& result, const std::string& text,
                          const std::string& source) {
  std::size_t line = 1;
  const std::size_t offset = static_cast<std::size_t>(result.offset);
  for (std::size_t i = 0; i < offset && i < text.size(); ++i) {
    if (text[i] == '\n') {
      ++line;
    }
  }
  std::ostringstream msg;
  msg << "XML is not well formed at line " << line << ": " << result.description();
  fail(source, msg.str());
}

pugi::xml_node robotRoot(const pugi::xml_document& doc, const std::string& source) {
  const pugi::xml_node robot = doc.child("robot");
  if (!robot) {
    fail(source, "no <robot> element at the top level");
  }
  if (robot.next_sibling("robot")) {
    fail(source, "more than one <robot> element; a URDF describes exactly one robot");
  }
  return robot;
}

}  // namespace

// ---------------------------------------------------------------------------
// Robot: factories
// ---------------------------------------------------------------------------

Robot Robot::fromUrdfString(const std::string& xml) { return fromUrdfString(xml, std::string()); }

Robot Robot::fromUrdfString(const std::string& xml, const std::string& tipLink) {
  pugi::xml_document doc;
  const pugi::xml_parse_result result = doc.load_string(xml.c_str());
  if (!result) {
    failXml(result, xml, "<urdf string>");
  }
  return buildRobot(robotRoot(doc, "<urdf string>"), tipLink.empty() ? nullptr : &tipLink);
}

Robot Robot::fromUrdfFile(const std::string& path) { return fromUrdfFile(path, std::string()); }

Robot Robot::fromUrdfFile(const std::string& path, const std::string& tipLink) {
  pugi::xml_document doc;
  const pugi::xml_parse_result result = doc.load_file(path.c_str());
  if (!result) {
    if (result.status == pugi::status_file_not_found) {
      fail("'" + path + "'", "file not found");
    }
    if (result.status == pugi::status_io_error) {
      fail("'" + path + "'", "file could not be read");
    }
    // The file text is not in hand here, so report the offset pugixml gave
    // rather than inventing a line number from nothing.
    std::ostringstream msg;
    msg << "XML is not well formed at byte offset " << result.offset << ": "
        << result.description();
    fail("'" + path + "'", msg.str());
  }
  return buildRobot(robotRoot(doc, "'" + path + "'"), tipLink.empty() ? nullptr : &tipLink);
}

// ---------------------------------------------------------------------------
// Robot: accessors
// ---------------------------------------------------------------------------

const std::string& Robot::name() const { return name_; }

int Robot::numJoints() const { return static_cast<int>(joints_.size()); }

int Robot::numDofs() const { return numDofs_; }

const Joint& Robot::joint(int i) const { return joints_[static_cast<std::size_t>(i)]; }

int Robot::numLinks() const { return static_cast<int>(links_.size()); }

const Link& Robot::link(int i) const { return links_[static_cast<std::size_t>(i)]; }

int Robot::rootLinkIndex() const { return rootLink_; }

int Robot::tipLinkIndex() const { return tipLink_; }

int Robot::findJoint(const std::string& jointName) const {
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].name == jointName) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int Robot::findLink(const std::string& linkName) const {
  for (std::size_t i = 0; i < links_.size(); ++i) {
    if (links_[i].name == linkName) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace robometrics
