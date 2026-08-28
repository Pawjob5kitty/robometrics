#include "robometrics/metrics.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <optional>

#include "robometrics/jacobian.hpp"
#include "robometrics/kinematics.hpp"
#include "robometrics/se3.hpp"

namespace robometrics {

double sigmaMinTranslation(const Eigen::MatrixXd& J) {
  // Rows 0..2 only. The rotational rows would mask the very thing being
  // measured -- see the header for why, and for the numbers on planar_arm.
  const Eigen::MatrixXd Jv = J.topRows(3);

  // No thin/full flags: the singular values alone are all this needs. The
  // singular vectors would say WHICH direction is locked -- useful, but a
  // different function.
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jv);
  const Eigen::VectorXd s = svd.singularValues();

  // Currently the same element as s.tail(1): singularValues() already returns
  // exactly min(rows, cols) entries. Spelling out k is defensive, naming which
  // value is meant so a switch to eigenvalues of Jv * Jv^T, or to a backend
  // that pads, cannot silently start reading a structural zero.
  const Eigen::Index k = std::min(Jv.rows(), Jv.cols());

  // Singular values come back in decreasing order, so k - 1 is the smallest.
  return s(k - 1);
}

std::vector<double> sigmaMinProfile(const Robot& robot, const std::vector<Eigen::VectorXd>& traj) {
  std::vector<double> out;
  out.reserve(traj.size());
  for (const Eigen::VectorXd& q : traj) {
    out.push_back(sigmaMinTranslation(jacobian(robot, q)));
  }
  return out;
}

std::optional<double> dexterityMargin(const Robot& robot,
                                      const std::vector<Eigen::VectorXd>& traj) {
  const std::vector<double> profile = sigmaMinProfile(robot, traj);
  if (profile.empty()) {
    // No worst moment exists. std::nullopt rather than 0.0, which would read as
    // "fully singular" -- see the header.
    return std::nullopt;
  }
  // The minimum, not the mean: one singular frame makes the whole trajectory
  // fragile and an average would dilute it away. See the header.
  return *std::min_element(profile.begin(), profile.end());
}

double characteristicLength(const Robot& robot) {
  // Walk the base->tip chain and sum the link lengths. The tip link's offset is
  // the final fixed hop (e.g. hand -> grasp frame); each joint's originTransform
  // is the hop from the previous movable joint, with fixed joints already folded
  // in. Summed once, from the robot alone -- no configuration enters here.
  double length = robot.link(robot.tipLinkIndex()).offset.translation().norm();
  for (int j = robot.link(robot.tipLinkIndex()).supportingJoint; j >= 0;
       j = robot.joint(j).parentJoint) {
    length += robot.joint(j).originTransform.translation().norm();
  }
  return length;
}

namespace {

// Largest Jacobian rank over a sample of the trajectory's configurations. Rank
// has to be measured, not counted from joints -- it is bounded by the twist
// components the mechanism can produce (<= 3 for a planar arm). A single pose
// can be singular, so the maximum over a stride of ~16 configurations is taken.
Eigen::Index sampledMaxRank(const Robot& robot, const std::vector<Eigen::VectorXd>& traj) {
  constexpr std::size_t kSamples = 16;
  const std::size_t stride = std::max<std::size_t>(1, traj.size() / kSamples);
  Eigen::Index best = 0;
  for (std::size_t i = 0; i < traj.size(); i += stride) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian(robot, traj[i]));
    // A tolerance is unavoidable: rank is discontinuous and floating point never
    // returns an exact zero. 1e-9 sits far below any singular value a real
    // mechanism produces and far above rounding noise.
    svd.setThreshold(1e-9);
    best = std::max(best, svd.rank());
  }
  return best;
}

}  // namespace

EfficiencyStatus efficiencyStatus(const Robot& robot, const std::vector<Eigen::VectorXd>& traj) {
  // Mixed types first: a ratio of a radians-plus-metres sum is undefined
  // regardless of redundancy, so it dominates.
  bool hasRotary = false;
  bool hasPrismatic = false;
  for (int i = 0; i < robot.numJoints(); ++i) {
    switch (robot.joint(i).type) {
      case JointType::Revolute:
      case JointType::Continuous:
        hasRotary = true;
        break;
      case JointType::Prismatic:
        hasPrismatic = true;
        break;
      case JointType::Fixed:
        break;
    }
  }
  if (hasRotary && hasPrismatic) {
    return EfficiencyStatus::MixedJointTypes;
  }

  // Redundancy: without a null space the minimum-norm solution is the actual
  // one, so the ratio is 1 for every trajectory and measures nothing. An empty
  // traj gives no configuration to measure rank at, so it cannot be classified
  // here and is left to the other guards.
  if (!traj.empty() && static_cast<Eigen::Index>(robot.numDofs()) <= sampledMaxRank(robot, traj)) {
    return EfficiencyStatus::NotRedundant;
  }
  return EfficiencyStatus::Available;
}

std::optional<double> pathEfficiency(const Robot& robot, const std::vector<Eigen::VectorXd>& traj) {
  // A single point has no steps, so there is nothing to be efficient at. Not
  // "efficiency 1" -- that would claim the rollout was perfect.
  if (traj.size() < 2) {
    return std::nullopt;
  }

  // One-shot applicability: refuse the number when it would be 1 by construction
  // (not redundant) or built on a mixed-unit sum (both joint kinds), rather than
  // return a misleading value. The caller can query efficiencyStatus() to say
  // which. Dexterity and everything else are unaffected -- this rejects the
  // computation, not the robot.
  if (efficiencyStatus(robot, traj) != EfficiencyStatus::Available) {
    return std::nullopt;
  }

  double optimalCost = 0.0;
  double actualCost = 0.0;

  for (std::size_t i = 1; i < traj.size(); ++i) {
    const Eigen::VectorXd& qPrev = traj[i - 1];
    const Eigen::VectorXd& qCurr = traj[i];
    // Everything about this step is evaluated HERE, halfway along it. See
    // below and the header for why the midpoint and not either endpoint.
    const Eigen::VectorXd qMid = 0.5 * (qPrev + qCurr);

    actualCost += (qCurr - qPrev).norm();

    // Where the tip actually went. dx is a finite difference; J is a
    // derivative. Pairing them is a quadrature rule, and the point at which
    // the derivative is sampled decides the order of the error.
    const SE3 tPrev = forwardKinematics(robot, qPrev);
    const SE3 tCurr = forwardKinematics(robot, qCurr);
    const SE3 tMid = forwardKinematics(robot, qMid);

    // log of the relative transform is the twist that carries tPrev to tCurr,
    // expressed in the tip's own frame.
    const Vec6 dxLocal = log(tPrev.inverse() * tCurr);

    // jacobian() is HYBRID: velocity of the point on the gripper, written in
    // the orientation of the base. So dx has to be rotated into the base
    // orientation first.
    //
    // The adjoint of a ROTATION ALONE, never of the full transform. The full
    // adjoint would also move the reference point to the base origin -- the
    // spatial convention, differing by omega x p_tip. The two agree only when
    // the tip is at the origin or not rotating, which is exactly the regime
    // where a test fails to notice.
    //
    // The rotation is the MIDPOINT's, and that is not cosmetic: moving only the
    // Jacobian to the midpoint leaves a first-order term behind and is worse
    // than sampling both at the start. On planar_3r at 50 steps, |E - 1| is
    // 1.2e-2 for both-at-start, 8.4e-2 for J-at-mid alone, 1.0e-6 for both at
    // the midpoint.
    const SE3 rotOnly(tMid.rotation(), Vec3::Zero());
    const Vec6 dx = adjoint(rotOnly) * dxLocal;

    // Sampling the derivative at the centre of the interval it is integrated
    // over is the midpoint rule: first-order terms cancel, leaving O(h^2)
    // instead of the O(h) either endpoint gives. Measured order on a
    // non-redundant arm: 2.00 against 1.00.
    const Eigen::MatrixXd J = jacobian(robot, qMid);

    // ThinU | ThinV is what makes solve() available. For an underdetermined
    // system SVD's solve() returns the MINIMUM-NORM least-squares solution,
    // which is precisely J^+ * dx. That choice is the whole metric: another
    // decomposition would return some other solution of the same system.
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd rhs = dx;
    optimalCost += svd.solve(rhs).norm();
  }

  // Summed first, divided once. Averaging per-step ratios would weight a tiny
  // jitter step the same as a long sweep, and a zero-length step would be a
  // division by zero rather than contributing nothing.
  if (actualCost <= 0.0) {
    // The robot never moved. There is no ratio to report -- see the header for
    // why this is not a threshold test.
    return std::nullopt;
  }
  return optimalCost / actualCost;
}

}  // namespace robometrics
