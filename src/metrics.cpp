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

  // JacobiSVD without thin/full unitary flags computes the singular values
  // alone, which is all this needs. The singular vectors would tell us WHICH
  // direction is locked; that is a useful thing to report, but it is a
  // different function.
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jv);
  const Eigen::VectorXd s = svd.singularValues();

  // Note this is currently the same element as s.tail(1): Eigen's
  // singularValues() already returns exactly min(rows, cols) entries and pads
  // with nothing. Spelling out k is defensive, not load-bearing -- it names
  // which value is meant, so a switch to eigenvalues of Jv * Jv^T or to a
  // backend that returns a full-length vector cannot silently start reading a
  // structural zero. See the header.
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
  // fragile and an average would dilute it away. The reasoning is in the
  // header, where a reader of the API will actually see it.
  return *std::min_element(profile.begin(), profile.end());
}

std::optional<double> pathEfficiency(const Robot& robot, const std::vector<Eigen::VectorXd>& traj) {
  // A single point has no steps, so there is nothing to be efficient at. Not
  // "efficiency 1" -- that would claim the rollout was perfect.
  if (traj.size() < 2) {
    return std::nullopt;
  }

  double optimalCost = 0.0;
  double actualCost = 0.0;

  for (std::size_t i = 1; i < traj.size(); ++i) {
    const Eigen::VectorXd& qPrev = traj[i - 1];
    const Eigen::VectorXd& qCurr = traj[i];

    actualCost += (qCurr - qPrev).norm();

    // Where the tip actually went over this step.
    const SE3 tPrev = forwardKinematics(robot, qPrev);
    const SE3 tCurr = forwardKinematics(robot, qCurr);

    // log of the relative transform is the twist that carries tPrev to tCurr.
    // It comes out in the TIP'S OWN frame, because that is the frame the
    // relative transform is expressed in.
    const Vec6 dxLocal = log(tPrev.inverse() * tCurr);

    // jacobian() is the HYBRID Jacobian: velocity of the point on the gripper,
    // written in the orientation of the base. So dx has to be rotated into the
    // base orientation before the two can be compared.
    //
    // The adjoint of the ROTATION ALONE, not of tPrev. The full adjoint would
    // also move the reference point to the base origin, which is the spatial
    // convention -- a different quantity, differing by omega x p_tip. The two
    // agree only when the tip sits at the origin or is not rotating, which is
    // exactly the regime where a test would fail to notice the mistake. This
    // is the same conversion jacobian.hpp documents for its numeric check.
    const SE3 rotOnly(tPrev.rotation(), Vec3::Zero());
    const Vec6 dx = adjoint(rotOnly) * dxLocal;

    // The Jacobian at the START of the step. A derivative is evaluated where
    // the step begins; using q[i] would pair this step's displacement with the
    // next step's linearisation, which is a first-order error that survives
    // refinement -- it does not shrink as the trajectory is sampled finer, it
    // just moves.
    const Eigen::MatrixXd J = jacobian(robot, qPrev);

    // ThinU | ThinV is what makes solve() available. For an underdetermined
    // system -- a redundant arm, more columns than rows -- SVD's solve()
    // returns the MINIMUM-NORM least-squares solution, which is precisely
    // J^+ * dx. That choice of solver is the whole metric: a different
    // decomposition would return some other solution of the same system and
    // the ratio would stop meaning anything.
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd rhs = dx;
    optimalCost += svd.solve(rhs).norm();
  }

  // Summed first, divided once. Averaging per-step ratios instead would weight
  // a tiny jitter step the same as a long sweep, and a step of length zero
  // would be a division by zero rather than simply contributing nothing.
  if (actualCost <= 0.0) {
    // The robot never moved. There is no ratio to report -- see the header for
    // why this is not a threshold test.
    return std::nullopt;
  }
  return optimalCost / actualCost;
}

}  // namespace robometrics
