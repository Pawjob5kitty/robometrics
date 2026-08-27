#include "robometrics/metrics.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <optional>

#include "robometrics/jacobian.hpp"

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

}  // namespace robometrics
