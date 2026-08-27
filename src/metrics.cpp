#include "robometrics/metrics.hpp"

#include <Eigen/SVD>
#include <algorithm>

#include "robometrics/jacobian.hpp"

namespace robometrics {

double sigmaMinTranslation(const Eigen::MatrixXd& J) {
  const Eigen::MatrixXd Jv = J.topRows(3);
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jv);
  const Eigen::VectorXd s = svd.singularValues();

  // Pocet nenulovych singularnich cisel je nejvys min(radky, sloupce).
  // Robot se dvema klouby nedosahne do vsech tri smeru, takze treti
  // singularni cislo je VZDY nula — strukturalne, ne kvuli konfiguraci.
  // tail(1) by tedy vracelo nulu pro kazdou pozu.
  const Eigen::Index k = std::min(Jv.rows(), Jv.cols());
  return s(k - 1);
}


std::vector<double> sigmaMinProfile(const Robot& robot,
                                    const std::vector<Eigen::VectorXd>& traj) {
  std::vector<double> out;
  out.reserve(traj.size());
  for (const Eigen::VectorXd& q : traj) {
    out.push_back(sigmaMinTranslation(jacobian(robot, q)));
  }
  return out;
}

}  // namespace robometrics