#pragma once

#include <Eigen/Core>

namespace robometrics {

/// Nejmensi singularni cislo TRANSLACNI casti Jacobianu (radky 0..2).
///
/// Jednotky jsou metry na radian, takze cislo je rozmerove konzistentni.
/// Cely 6xN Jacobian se na tohle nehodi: rotacni radky jsou jednotkove
/// vektory, takze sigma_min nikdy neklesne pod 1 a singularitu schova.
double sigmaMinTranslation(const Eigen::MatrixXd& J);

}  // namespace robometrics