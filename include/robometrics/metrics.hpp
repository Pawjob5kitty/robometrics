#pragma once

#include <Eigen/Core>
#include <vector>

#include "robometrics/urdf.hpp"

namespace robometrics {

/// Nejmensi singularni cislo TRANSLACNI casti Jacobianu (radky 0..2).
///
/// Jednotky jsou metry na radian, takze cislo je rozmerove konzistentni.
/// Cely 6xN Jacobian se na tohle nehodi: rotacni radky jsou jednotkove
/// vektory, takze sigma_min nikdy neklesne pod 1 a singularitu schova.
double sigmaMinTranslation(const Eigen::MatrixXd& J);

/// sigmaMinTranslation v kazdem bode trajektorie.
///
/// Vstup: posloupnost konfiguraci (kazda o delce numDofs()).
/// Vystup: stejne dlouhy vektor cisel.
///
/// Minimum tohoto profilu je metrika "bezpecne" — jak blizko selhani robot
/// byl v nejhorsim okamziku. Prumer ani koncova hodnota to nerekne: staci
/// jeden pruchod singularitou a cela trajektorie je krehka.
std::vector<double> sigmaMinProfile(const Robot& robot,
                                    const std::vector<Eigen::VectorXd>& traj);

}  // namespace robometrics