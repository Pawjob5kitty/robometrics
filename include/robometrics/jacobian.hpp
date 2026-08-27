#pragma once

#include <Eigen/Core>

#include "robometrics/kinematics.hpp"
#include "robometrics/urdf.hpp"

namespace robometrics {

/// Geometricky Jacobian v ramci baze. Rozmer 6 x numDofs().
/// Radky 0..2 = translacni cast, radky 3..5 = rotacni ([v; omega]).
Eigen::MatrixXd jacobian(const Robot& robot, const Eigen::VectorXd& q);

}  // namespace robometrics

/// Hybrid geometricky Jacobian: 6 x numDofs().
///
/// Radky 0..2 = translacni cast, radky 3..5 = rotacni ([v; omega]).
///
/// KONVENCE — v literature se dela obema zpusoby a zamena se tise prelozi:
///   hybrid  (tady) — rychlost bodu NA CHAPADLE, vyjadrena v orientaci baze
///   spatial        — rychlost fiktivniho bodu telesa v pocatku baze
/// Lisi se o omega x p_tip. Numericka kontrola proti FK musi prevadet
/// adjunktem SAMOTNE ROTACE, ne cele transformace.
///
/// Mimic klouby nemaji vlastni sloupec — prispivaji do sloupce sveho
/// driveru, vynasobene mimicMultiplier.