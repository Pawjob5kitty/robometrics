#pragma once

#include <Eigen/Core>

#include "robometrics/kinematics.hpp"
#include "robometrics/urdf.hpp"

namespace robometrics {

/// Geometric Jacobian in the base frame. Dimension 6 x numDofs().
/// Rows 0..2 = translational part, rows 3..5 = rotational ([v; omega]).
Eigen::MatrixXd jacobian(const Robot& robot, const Eigen::VectorXd& q);

}  // namespace robometrics

/// Hybrid geometric Jacobian: 6 x numDofs().
///
/// Rows 0..2 = translational part, rows 3..5 = rotational ([v; omega]).
///
/// CONVENTION -- the literature does this both ways and a mix-up compiles
/// silently:
///   hybrid  (here) -- velocity of the point ON THE GRIPPER, in the base
///                     orientation
///   spatial        -- velocity of a fictitious body point at the base origin
/// They differ by omega x p_tip. A numeric check against FK must convert with
/// the adjoint of the ROTATION ALONE, not of the full transform.
///
/// Mimic joints have no column of their own -- they contribute to their
/// driver's column, scaled by mimicMultiplier.