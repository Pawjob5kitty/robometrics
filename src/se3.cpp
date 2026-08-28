#include "robometrics/se3.hpp"

#include <algorithm>
#include <cmath>

// Twists are stacked [v; omega], translational part first. Half the literature
// uses [omega; v]; mixing the two compiles fine and silently swaps the
// off-diagonal blocks of the adjoint.

namespace robometrics {

Mat3 skew(const Vec3& w) {
  Mat3 S;
  // Index trap: this takes a Vec3, not a slice of a Vec6 twist. If the
  // argument comes out of a twist it must be the omega block (indices 3..5).
  // Passing the v block compiles, runs, and quietly computes something else.
  //
  // Formatting is off so the sign pattern stays visible; clang-format would
  // join this onto one line.
  // clang-format off
  S << 0.0,    -w.z(),  w.y(),
       w.z(),   0.0,   -w.x(),
      -w.y(),   w.x(),  0.0;
  // clang-format on
  return S;
}

// Rodrigues' formula: rotation by ||w|| about w/||w||. Written in W = skew(w)
// rather than a unit axis so w is never divided by its own norm, which keeps
// the small-angle case finite.
Mat3 rodrigues(const Vec3& w) {
  const double theta = w.norm();
  const Mat3 W = skew(w);

  // WHY A THRESHOLD AND NOT `theta == 0.0`. Two reasons.
  //
  // The formula degrades continuously, not at a point: `1 - cos(theta)`
  // cancels, so B carries a relative error of ~2*eps/theta^2 that grows without
  // bound as theta shrinks. An equality test leaves that whole band on the
  // direct branch.
  //
  // And exact zero is not the only input that produces NaN: below theta
  // ~1.5e-154 the product theta*theta underflows to zero and B evaluates
  // 0.0/0.0 for a perfectly ordinary nonzero input.
  //
  // WHY 1e-6. The upper bound binds. The truncated series must stay accurate to
  // full double precision, so the first dropped term has to sit below
  // eps = 2.2e-16 relative to the coefficient it corrects:
  //
  //   A: dropped theta^4/120  ->  theta < (120*eps)^(1/4) = 4.0e-4
  //   B: dropped theta^4/720  ->  theta < (360*eps)^(1/4) = 5.3e-4
  //
  // The floor is much weaker than it looks. B is only ever multiplied by W^2,
  // whose norm is theta^2, so its relative error contributes an ABSOLUTE error
  // of ~eps to R however small theta gets -- a large relative error in a
  // coefficient is harmless when the coefficient multiplies something equally
  // small. The honest floor is sqrt(eps) = 1.5e-8, below which `1 - cos(theta)`
  // collapses to exactly 0 and the coefficient stops meaning anything on its
  // own. 1e-6 sits near the middle of [1.5e-8, 4e-4] and serves exp() and log()
  // as well.
  constexpr double kSmall = 1e-6;

  double A, B;
  if (theta < kSmall) {
    const double t2 = theta * theta;
    A = 1.0 - t2 / 6.0;
    B = 0.5 - t2 / 24.0;
  } else {
    A = std::sin(theta) / theta;
    B = (1.0 - std::cos(theta)) / (theta * theta);
  }

  return Mat3::Identity() + A * W + B * W * W;
}

SE3::SE3() : R_(Mat3::Identity()), t_(Vec3::Zero()) {}

SE3::SE3(const Mat3& R, const Vec3& t) : R_(R), t_(t) {}

// The bottom row of M is ignored, not validated; see the invariant note in
// se3.hpp for why the class does not enforce it at runtime.
SE3::SE3(const Mat4& M) : R_(M.block<3, 3>(0, 0)), t_(M.block<3, 1>(0, 3)) {}

SE3 SE3::identity() { return SE3(Mat3::Identity(), Vec3::Zero()); }

const Mat3& SE3::rotation() const { return R_; }
const Vec3& SE3::translation() const { return t_; }

// Homogeneous form: [[R, t], [0, 1]]. Acting on (p; 1) gives (R*p + t; 1);
// acting on a direction written as (d; 0) gives (R*d; 0), so the translation
// drops out on its own. The trailing 0/1 encodes which of the two you have.
Mat4 SE3::matrix() const {
  Mat4 M = Mat4::Zero();  // starting from Zero() means the last row is already correct
  M.block<3, 3>(0, 0) = R_;
  M.block<3, 1>(0, 3) = t_;
  M(3, 3) = 1.0;
  return M;
}

// Closed form (R^T, -R^T * t). Inverting matrix() generically gives the same
// answer but costs an LU factorisation and rounds where nothing needs rounding.
SE3 SE3::inverse() const { return SE3(R_.transpose(), -R_.transpose() * t_); }

// The translation is NOT t + t_o: the right operand's translation is seen
// through the left operand's rotation. Dropping that R_* is the most common
// bug in a hand-written kinematic chain, and it is invisible whenever the test
// rotations happen to be identity.
SE3 SE3::operator*(const SE3& other) const {
  return SE3(R_ * other.rotation(), R_ * other.translation() + t_);
}

// Acts on a POINT. For a direction use rotation() * d instead.
Vec3 SE3::act(const Vec3& p) const { return R_ * p + t_; }

// ABSOLUTE tolerance, deliberately not Eigen::isApprox. Eigen's is relative to
// the operand norms, so near the origin it demands far more precision than any
// caller intends, while at magnitude 1e6 it accepts millimetres.
bool SE3::isApprox(const SE3& other, double tol) const {
  return (R_ - other.rotation()).cwiseAbs().maxCoeff() <= tol &&
         (t_ - other.translation()).cwiseAbs().maxCoeff() <= tol;
}

// Exponential map se(3) -> SE(3): integrate the constant twist x for one unit
// of time. The translational part is NOT simply v -- the body rotates while it
// translates, so the displacement is bent: t = V*v, where V is the average of
// the rotation over the motion and equals I when omega == 0.
SE3 exp(const Vec6& x) {
  const Vec3 v = x.head<3>();  // components 0,1,2 -- translational part
  const Vec3 w = x.tail<3>();  // components 3,4,5 -- rotational part

  const double theta = w.norm();
  const Mat3 W = skew(w);

  // Same threshold as rodrigues(). C is the more fragile of the two:
  // `theta - sin(theta)` cancels down to theta^3/6, so its relative error is
  // ~6*eps/theta^2, worse than B's.
  constexpr double kSmall = 1e-6;
  double B, C;
  if (theta < kSmall) {
    const double t2 = theta * theta;
    B = 0.5 - t2 / 24.0;
    C = 1.0 / 6.0 - t2 / 120.0;
  } else {
    B = (1.0 - std::cos(theta)) / (theta * theta);
    C = (theta - std::sin(theta)) / (theta * theta * theta);
  }

  const Mat3 V = Mat3::Identity() + B * W + C * W * W;
  return SE3(rodrigues(w), V * v);
}

// Logarithmic map SE(3) -> se(3), inverse of exp on the principal branch only:
// exp is not injective, so log() returns the representative with
// ||omega|| <= pi and log(exp(x)) == x holds only for ||omega|| < pi. That is a
// property of the map, not a defect.
//
// THREE numerical traps, all handled below.
//
//   (a) theta -> 0. `sin(theta)` in the axis step and `tan(theta/2)` in the
//       translation step both drive divisions towards 0/0. Handled by the
//       kSmall branches.
//
//   (b) acos() outside its domain. Mathematically (trace(R) - 1)/2 lies in
//       [-1, 1], but R has been through floating-point multiplication and has
//       drifted off the group. A trace of 3 + 1e-16 gives an argument of
//       1 + 5e-17, and std::acos returns NaN for anything outside the range,
//       which then propagates silently through every remaining line. The clamp
//       is not defensive padding, it is the projection back onto the domain the
//       input was supposed to be in.
//
//   (c) theta -> pi. As theta approaches pi, sin(theta) -> 0 and R - R^T -> 0
//       together, so the general axis step becomes 0/0 and the antisymmetric
//       part stops carrying the axis. The near-pi branch reads the axis from
//       the SYMMETRIC part instead (R + I == 2*n*n^T at pi), then recovers its
//       sign from the still-directional antisymmetric part; see the derivation
//       at the branch. The translation step needs no special case: at pi,
//       (theta/2)*cot(theta/2) -> 0, so V^-1's coefficient c stays finite.
Vec6 log(const SE3& T) {
  const Mat3& R = T.rotation();
  const Vec3& t = T.translation();

  double cosTheta = (R.trace() - 1.0) / 2.0;
  cosTheta = std::clamp(cosTheta, -1.0, 1.0);  // trap (b)
  const double theta = std::acos(cosTheta);

  constexpr double kSmall = 1e-6;

  // WHY kNearPi. The general branch scales R - R^T (whose entries are
  // 2*sin(theta)*n_i, each carrying absolute rounding ~eps) by theta/(2 sin
  // theta), so the axis picks up an absolute error ~ (theta / (2 sin theta)) *
  // eps ~ (pi / (2 * (pi - theta))) * eps as theta -> pi. Keeping that below the
  // round-trip tolerance 1e-9 needs pi - theta > pi*eps / (2e-9) ~ 3.5e-7, so
  // the general branch stays good until within ~3.5e-7 of pi. 1e-3 switches
  // three-and-a-half orders earlier, so at the seam the general branch is still
  // accurate to ~3.5e-13 and the symmetric branch to ~eps -- they agree far
  // inside tolerance, which is what keeps the two-branch result continuous. The
  // upper bound is loose: the symmetric method only needs 1 - cos(theta) not
  // small, and that stays near 2 for any theta this close to pi.
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kNearPi = 1e-3;

  Vec3 w;
  if (theta < kSmall) {
    // Near identity: (R - R^T)/2 is skew(w) itself to O(theta^3). Reading
    // (2,1), (0,2), (1,0) inverts skew()'s layout; the mirror positions would
    // negate the axis and still pass any test that only checks ||w||.
    const Mat3 S = 0.5 * (R - R.transpose());
    w = Vec3(S(2, 1), S(0, 2), S(1, 0));
  } else if (theta > kPi - kNearPi) {
    // Near pi the axis comes from the SYMMETRIC part, because R - R^T has lost
    // it. With Sym = (R + R^T)/2 = cos(theta) I + (1 - cos theta) n n^T, the
    // outer product is
    //
    //   n n^T = (Sym - cos(theta) I) / (1 - cos theta),
    //
    // exact for a true rotation, and 1 - cos(theta) ~ 2 here so the division is
    // well conditioned. Take the largest diagonal as the reference component --
    // it satisfies n_k^2 >= 1/3, so dividing the rest of its row by n_k is safe.
    const Mat3 sym = 0.5 * (R + R.transpose());
    const double a = 1.0 - cosTheta;  // in (~1, 2] for theta in (pi/2, pi]
    int k = 0;
    if (sym(1, 1) > sym(k, k)) {
      k = 1;
    }
    if (sym(2, 2) > sym(k, k)) {
      k = 2;
    }
    Vec3 n;
    n(k) = std::sqrt(std::max(0.0, (sym(k, k) - cosTheta) / a));
    for (int j = 0; j < 3; ++j) {
      if (j != k) {
        n(j) = sym(k, j) / (a * n(k));
      }
    }
    n.normalize();  // eps-level cleanup; the exact-coefficient rows above are
                    // already unit to ~eps, so this only removes roundoff

    // The symmetric part fixes the axis only up to sign. The antisymmetric part
    // R - R^T == 2*sin(theta)*skew(n) still carries that sign as long as
    // sin(theta) is above the noise floor -- it vanishes only exactly at pi,
    // where +n and -n are the same rotation anyway. Align n with it so the
    // result matches the general branch just below the seam and round-trips
    // (exp(theta*n) != exp(-theta*n) for theta < pi).
    const Vec3 axisSign(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    if (n.dot(axisSign) < 0.0) {
      n = -n;
    }
    w = theta * n;
  } else {
    // R - R^T == 2*sin(theta)*skew(n); scaling by theta/(2 sin theta) turns it
    // into theta*n, read out in skew()'s layout.
    const Mat3 S = (theta / (2.0 * std::sin(theta))) * (R - R.transpose());
    w = Vec3(S(2, 1), S(0, 2), S(1, 0));
  }

  // Undo the V factor exp() applied to the translation: v == V^-1 * t, where
  //
  //   V^-1 = I - W/2 + c*W^2,   c = (1 - (theta/2)*cot(theta/2)) / theta^2.
  //
  // The middle coefficient is exactly -1/2 for every theta. Only c is delicate:
  // its numerator cancels to theta^2/12 as theta -> 0, giving the 1/12 the
  // small branch uses. (cot is spelled 1/tan; std has no std::cot.)
  const Mat3 W = skew(w);
  Mat3 Vinv;
  if (theta < kSmall) {
    Vinv = Mat3::Identity() - 0.5 * W + (1.0 / 12.0) * W * W;
  } else {
    const double c = (1.0 - (theta / 2.0) / std::tan(theta / 2.0)) / (theta * theta);
    Vinv = Mat3::Identity() - 0.5 * W + c * W * W;
  }
  const Vec3 v = Vinv * t;

  Vec6 x;
  x.head<3>() = v;
  x.tail<3>() = w;
  return x;
}

// Adjoint: re-expresses a twist in another frame. If x is a velocity seen in
// frame B, adjoint(T_AB) * x is the same velocity written in frame A.
//
//        ┌─────────────┬─────────────┐
//        │      R      │  skew(t)*R  │   rows 0..2  ->  new v
//        ├─────────────┼─────────────┤
//        │      0      │      R      │   rows 3..5  ->  new omega
//        └─────────────┴─────────────┘
//          cols 0..2      cols 3..5
//
// The top-right block is the lever arm: a body spinning at omega about a point
// offset by t drags the reference point along at t x (R*omega). The bottom-left
// block is structurally zero, which is why det == det(R)^2 == 1 and the matrix
// is invertible for every T.
//
// Under the opposite [omega; v] convention the two off-diagonal blocks trade
// places, so this cannot be lifted from another codebase without checking which
// stacking order that codebase used.
Mat6 adjoint(const SE3& T) {
  const Mat3& R = T.rotation();
  const Vec3& t = T.translation();

  Mat6 A = Mat6::Zero();  // the bottom-left block is then already correct

  A.block<3, 3>(0, 0) = R;
  A.block<3, 3>(0, 3) = skew(t) * R;
  A.block<3, 3>(3, 3) = R;
  return A;
}

}  // namespace robometrics
