#include "robometrics/se3.hpp"

#include <algorithm>
#include <cmath>

namespace robometrics {

// ---------------------------------------------------------------------------
// Reading guide
// ---------------------------------------------------------------------------
// Everything in this file is built from one idea: a rigid motion can be written
// as `exp` of a constant velocity applied for one unit of time. The velocity
// lives in a 6-dimensional vector space (the Lie algebra se(3)), the motion
// lives in a curved 6-dimensional manifold (the Lie group SE(3)), and `exp` is
// the bridge. Nothing here needs abstract Lie theory -- every formula below is
// a closed form you can verify with linear algebra alone.
//
// Convention used throughout: a twist is stacked as [v; omega], translational
// part first. Half the literature uses [omega; v]. Mixing them compiles fine
// and silently swaps the off-diagonal blocks of the adjoint.
//
// One notational shortcut that removes almost all the algebra below: for a
// unit axis n,
//
//     skew(n)^2 == n*n^T - I     and     skew(n)^3 == -skew(n).
//
// Every matrix power collapses back to {I, skew, skew^2}, which is why all the
// closed forms in this file have the shape `I + a*W + b*W^2` -- there is never
// a third term to write.

// Returning by value rather than through an out-parameter: Mat3 is nine doubles
// (72 B) with no dynamic allocation, so it moves into the caller's storage via
// (N)RVO. An out-parameter would save nothing and would force every call site
// to declare a variable before it has a value.
//
// Taking `const Vec3&` rather than by value: Vec3 is only 24 B, so by-value
// would cost nothing measurable either. The reference is about intent -- it
// says "this function reads its argument and does not keep a copy". The rest of
// this API passes Mat3/Mat4 by reference because for those the copy does start
// to matter, and a uniform rule is easier to read than a per-type judgement.
Mat3 skew(const Vec3& w) {
  Mat3 S;
  // The cross product with a FIXED w is a linear map v |-> w x v, and every
  // linear map R^3 -> R^3 has exactly one matrix in the standard basis. That
  // matrix is what this function returns. There is no convention to choose
  // here -- the matrix is forced by the definition of the cross product.
  //
  // Recovering the signs from scratch, if you ever forget them. By definition,
  //
  //   w x v = (w2*v3 - w3*v2,  w3*v1 - w1*v3,  w1*v2 - w2*v1).
  //
  // Row i of S holds the coefficients of output component i as a linear
  // combination of (v1, v2, v3). Read them straight off the expression above:
  //
  //   row 0  (w2*v3 - w3*v2)  ->  ( 0,  -w3,   w2)
  //   row 1  (w3*v1 - w1*v3)  ->  ( w3,   0,  -w1)
  //   row 2  (w1*v2 - w2*v1)  ->  (-w2,  w1,    0)
  //
  //   skew(w) =  |  0   -w3   w2 |      w3 sits at (1,0) and (0,1),
  //              |  w3   0   -w1 |      w1 at (2,1) and (1,2),
  //              | -w2   w1    0 |      w2 at (0,2) and (2,0).
  //
  // Memorise the three-line table, not the matrix. The table regenerates the
  // matrix in ten seconds including the signs; a memorised matrix does not
  // survive a year of not looking at it.
  //
  // Index trap: these are indices into a Vec3, not into a Vec6 twist. A twist
  // is [v; omega], so if the argument comes out of a twist it must be the omega
  // block (indices 3..5). Feeding it the v block compiles, runs, and quietly
  // computes the wrong thing.
  //
  // Formatting is disabled on purpose: the statement fits on one line and
  // clang-format would join it. The 3x3 layout is the only thing that makes the
  // sign pattern visible at a glance.
  // clang-format off
  S << 0.0,    -w.z(),  w.y(),
       w.z(),   0.0,   -w.x(),
      -w.y(),   w.x(),  0.0;
  // clang-format on
  // The result is antisymmetric (S^T == -S) because the cross product is
  // anticommutative: a x b == -(b x a). Transposing S swaps the roles of the
  // two arguments, which flips the sign of every entry. The zero diagonal is
  // the same statement at a x a: w x w == -(w x w) forces w x w == 0, and the
  // diagonal entries are exactly the coefficients that would produce it.
  return S;
}

// Rodrigues' rotation formula: the rotation by angle ||w|| about the axis
// w/||w||. This is `exp` restricted to the rotation subgroup SO(3); the
// six-dimensional exp() below reuses it verbatim for its rotational part.
//
// Where the formula comes from. The matrix exponential of skew(w) is by
// definition the series I + W + W^2/2! + W^3/3! + ... With W = theta*K for a
// unit-axis K, the identity K^3 == -K makes every power collapse onto K or K^2:
//
//   K^1 = K      K^2 = K^2      K^3 = -K      K^4 = -K^2      K^5 = K   ...
//
// Splitting the series by which of the two it lands on gives exactly the sine
// and cosine series, hence
//
//   R = I + sin(theta)*K + (1 - cos(theta))*K^2.
//
// The code below writes it in terms of W = theta*K instead of K, which is why
// the coefficients carry the extra factors of theta: A = sin(theta)/theta
// multiplies W, and B = (1 - cos theta)/theta^2 multiplies W^2. Doing it this
// way avoids ever dividing w by theta, so the small-angle case stays finite.
Mat3 rodrigues(const Vec3& w) {
  const double theta = w.norm();
  const Mat3 W = skew(w);

  // Why a threshold and not `if (theta == 0.0)`. Two separate reasons.
  //
  // The mathematical one: the formula degrades continuously, not at a point.
  // cos(theta) rounds to a value near 1, so `1 - cos(theta)` cancels -- the
  // subtraction itself is exact, but its operands already carry an absolute
  // error of ~eps while the true result is only theta^2/2. The relative error
  // of B is therefore ~2*eps/theta^2 and grows without bound as theta shrinks.
  // An equality test against zero leaves that whole band on the direct branch.
  //
  // The blunt one: exact zero is not even the only input that produces NaN.
  // For theta below ~1.5e-154 the product theta*theta underflows to exactly
  // zero, so B evaluates 0.0/0.0 for a perfectly ordinary nonzero input. A
  // threshold covers that for free; an equality test does not.
  //
  // Below the threshold we evaluate the Taylor series of the coefficients
  // around theta = 0 instead:
  //
  //   sin(theta)/theta        = 1 - theta^2/6  + theta^4/120  - ...
  //   (1 - cos theta)/theta^2 = 1/2 - theta^2/24 + theta^4/720 - ...
  //
  // Both are even in theta and analytic at 0, so truncating after the theta^2
  // term leaves a relative error of order theta^4.
  //
  // Why 1e-6 specifically. Work out the upper bound first, since it is the one
  // that actually binds. The truncated series must still be accurate to full
  // double precision, so the first dropped term has to sit below
  // eps = 2.2e-16 relative to the coefficient it corrects:
  //
  //   A: dropped theta^4/120, coefficient ~1
  //        theta^4/120 < eps   ->  theta < (120*eps)^(1/4) = 4.0e-4
  //   B: dropped theta^4/720, coefficient ~1/2
  //        theta^4/360 < eps   ->  theta < (360*eps)^(1/4) = 5.3e-4
  //
  // A is the binding one, so the series is exact to the last bit only up to
  // theta ~ 4e-4. (Accounting for the fact that A is multiplied by W, whose
  // norm is theta, buys about another factor of 5 -- but there is no reason to
  // spend it.)
  //
  // The lower bound is much weaker than it looks, because of a damping effect
  // worth internalising: B is only ever multiplied by W^2, whose norm is
  // theta^2. B's relative error ~2*eps/theta^2 therefore contributes an
  // ABSOLUTE error of ~eps to R regardless of how small theta gets. A large
  // relative error in a coefficient is harmless when the coefficient multiplies
  // something equally small. The one hard failure is theta == 0 exactly, where
  // the direct branch evaluates 0/0 and yields NaN. Below theta ~ sqrt(eps) =
  // 1.5e-8, `1 - cos(theta)` also collapses to exactly 0, which is still
  // harmless for R but makes the coefficient meaningless in isolation -- so
  // staying above that is the honest floor.
  //
  // The usable band is therefore roughly [1.5e-8, 4e-4] -- about four and a
  // half orders of magnitude. 1e-6 sits near its middle, some 1.8 orders above
  // the floor and 2.6 below the ceiling, and the same constant serves exp() and
  // log() below.
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

  // Note that theta == 0 exactly falls into the first branch and yields
  // A = 1, B = 0.5 with W = 0, hence R = I. No division ever happens.
  return Mat3::Identity() + A * W + B * W * W;
}

SE3::SE3() : R_(Mat3::Identity()), t_(Vec3::Zero()) {}

SE3::SE3(const Mat3& R, const Vec3& t) : R_(R), t_(t) {}

// The bottom row of M is ignored, not validated. The class documents the
// homogeneous-matrix precondition and deliberately does not enforce it at
// runtime; see the invariant note in se3.hpp.
SE3::SE3(const Mat4& M) : R_(M.block<3, 3>(0, 0)), t_(M.block<3, 1>(0, 3)) {}

SE3 SE3::identity() { return SE3(Mat3::Identity(), Vec3::Zero()); }

const Mat3& SE3::rotation() const { return R_; }
const Vec3& SE3::translation() const { return t_; }

// The homogeneous 4x4 form exists so that a rigid motion becomes an ordinary
// matrix product: the affine map p |-> R*p + t is not linear in p, but
// appending a constant 1 to p makes it linear in the 4-vector. That extra
// coordinate is what the last row maintains.
//
//        col 0..2        col 3
//      ┌─────────────┬───────────┐
//      │             │           │
//      │    R (3x3)  │   t (3x1) │   rows 0..2
//      │             │           │
//      ├─────────────┼───────────┤
//      │   0   0   0 │     1     │   row 3
//      └─────────────┴───────────┘
//
// Acting on (p; 1) reproduces (R*p + t; 1); acting on a DIRECTION written as
// (d; 0) reproduces (R*d; 0), i.e. the translation drops out automatically.
// That is the whole reason for the fourth coordinate -- points and directions
// transform differently and the trailing 0/1 encodes which one you have.
Mat4 SE3::matrix() const {
  Mat4 M = Mat4::Zero();     // starting from Zero() means the last row is already correct
  M.block<3, 3>(0, 0) = R_;  // 3x3 block anchored at (0,0)
  M.block<3, 1>(0, 3) = t_;  // 3x1 column anchored at (0,3)
  M(3, 3) = 1.0;             // the single nonzero entry the last row still needs
  return M;
}

// Closed-form inverse. Solve R*p + t == q for p: p == R^-1 * (q - t), and for a
// rotation R^-1 == R^T because the columns are orthonormal. So the inverse is
// (R^T, -R^T * t) -- no linear system, no pivoting, and R^T is exact (it is a
// permutation of the stored entries, not a computation).
//
// Inverting the 4x4 matrix() generically would give the same answer, but it
// costs an LU factorisation and introduces rounding where none is needed.
SE3 SE3::inverse() const { return SE3(R_.transpose(), -R_.transpose() * t_); }

// Composition, derived by multiplying the two homogeneous matrices and reading
// off the blocks: applying `other` first and then `*this` maps
// p |-> R*(R_o*p + t_o) + t == (R*R_o)*p + (R*t_o + t).
//
// Note the translation is NOT t + t_o: the right operand's translation is seen
// through the left operand's rotation. Forgetting that R_* is the single most
// common bug in a hand-written kinematic chain, and it is invisible whenever
// the test rotations happen to be identity.
SE3 SE3::operator*(const SE3& other) const {
  return SE3(R_ * other.rotation(), R_ * other.translation() + t_);
}

// Acts on a POINT. For a direction vector call rotation() * d instead -- a
// direction has no position, so translating it is meaningless.
Vec3 SE3::act(const Vec3& p) const { return R_ * p + t_; }

// Componentwise comparison with an ABSOLUTE tolerance, deliberately not
// Eigen::isApprox. Eigen's version is relative to the operand norms, so for a
// translation near the origin it demands far more precision than any test
// intends, while for a translation of magnitude 1e6 it accepts millimetres.
// Rigid transforms have a natural physical scale; absolute is the honest
// comparison here.
bool SE3::isApprox(const SE3& other, double tol) const {
  return (R_ - other.rotation()).cwiseAbs().maxCoeff() <= tol &&
         (t_ - other.translation()).cwiseAbs().maxCoeff() <= tol;
}

// Exponential map se(3) -> SE(3): integrate the constant twist x for one unit
// of time and return where you end up.
//
// The rotational part is just Rodrigues. The translational part is NOT simply
// v: while the body translates along v it is simultaneously rotating, so the
// straight-line displacement gets bent. Integrating that gives
//
//   t = V * v,   V = I + B*W + C*W^2
//
// with the same W = skew(omega) as before. V is the average of the rotation
// over the motion (V == I when omega == 0, recovering pure translation), and
// the resulting path is a screw motion -- rotation about an axis combined with
// translation along it.
//
// V is literally that average: V = integral of exp(s*W) ds over s in [0, 1].
// Writing exp(s*W) = I + (sin(s*theta)/theta)*W + ((1 - cos(s*theta))/theta^2)*W^2
// and integrating each term needs only two elementary integrals:
//
//   integral of sin(s*theta) ds over [0,1]     = (1 - cos theta)/theta
//   integral of (1 - cos(s*theta)) ds over [0,1] = (theta - sin theta)/theta
//
// which gives
//
//   B = (1 - cos theta)/theta^2,   C = (theta - sin theta)/theta^3.
//
// So B showing up in both rodrigues() and here is structural, not a
// coincidence: these coefficients form a chain, and integrating walks one step
// along it.
//
//   sin(theta)/theta  --int-->  (1 - cos theta)/theta^2  --int-->  (theta - sin theta)/theta^3
//        A in R                    B in R, B in V                        C in V
//
// Read the two rows together: R uses the first two links of the chain, V uses
// the last two. B is the overlap because V's W coefficient is one integration
// past R's W coefficient, which lands exactly on R's W^2 coefficient. The same
// chain continues, and log()'s V^-1 below is what you get by inverting it.
SE3 exp(const Vec6& x) {
  const Vec3 v = x.head<3>();  // components 0,1,2 -- translational part
  const Vec3 w = x.tail<3>();  // components 3,4,5 -- rotational part

  const double theta = w.norm();
  const Mat3 W = skew(w);

  // Same threshold reasoning as in rodrigues(). C is the more fragile of the
  // two: `theta - sin(theta)` cancels down to theta^3/6, so its relative error
  // is ~6*eps/theta^2 -- worse than B's, and for the same reason (the true
  // result is tiny compared to the operands being subtracted). The series
  //
  //   (theta - sin theta)/theta^3 = 1/6 - theta^2/120 + theta^4/5040 - ...
  //
  // has no such problem. Again, both branches are finite at theta == 0; the
  // threshold is about accuracy, not about avoiding a division by zero.
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

// Logarithmic map SE(3) -> se(3): the twist that, run for one unit of time,
// produces T. Inverse of exp() on the principal branch only -- exp is not
// injective (adding 2*pi to the rotation angle returns the same transform), so
// log() picks the representative with ||omega|| <= pi and log(exp(x)) == x
// holds only for ||omega|| < pi. That is a property of the map, not a defect.
//
// Structure of the computation, in order:
//   1. angle    from the trace of R
//   2. axis     from the antisymmetric part of R
//   3. v        by applying V^-1 to the translation
//
// Step 1 works because trace(R) == 1 + 2*cos(theta) for any rotation: read it
// off the Rodrigues form, using trace(K) == 0 and trace(K^2) == -2 for a unit
// axis. Step 2 works because R - R^T kills the symmetric part of Rodrigues and
// leaves 2*sin(theta)*K.
//
// THREE numerical traps live in this function. Two are handled below, the third
// is not:
//
//   (a) theta -> 0. Both `sin(theta)` in step 2 and `tan(theta/2)` in step 3
//       drive divisions that approach 0/0. Handled by the kSmall branches, for
//       exactly the reasons documented in rodrigues().
//
//   (b) acos() outside its domain. Mathematically (trace(R) - 1)/2 lies in
//       [-1, 1], but R has been through floating-point multiplication and has
//       drifted slightly off the group. A trace of 3 + 1e-16 yields an argument
//       of 1 + 5e-17, and std::acos of anything outside [-1, 1] returns NaN --
//       which then propagates through every remaining line silently. Handled by
//       the clamp; the clamp is not defensive programming, it is the correct
//       projection back onto the domain the input was supposed to be in.
//
//   (c) theta -> pi. NOT HANDLED. As theta approaches pi, sin(theta) -> 0 while
//       R - R^T -> 0 as well, so step 2 becomes 0/0 with no cancellation-free
//       reformulation available: the antisymmetric part simply stops carrying
//       the axis. At exactly theta == pi the axis is only determined up to
//       sign, since a rotation by +pi and by -pi about the same axis are the
//       same transform -- there is no continuous choice to make, only a
//       deterministic one. The standard fix reads the axis out of the SYMMETRIC
//       part instead: R + I == 2*n*n^T at theta == pi, so the axis is recovered
//       (up to sign) from the largest diagonal entry, with the sign fixed by
//       convention. Until that branch exists, log() returns garbage for
//       rotations near half a turn. The current tests never reach it because
//       they bound each omega component by 1.8, keeping ||omega|| below pi.
Vec6 log(const SE3& T) {
  const Mat3& R = T.rotation();
  const Vec3& t = T.translation();

  double cosTheta = (R.trace() - 1.0) / 2.0;
  cosTheta = std::clamp(cosTheta, -1.0, 1.0);  // trap (b)
  const double theta = std::acos(cosTheta);

  constexpr double kSmall = 1e-6;
  Vec3 w;
  if (theta < kSmall) {
    // Near identity, R differs from I only at first order, and the
    // antisymmetric part is skew(w) itself up to O(theta^3). Equivalently:
    // this is the general branch with the factor theta/(2*sin theta) replaced
    // by its limit 1/2.
    const Mat3 S = 0.5 * (R - R.transpose());
    w = Vec3(S(2, 1), S(0, 2), S(1, 0));
  } else {
    // R - R^T == 2*sin(theta)*K, so scaling by theta/(2*sin theta) turns it
    // into theta*K == skew(w), which is what we want to read out.
    const Mat3 S = (theta / (2.0 * std::sin(theta))) * (R - R.transpose());
    w = Vec3(S(2, 1), S(0, 2), S(1, 0));
  }
  // Reading (2,1), (0,2), (1,0) is the exact inverse of skew()'s layout above:
  // those are the three entries holding +w1, +w2, +w3. Picking the mirrored
  // positions (1,2), (2,0), (0,1) would negate the whole axis -- and would
  // still pass any test that only checks ||w||.

  // Undo the V factor that exp() applied to the translation: exp gives
  // t == V*v, so v == V^-1 * t. V^-1 has its own closed form with the same
  // I + a*W + b*W^2 shape:
  //
  //   V^-1 = I - W/2 + c*W^2,   c = (1 - (theta/2)*cot(theta/2)) / theta^2.
  //
  // Note the middle coefficient is exactly -1/2 for every theta, no series
  // needed. Only c is delicate: (theta/2)*cot(theta/2) -> 1 as theta -> 0, so
  // the numerator cancels to theta^2/12 and c -> 1/12. That limit is what the
  // small branch uses. (cot is spelled 1/tan here; std has no std::cot.)
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

// Adjoint: the linear map that re-expresses a twist in another frame. If x is a
// velocity as seen in frame B, then adjoint(T_AB) * x is the same physical
// velocity written in frame A. Equivalently, it is the derivative of the
// conjugation S |-> T*S*T^-1, which is why exp(adjoint(T)*x) == T*exp(x)*T^-1.
//
// Block structure, in the [v; omega] convention this library uses:
//
//        ┌─────────────┬─────────────┐
//        │      R      │  skew(t)*R  │   rows 0..2  ->  new v
//        ├─────────────┼─────────────┤
//        │      0      │      R      │   rows 3..5  ->  new omega
//        └─────────────┴─────────────┘
//          cols 0..2      cols 3..5
//            (v)          (omega)
//
// How to read it:
//   - Bottom-right R: angular velocity is a free vector, so changing frames
//     only rotates it. Nothing else can affect it.
//   - Bottom-left 0: a pure translational velocity produces no rotation. This
//     block is structurally zero, which is also why det(adjoint) == det(R)^2
//     == 1 and the matrix is invertible for every T.
//   - Top-left R: the translational part is rotated, same as any vector.
//   - Top-right skew(t)*R: the lever arm. A body spinning at omega about a
//     point offset by t drags the reference point along at t x (R*omega) --
//     the same cross product that appears in rigid-body velocity composition.
//     This is the block that makes the adjoint more than a pair of rotations.
//
// With the opposite [omega; v] convention the two off-diagonal blocks trade
// places, so this function cannot be lifted from another codebase without
// first checking which stacking order that codebase used.
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
