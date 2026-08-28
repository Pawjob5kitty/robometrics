// Tests for the SE(3) core. Each case names the invariant it pins and what a
// break in it would look like.

#include <doctest/doctest.h>
#include <rapidcheck.h>

// .cross() lives in Eigen/Geometry, not Eigen/Core -- without this include the
// file compiles and fails only at link time on an undefined reference.
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cstdint>
#include <vector>

#include "robometrics/se3.hpp"

namespace {

using robometrics::Mat3;
using robometrics::Mat4;
using robometrics::Mat6;
using robometrics::rodrigues;
using robometrics::SE3;
using robometrics::Vec3;
using robometrics::Vec6;

/// Tolerance for I1, per the spec.
constexpr double kTol = 1e-9;

/// Looser tolerance where a general 6x6 inverse is computed. The error there
/// grows with the condition number; 1e-9 would be optimistic.
constexpr double kTolLu = 1e-8;

/// log(exp(x)) == x holds only on the principal branch, i.e. for ||omega|| < pi.
/// We keep each omega component in [-1.8, 1.8], so ||omega|| <= 1.8*sqrt(3) =
/// 3.117..., which is below pi. Bounds on the components suffice; nothing needs
/// to be discarded.
constexpr double kOmegaComp = 1.8;
constexpr double kTransComp = 10.0;

/// rapidcheck has no fixed-range double generator; we build one from an integer
/// generator so the bounds are exact and shrinking makes sense.
rc::Gen<double> genInRange(double lo, double hi) {
  constexpr int64_t kSteps = 1000000;
  return rc::gen::map(rc::gen::inRange<int64_t>(0, kSteps + 1), [lo, hi](int64_t k) {
    return lo + (hi - lo) * (static_cast<double>(k) / static_cast<double>(kSteps));
  });
}

/// CAUTION: callable only from inside a rapidcheck property; `operator*` on a
/// generator outside a running property throws.
Vec6 arbitraryTwist() {
  Vec6 x;
  for (int i = 0; i < 3; ++i) {
    x(i) = *genInRange(-kTransComp, kTransComp);
  }
  for (int i = 3; i < 6; ++i) {
    x(i) = *genInRange(-kOmegaComp, kOmegaComp);
  }
  return x;
}

double maxAbsDiff(const Vec6& a, const Vec6& b) { return (a - b).cwiseAbs().maxCoeff(); }

/// The same range as the translational twist components above -- no
/// mathematical reason, just reusing an existing bound, since skew() has no
/// singularity or domain restriction of its own (it is defined for every finite
/// w).
Vec3 arbitraryVec3() {
  Vec3 v;
  for (int i = 0; i < 3; ++i) {
    v(i) = *genInRange(-kTransComp, kTransComp);
  }
  return v;
}

}  // namespace

TEST_CASE("sanity: doctest and rapidcheck are linked and run") {
  // Nothing to do with SE(3). If this test vanished, a green CI would only mean
  // that nothing ran.
  CHECK(rc::check("reversing twice is the identity", [](const std::vector<int>& v) {
    std::vector<int> w(v.rbegin(), v.rend());
    std::reverse(w.begin(), w.end());
    RC_ASSERT(w == v);
  }));
}

TEST_CASE("I1: log(exp(x)) == x") {
  CHECK(rc::check("log(exp(x)) == x for ||omega|| < pi", [] {
    const Vec6 x = arbitraryTwist();
    const Vec6 roundTrip = robometrics::log(robometrics::exp(x));
    RC_ASSERT(maxAbsDiff(roundTrip, x) < kTol);
  }));
}

TEST_CASE("exp(0) is the identity") {
  const Vec6 zero = Vec6::Zero();
  const SE3 T = robometrics::exp(zero);
  CHECK(T.isApprox(SE3::identity(), kTol));
}

TEST_CASE("exp(x) * exp(-x) is the identity") {
  CHECK(rc::check("exp(x) * exp(-x) == I", [] {
    const Vec6 x = arbitraryTwist();
    const Vec6 negX = -x;
    const SE3 T = robometrics::exp(x) * robometrics::exp(negX);
    RC_ASSERT(T.isApprox(SE3::identity(), kTol));
  }));
}

TEST_CASE("adjoint(T) is invertible") {
  CHECK(rc::check("adjoint(exp(x)) is invertible", [] {
    const SE3 T = robometrics::exp(arbitraryTwist());
    const Mat6 a = robometrics::adjoint(T);

    const Eigen::FullPivLU<Mat6> lu(a);
    RC_ASSERT(lu.isInvertible());

    const Mat6 aInv = lu.inverse();
    const Mat6 residual = a * aInv - Mat6::Identity();
    RC_ASSERT(residual.cwiseAbs().maxCoeff() < kTolLu);
  }));
}

TEST_CASE("skew(a) * b == a.cross(b)") {
  // This is NOT one invariant among many -- it is the DEFINITION of skew(). If
  // this test fails, either the sign or the index order in the matrix is wrong;
  // nothing else can break here, because skew() has no other logic.
  CHECK(rc::check("skew(a) * b matches the cross product", [] {
    const Vec3 a = arbitraryVec3();
    const Vec3 b = arbitraryVec3();
    const Vec3 lhs = robometrics::skew(a) * b;
    const Vec3 rhs = a.cross(b);
    RC_ASSERT((lhs - rhs).cwiseAbs().maxCoeff() < kTol);
  }));
}

TEST_CASE("skew(w) is antisymmetric") {
  // Antisymmetry is an algebraic consequence of the cross product being
  // anticommutative. It catches errors the test above may not on the first try
  // -- a swapped row/column where the signs happen to "line up" for some
  // specific a, b, but break the symmetry of the matrix as a whole.
  CHECK(rc::check("skew(w) + skew(w)^T == 0", [] {
    const Vec3 w = arbitraryVec3();
    const Mat3 s = robometrics::skew(w);
    const Mat3 sum = s + s.transpose();
    RC_ASSERT(sum.cwiseAbs().maxCoeff() < kTol);
  }));
}

TEST_CASE("skew(w) * w == 0") {
  // w is an eigenvector of skew(w) with eigenvalue 0 -- a special case of
  // w x w == 0. Catches the case where the matrix is antisymmetric "by
  // accident" (e.g. an index permutation preserving S^T == -S) but the
  // off-diagonal entries do not match the real cross product.
  CHECK(rc::check("skew(w) * w == 0", [] {
    const Vec3 w = arbitraryVec3();
    const Vec3 result = robometrics::skew(w) * w;
    RC_ASSERT(result.cwiseAbs().maxCoeff() < kTol);
  }));
}

TEST_CASE("skew is linear: skew(a + b) == skew(a) + skew(b)") {
  // skew() is a closed form linear in the components of w, so linearity should
  // hold trivially -- the test is mainly against implementations that try to be
  // clever (branching on special cases, normalising) and quietly break it.
  CHECK(rc::check("skew(a + b) == skew(a) + skew(b)", [] {
    const Vec3 a = arbitraryVec3();
    const Vec3 b = arbitraryVec3();
    const Mat3 lhs = robometrics::skew(a + b);
    const Mat3 rhs = robometrics::skew(a) + robometrics::skew(b);
    RC_ASSERT((lhs - rhs).cwiseAbs().maxCoeff() < kTol);
  }));
}

TEST_CASE("identity is the unit transform") {
  const SE3 I = SE3::identity();
  CHECK(I.rotation().isApprox(Mat3::Identity()));
  CHECK(I.translation().isZero());
}

TEST_CASE("matrix() assembles R and t into a 4x4") {
  const Mat3 R = Mat3::Identity();
  const Vec3 t(1.0, 2.0, 3.0);
  const SE3 T(R, t);
  const Mat4 M = T.matrix();

  const Mat3 topLeft = M.block<3, 3>(0, 0);
  const Vec3 rightCol = M.block<3, 1>(0, 3);

  CHECK(topLeft.isApprox(R));
  CHECK(rightCol.isApprox(t));
  CHECK(M(3, 0) == 0.0);
  CHECK(M(3, 1) == 0.0);
  CHECK(M(3, 2) == 0.0);
  CHECK(M(3, 3) == 1.0);
}

TEST_CASE("the Mat4 constructor is inverse to matrix()") {
  const Mat3 R = Mat3::Identity();
  const Vec3 t(1.0, 2.0, 3.0);
  const SE3 orig(R, t);

  const Mat4 M = orig.matrix();
  const SE3 back(M);

  CHECK(back.rotation().isApprox(R));
  CHECK(back.translation().isApprox(t));
}

TEST_CASE("act applies the rotation then the translation") {
  const Vec3 t(1.0, 2.0, 3.0);
  const SE3 T(Mat3::Identity(), t);
  const Vec3 p(10.0, 0.0, 0.0);

  const Vec3 result = T.act(p);
  const Vec3 expected(11.0, 2.0, 3.0);

  CHECK(result.isApprox(expected));
}

TEST_CASE("composing transforms") {
  const SE3 A(Mat3::Identity(), Vec3(1.0, 0.0, 0.0));
  const SE3 B(Mat3::Identity(), Vec3(0.0, 2.0, 0.0));
  const SE3 C = A * B;

  const Vec3 p(0.0, 0.0, 5.0);
  CHECK(C.act(p).isApprox(A.act(B.act(p))));
}

TEST_CASE("rodrigues matches Eigen AngleAxis") {
  const Vec3 axis = Vec3(1.0, 2.0, 3.0).normalized();
  const double theta = 0.7;
  const Vec3 w = axis * theta;

  const Eigen::AngleAxisd aa(theta, axis);
  CHECK(rodrigues(w).isApprox(aa.toRotationMatrix(), 1e-12));
}

TEST_CASE("rodrigues of the zero vector returns the identity") {
  const Mat3 R = robometrics::rodrigues(Vec3::Zero());
  CHECK(R.allFinite());  // no NaN
  CHECK(R.isApprox(Mat3::Identity(), 1e-12));
}

TEST_CASE("rodrigues is stable for very small theta") {
  const Vec3 w(1e-9, 0.0, 0.0);
  const Mat3 R = robometrics::rodrigues(w);
  CHECK(R.allFinite());
  CHECK(R.isApprox(Mat3::Identity(), 1e-8));
}

TEST_CASE("the default constructor is the identity") {
  const SE3 T;
  CHECK(T.isApprox(SE3::identity(), 1e-12));
}
TEST_CASE("adjoint of identity is the identity matrix") {
  const Mat6 A = robometrics::adjoint(SE3::identity());
  CHECK(A.isApprox(Mat6::Identity(), 1e-12));
}
// ---------------------------------------------------------------------------
// log() at theta -> pi (hardening)
// ---------------------------------------------------------------------------
//
// The existing "log(exp(x)) == x" property bounds each omega component at 1.8,
// so ||omega|| reaches at most 1.8*sqrt(3) ~ 3.117 and never enters the near-pi
// branch (theta > pi - 1e-3 ~ 3.1406). Everything below covers exactly that
// gap. NONE of it duplicates the existing test: it lives above 3.117.
//
// A note on tolerances near pi. theta is recovered as acos((trace R - 1)/2),
// whose derivative is -1/sin(theta), so an eps error in the trace becomes an
// eps/sin(pi - theta) ~ eps/(pi - theta) error in the angle -- and hence in the
// round-trip exp(log(R)). This is inherent to angle-from-trace for any R given
// in doubles; it is the same for the old general branch and is NOT introduced
// by the symmetric-part axis extraction. The bounds below reflect that floor.

namespace {
const double kPiTest = std::acos(-1.0);

/// A unit axis that is deliberately NOT aligned with a basis vector, so a
/// component-swap or sign bug in the axis extraction cannot hide.
Vec3 arbitraryUnitAxis() {
  Vec3 a(*genInRange(-1.0, 1.0), *genInRange(-1.0, 1.0), *genInRange(-1.0, 1.0));
  if (a.norm() < 0.2) {
    a = Vec3(0.3, -0.6, 0.74);  // keep it off the axes when the draw is tiny
  }
  return a.normalized();
}
}  // namespace

TEST_CASE("log at exactly theta = pi recovers +-pi*n for several off-axis axes") {
  // The headline hole: the old code divided the vanishing R - R^T and returned
  // garbage here. Each axis is off the basis vectors so a wrong index or sign
  // in the new symmetric-part extraction shows up. The axis is only defined up
  // to sign at pi (a +pi and a -pi turn are the same rotation), so w must be
  // pi*n OR -pi*n.
  const Vec3 axes[] = {Vec3(0.0, 0.0, 1.0),  Vec3(1.0, 1.0, 1.0),  Vec3(1.0, -2.0, 3.0),
                       Vec3(-3.0, 1.0, 0.5), Vec3(2.0, 2.0, -1.0), Vec3(-1.0, -1.0, 4.0)};
  for (const Vec3& raw : axes) {
    const Vec3 n = raw.normalized();
    Vec6 x;
    x.head<3>() = Vec3(0.2, -0.1, 0.4);  // some translation, to exercise V^-1 too
    x.tail<3>() = kPiTest * n;
    const SE3 R = robometrics::exp(x);
    const Vec6 y = robometrics::log(R);

    // The transform must reconstruct -- this is exactly what the old garbage
    // broke. The 1e-6 bound is the sqrt(eps) floor of angle-from-trace AT pi:
    // the fp trace lands just above -1, so acos returns pi - O(sqrt(eps)) and
    // the round-trip cannot be tighter. (Old code returned NaN here.)
    CHECK(robometrics::exp(y).isApprox(R, 1e-6));

    // The axis DIRECTION comes from the symmetric part and is accurate to ~eps,
    // independent of that angle floor; check it separately from the angle. At
    // pi it is defined only up to sign, so +n or -n.
    const Vec3 w = y.tail<3>();
    const Vec3 axis = w.normalized();
    CHECK(std::min((axis - n).cwiseAbs().maxCoeff(), (axis + n).cwiseAbs().maxCoeff()) < 1e-6);
    CHECK(w.norm() <= kPiTest + 1e-6);
    CHECK(w.norm() > kPiTest - 1e-4);

    // NB: y.head<3>() is deliberately NOT compared to x.head<3>(). When the
    // axis comes back as -n, the twist (v, w) is a different but equally valid
    // representative whose exp is the same R -- so the translational component
    // legitimately differs. exp(log(R)) == R above is the invariant that holds.
  }
}

TEST_CASE("log round-trips for theta approaching pi from below and from above") {
  // "From above" means an input angle just over pi: exp wraps it to a turn of
  // pi - delta about the opposite axis, and log must still invert it. Catches a
  // near-pi branch that only works at the exact seam. The tolerance follows the
  // eps/(pi - theta) angle-conditioning floor described above.
  const Vec3 n = Vec3(1.0, -2.0, 3.0).normalized();
  const double deltas[] = {1e-4, 1e-7};
  const double signs[] = {-1.0, +1.0};  // pi - delta (below) and pi + delta (above)
  for (const double d : deltas) {
    for (const double sgn : signs) {
      const double angle = kPiTest + sgn * d;
      Vec6 x;
      x.head<3>() = Vec3(0.5, 0.2, -0.3);
      x.tail<3>() = angle * n;
      const SE3 R = robometrics::exp(x);
      const Vec6 y = robometrics::log(R);
      CHECK(robometrics::exp(y).isApprox(R, 1e-8));  // 1e-8 covers the 1e-7 case's floor
      CHECK(y.tail<3>().norm() <= kPiTest + 1e-12);  // principal branch: ||w|| <= pi
    }
  }
}

TEST_CASE("log is continuous across the near-pi branch seam") {
  // Straddle the switch at theta = pi - 1e-3: one point uses the general
  // branch, the other the symmetric branch. Their recovered axes must agree.
  // A mis-placed threshold, or a sign flip in the near-pi branch, makes the
  // axis jump (typically to -n), which this catches -- a plain round-trip test
  // would not, because exp(log(R)) == R holds for either sign only AT pi.
  const Vec3 n = Vec3(0.4, 0.7, -0.6).normalized();
  const double seam = kPiTest - 1e-3;
  const double h = 1e-6;

  auto axisAt = [&](double angle) {
    Vec6 x;
    x.head<3>() = Vec3::Zero();
    x.tail<3>() = angle * n;
    const Vec3 w = robometrics::log(robometrics::exp(x)).tail<3>();
    return (w.array() / w.norm()).matrix().eval();  // unit axis
  };

  const Vec3 below = axisAt(seam - h);  // general branch
  const Vec3 above = axisAt(seam + h);  // symmetric branch
  CHECK((above - below).cwiseAbs().maxCoeff() < 1e-6);
}

TEST_CASE("log(exp(x)) == x property in the near-pi band the old test never reaches") {
  // A second property test, deliberately NOT the existing one renamed: it draws
  // theta in [pi - 0.4, pi - 1e-6], which sits entirely above the old test's
  // ceiling of ~3.117 and straddles the branch seam at pi - 1e-3. The 1e-6 floor
  // on pi - theta keeps the angle-conditioning error (~eps/(pi - theta)) under
  // ~1e-10, comfortably inside the 1e-8 bound.
  CHECK(rc::check("exp(log(exp(x))) == exp(x) for theta near pi", [] {
    const Vec3 n = arbitraryUnitAxis();
    const double theta = *genInRange(kPiTest - 0.4, kPiTest - 1e-6);
    Vec6 x;
    for (int i = 0; i < 3; ++i) {
      x(i) = *genInRange(-kTransComp, kTransComp);
    }
    x.tail<3>() = theta * n;

    const SE3 R = robometrics::exp(x);
    const Vec6 y = robometrics::log(R);
    RC_ASSERT(robometrics::exp(y).isApprox(R, 1e-8));
    RC_ASSERT(y.tail<3>().norm() <= kPiTest + 1e-12);
  }));
}
