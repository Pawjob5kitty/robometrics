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