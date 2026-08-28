#pragma once

#include <Eigen/Core>

/// Rigid transforms in R^3 -- the group SE(3) and its Lie algebra se(3).
///
/// NAME CLASH: `robometrics::exp` and `robometrics::log` shadow `std::exp` and
/// `std::log`. Always call them qualified. An unqualified `exp(x)` in a
/// translation unit that also says `using namespace std;` compiles and calls
/// something else.
namespace robometrics {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;

/// A twist, stacked [v; omega] -- translational part first. The opposite
/// convention [omega; v] is just as common in the literature; the two cannot be
/// mixed, because swapping them transposes the block structure of the adjoint.
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

/// Default absolute tolerance for comparing transforms.
inline constexpr double kDefaultTol = 1e-9;

/// Cross-product matrix, the linear map v |-> w x v written as a matrix.
///
///   skew(w) = |  0   -w3   w2 |
///             |  w3   0   -w1 |
///             | -w2   w1   0  |
///
/// Post: skew(w) * v == w.cross(v) for every v.
///
/// Invariants: antisymmetric, skew(w) * w == 0, and linear in w.
///
/// CAUTION: takes a bare Vec3, not a slice of a twist. If the argument comes
/// out of a Vec6 it must be the omega block (indices 3..5), not v (0..2).
/// Getting that wrong compiles and quietly computes something else.
Mat3 skew(const Vec3& w);

/// Rodrigues' formula: rotation by angle ||w|| about the axis w/||w||. This is
/// exp() restricted to SO(3); the full exp() reuses it for its rotational part.
///
/// w is a ROTATION VECTOR (axis-angle): direction is the axis, magnitude is the
/// angle in radians. It is not a unit vector and must not be normalised by the
/// caller -- normalising throws the angle away.
///
/// Right-handed: positive theta rotates counter-clockwise looking down the axis
/// toward the origin. To first order R ~= I + skew(w), which is the cheapest
/// way to check the sign by hand.
///
/// Pre:  components of w are finite. Any magnitude is accepted; ||w|| > pi
///       simply wraps.
/// Post: R^T * R == I and det(R) == +1.
///
/// Must hold:
///   - rodrigues(Vec3::Zero()) == Mat3::Identity()
///   - rodrigues(-w) == rodrigues(w).transpose()
///   - rodrigues(w) * w == w
///   - rodrigues(s*w) * rodrigues(u*w) == rodrigues((s+u)*w) for scalars s, u;
///     rotations about DIFFERENT axes do not commute and no such law applies
///
/// Numerics: both coefficients divide by a power of theta and are 0/0 at
/// theta == 0. Below a threshold they come from a series expansion, not from a
/// special case for exactly zero -- see the implementation.
Mat3 rodrigues(const Vec3& w);

/// A homogeneous transform: a proper rotation composed with a translation.
///
/// Instance invariant: rotation() is orthonormal with det == +1.
///
/// The invariant is NOT checked at runtime -- the constructors assume it. After
/// a long chain of multiplications the rotation drifts off the group;
/// re-orthonormalising is the caller's responsibility, not the class's.
class SE3 {
public:
  /// Post: *this == identity().
  SE3();

  /// Pre: R satisfies the invariant above, t is finite.
  SE3(const Mat3& R, const Vec3& t);

  /// Pre: T is a valid homogeneous matrix. The last row is not validated.
  explicit SE3(const Mat4& T);

  static SE3 identity();

  const Mat3& rotation() const;
  const Vec3& translation() const;

  /// Post: the last row of the result is exactly [0 0 0 1], not merely close.
  Mat4 matrix() const;

  /// Post: (*this) * inverse() == identity(), to numerical precision.
  SE3 inverse() const;

  /// Composition, left to right as with matrices. Not commutative.
  /// Post: (A * B).act(p) == A.act(B.act(p)) for every point p.
  SE3 operator*(const SE3& rhs) const;

  /// Acts on a POINT -- applies both the rotation and the translation. A
  /// direction must not be translated; use rotation() * v for that.
  Vec3 act(const Vec3& p) const;

  /// Componentwise comparison with an ABSOLUTE tolerance. Deliberately not
  /// Eigen::isApprox, which is relative and therefore surprising near the
  /// origin.
  bool isApprox(const SE3& other, double tol = kDefaultTol) const;

private:
  Mat3 R_;
  Vec3 t_;
};

/// Exponential map se(3) -> SE(3): the twist read as a constant screw motion
/// integrated for one unit of time.
///
/// Pre:  every component of twist is finite.
/// Post: the result satisfies the SE3 invariant.
///
/// Must hold:
///   - exp(Vec6::Zero()) == identity()
///   - exp(-x) == exp(x).inverse()
///   - exp(s*x) * exp(u*x) == exp((s+u)*x) for scalars s, u; in general,
///     though, exp(x) * exp(y) != exp(x + y)
SE3 exp(const Vec6& twist);

/// Logarithmic map SE(3) -> se(3), inverse to exp on the principal branch.
///
/// Pre:  T satisfies the SE3 invariant.
/// Post: ||log(T).tail<3>()|| <= pi.
///       exp(log(T)) == T always holds.
///       log(exp(x)) == x holds ONLY for ||x.tail<3>()|| < pi. Beyond that the
///       angle wraps and the identity fails -- a property of exp, which is not
///       injective, not an implementation defect.
///
/// The singularity at pi is handled: the general axis extraction uses the
/// antisymmetric part of R, which vanishes there, so within a threshold of pi
/// the axis is taken from the symmetric part (R + I == 2*n*n^T) and its sign
/// recovered from the antisymmetric part. See the implementation.
Vec6 log(const SE3& T);

/// Adjoint -- the linear map that carries a twist from one frame to another.
///
/// The block structure follows the [v; omega] convention. Under [omega; v] the
/// blocks trade places, so this cannot be lifted from another codebase without
/// checking that codebase's convention.
///
/// Pre:  T satisfies the SE3 invariant.
/// Post: det == 1, so the matrix is invertible for every T.
///
/// Must hold:
///   - adjoint(SE3::identity()) == Mat6::Identity()
///   - adjoint(A * B) == adjoint(A) * adjoint(B)
///   - adjoint(T.inverse()) == adjoint(T).inverse()
///   - exp(adjoint(T) * x) == T * exp(x) * T.inverse()
Mat6 adjoint(const SE3& T);

}  // namespace robometrics
