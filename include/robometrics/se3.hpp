#pragma once

#include <Eigen/Core>

/// Rigid transforms in R^3 -- the group SE(3) and its Lie algebra se(3).
///
/// NAME CLASH WARNING: `robometrics::exp` and `robometrics::log` shadow
/// `std::exp` / `std::log`. Always call them qualified. An unqualified `exp(x)`
/// in a translation unit that also says `using namespace std;` compiles and
/// calls something else entirely.
namespace robometrics {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;

/// A twist, stacked as [v; omega] -- translational part first, rotational
/// second. The opposite convention [omega; v] is just as common in the
/// literature; the two cannot be mixed, because swapping them transposes the
/// block structure of the adjoint.
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

/// Default absolute tolerance for comparing transforms.
inline constexpr double kDefaultTol = 1e-9;

/// Cross-product matrix (also "skew-symmetric matrix" or "hat operator", often
/// written [w]_x): the linear map v |-> w x v expressed as a matrix product.
///
///   skew(w) = |  0   -w3   w2 |
///             |  w3   0   -w1 |
///             | -w2   w1   0  |
///
/// where w = (w1, w2, w3).
///
/// Where it comes from: for a fixed w, the cross product w x (.) is a LINEAR
/// map R^3 -> R^3 -- visible straight from the definition, since every
/// component of w x v is a linear combination of the components of v. Every
/// linear map R^3 -> R^3 has exactly one matrix in a given basis, and the
/// matrix of v |-> w x v is precisely skew(w). This is not an approximation or
/// a modelling choice; it is the only way to write a cross product as a matrix.
///
/// Why that is useful: once the cross product is a matrix, it obeys linear
/// algebra -- it composes with other matrices, transposes, and can be fed to
/// the matrix exponential. That is exactly what Rodrigues' formula needs
/// (R = I + sin(theta)K + (1 - cos theta)K^2 with K = skew(axis)), and what
/// log() needs when extracting a rotation axis out of R. skew() is therefore
/// the function everything else in this header stands on.
///
/// Pre:  the components of w are finite (no NaN, no Inf).
/// Post: skew(w) * v == w.cross(v) for every v -- this is the defining
///       property; everything below follows from it.
///
/// Invariants:
///   - skew(w) is antisymmetric: skew(w).transpose() == -skew(w).
///   - skew(w) * w == 0 -- w is an eigenvector with eigenvalue 0 (the cross
///     product of a vector with itself is zero).
///   - linear in w: skew(a + b) == skew(a) + skew(b),
///                  skew(c * w) == c * skew(w) for a scalar c.
///
/// CAUTION: skew() takes a bare Vec3, not a slice of a twist. A twist Vec6 is
/// [v; omega] (see above) -- if the argument comes out of a twist it must be
/// omega (indices 3..5), not v (indices 0..2). Getting that wrong produces code
/// that compiles and quietly computes something else.
Mat3 skew(const Vec3& w);

/// Rodrigues' rotation formula: the rotation by angle ||w|| about the axis
/// w/||w||, in closed form. This is exp() restricted to the rotation subgroup
/// SO(3); the full exp() below reuses it verbatim for its rotational part.
///
/// The parameter w is a ROTATION VECTOR (also called axis-angle): its direction
/// is the axis, its magnitude is the angle in radians. It is not a unit vector
/// and must not be normalised by the caller -- normalising would throw away the
/// angle. Rotation vectors are the natural coordinates here because they are
/// exactly the rotational half of a twist.
///
///   R = I + A*W + B*W^2,   W = skew(w),
///   A = sin(theta)/theta,  B = (1 - cos theta)/theta^2,  theta = ||w||
///
/// Where it comes from: the matrix exponential of W is the series
/// I + W + W^2/2! + ..., and for a unit axis K the identity K^3 == -K collapses
/// every power back onto K or K^2. Splitting the series by which of the two it
/// lands on reproduces the sine and cosine series term for term. Writing the
/// result in W rather than K is what puts theta into the denominators; the
/// payoff is that w is never divided by its own norm, so the small-angle case
/// stays finite. See the implementation for the numerics.
///
/// Right-handed convention: positive theta rotates counter-clockwise when
/// looking down the axis toward the origin, i.e. the same handedness as the
/// cross product that skew() encodes. First order in theta this reads
/// R ~= I + skew(w), which is the cheapest way to check the sign by hand.
///
/// Pre:  components of w are finite. Any magnitude is accepted -- ||w|| > pi
///       simply wraps, since rotating by theta and by theta + 2*pi are the same
///       transform.
/// Post: R^T * R == I and det(R) == +1, i.e. the result satisfies the SE3
///       rotation invariant.
///
/// Must hold:
///   - rodrigues(Vec3::Zero()) == Mat3::Identity()
///   - rodrigues(-w) == rodrigues(w).transpose()   (inverse rotation)
///   - rodrigues(w) * w == w                       (the axis is fixed)
///   - rodrigues(s*w) * rodrigues(u*w) == rodrigues((s+u)*w) for scalars s, u;
///     rotations about DIFFERENT axes do not commute and no such law applies
///
/// Numerics: both coefficients divide by a power of theta and are 0/0 at
/// theta == 0. Below a threshold they must come from a series expansion, not
/// from a special case for exactly zero.
Mat3 rodrigues(const Vec3& w);

/// A homogeneous transform in SE(3): a proper rotation composed with a
/// translation.
///
/// Instance invariant:
///   - rotation() is orthonormal, i.e. R^T * R == I
///   - det(rotation()) == +1, i.e. a proper rotation, not a reflection
///
/// The invariant is NOT checked at runtime -- the constructors assume it. After
/// a long chain of multiplications the rotation drifts numerically off the
/// group; re-orthonormalising is the caller's responsibility, not the class's.
class SE3 {
public:
  /// Post: *this == identity().
  SE3();

  /// Pre:  R satisfies the invariant above, t is finite.
  /// Post: rotation() == R, translation() == t.
  SE3(const Mat3& R, const Vec3& t);

  /// Pre:  T is a valid homogeneous matrix -- its top-left 3x3 block satisfies
  ///       the invariant and its last row is [0 0 0 1].
  /// Post: matrix() == T.
  explicit SE3(const Mat4& T);

  /// The group identity: R == I, t == 0.
  /// Post: identity() * A == A * identity() == A for every A.
  static SE3 identity();

  const Mat3& rotation() const;
  const Vec3& translation() const;

  /// Post: the last row of the result is exactly [0 0 0 1], not merely close.
  Mat4 matrix() const;

  /// Post: (*this) * inverse() == inverse() * (*this) == identity(), to
  ///       numerical precision.
  /// Note: the SE(3) inverse has a closed form. Inverting the general 4x4
  ///       matrix gives the same answer, but costs more and is less accurate.
  SE3 inverse() const;

  /// Composition, left to right as with matrices. Not commutative.
  /// Post: (A * B).act(p) == A.act(B.act(p)) for every point p.
  SE3 operator*(const SE3& rhs) const;

  /// Acts on a POINT -- applies both the rotation and the translation.
  /// A direction vector must not be translated; use rotation() * v for that.
  Vec3 act(const Vec3& p) const;

  /// Componentwise comparison with an ABSOLUTE tolerance.
  /// Deliberately not Eigen::isApprox, which is relative and therefore gives
  /// surprising answers for translations near the origin.
  bool isApprox(const SE3& other, double tol = kDefaultTol) const;

private:
  Mat3 R_;
  Vec3 t_;
};

/// Exponential map se(3) -> SE(3).
///
/// Reads a twist as a constant screw motion integrated for one unit of time, so
/// exp(x) is where you end up after following x.
///
/// Pre:  every component of twist is finite (no NaN, no Inf).
/// Post: the result satisfies the SE3 invariant.
///
/// Must hold:
///   - exp(Vec6::Zero()) == identity()
///   - exp(-x) == exp(x).inverse()
///   - exp(s*x) * exp(u*x) == exp((s+u)*x) for scalars s, u (the one-parameter
///     subgroup law); in general, though, exp(x) * exp(y) != exp(x + y)
///
/// Numerics: the naive form divides by the norm of omega and loses significant
/// digits as ||omega|| -> 0, well before it would divide by zero. Below a
/// threshold it needs a series expansion, not a special case for exactly zero.
SE3 exp(const Vec6& twist);

/// Logarithmic map SE(3) -> se(3), inverse to exp on the principal branch.
///
/// Pre:  T satisfies the SE3 invariant.
/// Post: ||log(T).tail<3>()|| <= pi.
///       exp(log(T)) == T always holds.
///       log(exp(x)) == x holds ONLY for ||x.tail<3>()|| < pi. Beyond that the
///       angle wraps into (-pi, pi] and the identity fails -- that is not an
///       implementation defect, it is a property of exp, which is not
///       injective.
///
/// Singularities: at ||omega|| == pi the axis is determined only up to sign, so
/// the choice must be deterministic. For ||omega|| -> 0 the same remark about
/// series expansion applies as for exp.
Vec6 log(const SE3& T);

/// Adjoint -- the linear map that carries a twist from one frame to another.
///
/// The block structure follows the [v; omega] convention. Under [omega; v] the
/// blocks trade places, so this function cannot be lifted from another codebase
/// without first checking that codebase's convention.
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
