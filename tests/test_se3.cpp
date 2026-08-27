// Tests for the SE(3) core. Each case names the invariant it pins and what a
// break in it would look like.

#include <doctest/doctest.h>
#include <rapidcheck.h>

// .cross() nežije v Eigen/Core, ale v Eigen/Geometry — bez tohohle includu
// se soubor přeloží a spadne až při linkování na undefined reference.
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

/// Tolerance pro I1 podle zadání.
constexpr double kTol = 1e-9;

/// Volnější tolerance tam, kde se počítá obecná inverze 6x6. Chyba tam roste
/// s číslem podmíněnosti, 1e-9 by bylo optimistické.
constexpr double kTolLu = 1e-8;

/// log(exp(x)) == x platí jen na hlavní větvi, tj. pro ||omega|| < pi. Držíme
/// každou složku omega v <-1.8, 1.8>, takže ||omega|| <= 1.8*sqrt(3) = 3.117...
/// což je pod pi. Meze na složkách stačí, není potřeba nic zahazovat.
constexpr double kOmegaComp = 1.8;
constexpr double kTransComp = 10.0;

/// rapidcheck nemá generátor doublů s pevným rozsahem; postavíme ho z
/// celočíselného, aby meze byly přesné a shrinking dával smysl.
rc::Gen<double> genInRange(double lo, double hi) {
  constexpr int64_t kSteps = 1000000;
  return rc::gen::map(rc::gen::inRange<int64_t>(0, kSteps + 1), [lo, hi](int64_t k) {
    return lo + (hi - lo) * (static_cast<double>(k) / static_cast<double>(kSteps));
  });
}

/// POZOR: volatelné jen zevnitř rapidcheck property, `operator*` na generátoru
/// mimo běžící property vyhodí výjimku.
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

/// Rozsah stejný jako translační složky twistu výše — žádný matematický
/// důvod, jen recyklace existující meze, protože skew() nemá vlastní
/// singularitu ani doménové omezení (je definovaná pro každé konečné w).
Vec3 arbitraryVec3() {
  Vec3 v;
  for (int i = 0; i < 3; ++i) {
    v(i) = *genInRange(-kTransComp, kTransComp);
  }
  return v;
}

}  // namespace

TEST_CASE("sanity: doctest i rapidcheck jsou slinkované a běží") {
  // Nemá to co dělat s SE(3). Kdyby tenhle test zmizel, zelená CI by
  // znamenala jen to, že se nic nespustilo.
  CHECK(rc::check("dvojité obrácení pořadí je identita", [](const std::vector<int>& v) {
    std::vector<int> w(v.rbegin(), v.rend());
    std::reverse(w.begin(), w.end());
    RC_ASSERT(w == v);
  }));
}

TEST_CASE("I1: log(exp(x)) == x") {
  CHECK(rc::check("log(exp(x)) == x pro ||omega|| < pi", [] {
    const Vec6 x = arbitraryTwist();
    const Vec6 roundTrip = robometrics::log(robometrics::exp(x));
    RC_ASSERT(maxAbsDiff(roundTrip, x) < kTol);
  }));
}

TEST_CASE("exp(0) je identita") {
  const Vec6 zero = Vec6::Zero();
  const SE3 T = robometrics::exp(zero);
  CHECK(T.isApprox(SE3::identity(), kTol));
}

TEST_CASE("exp(x) * exp(-x) je identita") {
  CHECK(rc::check("exp(x) * exp(-x) == I", [] {
    const Vec6 x = arbitraryTwist();
    const Vec6 negX = -x;
    const SE3 T = robometrics::exp(x) * robometrics::exp(negX);
    RC_ASSERT(T.isApprox(SE3::identity(), kTol));
  }));
}

TEST_CASE("adjoint(T) je regulární") {
  CHECK(rc::check("adjoint(exp(x)) je invertovatelná", [] {
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
  // Tohle NENÍ jen jeden z mnoha invariantů — je to DEFINICE skew(). Pokud
  // tenhle test padá, je špatně buď znaménko, nebo pořadí indexů v matici;
  // nic jiného se tu rozbít nedá, protože skew() žádnou jinou logiku nemá.
  CHECK(rc::check("skew(a) * b odpovídá cross productu", [] {
    const Vec3 a = arbitraryVec3();
    const Vec3 b = arbitraryVec3();
    const Vec3 lhs = robometrics::skew(a) * b;
    const Vec3 rhs = a.cross(b);
    RC_ASSERT((lhs - rhs).cwiseAbs().maxCoeff() < kTol);
  }));
}

TEST_CASE("skew(w) je antisymetrická") {
  // Antisymetrie je algebraický důsledek antikomutativity cross productu.
  // Chytí konkrétně chyby, které test výše nemusí odhalit na první pokus —
  // třeba prohozený řádek/sloupec, kde by se náhoda ve znaménkách "srovnala"
  // pro některé konkrétní a, b, ale porušila by symetrii matice jako celku.
  CHECK(rc::check("skew(w) + skew(w)^T == 0", [] {
    const Vec3 w = arbitraryVec3();
    const Mat3 s = robometrics::skew(w);
    const Mat3 sum = s + s.transpose();
    RC_ASSERT(sum.cwiseAbs().maxCoeff() < kTol);
  }));
}

TEST_CASE("skew(w) * w == 0") {
  // w je vlastní vektor skew(w) s vlastním číslem 0 — speciální případ
  // w×w == 0. Chytí chybu, kdy je matice antisymetrická "náhodou" (např.
  // permutací indexů, která zachová S^T == -S), ale mimodiagonální prvky
  // neodpovídají skutečnému cross productu.
  CHECK(rc::check("skew(w) * w == 0", [] {
    const Vec3 w = arbitraryVec3();
    const Vec3 result = robometrics::skew(w) * w;
    RC_ASSERT(result.cwiseAbs().maxCoeff() < kTol);
  }));
}

TEST_CASE("skew je lineární: skew(a + b) == skew(a) + skew(b)") {
  // skew() je uzavřený vzorec lineární ve složkách w, takže linearita by
  // měla platit triviálně — test je tu hlavně proti implementacím, co by
  // se snažily být chytré (větvení na speciální případy, normalizace) a
  // linearitu tiše porušily.
  CHECK(rc::check("skew(a + b) == skew(a) + skew(b)", [] {
    const Vec3 a = arbitraryVec3();
    const Vec3 b = arbitraryVec3();
    const Mat3 lhs = robometrics::skew(a + b);
    const Mat3 rhs = robometrics::skew(a) + robometrics::skew(b);
    RC_ASSERT((lhs - rhs).cwiseAbs().maxCoeff() < kTol);
  }));
}

TEST_CASE("identity je jednotkova transformace") {
  const SE3 I = SE3::identity();
  CHECK(I.rotation().isApprox(Mat3::Identity()));
  CHECK(I.translation().isZero());
}

TEST_CASE("matrix() sklada R a t do 4x4") {
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

TEST_CASE("konstruktor z Mat4 je inverzni k matrix()") {
  const Mat3 R = Mat3::Identity();
  const Vec3 t(1.0, 2.0, 3.0);
  const SE3 orig(R, t);

  const Mat4 M = orig.matrix();
  const SE3 back(M);

  CHECK(back.rotation().isApprox(R));
  CHECK(back.translation().isApprox(t));
}

TEST_CASE("act aplikuje rotaci a pak translaci") {
  const Vec3 t(1.0, 2.0, 3.0);
  const SE3 T(Mat3::Identity(), t);
  const Vec3 p(10.0, 0.0, 0.0);

  const Vec3 result = T.act(p);
  const Vec3 expected(11.0, 2.0, 3.0);

  CHECK(result.isApprox(expected));
}

TEST_CASE("skladani transformaci") {
  const SE3 A(Mat3::Identity(), Vec3(1.0, 0.0, 0.0));
  const SE3 B(Mat3::Identity(), Vec3(0.0, 2.0, 0.0));
  const SE3 C = A * B;

  const Vec3 p(0.0, 0.0, 5.0);
  CHECK(C.act(p).isApprox(A.act(B.act(p))));
}

TEST_CASE("rodrigues odpovida Eigen AngleAxis") {
  const Vec3 axis = Vec3(1.0, 2.0, 3.0).normalized();
  const double theta = 0.7;
  const Vec3 w = axis * theta;

  const Eigen::AngleAxisd aa(theta, axis);
  CHECK(rodrigues(w).isApprox(aa.toRotationMatrix(), 1e-12));
}

TEST_CASE("rodrigues pro nulovy vektor vraci identitu") {
  const Mat3 R = robometrics::rodrigues(Vec3::Zero());
  CHECK(R.allFinite());  // zadne NaN
  CHECK(R.isApprox(Mat3::Identity(), 1e-12));
}

TEST_CASE("rodrigues pro velmi male theta je stabilni") {
  const Vec3 w(1e-9, 0.0, 0.0);
  const Mat3 R = robometrics::rodrigues(w);
  CHECK(R.allFinite());
  CHECK(R.isApprox(Mat3::Identity(), 1e-8));
}

TEST_CASE("vychozi konstruktor je identita") {
  const SE3 T;
  CHECK(T.isApprox(SE3::identity(), 1e-12));
}
TEST_CASE("adjoint identity je jednotkova matice") {
  const Mat6 A = robometrics::adjoint(SE3::identity());
  CHECK(A.isApprox(Mat6::Identity(), 1e-12));
}