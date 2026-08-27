// Testy forward kinematics.
//
// Struktura je zamerne stejna jako u SE(3): nejdriv pripady s rucne spocitanym
// vysledkem (ty rikaji, jestli je vzorec spravne), pak invarianty (ty chyti
// regrese, na ktere konkretni cisla nestaci).

#include <doctest/doctest.h>

#include <Eigen/Core>
#include <cmath>
#include <string>
#include <vector>

#include "robometrics/kinematics.hpp"
#include "robometrics/urdf.hpp"

namespace {

using robometrics::forwardKinematics;
using robometrics::forwardKinematicsAll;
using robometrics::Mat3;
using robometrics::Robot;
using robometrics::SE3;
using robometrics::Vec3;

/// Volnejsi nez kDefaultTol, ze stejneho duvodu jako v test_urdf.cpp: rotace
/// jdou pres rodrigues(), takze cos(pi/2) vyjde 6.1e-17 misto nuly. Rad chyby
/// je 1e-16, takze 1e-12 ma ctyri rady rezervy.
constexpr double kTol = 1e-12;

const double kHalfPi = std::acos(0.0);

std::string fixture(const char* fileName) {
  return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + fileName;
}

Eigen::VectorXd zeros(int n) { return Eigen::VectorXd::Zero(n); }

}  // namespace

// ---------------------------------------------------------------------------
// q = 0
// ---------------------------------------------------------------------------

TEST_CASE("pri q = 0 je vysledek slozeni samych originTransform") {
  // Pri nulove konfiguraci je motion(0) identita, takze z jointTransform
  // zbyde holy originTransform. Vysledek proto MUSI byt jejich soucin podel
  // retezu — a to se da spocitat nezavisle, bez FK.
  //
  // Kdyby se nekde pletlo poradi nasobeni (local * parent misto
  // parent * local), tenhle test to chyti u prvniho kloubu s nenulovym
  // originem, protoze translace se skladaji pres rotaci rodice.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  const SE3 actual = forwardKinematics(robot, zeros(robot.numDofs()));

  // Rucne: joint1 (0,0,0.1) * joint2 (0.5,0,0) * tool offset (0,0,0.05).
  // Vsechny rotace jsou identita, takze se translace jen sectou.
  SE3 expected = SE3::identity();
  for (int i = 0; i < robot.numJoints(); ++i) {
    expected = expected * robot.joint(i).originTransform;
  }
  expected = expected * robot.link(robot.tipLinkIndex()).offset;

  CHECK(actual.isApprox(expected, kTol));
  CHECK(actual.translation().isApprox(Vec3(0.5, 0.0, 0.15)));
  CHECK(actual.rotation().isApprox(Mat3::Identity()));
}

TEST_CASE("pri q = 0 je koren v identite") {
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  const std::vector<SE3> all = forwardKinematicsAll(robot, zeros(robot.numDofs()));
  CHECK(all[static_cast<std::size_t>(robot.rootLinkIndex())].isApprox(SE3::identity(), kTol));
}

TEST_CASE("koren je v identite pro libovolne q") {
  // Silnejsi tvrzeni nez predchozi test: koren se nesmi hnout ani kdyz se
  // klouby hybaji. Kdyby FK omylem aplikovala prvni kloub i na bazi, tohle
  // spadne, zatimco test s q = 0 by prosel.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  Eigen::VectorXd q(2);
  q << 0.7, -1.3;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);
  CHECK(all[static_cast<std::size_t>(robot.rootLinkIndex())].isApprox(SE3::identity(), kTol));
}

// ---------------------------------------------------------------------------
// Jednokloubovy robot proti rucnimu vypoctu
// ---------------------------------------------------------------------------

TEST_CASE("jeden kloub, rotace kolem z o pi/2") {
  // Rucni vypocet. Kloub sedi v (0, 0, 0.1) a otaci se kolem z. Za nim je
  // fixed rameno (0.5, 0, 0) k linku 'tip'.
  //
  //   q = 0:      tip = (0.5, 0, 0.1),  R = I
  //
  //   q = pi/2:   rotace kolem z posle x na y, tedy rameno (0.5, 0, 0) se
  //               zobrazi na (0, 0.5, 0). Pricteme vysku kloubu:
  //               tip = (0, 0.5, 0.1)
  //
  //               R = Rz(pi/2) = | 0  -1   0 |
  //                              | 1   0   0 |
  //                              | 0   0   1 |
  //
  // Vyska 0.1 se NESMI otocit — kloub je nad bazi, ne za rotaci. Kdyby se
  // poradi nasobeni prohodilo, vyslo by tipu z = 0 a y = 0.6 nebo podobne.
  const Robot robot = Robot::fromUrdfFile(fixture("single_joint.urdf"));
  REQUIRE(robot.numDofs() == 1);

  Eigen::VectorXd q(1);

  q << 0.0;
  const SE3 atZero = forwardKinematics(robot, q);
  CHECK(atZero.translation().isApprox(Vec3(0.5, 0.0, 0.1)));
  CHECK(atZero.rotation().isApprox(Mat3::Identity()));

  q << kHalfPi;
  const SE3 atQuarter = forwardKinematics(robot, q);
  CHECK((atQuarter.translation() - Vec3(0.0, 0.5, 0.1)).cwiseAbs().maxCoeff() < kTol);

  Mat3 expectedR;
  // clang-format off
  expectedR << 0.0, -1.0, 0.0,
               1.0,  0.0, 0.0,
               0.0,  0.0, 1.0;
  // clang-format on
  CHECK((atQuarter.rotation() - expectedR).cwiseAbs().maxCoeff() < kTol);
}

TEST_CASE("rotace o pi vrati rameno na opacnou stranu") {
  // Druhy rucni pripad, uz bez matice: pri q = pi musi byt rameno presne
  // zrcadlene kolem osy. Chyti chybu ve znamenku uhlu, kterou by pi/2
  // teoreticky mohlo minout, kdyby se osa otocila zaroven se znamenkem.
  const Robot robot = Robot::fromUrdfFile(fixture("single_joint.urdf"));
  Eigen::VectorXd q(1);
  q << 2.0 * kHalfPi;
  const SE3 T = forwardKinematics(robot, q);
  CHECK((T.translation() - Vec3(-0.5, 0.0, 0.1)).cwiseAbs().maxCoeff() < kTol);
}

TEST_CASE("znamenko uhlu odpovida pravidlu prave ruky") {
  // Kladne q kolem z musi poslat rameno ze smeru +x do +y, ne do -y. Tohle je
  // jediny test, ktery rozlisi rodrigues(w) od rodrigues(-w) — vsechny
  // invarianty nize plati pro obe.
  const Robot robot = Robot::fromUrdfFile(fixture("single_joint.urdf"));
  Eigen::VectorXd q(1);
  q << 0.3;
  const SE3 T = forwardKinematics(robot, q);
  CHECK(T.translation().y() > 0.0);
  CHECK(T.translation().x() > 0.0);  // porad blize k +x nez k +y
}

// ---------------------------------------------------------------------------
// Dvoukloubovy retez pri nenulovem q — poradi nasobeni
// ---------------------------------------------------------------------------
//
// Tahle sekce vznikla az po mutacnim testu. Puvodni testy NECHYTILY dve
// zakladni zamenyv poradi:
//
//   (1) motion * originTransform misto originTransform * motion
//   (2) local * parent misto parent * local pri skladani retezu
//
// Duvod byl pokazde ten samy: jediny test s absolutni polohou pri nenulovem q
// byl jednokloubovy robot s osou z a originem (0, 0, 0.1). Rotace kolem z
// necha vektor (0, 0, 0.1) na miste, takze obe zamenenne verze daly totez.
// Zbyle testy pri nenulovem q byly bud relativni (vzajemna poloha dvou linku),
// nebo self-consistentni (obe strany prosly stejnou zamenou).
//
// Poucenie do budoucna: pri overovani poradi nasobeni musi mit testovaci
// pripad translaci, ktera NENI rovnobezna s osou otaceni. Jinak transformace
// komutuji a test nerozlisi nic.

TEST_CASE("dva klouby pri nenulovem q, rucne spocitane") {
  // two_joint.urdf:
  //   joint1: origin (0, 0, 0.1), osa z
  //   joint2: origin (0.3, 0, 0), osa y      <- translace kolma na osu z
  //   tip = link2, bez fixed offsetu
  //
  // Pripad q = (0, pi/2). Rucne:
  //
  //   local1 = trans(0,0,0.1) * Rz(0)     = SE3(I,  (0, 0, 0.1))
  //   local2 = trans(0.3,0,0) * Ry(pi/2)  = SE3(Ry, (0.3, 0, 0))
  //
  //   X2 = local1 * local2 = SE3(Ry, I*(0.3,0,0) + (0,0,0.1))
  //                        = SE3(Ry, (0.3, 0, 0.1))
  //
  // Kdyby se motion aplikovalo zleva (chyba 1), byl by local2 roven
  // SE3(Ry, Ry*(0.3,0,0)) = SE3(Ry, (0, 0, -0.3)) a tip by skoncil
  // v (0, 0, -0.2).
  //
  // Kdyby se retez skladal obracene (chyba 2), vyslo by
  // local2 * local1 = SE3(Ry, Ry*(0,0,0.1) + (0.3,0,0)) = SE3(Ry, (0.4, 0, 0)).
  const Robot robot = Robot::fromUrdfFile(fixture("two_joint.urdf"));
  REQUIRE(robot.numDofs() == 2);

  Eigen::VectorXd q(2);
  q << 0.0, kHalfPi;
  const SE3 T = forwardKinematics(robot, q);

  CHECK((T.translation() - Vec3(0.3, 0.0, 0.1)).cwiseAbs().maxCoeff() < kTol);

  Mat3 expectedR;  // Ry(pi/2)
  // clang-format off
  expectedR <<  0.0, 0.0, 1.0,
                0.0, 1.0, 0.0,
               -1.0, 0.0, 0.0;
  // clang-format on
  CHECK((T.rotation() - expectedR).cwiseAbs().maxCoeff() < kTol);

  // A explicitne, ze to nejsou ty dve chybne varianty — kdyby nekdo poradi
  // prohodil, tenhle radek pojmenuje, co se stalo.
  CHECK((T.translation() - Vec3(0.0, 0.0, -0.2)).cwiseAbs().maxCoeff() > 0.1);
  CHECK((T.translation() - Vec3(0.4, 0.0, 0.0)).cwiseAbs().maxCoeff() > 0.1);
}

TEST_CASE("dva klouby, oba nenulove, rucne spocitane") {
  // Obecny pripad q = (pi/2, pi/2), kde nic nekomutuje.
  //
  //   local1 = trans(0,0,0.1) * Rz(pi/2) = SE3(Rz, (0, 0, 0.1))
  //   local2 = trans(0.3,0,0) * Ry(pi/2) = SE3(Ry, (0.3, 0, 0))
  //
  //   translace: R1 * t2 + t1 = Rz(pi/2)*(0.3,0,0) + (0,0,0.1)
  //                           = (0, 0.3, 0) + (0, 0, 0.1)
  //                           = (0, 0.3, 0.1)
  //
  //   rotace:    Rz(pi/2) * Ry(pi/2)
  //     Rz = |0 -1  0|      Ry = | 0  0  1|
  //          |1  0  0|           | 0  1  0|
  //          |0  0  1|           |-1  0  0|
  //
  //     radek 0 Rz = (0,-1,0) krat sloupce Ry (0,0,-1),(0,1,0),(1,0,0)
  //                                        -> ( 0, -1,  0)
  //     radek 1 Rz = (1, 0,0)              -> ( 0,  0,  1)
  //     radek 2 Rz = (0, 0,1)              -> (-1,  0,  0)
  const Robot robot = Robot::fromUrdfFile(fixture("two_joint.urdf"));

  Eigen::VectorXd q(2);
  q << kHalfPi, kHalfPi;
  const SE3 T = forwardKinematics(robot, q);

  CHECK((T.translation() - Vec3(0.0, 0.3, 0.1)).cwiseAbs().maxCoeff() < kTol);

  Mat3 expectedR;
  // clang-format off
  expectedR <<  0.0, -1.0, 0.0,
                0.0,  0.0, 1.0,
               -1.0,  0.0, 0.0;
  // clang-format on
  CHECK((T.rotation() - expectedR).cwiseAbs().maxCoeff() < kTol);
}

TEST_CASE("jointTransform aplikuje motion zprava, ne zleva") {
  // Nejmensi mozny test toho jednoho rozhodnuti, izolovane od zbytku FK.
  // Osa je z, translace originu je podel x — tedy KOLMO na osu, jinak by obe
  // varianty komutovaly a test by nerozlisil nic.
  //
  //   origin = trans(0.2, 0, 0),  osa z,  q = pi/2
  //
  //   spravne  origin * motion = SE3(Rz, (0.2, 0, 0))
  //            translace se neotoci, protoze rotace prijde az za ni
  //
  //   spatne   motion * origin = SE3(Rz, Rz*(0.2,0,0)) = SE3(Rz, (0, 0.2, 0))
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"a\"/>"
      "    <origin xyz=\"0.2 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-3.2\" upper=\"3.2\" effort=\"1\" velocity=\"1\"/></joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);
  const SE3 T = robometrics::jointTransform(robot.joint(0), kHalfPi);

  CHECK((T.translation() - Vec3(0.2, 0.0, 0.0)).cwiseAbs().maxCoeff() < kTol);
  CHECK((T.translation() - Vec3(0.0, 0.2, 0.0)).cwiseAbs().maxCoeff() > 0.1);
}

// ---------------------------------------------------------------------------
// Vztah forwardKinematics a forwardKinematicsAll
// ---------------------------------------------------------------------------

TEST_CASE("forwardKinematicsAll na indexu tipu se rovna forwardKinematics") {
  // Indexovat pres tipLinkIndex(), ne pres .back(): u stromu s chapadlem je
  // posledni link v poli nahodny prst, ne end-effector. Poradi linku je vec
  // topologickeho pruchodu, tip je vec zadani.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 0.4, -0.9;

  const std::vector<SE3> all = forwardKinematicsAll(robot, q);
  const SE3 tip = forwardKinematics(robot, q);

  REQUIRE(all.size() == static_cast<std::size_t>(robot.numLinks()));
  CHECK(all[static_cast<std::size_t>(robot.tipLinkIndex())].isApprox(tip, kTol));
}

TEST_CASE("ramec kloubu je poloha jeho child linku") {
  // Most k Jacobianu: ten iteruje pres klouby, ne pres linky. Tady se overuje,
  // ze prevod mezi tim je opravdu jen indexace pres childLink, jak to slibuje
  // dokumentace v kinematics.hpp.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 0.2, 0.5;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);

  // Slozime polohu prvniho kloubu rucne a porovname s polohou jeho child linku.
  const robometrics::Joint& j0 = robot.joint(0);
  const SE3 expected = robometrics::jointTransform(j0, q(j0.dofIndex));
  CHECK(all[static_cast<std::size_t>(j0.childLink)].isApprox(expected, kTol));
}

// ---------------------------------------------------------------------------
// Fixed klouby v FK
// ---------------------------------------------------------------------------

TEST_CASE("koncovy fixed kloub se projevi v poloze tipu") {
  // Parser ho ulozi jako Link::offset; tenhle test overuje, ze ho FK skutecne
  // pouzije. Bez nej by tip skoncil na linku 'link2', tedy o 0.05 niz —
  // presne ta chyba, kvuli ktere ma Link vlastni offset.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 0.0, 0.0;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);

  const SE3 link2 = all[static_cast<std::size_t>(robot.findLink("link2"))];
  const SE3 tool = all[static_cast<std::size_t>(robot.findLink("tool"))];

  // Rozdil mezi nimi je presne fixed origin 0.05 podel z linku link2.
  const SE3 relative = link2.inverse() * tool;
  CHECK(relative.translation().isApprox(Vec3(0.0, 0.0, 0.05)));
  CHECK(relative.rotation().isApprox(Mat3::Identity()));
}

TEST_CASE("fixed offset je tuhy vuci svemu kloubu pro libovolne q") {
  // Silnejsi verze predchoziho: vzajemna poloha link2 a tool nesmi na q
  // zaviset vubec. Kdyby se fixed offset omylem aplikoval na spatne strane
  // nasobeni, pri q = 0 by to porad vychazelo a tady uz ne.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 1.1, -0.6;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);

  const SE3 relative = all[static_cast<std::size_t>(robot.findLink("link2"))].inverse() *
                       all[static_cast<std::size_t>(robot.findLink("tool"))];
  CHECK(relative.isApprox(SE3(Mat3::Identity(), Vec3(0.0, 0.0, 0.05)), kTol));
}

// ---------------------------------------------------------------------------
// Prismatic a mimic klouby
// ---------------------------------------------------------------------------

TEST_CASE("prismatic kloub posouva, mimic kloub jde proti nemu") {
  // Rucni vypocet pro chapadlo:
  //
  //   arm q = 0        => hand v (0, 0, 0.4)
  //   finger_left q = 0.03, origin (0, 0.02, 0.05), osa y
  //                    => left  = (0, 0.4) + (0, 0.02, 0.05) + (0, 0.03, 0)
  //                             = (0, 0.05, 0.45)
  //   finger_right mimic, multiplier -1 => hodnota -0.03
  //                       origin (0, -0.02, 0.05), osa y
  //                    => right = (0, -0.02, 0.05) + (0, -0.03, 0) posunute
  //                             = (0, -0.05, 0.45)
  //
  // Prsty tedy vyjdou symetricky kolem osy. Kdyby se multiplier ignoroval,
  // right by skoncil v (0, 0.01, 0.45) a symetrie by zmizela.
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "hand");
  REQUIRE(robot.numDofs() == 2);
  REQUIRE(robot.numJoints() == 3);

  Eigen::VectorXd q(2);
  q << 0.0, 0.03;
  const std::vector<SE3> all = forwardKinematicsAll(robot, q);

  const Vec3 left = all[static_cast<std::size_t>(robot.findLink("left"))].translation();
  const Vec3 right = all[static_cast<std::size_t>(robot.findLink("right"))].translation();

  CHECK((left - Vec3(0.0, 0.05, 0.45)).cwiseAbs().maxCoeff() < kTol);
  CHECK((right - Vec3(0.0, -0.05, 0.45)).cwiseAbs().maxCoeff() < kTol);

  // Symetrie zvlast, protoze rika presne to, co ma mimic multiplier znamenat.
  CHECK(left.y() == doctest::Approx(-right.y()));
}

TEST_CASE("mimic offset se aplikuje") {
  // multiplier sam o sobe by prosel i kdyby se offset zahazoval, protoze ve
  // fixture je nulovy. Tenhle pripad ho ma nenulovy.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/><link name=\"b\"/>"
      "  <joint name=\"driver\" type=\"prismatic\">"
      "    <parent link=\"base\"/><child link=\"a\"/><axis xyz=\"1 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>"
      "  <joint name=\"follower\" type=\"prismatic\">"
      "    <parent link=\"a\"/><child link=\"b\"/><axis xyz=\"1 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "    <mimic joint=\"driver\" multiplier=\"2\" offset=\"0.5\"/></joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);
  REQUIRE(robot.numDofs() == 1);

  Eigen::VectorXd q(1);
  q << 0.1;
  // driver   = 0.1
  // follower = 2 * 0.1 + 0.5 = 0.7
  // celkem podel x: 0.1 + 0.7 = 0.8
  const SE3 tip = forwardKinematics(robot, q);
  CHECK(tip.translation().x() == doctest::Approx(0.8));
}

TEST_CASE("mimic kloub se hybe i kdyz je jeho q nulove") {
  // Dusledek nenuloveho offsetu: pri q = 0 neni follower v nule, ale v 0.5.
  // Test existuje proto, ze "vsechno v nule" je nejcastejsi predpoklad, ktery
  // u mimic kloubu s offsetem neplati.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/><link name=\"b\"/>"
      "  <joint name=\"driver\" type=\"prismatic\">"
      "    <parent link=\"base\"/><child link=\"a\"/><axis xyz=\"1 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>"
      "  <joint name=\"follower\" type=\"prismatic\">"
      "    <parent link=\"a\"/><child link=\"b\"/><axis xyz=\"1 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "    <mimic joint=\"driver\" multiplier=\"1\" offset=\"0.5\"/></joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);
  const SE3 tip = forwardKinematics(robot, zeros(1));
  CHECK(tip.translation().x() == doctest::Approx(0.5));
}

// ---------------------------------------------------------------------------
// Invarianty
// ---------------------------------------------------------------------------

TEST_CASE("kazda vysledna poloha je platna transformace SE(3)") {
  // Strukturalni kontrola. Chyti chyby, ktere konkretni cisla minou — treba
  // kdyby se nekam vloudilo meritko nebo zrcadleni; vysledek by pak porad
  // "nekde byl", jen by to nebyla tuha transformace.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd q(2);
  q << 2.4, -1.7;
  for (const SE3& T : forwardKinematicsAll(robot, q)) {
    const Mat3 r = T.rotation();
    CHECK((r.transpose() * r - Mat3::Identity()).cwiseAbs().maxCoeff() < kTol);
  }
}

TEST_CASE("otoceni jednoho kloubu tam a zpet vrati puvodni polohu") {
  // Rika, ze FK je funkce q a nic si nepamatuje mezi volanimi. Trivialne
  // splneno pro cistou funkci — presne proto je to dobry test na to, jestli
  // se nekam nedostal stav (cache, static, mutovany Robot).
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  Eigen::VectorXd a(2);
  a << 0.3, 1.2;
  Eigen::VectorXd b(2);
  b << -2.0, 0.4;

  const SE3 first = forwardKinematics(robot, a);
  forwardKinematics(robot, b);
  const SE3 again = forwardKinematics(robot, a);
  CHECK(first.isApprox(again, kTol));
}

TEST_CASE("continuous kloub prijme uhel mimo interval (-pi, pi)") {
  // Continuous kloub nema meze, takze q = 7 rad je legitimni vstup. Vysledek
  // musi byt stejny jako pro 7 - 2*pi — ne proto, ze bychom uhel zabalovali,
  // ale protoze rotace o 2*pi je identita.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/>"
      "  <joint name=\"wheel\" type=\"continuous\">"
      "    <parent link=\"base\"/><child link=\"a\"/>"
      "    <origin xyz=\"0.2 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/></joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);

  Eigen::VectorXd big(1);
  big << 7.0;
  Eigen::VectorXd wrapped(1);
  wrapped << 7.0 - 4.0 * kHalfPi;

  // Volnejsi tolerance: 7 - 2*pi je odecteni skoro stejnych cisel, takze uhel
  // sam nese chybu radu 1e-16, ktera se do rotace propise primo.
  CHECK(forwardKinematics(robot, big).isApprox(forwardKinematics(robot, wrapped), 1e-14));
}

// ---------------------------------------------------------------------------
// Chybove stavy
// ---------------------------------------------------------------------------

TEST_CASE("spatny rozmer q je vyjimka") {
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  CHECK_THROWS_AS(forwardKinematics(robot, zeros(1)), std::invalid_argument);
  CHECK_THROWS_AS(forwardKinematics(robot, zeros(3)), std::invalid_argument);
  CHECK_THROWS_AS(forwardKinematicsAll(robot, zeros(0)), std::invalid_argument);
  CHECK_NOTHROW(forwardKinematics(robot, zeros(2)));
}

TEST_CASE("zprava o spatnem rozmeru q uvadi obe cisla") {
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  try {
    forwardKinematics(robot, zeros(5));
    FAIL("ocekavana vyjimka std::invalid_argument");
  } catch (const std::invalid_argument& e) {
    const std::string message = e.what();
    CHECK(message.find('5') != std::string::npos);
    CHECK(message.find('2') != std::string::npos);
    CHECK(message.find("fixed_chain") != std::string::npos);
  }
}

TEST_CASE("u robota s mimic klouby zprava vysvetli rozdil poctu") {
  // numJoints() == 3, numDofs() == 2. Kdo pouzije numJoints(), dostane
  // vyjimku — a ta mu musi rovnou rict proc, jinak to vypada jako chyba
  // knihovny.
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "hand");
  try {
    forwardKinematics(robot, zeros(robot.numJoints()));
    FAIL("ocekavana vyjimka std::invalid_argument");
  } catch (const std::invalid_argument& e) {
    CHECK(std::string(e.what()).find("mimic") != std::string::npos);
  }
}
