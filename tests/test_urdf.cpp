// Testy parseru URDF.
//
// Fixtury jsou soubory v tests/fixtures/, ne inline stringy — chceme testovat
// i cestu pres Robot::fromUrdfFile(), a soubor se da otevrit a precist, kdyz
// test spadne. Cestu k nim predava CMake makrem ROBOMETRICS_FIXTURE_DIR.
//
// U chybovych testu se schvalne netestuje jen "hodilo to vyjimku", ale i to,
// CO ve zprave stoji. Zprava je tady soucast API: kdyz nekdo dostane URDF od
// tretiho vyrobce, jedine, co ma k dispozici, je ta veta.

#include <doctest/doctest.h>

// determinant() nezije v Eigen/Core, ale v Eigen/LU — stejna past jako
// .cross() v test_se3.cpp: prelozi se to a spadne az pri linkovani.
#include <Eigen/LU>
#include <string>
#include <vector>

#include "robometrics/urdf.hpp"

namespace {

using robometrics::JointType;
using robometrics::Mat3;
using robometrics::Robot;
using robometrics::SE3;
using robometrics::UrdfError;
using robometrics::Vec3;

/// Tolerance pro porovnani transformaci. Volnejsi nez kDefaultTol (1e-9),
/// protoze rpy prochazi pres rodrigues() — cos(pi/2) vyjde 6.1e-17 misto nuly
/// a stejne drobne zbytky se objevi po vsech trech nasobenich. Rad chyby je
/// 1e-16, takze 1e-12 ma ctyri rady rezervy a porad by chytlo jakoukoli
/// skutecnou chybu v konvenci nebo znamenku.
constexpr double kTol = 1e-12;

std::string fixture(const char* fileName) {
  return std::string(ROBOMETRICS_FIXTURE_DIR) + "/" + fileName;
}

/// Sestavi Mat3 po radcich, aby zapis v testu vypadal jako matice na papire.
/// Eigen ma operator<<, ale ten se v testu spatne cte kvuli carkam.
Mat3 rows(double a, double b, double c, double d, double e, double f, double g, double h,
          double i) {
  Mat3 m;
  // clang-format off
  m << a, b, c,
       d, e, f,
       g, h, i;
  // clang-format on
  return m;
}

}  // namespace

// ---------------------------------------------------------------------------
// Validni minimalni URDF
// ---------------------------------------------------------------------------

TEST_CASE("dvoukloubovy retez se nacte se spravnymi hodnotami") {
  // Zakladni test "nacte se to vubec". Kdyby padal jen tenhle, chyba je nekde
  // uplne dole (XML, jmena atributu), ne v topologii.
  const Robot robot = Robot::fromUrdfFile(fixture("two_joint.urdf"));

  CHECK(robot.name() == "two_joint");
  CHECK(robot.numJoints() == 2);
  CHECK(robot.numDofs() == 2);  // zadny mimic => stejne jako numJoints()
  CHECK(robot.numLinks() == 3);

  // Poradi kloubu musi byt od baze k tipu bez ohledu na poradi v souboru.
  CHECK(robot.joint(0).name == "joint1");
  CHECK(robot.joint(1).name == "joint2");

  // parentJoint == -1 znamena "visi primo na korenovem linku". Pro joint2 to
  // musi byt index 0 — tohle je pole, po kterem FK leze.
  CHECK(robot.joint(0).parentJoint == -1);
  CHECK(robot.joint(1).parentJoint == 0);

  CHECK(robot.joint(0).type == JointType::Revolute);
  CHECK(robot.joint(0).axis.isApprox(Vec3::UnitZ()));
  CHECK(robot.joint(1).axis.isApprox(Vec3::UnitY()));

  CHECK(robot.joint(0).lowerLimit == doctest::Approx(-1.5));
  CHECK(robot.joint(0).upperLimit == doctest::Approx(1.5));
  CHECK(robot.joint(0).effortLimit == doctest::Approx(87.0));
  CHECK(robot.joint(0).velocityLimit == doctest::Approx(2.61));

  CHECK(robot.joint(0).originTransform.translation().isApprox(Vec3(0.0, 0.0, 0.1)));
  CHECK(robot.joint(1).originTransform.translation().isApprox(Vec3(0.3, 0.0, 0.0)));

  // Jediny list => autodetekce tipu ho musi najit sama, bez druheho argumentu.
  CHECK(robot.tipLinkIndex() == robot.findLink("link2"));
  CHECK(robot.rootLinkIndex() == robot.findLink("base"));
}

TEST_CASE("fromUrdfString dava stejny vysledek jako fromUrdfFile") {
  // Kdyby se ty dve cesty rozesly, testy nad stringy by prestaly rikat cokoli
  // o tom, co se stane se skutecnym souborem.
  const Robot fromFile = Robot::fromUrdfFile(fixture("two_joint.urdf"));

  const std::string xml =
      "<robot name=\"two_joint\">"
      "  <link name=\"base\"/><link name=\"link1\"/><link name=\"link2\"/>"
      "  <joint name=\"joint1\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"link1\"/>"
      "    <origin xyz=\"0 0 0.1\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-1.5\" upper=\"1.5\" effort=\"87\" velocity=\"2.61\"/>"
      "  </joint>"
      "  <joint name=\"joint2\" type=\"revolute\">"
      "    <parent link=\"link1\"/><child link=\"link2\"/>"
      "    <origin xyz=\"0.3 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 1 0\"/>"
      "    <limit lower=\"-2\" upper=\"2\" effort=\"12\" velocity=\"2.61\"/>"
      "  </joint>"
      "</robot>";
  const Robot fromString = Robot::fromUrdfString(xml);

  REQUIRE(fromString.numJoints() == fromFile.numJoints());
  for (int i = 0; i < fromFile.numJoints(); ++i) {
    CHECK(fromString.joint(i).name == fromFile.joint(i).name);
    CHECK(fromString.joint(i).originTransform.isApprox(fromFile.joint(i).originTransform, kTol));
  }
}

// ---------------------------------------------------------------------------
// Skladani fixed kloubu
// ---------------------------------------------------------------------------

TEST_CASE("fixed kloub uprostred retezu se slozi do nasledujiciho kloubu") {
  // Nejdulezitejsi test celeho parseru. Fixed kloub nesmi zmizet ani zustat
  // jako samostatny stupen volnosti — musi se prolnout do originTransform
  // dalsiho pohybliveho kloubu.
  //
  // Retez: base -[joint1]-> link1 -[mount_fixed 0.2]-> mount -[joint2 0.3]-> link2
  // Takze joint2 zacina na 0.2 + 0.3 = 0.5 od link1, ne na 0.3.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  CHECK(robot.numJoints() == 2);  // ctyri klouby v souboru, dva pohyblive
  CHECK(robot.numLinks() == 5);   // linky se ale nezahazuji, vsech pet zustava

  CHECK(robot.joint(0).name == "joint1");
  CHECK(robot.joint(1).name == "joint2");

  // Kdyby se mount_fixed zahodil, vyslo by 0.3. Kdyby zustal jako kloub,
  // numJoints() by bylo 4 a tenhle CHECK by se ani nedostal ke slovu.
  CHECK(robot.joint(1).originTransform.translation().isApprox(Vec3(0.5, 0.0, 0.0)));

  // parentJoint preskakuje fixed kloub: joint2 visi na joint1, ne na nicem.
  CHECK(robot.joint(1).parentJoint == 0);

  // Link 'mount' existuje dal a je nesen kloubem joint1, posunuty o 0.2.
  const int mount = robot.findLink("mount");
  REQUIRE(mount >= 0);
  CHECK(robot.link(mount).supportingJoint == 0);
  CHECK(robot.link(mount).offset.translation().isApprox(Vec3(0.2, 0.0, 0.0)));
}

TEST_CASE("koncovy fixed kloub prezije jako offset linku") {
  // Tenhle pripad by skladani "dopredu do dalsiho pohybliveho kloubu"
  // ztratilo — za tool_fixed uz zadny pohyblivy kloub neni. A prave takhle
  // vypada konec kazdeho realneho manipulatoru (hand -> flange -> grasp
  // target), takze by se end-effector tise usadil o par centimetru vedle.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));

  const int tool = robot.findLink("tool");
  REQUIRE(tool >= 0);

  // Nesen poslednim pohyblivym kloubem, posunuty o svuj fixed origin.
  CHECK(robot.link(tool).supportingJoint == 1);
  CHECK(robot.link(tool).offset.translation().isApprox(Vec3(0.0, 0.0, 0.05)));

  // A je to autodetekovany tip, protoze je to jediny list.
  CHECK(robot.tipLinkIndex() == tool);
}

TEST_CASE("korenovy link nema nosny kloub a sedi v identite") {
  // Referencni ramec cele knihovny. Kdyby koren dostal nenulovy offset,
  // posunulo by to uplne vsechno a zadny jiny test by to nemusel odhalit,
  // protoze vsechny polohy by se posunuly stejne.
  const Robot robot = Robot::fromUrdfFile(fixture("fixed_chain.urdf"));
  const int root = robot.rootLinkIndex();

  CHECK(robot.link(root).name == "base");
  CHECK(robot.link(root).supportingJoint == -1);
  CHECK(robot.link(root).offset.isApprox(SE3::identity(), kTol));
}

// ---------------------------------------------------------------------------
// rpy konvence proti rucne spocitanym cislum
// ---------------------------------------------------------------------------

TEST_CASE("rpy = (0, 0, pi/2) je rotace kolem z") {
  // Nejjednodussi pripad, ale sam o sobe NIC nerika o poradi nasobeni: kdyz
  // jsou dva ze tri uhlu nulove, Rz*Ry*Rx a Rx*Ry*Rz daji totez. Je tu jako
  // kontrola znamenka, ne konvence.
  //
  //   Rz(pi/2) = | 0  -1   0 |
  //              | 1   0   0 |
  //              | 0   0   1 |
  const Robot robot = Robot::fromUrdfFile(fixture("rpy_cases.urdf"));
  const int idx = robot.findJoint("case_yaw");
  REQUIRE(idx >= 0);

  // clang-format off
  const Mat3 expected = rows( 0.0, -1.0,  0.0,
                              1.0,  0.0,  0.0,
                              0.0,  0.0,  1.0);
  // clang-format on
  const Mat3 actual = robot.joint(idx).originTransform.rotation();
  CHECK((actual - expected).cwiseAbs().maxCoeff() < kTol);
}

TEST_CASE("rpy = (pi/2, pi/2, 0) rozlisuje Rz*Ry*Rx od Rx*Ry*Rz") {
  // Tohle je ten test, ktery konvenci skutecne pribiji. Rucni vypocet:
  //
  //   Rx(pi/2) = | 1  0  0 |     Ry(pi/2) = |  0  0  1 |     Rz(0) = I
  //              | 0  0 -1 |                |  0  1  0 |
  //              | 0  1  0 |                | -1  0  0 |
  //
  //   Rz(0)*Ry(pi/2)*Rx(pi/2) = Ry*Rx:
  //     radek 0 Ry = ( 0, 0, 1) krat sloupce Rx (1,0,0),(0,0,1),(0,-1,0)
  //                             -> ( 0,  1,  0)
  //     radek 1 Ry = ( 0, 1, 0) -> ( 0,  0, -1)
  //     radek 2 Ry = (-1, 0, 0) -> (-1,  0,  0)
  //
  //   spravne (Rz*Ry*Rx)      spatne (Rx*Ry*Rz)
  //     |  0   1   0 |          | 0  0  1 |
  //     |  0   0  -1 |          | 1  0  0 |
  //     | -1   0   0 |          | 0  1  0 |
  //
  // Kdyby byla konvence obracena, vysla by ta druha matice — a ta je shodou
  // okolnosti spravnym vysledkem pro case_ry nize. Test by tedy neselhal na
  // "necislo", ale na uplne konkretni jinou rotaci.
  const Robot robot = Robot::fromUrdfFile(fixture("rpy_cases.urdf"));
  const int idx = robot.findJoint("case_rp");
  REQUIRE(idx >= 0);

  // clang-format off
  const Mat3 expected   = rows( 0.0,  1.0,  0.0,
                                0.0,  0.0, -1.0,
                               -1.0,  0.0,  0.0);
  const Mat3 wrongOrder = rows( 0.0,  0.0,  1.0,
                                1.0,  0.0,  0.0,
                                0.0,  1.0,  0.0);
  // clang-format on
  const Mat3 actual = robot.joint(idx).originTransform.rotation();

  CHECK((actual - expected).cwiseAbs().maxCoeff() < kTol);
  // Explicitne i to, ze to NENI obracene poradi — kdyby nekdo v budoucnu
  // prohodil rz*ry*rx za rx*ry*rz, tenhle radek pojmenuje presne to, co se
  // stalo, misto aby jen zhaslo cislo v predchozim CHECKu.
  CHECK((actual - wrongOrder).cwiseAbs().maxCoeff() > 0.5);

  // xyz se aplikuje nezavisle na rpy, takze translaci overujeme zvlast.
  CHECK(robot.joint(idx).originTransform.translation().isApprox(Vec3(0.1, -0.2, 0.3)));
}

TEST_CASE("rpy = (pi/2, 0, pi/2) je cyklicka permutace os") {
  // Treti pripad, dva nenulove uhly a mezi nimi nula. Rucni vypocet:
  //
  //   Rz(pi/2)*Ry(0)*Rx(pi/2) = Rz*Rx:
  //     radek 0 Rz = (0, -1, 0) krat sloupce Rx (1,0,0),(0,0,1),(0,-1,0)
  //                              -> (0, 0, 1)
  //     radek 1 Rz = (1,  0, 0) -> (1, 0, 0)
  //     radek 2 Rz = (0,  0, 1) -> (0, 1, 0)
  //
  //   | 0  0  1 |    x -> y, y -> z, z -> x
  //   | 1  0  0 |
  //   | 0  1  0 |
  //
  // Snadna kontrola bez pocitani: sloupce matice jsou obrazy bazovych vektoru,
  // takze prvni sloupec (0,1,0) rika, ze x se zobrazi na y. To odpovida.
  const Robot robot = Robot::fromUrdfFile(fixture("rpy_cases.urdf"));
  const int idx = robot.findJoint("case_ry");
  REQUIRE(idx >= 0);

  // clang-format off
  const Mat3 expected = rows( 0.0,  0.0,  1.0,
                              1.0,  0.0,  0.0,
                              0.0,  1.0,  0.0);
  // clang-format on
  const Mat3 actual = robot.joint(idx).originTransform.rotation();
  CHECK((actual - expected).cwiseAbs().maxCoeff() < kTol);

  // Kontrola pres obrazy bazovych vektoru, nezavisla na tom, jestli jsem
  // matici vypsal spravne po radcich nebo po sloupcich.
  CHECK((actual * Vec3::UnitX()).isApprox(Vec3::UnitY()));
  CHECK((actual * Vec3::UnitY()).isApprox(Vec3::UnitZ()));
  CHECK((actual * Vec3::UnitZ()).isApprox(Vec3::UnitX()));
}

TEST_CASE("rpy vysledek je vzdy vlastni rotace") {
  // Strukturalni invariant, ktery plati bez ohledu na konvenci. Chyti chyby,
  // ktere konkretni cisla vyse minou — treba kdyby nekdo omylem slozil rotace
  // s nejakym meritkem nebo zrcadlenim.
  const Robot robot = Robot::fromUrdfFile(fixture("rpy_cases.urdf"));
  for (int i = 0; i < robot.numJoints(); ++i) {
    const Mat3 r = robot.joint(i).originTransform.rotation();
    CHECK((r.transpose() * r - Mat3::Identity()).cwiseAbs().maxCoeff() < kTol);
    CHECK(r.determinant() == doctest::Approx(1.0));
  }
}

// ---------------------------------------------------------------------------
// Mimic klouby
// ---------------------------------------------------------------------------

TEST_CASE("mimic kloub se pocita do numJoints, ale ne do numDofs") {
  // Tohle je duvod, proc ty dva pocty existuji zvlast. Kdyby se mimic tag
  // ignoroval, numDofs() by bylo 3 a q z nahraneho rolloutu (delky 2) by se
  // indexovalo posunute — bez jedine vyjimky, jen se spatnymi cisly.
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "left");

  CHECK(robot.numJoints() == 3);
  CHECK(robot.numDofs() == 2);

  const int right = robot.findJoint("finger_right");
  const int left = robot.findJoint("finger_left");
  REQUIRE(right >= 0);
  REQUIRE(left >= 0);

  CHECK(robot.joint(right).isMimic());
  CHECK(robot.joint(right).mimicSource == left);
  CHECK(robot.joint(right).mimicMultiplier == doctest::Approx(-1.0));
  CHECK(robot.joint(right).mimicOffset == doctest::Approx(0.0));
  CHECK(robot.joint(right).dofIndex == -1);  // hodnota se odvozuje, necte se z q

  CHECK_FALSE(robot.joint(left).isMimic());
  CHECK(robot.joint(left).dofIndex >= 0);
}

TEST_CASE("dofIndex je hustý a bez der") {
  // Invariant, na kterem stoji indexovani q: nezavisle joint dostanou
  // 0, 1, ..., numDofs()-1, kazde presne jednou. Kdyby v ocislovani vznikla
  // dira, FK by cetla q mimo rozsah nebo by nejaky kloub tise zustal na nule.
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "left");

  std::vector<bool> seen(static_cast<std::size_t>(robot.numDofs()), false);
  for (int i = 0; i < robot.numJoints(); ++i) {
    const int dof = robot.joint(i).dofIndex;
    if (robot.joint(i).isMimic()) {
      CHECK(dof == -1);
      continue;
    }
    REQUIRE(dof >= 0);
    REQUIRE(dof < robot.numDofs());
    CHECK_FALSE(seen[static_cast<std::size_t>(dof)]);
    seen[static_cast<std::size_t>(dof)] = true;
  }
  for (const bool used : seen) {
    CHECK(used);
  }
}

// ---------------------------------------------------------------------------
// Chybove stavy
// ---------------------------------------------------------------------------

TEST_CASE("chybejici <axis> u revolute kloubu je chyba se srozumitelnou zpravou") {
  // Spec by dosadila "1 0 0". Tenhle parser to odmita, protoze tise otacet
  // kolem x je presne ten druh chyby, ktery vyrobi robota, co vypada
  // pravdepodobne a je spatne.
  try {
    Robot::fromUrdfFile(fixture("missing_axis.urdf"));
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    // Zprava musi pojmenovat konkretni kloub — na 3000radkovem URDF je
    // "chybi axis" bez jmena k nicemu.
    CHECK(message.find("joint2") != std::string::npos);
    CHECK(message.find("axis") != std::string::npos);
    // A musi rict, ze to je vedome rozhodnuti, ne ze soubor je nevalidni.
    CHECK(message.find("default") != std::string::npos);
    CHECK(e.where().find("joint2") != std::string::npos);
  }
}

TEST_CASE("neznamy typ kloubu je chyba, ktera vyjmenuje podporovane typy") {
  try {
    Robot::fromUrdfFile(fixture("unknown_type.urdf"));
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("joint1") != std::string::npos);
    CHECK(message.find("screw") != std::string::npos);     // co tam bylo napsano
    CHECK(message.find("revolute") != std::string::npos);  // co je povolene
  }
}

TEST_CASE("planar a floating jsou odmitnuty jako vicedimenzionalni") {
  // Odlisena hlaska od uplne neznameho typu: tyhle dva JSOU platne URDF,
  // jen se nevejdou do "jeden skalar na kloub". To je nase omezeni, ne chyba
  // v souboru, a zprava to musi rict.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"floating\">"
      "    <parent link=\"base\"/><child link=\"l\"/>"
      "  </joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("floating") != std::string::npos);
    CHECK(message.find("degree of freedom") != std::string::npos);
  }
}

TEST_CASE("nulova osa je chyba, ne pripad k normalizaci") {
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 0\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "  </joint>"
      "</robot>";
  CHECK_THROWS_AS(Robot::fromUrdfString(xml), UrdfError);
}

TEST_CASE("osa se normalizuje") {
  // Nenormalizovana osa v souboru neni chyba, ale nesmi se pouzit tak, jak je:
  // kloub by se otacel o ||axis|| * q misto o q. Chyba meritka na kazde
  // metrice, a v testu s cistymi osami naprosto neviditelna.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 5\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "  </joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(xml);
  CHECK(robot.joint(0).axis.isApprox(Vec3::UnitZ()));
  CHECK(robot.joint(0).axis.norm() == doctest::Approx(1.0));
}

TEST_CASE("vic listu znamena, ze tip nejde uhodnout") {
  // Kazdy robot s chapadlem. Hadat mezi prsty by dalo end-effector na nahodny
  // z nich; zprava proto kandidaty vypise, aby oprava byla zrejma z hlasky.
  try {
    Robot::fromUrdfFile(fixture("mimic_gripper.urdf"));
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("left") != std::string::npos);
    CHECK(message.find("right") != std::string::npos);
  }
}

TEST_CASE("explicitne zadany tip prebije autodetekci") {
  const Robot robot = Robot::fromUrdfFile(fixture("mimic_gripper.urdf"), "hand");
  CHECK(robot.tipLinkIndex() == robot.findLink("hand"));
}

TEST_CASE("neexistujici tip je chyba") {
  CHECK_THROWS_AS(Robot::fromUrdfFile(fixture("two_joint.urdf"), "nonexistent"), UrdfError);
}

TEST_CASE("link se dvema rodici neni strom") {
  // Uzavreny retez (paralelni mechanismus). FK by nemela jednoznacnou
  // odpoved, takze se to musi odmitnout, ne vybrat jednu vetev.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/><link name=\"b\"/>"
      "  <joint name=\"j1\" type=\"fixed\"><parent link=\"base\"/><child link=\"b\"/></joint>"
      "  <joint name=\"j2\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/></joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("exactly one parent") != std::string::npos);
  }
}

TEST_CASE("dva koreny znamena dva nespojene stromy") {
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"baseA\"/><link name=\"a\"/><link name=\"baseB\"/><link name=\"b\"/>"
      "  <joint name=\"j1\" type=\"fixed\"><parent link=\"baseA\"/><child link=\"a\"/></joint>"
      "  <joint name=\"j2\" type=\"fixed\"><parent link=\"baseB\"/><child link=\"b\"/></joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("baseA") != std::string::npos);
    CHECK(message.find("baseB") != std::string::npos);
  }
}

TEST_CASE("odkaz na nedeklarovany link je chyba, ktera ho pojmenuje") {
  // Typicky preklep ve jmenu. Bez jmena v hlasce se hleda spatne.
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/>"
      "  <joint name=\"j\" type=\"fixed\">"
      "    <parent link=\"base\"/><child link=\"typo_link\"/>"
      "  </joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("typo_link") != std::string::npos);
  }
}

TEST_CASE("chybejici <limit> u revolute kloubu je chyba, u continuous ne") {
  // Rozdil je ze specu: revolute je omezeny z definice, continuous neni.
  const std::string revolute =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 1\"/>"
      "  </joint>"
      "</robot>";
  CHECK_THROWS_AS(Robot::fromUrdfString(revolute), UrdfError);

  const std::string continuous =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"continuous\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 1\"/>"
      "  </joint>"
      "</robot>";
  const Robot robot = Robot::fromUrdfString(continuous);
  CHECK(robot.joint(0).type == JointType::Continuous);
  CHECK(robot.joint(0).lowerLimit == -robometrics::kUnbounded);
  CHECK(robot.joint(0).upperLimit == robometrics::kUnbounded);
}

TEST_CASE("necislo v atributu je chyba, ktera cituje, co tam stalo") {
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"l\"/>"
      "  <joint name=\"j\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"l\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"nope\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "  </joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    const std::string message = e.what();
    CHECK(message.find("lower") != std::string::npos);
    CHECK(message.find("nope") != std::string::npos);
  }
}

TEST_CASE("rozbite XML hlasi cislo radku") {
  const std::string xml =
      "<robot name=\"r\">\n"
      "  <link name=\"base\"/>\n"
      "  <joint name=\"j\" type=\"fixed\">\n"
      "</robot>\n";
  try {
    Robot::fromUrdfString(xml);
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("line") != std::string::npos);
  }
}

TEST_CASE("neexistujici soubor se poz na jako neexistujici soubor") {
  // Ne jako "parse error" — jinak clovek hleda chybu v URDF, ktere neexistuje.
  try {
    Robot::fromUrdfFile(fixture("this_file_does_not_exist.urdf"));
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("not found") != std::string::npos);
  }
}

TEST_CASE("mimic na mimic je odmitnuty s vysvetlenim") {
  const std::string xml =
      "<robot name=\"r\">"
      "  <link name=\"base\"/><link name=\"a\"/><link name=\"b\"/><link name=\"c\"/>"
      "  <joint name=\"j1\" type=\"revolute\">"
      "    <parent link=\"base\"/><child link=\"a\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/></joint>"
      "  <joint name=\"j2\" type=\"revolute\">"
      "    <parent link=\"a\"/><child link=\"b\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "    <mimic joint=\"j1\"/></joint>"
      "  <joint name=\"j3\" type=\"revolute\">"
      "    <parent link=\"b\"/><child link=\"c\"/><axis xyz=\"0 0 1\"/>"
      "    <limit lower=\"-1\" upper=\"1\" effort=\"1\" velocity=\"1\"/>"
      "    <mimic joint=\"j2\"/></joint>"
      "</robot>";
  try {
    Robot::fromUrdfString(xml);
    FAIL("ocekavana vyjimka UrdfError");
  } catch (const UrdfError& e) {
    CHECK(std::string(e.what()).find("chained") != std::string::npos);
  }
}
