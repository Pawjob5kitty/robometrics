# robometrics

*[English version: README.md](README.md)*

Kinematické metriky pro offline hodnocení robotických manipulačních politik
(LIBERO, LeRobot). C++20, běží nad uloženými rollouty, za běhu nekomunikuje
s ničím. Načte URDF robota a jeho zaznamenané trajektorie kloubů a pro každý
rollout vypíše, jak volně se mohl pohybovat end-effector a kolik kloubového
pohybu bylo zbytečné — jako jedno CSV, jedním příkazem.

```bash
robometrics analyze --urdf panda.urdf --tip panda_hand rollouts/*.csv --out report.csv
```

## Od URDF po CSV

```bash
# 1. (volitelné) převod LIBERO/robosuite HDF5 do formátu rolloutu
pip install h5py
python python/robometrics_convert.py libero demos/*.hdf5 \
    --out rollouts/ --robot panda_arm.urdf

# 2. build a analýza
cmake --preset release && cmake --build --preset release
./build/release/robometrics analyze \
    --urdf panda.urdf --tip panda_hand \
    rollouts/*.csv --out report.csv --profile-out profiles/
```

Report je jeden řádek na rollout, na stdout nebo do `--out`:

```csv
file,dofs,steps,success,dexterity_norm,path_efficiency,low_dex_spans,worst_at
demo_001.csv,7,412,1,0.140,0.981,0,203
demo_042.csv,7,388,1,0.031,0.947,1,341
```

Souhrn a lokalizace každého úseku nízké obratnosti jde na stderr:

```
characteristic length: 1.319 m (computed from URDF)
skipping demo_017.csv: demo_017.csv:88: expected 8 fields (t plus 7 joint values), found 7
demo_042.csv: 1 low-dexterity span  [338..361 worst 0.031]
50 rollouts, 49 ok, 1 failed to parse
dexterity (norm):   median 0.107   p05 0.070   min 0.054
path efficiency:    median 0.981   p05 0.956   min 0.932
0 rollouts (0%) have at least one low-dexterity span
```

Rollout, který se nepodaří načíst, se přeskočí, běh nesestřelí; návratový kód je
0, když prošel aspoň jeden, jinak nenulový. Spany na stderr jsou **inkluzivní**
(`[338..361]` = kroky 338 až 361), zatímco `Span::begin/end` v C++ je
půlotevřený. `--profile-out` vypíše `step,t,dexterity` na krok do samostatného
CSV pro každý rollout, ke kreslení grafů. Obratnost je normalizovaná
charakteristickou délkou robota L — součtem délek článků z URDF, vypsaným na
stderr jako výše; přepínačem `--char-length` ji lze přebít, když geometrii URDF
nelze věřit.

## Co měří

- **`dexterity_norm`** — nejmenší singulární číslo translační části Jacobiánu
  v nejhorším kroku trajektorie: kolik metrů za sekundu pohybu tipu koupí
  jednotka kloubové rychlosti v nejméně citlivém směru. Nula znamená zamčený
  směr (singularita). Tato surová hodnota se pak vydělí charakteristickou délkou
  robota L — součtem délek článků z URDF — takže hlášené číslo je bezrozměrné:
  σ_min vydělené velikostí robota, aby stejná rezerva znamenala totéž na ramenou
  různého dosahu. Je to *obratnost*, ne bezpečnost (neví nic o kontaktu ani
  o zátěži) a ne mobilita (je to vlastnost konfigurace, ne mechanismu). Podle
  Yoshikawy (1985); nejkratší poloosa je z Klein a Blaho (1987).
- **`path_efficiency`** — poměr minimálního kloubového pohybu, který by vyrobil
  zaznamenanou dráhu tipu, ke skutečně vynaloženému kloubovému pohybu, v
  `(0, 1]`. Měří plýtvání v prostoru kloubů: pohyb, který kloubem otočil, ale
  tipem nehnul. Je **N/A** (prázdný sloupec) pro neredundantního robota, kde by
  poměr byl kvůli chybějícímu nullspace konstantní 1, a pro rameno mísící rotační
  a posuvné klouby, kde by `‖Δq‖` sčítalo radiány a metry; CLI řekne, který důvod
  platí.

## Výsledek na LIBERO-Spatial

Spuštěno na celém `libero_spatial` — 10 úloh × 50 lidských teleoperačních
demonstrací, 500 rolloutů, 62 250 kroků — na Franka Panda (7-DOF rameno,
prstové klouby zafixované, charakteristická délka 1.319 m). Analýza 500 rolloutů
trvá **0,7 s na jednom jádru CPU** (~5 MB špička); převod 6 GB HDF5 předtím
zabere zhruba 1,5 s.

```
                                    medián   min      rozsah
dexterity_norm (bezrozměrné)        0.107    0.054    0.054 .. 0.148
path_efficiency                     0.981    0.932    0.932 .. 0.999
úseky nízké obratnosti (norm < 0.038)   0 z 500 rolloutů
```

Mediány po úlohách jsou v úzkém pásmu — obratnost 0.088 až 0.133, efektivita
0.973 až 0.988 — a žádný rollout v celé sadě neklesne pod normalizovaný práh
obratnosti 0.038 ani nepřeleze efektivitu 1.0.

**Ber to jako zjištění o datasetu, ne o nástroji.** `libero_spatial` je z
konstrukce homogenní: deset variant téhož pick-and-place, samá lidská
teleoperace ponechaná jen při splnění úlohy. Demonstrace se drží daleko od
singularit a plýtvají málo kloubovým pohybem, a metriky to napříč všemi 500
konzistentně říkají. Dataset se skriptovanými nebo naučenými politikami, nebo
s úlohami, které ženou rameno k jeho mezím, je místo, kde by se rozptyl a spany
objevily.

## Co NEDĚLÁ

Všechny metriky jsou **kinematické** — počítají se z `q` a z URDF, nic víc.
Konkrétně:

- **Nic o kontaktu.** Síly, tření, uchopení, jestli robot něco upustil nebo
  rozdrtil. Vysoká `dexterity_norm` neříká nic o skleničce v chapadle.
- **Nic o percepci.** Kamery, segmentace, jestli politika viděla správný objekt.
- **Nic o dynamice.** Momenty, setrvačnost, gravitace, jestli je pohyb vůbec
  proveditelný v rámci momentových limitů.
- **Nic o splnění úlohy.** `success` se jen přenáší ze vstupního souboru; tahle
  knihovna ho nikdy nevyhodnocuje.
- **Nic o kolizích.** Kolizní geometrie z URDF se přeskakuje.
- **Porovnání napříč roboty jen do té míry, jak se dá věřit URDF.**
  `dexterity_norm` i `path_efficiency` jsou z návrhu bezrozměrné, takže jsou
  určené k porovnávání napříč roboty — ale normalizace obratnosti je jen tak
  dobrá jako charakteristická délka načtená z URDF (viz Známé mezery).

## Známé mezery

- **`dexterity_norm` je popis, ne predikce.** Je nekalibrovaná. Nízká hodnota
  značí konfiguraci blízko kinematické singularity, ale nic v této knihovně
  neprokazuje, že nízká hodnota předpovídá selhání úlohy nebo že vysoká
  předpovídá úspěch. Ber ji jako *kde rameno pracovalo nejtíž*, ne jako známku
  toho, jestli byla politika dobrá — korelace s výsledky je neměřená a musela by
  se ukázat, ne předpokládat.
- **Normalizace je jen tak dobrá jako URDF.** `dexterity_norm` dělí `σ_min`
  charakteristickou délkou L — součtem délek článků podél řetěze báze→tip. URDF
  s chybějícími, nulovými nebo nesmyslnými délkami článků dá špatnou (nebo
  nulovou) L a každé číslo obratnosti tu chybu tiše zdědí. `--char-length` L
  přebije, když geometrii nelze věřit. (Právě tahle normalizace dělá práh
  bezrozměrným a nezávislým na robotovi, místo starého `0.05 m/rad` závislého na
  dosahu.)
- **`path_efficiency` potřebuje redundantní rameno s jednotným typem kloubů;
  jinak je N/A.** Redundance je `numDofs() > rank(J)`, ne „víc kloubů než rozměrů
  úlohy" — rovinné rameno má `rank(J) ≤ 3` bez ohledu na počet kloubů, takže
  planar 3R redundantní není a 6-DOF UR dostane N/A, zatímco 7-DOF Franka ne. Je
  N/A i pro rameno mísící rotační a posuvné klouby, kde by `‖Δq‖` sčítalo radiány
  a metry. Obojí vrátí prázdný sloupec s důvodem na stderr, místo zavádějící
  konstantní 1 nebo poměru ve smíšených jednotkách, které vracelo dřív.
- Řetězené `mimic` klouby (mimic, jehož zdroj je sám mimic) jsou odmítnuté;
  jednoúrovňové fungují.
- `planar` a `floating` klouby jsou odmítnuté jako vícedimenzionální.
- Shodné časové značky v rolloutu jsou povolené, protože žádná metrika podle
  času nederivuje. Ta tolerance by musela padnout, jakmile přijde časová metrika.

## Konvence

Tři věci, které se v literatuře dělají oběma způsoby; záměna se přeloží tiše,
takže jsou tady i v kódu.

- **Twist je `[v; omega]`** — translační část první. Opačná konvence prohodí
  mimodiagonální bloky adjunktu.
- **`rpy` v URDF je fixed-axis roll-pitch-yaw**, `R = Rz(yaw)·Ry(pitch)·Rx(roll)`
  (ekvivalentně intrinsic ZYX). Obrácené pořadí `Rx·Ry·Rz` naparsuje každé
  reálné URDF a popisuje jiného robota.
- **Osa kloubu žije ve vlastním rámci**, takže se pohyb aplikuje zprava:
  `X_child = X_parent · origin · motion(q)`.
- **Jacobián je hybridní**, ne spatial: rychlost bodu na chapadle v orientaci
  báze. Ty dva se liší o `omega × p_tip`.

## Formát rolloutu

Textový, `git diff`-ovatelný, bez závislostí. Povinné je jen `dofs`; neznámé
metadatové klíče se zachovají. `t` v sekundách, `q` v SI (radiány / metry);
hodnoty musí být konečné a `t` nesmí klesat.

```
# robometrics rollout v1
# robot: panda_arm.urdf
# dofs: 7
# success: 1
t,q0,q1,q2,q3,q4,q5,q6
0.000,0.0,-0.785,0.0,-2.356,0.0,1.571,0.785
0.050,0.012,-0.781,0.0,-2.351,0.0,1.570,0.785
```

Volitelný převodník bere úhly kloubů z `obs/joint_states` (měřený stav),
**ne** z `actions` (příkazy controlleru — u výchozího OSC controlleru LIBERO
jsou to end-effector delty, ne úhly kloubů vůbec). `success` je v těchto
souborech zapsán nepodmíněně nahrávačem, takže se hlásí jako `assumed`, ne
měřený. Viz docstring modulu v `python/robometrics_convert.py`.

## Build

CMake ≥ 3.20, Ninja a GCC nebo Clang. Závislosti (Eigen, doctest, rapidcheck,
pugixml) si stáhne CMake přes `FetchContent`; Python převodník není součástí
buildu a k použití knihovny není potřeba.

```bash
cmake --preset release      && cmake --build --preset release      && ctest --preset release
cmake --preset debug-asan   && cmake --build --preset debug-asan   && ctest --preset debug-asan
```

Testy jsou property i unit, přes doctest a rapidcheck: 163 case, 771 assertions.
Formát kontroluje `clang-format 21.1.8`.

## Licence

Apache License 2.0 — viz [LICENSE](LICENSE).

Copyright 2026 Pawjob5kitty
