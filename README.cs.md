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
file,dofs,steps,success,dexterity_margin,path_efficiency,low_dex_spans,worst_at
demo_001.csv,7,412,1,0.184,0.981,0,203
demo_042.csv,7,388,1,0.121,0.947,1,341
```

Souhrn a lokalizace každého úseku nízké obratnosti jde na stderr:

```
skipping demo_017.csv: demo_017.csv:88: expected 8 fields (t plus 7 joint values), found 7
demo_042.csv: 1 low-dexterity span  [338..361 worst 0.031]
50 rollouts, 49 ok, 1 failed to parse
dexterity margin:   median 0.141   p05 0.092   min 0.072
path efficiency:    median 0.981   p05 0.956   min 0.932
0 rollouts (0%) have at least one low-dexterity span
```

Rollout, který se nepodaří načíst, se přeskočí, běh nesestřelí; návratový kód je
0, když prošel aspoň jeden, jinak nenulový. Spany na stderr jsou **inkluzivní**
(`[338..361]` = kroky 338 až 361), zatímco `Span::begin/end` v C++ je
půlotevřený. `--profile-out` vypíše `step,t,dexterity` na krok do samostatného
CSV pro každý rollout, ke kreslení grafů.

## Co měří

- **`dexterityMargin`** — nejmenší singulární číslo translační části Jacobiánu
  v nejhorším kroku trajektorie: kolik metrů za sekundu pohybu tipu koupí
  jednotka kloubové rychlosti v nejméně citlivém směru. Nula znamená zamčený
  směr (singularita). Je to *obratnost*, ne bezpečnost (neví nic o kontaktu ani
  o zátěži) a ne mobilita (je to vlastnost konfigurace, ne mechanismu). Podle
  Yoshikawy (1985); nejkratší poloosa je z Klein a Blaho (1987).
- **`pathEfficiency`** — poměr minimálního kloubového pohybu, který by vyrobil
  zaznamenanou dráhu tipu, ke skutečně vynaloženému kloubovému pohybu, v
  `(0, 1]`. Měří plýtvání v prostoru kloubů: pohyb, který kloubem otočil, ale
  tipem nehnul.

## Výsledek na LIBERO-Spatial

Spuštěno na celém `libero_spatial` — 10 úloh × 50 lidských teleoperačních
demonstrací, 500 rolloutů, 62 250 kroků — na Franka Panda (7-DOF rameno,
prstové klouby zafixované). Analýza 500 rolloutů trvá **0,7 s na jednom jádru
CPU** (~5 MB špička); převod 6 GB HDF5 předtím zabere zhruba 1,5 s.

```
                              medián   min      rozsah
dexterity_margin (m/rad)      0.141    0.072    0.072 .. 0.195
path_efficiency               0.981    0.932    0.932 .. 0.999
úseky nízké obratnosti (< 0.05)   0 z 500 rolloutů
```

Mediány po úlohách jsou v úzkém pásmu — obratnost 0.116 až 0.176, efektivita
0.973 až 0.988 — a žádný rollout v celé sadě neklesne pod práh obratnosti 0.05
ani nepřeleze efektivitu 1.0.

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
  rozdrtil. Vysoká `dexterityMargin` neříká nic o skleničce v chapadle.
- **Nic o percepci.** Kamery, segmentace, jestli politika viděla správný objekt.
- **Nic o dynamice.** Momenty, setrvačnost, gravitace, jestli je pohyb vůbec
  proveditelný v rámci momentových limitů.
- **Nic o splnění úlohy.** `success` se jen přenáší ze vstupního souboru; tahle
  knihovna ho nikdy nevyhodnocuje.
- **Nic o kolizích.** Kolizní geometrie z URDF se přeskakuje.
- **Neporovnává napříč roboty.** Prahy i hodnoty škálují s velikostí robota
  (viz níže).

## Známé mezery

- `log(T)` neošetřuje `θ → π`. Osa se získává z antisymetrické části `R`, která
  tam mizí; oprava vede přes symetrickou část (`R + I == 2·n·nᵀ`), viz komentář
  v `src/se3.cpp`. Testy se tomu vyhýbají omezením každé složky `omega` na 1.8.
- **`pathEfficiency` je 1 pro každého neredundantního robota.** Podmínka je
  `numDofs() > rank(J)`, ne „víc kloubů než rozměrů úlohy" — rovinné rameno má
  `rank(J) ≤ 3` bez ohledu na počet kloubů, takže planar 3R redundantní *není*.
  Znamená to 7-DOF Franka ano, 6-DOF UR ne; CLI na to varuje na stderr.
- **Práh obratnosti škáluje s velikostí robota.** `σ_min` je v m/rad, tedy
  úměrné dosahu. Výchozí `0.05` je kalibrované na Frankovu škálu (~0.85 m); na
  menším rameni označí skoro všechno, na větším skoro nic. Zásadové řešení je
  normalizovat charakteristickou délkou z URDF.
- **`‖Δq‖` míchá radiány s metry** u ramene s rotačními i posuvnými klouby.
  Čitatel i jmenovatel `pathEfficiency` nesou stejnou vadu, takže poměr je míň
  špatný než každá půlka, ale správný není.
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

Testy jsou property i unit, přes doctest a rapidcheck: 148 case, 630 assertions.
Formát kontroluje `clang-format 21.1.8`.

## Licence

Apache License 2.0 — viz [LICENSE](LICENSE).

Copyright 2026 Pawjob5kitty
