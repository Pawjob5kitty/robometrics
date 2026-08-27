# robometrics

Kinematické metriky pro offline hodnocení robotických politik (LIBERO, LeRobot).
C++20, běží nad uloženými rollouty, za běhu nekomunikuje s ničím.

Vertikální řez: **soubor s rolloutem na vstupu, CSV s metrikami na výstupu.**

```bash
robometrics analyze --urdf panda.urdf --tip panda_hand rollouts/*.csv --out report.csv
```

## Stav

| modul | stav |
|---|---|
| `se3` — `skew`, `rodrigues`, `SE3`, `exp`, `log`, `adjoint` | hotovo |
| `urdf` — parser URDF, kinematický strom | hotovo |
| `kinematics` — forward kinematics | hotovo |
| `jacobian` — hybridní geometrický Jacobián | hotovo |
| `metrics` — `dexterityMargin`, `pathEfficiency` | hotovo |
| `rollout` — vlastní textový formát, čtení i zápis | hotovo |
| `report` — `analyze`, lokalizace nízké obratnosti | hotovo |
| CLI `robometrics analyze` | hotovo |
| `python/robometrics_convert.py` — LIBERO/robosuite HDF5 → CSV | hotovo, volitelné |

Testy jsou property i unit, přes doctest a rapidcheck. Žádný test není označený
`doctest::may_fail()` — pokud se takový dekorátor někde objeví, znamená to, že
se testuje něco nehotového, a musí zmizet, jakmile implementace dorazí.

## Co to NEDĚLÁ

Poctivý seznam, protože každá z těch věcí zní jako něco, co by knihovna s tímhle
jménem mohla umět, a neumí:

- **Nic o kontaktu.** Síly, tření, uchopení, jestli robot něco upustil nebo
  rozdrtil. `dexterityMargin` může být vysoká v pozici, kde chapadlo drtí
  sklenici.
- **Nic o percepci.** Kamery, segmentace, jestli politika vidí správný objekt.
- **Nic o dynamice.** Momenty, setrvačnost, gravitace, jestli je pohyb vůbec
  proveditelný v rámci momentových limitů. Všechny metriky jsou **kinematické** —
  počítají se z `q` a z URDF, nic víc.
- **Nic o splnění úlohy.** `success` se jen přenáší ze vstupního souboru; tahle
  knihovna ho nikdy nevyhodnocuje.
- **Nic o kolizích.** Kolizní geometrie z URDF se přeskakuje.
- **Neporovnává napříč roboty.** Prahy i hodnoty škálují s velikostí robota,
  viz omezení níže.

Metriky měří **jak volně se end-effector mohl v dané konfiguraci pohybovat** a
**kolik kloubového pohybu bylo zbytečné**. Nic jiného.

### Známé mezery

- `log(T)` neošetřuje `θ → π`. Antisymetrická část `R` tam přestane nést osu a
  vrací nesmysl. Oprava vede přes symetrickou část (`R + I == 2·n·nᵀ`), viz
  komentář v `src/se3.cpp`. Testy tam nedosáhnou, protože omezují složky
  `omega` na 1.8.
- **`pathEfficiency` je 1 pro každého neredundantního robota.** Podmínka je
  `numDofs() > rank(J)`, ne „víc kloubů než rozměrů úlohy" — rovinné rameno má
  `rank(J) ≤ 3` bez ohledu na počet kloubů, takže planar 3R redundantní **není**.
  Znamená to 7-DOF Franka ano, 6-DOF UR ne. CLI na to varuje na stderr.
- **Práh nízké obratnosti škáluje s velikostí robota.** `σ_min` je v m/rad, tedy
  úměrné dosahu. Výchozí `0.05` je kalibrované na Frankovu škálu (~0.85 m); na
  malém rameni označí skoro všechno, na velkém skoro nic. Správně by se mělo
  normalizovat charakteristickou délkou z URDF.
- **`‖Δq‖` míchá radiány s metry**, pokud má robot rotační i posuvné klouby.
  Čitatel i jmenovatel `pathEfficiency` nesou stejnou vadu, takže poměr je míň
  špatný než každá půlka, ale správný není.
- Řetězené `mimic` klouby (mimic, jehož zdroj je sám mimic) parser odmítá.
  Jednoúrovňové fungují.
- `planar` a `floating` klouby jsou odmítnuté — jsou vícedimenzionální a
  nevejdou se do „jeden skalár na kloub".
- Shodné časové značky v rolloutu jsou povolené, protože žádná metrika podle
  času nederivuje. Až přijde časová metrika, tahle tolerance musí padnout.

## Použití od URDF po CSV

### 1. Převod dat (volitelné)

Pokud máš LIBERO/robosuite HDF5:

```bash
pip install h5py
python python/robometrics_convert.py libero demos/*.hdf5 \
    --out rollouts/ --robot panda_arm_hand.urdf
```

Skript **není součástí CMake buildu** a knihovna ho k ničemu nepotřebuje.
Do jeho docstringu je napsané to podstatné: `q` se bere z `obs/joint_pos`
(měřený stav), **ne** z `actions` (příkazy, u LIBERO výchozího OSC controlleru
dokonce end-effector delty, ne úhly kloubů vůbec), a jak se odvozuje `success`.

### 2. Analýza

```bash
cmake --preset release && cmake --build --preset release
./build/release/robometrics analyze \
    --urdf panda.urdf --tip panda_hand \
    rollouts/*.csv --out report.csv --profile-out profiles/
```

Report na stdout (nebo do `--out`):

```csv
file,dofs,steps,success,dexterity_margin,path_efficiency,low_dex_spans,worst_at
demo_001.csv,7,412,1,0.184,0.912,0,203
demo_042.csv,7,388,1,0.008,0.731,2,341
```

Souhrn a lokalizace na stderr:

```
skipping demo_017.csv: demo_017.csv:88: expected 8 fields (t plus 7 joint values), found 7
demo_042.csv: 2 low-dexterity spans  [112..140 worst 0.031]  [338..361 worst 0.008]
42 rollouts, 41 ok, 1 failed to parse
dexterity margin:   median 0.152   p05 0.011   min 0.003
path efficiency:    median 0.887   p05 0.702   min 0.610
8 rollouts (19%) have at least one low-dexterity span
```

Rollout, který se nepodaří načíst, běh nesestřelí. Návratový kód je 0, když
prošel aspoň jeden, jinak nenulový.

**Spany na stderr jsou inkluzivní** (`[112..140]` = kroky 112 až 140 včetně),
zatímco `Span::begin/end` v C++ je půlotevřený. Číslo `0.31` je k ničemu;
„kroky 338–361, nejhorší 0.008" se dá otevřít a podívat se.

`--profile-out` vypíše `step,t,dexterity` na krok do samostatných CSV, pro
kreslení grafů.

### 3. Formát rolloutu

```
# robometrics rollout v1
# robot: panda_arm_hand.urdf
# dofs: 7
# success: 1
t,q0,q1,q2,q3,q4,q5,q6
0.000,0.0,-0.785,0.0,-2.356,0.0,1.571,0.785
0.050,0.012,-0.781,0.0,-2.351,0.0,1.570,0.785
```

Textový, `git diff`-ovatelný, bez závislostí. Povinné je jen `dofs`; neznámé
klíče se zachovají. `t` v sekundách, `q` v SI. Hodnoty musí být konečné a `t`
nesmí klesat. Chyby nesou číslo řádku:

```
demo_001.csv:12: expected 8 fields (t plus 7 joint values), found 7
demo_001.csv:2: column header has 6 columns but metadata says dofs=7, which needs 8 (t plus q0..6)
```

## Konvence

Tyhle tři věci se v literatuře běžně dělají oběma způsoby. Míchat je nelze a
záměna se vždycky přeloží, takže je to napsané i tady, nejen v komentářích.

**Twist je `[v; omega]`** — translační část první. Opačná konvence `[omega; v]`
prohodí mimodiagonální bloky adjunktu.

**`rpy` v URDF je fixed-axis roll-pitch-yaw**, tedy `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.
Stejnou matici dá intrinsic ZYX. Obrácené pořadí `Rx·Ry·Rz` naparsuje každé
reálné URDF bez chyby a popisuje jiného robota.

**Osa kloubu žije v jeho vlastním rámci**, ne ve světovém — pohyb se proto
aplikuje zprava: `X_child = X_parent · origin · motion(q)`.

**Jacobián je hybridní**, ne spatial: rychlost bodu na chapadle vyjádřená
v orientaci báze. Liší se o `omega × p_tip`.

## Co parser dělá s URDF

1. Načte `<joint>` a `<link>`. Odmítne `planar`/`floating` a neznámé typy.
2. Najde kořen — jediný link, který není ničí dítě.
3. Topologicky seřadí klouby, aby rodič byl vždy před dítětem.
4. **Složí `fixed` klouby.** Prostřední se prolnou do `originTransform`
   následujícího pohyblivého kloubu, koncové přežijí jako `Link::offset`.
   Nic se nezahazuje — koncový fixed řetěz (`hand → flange → grasp target`) je
   běžný tvar konce manipulátoru.
5. Zapíše ke každému linku, který pohyblivý kloub ho nese.
6. Vyřeší `<mimic>` a přidělí indexy do konfiguračního vektoru.

Geometrie a topologie jsou jediné, co se čte. Inercie, meshe, kolizní geometrie,
materiály a gazebo tagy se přeskakují — knihovna hodnotí nahrané rollouty,
nesimuluje dynamiku.

Pozor na dvě čísla:

```cpp
robot.numJoints()  // vsechny pohyblive klouby (nezavisle + mimic)
robot.numDofs()    // delka q — jen nezavisle
```

Liší se přesně tehdy, když má robot `mimic` klouby. Bez nich jsou si rovné.

## Build

Potřebuješ CMake ≥ 3.20, Ninja a GCC nebo Clang. Závislosti (Eigen, doctest,
rapidcheck, pugixml) si stáhne CMake sám přes `FetchContent`.

Build vyrobí knihovnu `robometrics`, testy a binárku `robometrics` (CLI).
Python skript v `python/` součástí buildu **není**.

```bash
# release
cmake --preset release
cmake --build --preset release
ctest --preset release

# debug + AddressSanitizer/UndefinedBehaviorSanitizer
cmake --preset debug-asan
cmake --build --preset debug-asan
ctest --preset debug-asan
```

Preset `debug-asan` navíc přidává `-fno-sanitize-recover=all`, aby první nález
proces shodil. Bez toho UBSan chybu jen vypíše, test doběhne s návratovým kódem
0 a CI zůstane falešně zelená.

Adresář presetu se vytváří pod `build/`. Pokud byl vygenerovaný na jiné cestě
(typicky na Windows a pak čtený z WSL), CMake odmítne cache s hláškou o neshodě
adresáře — smaž ten podadresář a překonfiguruj.

### Testy

```bash
# vsechno
ctest --preset debug-asan --output-on-failure

# jen jeden test case, s vypisem
./build/debug-asan/tests/robometrics_tests -ts="*rpy*"

# vic opakovani property testu
RC_PARAMS="max_success=10000" ./build/debug-asan/tests/robometrics_tests

# reprodukce konkretniho padu (seed vypise rapidcheck sam)
RC_PARAMS="seed=42" ./build/debug-asan/tests/robometrics_tests
```

URDF fixtury jsou soubory v `tests/fixtures/`, ne inline stringy — testuje se
tím i cesta přes `load_file()` a při pádu se dají otevřít. Cestu předává CMake
makrem `ROBOMETRICS_FIXTURE_DIR`.

⚠️ V XML komentáři **nesmí být dvě pomlcky za sebou**. ASCII schéma řetězce
(`base --[joint1]--> link1`) komentář předčasně ukončí a pugixml pak hlásí
„Start-end tags mismatch" o desítky řádků dál.

### Sanitizery a Windows

`debug-asan` se na Windows s MSVC ABI sestaví a slinkuje, ale **použitelný tam
není** — ověřeno na clang 21.1.8:

1. ASan na Windows nepodporuje debug CRT, běh spadne na `bad-free` uvnitř
   `ucrtbased.dll`. Přepnutí na release CRT
   (`-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL`) tohle odstraní.
2. Zůstane ale pád v `access-violation` při unwindingu C++ výjimky skrz
   instrumentovaný rámec (rapidcheck chytá výjimky z property). ASan + SEH.

Sanitizery proto pouštěj v WSL nebo se spolehni na CI job `debug-asan`. Release
preset na Windows funguje normálně.

Pokud bys to na Windows přesto zkoušel: runtime ASanu je DLL, která není na
`PATH`, a bez ní binárka skončí na `0xc0000135` (DLL not found):

```bash
export PATH="/c/Program Files/LLVM/lib/clang/21/lib/windows:$PATH"
```

### clangd

`compile_commands.json` vzniká v adresáři presetu. Nasměruj na něj clangd
souborem `.clangd` v kořeni repozitáře:

```yaml
CompileFlags:
  CompilationDatabase: build/release
```

## Formátování

```bash
clang-format -i $(git ls-files '*.hpp' '*.cpp')
```

CI kontroluje formát přes `clang-format 21.1.8`. Distribuční balíčky se mezi
major verzemi liší ve výstupu, takže si nainstaluj přesně tuhle — stejnou cestou
jako CI:

```bash
python3 -m venv .venv-format
.venv-format/bin/pip install clang-format==21.1.8
.venv-format/bin/clang-format --dry-run --Werror $(git ls-files '*.hpp' '*.cpp')
```

Maticové literály jsou obalené `// clang-format off` / `on`. Bez toho se sloučí
na jeden řádek a znaménkový vzor, kvůli kterému tam jsou rozepsané, zmizí.

## Licence

TBD
