# robometrics

Kinematické metriky pro offline hodnocení robotických politik (LIBERO, LeRobot).
C++20, běží nad uloženými rollouty, za běhu nekomunikuje s ničím.

## Stav

| modul | stav |
|---|---|
| `se3` — `skew`, `rodrigues`, `SE3`, `exp`, `log`, `adjoint` | hotovo |
| `urdf` — parser URDF, kinematický strom | hotovo |
| `kinematics` — forward kinematics | hotovo |
| Jacobián | chybí |
| metriky nad rollouty | chybí |

Testy jsou property i unit, přes doctest a rapidcheck. Žádný test není označený
`doctest::may_fail()` — pokud se takový dekorátor někde objeví, znamená to, že
se testuje něco nehotového, a musí zmizet, jakmile implementace dorazí.

### Známé mezery

- `log(T)` neošetřuje `θ → π`. Antisymetrická část `R` tam přestane nést osu a
  vrací nesmysl. Oprava vede přes symetrickou část (`R + I == 2·n·nᵀ`), viz
  komentář v `src/se3.cpp`. Testy tam nedosáhnou, protože omezují složky
  `omega` na 1.8.
- Řetězené `mimic` klouby (mimic, jehož zdroj je sám mimic) parser odmítá.
  Jednoúrovňové fungují.
- `planar` a `floating` klouby jsou odmítnuté — jsou vícedimenzionální a
  nevejdou se do „jeden skalár na kloub".

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
