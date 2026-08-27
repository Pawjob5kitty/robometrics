# robometrics

*[Česká verze: README.cs.md](README.cs.md)*

Kinematic metrics for the offline evaluation of robot manipulation policies
(LIBERO, LeRobot). C++20, runs over recorded rollouts, talks to nothing at run
time. It reads a robot's URDF and its recorded joint trajectories and reports,
per rollout, how freely the end-effector could move and how much of the joint
motion was wasted — as one CSV, in one command.

```bash
robometrics analyze --urdf panda.urdf --tip panda_hand rollouts/*.csv --out report.csv
```

## From URDF to CSV

```bash
# 1. (optional) convert LIBERO/robosuite HDF5 to the rollout format
pip install h5py
python python/robometrics_convert.py libero demos/*.hdf5 \
    --out rollouts/ --robot panda_arm.urdf

# 2. build and analyse
cmake --preset release && cmake --build --preset release
./build/release/robometrics analyze \
    --urdf panda.urdf --tip panda_hand \
    rollouts/*.csv --out report.csv --profile-out profiles/
```

The report is one row per rollout, on stdout or to `--out`:

```csv
file,dofs,steps,success,dexterity_margin,path_efficiency,low_dex_spans,worst_at
demo_001.csv,7,412,1,0.184,0.981,0,203
demo_042.csv,7,388,1,0.121,0.947,1,341
```

A summary, and the localisation of every low-dexterity stretch, goes to stderr:

```
skipping demo_017.csv: demo_017.csv:88: expected 8 fields (t plus 7 joint values), found 7
demo_042.csv: 1 low-dexterity span  [338..361 worst 0.031]
50 rollouts, 49 ok, 1 failed to parse
dexterity margin:   median 0.141   p05 0.092   min 0.072
path efficiency:    median 0.981   p05 0.956   min 0.932
0 rollouts (0%) have at least one low-dexterity span
```

A rollout that fails to load is skipped, not fatal; the exit code is 0 when at
least one rollout was analysed, non-zero otherwise. Spans on stderr are
**inclusive** (`[338..361]` = steps 338 to 361), while `Span::begin/end` in C++
is half-open. `--profile-out` writes a per-step `step,t,dexterity` CSV per
rollout, for plotting.

## What it measures

- **`dexterityMargin`** — the smallest singular value of the translational
  Jacobian at the worst step of the trajectory: how many metres per second of
  tip motion one unit of joint speed buys in the least-responsive direction.
  Zero means a direction is locked (a singularity). It is *dexterity*, not
  safety (it knows nothing about contact or payload) and not mobility (it is a
  property of a configuration, not of the mechanism). After Yoshikawa (1985);
  the shortest-semi-axis reading is from Klein and Blaho (1987).
- **`pathEfficiency`** — the ratio of the minimum-norm joint motion that would
  have produced the recorded tip path to the joint motion actually spent, in
  `(0, 1]`. It measures joint-space waste: motion that turned the joints without
  moving the tip.

## Result on LIBERO-Spatial

Run over all of `libero_spatial` — 10 tasks × 50 human teleoperation
demonstrations, 500 rollouts, 62 250 steps — on a Franka Panda (7-DOF arm,
gripper fingers fixed). The 500-rollout analysis takes **0.7 s on one CPU core**
(~5 MB peak); converting the 6 GB of HDF5 first takes about 1.5 s.

```
                              median    min      range
dexterity_margin (m/rad)      0.141    0.072    0.072 .. 0.195
path_efficiency               0.981    0.932    0.932 .. 0.999
low-dexterity spans (< 0.05)      0 of 500 rollouts
```

Per-task medians span a narrow band — dexterity 0.116 to 0.176, efficiency
0.973 to 0.988 — and no rollout in the whole suite drops below the 0.05
dexterity threshold or above 1.0 in efficiency.

**Read this as a finding about the dataset, not a limitation of the tool.**
`libero_spatial` is homogeneous by construction: ten variants of the same
pick-and-place, all human teleoperation kept only when the task was solved. The
demonstrations stay well clear of singularities and waste little joint motion,
and the metrics say so consistently across all 500. A dataset with scripted or
learned policies, or with tasks that drive the arm near its limits, is where
the spread and the spans would appear.

## What it does NOT do

Every metric is **kinematic** — computed from `q` and the URDF, nothing else.
In particular:

- **No contact.** Forces, friction, grasping, whether the robot dropped or
  crushed something. A high `dexterityMargin` says nothing about the glass in
  the gripper.
- **No perception.** Cameras, segmentation, whether the policy saw the right
  object.
- **No dynamics.** Torques, inertia, gravity, whether a motion is even feasible
  within torque limits.
- **No task success.** `success` is passed through from the input file; this
  library never evaluates it.
- **No collisions.** URDF collision geometry is skipped.
- **No cross-robot comparison.** Thresholds and values scale with robot size
  (see below).

## Known gaps

- `log(T)` does not handle `θ → π`. The axis is recovered from the
  antisymmetric part of `R`, which vanishes there; the fix routes through the
  symmetric part (`R + I == 2·n·nᵀ`), documented in `src/se3.cpp`. The tests
  stay clear of it by bounding each `omega` component at 1.8.
- **`pathEfficiency` is 1 for any non-redundant robot.** The condition is
  `numDofs() > rank(J)`, not "more joints than task dimensions" — a planar arm
  has `rank(J) ≤ 3` however many joints it has, so a planar 3R is *not*
  redundant. This means it says something about a 7-DOF Franka and nothing about
  a 6-DOF UR; the CLI warns on stderr when the robot is non-redundant.
- **The dexterity threshold scales with robot size.** `σ_min` is in m/rad, so it
  is proportional to reach. The default `0.05` is calibrated to Panda scale
  (~0.85 m); on a smaller arm it flags almost everything, on a larger one almost
  nothing. The principled fix is to normalise by a characteristic length from
  the URDF.
- **`‖Δq‖` mixes radians and metres** for an arm with both revolute and
  prismatic joints. Numerator and denominator of `pathEfficiency` carry the same
  defect, so the ratio is less wrong than either half, but not right.
- Chained `mimic` joints (a mimic whose source is itself a mimic) are rejected;
  single-level ones work.
- `planar` and `floating` joints are rejected as multi-DOF.
- Equal consecutive timestamps in a rollout are allowed, because no metric here
  differentiates by time. That tolerance would have to go the moment a
  time-based metric is added.

## Conventions

Three things the literature does both ways; a mix-up compiles silently, so they
are stated here as well as in the code.

- **A twist is `[v; omega]`** — translational part first. The other convention
  transposes the off-diagonal blocks of the adjoint.
- **URDF `rpy` is fixed-axis roll-pitch-yaw**, `R = Rz(yaw)·Ry(pitch)·Rx(roll)`
  (equivalently intrinsic ZYX). The reversed order `Rx·Ry·Rz` parses every real
  URDF and describes a different robot.
- **A joint axis lives in its own frame**, so motion multiplies from the right:
  `X_child = X_parent · origin · motion(q)`.
- **The Jacobian is hybrid**, not spatial: the velocity of the point on the
  gripper, in the base orientation. The two differ by `omega × p_tip`.

## Rollout format

Text, `git diff`-able, no dependencies. Only `dofs` is required; unknown
metadata keys are preserved. `t` is in seconds, `q` in SI (radians / metres);
values must be finite and `t` must not decrease.

```
# robometrics rollout v1
# robot: panda_arm.urdf
# dofs: 7
# success: 1
t,q0,q1,q2,q3,q4,q5,q6
0.000,0.0,-0.785,0.0,-2.356,0.0,1.571,0.785
0.050,0.012,-0.781,0.0,-2.351,0.0,1.570,0.785
```

The optional converter reads joint angles from `obs/joint_states` (the measured
state), **not** from `actions` (controller commands — for LIBERO's default OSC
controller these are end-effector deltas, not joint angles at all). `success` in
these files is written unconditionally by the recorder, so it is reported as
`assumed`, not measured. See the module docstring in
`python/robometrics_convert.py`.

## Build

CMake ≥ 3.20, Ninja, and GCC or Clang. Dependencies (Eigen, doctest,
rapidcheck, pugixml) are fetched by CMake via `FetchContent`; the Python
converter is not part of the build and is not needed to use the library.

```bash
cmake --preset release      && cmake --build --preset release      && ctest --preset release
cmake --preset debug-asan   && cmake --build --preset debug-asan   && ctest --preset debug-asan
```

Tests are property- and unit-based, via doctest and rapidcheck: 148 cases, 630
assertions. Formatting is checked with `clang-format 21.1.8`.

## License

TBD
