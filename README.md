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
file,dofs,steps,success,dexterity_norm,path_efficiency,low_dex_spans,worst_at
demo_001.csv,7,412,1,0.140,0.981,0,203
demo_042.csv,7,388,1,0.031,0.947,1,341
```

A summary, and the localisation of every low-dexterity stretch, goes to stderr:

```
characteristic length: 1.319 m (computed from URDF)
skipping demo_017.csv: demo_017.csv:88: expected 8 fields (t plus 7 joint values), found 7
demo_042.csv: 1 low-dexterity span  [338..361 worst 0.031]
50 rollouts, 49 ok, 1 failed to parse
dexterity (norm):   median 0.107   p05 0.070   min 0.054
path efficiency:    median 0.981   p05 0.956   min 0.932
0 rollouts (0%) have at least one low-dexterity span
```

A rollout that fails to load is skipped, not fatal; the exit code is 0 when at
least one rollout was analysed, non-zero otherwise. Spans on stderr are
**inclusive** (`[338..361]` = steps 338 to 361), while `Span::begin/end` in C++
is half-open. `--profile-out` writes a per-step `step,t,dexterity` CSV per
rollout, for plotting. The dexterity is normalised by the robot's characteristic
length L — the summed link lengths from the URDF, printed on stderr as above;
pass `--char-length` to override it when the URDF geometry cannot be trusted.

## What it measures

- **`dexterity_norm`** — the smallest singular value of the translational
  Jacobian at the worst step of the trajectory: how many metres per second of
  tip motion one unit of joint speed buys in the least-responsive direction.
  Zero means a direction is locked (a singularity). That raw value is then
  divided by the robot's characteristic length L — the summed link lengths from
  the URDF — so the reported number is dimensionless: σ_min scaled by the robot's
  size, so the same margin means the same thing on arms of different reach. It is
  *dexterity*, not safety (it knows nothing about contact or payload) and not
  mobility (it is a property of a configuration, not of the mechanism). After
  Yoshikawa (1985); the shortest-semi-axis reading is from Klein and Blaho (1987).
- **`path_efficiency`** — the ratio of the minimum-norm joint motion that would
  have produced the recorded tip path to the joint motion actually spent, in
  `(0, 1]`. It measures joint-space waste: motion that turned the joints without
  moving the tip. It is **N/A** (an empty column) for a non-redundant robot,
  where the ratio would be a constant 1 for lack of a null space, and for an arm
  mixing revolute and prismatic joints, where `‖Δq‖` would add radians and
  metres; the CLI states which reason applies.

## Result on LIBERO-Spatial

Run over all of `libero_spatial` — 10 tasks × 50 human teleoperation
demonstrations, 500 rollouts, 62 250 steps — on a Franka Panda (7-DOF arm,
gripper fingers fixed, characteristic length 1.319 m). The 500-rollout analysis
takes **0.7 s on one CPU core** (~5 MB peak); converting the 6 GB of HDF5 first
takes about 1.5 s.

```
                                    median    min      range
dexterity_norm (dimensionless)      0.107    0.054    0.054 .. 0.148
path_efficiency                     0.981    0.932    0.932 .. 0.999
low-dexterity spans (norm < 0.038)      0 of 500 rollouts
```

Per-task medians span a narrow band — dexterity 0.088 to 0.133, efficiency
0.973 to 0.988 — and no rollout in the whole suite drops below the normalised
0.038 dexterity threshold or above 1.0 in efficiency.

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
  crushed something. A high `dexterity_norm` says nothing about the glass in
  the gripper.
- **No perception.** Cameras, segmentation, whether the policy saw the right
  object.
- **No dynamics.** Torques, inertia, gravity, whether a motion is even feasible
  within torque limits.
- **No task success.** `success` is passed through from the input file; this
  library never evaluates it.
- **No collisions.** URDF collision geometry is skipped.
- **Cross-robot comparison only as far as the URDF is trustworthy.**
  `dexterity_norm` and `path_efficiency` are dimensionless by design, so they are
  meant to be compared across robots — but the dexterity normalisation is only as
  good as the characteristic length read from the URDF (see Known gaps).

## Known gaps

- **`dexterity_norm` is a description, not a prediction.** It is uncalibrated. A
  low value marks a configuration close to a kinematic singularity, but nothing
  in this library establishes that a low value predicts task failure, or that a
  high one predicts success. Read it as *where the arm was working hardest*, not
  as a score of whether the policy was good — the correlation with outcomes is
  unmeasured and would have to be shown, not assumed.
- **Normalisation is only as good as the URDF.** `dexterity_norm` divides `σ_min`
  by a characteristic length L — the summed link lengths along the base→tip
  chain. A URDF with missing, zero, or nonsense link lengths gives a wrong (or
  zero) L, and every dexterity number inherits the error silently. `--char-length`
  overrides L when the geometry cannot be trusted. (This normalisation is what
  makes the threshold dimensionless and robot-independent, replacing the old
  reach-dependent `0.05 m/rad`.)
- **`path_efficiency` needs a redundant, uniform-joint arm; otherwise it is N/A.**
  Redundancy is `numDofs() > rank(J)`, not "more joints than task dimensions" — a
  planar arm has `rank(J) ≤ 3` however many joints it has, so a planar 3R is not
  redundant and a 6-DOF UR gets N/A while a 7-DOF Franka does not. It is also N/A
  for an arm mixing revolute and prismatic joints, where `‖Δq‖` would add radians
  and metres. Both return an empty column with the reason on stderr, rather than
  the misleading constant 1 or a mixed-unit ratio they used to.
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

Tests are property- and unit-based, via doctest and rapidcheck: 163 cases, 771
assertions. Formatting is checked with `clang-format 21.1.8`.

## License

Apache License 2.0 — see [LICENSE](LICENSE).

Copyright 2026 Pawjob5kitty
