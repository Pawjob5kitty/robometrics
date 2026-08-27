#!/usr/bin/env python3
"""Convert recorded demonstrations into the robometrics rollout format.

WHY THIS IS PYTHON AND NOT C++
------------------------------
The C++ core builds anywhere that has Eigen and pugixml, and nothing else.
Reading HDF5 or parquet would drag a large, version-sensitive dependency into
every build of a library whose main job is computing a Jacobian.

The formats demonstrations arrive in are also the fastest-moving part of this
whole pipeline -- dataset layouts change between releases of LIBERO and
LeRobot in ways the metrics never do. Keeping the conversion here means that
churn costs an edit to one script instead of a rebuild and a dependency bump.

This file is deliberately NOT part of the CMake build and is not needed to use
the library.

ADDING A NEW SOURCE
-------------------
Write one function with this signature and register it in SOURCES:

    def read_<name>(path: str, *, on_error: Callable[[str], None]) -> Iterator[Rollout]

It yields Rollout objects; everything else -- the output format, the file
naming, the CLI -- is shared. That is the whole extension point.

Two failure granularities, and the distinction matters. A problem with the
FILE (cannot open it, wrong layout) should raise: there is nothing to salvage.
A problem with one EPISODE inside an otherwise good file should call on_error
and skip that episode, because the other fifty episodes are still fine. Only
the reader knows which is which, so the reader makes the call.

USAGE
-----
    python robometrics_convert.py libero demo.hdf5 --out rollouts/
    python robometrics_convert.py libero 'data/*.hdf5' --out rollouts/ --dt 0.05
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from dataclasses import dataclass, field
from typing import Callable, Iterator, List


# ---------------------------------------------------------------------------
# The output format
# ---------------------------------------------------------------------------


@dataclass
class Rollout:
    """One trajectory, ready to be written out."""

    name: str
    t: List[float]
    q: List[List[float]]
    meta: dict = field(default_factory=dict)

    @property
    def dofs(self) -> int:
        return len(self.q[0]) if self.q else 0


def write_rollout(rollout: Rollout, out_dir: str) -> str:
    """Writes one rollout and returns the path it went to.

    Numbers go out through repr(), which in Python 3 is the shortest decimal
    that round-trips -- the same guarantee the C++ writer makes, so a file that
    goes through both is unchanged.
    """
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, rollout.name + ".csv")

    with open(path, "w", encoding="utf-8") as f:
        f.write("# robometrics rollout v1\n")
        f.write(f"# dofs: {rollout.dofs}\n")
        # Sorted, so two runs produce byte-identical files and the output is
        # diffable. The C++ writer does the same for the same reason.
        for key in sorted(rollout.meta):
            value = str(rollout.meta[key]).replace("\n", " ")
            f.write(f"# {key}: {value}\n")
        f.write("t," + ",".join(f"q{i}" for i in range(rollout.dofs)) + "\n")
        for time, row in zip(rollout.t, rollout.q):
            f.write(repr(float(time)) + "," + ",".join(repr(float(v)) for v in row) + "\n")
    return path


def validate(rollout: Rollout) -> None:
    """Catches here what the C++ parser would otherwise catch later.

    Failing at conversion time is worth the few lines: the error can still name
    the source dataset and the demo inside it, which the C++ side cannot,
    because by then that context is gone.
    """
    if not rollout.q:
        raise ValueError(f"{rollout.name}: no steps")
    width = len(rollout.q[0])
    if width == 0:
        raise ValueError(f"{rollout.name}: zero joints")
    for i, row in enumerate(rollout.q):
        if len(row) != width:
            raise ValueError(
                f"{rollout.name}: step {i} has {len(row)} joints, expected {width}"
            )
        for j, v in enumerate(row):
            if not _finite(v):
                raise ValueError(f"{rollout.name}: step {i}, joint {j} is not finite ({v})")
    for i in range(1, len(rollout.t)):
        if rollout.t[i] < rollout.t[i - 1]:
            raise ValueError(
                f"{rollout.name}: t decreases at step {i} "
                f"({rollout.t[i]} after {rollout.t[i - 1]})"
            )


def _finite(v: float) -> bool:
    return v == v and v not in (float("inf"), float("-inf"))


# ---------------------------------------------------------------------------
# LIBERO / robosuite HDF5
# ---------------------------------------------------------------------------
#
# WHERE THE JOINT ANGLES COME FROM, and how success is decided. This is the
# part nobody can guess and the part that decides whether every downstream
# number is about the right thing, so it is spelled out rather than left to be
# read off the code.
#
# Both claims below were checked against the actual libero_spatial release
# (10 files, 500 demos, 62250 steps), not only against the generator source.
#
# LAYOUT, as it really is on disk:
#
#     /data                             group, one child per demonstration
#     /data.attrs["num_demos"]          50 per file
#     /data.attrs["env_args"]           JSON: env_name, control_freq, ...
#     /data.attrs["problem_info"]       JSON: language_instruction
#     /data/demo_0/obs/joint_states     (T, 7)  float64  <- q, the arm
#     /data/demo_0/obs/gripper_states   (T, 2)  float64  both fingers, measured
#     /data/demo_0/obs/ee_states        (T, 6)  pos + axis-angle
#     /data/demo_0/actions              (T, 7)
#     /data/demo_0/robot_states         (T, 9)
#     /data/demo_0/states               (T, 92) full simulator state
#     /data/demo_0/rewards, dones       (T,)    uint8
#
# THE KEY IS obs/joint_states, NOT obs/joint_pos. The observable inside the
# simulator is called robot0_joint_pos, but LIBERO's scripts/create_dataset.py
# stores it under the name joint_states:
#
#     joint_states.append(obs["robot0_joint_pos"])            # line 201
#     obs_grp.create_dataset("joint_states", data=...)        # line 243
#
# There is no obs/joint_pos in the files at all. An earlier version of this
# converter looked for that name and would have failed on every demo.
#
# q COMES FROM obs/joint_states, NOT FROM actions. `actions` are commands in
# whatever space the controller uses -- for LIBERO's default OSC controller
# they are end-effector deltas, not joint angles at all, and even with a
# joint-space controller they are targets the arm may never have reached.
# joint_states is the measured state, which is what a kinematic metric has to
# be computed on. Using actions would produce a full set of plausible numbers
# about a trajectory the robot never executed.
#
# The arm only, seven values. Gripper fingers live in gripper_states and are
# NOT appended by default: the analysed URDF is normally the arm alone, and a
# width mismatch is exactly what the C++ side is there to reject. --gripper-key
# brings them in for a URDF that models the fingers.
#
# SUCCESS CANNOT BE MEASURED FROM THESE FILES. create_dataset.py writes both
# arrays unconditionally:
#
#     dones   = np.zeros(len(actions)); dones[-1]   = 1     # lines 224-227
#     rewards = np.zeros(len(actions)); rewards[-1] = 1
#
# So a "reward > 0" test passes for every demonstration ever written, whatever
# happened in it. Verified: across all 500 libero_spatial demos, rewards and
# dones are exactly [0, ..., 0, 1] in 500 of 500 cases -- zero information.
#
# The value 1 is still correct, because LIBERO demonstrations are human
# teleoperation kept only when the task was solved. What must not happen is
# reporting it as MEASURED. The output therefore carries success_source, and
# for these files it says "assumed", so a reader can tell a real success signal
# from a convention. If a future dataset ever writes rewards that deviate from
# that constant pattern, _libero_success notices and says so.
#
# OUTPUT NAMING. LIBERO files are called "<task>_demo.hdf5" and their episodes
# "demo_<n>", so a trailing "_demo" is stripped from the file stem before
# joining: "<task>_demo_0.csv" rather than "<task>_demo_demo_0.csv". The full
# original filename stays in the `source` metadata key, so nothing is lost.
#
# TIME. robosuite does not store timestamps. They are synthesised from a fixed
# control period (--dt, default 0.05 s = 20 Hz, matching env_args control_freq
# in these files). Since every metric in this library is geometric and never
# differentiates by time, this only affects plot axes -- but it is synthesised,
# and the metadata says so.


def read_libero(
    path: str,
    dt: float = 0.05,
    gripper_key: str = "",
    on_error: Callable[[str], None] = lambda msg: None,
) -> Iterator[Rollout]:
    try:
        import h5py  # imported lazily so the other sources work without it
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise SystemExit(
            "reading HDF5 needs h5py: pip install h5py\n(original error: %s)" % exc
        ) from exc

    import json

    # LIBERO names every file "<task>_demo.hdf5" and every episode inside it
    # "demo_<n>", so the naive stem + episode gives "<task>_demo_demo_0" -- the
    # word twice, in an already 80-character name that ends up in the `file`
    # column of every report. Trimming the file's trailing "_demo" loses
    # nothing: which episode it was still comes from the episode name, and the
    # untrimmed original is preserved in the `source` metadata key.
    stem = os.path.splitext(os.path.basename(path))[0]
    if stem.endswith("_demo"):
        stem = stem[: -len("_demo")]

    with h5py.File(path, "r") as f:
        if "data" not in f:
            raise ValueError(f"{path}: no /data group; is this a robosuite-style file?")
        data = f["data"]

        env_name = ""
        raw_env = data.attrs.get("env_args", "")
        if raw_env:
            try:
                env_name = json.loads(raw_env).get("env_name", "")
            except (ValueError, TypeError):
                env_name = ""

        # Sorted by trailing number, so demo_2 comes before demo_10 and the
        # output file order matches the obvious reading order.
        for demo in sorted(data.keys(), key=_demo_sort_key):
            # Per-episode failures are contained here rather than raised. A
            # generator cannot be resumed after it raises, so letting one bad
            # episode escape would silently drop every episode after it -- the
            # caller would see a partial conversion and a single error message
            # that does not mention the ones it lost.
            try:
                group = data[demo]
                if "obs" not in group or "joint_states" not in group["obs"]:
                    raise ValueError(
                        "no obs/joint_states. Joint angles are the measured state and "
                        "cannot be substituted with actions; see the module docstring."
                    )

                q = [list(map(float, row)) for row in group["obs"]["joint_states"][()]]

                if gripper_key:
                    if gripper_key not in group["obs"]:
                        raise ValueError(f"no obs/{gripper_key}")
                    grip = group["obs"][gripper_key][()]
                    if len(grip) != len(q):
                        raise ValueError(
                            f"{gripper_key} has {len(grip)} steps but joint_states has {len(q)}"
                        )
                    q = [row + list(map(float, g)) for row, g in zip(q, grip)]

                success, source = _libero_success(group)
            except Exception as exc:  # noqa: BLE001 - one episode, not the file
                on_error(f"{os.path.basename(path)}/{demo}: {exc}")
                continue

            meta = {
                "robot": "",  # filled in by the caller if --robot was given
                "source": os.path.basename(path),
                "source_demo": demo,
                "success": success,
                "success_source": source,
                "dt_synthetic": repr(dt),
            }
            if env_name:
                meta["task"] = env_name

            yield Rollout(
                name=f"{stem}_{demo}",
                t=[i * dt for i in range(len(q))],
                q=q,
                meta=meta,
            )


def _demo_sort_key(name: str):
    tail = name.rsplit("_", 1)[-1]
    return (0, int(tail)) if tail.isdigit() else (1, name)


def _libero_success(group) -> tuple:
    """Returns (success, the rule that decided it), never a bare guess.

    LIBERO's generator writes rewards and dones as the constant pattern
    [0, ..., 0, 1] for every demonstration, so reading them proves nothing.
    This checks for exactly that pattern and reports "assumed" when it finds
    it, rather than dressing a constant up as a measurement.

    A file whose rewards DEVIATE from the pattern is a different matter -- that
    is a real signal and is reported as measured.
    """
    rewards = group["rewards"][()] if "rewards" in group else None
    if rewards is not None and len(rewards):
        n = len(rewards)
        constant = [0] * (n - 1) + [1]
        if [int(v) for v in rewards] != constant:
            # Not the generator's constant, so somebody actually recorded
            # something. Trust it.
            return (1 if float(max(rewards)) > 0.0 else 0), "reward>0 (non-constant rewards)"

    # The constant pattern, or no rewards at all. LIBERO demonstrations are
    # human teleoperation filtered for success, so 1 is right -- but it is a
    # property of how the dataset was built, not of this episode.
    return 1, "assumed (LIBERO rewards/dones are written unconditionally)"


# ---------------------------------------------------------------------------
# Registry and CLI
# ---------------------------------------------------------------------------

SOURCES: dict = {
    "libero": read_libero,
}


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Convert demonstrations into the robometrics rollout format.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="See the module docstring for where q and success come from.",
    )
    parser.add_argument("source", choices=sorted(SOURCES), help="input format")
    parser.add_argument("inputs", nargs="+", help="input files (globs are expanded)")
    parser.add_argument("--out", required=True, help="output directory")
    parser.add_argument(
        "--dt", type=float, default=0.05,
        help="synthetic control period in seconds (default 0.05 = 20 Hz)")
    parser.add_argument(
        "--robot", default="",
        help="URDF filename to record in the metadata, e.g. panda_arm_hand.urdf")
    parser.add_argument(
        "--gripper-key", default="",
        help="also append this obs key to q, for a URDF that includes the fingers")
    args = parser.parse_args(argv)

    paths: List[str] = []
    for pattern in args.inputs:
        expanded = sorted(glob.glob(pattern))
        # A pattern that matches nothing is almost always a typo or an unquoted
        # glob the shell already ate. Passing it through as a literal filename
        # would report "file not found" for a name the user never typed.
        if not expanded:
            if any(c in pattern for c in "*?["):
                print(f"warning: pattern matched nothing: {pattern}", file=sys.stderr)
                continue
            expanded = [pattern]
        paths.extend(expanded)

    if not paths:
        print("nothing to convert", file=sys.stderr)
        return 1

    reader: Callable = SOURCES[args.source]
    kwargs = {"dt": args.dt}
    if args.gripper_key:
        kwargs["gripper_key"] = args.gripper_key

    written = 0
    failed = 0
    skipped_episodes = 0

    def note(message: str) -> None:
        nonlocal skipped_episodes
        skipped_episodes += 1
        print(f"skipping {message}", file=sys.stderr)

    kwargs["on_error"] = note

    for path in paths:
        try:
            for rollout in reader(path, **kwargs):
                if args.robot:
                    rollout.meta["robot"] = args.robot
                else:
                    rollout.meta.pop("robot", None)
                validate(rollout)
                out = write_rollout(rollout, args.out)
                print(out)
                written += 1
        except Exception as exc:  # noqa: BLE001 - one bad file must not stop a batch
            # Same rule as the C++ CLI: a corrupt input among forty costs one
            # rollout, not the run.
            print(f"skipping {path}: {exc}", file=sys.stderr)
            failed += 1

    print(
        f"{written} rollouts written, {failed} inputs failed, "
        f"{skipped_episodes} episodes skipped",
        file=sys.stderr,
    )
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
