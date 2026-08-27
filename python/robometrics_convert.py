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
# LAYOUT. A robosuite-style demonstration file is a single HDF5 with:
#
#     /data                          group, one child per demonstration
#     /data/demo_0                   one episode
#     /data/demo_0/obs/joint_pos     (T, n) float, joint angles per step
#     /data/demo_0/actions           (T, a) float, what the policy commanded
#     /data/demo_0/dones             (T,)   int,   episode-termination flags
#     /data/demo_0/rewards           (T,)   float, per-step reward
#     /data/<demo>.attrs["num_samples"]     step count
#     /data.attrs["env_args"]        JSON string describing the environment
#
# q COMES FROM obs/joint_pos, NOT FROM actions. This is the single most
# important line in this file. `actions` are commands in whatever space the
# controller uses -- for LIBERO's default OSC controller they are end-effector
# deltas, not joint angles at all, and even with a joint-space controller they
# are targets the arm may never have reached. obs/joint_pos is the measured
# state, which is what a kinematic metric has to be computed on. Using actions
# would produce a full set of plausible numbers about a trajectory the robot
# never executed.
#
# The arm joints only. robosuite concatenates gripper joints into some
# observation keys; `joint_pos` is the arm, and `--gripper-key` can bring the
# gripper in when the URDF being analysed includes the fingers. Mixing them
# when the URDF does not expect them produces a width mismatch, which the C++
# side rejects -- loudly, which is the desired outcome.
#
# SUCCESS is derived, not stored. robosuite files carry no success flag; the
# convention across these datasets is that a demonstration reaching a nonzero
# reward, or a `dones` flag of 1 before the recording ends, completed the task.
# In practice LIBERO demonstrations are all successful by construction -- they
# are human teleoperation kept only when the task was solved -- so the honest
# default is 1 with the derivation recorded in the metadata, rather than a
# number invented in silence. `success_source` in the output says which rule
# fired, so a reader can tell a measured success from an assumed one.
#
# TIME. robosuite does not store timestamps. They are synthesised from a fixed
# control period (--dt, default 0.05 s = 20 Hz, LIBERO's default). Since every
# metric in this library is geometric and never differentiates by time, this
# only affects plot axes -- but it is synthesised, and the metadata says so.


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

    stem = os.path.splitext(os.path.basename(path))[0]

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
                if "obs" not in group or "joint_pos" not in group["obs"]:
                    raise ValueError(
                        "no obs/joint_pos. Joint angles are the measured state and "
                        "cannot be substituted with actions; see the module docstring."
                    )

                q = [list(map(float, row)) for row in group["obs"]["joint_pos"][()]]

                if gripper_key:
                    if gripper_key not in group["obs"]:
                        raise ValueError(f"no obs/{gripper_key}")
                    grip = group["obs"][gripper_key][()]
                    if len(grip) != len(q):
                        raise ValueError(
                            f"{gripper_key} has {len(grip)} steps but joint_pos has {len(q)}"
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
    """Returns (success, rule-that-decided), never a bare guess."""
    if "rewards" in group:
        rewards = group["rewards"][()]
        if len(rewards) and float(max(rewards)) > 0.0:
            return 1, "reward>0"
    if "dones" in group:
        dones = group["dones"][()]
        if len(dones) and int(dones[-1]) == 1:
            return 1, "final done flag"
    # No evidence either way. LIBERO demonstrations are successful by
    # construction -- they are teleoperation kept only when the task was solved
    # -- so 1 is the honest default, but the source field says it was assumed
    # rather than measured.
    return 1, "assumed (LIBERO demonstrations are filtered for success)"


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
