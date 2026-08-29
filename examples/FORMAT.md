# robometrics JSON report format

The document written by `robometrics analyze --json report.json ...`. A **public,
versioned output format** — write a reader against this page, not the source. The
CSV (`--out`) is for pipelines; the JSON adds the full per-step profile.

## TL;DR

One object. `rollouts` is the payload; everything else is context.

```jsonc
{
  "format_version": 1,              // bump = breaking change (see Stability)
  "tool": "robometrics",
  "robot":   { "urdf": "panda_arm.urdf", "num_dofs": 7, "characteristic_length": 1.31926 },
  "params":  { "threshold_normalized": 0.0379, "tip": "panda_hand" },
  "summary": { "num_rollouts": 500, "dexterity_median": 0.1066, "rollouts_with_span": 0, /* ... */ },
  "rollouts": [
    {
      "file": "stack_bowls_demo_7",
      "task": "stack_bowls",          // file minus trailing _demo_<N>; group by this
      "steps": 3,
      "success": true,                // from metadata; true | false | null
      "dexterity_min": 0.0542,        // worst normalised dexterity; null if 0 steps
      "dexterity_worst_at": 1,        // 0-based step index of the min
      "efficiency": 0.9971,           // (0,1]; null unless efficiency_status == "available"
      "efficiency_status": "available",
      "spans": [ {"begin": 1, "end": 2, "worst": 0.0542} ],  // HALF-OPEN [begin, end)
      "profile": [0.1805, 0.0542, 0.2003]  // per-step; null (--profiles none) or [] (0 steps)
    }
  ]
}
```

- **`dexterity_*` values are dimensionless** — `sigma_min` divided by
  `robot.characteristic_length`, so they compare across robot sizes.
- **A step is low-dexterity when its dexterity `< params.threshold_normalized`.**
  `spans` are the contiguous runs where that holds; `dexterity_min` is the
  trajectory's worst.
- **Numbers are 6 significant figures** — identical to the same value in the CSV.
  Encoding is UTF-8; a non-finite value would be `null` (valid data never
  produces one).
- **All fields are always present.** Absence/N/A is `null`, never a missing key.

## Gotchas

- **Spans are half-open.** `{"begin": 338, "end": 362}` means the last *bad* step
  is **361**, not 362 — the span covers `[begin, end)`. The stderr summary prints
  the same span *inclusive* as `[338..361]`; the JSON keeps the half-open pair.
  Iterate `range(begin, end)`.
- **`null` is never `0.0`.** `null` means no value (empty trajectory, metric not
  applicable, unknown metadata). `0.0` is a real measurement. Do not conflate.
- **`efficiency` is `null` unless `efficiency_status == "available"`.** The status
  is a **robot-level** property, the same for every rollout in a run: if it is
  `"not_redundant"` or `"mixed_joint_types"`, *every* rollout's `efficiency` is
  `null`. Even when `"available"`, a too-short or motionless rollout is `null`.

## Fields

Lengths are metres; efficiency is a ratio in `(0, 1]`; step indices are 0-based.
"Null?" is whether the value can be `null`.

### `robot`

| field                  | type    | null? | meaning                                                    |
|------------------------|---------|-------|------------------------------------------------------------|
| `urdf`                 | string  | no    | URDF path as given on the command line.                    |
| `num_dofs`             | integer | no    | Degrees of freedom (length of each configuration `q`).     |
| `characteristic_length`| number  | no    | Length `L` dexterity is normalised by (metres).            |

### `params`

| field                  | type    | null? | meaning                                                    |
|------------------------|---------|-------|------------------------------------------------------------|
| `threshold_normalized` | number  | no    | Dimensionless low-dexterity threshold applied (`--threshold` ÷ `L`). |
| `tip`                  | string  | yes   | End-effector link (`--tip`); `null` when auto-detected.    |

### `summary`

Same numbers as the stderr summary; dexterity/efficiency aggregates are `null`
when there were no rollouts to aggregate. Percentiles are nearest-rank.

| field                  | type    | null? | meaning                                                    |
|------------------------|---------|-------|------------------------------------------------------------|
| `num_rollouts`         | integer | no    | Rollouts analysed (length of `rollouts`).                  |
| `num_failed`           | integer | no    | Input files skipped as unparseable (not in `rollouts`).    |
| `dexterity_median`     | number  | yes   | Median of per-rollout `dexterity_min`.                     |
| `dexterity_p05`        | number  | yes   | 5th-percentile `dexterity_min`.                            |
| `dexterity_min`        | number  | yes   | Smallest `dexterity_min` across rollouts.                  |
| `efficiency_median`    | number  | yes   | Median `efficiency` over rollouts that produced one.       |
| `efficiency_available` | integer | no    | How many rollouts produced an `efficiency` value.          |
| `efficiency_na_reason` | string  | yes   | Run-wide reason efficiency is N/A; `null` when available. Equals the `efficiency_status` shared by every rollout when it is not `available`. |
| `rollouts_with_span`   | integer | no    | Rollouts with at least one span.                           |

### `rollouts[]`

| field                | type    | null? | meaning                                                      |
|----------------------|---------|-------|--------------------------------------------------------------|
| `file`               | string  | no    | Input file stem (no directory, no extension).                |
| `task`               | string  | no    | `file` with a trailing `_demo_<N>` removed (e.g. `pick_place_demo_7` → `pick_place`); `file` if none. |
| `steps`              | integer | no    | Recorded steps (equals `profile` length when embedded).      |
| `success`            | bool    | yes   | From `success` metadata; `null` if absent or non-boolean.    |
| `dexterity_min`      | number  | yes   | Worst (smallest) normalised dexterity; the min of `profile`. |
| `dexterity_worst_at` | integer | yes   | 0-based step index of `dexterity_min`.                       |
| `efficiency`         | number  | yes   | Path efficiency; `null` when N/A (see Gotchas).              |
| `efficiency_status`  | string  | no    | `available` \| `not_redundant` \| `mixed_joint_types`.       |
| `spans`              | array   | no    | Low-dexterity runs (see below); `[]` when none.              |
| `profile`            | array   | yes   | Per-step normalised dexterity; `null` (`--profiles none`) or `[]` (0 steps). |

### `spans[]`

| field   | type    | null? | meaning                                                       |
|---------|---------|-------|---------------------------------------------------------------|
| `begin` | integer | no    | First step of the span (inclusive, 0-based).                  |
| `end`   | integer | no    | One past the last step (**half-open**: covers `[begin, end)`).|
| `worst` | number  | no    | Smallest normalised dexterity in the span (`< threshold_normalized`). |

To keep profiles out of the JSON but still have them, use `--profiles none` with
`--profile-out DIR/`: one `step,t,dexterity` CSV per rollout, same values. For
what `sigma_min` measures and why it is normalised, see the README and the source
comments — the algorithm is out of scope here.

## Reading it

```python
import json

with open("report.json") as f:
    report = json.load(f)

threshold = report["params"]["threshold_normalized"]

for rollout in report["rollouts"]:
    # dexterity_min is null for a zero-step rollout -- guard before comparing.
    if rollout["dexterity_min"] is not None and rollout["dexterity_min"] < threshold:
        print(f'{rollout["file"]}: dips to {rollout["dexterity_min"]}')

    for span in rollout["spans"]:
        steps = range(span["begin"], span["end"])   # half-open: excludes `end`
        last_bad = span["end"] - 1
        print(f'  low steps {span["begin"]}..{last_bad} '
              f'({len(steps)} steps), worst {span["worst"]}')

    # efficiency is only meaningful when the status says so.
    if rollout["efficiency_status"] == "available" and rollout["efficiency"] is not None:
        print(f'  efficiency {rollout["efficiency"]}')
```

## Stability

`format_version` is the contract. Within a major version: fields may be **added**
(ignore ones you do not know), and the **meaning, type, and unit of an existing
field will not change** nor will a field be removed. Enumerated strings (e.g.
`efficiency_status`) may gain values — treat an unrecognised one as "a reason I
do not know" rather than failing. A change that breaks those promises increments
`format_version`.

## Minimal valid document

The smallest well-formed report — a robot, no rollouts analysed:

```json
{
  "format_version": 1,
  "tool": "robometrics",
  "robot": {
    "urdf": "arm.urdf",
    "num_dofs": 2,
    "characteristic_length": 0.6
  },
  "params": {
    "threshold_normalized": 0.0833333,
    "tip": null
  },
  "summary": {
    "num_rollouts": 0,
    "num_failed": 0,
    "dexterity_median": null,
    "dexterity_p05": null,
    "dexterity_min": null,
    "efficiency_median": null,
    "efficiency_available": 0,
    "efficiency_na_reason": null,
    "rollouts_with_span": 0
  },
  "rollouts": []
}
```

A single populated rollout looks like:

```json
{
  "file": "stack_bowls_demo_7",
  "task": "stack_bowls",
  "steps": 3,
  "success": true,
  "dexterity_min": 0.0542,
  "dexterity_worst_at": 1,
  "efficiency": 0.9971,
  "efficiency_status": "available",
  "spans": [
    {"begin": 1, "end": 2, "worst": 0.0542}
  ],
  "profile": [0.1805, 0.0542, 0.2003]
}
```
