# Predictive Realizability Benchmark

A small, implementation-independent benchmark for testing whether a planned or time-parameterized joint trajectory remains within its declared physical limits.

## Motivation

Motion planning commonly follows:

```text
geometric path
      ↓
time parameterization
      ↓
trajectory
      ↓
controller
      ↓
robot
```

The benchmark asks a simple but important question:

> **Does the trajectory remain realizable after each transformation in the pipeline?**

This repository is intentionally independent of MoveIt internals. A MoveIt trajectory can be exported to CSV and analyzed here.

## Current scope

The first release evaluates:

- joint position limits (optional)
- joint velocity limits
- joint acceleration limits
- peak-to-limit ratios
- first violation time
- per-joint margins
- machine-readable JSON output

The benchmark does **not** claim to estimate physical robot acceleration. It evaluates the trajectory representation supplied to the analyzer.

## Repository structure

```text
predictive-realizability/
├── README.md
├── requirements.txt
├── pyproject.toml
├── layer1_moveit_issue/
│   └── MOVEIT_ISSUE.md
├── src/realizability/
│   ├── __init__.py
│   └── analyzer.py
├── examples/
│   ├── limits.json
│   ├── demo_trajectory.csv
│   └── scenarios/            # one worked example per check, each isolating a single violation
│       ├── 01_pass/
│       ├── 02_acceleration_violation/
│       ├── 03_velocity_violation/
│       └── 04_position_violation/
├── tests/
│   └── test_analyzer.py
└── docs/
    ├── data_format.md
    ├── methodology.md
    └── release_checklist.md
```

## Worked examples

`examples/scenarios/` has four synthetic single-joint trajectories, each with its own
`trajectory.csv`, `limits.json`, and a short `README.md` explaining the construction and
expected result:

| scenario | isolates | audit |
|---|---|---|
| `01_pass` | nothing (baseline) | PASS |
| `02_acceleration_violation` | acceleration only | FAIL |
| `03_velocity_violation` | velocity only | FAIL |
| `04_position_violation` | position only | FAIL |

Each is deliberately synthetic (not a real MoveIt export — see the roadmap's v0.2) and each
limits.json is tuned so only the named check can fail; the other two stay comfortably within
bounds. Run any of them directly:

```bash
python -m realizability.analyzer examples/scenarios/02_acceleration_violation/trajectory.csv \
  --limits examples/scenarios/02_acceleration_violation/limits.json
```

## Quick start

```bash
python -m pip install -e .
python -m realizability.analyzer \
  examples/demo_trajectory.csv \
  --limits examples/limits.json \
  --output report.json
```

The command prints a compact summary and writes a JSON report.

## CSV format

The first column is time in seconds. Remaining columns are joint positions in radians:

```csv
time,panda_joint1,panda_joint2
0.00,0.0,0.0
0.01,0.001,0.000
0.02,0.004,0.001
```

Acceleration is estimated by numerical differentiation of the supplied position trajectory. This is intentionally labeled as an **offline trajectory audit**, not a measurement.

See `docs/data_format.md`.

## Metrics

For joint \(i\):

\[
\rho_{v,i} =
\frac{\max_t |\dot q_i(t)|}{v_{i,\max}},
\qquad
\rho_{a,i} =
\frac{\max_t |\ddot q_i(t)|}{a_{i,\max}}.
\]

The minimum normalized margin is

\[
m_i = 1-\rho_i.
\]

A negative margin indicates a limit violation.

## MoveIt connection

The benchmark was motivated by concrete MoveIt 2 discussions, including acceleration and acceleration-scaling reports ([moveit/moveit2#3778](https://github.com/moveit/moveit2/issues/3778), [#3779](https://github.com/moveit/moveit2/issues/3779)) and is referenced from [moveit/moveit2#3849](https://github.com/moveit/moveit2/issues/3849). See `layer1_moveit_issue/MOVEIT_ISSUE.md`.

The benchmark intentionally avoids saying "MoveIt is broken." Instead, it asks a more general and testable question:

```text
planning feasibility
        ↓
trajectory transformation
        ↓
realizability audit
```

This is the empirical foundation for the broader Predictive Realizability framework.

## Reproducibility policy

Every published benchmark result should record:

- robot and joint model
- ROS / MoveIt version
- planner
- time parameterizer
- limits
- scaling factors
- trajectory file
- sampling rate
- analysis command
- software commit

## Roadmap

### v0.1
Offline trajectory auditing.

### v0.2
MoveIt 2 export helper.

### v0.3
Trajectory-stage comparison:

```text
OMPL → TOTG → Ruckig → controller reference
```

### v0.4
Predictive margin:

\[
\hat m_{\mathrm{phys}}(t+H)
\]

with an explicit warning horizon.

### v0.5
Planner feedback / trajectory regeneration experiments.

## Citation

If you use this benchmark in research, cite the accompanying Predictive Realizability paper when available.
