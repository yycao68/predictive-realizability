# Predictive Realizability Benchmark

A small, implementation-independent benchmark for testing whether a planned or time-parameterized joint trajectory remains within its declared kinematic limits (position, velocity, acceleration) — not a torque, actuator, contact, or thermal model.

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
│   ├── analyzer.py
│   ├── moveit_export.py      # v0.2: JointTrajectory JSON -> this tool's CSV format
│   ├── compare_stages.py     # v0.3: audit the same motion across pipeline stages
│   ├── lookahead_margin.py   # v0.4: receding-horizon warning time / lead time
│   ├── retime.py             # v0.5: uniform retiming, restores velocity/acceleration only
│   ├── plotting.py           # --plot: values vs. limits, matplotlib optional-only
│   └── representation_sensitivity.py  # is a verdict a sampling-density artifact?
├── examples/
│   ├── limits.json
│   ├── demo_trajectory.csv
│   ├── scenarios/             # one worked example per check, each isolating a single violation
│   │   ├── 01_pass/
│   │   ├── 02_acceleration_violation/
│   │   ├── 03_velocity_violation/
│   │   └── 04_position_violation/
│   └── moveit_capture/        # a real (non-synthetic) MoveIt 2 capture, see its own README
│       └── panda_goal1/
├── tests/
│   ├── test_analyzer.py
│   ├── test_moveit_export.py
│   ├── test_compare_stages.py
│   ├── test_lookahead_margin.py
│   ├── test_retime.py
│   ├── test_plotting.py
│   └── test_representation_sensitivity.py
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

Each is deliberately synthetic (not a real MoveIt export — see the roadmap's v0.2) and each limits.json is tuned so only the named check can fail; the other two stay comfortably within bounds. Run any of them directly:

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

The command prints a compact summary and writes a JSON report. Add `--plot report.png` for a
plot of trajectory values against declared limits (needs `pip install -e ".[plot]"` for
matplotlib — kept optional so the core audit path has no plotting dependency):

```bash
python -m pip install -e ".[plot]"
python -m realizability.analyzer examples/moveit_capture/panda_goal1/trajectory_live.csv \
  --limits examples/moveit_capture/panda_goal1/limits.json --plot report.png
```

Velocity and acceleration are plotted as a ratio (any number of joints, one shared 1.0 line);
position — which has no single natural ratio for an asymmetric `[min, max]` bound — is plotted
as the raw value with each joint's own limit lines.

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

## Is the planned-vs-live finding just a sampling-density artifact?

`examples/moveit_capture/panda_goal1`'s planned export (10 waypoints) audits PASS; the live
controller reference (289 samples, same plan) audits FAIL. The obvious objection: is that
just an artifact of comparing two arbitrarily different sample counts, rather than a real
representation-dependent finding? Measured directly, not argued: `representation_sensitivity.py`
takes the live capture alone and uniformly subsamples it at a swept range of densities, so the
underlying motion is held fixed and only the sampling density changes.

```bash
python -m realizability.representation_sensitivity \
  examples/moveit_capture/panda_goal1/trajectory_live.csv \
  --limits examples/moveit_capture/panda_goal1/limits.json
```
```text
Active-motion window: 84 samples, 0.830s
N (target)  N (actual)   peak velocity   peak acceleration     audit
        10          10          0.648x              0.941x      PASS
        20          20          0.694x              0.949x      PASS
        50          50          0.730x              1.017x      FAIL
        84          84          0.731x              1.029x      FAIL

Verdict changes between: [(20, 50)]
```

The verdict crosses PASS→FAIL between N=20 and N=50, converging monotonically toward the full
84-sample value as density increases — a real threshold, not noise. (Not a claim that uniform
subsampling reproduces TOTG's own bang-bang-optimal waypoint placement at a given count — it
answers the narrower question of how detection degrades under this one simple, common
sparsification strategy, holding the true motion fixed.)

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

### v0.2 — done
MoveIt 2 export helper (`realizability.moveit_export`): converts a JSON-decoded
`trajectory_msgs/msg/JointTrajectory` (or a `RobotTrajectory` wrapping one) into this tool's
CSV format, no ROS install required for the conversion itself. Validated against a real
capture, not just synthetic fixtures — see `examples/moveit_capture/panda_goal1/`, which also
documents a real methodological finding: a raw *planned* trajectory's sparse waypoints can
under-represent a bang-bang-type acceleration profile under numerical differentiation, while
the live controller reference stream (densely sampled) reproduces the real violation.

```bash
python -m realizability.moveit_export examples/moveit_capture/panda_goal1/raw_export_live.json \
  --output /tmp/trajectory.csv
python -m realizability.analyzer /tmp/trajectory.csv \
  --limits examples/moveit_capture/panda_goal1/limits.json
```

### v0.3 — done
Trajectory-stage comparison (`realizability.compare_stages`): audits the same underlying
motion across any number of named pipeline stages and reports the first stage where the
realizability audit transitions from PASS to FAIL — not tied to any specific pipeline
(OMPL/TOTG/Ruckig/controller are example stage names, not requirements). Applied to
`examples/moveit_capture/panda_goal1/`'s own planned-vs-live data:

```bash
python -m realizability.compare_stages \
  planned=examples/moveit_capture/panda_goal1/trajectory_planned.csv \
  live=examples/moveit_capture/panda_goal1/trajectory_live.csv \
  --limits examples/moveit_capture/panda_goal1/limits.json
```
```text
stage                  peak velocity   peak acceleration     audit
planned                       0.653x              0.941x      PASS
live                          0.731x              1.029x      FAIL

Realizability lost at stage 'live' (the stage before it still passed).
```

### v0.4 — done
Look-ahead margin (`realizability.lookahead_margin`) — deliberately not named "predictive
margin": this tool has no dynamics model or torque, so it cannot reproduce the Predictive
Realizability paper's own $m_{\mathrm{phys}}$ certificate, and that name is kept free for
that future implementation rather than implied by this one. What it *can* compute from a
trajectory alone: since the full CSV represents an already-known plan, a receding-horizon
scan over it — at each sample, look ahead over `[t, t+H]` using data already in the plan —
is a legitimate look-ahead check. It reports the first warning time, the actual (unwindowed)
violation time, and the resulting lead time.

```bash
python -m realizability.lookahead_margin examples/moveit_capture/panda_goal1/trajectory_live.csv \
  --limits examples/moveit_capture/panda_goal1/limits.json --horizon 0.2
```
```text
Horizon: 0.200 s over 289 samples
First warning:    t=0.270s
Actual violation: t=0.470s
Lead time:        0.200s (predicted before it happened)
```

On this real trajectory the lead time comes out equal to the horizon itself across a
0.05–0.3s sweep — expected here because the acceleration approaches the limit
monotonically in this window, not a general guarantee for every signal shape.

### v0.5 — done
Planner feedback / trajectory regeneration (`realizability.retime`), scoped to the one
mechanism this tool can implement without a dynamics model: uniform retiming. Slowing a
trajectory down by a factor $\lambda$ (geometric path unchanged) scales velocity by
$1/\lambda$ and acceleration by $1/\lambda^2$ — both purely kinematic. This is the same
mechanism the Predictive Realizability paper calls Level 1, and it inherits the same real
limitation: retiming cannot fix a **position** violation, since $q(t)$ itself never changes
under pure time-dilation.

```bash
python -m realizability.retime examples/moveit_capture/panda_goal1/trajectory_live.csv \
  --limits examples/moveit_capture/panda_goal1/limits.json
```
```text
Retime factor: 1.0146x (binding: panda_joint7 acceleration)
Before: velocity 0.731x, acceleration 1.029x, position_violation=False
After:  velocity 0.721x, acceleration 1.000x, position_violation=False
Fully restored: True
```

On the real goal1 acceleration violation, a **1.46% slowdown** is enough to fully restore it.
On a position violation (`examples/scenarios/04_position_violation`), retiming correctly
reports nothing it can do — `position_violation_remains: true`, `fully_restored: false` —
rather than silently claiming success.

### v0.6 — done
Representation-sensitivity sweep (`realizability.representation_sensitivity`) — see
"Is the planned-vs-live finding just a sampling-density artifact?" above. Turns the
planned-vs-live anecdote into a measured sweep over one held-fixed trajectory, showing a
real PASS→FAIL threshold between N=20 and N=50 samples, not an artifact of comparing two
arbitrarily different captures.

## Citation

If you use this benchmark in research, cite the accompanying Predictive Realizability paper when available.
