# Scenario 01 — pass

A minimum-jerk single-joint motion (`joint1`, distance 0.433 rad over 1.0 s) that stays inside
all three declared bounds. This is the same profile shipped as `examples/demo_trajectory.csv`.

| check | peak | limit | ratio |
|---|---|---|---|
| position | 0.433 rad | ±1.0 rad | 0.43 |
| velocity | 0.812 rad/s | 2.0 rad/s | 0.41 |
| acceleration | 2.497 rad/s² | 4.0 rad/s² | 0.62 |

```
python -m realizability.analyzer examples/scenarios/01_pass/trajectory.csv \
  --limits examples/scenarios/01_pass/limits.json
```
Expected: `Audit: PASS`.
