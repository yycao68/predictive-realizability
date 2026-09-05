# Scenario 03 — velocity violation only

A minimum-jerk profile (distance 1.0 rad over 1.0 s) audited against a velocity limit
(1.5 rad/s) tighter than its natural peak velocity (1.875 rad/s), while position and
acceleration limits are set generously wide. Isolates the velocity check.

| check | peak | limit | ratio |
|---|---|---|---|
| position | 1.0 rad | ±2.0 rad | 0.50 |
| velocity | 1.875 rad/s | 1.5 rad/s | **1.25 — violation** |
| acceleration | 5.766 rad/s² | 10.0 rad/s² | 0.58 |

```
python -m realizability.analyzer examples/scenarios/03_velocity_violation/trajectory.csv \
  --limits examples/scenarios/03_velocity_violation/limits.json
```
Expected: `Audit: FAIL`, velocity ratio > 1.0 only.
