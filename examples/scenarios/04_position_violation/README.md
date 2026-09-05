# Scenario 04 — position violation only

A slower, larger-excursion minimum-jerk profile (distance 1.5 rad over 2.0 s) that reaches
1.5 rad against a declared `max_position` of 1.0 rad, while velocity and acceleration limits
are set generously wide. Isolates the position check (added specifically because the analyzer
did not implement it until a review pass found the gap between README and code).

| check | peak | limit | ratio |
|---|---|---|---|
| position | 1.5 rad | ±1.0 rad | **1.50 — violation** |
| velocity | 1.406 rad/s | 3.0 rad/s | 0.47 |
| acceleration | 2.164 rad/s² | 5.0 rad/s² | 0.43 |

```
python -m realizability.analyzer examples/scenarios/04_position_violation/trajectory.csv \
  --limits examples/scenarios/04_position_violation/limits.json
```
Expected: `Audit: FAIL`, position violation flagged, both ratios stay under 1.0.
