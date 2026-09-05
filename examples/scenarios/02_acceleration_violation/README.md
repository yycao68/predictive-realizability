# Scenario 02 — acceleration violation only

`q(t) = (A/ω)·t − (A/ω²)·sin(ωt)` with `A = 6.0 rad/s²`, `ω = 2π·2 rad/s` — a sinusoidal acceleration profile at high enough frequency that velocity and position stay small while acceleration alone breaches its limit. Isolates the acceleration check: position and velocity limits are set generously wide so only acceleration can fail.

| check | peak | limit | ratio |
|---|---|---|---|
| position | 0.478 rad | ±2.0 rad | 0.24 |
| velocity | 0.954 rad/s | 3.0 rad/s | 0.32 |
| acceleration | 5.957 rad/s² | 4.0 rad/s² | **1.49 — violation** |

```
python -m realizability.analyzer examples/scenarios/02_acceleration_violation/trajectory.csv \
  --limits examples/scenarios/02_acceleration_violation/limits.json
```
Expected: `Audit: FAIL`, acceleration ratio > 1.0 only.
