# Release checklist

Before publishing the empirical MoveIt benchmark (v0.2, real exported trajectory — held off
for now, see below):

- [ ] Replace demo trajectory with the exact reproducible MoveIt trajectory.
- [ ] Include exact MoveIt/ROS versions and commit/package versions.
- [ ] Include the exact `joint_limits.yaml`.
- [ ] Include the planning request / goal sequence.
- [ ] Include the raw exported trajectory.
- [ ] Record the complete trajectory-processing chain.
- [ ] Record sampling and differentiation method.
- [ ] Save the machine-readable JSON report.
- [ ] Add plots of trajectory values versus declared limits.
- [x] Distinguish commanded/reference acceleration from measured physical acceleration — done in `MOVEIT_ISSUE.md`'s "Important distinction" and the README's CSV-format note; both call this an audit, not a measurement.
- [x] Link the upstream MoveIt issue and related discussions — README and `MOVEIT_ISSUE.md` link #3778, #3779, #3849.
- [ ] Run the benchmark from a clean environment (i.e. against a real MoveIt export — the synthetic scenarios below already run clean, but that's not what this item is asking).
- [ ] Tag the repository with a version.

## Local scenario coverage (done, v0.1 — no MoveIt/Docker required)

`examples/scenarios/01_pass` through `04_position_violation`: one worked example per check
(position, velocity, acceleration) plus a clean baseline, each isolating exactly one violation.
Covered by `tests/test_analyzer.py`'s `test_scenario_*` tests. This is deliberately synthetic
coverage of the *tool*, not the MoveIt case specifically — the items above, about the real
exported trajectory, remain open and gated on v0.2.
