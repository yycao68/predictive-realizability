# Release checklist

Before publishing the empirical MoveIt benchmark:

- [x] Replace demo trajectory with the exact reproducible MoveIt trajectory — `examples/moveit_capture/panda_goal1/` added alongside the synthetic demo (not a replacement of it — both are kept, for different purposes).
- [x] Include exact MoveIt/ROS versions and commit/package versions — ROS 2 Jazzy, `panda_moveit_config` 3.1.0, recorded in that example's own README.
- [x] Include the exact `joint_limits.yaml` — `examples/moveit_capture/panda_goal1/limits.json`.
- [x] Include the planning request / goal sequence — the specific Cartesian goal and scaling factor, recorded in that example's README.
- [x] Include the raw exported trajectory — both `raw_export_planned.json` and `raw_export_live.json` (see below for why there are two).
- [x] Record the complete trajectory-processing chain — OMPL RRTConnect → TOTG → live controller reference, documented explicitly including where the two exports diverge.
- [x] Record sampling and differentiation method — controller publish rate (irregular; `np.gradient` supports nonuniform sampling), documented in the example's README.
- [x] Save the machine-readable JSON report — `report_planned.json` / `report_live.json` shipped alongside.
- [ ] Add plots of trajectory values versus declared limits.
- [x] Distinguish commanded/reference acceleration from measured physical acceleration — done in `MOVEIT_ISSUE.md`'s "Important distinction" and the README's CSV-format note; both call this an audit, not a measurement.
- [x] Link the upstream MoveIt issue and related discussions — README and `MOVEIT_ISSUE.md` link #3778, #3779, #3849.
- [x] Run the benchmark from a clean environment — fresh Docker container, freshly installed packages, confirmed working end to end (`moveit_export` → `analyzer`) on the real capture.
- [ ] Tag the repository with a version.

## A real finding from doing this (not anticipated when this checklist was written)

The raw *planned* trajectory (`raw_export_planned.json`, 10 sparse waypoints) audits as PASS;
the live controller reference for the identical plan (`raw_export_live.json`, 289 samples)
audits as FAIL on `panda_joint7`. This isn't a bug — TOTG's own embedded acceleration for that
joint is exactly bang-bang (±4.705 rad/s², switching sign at the profile's midpoint), and
numerically differentiating only 10 unevenly-spaced waypoints smooths across that
discontinuity. See `examples/moveit_capture/panda_goal1/README.md` for the full analysis. This
means a "raw exported trajectory" is not on its own sufficient for peak-acceleration auditing
of a bang-bang-type profile — worth remembering before checking off similar items in the future
for a different trajectory.

## Local scenario coverage (done, v0.1 — no MoveIt/Docker required)

`examples/scenarios/01_pass` through `04_position_violation`: one worked example per check
(position, velocity, acceleration) plus a clean baseline, each isolating exactly one violation.
Covered by `tests/test_analyzer.py`'s `test_scenario_*` tests. This is deliberately synthetic
coverage of the *tool*, not the MoveIt case specifically — the real MoveIt capture above
(`examples/moveit_capture/panda_goal1/`) is the complementary, non-synthetic evidence.
