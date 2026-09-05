# Real MoveIt capture — panda_goal1

The first real (non-synthetic) example in this repository: a trajectory actually planned
and executed by MoveIt 2, exported via `realizability.moveit_export` and audited by the
same tool used on the synthetic scenarios in `examples/scenarios/`.

## Environment

- ROS 2 Jazzy (`osrf/ros:jazzy-desktop`), fresh container
- `ros-jazzy-moveit-resources-panda-moveit-config` 3.1.0, unmodified
- `demo.launch.py`, `ros2_control_hardware_type:=mock_components`
- Planner: OMPL RRTConnect, default config, `max_acceleration_scaling_factor=0.941`
- Goal: Cartesian pose `(x, y, z) = (0.2191, -0.0828, 0.7125)` for `panda_link8`, downward-facing
  orientation — goal 1 of [moveit/moveit2#3778](https://github.com/moveit/moveit2/issues/3778)'s
  own "frame 1782355872" reproducer, also used in this project's earlier reproduction
  (see `layer1_moveit_issue/MOVEIT_ISSUE.md`)
- `limits.json` here is the real `joint_limits.yaml` shipped with this config, exactly.

## Two exports of the *same* plan — and why they disagree

**`raw_export_planned.json` / `trajectory_planned.csv`** — the raw `trajectory_msgs/msg/
JointTrajectory` from `arm.plan()`'s result, exported immediately after planning, before
execution. Only **10 waypoints** over 0.826 s (TOTG parameterizes the OMPL path's own sparse
waypoints; it does not resample to a fixed time grid). Audited: **PASS** (peak acceleration
ratio 0.941, on `panda_joint7`).

**`raw_export_live.json` / `trajectory_live.csv`** — the controller's own live `reference`
stream, sampled at its real publish rate during actual execution (**289 samples** over 2.88 s,
`/panda_arm_controller/controller_state`). Audited: **FAIL** — `panda_joint7` reaches ratio
**1.029**, a real violation the sparse planned export does not show at all.

This is not a bug in the analyzer — it is a genuine, reportable methodological finding from
using real data, and it is more specific than "sparse sampling misses peaks in general."
`raw_export_planned.json`'s own embedded `accelerations` field for `panda_joint7` is exactly
bang-bang: **`-4.705` rad/s² for the first 5 waypoints, then `+4.705` for the last 5** — a clean
sign switch at the profile's midpoint (`t=0.4→0.5s`), consistent with TOTG's time-optimal
bang-bang parameterization. Numerically differentiating only the 10 sparse, unevenly-spaced
position waypoints (`np.gradient`, central differences) straddles that discontinuity and
smooths it out — the largest single-point discrepancy between the embedded and the
numerically-estimated acceleration is **4.10 rad/s²** (out of a 4.705 rad/s² signal), even
though the numerical estimate *happens* to reach the same 4.705 peak at the two waypoints
immediately flanking the switch, which is what makes `report_planned.json`'s peak-ratio number
misleadingly close to correct while individual points are not. A raw planned-trajectory export
sampled this coarsely is therefore not reliable for peak-acceleration auditing of a bang-bang-type
profile by numerical differentiation alone; the live, densely-resampled controller reference is
the artifact that actually surfaces the violation here.

| | planned (10 pts) | live reference (289 pts) |
|---|---|---|
| duration | 0.826 s | 2.880 s |
| peak velocity ratio | 0.653 | 0.731 |
| peak acceleration ratio | 0.941 | **1.029 (violation, `panda_joint7`)** |

## Plots

`plot_planned.png` and `plot_live.png` (generated via `analyzer.py --plot`) show this
directly: the planned export's acceleration-ratio panel dips toward zero right at the
`t=0.4s` sign switch instead of showing the true discontinuous jump — a visible symptom of
the under-sampling described above — while the live capture's panel clearly crosses the 1.0
line on `panda_joint7` around `t=0.47s`.

## Reproduce

```bash
python -m realizability.moveit_export examples/moveit_capture/panda_goal1/raw_export_live.json \
  --output /tmp/trajectory_live.csv
python -m realizability.analyzer /tmp/trajectory_live.csv \
  --limits examples/moveit_capture/panda_goal1/limits.json
```

## Sampling and differentiation

Live capture: `/panda_arm_controller/controller_state`'s own publish rate (irregular, not a
fixed dt — `analyzer.py`'s `np.gradient` supports nonuniform sampling directly). Acceleration
throughout is estimated by numerical differentiation of position, per this tool's stated
"audit, not measurement" scope (`docs/methodology.md`) — `reference.accelerations` values in
the raw export are the controller's own commanded reference, not a physical measurement either.
