# MoveIt 2: Planning-to-Execution Realizability Gap in Time-Parameterized Trajectories

## Summary

This report documents a **planning-to-execution realizability question** in MoveIt 2 time-parameterized trajectories.

The key distinction is:

> A trajectory can be accepted by the planning pipeline while the trajectory actually presented to the controller may not satisfy the same kinematic limits used by the planner.

This is not intended to claim that MoveIt 2 or TOTG is generally unsafe. The purpose is to establish a precise, reproducible test for whether the declared trajectory constraints remain satisfied after every trajectory-processing stage.

## Why this is worth tracking

MoveIt documentation states that time parameterization generates trajectories accounting for joint velocity and acceleration limits. The current documentation also notes that TOTG fits path segments and resamples the trajectory, and that controller execution is separate from MoveIt.

That creates a useful engineering question:

**At which stage should constraint compliance be certified, and how should it be re-checked after trajectory transformations?**

This is not only a documentation question — it is checkable directly against the code. In the default `panda_moveit_config` pipeline, `ompl_planning.yaml`'s `response_adapters` run `default_planning_response_adapters/AddTimeOptimalParameterization` (TOTG) followed immediately by `default_planning_response_adapters/ValidateSolution`. Reading `ValidateSolution`'s source (`moveit_ros/planning/planning_response_adapter_plugins/src/validate_path.cpp`) shows it calls `planning_scene->isPathValid()`, which checks collision avoidance, path constraints, and per-waypoint state feasibility — **it never inspects velocity or acceleration**. So on this pipeline, no adapter in the default chain verifies that the trajectory TOTG just produced satisfies the `joint_limits.yaml` bounds it was supposed to enforce. A client that receives `error_code.val == SUCCESS` has no guarantee, from MoveIt itself, that the returned trajectory's commanded accelerations stay within the declared limits.

This is the motivation for a companion independent benchmark tool, published at [yycao68/predictive-realizability](https://github.com/yycao68/predictive-realizability), that evaluates trajectory realizability independent of MoveIt internals — see "Companion benchmark" below.

## Related observations

MoveIt 2 already has public reports concerning acceleration/scaling-limit behavior, including:

- moveit/moveit2#3778: commanded Panda accelerations reported above the declared `max_acceleration`.
- moveit/moveit2#3779: requested acceleration scaling not fully reflected in commanded acceleration.

These reports should be treated as evidence of concrete cases, not as evidence that every MoveIt/TOTG trajectory violates limits. Both are single-reporter, single-fuzzing-run reports with no maintainer engagement yet (zero comments, OPEN) as of this check — an independent reproduction of #3778's core claim is below, so this issue is not solely relying on trusting that report.

### Independent reproduction of #3778 (2026-09-04)

Fresh `osrf/ros:jazzy-desktop` container, `ros-jazzy-moveit-resources-panda-moveit-config` 3.1.0 unmodified, `demo.launch.py` with `mock_components`. Sent #3778's own 5-goal Cartesian sequence ("frame 1782355872") via `moveit_py`, OMPL RRTConnect default config, `max_acceleration_scaling_factor=0.941`, recording peak `|reference.accelerations|` (this controller version's field name; #3778 calls it `desired.accelerations`) from `/panda_arm_controller/controller_state`. Confirmed the declared limits match #3778's table exactly, and confirmed only `AddTimeOptimalParameterization` (TOTG) is configured in this pipeline — no TOPP-RA adapter exists in this package's default pipelines, so "TOTG/TOPP-RA" in #3778/#3779's titles is TOTG on this setup.

Ran 3 times (RRTConnect is randomized — each run takes a different geometric path, not a chosen worst case). Peak commanded-to-limit ratio per joint:

| joint | limit (rad/s²) | run 1 | run 2 | run 3 |
|---|---|---|---|---|
| panda_joint1 | 3.75 | 1.05× | 1.36× | 0.99× |
| panda_joint2 | 1.875 | 1.21× | 1.11× | 1.14× |
| panda_joint3 | 2.5 | 1.02× | 0.98× | 1.09× |
| panda_joint4 | 3.125 | 0.64× | 0.21× | 0.23× |
| panda_joint5 | 3.75 | 1.28× | 0.34× | 0.29× |
| panda_joint6 | 5.0 | 0.52× | 0.21× | 0.51× |
| panda_joint7 | 5.0 | 1.04× | 1.03× | 0.70× |

Every one of the 3 runs violated the declared limit on at least one joint. None reached #3778's reported 2.75× worst case — expected, since these runs reuse the goal poses but not the exact path.

### A versioned real capture, and why the *planned* trajectory alone is not enough (2026-09-04)

The reproduction above sampled peak ratios live; the benchmark repository now also ships the
actual exported trajectory data for one specific plan (goal 1 above), so the claim is
independently checkable rather than resting on a summary table —
[`examples/moveit_capture/panda_goal1/`](https://github.com/yycao68/predictive-realizability/tree/main/examples/moveit_capture/panda_goal1).

Two exports of the *same* plan disagree, and the reason is specific and worth stating
precisely. The raw planned `trajectory_msgs/msg/JointTrajectory` from `arm.plan()`'s result —
only 10 sparse waypoints, since TOTG parameterizes OMPL's own sparse path rather than
resampling to a fixed time grid — audits as **PASS** (peak acceleration ratio 0.941). The
controller's own live `reference` stream for the identical plan, sampled at its real publish
rate during execution (289 samples), audits as **FAIL**: `panda_joint7` reaches ratio
**1.029**.

This is not sampling noise. The planned export's own embedded `accelerations` field for
`panda_joint7` is exactly bang-bang — `-4.705` rad/s² for the first 5 waypoints, `+4.705` for
the last 5, switching sign at the profile's midpoint — consistent with TOTG's time-optimal
parameterization. Differentiating only 10 unevenly-spaced position waypoints straddles that
discontinuity and smooths it out (largest single-point discrepancy: 4.10 rad/s² against the
4.705 rad/s² signal itself), even though the two waypoints immediately flanking the switch
happen to recover the correct peak, which is what makes the planned export's *aggregate*
ratio look misleadingly close to a real audit. **A raw planned-trajectory export is therefore
not on its own sufficient evidence that a trajectory is realizable** — the live, densely
sampled controller reference (or an equivalent fine resampling of the plan) is needed to
surface a violation like this one.

**Update (2026-09-05), addressing questions raised in review of this data:** the live
capture's reported total duration (2.880 s) is noticeably longer than the planned
trajectory's (0.826 s), which could look like the two aren't really the same execution. They
are — the *active motion* in the live capture is 0.830 s (now reported directly as
`active_duration_s` in every audit, alongside `leading_idle_s`/`trailing_idle_s`), matching
the planned duration to within 0.5%. The remaining ~2.05 s is deliberate trailing idle time
the capture script added to catch trailing controller-state samples after the robot reached
its goal; the reported violation at `t≈0.470s` sits well inside the real motion window, not
the idle tail. The benchmark also now supports plotting trajectory values against declared
limits directly (`analyzer.py --plot`) — see the rendered
[live-capture](https://github.com/yycao68/predictive-realizability/blob/main/examples/moveit_capture/panda_goal1/plot_live.png)
and
[planned-trajectory](https://github.com/yycao68/predictive-realizability/blob/main/examples/moveit_capture/panda_goal1/plot_planned.png)
plots for this same data — the latter visibly shows the bang-bang under-sampling described
above as a dip toward zero right at the sign switch, instead of the true discontinuous jump.

## Reproducibility protocol

For a rigorous report, please record:

- ROS 2 distribution
- MoveIt 2 version / commit
- robot model and configuration package
- time-parameterization method
- velocity and acceleration limits
- scaling factors
- planning request
- generated `JointTrajectory`
- controller-side reference trajectory, if available
- exact method used to compute acceleration
- sampling interval
- whether the value is a planner-generated field or numerically differentiated data

### Required test

For each joint \(i\), evaluate

\[
\rho_{a,i} =
\frac{\max_t |\ddot q_i(t)|}{a_{i,\max}}.
\]

Interpretation:

- \(\rho_a < 1\): within the declared acceleration bound
- \(\rho_a \approx 1\): at the boundary
- \(\rho_a > 1\): acceleration-limit violation

The same construction should be used for velocity.

## Important distinction

A controller-state field such as `desired.accelerations` is a **reference command**, not a measurement of the physical acceleration of the robot.

Therefore:

1. A violation of `desired.accelerations` is evidence that the commanded reference trajectory exceeds the declared limit.
2. It is not, by itself, evidence that the physical robot achieved that acceleration.
3. Hardware execution should be evaluated separately when making claims about physical safety.

## Broader research question

The issue motivates a more general concept:

\[
\text{planning feasibility}
\;\not\Rightarrow\;
\text{execution realizability}.
\]

A planner may certify geometry and nominal kinematics, while later transformations, interpolation, controller interfaces, actuator limits, contact conditions, payload, or disturbances can change the physical margin.

The proposed **Predictive Realizability** framework asks whether this loss of physical margin can be predicted *before* the constraint becomes active, and whether the resulting certificate can be fed back to trajectory generation or replanning.

## Scope

This report is deliberately narrower than the Predictive Realizability paper.

It does **not** claim:

- that MoveIt is generally unsafe;
- that TOTG is universally incorrect;
- that a commanded acceleration violation necessarily causes hardware damage;
- that a single issue proves a general failure mode.

The goal is a reproducible benchmark and a clear interface between planning and execution.

## Requested discussion

The most useful feedback from MoveIt maintainers would be:

1. Which trajectory representation should be considered authoritative for checking acceleration limits?
2. At what stage should limits be re-certified after time parameterization, resampling, or smoothing?
3. Is the observed behavior expected for the tested edge case?
4. If expected, what is the recommended way for downstream applications to obtain a formally realizable trajectory?
5. If unexpected, where should the constraint check or correction be implemented?

## Companion benchmark

A companion independent benchmark tool ([yycao68/predictive-realizability](https://github.com/yycao68/predictive-realizability)) makes this kind of experiment reproducible without requiring the Predictive Realizability paper. It separates:

- trajectory generation,
- trajectory transformation,
- constraint evaluation,
- reporting,
- and future predictive certification.

This separation is intentional: the benchmark should remain useful even if a particular MoveIt implementation changes.

The repository currently ships four worked example scenarios (`examples/scenarios/01_pass` through `04_position_violation`), each a synthetic single-joint trajectory tuned so exactly one of position/velocity/acceleration fails while the other two stay within bounds. These can be run directly with no MoveIt or ROS installation:

```bash
python -m realizability.analyzer examples/scenarios/02_acceleration_violation/trajectory.csv \
  --limits examples/scenarios/02_acceleration_violation/limits.json
```

A helper that exports a real MoveIt trajectory into this tool's CSV format (replacing the synthetic examples above with the actual Panda case from this issue) is the next planned step, not yet built.
