# Release checklist

Before publishing the empirical MoveIt benchmark:

- [ ] Replace demo trajectory with the exact reproducible MoveIt trajectory.
- [ ] Include exact MoveIt/ROS versions and commit/package versions.
- [ ] Include the exact `joint_limits.yaml`.
- [ ] Include the planning request / goal sequence.
- [ ] Include the raw exported trajectory.
- [ ] Record the complete trajectory-processing chain.
- [ ] Record sampling and differentiation method.
- [ ] Save the machine-readable JSON report.
- [ ] Add plots of trajectory values versus declared limits.
- [ ] Distinguish commanded/reference acceleration from measured physical acceleration.
- [ ] Link the upstream MoveIt issue and related discussions.
- [ ] Run the benchmark from a clean environment.
- [ ] Tag the repository with a version.
