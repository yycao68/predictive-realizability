# Data format

## Trajectory CSV

Required columns:

```text
time,<joint_1>,<joint_2>,...
```

- `time`: seconds
- revolute joint positions: radians
- prismatic joint positions: meters

Time must be strictly increasing.

## Limits JSON

```json
{
  "joints": {
    "joint1": {
      "min_position": -2.9,
      "max_position": 2.9,
      "max_velocity": 2.0,
      "max_acceleration": 4.0
    }
  }
}
```

Any of `min_position`/`max_position`/`max_velocity`/`max_acceleration` may be omitted when not being audited. `max_velocity`/`max_acceleration` are symmetric bounds on `|value|`; `min_position`/`max_position` are independent (most joint ranges are not symmetric around zero) and either may be given alone.

A limit of `0.0` for `max_velocity`/`max_acceleration` is treated as "must stay at zero" (e.g. a locked joint) — any nonzero value is reported as a violation with `ratio = inf`, rather than raising a division error.

## Why positions are the input

The repository is designed to audit trajectories independently of a particular ROS message type.

A future MoveIt adapter can export `trajectory_msgs/JointTrajectory`,
`moveit_msgs/RobotTrajectory`, or controller reference trajectories to this
common representation. This keeps the analysis layer independent of MoveIt
implementation details.
