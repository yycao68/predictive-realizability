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
      "max_velocity": 2.0,
      "max_acceleration": 4.0
    }
  }
}
```

Velocity or acceleration may be omitted when not being audited.

## Why positions are the input

The repository is designed to audit trajectories independently of a particular ROS message type.

A future MoveIt adapter can export `trajectory_msgs/JointTrajectory`,
`moveit_msgs/RobotTrajectory`, or controller reference trajectories to this
common representation. This keeps the analysis layer independent of MoveIt
implementation details.
