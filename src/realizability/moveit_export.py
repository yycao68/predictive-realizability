"""Convert a MoveIt/ROS 2 trajectory_msgs/msg/JointTrajectory export into this
tool's CSV format (time,<joint1>,<joint2>,...), so a real MoveIt trajectory can
be audited without this repository depending on rclpy or any ROS 2 install.

Input is the message decoded from JSON, e.g. via:

    ros2 topic echo /panda_arm_controller/joint_trajectory --once -f json > traj.json

or by serializing a `moveit_msgs/msg/RobotTrajectory`'s `.joint_trajectory`
field the same way. Only `joint_names` and each point's `positions` and
`time_from_start` are used -- velocities/accelerations, if present, are
ignored, since this tool derives them itself (see docs/methodology.md
"Audit, not measurement").
"""
import argparse
import json
from pathlib import Path


def _time_from_start_seconds(point):
    tfs = point.get("time_from_start")
    if tfs is None:
        raise ValueError("Trajectory point missing 'time_from_start'")
    sec = tfs.get("sec", tfs.get("secs", 0))
    nanosec = tfs.get("nanosec", tfs.get("nsecs", 0))
    return float(sec) + float(nanosec) * 1e-9


def joint_trajectory_to_rows(traj):
    """Convert a decoded trajectory_msgs/msg/JointTrajectory dict into
    (joint_names, [(t, [positions...]), ...]), validated the same way
    analyzer.load_csv() validates a CSV: at least three samples, strictly
    increasing time.
    """
    joint_names = traj.get("joint_names")
    if not joint_names:
        raise ValueError("Trajectory has no 'joint_names'")

    points = traj.get("points") or []
    if len(points) < 3:
        raise ValueError("At least three trajectory points are required")

    rows = []
    for i, point in enumerate(points):
        positions = point.get("positions")
        if positions is None or len(positions) != len(joint_names):
            raise ValueError(
                f"Point {i}: expected {len(joint_names)} positions "
                f"(one per joint_names), got {len(positions) if positions is not None else 'none'}"
            )
        rows.append((_time_from_start_seconds(point), list(positions)))

    times = [t for t, _ in rows]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError(
            "time_from_start must be strictly increasing across points -- "
            "this converter does not deduplicate or reorder samples"
        )

    return joint_names, rows


def write_csv(joint_names, rows, output_path):
    with open(output_path, "w", newline="") as f:
        f.write("time," + ",".join(joint_names) + "\n")
        for t, positions in rows:
            f.write(f"{t:.6f}," + ",".join(f"{p:.9f}" for p in positions) + "\n")


def convert(input_path, output_path):
    traj = json.loads(Path(input_path).read_text())
    # Accept either a bare JointTrajectory, or a RobotTrajectory / topic-echo
    # wrapper that nests it one level under "joint_trajectory".
    if "joint_names" not in traj and "joint_trajectory" in traj:
        traj = traj["joint_trajectory"]
    joint_names, rows = joint_trajectory_to_rows(traj)
    write_csv(joint_names, rows, output_path)
    return joint_names, rows


def main():
    p = argparse.ArgumentParser(
        description="Convert a JSON-exported trajectory_msgs/msg/JointTrajectory "
        "into this tool's CSV format."
    )
    p.add_argument("input", help="Path to the JSON-decoded JointTrajectory (or RobotTrajectory) message")
    p.add_argument("--output", required=True, help="Path to write the CSV to")
    args = p.parse_args()

    joint_names, rows = convert(args.input, args.output)
    print(f"Wrote {args.output}: {len(rows)} samples, {len(joint_names)} joints ({', '.join(joint_names)})")


if __name__ == "__main__":
    main()
