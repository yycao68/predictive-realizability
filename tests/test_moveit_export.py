import json
from pathlib import Path

import pytest

from realizability.analyzer import load_csv, analyze
from realizability.moveit_export import convert, joint_trajectory_to_rows

ROOT = Path(__file__).resolve().parents[1]


def _traj(points, joint_names=("joint1", "joint2")):
    return {
        "joint_names": list(joint_names),
        "points": points,
    }


def _pt(t_sec, positions):
    return {"positions": positions, "time_from_start": {"sec": int(t_sec), "nanosec": int((t_sec % 1) * 1e9)}}


def test_convert_writes_expected_csv(tmp_path):
    traj = _traj([
        _pt(0.0, [0.0, 0.0]),
        _pt(0.5, [0.1, -0.1]),
        _pt(1.0, [0.2, -0.2]),
    ])
    input_path = tmp_path / "traj.json"
    input_path.write_text(json.dumps(traj))
    output_path = tmp_path / "out.csv"

    joint_names, rows = convert(input_path, output_path)
    assert joint_names == ["joint1", "joint2"]
    assert len(rows) == 3

    t, joints, q = load_csv(output_path)
    assert joints == ["joint1", "joint2"]
    assert list(t) == [0.0, 0.5, 1.0]
    assert q[1].tolist() == [0.1, -0.1]


def test_convert_accepts_robot_trajectory_wrapper(tmp_path):
    # A moveit_msgs/msg/RobotTrajectory (or `ros2 topic echo -f json` on a
    # topic publishing one) nests the JointTrajectory one level down.
    wrapped = {"joint_trajectory": _traj([
        _pt(0.0, [0.0]),
        _pt(0.1, [0.01]),
        _pt(0.2, [0.02]),
    ], joint_names=("joint1",))}
    input_path = tmp_path / "traj.json"
    input_path.write_text(json.dumps(wrapped))
    output_path = tmp_path / "out.csv"

    joint_names, rows = convert(input_path, output_path)
    assert joint_names == ["joint1"]
    assert len(rows) == 3


def test_rejects_fewer_than_three_points():
    traj = _traj([_pt(0.0, [0.0, 0.0]), _pt(0.5, [0.1, -0.1])])
    with pytest.raises(ValueError, match="three"):
        joint_trajectory_to_rows(traj)


def test_rejects_non_increasing_time():
    traj = _traj([
        _pt(0.0, [0.0, 0.0]),
        _pt(0.5, [0.1, -0.1]),
        _pt(0.5, [0.2, -0.2]),  # duplicate timestamp
    ])
    with pytest.raises(ValueError, match="increasing"):
        joint_trajectory_to_rows(traj)


def test_rejects_position_count_mismatch():
    traj = _traj([
        _pt(0.0, [0.0, 0.0]),
        _pt(0.5, [0.1]),  # only one value for two joints
        _pt(1.0, [0.2, -0.2]),
    ])
    with pytest.raises(ValueError, match="positions"):
        joint_trajectory_to_rows(traj)


def test_end_to_end_converted_trajectory_can_be_analyzed(tmp_path):
    # A trajectory whose acceleration clearly exceeds a declared limit,
    # expressed the way a real MoveIt JointTrajectory export would be, run
    # through the full convert -> load_csv -> analyze pipeline.
    points = []
    for i in range(11):
        t = i * 0.1
        points.append(_pt(t, [5.0 * t**2]))
    traj = _traj(points, joint_names=("joint1",))
    input_path = tmp_path / "traj.json"
    input_path.write_text(json.dumps(traj))
    output_path = tmp_path / "out.csv"

    convert(input_path, output_path)
    t, joints, q = load_csv(output_path)
    limits = {"joints": {"joint1": {"max_acceleration": 4.0}}}
    report = analyze(t, joints, q, limits)
    assert report["joints"]["joint1"]["acceleration_violation"]


def test_real_capture_planned_export_converts_and_passes():
    # The raw sparse planned trajectory (10 waypoints) -- see this example's
    # own README for why this one passes despite the live capture below
    # failing on the same underlying plan.
    d = ROOT / "examples/moveit_capture/panda_goal1"
    joint_names, rows = convert(d / "raw_export_planned.json", Path("/tmp/_test_planned.csv"))
    t, joints, q = load_csv("/tmp/_test_planned.csv")
    limits = json.loads((d / "limits.json").read_text())
    report = analyze(t, joints, q, limits)
    assert report["overall"]["realizability_audit_pass"]


def test_real_capture_live_export_converts_and_fails():
    # The live controller reference stream (289 samples) for the identical
    # goal/plan -- reproduces a real acceleration violation on panda_joint7
    # that the sparse planned export above does not show.
    d = ROOT / "examples/moveit_capture/panda_goal1"
    joint_names, rows = convert(d / "raw_export_live.json", Path("/tmp/_test_live.csv"))
    t, joints, q = load_csv("/tmp/_test_live.csv")
    limits = json.loads((d / "limits.json").read_text())
    report = analyze(t, joints, q, limits)
    assert report["joints"]["panda_joint7"]["acceleration_violation"]
    assert not report["overall"]["realizability_audit_pass"]
