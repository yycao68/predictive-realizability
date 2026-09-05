import json
import math
from pathlib import Path

import numpy as np

from realizability.analyzer import load_csv
from realizability.retime import regenerate, required_retime_factor

ROOT = Path(__file__).resolve().parents[1]


def test_retiming_restores_an_acceleration_violation():
    # a(t) = 10 (constant, from q=5t^2), ratio 2.5x against a 4.0 limit.
    # Acceleration scales as 1/lambda^2, so the exact factor needed is
    # sqrt(2.5) -- verify both the closed form and that re-auditing the
    # actually-retimed trajectory confirms it, not just the formula.
    t = np.linspace(0, 1, 101)
    q = np.column_stack([5.0 * t**2])
    limits = {"joints": {"joint1": {"max_acceleration": 4.0}}}
    result = regenerate(t, ["joint1"], q, limits)
    assert abs(result["retime_factor"] - math.sqrt(2.5)) < 1e-6
    assert result["before"]["max_acceleration_ratio"] > 1.0
    assert result["after"]["max_acceleration_ratio"] <= 1.0
    assert result["fully_restored"]


def test_retiming_does_not_touch_position_violations():
    d = ROOT / "examples/scenarios/04_position_violation"
    t, joints, q = load_csv(d / "trajectory.csv")
    limits = json.loads((d / "limits.json").read_text())
    result = regenerate(t, joints, q, limits)
    assert result["retime_factor"] == 1.0  # velocity/acceleration were never binding here
    assert result["position_violation_remains"]
    assert not result["fully_restored"]
    # position itself is byte-for-byte the same trajectory, only time changed
    assert result["before"]["position_violation"] == result["after"]["position_violation"]


def test_zero_limit_joint_is_unresolvable_not_silently_infinite():
    t = np.linspace(0, 1, 11)
    q = np.column_stack([t, np.zeros_like(t)])  # joint1 moves, joint2 doesn't
    joints = ["joint1", "joint2"]
    limits = {
        "joints": {
            "joint1": {"max_velocity": 0.0},  # locked joint that nonetheless moves
            "joint2": {"max_velocity": 1.0},
        }
    }
    lam, binding, unresolvable = required_retime_factor(t, joints, q, limits)
    assert math.isfinite(lam)
    assert {"joint": "joint1", "signal": "velocity"} in unresolvable


def test_already_passing_trajectory_needs_no_retiming():
    d = ROOT / "examples/scenarios/01_pass"
    t, joints, q = load_csv(d / "trajectory.csv")
    limits = json.loads((d / "limits.json").read_text())
    result = regenerate(t, joints, q, limits)
    assert result["retime_factor"] == 1.0
    assert result["binding_constraint"] is None
    assert result["fully_restored"]


def test_real_capture_restored_with_a_small_retime_factor():
    d = ROOT / "examples/moveit_capture/panda_goal1"
    t, joints, q = load_csv(d / "trajectory_live.csv")
    limits = json.loads((d / "limits.json").read_text())
    result = regenerate(t, joints, q, limits)
    assert result["before"]["max_acceleration_ratio"] > 1.0
    assert result["fully_restored"]
    assert result["after"]["max_velocity_ratio"] <= 1.0
    assert result["after"]["max_acceleration_ratio"] <= 1.0
    # A small slowdown, not a large one -- this specific violation is mild.
    assert 1.0 < result["retime_factor"] < 1.1
