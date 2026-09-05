import json
from pathlib import Path

import numpy as np
import pytest

from realizability.analyzer import load_csv, analyze

ROOT = Path(__file__).resolve().parents[1]


def test_demo_passes():
    t, joints, q = load_csv(ROOT / "examples/demo_trajectory.csv")
    limits = json.loads((ROOT / "examples/limits.json").read_text())
    report = analyze(t, joints, q, limits)
    assert report["overall"]["realizability_audit_pass"]


def test_violation_is_detected():
    t = np.linspace(0, 1, 101)
    q = np.column_stack([5.0 * t**2, np.zeros_like(t)])
    joints = ["joint1", "joint2"]
    limits = {
        "joints": {
            "joint1": {"max_acceleration": 4.0},
            "joint2": {"max_acceleration": 4.0},
        }
    }
    report = analyze(t, joints, q, limits)
    assert report["joints"]["joint1"]["acceleration_violation"]


def test_zero_limit_does_not_raise_and_is_flagged():
    # A locked joint (max_velocity=0.0) must not raise ZeroDivisionError, and
    # any nonzero motion on it must be reported as a violation.
    t = np.linspace(0, 1, 11)
    q = np.column_stack([t, np.zeros_like(t)])
    joints = ["joint1", "joint2"]
    limits = {
        "joints": {
            "joint1": {"max_velocity": 0.0},
            "joint2": {"max_velocity": 1.0},
        }
    }
    report = analyze(t, joints, q, limits)
    assert report["joints"]["joint1"]["velocity_violation"]
    assert report["joints"]["joint1"]["velocity_ratio"] == float("inf")
    assert not report["joints"]["joint2"]["velocity_violation"]


def test_missing_joint_limit_raises_from_analyze_directly():
    # analyze() is the public library entry point (not just main()'s CLI path)
    # and must validate its own input rather than raising a bare KeyError.
    t = np.linspace(0, 1, 5)
    q = np.column_stack([t, t])
    joints = ["joint1", "joint2"]
    limits = {"joints": {"joint1": {"max_velocity": 1.0}}}
    with pytest.raises(ValueError, match="joint2"):
        analyze(t, joints, q, limits)


def test_position_limit_violation_detected():
    t = np.linspace(0, 1, 11)
    q = np.column_stack([np.linspace(0, 2.0, 11)])
    joints = ["joint1"]
    limits = {"joints": {"joint1": {"max_position": 1.0}}}
    report = analyze(t, joints, q, limits)
    assert report["joints"]["joint1"]["position_violation"]
    assert not report["overall"]["realizability_audit_pass"]


def _load_scenario(name):
    d = ROOT / "examples/scenarios" / name
    t, joints, q = load_csv(d / "trajectory.csv")
    limits = json.loads((d / "limits.json").read_text())
    return analyze(t, joints, q, limits)


def test_scenario_01_pass():
    report = _load_scenario("01_pass")
    assert report["overall"]["realizability_audit_pass"]


def test_scenario_02_isolates_acceleration_violation():
    report = _load_scenario("02_acceleration_violation")
    j = report["joints"]["joint1"]
    assert j["acceleration_violation"]
    assert not j["velocity_violation"]
    assert not j["position_violation"]
    assert not report["overall"]["realizability_audit_pass"]


def test_scenario_03_isolates_velocity_violation():
    report = _load_scenario("03_velocity_violation")
    j = report["joints"]["joint1"]
    assert j["velocity_violation"]
    assert not j["acceleration_violation"]
    assert not j["position_violation"]
    assert not report["overall"]["realizability_audit_pass"]


def test_scenario_04_isolates_position_violation():
    report = _load_scenario("04_position_violation")
    j = report["joints"]["joint1"]
    assert j["position_violation"]
    assert not j["velocity_violation"]
    assert not j["acceleration_violation"]
    assert not report["overall"]["realizability_audit_pass"]


def test_active_duration_excludes_trailing_idle_time():
    # A trajectory that moves for 0.5s then holds still for another 1.0s --
    # active_duration_s should reflect the real motion, not the full capture.
    t = np.concatenate([np.linspace(0, 0.5, 51), np.linspace(0.51, 1.5, 100)])
    moving_part = np.linspace(0, 1.0, 51)
    held_part = np.full(100, 1.0)
    q = np.column_stack([np.concatenate([moving_part, held_part])])
    joints = ["joint1"]
    limits = {"joints": {"joint1": {"max_velocity": 10.0}}}  # generous, not the point of this test
    report = analyze(t, joints, q, limits)
    assert report["duration_s"] > 1.4
    assert abs(report["active_duration_s"] - 0.5) < 0.02
    assert report["trailing_idle_s"] > 0.9
    assert report["leading_idle_s"] < 0.02


def test_acceleration_source_is_labeled_numerical():
    d = ROOT / "examples/scenarios/01_pass"
    report = _load_scenario("01_pass")
    assert report["velocity_source"] == "numerical_derivative"
    assert report["acceleration_source"] == "numerical_derivative"
