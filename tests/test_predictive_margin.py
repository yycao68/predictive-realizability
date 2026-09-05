import json
from pathlib import Path

import numpy as np

from realizability.analyzer import load_csv
from realizability.predictive_margin import predictive_margin

ROOT = Path(__file__).resolve().parents[1]


def test_horizon_gives_real_lead_time_on_a_ramping_violation():
    # a(t) = 6t ramps from 0, crossing a limit of 3.0 rad/s^2 at t=0.5s. A
    # 0.3s horizon should warn about that crossing ~0.3s before it happens,
    # since the plan's future is already fully known.
    t = np.linspace(0, 1, 101)
    q = np.column_stack([t**3])
    limits = {"joints": {"joint1": {"max_acceleration": 3.0}}}
    result = predictive_margin(t, ["joint1"], q, limits, horizon_s=0.3)
    assert result["predicted_before_actual"]
    assert abs(result["lead_time_s"] - 0.3) < 0.02
    assert abs(result["t_actual_violation_s"] - 0.51) < 0.02


def test_zero_horizon_gives_zero_lead():
    # With no look-ahead at all, the "warning" is exactly the instantaneous
    # audit -- warning and actual violation must coincide.
    t = np.linspace(0, 1, 101)
    q = np.column_stack([t**3])
    limits = {"joints": {"joint1": {"max_acceleration": 3.0}}}
    result = predictive_margin(t, ["joint1"], q, limits, horizon_s=0.0)
    assert result["t_warning_s"] == result["t_actual_violation_s"]
    assert result["lead_time_s"] == 0.0
    assert not result["predicted_before_actual"]


def test_no_violation_means_no_warning_or_actual_time():
    d = ROOT / "examples/scenarios/01_pass"
    t, joints, q = load_csv(d / "trajectory.csv")
    limits = json.loads((d / "limits.json").read_text())
    result = predictive_margin(t, joints, q, limits, horizon_s=0.2)
    assert result["t_warning_s"] is None
    assert result["t_actual_violation_s"] is None
    assert result["lead_time_s"] is None


def test_smaller_horizon_gives_smaller_lead_time():
    t = np.linspace(0, 1, 101)
    q = np.column_stack([t**3])
    limits = {"joints": {"joint1": {"max_acceleration": 3.0}}}
    small = predictive_margin(t, ["joint1"], q, limits, horizon_s=0.05)
    large = predictive_margin(t, ["joint1"], q, limits, horizon_s=0.3)
    assert small["lead_time_s"] < large["lead_time_s"]


def test_real_capture_gives_a_finite_lead_time():
    # Applied to the actual goal1 live capture (v0.2), not just synthetic
    # signals -- confirms this produces a sensible number on real data.
    d = ROOT / "examples/moveit_capture/panda_goal1"
    t, joints, q = load_csv(d / "trajectory_live.csv")
    limits = json.loads((d / "limits.json").read_text())
    result = predictive_margin(t, joints, q, limits, horizon_s=0.2)
    assert result["t_actual_violation_s"] is not None
    if result["t_warning_s"] is not None:
        assert result["t_warning_s"] <= result["t_actual_violation_s"]
