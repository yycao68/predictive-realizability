import json
from pathlib import Path
import numpy as np

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
