import json
from pathlib import Path

import numpy as np

from realizability.analyzer import load_csv
from realizability.representation_sensitivity import crop_to_active, downsample, sweep

ROOT = Path(__file__).resolve().parents[1]


def test_crop_to_active_drops_trailing_idle_time():
    t = np.concatenate([np.linspace(0, 0.5, 51), np.linspace(0.51, 1.5, 100)])
    moving_part = np.linspace(0, 1.0, 51)
    held_part = np.full(100, 1.0)
    q = np.column_stack([np.concatenate([moving_part, held_part])])
    t_c, joints, q_c = crop_to_active(t, ["joint1"], q)
    assert t_c[-1] - t_c[0] < 0.52  # active window only, not the full 1.5s


def test_downsample_always_keeps_first_and_last_sample():
    t = np.linspace(0, 1, 100)
    q = np.column_stack([t])
    t_s, q_s = downsample(t, q, 10)
    assert t_s[0] == t[0]
    assert t_s[-1] == t[-1]
    assert len(t_s) <= 10


def test_downsample_no_op_when_target_exceeds_available_samples():
    t = np.linspace(0, 1, 20)
    q = np.column_stack([t])
    t_s, q_s = downsample(t, q, 200)
    assert len(t_s) == 20  # can't manufacture samples that don't exist


def test_real_capture_shows_a_threshold_crossing():
    # The actual finding: sparse sampling of the real goal1 live capture
    # audits PASS, denser sampling of the SAME underlying signal audits FAIL --
    # not an artifact of comparing two unrelated captures at different
    # densities, since this sweeps ONE trajectory.
    d = ROOT / "examples/moveit_capture/panda_goal1"
    t, joints, q = load_csv(d / "trajectory_live.csv")
    limits = json.loads((d / "limits.json").read_text())
    t, joints, q = crop_to_active(t, joints, q)

    results = sweep(t, joints, q, limits, [10, 20, 50, len(t)])
    passes = [r["report"]["overall"]["realizability_audit_pass"] for r in results]

    assert passes[0]  # sparsest (N=10) passes
    assert not passes[-1]  # densest (full active window) fails
    assert len(set(passes)) > 1  # a real crossing occurred somewhere in between


def test_sweep_ratio_is_monotonic_non_decreasing_on_this_real_capture():
    # Not asserted as a general law (not every signal need behave this way),
    # but true and worth locking in as a regression check for this specific,
    # already-verified real trajectory.
    d = ROOT / "examples/moveit_capture/panda_goal1"
    t, joints, q = load_csv(d / "trajectory_live.csv")
    limits = json.loads((d / "limits.json").read_text())
    t, joints, q = crop_to_active(t, joints, q)

    results = sweep(t, joints, q, limits, [10, 20, 50, len(t)])
    ratios = [r["report"]["overall"]["max_acceleration_ratio"] for r in results]
    assert all(b >= a - 1e-9 for a, b in zip(ratios, ratios[1:]))
