import json
from pathlib import Path

from realizability.compare_stages import compare, summarize

ROOT = Path(__file__).resolve().parents[1]


def _scenario_path(name):
    return ROOT / "examples/scenarios" / name


def test_realizability_lost_between_stages_real_capture():
    # The actual v0.2 finding: passes at the planned stage, fails at the live
    # controller-reference stage for the identical underlying plan.
    d = ROOT / "examples/moveit_capture/panda_goal1"
    limits = json.loads((d / "limits.json").read_text())
    results = compare(
        [("planned", d / "trajectory_planned.csv"), ("live", d / "trajectory_live.csv")],
        limits,
    )
    summary = summarize(results)
    assert summary["first_failure"] == "live"
    assert not summary["pass_through_all"]


def test_pass_through_all_stages():
    # Reuse the same passing scenario twice as two "stages" -- realizability
    # is trivially preserved when nothing changes between them.
    d = _scenario_path("01_pass")
    limits = json.loads((d / "limits.json").read_text())
    results = compare(
        [("stage_a", d / "trajectory.csv"), ("stage_b", d / "trajectory.csv")],
        limits,
    )
    summary = summarize(results)
    assert summary["pass_through_all"]
    assert summary["first_failure"] is None


def test_first_stage_already_fails_has_no_upstream_to_blame():
    d = _scenario_path("02_acceleration_violation")
    limits = json.loads((d / "limits.json").read_text())
    results = compare([("only_stage", d / "trajectory.csv")], limits)
    summary = summarize(results)
    assert summary["first_failure"] == "only_stage"
    assert not summary["pass_through_all"]
