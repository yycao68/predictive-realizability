import json
from pathlib import Path

import pytest

matplotlib = pytest.importorskip("matplotlib")

from realizability.analyzer import load_csv
from realizability.plotting import plot_report

ROOT = Path(__file__).resolve().parents[1]


def test_plot_report_writes_a_file_for_full_scenario(tmp_path):
    d = ROOT / "examples/moveit_capture/panda_goal1"
    t, joints, q = load_csv(d / "trajectory_live.csv")
    limits = json.loads((d / "limits.json").read_text())
    output = tmp_path / "plot.png"

    plot_report(t, joints, q, limits, output)

    assert output.exists()
    assert output.stat().st_size > 0


def test_plot_report_handles_position_only_panel(tmp_path):
    d = ROOT / "examples/scenarios/04_position_violation"
    t, joints, q = load_csv(d / "trajectory.csv")
    limits = json.loads((d / "limits.json").read_text())
    output = tmp_path / "plot.png"

    plot_report(t, joints, q, limits, output)

    assert output.exists()
    assert output.stat().st_size > 0


def test_plot_report_raises_when_nothing_to_plot():
    import numpy as np
    t = np.linspace(0, 1, 5)
    q = np.column_stack([t])
    limits = {"joints": {"joint1": {}}}  # no velocity/acceleration/position declared
    with pytest.raises(ValueError, match="nothing to plot"):
        plot_report(t, ["joint1"], q, limits, "/tmp/_unused.png")
