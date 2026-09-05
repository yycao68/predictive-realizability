"""Plot trajectory values against declared limits (docs/release_checklist.md's
long-open "Add plots" item).

Isolated in its own module, imported lazily by analyzer.main() only when
--plot is actually requested, so matplotlib stays an optional dependency --
the core numpy-only audit path is unaffected if it isn't installed.

Velocity and acceleration are plotted as a ratio (|value|/limit) so any
number of joints with different absolute limits share one axis and one
reference line at 1.0; position has no single natural "ratio" for an
asymmetric [min, max] bound, so it is plotted as the raw value with each
joint's own limit lines.
"""
import numpy as np

from realizability.analyzer import compute_derivatives


def _ratio_panel(ax, t, values, joints, limit_key, limits, ylabel):
    plotted = False
    for j, name in enumerate(joints):
        limit = limits["joints"][name].get(limit_key)
        if limit is None:
            continue
        plotted = True
        limit = float(limit)
        ratio = np.abs(values[:, j]) / limit if limit > 0.0 else np.where(
            np.abs(values[:, j]) > 0.0, np.inf, 0.0
        )
        ax.plot(t, ratio, label=name)
    if plotted:
        ax.axhline(1.0, color="red", linestyle="--", linewidth=1, label="declared limit")
        ax.set_ylabel(ylabel)
        ax.legend(fontsize="small", loc="upper right")
    return plotted


def plot_report(t, joints, q, limits, output_path):
    import matplotlib
    matplotlib.use("Agg")  # headless: this is a file-writing CLI tool, not a GUI
    import matplotlib.pyplot as plt

    v, a = compute_derivatives(t, q)

    has_velocity = any(limits["joints"][name].get("max_velocity") is not None for name in joints)
    has_acceleration = any(limits["joints"][name].get("max_acceleration") is not None for name in joints)
    has_position = any(
        limits["joints"][name].get("min_position") is not None
        or limits["joints"][name].get("max_position") is not None
        for name in joints
    )

    panels = [p for p in (has_velocity, has_acceleration, has_position) if p]
    if not panels:
        raise ValueError("No joint declares any velocity/acceleration/position limit -- nothing to plot")

    n_rows = sum([has_velocity, has_acceleration, has_position])
    fig, axes = plt.subplots(n_rows, 1, sharex=True, figsize=(9, 2.6 * n_rows))
    if n_rows == 1:
        axes = [axes]
    row = iter(axes)

    if has_velocity:
        _ratio_panel(next(row), t, v, joints, "max_velocity", limits, "velocity ratio")
    if has_acceleration:
        _ratio_panel(next(row), t, a, joints, "max_acceleration", limits, "acceleration ratio")
    if has_position:
        ax = next(row)
        for j, name in enumerate(joints):
            lim = limits["joints"][name]
            pmin, pmax = lim.get("min_position"), lim.get("max_position")
            if pmin is None and pmax is None:
                continue
            line, = ax.plot(t, q[:, j], label=name)
            if pmin is not None:
                ax.axhline(float(pmin), color=line.get_color(), linestyle=":", linewidth=1)
            if pmax is not None:
                ax.axhline(float(pmax), color=line.get_color(), linestyle=":", linewidth=1)
        ax.set_ylabel("position (dotted: limits)")
        ax.legend(fontsize="small", loc="upper right")

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_sensitivity_sweep(results, output_path):
    """Peak velocity/acceleration ratio vs. sampling density (representation_sensitivity.py),
    the plot form of docs/methodology.md's representation-sensitivity sweep."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n_actual = [r["n_actual"] for r in results]
    v_ratio = [r["report"]["overall"]["max_velocity_ratio"] for r in results]
    a_ratio = [r["report"]["overall"]["max_acceleration_ratio"] for r in results]

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(n_actual, v_ratio, marker="o", label="velocity ratio")
    ax.plot(n_actual, a_ratio, marker="o", label="acceleration ratio")
    ax.axhline(1.0, color="red", linestyle="--", linewidth=1, label="declared limit")
    ax.set_xlabel("sampling density (N actual samples)")
    ax.set_ylabel("peak ratio")
    ax.legend(fontsize="small")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
