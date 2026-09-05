"""A receding-horizon predictive margin over an already-known trajectory.

Scope, stated precisely: this tool only has kinematic trajectory data
(position, and derived velocity/acceleration) -- no dynamics model, no
torque, no uncertainty bound. It cannot reproduce the Predictive
Realizability paper's own m_phys certificate, which is defined over
predicted torque with an explicit uncertainty term. What it CAN honestly
compute from a CSV alone: since the full trajectory represents an
already-known plan (its "future" samples exist in the file before any
execution reaches them), a receding-horizon scan over that plan is a
legitimate predictive check -- at sample k, it looks ahead over
[t_k, t_k+horizon] using data already available in the plan, exactly what
an online monitor with access to the upcoming plan (not the future
execution) could compute in real time.

This produces two things a post-hoc audit (analyzer.py) does not:
- a warning time: the first t_k at which the look-ahead window itself
  contains a violation, which is <= the time the violation actually occurs;
- the resulting lead time: how much earlier the horizon check would have
  flagged trouble, compared to analyzer.py's own first_violation_s fields.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from realizability.analyzer import compute_derivatives, load_csv


def _window_worst_ratio(signal_col, limit):
    """Peak |signal| / limit over an already-sliced window, same zero-limit
    handling as analyzer._bound_report."""
    if len(signal_col) == 0:
        return 0.0
    peak = float(np.max(np.abs(signal_col)))
    limit = float(limit)
    if limit > 0.0:
        return peak / limit
    return float("inf") if peak > 0.0 else 0.0


def predictive_margin(t, joints, q, limits, horizon_s):
    """Returns a dict with a per-sample warning-ratio time series and the
    warning/actual/lead-time summary, for the worst joint/signal combined.
    """
    v, a = compute_derivatives(t, q)
    n = len(t)

    warning_ratio = np.zeros(n)
    for k in range(n):
        window = (t >= t[k]) & (t <= t[k] + horizon_s)
        worst = 0.0
        for j, name in enumerate(joints):
            lim = limits["joints"][name]
            vmax = lim.get("max_velocity")
            if vmax is not None:
                worst = max(worst, _window_worst_ratio(v[window, j], vmax))
            amax = lim.get("max_acceleration")
            if amax is not None:
                worst = max(worst, _window_worst_ratio(a[window, j], amax))
        warning_ratio[k] = worst

    # Actual (unwindowed) instantaneous violation time: horizon=0 case.
    actual_ratio = np.zeros(n)
    for k in range(n):
        worst = 0.0
        for j, name in enumerate(joints):
            lim = limits["joints"][name]
            vmax = lim.get("max_velocity")
            if vmax is not None:
                worst = max(worst, _window_worst_ratio(v[k:k + 1, j], vmax))
            amax = lim.get("max_acceleration")
            if amax is not None:
                worst = max(worst, _window_worst_ratio(a[k:k + 1, j], amax))
        actual_ratio[k] = worst

    warn_idx = np.flatnonzero(warning_ratio > 1.0)
    actual_idx = np.flatnonzero(actual_ratio > 1.0)

    t_warning = float(t[warn_idx[0]]) if len(warn_idx) else None
    t_actual = float(t[actual_idx[0]]) if len(actual_idx) else None
    lead_time_s = (t_actual - t_warning) if (t_warning is not None and t_actual is not None) else None

    return {
        "horizon_s": horizon_s,
        "samples": int(n),
        "series": [
            {"t": float(t[k]), "warning_ratio": float(warning_ratio[k]), "actual_ratio": float(actual_ratio[k])}
            for k in range(n)
        ],
        "t_warning_s": t_warning,
        "t_actual_violation_s": t_actual,
        "lead_time_s": lead_time_s,
        "predicted_before_actual": (
            t_warning is not None and t_actual is not None and t_warning < t_actual
        ),
    }


def main():
    p = argparse.ArgumentParser(
        description="Receding-horizon predictive margin over an already-known trajectory."
    )
    p.add_argument("csv")
    p.add_argument("--limits", required=True)
    p.add_argument("--horizon", type=float, required=True, help="Look-ahead window, in seconds")
    p.add_argument("--output", default=None)
    args = p.parse_args()

    t, joints, q = load_csv(args.csv)
    limits = json.loads(Path(args.limits).read_text())
    result = predictive_margin(t, joints, q, limits, args.horizon)

    print(f"Horizon: {args.horizon:.3f} s over {result['samples']} samples")
    if result["t_actual_violation_s"] is None:
        print("No violation occurs in this trajectory -- nothing to predict.")
    elif result["t_warning_s"] is None:
        print(f"Violation occurs at t={result['t_actual_violation_s']:.3f}s but the "
              f"{args.horizon:.3f}s horizon never gave a warning before it -- "
              "the trajectory changes faster than this horizon can look ahead.")
    else:
        print(f"First warning:    t={result['t_warning_s']:.3f}s")
        print(f"Actual violation: t={result['t_actual_violation_s']:.3f}s")
        print(f"Lead time:        {result['lead_time_s']:.3f}s "
              f"({'predicted before it happened' if result['predicted_before_actual'] else 'no lead -- coincides with the violation itself'})")

    if args.output:
        Path(args.output).write_text(json.dumps(result, indent=2))
        print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
