import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        if not fields or fields[0] != "time":
            raise ValueError("CSV must have first column named 'time'")
        rows = list(reader)

    t = np.asarray([float(r["time"]) for r in rows], dtype=float)
    joints = fields[1:]
    q = np.asarray([[float(r[j]) for j in joints] for r in rows], dtype=float)

    if len(t) < 3:
        raise ValueError("At least three samples are required")
    if np.any(np.diff(t) <= 0):
        raise ValueError("time must be strictly increasing")
    return t, joints, q


def compute_derivatives(t, q):
    """Velocity and acceleration via numerical differentiation of position.

    Shared by analyze() and lookahead_margin.py so both differentiate the
    same way -- np.gradient supports nonuniform sampling and is appropriate
    for an offline audit. This is not a physical measurement.
    """
    v = np.gradient(q, t, axis=0)
    a = np.gradient(v, t, axis=0)
    return v, a


def _bound_report(values, t, limit, prefix):
    """Peak-to-limit audit for one signal (velocity or acceleration) on one joint.

    A limit of 0.0 (e.g. a locked joint) means "must stay at zero" -- ratio is
    reported as inf on any nonzero value rather than raising ZeroDivisionError.
    """
    abs_values = np.abs(values)
    peak = float(np.max(abs_values))
    peak_idx = int(np.argmax(abs_values))
    limit = float(limit)

    if limit > 0.0:
        ratio = peak / limit
    else:
        ratio = float("inf") if peak > 0.0 else 0.0

    violated = abs_values > limit
    return {
        f"peak_{prefix}": peak,
        f"{prefix}_limit": limit,
        f"{prefix}_ratio": ratio,
        f"{prefix}_margin": 1.0 - ratio,
        f"{prefix}_violation": bool(ratio > 1.0),
        f"{prefix}_peak_time_s": float(t[peak_idx]),
        f"{prefix}_first_violation_s": (
            float(t[np.flatnonzero(violated)[0]]) if np.any(violated) else None
        ),
    }


def _active_span(t, q, tol=1e-6):
    """The [start, end] time span over which any joint's position actually
    changes between consecutive samples, distinct from the full recorded
    duration -- a capture can include leading/trailing idle time (e.g. a
    robot holding its final pose after motion ends) that inflates
    duration_s without reflecting real motion. Returns (start, end,
    leading_idle_s, trailing_idle_s); if nothing moves at all, the span
    collapses to a single instant at t[0].
    """
    moving = np.max(np.abs(np.diff(q, axis=0)), axis=1) > tol
    if not np.any(moving):
        return float(t[0]), float(t[0]), 0.0, float(t[-1] - t[0])
    first, last = np.flatnonzero(moving)[0], np.flatnonzero(moving)[-1]
    start, end = float(t[first]), float(t[last + 1])
    return start, end, start - float(t[0]), float(t[-1]) - end


def analyze(t, joints, q, limits):
    missing = [j for j in joints if j not in limits["joints"]]
    if missing:
        raise ValueError(f"Missing limits for joints: {missing}")

    v, a = compute_derivatives(t, q)
    active_start, active_end, leading_idle_s, trailing_idle_s = _active_span(t, q)

    report = {
        "samples": int(len(t)),
        "duration_s": float(t[-1] - t[0]),
        "active_duration_s": active_end - active_start,
        "leading_idle_s": leading_idle_s,
        "trailing_idle_s": trailing_idle_s,
        "joints": {},
    }

    for j, name in enumerate(joints):
        lim = limits["joints"][name]
        item = {}

        pmin, pmax = lim.get("min_position"), lim.get("max_position")
        if pmin is not None or pmax is not None:
            pos = q[:, j]
            below = pos < pmin if pmin is not None else np.zeros_like(pos, dtype=bool)
            above = pos > pmax if pmax is not None else np.zeros_like(pos, dtype=bool)
            violated = below | above
            item["min_position"] = float(pmin) if pmin is not None else None
            item["max_position"] = float(pmax) if pmax is not None else None
            item["position_violation"] = bool(np.any(violated))
            item["position_first_violation_s"] = (
                float(t[np.flatnonzero(violated)[0]]) if np.any(violated) else None
            )

        vmax = lim.get("max_velocity")
        if vmax is not None:
            item.update(_bound_report(v[:, j], t, vmax, "velocity"))

        amax = lim.get("max_acceleration")
        if amax is not None:
            item.update(_bound_report(a[:, j], t, amax, "acceleration"))

        report["joints"][name] = item

    report["overall"] = {
        "max_velocity_ratio": max(
            (x["velocity_ratio"] for x in report["joints"].values() if "velocity_ratio" in x),
            default=0.0,
        ),
        "max_acceleration_ratio": max(
            (x["acceleration_ratio"] for x in report["joints"].values() if "acceleration_ratio" in x),
            default=0.0,
        ),
        "position_violation": any(
            x.get("position_violation", False) for x in report["joints"].values()
        ),
    }
    report["overall"]["realizability_audit_pass"] = (
        report["overall"]["max_velocity_ratio"] <= 1.0
        and report["overall"]["max_acceleration_ratio"] <= 1.0
        and not report["overall"]["position_violation"]
    )
    report["velocity_source"] = "numerical_derivative"
    report["acceleration_source"] = "numerical_derivative"
    return report


def main():
    p = argparse.ArgumentParser(description="Audit a joint trajectory for velocity/acceleration limits.")
    p.add_argument("csv")
    p.add_argument("--limits", required=True)
    p.add_argument("--output", default=None)
    p.add_argument("--plot", default=None, metavar="PATH.png",
                    help="Write a plot of trajectory values vs. declared limits (requires matplotlib)")
    args = p.parse_args()

    t, joints, q = load_csv(args.csv)
    limits = json.loads(Path(args.limits).read_text())

    report = analyze(t, joints, q, limits)

    print(f"Samples: {report['samples']}")
    print(f"Duration: {report['duration_s']:.6f} s "
          f"(active motion: {report['active_duration_s']:.6f} s"
          + (f", leading idle: {report['leading_idle_s']:.3f} s" if report["leading_idle_s"] > 0.001 else "")
          + (f", trailing idle: {report['trailing_idle_s']:.3f} s" if report["trailing_idle_s"] > 0.001 else "")
          + ")")
    print(f"Peak velocity ratio: {report['overall']['max_velocity_ratio']:.6f}")
    print(f"Peak acceleration ratio: {report['overall']['max_acceleration_ratio']:.6f}")
    if report["overall"]["position_violation"]:
        print("Position limit violation detected")
    print("Audit:", "PASS" if report["overall"]["realizability_audit_pass"] else "FAIL")

    if args.output:
        Path(args.output).write_text(json.dumps(report, indent=2))
        print(f"Wrote {args.output}")

    if args.plot:
        try:
            from realizability.plotting import plot_report
        except ImportError as e:
            raise SystemExit(
                "--plot requires matplotlib, which is not installed. "
                "Install it with: pip install matplotlib"
            ) from e
        plot_report(t, joints, q, limits, args.plot)
        print(f"Wrote {args.plot}")


if __name__ == "__main__":
    main()
