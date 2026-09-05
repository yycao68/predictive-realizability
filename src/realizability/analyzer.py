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


def analyze(t, joints, q, limits):
    missing = [j for j in joints if j not in limits["joints"]]
    if missing:
        raise ValueError(f"Missing limits for joints: {missing}")

    # np.gradient supports nonuniform sampling and is appropriate for an
    # offline audit. This is not a physical acceleration measurement.
    v = np.gradient(q, t, axis=0)
    a = np.gradient(v, t, axis=0)

    report = {
        "samples": int(len(t)),
        "duration_s": float(t[-1] - t[0]),
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
    return report


def main():
    p = argparse.ArgumentParser(description="Audit a joint trajectory for velocity/acceleration limits.")
    p.add_argument("csv")
    p.add_argument("--limits", required=True)
    p.add_argument("--output", default=None)
    args = p.parse_args()

    t, joints, q = load_csv(args.csv)
    limits = json.loads(Path(args.limits).read_text())

    report = analyze(t, joints, q, limits)

    print(f"Samples: {report['samples']}")
    print(f"Duration: {report['duration_s']:.6f} s")
    print(f"Peak velocity ratio: {report['overall']['max_velocity_ratio']:.6f}")
    print(f"Peak acceleration ratio: {report['overall']['max_acceleration_ratio']:.6f}")
    if report["overall"]["position_violation"]:
        print("Position limit violation detected")
    print("Audit:", "PASS" if report["overall"]["realizability_audit_pass"] else "FAIL")

    if args.output:
        Path(args.output).write_text(json.dumps(report, indent=2))
        print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
