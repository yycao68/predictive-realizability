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


def analyze(t, joints, q, limits):
    # np.gradient supports nonuniform sampling and is appropriate for an
    # offline audit. This is not a physical acceleration measurement.
    v = np.column_stack([np.gradient(q[:, j], t) for j in range(q.shape[1])])
    a = np.column_stack([np.gradient(v[:, j], t) for j in range(q.shape[1])])

    report = {
        "samples": int(len(t)),
        "duration_s": float(t[-1] - t[0]),
        "joints": {},
    }

    for j, name in enumerate(joints):
        lim = limits["joints"][name]
        vmax = lim.get("max_velocity")
        amax = lim.get("max_acceleration")

        item = {}
        if vmax is not None:
            peak_v = float(np.max(np.abs(v[:, j])))
            ratio_v = peak_v / float(vmax)
            idx = int(np.argmax(np.abs(v[:, j])))
            item["peak_velocity"] = peak_v
            item["velocity_limit"] = float(vmax)
            item["velocity_ratio"] = ratio_v
            item["velocity_margin"] = 1.0 - ratio_v
            item["velocity_violation"] = bool(ratio_v > 1.0)
            item["velocity_first_violation_s"] = (
                float(t[np.flatnonzero(np.abs(v[:, j]) > vmax)[0]])
                if np.any(np.abs(v[:, j]) > vmax) else None
            )

        if amax is not None:
            peak_a = float(np.max(np.abs(a[:, j])))
            ratio_a = peak_a / float(amax)
            item["peak_acceleration"] = peak_a
            item["acceleration_limit"] = float(amax)
            item["acceleration_ratio"] = ratio_a
            item["acceleration_margin"] = 1.0 - ratio_a
            item["acceleration_violation"] = bool(ratio_a > 1.0)
            item["acceleration_first_violation_s"] = (
                float(t[np.flatnonzero(np.abs(a[:, j]) > amax)[0]])
                if np.any(np.abs(a[:, j]) > amax) else None
            )

        report["joints"][name] = item

    report["overall"] = {
        "max_velocity_ratio": max(
            (x.get("velocity_ratio", 0.0) for x in report["joints"].values()),
            default=0.0,
        ),
        "max_acceleration_ratio": max(
            (x.get("acceleration_ratio", 0.0) for x in report["joints"].values()),
            default=0.0,
        ),
    }
    report["overall"]["realizability_audit_pass"] = (
        report["overall"]["max_velocity_ratio"] <= 1.0
        and report["overall"]["max_acceleration_ratio"] <= 1.0
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

    missing = [j for j in joints if j not in limits["joints"]]
    if missing:
        raise ValueError(f"Missing limits for joints: {missing}")

    report = analyze(t, joints, q, limits)

    print(f"Samples: {report['samples']}")
    print(f"Duration: {report['duration_s']:.6f} s")
    print(f"Peak velocity ratio: {report['overall']['max_velocity_ratio']:.6f}")
    print(f"Peak acceleration ratio: {report['overall']['max_acceleration_ratio']:.6f}")
    print("Audit:", "PASS" if report["overall"]["realizability_audit_pass"] else "FAIL")

    if args.output:
        Path(args.output).write_text(json.dumps(report, indent=2))
        print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
