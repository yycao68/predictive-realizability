"""Is a realizability audit's verdict an artifact of how densely a trajectory
happens to be sampled?

Motivated directly by examples/moveit_capture/panda_goal1: the sparse planned
export (10 waypoints) audits PASS, the dense live capture (289 samples)
audits FAIL, for the same underlying plan. The obvious question a reviewer
would ask is whether that is just an artifact of comparing two arbitrarily
different sampling densities, rather than a real representation-dependent
finding. This module answers it by measurement instead of argument: take one
trajectory (the densest available, closest to ground truth) and audit it at
a swept range of sampling densities, obtained by uniformly SUBSAMPLING that
same signal -- not by comparing two unrelated captures.

This is deliberately not a claim that uniform subsampling reproduces what
TOTG's own bang-bang-optimal waypoint placement would do at a given count --
TOTG chooses waypoints at switching points, not at uniform time intervals.
It answers the narrower, still useful question: holding the underlying
motion fixed, how does peak-ratio detection change as sampling density
drops, for this one common (and simplest) sparsification strategy.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from realizability.analyzer import analyze, load_csv, _active_span


def crop_to_active(t, joints, q, tol=1e-6):
    """Restrict to the active-motion window (analyzer._active_span) before
    downsampling -- otherwise most of a uniform subsample of a capture with a
    long idle tail would just be redundant hold-still points, diluting the
    sweep rather than sampling the motion itself."""
    start, end, _, _ = _active_span(t, q, tol=tol)
    mask = (t >= start) & (t <= end)
    return t[mask], joints, q[mask]


def downsample(t, q, n_target):
    """n_target indices, as evenly spaced as possible across the full range
    (always including the first and last sample)."""
    n = len(t)
    if n_target >= n:
        return t, q
    idx = np.round(np.linspace(0, n - 1, n_target)).astype(int)
    idx = np.unique(idx)  # rounding can collide at low n_target; dedupe rather than pad
    return t[idx], q[idx]


def sweep(t, joints, q, limits, densities):
    """Returns a list of {"n_target": ..., "n_actual": ..., "report": ...}
    in the order given, using the SAME underlying (cropped) trajectory for
    every density."""
    results = []
    for n_target in densities:
        t_s, q_s = downsample(t, q, n_target)
        if len(t_s) < 3:
            continue  # analyze() itself requires >= 3 samples
        report = analyze(t_s, joints, q_s, limits)
        results.append({"n_target": n_target, "n_actual": int(len(t_s)), "report": report})
    return results


def main():
    p = argparse.ArgumentParser(
        description="Sweep sampling density (uniform subsampling of one dense trajectory) "
        "and report how peak-ratio detection changes."
    )
    p.add_argument("csv", help="The densest available trajectory CSV (e.g. a live capture)")
    p.add_argument("--limits", required=True)
    p.add_argument("--densities", default="10,20,50,100,200",
                    help="Comma-separated target sample counts; the full (cropped) density "
                    "is always appended automatically")
    p.add_argument("--output", default=None)
    p.add_argument("--plot", default=None, metavar="PATH.png",
                    help="Write a plot of peak ratio vs. sampling density (requires matplotlib)")
    args = p.parse_args()

    t, joints, q = load_csv(args.csv)
    limits = json.loads(Path(args.limits).read_text())
    t, joints, q = crop_to_active(t, joints, q)

    densities = [int(x) for x in args.densities.split(",")]
    densities.append(len(t))  # the full cropped density, as the reference point
    densities = sorted(set(densities))

    results = sweep(t, joints, q, limits, densities)

    print(f"Active-motion window: {len(t)} samples, {t[-1] - t[0]:.3f}s")
    print(f"{'N (target)':>10}{'N (actual)':>12}{'peak velocity':>16}{'peak acceleration':>20}{'audit':>10}")
    for r in results:
        o = r["report"]["overall"]
        audit = "PASS" if o["realizability_audit_pass"] else "FAIL"
        print(f"{r['n_target']:>10}{r['n_actual']:>12}{o['max_velocity_ratio']:>15.3f}x"
              f"{o['max_acceleration_ratio']:>19.3f}x{audit:>10}")

    passes = [r["report"]["overall"]["realizability_audit_pass"] for r in results]
    if len(set(passes)) > 1:
        flips = [(a["n_actual"], b["n_actual"]) for a, b in zip(results, results[1:])
                 if a["report"]["overall"]["realizability_audit_pass"] != b["report"]["overall"]["realizability_audit_pass"]]
        print(f"\nVerdict changes between: {flips}")
    else:
        print(f"\nVerdict is consistent ({'PASS' if passes[0] else 'FAIL'}) across every density tried.")

    if args.output:
        out = {"active_window_samples": len(t), "active_window_duration_s": float(t[-1] - t[0]),
               "results": results}
        Path(args.output).write_text(json.dumps(out, indent=2))
        print(f"\nWrote {args.output}")

    if args.plot:
        try:
            from realizability.plotting import plot_sensitivity_sweep
        except ImportError as e:
            raise SystemExit(
                "--plot requires matplotlib, which is not installed. "
                "Install it with: pip install matplotlib"
            ) from e
        plot_sensitivity_sweep(results, args.plot)
        print(f"Wrote {args.plot}")


if __name__ == "__main__":
    main()
