"""Compare the same underlying motion's trajectory as it passes through
multiple pipeline stages (e.g. a raw planned trajectory vs. the executed
controller reference), and report the first stage at which the
realizability audit transitions from PASS to FAIL.

This generalizes examples/moveit_capture/panda_goal1's own planned-vs-live
comparison (v0.2) into a reusable tool: it does not assume any specific
pipeline (OMPL/TOTG/Ruckig/controller) -- stages are just named CSVs, in
the order the caller supplies them, each audited against the same limits.
"""
import argparse
import json
from pathlib import Path

from realizability.analyzer import load_csv, analyze


def compare(stage_files, limits):
    """stage_files: [(name, path), ...] in pipeline order.

    Returns a list of {"name": ..., "report": ...} in the same order.
    """
    results = []
    for name, path in stage_files:
        t, joints, q = load_csv(path)
        report = analyze(t, joints, q, limits)
        results.append({"name": name, "report": report})
    return results


def summarize(results):
    """Find the first stage where the overall audit is not PASS.

    Returns a dict: {"first_failure": <name or None>, "pass_through_all": bool}.
    Does not assume failures are monotonic (a later stage could recover) --
    reports strictly the first one, and separately whether every stage
    passed.
    """
    all_pass = all(r["report"]["overall"]["realizability_audit_pass"] for r in results)
    first_failure = None
    for r in results:
        if not r["report"]["overall"]["realizability_audit_pass"]:
            first_failure = r["name"]
            break
    return {"first_failure": first_failure, "pass_through_all": all_pass}


def _parse_stage_arg(arg):
    if "=" not in arg:
        raise argparse.ArgumentTypeError(
            f"Expected NAME=PATH.csv, got '{arg}' (missing '=')"
        )
    name, path = arg.split("=", 1)
    return name, path


def main():
    p = argparse.ArgumentParser(
        description="Compare the same trajectory across pipeline stages and find "
        "where realizability is lost."
    )
    p.add_argument(
        "stages",
        nargs="+",
        type=_parse_stage_arg,
        help="One or more NAME=PATH.csv pairs, in pipeline order (e.g. planned=a.csv live=b.csv)",
    )
    p.add_argument("--limits", required=True)
    p.add_argument("--output", default=None)
    args = p.parse_args()

    limits = json.loads(Path(args.limits).read_text())
    results = compare(args.stages, limits)
    summary = summarize(results)

    print(f"{'stage':<20}{'peak velocity':>16}{'peak acceleration':>20}{'audit':>10}")
    for r in results:
        o = r["report"]["overall"]
        audit = "PASS" if o["realizability_audit_pass"] else "FAIL"
        print(f"{r['name']:<20}{o['max_velocity_ratio']:>15.3f}x{o['max_acceleration_ratio']:>19.3f}x{audit:>10}")

    print()
    if summary["pass_through_all"]:
        print(f"Realizability preserved across all {len(results)} stages.")
    elif summary["first_failure"] == results[0]["name"]:
        print(f"Already fails at the first supplied stage ('{summary['first_failure']}') -- "
              "no upstream stage was supplied to localize where it was lost.")
    else:
        print(f"Realizability lost at stage '{summary['first_failure']}' "
              f"(the stage before it still passed).")

    if args.output:
        out = {"stages": [r["name"] for r in results],
               "reports": {r["name"]: r["report"] for r in results},
               "summary": summary}
        Path(args.output).write_text(json.dumps(out, indent=2))
        print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
