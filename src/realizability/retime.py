"""Uniform retiming: the one trajectory-regeneration mechanism this tool can
honestly implement without a dynamics model.

Slowing a trajectory down uniformly (t' = lambda*(t-t[0]) + t[0], lambda >= 1,
geometric path q(t) unchanged) scales velocity by 1/lambda and acceleration by
1/lambda^2 -- both purely kinematic facts, no torque or dynamics required.
This is the same mechanism the Predictive Realizability paper calls Level 1
retiming, and it inherits the same real limitation the paper's own retiming
Lemma states: retiming never changes q(t) itself, so a position-limit
violation is completely unaffected by it. This module reports that
explicitly rather than silently claiming "restored" when only velocity/
acceleration were addressed.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from realizability.analyzer import analyze, compute_derivatives, load_csv
from realizability.moveit_export import write_csv


def required_retime_factor(t, joints, q, limits):
    """The minimal uniform lambda >= 1 that brings every joint's velocity and
    acceleration ratio to <= 1.0, plus which joint/signal was binding.

    A joint whose declared limit is 0.0 with nonzero peak motion cannot be
    resolved by any finite lambda (velocity/acceleration only ever scale
    toward zero in the limit) -- such joints are reported separately in
    `unresolvable`, not folded into lambda as an infinite value.
    """
    v, a = compute_derivatives(t, q)
    lam = 1.0
    binding = None
    unresolvable = []

    for j, name in enumerate(joints):
        lim = limits["joints"][name]

        vmax = lim.get("max_velocity")
        if vmax is not None:
            peak_v = float(np.max(np.abs(v[:, j])))
            if vmax > 0.0:
                lam_v = peak_v / vmax
                if lam_v > lam:
                    lam, binding = lam_v, {"joint": name, "signal": "velocity"}
            elif peak_v > 0.0:
                unresolvable.append({"joint": name, "signal": "velocity"})

        amax = lim.get("max_acceleration")
        if amax is not None:
            peak_a = float(np.max(np.abs(a[:, j])))
            if amax > 0.0:
                lam_a = math.sqrt(peak_a / amax)
                if lam_a > lam:
                    lam, binding = lam_a, {"joint": name, "signal": "acceleration"}
            elif peak_a > 0.0:
                unresolvable.append({"joint": name, "signal": "acceleration"})

    if lam > 1.0:
        # The closed-form factor lands exactly at ratio == 1.0 mathematically;
        # floating-point roundoff in the sqrt/square roundtrip can then push
        # the *achieved* ratio a few ULPs past 1.0, which the strict "ratio >
        # 1.0" violation check would flag. A small relative safety margin
        # keeps the retimed trajectory genuinely, not just theoretically,
        # within limits -- verified by re-auditing in regenerate(), not just
        # asserted here.
        lam *= 1.0 + 1e-9

    return lam, binding, unresolvable


def retime_trajectory(t, lam):
    """t' = lambda * (t - t[0]) + t[0] -- uniform time dilation from the
    trajectory's own start; the position sequence q is unchanged."""
    return t[0] + lam * (t - t[0])


def regenerate(t, joints, q, limits):
    """Compute the minimal retiming factor, apply it, and verify by
    re-auditing the retimed trajectory -- rather than assuming the closed-form
    factor is sufficient, matching this project's own standing practice of
    checking a result, not just deriving and trusting it.
    """
    before = analyze(t, joints, q, limits)
    lam, binding, unresolvable = required_retime_factor(t, joints, q, limits)
    t_new = retime_trajectory(t, lam)
    after = analyze(t_new, joints, q, limits)

    velocity_acceleration_restored = (
        after["overall"]["max_velocity_ratio"] <= 1.0
        and after["overall"]["max_acceleration_ratio"] <= 1.0
        and not unresolvable
    )
    return {
        "retime_factor": lam,
        "binding_constraint": binding,
        "unresolvable_by_retiming": unresolvable,
        "before": before["overall"],
        "after": after["overall"],
        "velocity_acceleration_restored": velocity_acceleration_restored,
        "position_violation_remains": after["overall"]["position_violation"],
        "fully_restored": velocity_acceleration_restored and not after["overall"]["position_violation"],
        "t_retimed": t_new.tolist(),
    }


def main():
    p = argparse.ArgumentParser(
        description="Compute and apply the minimal uniform retiming factor that restores "
        "velocity/acceleration limits (does not address position violations)."
    )
    p.add_argument("csv")
    p.add_argument("--limits", required=True)
    p.add_argument("--output-csv", default=None, help="Write the retimed trajectory here")
    p.add_argument("--output-report", default=None)
    args = p.parse_args()

    t, joints, q = load_csv(args.csv)
    limits = json.loads(Path(args.limits).read_text())
    result = regenerate(t, joints, q, limits)

    print(f"Retime factor: {result['retime_factor']:.4f}x "
          + (f"(binding: {result['binding_constraint']['joint']} {result['binding_constraint']['signal']})"
             if result["binding_constraint"] else "(already within limits, no retiming needed)"))
    print(f"Before: velocity {result['before']['max_velocity_ratio']:.3f}x, "
          f"acceleration {result['before']['max_acceleration_ratio']:.3f}x, "
          f"position_violation={result['before']['position_violation']}")
    print(f"After:  velocity {result['after']['max_velocity_ratio']:.3f}x, "
          f"acceleration {result['after']['max_acceleration_ratio']:.3f}x, "
          f"position_violation={result['after']['position_violation']}")

    if result["unresolvable_by_retiming"]:
        print(f"Cannot be resolved by any finite retiming: {result['unresolvable_by_retiming']}")
    if result["position_violation_remains"]:
        print("Position violation remains -- retiming does not change q(t), only its timing. "
              "Restoring this would require reshaping or rerouting, out of this tool's scope.")
    print("Fully restored:" if result["fully_restored"] else "NOT fully restored:",
          result["fully_restored"])

    if args.output_csv:
        write_csv(joints, list(zip(result["t_retimed"], q.tolist())), args.output_csv)
        print(f"Wrote retimed trajectory to {args.output_csv}")
    if args.output_report:
        Path(args.output_report).write_text(json.dumps(result, indent=2))
        print(f"Wrote {args.output_report}")


if __name__ == "__main__":
    main()
