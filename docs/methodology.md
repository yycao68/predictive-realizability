# Methodology

## 1. Audit, not measurement

The benchmark computes derivatives from a supplied trajectory representation:

\[
\hat v = \frac{dq}{dt}, \qquad
\hat a = \frac{d^2q}{dt^2}.
\]

These are trajectory-level quantities. They should not be described as measured physical acceleration.

## 2. Constraint ratio

For a quantity \(x\) with limit \(x_{\max}\):

\[
\rho = \frac{\max |x|}{x_{\max}}.
\]

The corresponding normalized margin is

\[
m = 1-\rho.
\]

A negative margin means the supplied trajectory violates the declared bound.

## 3. Predictive Realizability connection

A post-hoc checker answers:

> Did the trajectory violate the constraint?

Predictive Realizability asks:

> Can future loss of margin be predicted early enough to modify the trajectory before the violation occurs?

Thus the benchmark separates:

```text
retrospective audit
       ↓
predictive certificate
```

The first establishes the empirical gap; the second is the research contribution.

`realizability.lookahead_margin` (v0.4) is a first, deliberately narrow step toward the
second, not a reproduction of it — named "look-ahead," not "predictive," specifically to
keep that name free for the real dynamics-aware certificate rather than implying this module
already is one. It has no dynamics model, no torque, and no uncertainty bound, so it cannot
compute the paper's own $m_{\mathrm{phys}}$ certificate. What it audits instead: since a
trajectory CSV represents an already-known plan, scanning it with a receding look-ahead
window (rather than over the whole trajectory at once) is a legitimate look-ahead check, and
it reports the resulting warning time, actual violation time, and lead time. This is the
empirical floor the real certificate would need to beat, not a stand-in for it.

## 4. Stage-wise auditing

For a planning pipeline:

```text
P0: geometric path
P1: time parameterization
P2: smoothing / resampling
P3: controller reference
P4: physical execution
```

audit every available representation. A trajectory that passes at P1 but fails at P2 demonstrates that a downstream transformation changed its realizability.

## 5. Avoiding overclaim

A failed audit does not automatically imply hardware damage, physical instability, controller failure, or universal software incorrectness. It establishes only that the evaluated trajectory representation exceeds the declared limit under the stated analysis method.

## 6. Regeneration scope

The paper's own hierarchy for restoring a lost margin is retime, then reshape, then reroute,
then fall back — each a progressively larger intervention, retime being the cheapest and
reshape/reroute needing a dynamics model this tool does not have. `realizability.retime`
(v0.5) implements only retime: a uniform time-dilation $t' = \lambda t$ that scales velocity
by $1/\lambda$ and acceleration by $1/\lambda^2$ while leaving $q(t)$ itself unchanged. That
last fact is a real, load-bearing limitation, not an implementation gap to fill in later:
retiming provably cannot restore a position-limit violation, because position never changes
under pure time-dilation. `retime.py` reports this explicitly (`position_violation_remains`)
rather than treating "audit still fails after retiming" as a bug.

A second, deeper limitation this tool cannot even represent yet, since it has no torque at
all: retiming only helps a *kinematic* violation. In the manipulator equation
$\tau = M(q)\ddot q + C(q,\dot q)\dot q + g(q)$, the inertial and Coriolis terms shrink under
retiming ($1/\lambda^2$ and $1/\lambda$ respectively), but the gravity term $g(q)$ depends
only on position and does not shrink at all — a trajectory that is torque-infeasible because
of gravity alone would remain torque-infeasible no matter how much it is slowed down. This is
anticipated future work (a dynamics-aware audit), not a claim this tool currently checks.
