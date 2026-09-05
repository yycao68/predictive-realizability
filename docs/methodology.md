# Methodology

## 1. Audit, not measurement

The benchmark computes derivatives from a supplied trajectory representation:

\[
\hat v = \frac{dq}{dt}, \qquad
\hat a = \frac{d^2q}{dt^2}.
\]

These are trajectory-level quantities. They should not be described as measured
physical acceleration.

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

## 4. Stage-wise auditing

For a planning pipeline:

```text
P0: geometric path
P1: time parameterization
P2: smoothing / resampling
P3: controller reference
P4: physical execution
```

audit every available representation. A trajectory that passes at P1 but fails at
P2 demonstrates that a downstream transformation changed its realizability.

## 5. Avoiding overclaim

A failed audit does not automatically imply hardware damage, physical instability,
controller failure, or universal software incorrectness. It establishes only that
the evaluated trajectory representation exceeds the declared limit under the
stated analysis method.
