# Planar optimal control

## What this solves

`solvers/planar_ocp.py` solves a minimum-time hot lap of the planar vehicle on
the closed signed track. The lap is transcribed in the **distance domain**: the
independent variable is the path distance, so the mesh follows the track and
the periodic boundary is exact.

States at every node:

| Index | State | Unit |
|---|---|---|
| 0 | lateral deviation from the path | m |
| 1 | heading error against the path tangent | rad |
| 2 | body longitudinal velocity | m/s |
| 3 | body lateral velocity | m/s |
| 4 | yaw rate | rad/s |
| 5 | battery state of charge | 1 |
| 6 | elapsed time | s |

Controls at every node: the throttle, the brake, and the road-wheel steering
angle.

Algebraic variables at every node: four vertical tire loads, four wheel angular
speeds, and the pack current.

## The load coupling stays algebraic

A simulation iterates the vertical load and the tire force to consistency. A
nonlinear program must not nest that iteration, because the solver cannot see
through it. This module carries the four loads as **variables** and imposes the
consistency equations, so the coupling is exact and differentiable.

## The wheel speeds are algebraic, not states

The seven-degree-of-freedom simulation carries the wheel speeds as states. The
optimal control does not, and the reason is stiffness.

The wheel equation has a time constant

```text
tau = J * v / (Re^2 * Kx)
```

which is about **1.4 ms**, or about **3.5 cm of travel** at 25 m/s. A mesh of
5 to 20 m is therefore 150 to 600 time constants long. The sensitivity of one
node's wheel speed to the previous node's is `exp(-ds / 0.035)`, which is
effectively zero. The wheel states decouple from each other, and the constraint
Jacobian becomes badly conditioned. Measured behaviour: the dynamic-wheel
formulation converged only on a 20 m mesh and stalled on every finer mesh,
leaving a load residual of 12 to 41 N.

The optimal control therefore imposes the **quasi-steady wheel balance**

```text
T_i - Re_i * Fx_i + M_rolling_i = 0
```

which is the correct reduction at lap scale, and which removes the stiffness.
The seven-degree-of-freedom model stays the simulation model and is validated
separately.

### The balance needs a traction bound

A quasi-steady wheel can only balance a torque that the tire can transmit. If
the demanded torque exceeds the friction limit, no wheel speed satisfies the
balance, and a solver stalls on an equation that nothing can satisfy. The
formulation therefore also imposes

```text
|T_i| <= Re_i * mu_x_i * Fz_i
```

at every wheel. Without this bound the solver leaves a residual of 30 to 100 on
the wheel balance.

## Verification

### The symbolic physics matches the simulation physics

`solvers/planar_symbolic.py` expresses the tire a second time in CasADi. That
duplication is deliberate, and the cross-comparison is the verification.

- The symbolic tire against the NumPy tire over 405 states: worst relative
  difference **9.1e-15** in the forces and moments.
- The symbolic load equations against the NumPy model at matched states: worst
  residual **3.4e-08 N**, which is the NumPy fixed-point tolerance.

This cross-check found a real defect. The first version rotated the tire forces
with a **scalar** steering angle, so the front steering angle was applied to the
rear wheels as well. The error was invisible at zero steering and grew with
steering: 5.6 N at 3 degrees, 41 N at 12 degrees. The rotation now uses one
angle per wheel.

### A converged solve

At a 20 m mesh with a 9000 iteration limit, IPOPT returns `Solve_Succeeded`:

| Quantity | Value |
|---|---|
| Iterations | 5972 |
| Algebraic residual | 1.4e-07 |
| Periodicity residual | 0.0 |
| Lap time | 36.294 s |

The formulation is therefore consistent and solvable. A 2500 iteration limit is
**not** enough, which is the trap: the solver stops with a plausible lap time
and a load residual of 30 N.

## The mesh must resolve the track curvature

This is the most important limitation, and it is a property of the discretization
rather than of the model.

The path is sampled at the mesh nodes only. A corner about 10 m long that falls
between two nodes is never seen by the mesh. The optimisation then legitimately
drives a much faster lap on the **smoothed** path that it was given.

Measured lap time against mesh spacing, all with the same physics:

| Mesh spacing | Nodes | Lap time |
|---|---|---|
| 20 m | 52 | 36.3 s |
| 10 m | 103 | 39.3 s |
| 5 m | 204 | 40.5 s |

The lap time **rises** as the mesh refines, which is the signature of a coarse
mesh being exploited. The quasi-steady point-mass reference on the same track,
which uses the full 0.5 m path data, gives **68.3 s**. The optimal control must
not be trusted at these spacings.

The track has corners down to a 6.18 m radius, so a trustworthy mesh needs a
1 to 2 m spacing, which is 500 to 1000 nodes. That problem is much larger than
any mesh solved here, and each rung of refinement grows the cost by more than
the node count, because the iteration count grows as well.

**Do not quote a lap time from this module until the mesh resolves the
corners.** The correct next step is a 1 to 2 m solve, warm started from the
converged 20 m and 10 m solutions.

## Stiff integrators: what is available and what fits

The user asked to investigate an HHT integrator from a library. The findings:

| Library | Available | Provides HHT |
|---|---|---|
| CasADi 3.8.1 | installed for this work | no |
| IPOPT | with CasADi | no |
| sundials / scikits.odes | no | no |
| scipy 1.17.1 | yes | no |

No installed library provides HHT, and in any case **HHT is the wrong tool
here**. HHT-alpha is a second-order implicit integrator for structural systems
of the form `M q'' + C q' + K q = F`. The wheel equation is first order, so it
does not take that form. HHT is the right choice for the second-order
suspension and unsprung models of Model 3 and Model 4, not for the planar
wheel spin.

For stiff **first-order** systems, scipy already provides three implicit
integrators: `BDF`, `Radau`, and `LSODA`.

### Measured: the implicit integrators handle this stiffness

A 1.2 s run at 8 m/s with a 2 degree steer and a 400 N*m drive request, against
an explicit RK4 reference at a 0.2 ms step:

| Method | Step | Evaluations | Final speed | Error against reference |
|---|---|---|---|---|
| RK4 explicit | 0.2 ms | 28800 | 11.428808 | reference |
| RK4 explicit | 1 ms | 5760 | 11.4280 | 8.2e-04 |
| RK4 explicit | 2 ms | 2880 | 11.4270 | 1.8e-03 |
| RK4 explicit | 4 ms | 1440 | 11.4249 | 3.9e-03 |
| BDF | 1 to 20 ms | 1037 | 11.4290 | 2.0e-04 |
| Radau | 1 to 20 ms | 2293 | 11.4290 | 2.0e-04 |
| LSODA | 1 to 20 ms | 1086 | 11.4290 | 2.0e-04 |

Two results matter:

1. The implicit integrators are **completely insensitive to the output step**.
   The evaluation count is identical at 1 ms and at 20 ms, because the adaptive
   step control handles the stiffness internally.
2. They are **about ten times cheaper** than explicit RK4 at the accuracy where
   RK4 is still stable.

The 2.0e-04 difference is the RK4 reference's own error, not the implicit
methods': all three implicit methods agree with each other to better than 1e-6,
and they carry a tighter tolerance than the reference.

**Recommendation:** use `BDF` or `LSODA` for any simulation of a model that
contains the wheel dynamics. Reserve HHT for the second-order suspension models
of Model 3 and Model 4, and only if a library supplies it.

## Known limitations

- The lap time is not yet trustworthy, for the mesh-resolution reason above.
- The transcription is a trapezoid rule, which is second order. A finer mesh
  would benefit from a higher-order collocation.
- The lateral error bound is a few centimetres, which is a driving-line
  tolerance, not a track-width limit.
- The pack current carries bounds but no equation, so the driver may draw any
  current the pack allows. That is deliberate.
- The steering uses parallel steering, because the Ackermann table is a zero
  array. A non-zero table would change the two front wheel angles, and the
  formulation already accepts one angle per wheel.
- The motor torque and power bounds are imposed at the peak rating, so the
  thermal duty cycle is not modelled.

## Running it

CasADi and IPOPT are optional. They are not on the default import path, so the
tests skip when CasADi is absent.

```bash
python3 -m pip install --target /tmp/casadi_pkgs casadi
PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_planar_ocp
```
