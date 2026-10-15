# Planar optimal control

> **ARCHIVED HISTORICAL RECORD — NOT SUPPORTED OR ACTIONABLE.** The planar OCP
> implementation and its result-construction/seed APIs are absent from the
> current source tree and will not be restored as part of this verification
> work. **Every solver result, lap-time claim, convergence statement, mesh
> recommendation, algorithm description, and reproduction instruction in this
> document refers to past code and must not be treated as a current result or
> instructions for an available API.** In particular, the saved IEM24 20 m /
> 52-node geometry is invalid; the old 43.9 s placeholder result is historical,
> not a present validated result. Solver status alone does not validate a lap.
> This document is retained as provenance, not current validation evidence.
> OCP-specific tests are outside default unittest discovery; extant planar
> parameter and geometry functionality remains tested separately.

> **Historical result (unsupported):** the old placeholder vehicle run was
> reported at 43.9 s under the historical configuration described below. That
> result, its mesh checks, and its route-verification claims are not current
> validated results. The old run was a **path-following** formulation, not a
> free racing-line optimum. The detailed record below is preserved for context
> only; see the prominent support warning above before interpreting any value.

> **Correction (fine-mesh investigation):** the earlier 36.294 s result and
> coarse-to-fine table are **historical, not validated lap times**. The
> previous formulation omitted the motor-to-pack power equality and divided
> battery charge by time rather than distance; its load transfer used body
> velocity derivatives instead of inertial acceleration. Result construction
> also attempted to unpack vector outputs into scalars, so a returned lap
> from that source revision could not have passed the current result path.
> Every number above was produced after those corrections.

## Historical formulation description — unavailable in the current tree

The retired `solvers/planar_ocp.py` implementation was described as solving a
minimum-time hot lap of the planar vehicle on the closed signed track. The lap
was transcribed in the **distance domain**: the independent variable was path
distance, so the mesh followed the track and the periodic boundary was exact.
This describes archived code only; it is not an available API or a current
solver instruction.

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

### Historical IPOPT status — invalid physics

Before the power/current and load-transfer corrections, a 20 m mesh with a
9000-iteration limit *reported* `Solve_Succeeded`. This status must not be
used as a physical validation, and the result-construction code in that
revision could not have produced a complete result object:

| Quantity | Value |
|---|---|
| Iterations | 5972 |
| Algebraic residual | 1.4e-07 |
| Periodicity residual | 0.0 |
| Lap time | 36.294 s |

This only measures IPOPT's old, incomplete mathematical system. The corrected
formulation includes electric conservation, spatial SOC, motor speed, inertial
load transfer, and independent post-solve feasibility checks. A plausible lap
time by itself does not demonstrate that those equations hold.

## The mesh must resolve the track curvature

This is the most important limitation, and it is a property of the discretization
rather than of the model.

The path is sampled at the mesh nodes only. A corner about 10 m long that falls
between two nodes is never seen by the mesh. The optimisation then legitimately
drives a much faster lap on the **smoothed** path that it was given.

Historical pre-correction lap times against mesh spacing; **not feasible
minimum-time results** under the corrected physical/electrical system:

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

- The lap time is for the **placeholder obfuscated vehicle**, not the real car,
  and it is a minimum-time optimum rather than a lap a driver would repeat.
- The lap time is a **path-following** result inside a 0.05 m corridor around
  the recorded centreline. It is not a free racing-line optimum: the 0.25 m
  corridor case is 2.1 s faster, and the repository holds no track-width data
  to do better. Treat 43.9 s as an upper bound on the free-line optimum.
- The 20 m and 10 m meshes are not usable for lap time: they miss 4.3 and
  0.17 rad of the lap's 6.28 rad of turning.
- The heading bound is insensitive (0.10 % across a factor of two) but the
  lateral corridor is not (4.8 % from 0.05 m to 0.25 m). Both are active at
  the reported operating point.
- The transcription is a trapezoid rule, which is second order.
- The steering uses parallel steering, because the Ackermann table is a zero
  array. The OCP applies one common front angle; a non-zero Ackermann table
  would need separate front wheel angles in this symbolic formulation.
- The symbolic brake command scales the full-pedal brake torque linearly.
  That matches the current constant pedal-ratio placeholder, but a non-constant
  pedal-ratio table would need symbolic interpolation and pressure limiting.
- The motor torque and power bounds are imposed at the peak rating, so the
  thermal duty cycle is not modelled. Regeneration is not modelled either:
  friction braking has no energy recovery.
- Pitch and roll are not states, so the model has no suspension degree of
  freedom and no roll-induced camber change.

## Running it

CasADi and IPOPT are optional. They are not on the default import path, so the
tests skip when CasADi is absent.

```bash
python3 -m pip install --target /tmp/casadi_pkgs casadi
PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_planar_ocp
```

## Refinement from a coarse solution did not work

The obvious next step is to solve a coarse mesh, interpolate the solution onto
a finer mesh, and solve again. It was implemented as a refinement ladder, and
it measured **worse than a cold start**. These numbers predate the electrical
and inertial corrections, so they demonstrate the *refinement strategy*
failing rather than any valid lap time:

| Target | Start | Iterations | Lap time | Algebraic residual | Status |
|---|---|---|---|---|---|
| 20 m | cold | 5972 | 36.294 s | 1.4e-07 | `Solve_Succeeded` |
| 10 m | converged 20 m | 9000 | 78.054 s | 1.4e+01 | `Maximum_Iterations_Exceeded` |
| 5 m | converged 20 m | 9000 | 67.092 s | 1.0e+02 | `Maximum_Iterations_Exceeded` |

The 10 m rung did not merely fail to improve; it drove the progress speed down
to 2.3 m/s and produced a **slower** lap than the coarse mesh it started from.
The interpolated loads and wheel speeds are consistent for the coarse mesh but
not for the finer one, and the solver follows that bad start downhill instead
of recovering.

`warm_start` therefore defaults to **off**. A cold start at 20 m is the only
configuration that reaches `Solve_Succeeded` here.

## Corrected-physics fine-mesh attempt

The electrical and inertial corrections were not cosmetic:

- The old pack current was an unconstrained decision variable. It could be
  zero while the four motors drew positive power. The corrected OCP enforces
  `terminal_voltage * pack_current == sum(motor electrical powers)` at every
  node. The numerical motor envelope also no longer counts motor efficiency
  twice when computing bus power.
- The previous SOC expression was a time derivative inside a distance-domain
  NLP; it now divides by the path progress speed.
- Lateral load transfer now uses `sum(Fy)/mass`, not `dv_y/dt`. A car in a
  steady turn can have zero `dv_y/dt` while still sustaining `r*v_x` lateral
  acceleration and substantial load transfer.
- The voltage-scaled motor speed limit is now an explicit per-wheel inequality.
- A successful IPOPT status is only reported as `converged` when the unscaled
  load/wheel/power residuals, spatial defects, inequalities, and periodicity
  all pass explicit post-solve thresholds. A failed solve still retains its
  traces and measured residuals for diagnosis.

The dense stored path has a maximum curvature of 0.16170 1/m. A uniform 2 m
mesh samples a peak of 0.16170 1/m with a maximum piecewise-linear curvature
reconstruction error of **0.00811 1/m**; a 1 m mesh samples 0.16166 1/m with
**0.00259 1/m**. A 20 m mesh misses the peak (0.12776 1/m) with an error of
0.2020 1/m.

### The coarse lap hid four radians of turning

The mesh samples the path only at its nodes. Integrating the **sampled**
curvature and comparing it with the path's actual heading change exposes how
much turning a mesh loses:

| Mesh | `integral(kappa ds)` | Path heading change | Error |
|---|---:|---:|---:|
| 20 m | -1.969 rad | -6.284 rad | **4.315 rad** |
| 10 m | -6.118 rad | -6.284 rad | 0.166 rad |
| 2 m | -6.290 rad | -6.284 rad | 0.0056 rad |
| 1 m | -6.287 rad | -6.284 rad | 0.0033 rad |

A 20 m transcription therefore misses **over four of the lap's 6.28 radians of
turning**. That is why its lap time looked fast: the vehicle was solving a
nearly straight track. `_quasi_steady_speed` also caps the seed speed at the
voltage-scaled motor limit, so it no longer seeds a speed the motor cannot
reach.

### A balanced low-speed seed

A cold start on a 1 to 2 m mesh does not converge, and projecting the coarse
solution onto a fine mesh made things worse. `solvers/planar_equilibrium.py`
instead builds a different kind of start.

At every node it solves eleven local unknowns - body lateral velocity,
steering, throttle, four vertical loads and four wheel speeds - against eleven
equations: the four load balances, the four quasi-steady wheel balances, the
battery power equality, and **zero body-x, body-y and yaw acceleration**. The
heading cancels the sideslip and the yaw rate follows the path, so the path
states have zero derivative and the seed is a genuine steady turn.

| Mesh | Nodes | Seed speed | Lap time of the seed | Max algebraic residual |
|---|---:|---:|---:|---:|
| 20 m | 52 | 10 m/s | 101.6 s | 4.2e-09 |
| 10 m | 103 | 10 m/s | 101.6 s | 2.2e-09 |
| 5 m | 204 | 10 m/s | 101.6 s | 4.4e-09 |
| 2 m | 509 | 10 m/s | 101.6 s | 9.3e-09 |
| 1 m | 1018 | 10 m/s | 101.6 s | 1.1e-08 |

The seed is algebraically exact to about 1e-8, so IPOPT starts from a feasible
mechanical and electrical point. Its remaining spatial defect is not zero -
curvature, sideslip and yaw rate still change between nodes - so the NLP must
close that gap before a lap can be trusted.

The seed speed is limited by friction, not by the solver. A 10 m/s turn at the
tightest 0.146 1/m corner needs 14.6 m/s^2 of lateral acceleration; 12 m/s
needs 21 m/s^2 and **no balanced equilibrium exists**, so `equilibrium_start`
raises rather than returning a silently infeasible start.

### The heading-error bound was too tight

The tightest corner needs about **0.095 rad** of sideslip-related heading error
to hold a balanced centred turn. The original `maximum_heading_error = 0.05`
therefore **excluded the balanced state at the tightest corners**. It is now
0.20 rad, which admits the balanced turn while still keeping the car pointed
along the path. The lateral-error bound stays at 0.05 m.

### Simultaneous throttle and brake was a hidden control

Inspection of an unscaled 1 m iterate found `throttle * brake` reaching
**0.519**, with 18.6 m of integrated overlap. Pressing both pedals together
produces a net axle torque that the driver cannot command and that acts as a
hidden, unmodelled yaw control, because the fixed split makes the front and
rear torques cancel differently when the inside wheels are light.

The solver now constrains `throttle * brake <= 0.001` at every node, applied to
the initial guess as well.

### The experiments so far are not convergence results

| Mesh | IPOPT iterations | Approximate lap | Max algebraic residual | Max spatial defect | Status |
|---|---:|---:|---:|---:|---|
| 10 m, cold/unscaled | 200 | 42.899 s | 1.73 | 0.00650 | iteration limit |
| 2 m, cold/unscaled | 1400 | 44.158 s | 0.196 | 0.000324 | iteration limit |
| 2 m, projected/scaled | 1200 | 90.878 s | 14461 | 2.431 | iteration limit |
| 1 m, cold/unscaled | 350 | 59.115 s | 1578 | 1.055 | iteration limit |
| 1 m, dual restart | 1600 | 50.405 s | 1151 | 1.592 | iteration limit |
| 1 m, cold, exclusive pedals | 2200 | 77.502 s | 21.30 | 1.021 | iteration limit |

The columns mix newtons, newton-metres and watts, so the algebraic residual is
a screening number only. Every run also reports a separate electrical-power
mismatch, inequality violation and curvature-closure error in its JSON output.
Neither algebraic projection nor constraint row scaling helped the cold
starts; both remain optional and are disabled by default.

`tools/solve_planar_mesh.py` checkpoints every rung and refuses to propagate an
infeasible intermediate result to a **finer** mesh. It can continue the *same*
mesh from an iteration-limited checkpoint with `--resume-same-mesh`, and new
checkpoints store IPOPT primal/dual multipliers for an exact-mesh restart.

## A converged 1 m lap

Three changes together turn the fine mesh from unconverged to solved: the
balanced equilibrium seed, the **exact sparse Hessian**, and a **steering rate
bound**.

| Configuration at 1 m | Iterations | Lap | Status |
|---|---:|---:|---|
| equilibrium seed, limited-memory Hessian | 3000 | 87.065 s | iteration limit |
| equilibrium seed, scaled rows, limited-memory | 1500 | 64.176 s | iteration limit |
| equilibrium seed, exact Hessian, no rate bound | 340 | 43.884 s | `Solve_Succeeded` |
| **equilibrium seed, exact Hessian, rate bound** | **296** | **43.907 s** | **`Solve_Succeeded`** |

The exact Hessian costs about three times more per iteration (0.86 s against
0.28 s) and needs about nine times fewer, so it also wins on total time. The
tire and the vertical-load coupling are strongly nonlinear, and a secant
Hessian of that coupling is poor. It is now the default.

### The steering had to be rate limited

The 1 m optimum with no rate bound **chatters between the steering stops**:

| 1 m solution | Steering range | Max step between nodes | Max rate |
|---|---:|---:|---:|
| no rate bound | -30.00 to +30.00 deg | 35.08 deg | **15.62 rad/s** |
| rate limited | -18.77 to +21.04 deg | 13.18 deg | 3.14 rad/s |

15.62 rad/s is 895 deg/s at the road wheel. The vehicle limit is 3.14 rad/s
(180 deg/s), so the unconstrained optimum was demanding **five times the rate
the rack can deliver**, and it did so by pinning the angle at both mechanical
stops. That is a discretisation artefact, not a driving technique.

The road-wheel rate is what the vehicle already declares as
`steering_maximum_rate`. The optimal control now imposes

```text
|steer[k+1] - steer[k]| <= steering_maximum_rate * travel_time(k)
```

with `travel_time` the trapezoidal time across the interval. Imposing it costs
only **0.023 s**. The bound binds (the reported rate is exactly 3.14 rad/s), so
the fastest corners are now rate limited rather than grip limited.

`maximum_steering_rate=inf` removes the rows entirely. That matters: leaving a
literal infinity in the constraint makes CASAdi emit an infinite Jacobian
entry and IPOPT returns `Invalid_Number_Detected`.

### The answer does not change with the mesh

| Mesh | Nodes | Iterations | Lap time | Curvature error | Geometric check |
|---|---:|---:|---:|---:|---|
| 5 m | 204 | - | 43.600 s | 0.03659 1/m | fails |
| 2 m | 509 | 224 | 43.802 s | 0.00821 1/m | fails |
| 1 m | 1018 | 296 | 43.907 s | 0.00264 1/m | **passes** |

The 2 m and 1 m meshes agree to **0.105 s, or 0.24 %**, and both reach
`Solve_Succeeded` from the same seed. That is the mesh-convergence evidence the
earlier coarse meshes could not provide, and it is why the minimum-time lap is
quoted as **43.9 s**.

Only the 1 m mesh also resolves the path: its maximum piecewise-linear
curvature reconstruction error is 0.00264 1/m against the 0.003 threshold. The
5 m and 2 m meshes fail that geometric check even where their lap times agree,
which is why the 1 m result is the one reported.

The two solutions describe the same lap, not just the same number: their speed
profiles differ by 0.176 m/s RMS with a 0.836 m/s worst case, and their minimum
and maximum speeds agree to 0.06 m/s and 0.01 m/s.

### Independent verification of the lap

The route below is re-evaluated **outside** the solver, with the NumPy
seven-DOF dynamics running its own vertical-load iteration, and against the
stored 0.5 m path data.

| Check | Measured | Threshold |
|---|---:|---:|
| Load equations | 1.2e-07 N | 1e-03 |
| Quasi-steady wheel balances | 2.1e-08 N*m | 1e-03 |
| Pack power equality | 2.4e-07 W | 1e-03 |
| Spatial dynamics defect | 1.2e-10 | 1e-04 |
| Periodicity of the mechanical states | 0.0 | 1e-04 |
| Sampled inequality violation | 9.7e-09 | 1e-04 |
| Simultaneous throttle and brake | 0.0010000 | 1e-03 |
| Steering rate against the rack limit | 3.14 rad/s | 3.14 |
| NumPy load replay against the solution | 5.8e-06 N | 1e-02 |
| NumPy wheel-torque balance | 8.2e-06 N*m | 1e-03 |
| Nodes the NumPy loop could not re-solve | 0 of 1018 | 0 |
| Wheels lifted off the ground | 0 of 1018 | 0 |
| Curvature reconstruction error | 0.00264 1/m | 0.003 |
| Curvature integral against heading change | 0.00331 rad | 0.01 |

Lap time **43.907 s** over 1016.70065 m, an average of 23.2 m/s (83.4 km/h).
The fastest wheel reaches 178.92 rad/s against a voltage-scaled motor limit of
179.73 rad/s, so the car is **field-weakening limited at the end of the
straight**, not grip limited there. The battery ends the lap at 0.910 state of
charge, so the pack is not a limiting factor at this length.

### The heading bound is insensitive, the lateral bound is not

Both path-following bounds are active in every solution, so both were swept.
They behave very differently.

Heading allowance, at a 0.05 m lateral bound and with the rate limit active:

| Maximum heading error | Lap time | Iterations | Status |
|---:|---:|---:|---|
| 0.10 rad | 43.952 s | 208 | `Solve_Succeeded` |
| 0.15 rad | 43.920 s | 1101 | `Solve_Succeeded` |
| 0.20 rad | 43.907 s | 296 | `Solve_Succeeded` |

Doubling the heading allowance from 0.10 to 0.20 rad changes the lap by
**0.045 s, or 0.10 %**. All three cases converged and passed the same
geometric and replay checks. The 0.10 rad case is still above the 0.095 rad
that the tightest balanced turn needs, so it stays physically admissible.

Lateral allowance, at a 0.20 rad heading bound, all converged and verified:

| Lateral corridor | Lap time | Iterations | Status |
|---:|---:|---:|---|
| 0.02 m | 44.604 s | 645 | `Solve_Succeeded` |
| 0.05 m | 43.907 s | 296 | `Solve_Succeeded` |
| 0.25 m | 41.800 s | 342 | `Solve_Succeeded` |

The lateral bound is **about 47 times more influential** than the heading
bound by relative lap-time change. Widening the corridor from 0.05 m to
0.25 m removes **2.11 s, or 4.8 %**, because a wider corridor lets the car
straighten the corners.

This is a genuine racing-line freedom rather than a solver artefact, and it
changes how the headline number must be read:

> **43.9 s is the minimum-time lap constrained to a 0.05 m corridor around the
> recorded centreline.** It is a *path-following* optimum, not a free
> racing-line optimum. A free line would be faster, and the 0.25 m column
> shows how much faster.

The corridor is not a track-width limit. The track is a recorded centreline
and this repository holds no usable width data, so the corridor is the only
control on how far the car may leave that centreline. A free-line optimum
needs the track edges, which is why it is listed first under what to do next.

## What to do next

1. Recover the **track width** and re-solve with the true corridor, or with a
   free lateral offset between the track edges. The lateral corridor is the
   single largest modelling influence measured here: 0.02 m to 0.25 m spans
   2.8 s, so this is worth more than any further solver tuning.
2. Replace the trapezoid rule with a higher-order collocation once the mesh is
   fine enough for it to matter; at 1 m the trapezoid error is already smaller
   than the physical modelling uncertainty.
3. Add the thermal, regen and pedal-ratio-curve models listed under known
   limitations before treating the result as a setup tool.
4. Re-run the 1 m solve against the real vehicle once the obfuscated
   placeholder parameters are replaced. The solver configuration should carry
   over unchanged; only `--equilibrium-speed` may need retuning, because the
   seed exists only where a balanced turn does.
