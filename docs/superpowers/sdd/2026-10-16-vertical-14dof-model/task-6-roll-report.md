# Task 6 roll + longitudinal force path — scoped report

Scope: the lateral (roll) and longitudinal (pitch) force paths of
`evaluate_vertical_dynamics`, plus their tests. Not covered: the inertance mass
matrix (untouched by this work), the steady-corner solver, reports/ledger, and
Task 7. Nothing is committed. No full-suite run was made by this task; the
focused set is listed under Verification.

Two phases, both in `src/simplified_vehicle_models/formulations/vertical_14dof.py`:

- **Phase 1 (roll, "Form B")** — replaced the RC-jacking roll moment with the
  lateral link-force path.
- **Phase 2 (longitudinal)** — added the missing pitch link-force moment and the
  unsprung longitudinal transfer, and reworked anti-dive/anti-squat.

## Sign conventions used throughout

heave positive **down**; roll positive **right side down** (`y` right); pitch
positive **nose up** (`x` forward); jounce positive **compression** (hub moves
UP relative to the body, `hub_down = z_body - jounce`). `corner_forces`
positive (compression) pushes the body UP and the hub DOWN.

## Phase 1 — lateral: Form B

`Y_i = bfy_i - mu_i*ay` is the body-side link force (the hub accelerates at
`ay`, whole-vehicle `ay = sum(bfy)/m_total`), so `sum(Y) = m_s*ay`.

- RC jacking is an **internal vertical pair** `D_i = -sign(y_i)*2*h_rc,i*Y_i/track`
  (body up, hub down) through the swing-arm geometry, added to **both** the body
  force `B` and the hub force `H`.
- The unsprung lateral inertia adds a **hub-only** pair
  `Du_i = -sign(y_i)*2*mu_i*ay*r_1/track`.
- `gen[1] = -sum(y*B) + sum(y*H) - h_s*sum(Y)`; the pre-existing `rc_jacking`
  term (`sum(bfy*h_rc)`) was **deleted**.

Why deleting `rc_jacking` is required: it is only the `h_rc` half of the pair.
Keeping it as a body-only moment gives `-(h_s - h_rc)*sum(Y)` and loses the
`h_rc` part of the transfer. With Form B the whole-vehicle identity
`sum(y_i*N_i) = -m_total*h_cg*ay` holds; `Du` is what supplies the unsprung
`r_1` share (`m_s*h_s + m_u*r_1 = m_total*h_cg`).

Verified numbers (30 m/s, sideslip 0.36 m/s, `ay = +4.554 m/s^2`):

- level body, lateral forces on: `gen[1] = -392.13 N*m = -m_total*h_cg*ay` to
  machine precision, roll acceleration `-33.2 rad/s^2` (the pre-Form-B `+h_rc*sum(Fy)`
  gave `+64.3`, the wrong sign);
- dynamic-relaxation steady state: `roll = -5.77e-3 rad`,
  `N = [1277.0, 950.4, 1427.4, 1081.5]` (left > right on both axles),
  `sum(y_i*N_i) = -417.004 = -m_total*h_cg*ay` exactly.

### LLTD analytic reference (valid)

The classic linear reference holds. Roll stiffness per axle is
`0.5*track^2` times the wheel rate (spring **plus** anti-roll bar) in series
with the tire vertical stiffness, all at the static design position:

| axle | wheel rate [N/m] | ARB [N/m] | tire rate [N/m] | K_phi [N*m/rad] |
|---|---|---|---|---|
| front | 78807 | 0 | 80418 | 30599 |
| rear | 61294 | 21015 | 80656 | 31319 |

Predicted front share `K_phi,f/(K_phi,f+K_phi,r) = 0.494`; the model's relaxed
steady state gives `326.60/(326.60+345.99) = 0.4856` — 0.0086 from the
prediction, tested with a 0.02 absolute tolerance. The compiled planar design
value `lateral_load_transfer_distribution = 0.42` is a different quantity and is
**not** the oracle. Note `axle.wheel_rate_spring` returns the preloaded spring
**force** [N], not a rate; the rate is its derivative (this cost some time to
untangle and is worth a rename or a docstring fix in a later task).

This is an independent oracle for the *total split* only. It cannot see the
jacking path at all: the RC-jacking contribution to the share is ~0.009
(the reviewer's figure) and the mutations move the measured share by at most that
much (measured 0.4856 production, 0.4836 with `Du` deleted, 0.4949 with `D` moved
to the body only, 0.4854 with the unsprung pitch couple flipped), while the
predicted reference itself sits 0.0086 from the measurement — so no tolerance
below ~0.015 is supportable and 0.005 is not. The tolerance was tightened
0.03 -> 0.02 and the jacking path is pinned instead by the absolute load oracle
below.

#### Absolute axle-transfer oracle (independent)

At a relaxed steady state each corner's steady hub balance is Newton's law with
the link force paths written geometrically, from the evaluator's **reported**
forces plus the geometry constants:

```
N_i  = mu_i*g + S_i + D_i + Du_i + JP_i
Y_i  = bfy_i - mu_i*ay
D_i  = -sign(y_i)*2*h_rc,i*Y_i/track
Du_i = -sign(y_i)*2*mu_i*ay*r_1/track
JP_i = [-1,-1,+1,+1]*0.5*m_u*ax*r_1/L
```

(`S_i` = reported corner force: spring + ARB + damper.) Subtracting the left/right
pair, and using that the static state is left/right symmetric, gives the
reviewer's axle form

`Delta N_axle = Delta B_axle + (2/track)*[h_rc*bfy_axle + (r_1 - h_rc)*mu_axle*ay]`

as an absolute statement. Measured at the relaxed lateral state
(`ay = 4.843`, `ax = -2.965`): per-corner residual `<= 2.6e-5 N` and per-axle
residual `<= 6e-7 N`, against jacking terms of 19-21 N (`D`, `Du`) and 10 N
(`JP`) per corner. It is absolute (nothing fitted) and tight, and it fails by
~20 N (four orders of magnitude) for each of the three mutations above.

## Phase 2 — longitudinal

### 1. Pitch link-force moment

`H_x,i = bfx_i - mu_i*ax`, so `sum(H_x) = m_s*ax + drag`, and
`gen[2] += h_s*sum(H_x)`. The tire forces act at the contact patch and the hub
inertia `mu_i*ax` acts at the unsprung CG height `r_1`; the body-side link
moment `h_s*sum(H_x)` plus the unsprung couple `jacking_pitch` reproduce that
ground-level moment balance, giving the whole-vehicle identity (verified to
4.5e-13 at 30 m/s)

`sum(x_i*N_i) = -m_total*h_cg*ax - h_s*drag + aero_arm*downforce + sum(x_i*mu_i*g)`.

Sign: a forward force below the CG is nose-up, so acceleration squats (`+pitch`)
and braking dives (`-pitch`). Drag acts at the sprung CG, only 21 mm above the
total CG, so its own moment is small but not zero.

Consequence: the pre-Phase-2 pitch equation reduced to
`sum(x*N) = sum(x*m_u*g) + downforce*aero_arm`, which contains **no**
longitudinal-force term at all, so the old code produced essentially no
braking load transfer. That is the defect this fixes.

### 2. Unsprung longitudinal transfer (hub-only)

`jacking_pitch = [-1,-1,+1,+1] * 0.5 * m_u * ax * r_1 / L` — hub-only, half to
each wheel, front negative, rear positive. Derivation: the unsprung share of the
longitudinal load transfer must be `m_u*r_1*ax` of moment, and the front pair /
rear pair cancel in total force while giving `sum(x*jacking_pitch) = -m_u*r_1*ax`.
With it, the difference identity between two states at the same speed is exact:
`d(sum(x*N)) = -m_total*h_cg*d(ax)`.

Verified at 30 m/s with anti zeroed, solved to a steady state at
`ax = -17.675 m/s^2` (coasting `ax = -2.965`, i.e. exactly 1.5 g of braking):

- front axle load gain `825.101 N` against the analytic
  `m_total*1.5*g*h_cg/L = 825.100 N` (the brief's 825.1), rear loss equal;
- `d(sum(x*N)) = 1266.5294 N*m` against `-m_total*h_cg*d(ax) = 1266.5294 N*m`.

### 3. Ruling: anti-dive/anti-squat sign and scale

**Ruling: the real-car "anti" percentage is the share of the whole-vehicle
longitudinal load transfer `m*ax*h_cg/L` that the suspension link reacts, so the
per-axle coefficient is `k = anti*(h_cg/L)/share`, with `share` the axle's brake
bias under braking and its driven-axle share under drive; the body-side force is
`-k*bfx` at the front and `+k*bfx` at the rear. This is adopted because it is the
definition that makes the front link force equal `anti * m*ax*h_cg/L` while the
realized axle force split equals the design share (at speed drag contributes to
`m*a` without an axle force and the load-dependent tire split moves away from the
bias, so the equality is then approximate; noted at the scale line in the code),
and it reproduces the check
`anti_front_force / front_tire_gain = 0.542 = anti_brake`.
Cost if wrong: the dive/squat share between the springs and the links is wrong
proportionally to the error in the percentage reference; the tire load transfer,
the total vertical balance and the vehicle frequencies are unaffected (the pair
is internal), so a wrong scale is a body-attitude error, not a handling-balance
error.** Derivation: a thrust line at angle `theta` reacts `F_x*tan(theta)` at the
axle; 100% anti means the link reacts the whole front gain `m*ax*h/L`, so
`tan(theta) = anti*h/(L*phi_ref)` and the force is
`F_x,axle*tan(theta) = anti*(h_cg/L)*F_x,axle/phi_ref = anti*(h_cg/L)*F_x,total`.
This is the standard anti-dive definition; the `1/phi_ref` is what converts the
axle force to the total-force reference the percentage is quoted against.

The compiled values are front `anti_brake = 0.542`, `anti_drive = 0.32`; rear
`0.574` / `0.55`; `brake_bias = 0.65` (front); rear-only drive
(`front_torque_fraction = 0`), so the drive shares are `(0.0, 1.0)`.

At the -1.5 g stop the new front anti force is `+227.89 N` per wheel (body up,
anti-dive), total `455.8 N` = 0.552 of the 825.1 N front gain (the 54.2%
percentage, within the accuracy of the hand check). The old expression
`bfx*anti*(h_s/L)` gave `-158.56 N` per front wheel, i.e. `317.1 N` **downward**
on the body: pro-dive, the wrong sign.

- **Per-wheel branch selection**: the old `np.mean(bfx) >= 0` axle test is
  replaced by a per-wheel `bfx[i] >= 0` test, so a split axle (one wheel braking,
  one driving) no longer flips the whole axle.
- **Zero reference share**: an axle that carries none of the reference force
  (undriven front axle under traction) gets zero anti force instead of an
  unbounded `1/0`. Documented in the code.
- The reference shares come from design constants (`brake_bias`,
  `front_torque_fraction` via `axle_drive_shares`), not from the instantaneous
  force split, so the force stays bounded and continuous through the
  brake/drive transition.

### 4. Invariance and share checks

At the same `ax` with anti on vs off, the tire loads agree to `4.5e-13 N`
(measured; the test now asserts `1e-6 N`, previously `1 N`) and the front gain is
825.101 N either way (the pair is internal). The front spring force drops from
2663.2 to 2207.5 N and the rear spring force rises from 1562.8 to 2019.5 N (the
pair pushes the rear body down), and the dive drops from -0.01078 to -0.00647
rad, which is what anti-dive is for.

The anti force itself is now read out of the **evaluator**, not recomputed: the
production vehicle and an anti-zeroed copy are evaluated at the same state and
tire states, so `gen_zeroed - gen_production` isolates the pair (heave, roll and
pitch cancel to 5e-15). At the -1.5 g stop that pair is `+227.89 N`/wheel at the
front and `-228.37 N`/wheel at the rear, matching
`anti_frac*(h_cg/L)*bfx_i/share` to `1e-6 N`; a mutant that uses `h_s` instead of
`h_cg` fails that check (the two heights are 0.300 vs 0.321 m, a 7% difference).

## Tests

An **independent oracle** is a check whose comparison quantity comes from outside
the evaluator (closed-form reference, a second integration scheme, an externally
derived geometry rule). A **self-consistency/regression check** reuses the
evaluator's own reported forces and decomposition, so it pins the assembly against
future edits but is *not* evidence that the physics is right. The two are marked
as such below.

`tests/test_vertical_14dof.py` — `LateralLoadTransferTests`:

- *independent*: +`ay` gives a negative roll acceleration at the static state
  (sign), and the RK4 dynamic relaxation (no Jacobian, no generalized-force solve)
  reaches a fixed point with left > right on both axles;
- *independent*: LLTD against the roll-stiffness distribution (total split only,
  loose — see above);
- *independent*: the new absolute axle-transfer oracle
  `test_axle_load_transfer_matches_the_reported_forces_and_link_geometry`
  (per-corner and per-axle, `1e-3 N`);
- *self-consistency*: `gen[1] = -m_total*h_cg*ay` at the level body (algebraic
  identity of the documented link sum and `m_s*h_s + m_u*r_1 = m_total*h_cg`);
- *self-consistency*: Newton-Euler force path with `Du` (`gen` reassembled with
  the same body/hub split, and `M0 @ a = Q`);
- *invariance/regression (not a magnitude oracle)*: `gen[0..2]` invariant to
  `h_rc` (doubling it changes only the jounce rows);
- *independent*: mirror symmetry with nonzero `Y`, `D` and `Du` (`Y` odd, `D`/`Du`
  even).

`tests/test_vertical_14dof.py` — `LongitudinalLoadTransferTests`:

- *independent*: the -1.5 g braking front-axle gain `825.101 N` against the
  closed-form `m_total*1.5*g*h_cg/L` (the brief's 825.1) and the `m*ax*h/L` figure;
- *regression anchor*: the `d(sum(x*N)) = -m_total*h_cg*d(ax)` identity between
  two states at one speed;
- *independent*: the anti pair read out of the evaluator against the documented
  `anti*(h_cg/L)*F_x,axle/share` rule (braking and driving branches, signs, the
  undriven-axle zero share, `h_cg` vs `h_s`);
- *independent*: anti on/off total-load invariance (`1e-6 N` now, the solve is
  exact to 4.5e-13), the front/rear spring-force signs and the dive reduction;
- *independent*: `+Fx` squats and `-Fx` dives, isolated from the drag/aero moments;
- *independent*: left/right mirror of the braking rates and the unsprung pitch pair.

Elsewhere in the same file, and in `tests/test_vertical_equilibrium.py`: *independent*
are the finite-difference linearisation stability (eigenvalues of a numerical
Jacobian, a different route from the algebraic assembly), the mass-block
SPD/structure checks, the perturbation sign checks, the static-equilibrium
stationarity and the mirror-symmetry checks. The whole-vehicle moment identities
(`sum(y*N) = -m_total*h_cg*ay`, `sum(x*N)` above) *follow from* the same force
path, so they are regression anchors for the internal split, not independent
validation of it — that is what the axle-transfer oracle supplies. *Self-consistency*
are `M0 @ a = Q` in `VerticalMassMatrixTests` and the `gen` reassembly (both marked
in the test bodies).

Updated because they encoded the old force path (assertions tightened, none
weakened):

- `VerticalMassMatrixTests.test_m0_times_acceleration_equals_independent_newton_euler_generalized_force`
  now includes the anti rule, the pitch link moment and the unsprung pitch pair;
- `VerticalMassMatrixTests.test_b_positive_speed_equilibrium_is_stationary_in_all_seven_coordinates`
  now refines the state on the evaluator's generalized forces. The old assertion
  (the static solver's state is stationary at speed) is **false with the correct
  physics**, because the static solver has no longitudinal force;
- `tests/test_vertical_equilibrium.py` uses the same refinement and adds the
  static-solver-vs-evaluator difference.

### Hardening pass (independent-review findings)

- `test_anti_sign_scale_and_per_wheel_selection` no longer compares the helper
  `anti_forces()` with an inline copy of the same rule: it isolates the pair from
  the **evaluator** (`_anti_pair`) and checks sign, scale, the `h_cg`-not-`h_s`
  ratio, the driving branch and the rear spring-force sign.
- LLTD tolerance tightened 0.03 -> 0.02 (the measured-vs-predicted gap is 0.0086,
  so 0.005 is unsupportable) and the new absolute axle-transfer oracle added.
- Self-consistency checks are now labelled as such in the test bodies, and the
  verification claims in this report distinguish them from the independent oracles.
- `test_anti_is_an_internal_pair_...`: load tolerance 1 N -> `1e-6 N` and the
  front-transfer bound 1 N -> 0.01 N (measured residuals 4.5e-13 and 3.9e-4).
- `test_evaluator_vertical_rates_vanish_at_nonzero_speed_equilibrium` now asserts
  the **signed** front-axle change (the front loses 0.83 N at 8 m/s, i.e. the load
  moves rearward) instead of hiding the sign behind `abs()`; the docstring prose
  was corrected to match.

New shared test helpers: `solve_vertical_steady_state` (Newton on
`generalized_forces`), `relax_vertical` (RK4 relaxation), `braking_state`
(slip-ratio continuation with load updates), `anti_forces`,
`longitudinal_jacking_pitch`, `roll_stiffness_front_share`, and `_anti_pair`
(the evaluator-difference isolation; `anti_forces` remains for the two Newton-
Euler assembly tests and is no longer compared with a copy of itself).

## Verification

Focused runs only (the full suite is run by the parent task, not here):

```
PYTHONPATH=src python3 -m unittest tests.test_vertical_14dof tests.test_vertical_equilibrium
# Ran 55 tests ... OK   (~32 s)   [54 before this hardening; +1 new oracle test]

PYTHONPATH=src python3 -m unittest \
  tests.test_vertical_14dof.LateralLoadTransferTests \
  tests.test_vertical_14dof.LongitudinalLoadTransferTests
# Ran 12 tests ... OK   (~15 s)
```

Mutation check (`src` and `tests` copied to `/tmp/mut_h<n>/`, run there; the
baseline is 55 tests, all OK):

| mutant | result |
|---|---|
| drop `Du` (`jacking_unsprung`) from the hub force | 5 failures, incl. the new oracle by 20.8 N |
| move `D` (`jacking`) to the body only | 7 failures, incl. the new oracle by 19.4 N |
| flip the unsprung pitch couple `jacking_pitch` | 6 failures, incl. the new oracle by 20.5 N |
| anti ratio uses `h_s` instead of `h_cg` | 3 failures, incl. `test_anti_sign_scale_and_per_wheel_selection` |

The LLTD test alone catches none of the first three (its measured share moves by
only 0.0020, 0.0093 and 0.0002 against a 0.02 tolerance), which is exactly why the
absolute oracle was added.

`git diff --check` is clean. The braking bisection dominates the runtime.

## Caveats and open items

- **Static solver has no longitudinal tire force.** It ignores drag at speed, so
  its solution is not the evaluator's steady state; the two differ by a small
  pitch and load redistribution (at 8 m/s: pitch 0.001218 -> 0.001247 rad, front
  axle -0.83 N, total load unchanged). The front-axle change is **negative**: drag
  at the sprung CG (21 mm above the total CG) pitches the body nose-up and moves a
  little load rearward. Tests that used it as an oracle were
  updated, not weakened. A solver-side fix is a separate task.
- **One `ay` for all four hubs.** The lateral link forces use the whole-vehicle
  `ay`; yaw-acceleration lever terms (`psi_ddot*x_i`) are ignored.
- **Constant `h_s`, `r_1`, `h_rc`** at small angle; the RC lateral-position
  spline and the RC dependence on jounce are ignored. `h_rc` is read as height
  above ground (the LUT calls it the static RC height, 41/57 mm). The RC query
  is `|roll|`, so `h_rc` is even under mirroring.
- **Unsprung CG height is the tire free radius `r_1` only** for all four corners;
  the unsprung share of the longitudinal couple is split half per wheel rather
  than by axle unsprung mass. The total couple (and therefore the total transfer,
  825.101 N) is exact either way; the split is a modelling choice.
- **Anti sign and scale** rest on the ruling above plus the real-car anti-geometry
  convention (braking and drive values are distinct). No independent measurement of
  the thrust-line angles was available.
- **Anti-dive does not change the tire load transfer** (internal pair). If a
  later task expects geometry to alter LLTD or total transfer, that expectation
  is inconsistent with this force path and needs its own review.
- **Front anti-roll bar is zero** and the roll stiffness ratio (0.494) is very
  close to 50/50 by accident of the current rates; a change to the wheel rates
  will move the LLTD reference.
- The inertance mass matrix and its report are untouched. `inertance = 0` in the
  compiled model, so `mass_matrix` is `None` on all production paths.
- `sum(x_i*N_i) = -m_total*h_cg*ax` holds only as a **difference between two
  states at the same speed**; the absolute value carries the unsprung weight
  moment and the aero arm (e.g. +60.6 vs +18.2 N*m at 8 m/s). Do not use the
  absolute form as an oracle without those terms.
