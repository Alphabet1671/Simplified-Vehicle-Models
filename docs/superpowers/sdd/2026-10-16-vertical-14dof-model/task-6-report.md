# Task 6 report — vertical 14DoF equations of motion and equilibrium

## Status

Implemented, pending the final independent review recorded in the ledger. Nothing is committed yet; the commit is made after that review. Scoped detail lives in the sibling reports:

| Report | Scope |
|---|---|
| `task-6-mass-report.md` | Physical 7×7 sprung/unsprung mass block, inerter `b/MR²`, `forcing`/`mass_matrix` semantics |
| `task-6-roll-report.md` | Lateral (roll) and longitudinal (pitch, anti) force closure, rulings, test hardening |
| `task-6-corner-design.md`, `task-6-corner-report.md` | `solve_steady_corner` design and results |
| `task-6-acceptance-report.md` | Modal, inertance, energy and planar-reduction gates |

## What was built

- `evaluate_vertical_dynamics` returns the solved physical `rates`, plus `forcing` (21) and `generalized_forces` (7). With zero inertance `mass_matrix is None` and `forcing == rates`. With positive inertance `mass_matrix` is a 21×21 identity with the 7×7 physical block embedded, and `RateSystem(out.forcing, out.mass_matrix).solve()` reproduces `rates`.
- Sign conventions: heave positive down, SAE roll positive right-side-down, pitch positive nose-up, jounce positive in compression, `hub_down = z_body − jounce`. Vertical, roll and pitch kinematics use sprung-CG arms; horizontal and yaw dynamics use total-CG arms.
- Lateral closure ("Form B"): `Y = bfy − um·ay`; the roll-centre jacking term `D` is applied to both body and hub, the unsprung term `Du` to the hub only; `gen[1] = −Σy·B + Σy·H − h_s·ΣY`. Positive `ay` gives negative roll and heavier left tires, and `Σy·N = −m_total·h_cg·ay`.
- Longitudinal closure: `gen[2] += h_s·ΣH_x`, a hub-only unsprung pitch couple, a corrected front anti sign, and anti force `anti·(h_cg/L)·Fx_axle/share` selected per wheel for drive or brake.
- `solve_steady_corner(vehicle, speed, lateral_acceleration)` is a 14-unknown scaled Newton solver with 0.1 g continuation. It raises `ValueError` below 3 m/s and `RuntimeError` on non-convergence, wheel lift, limits or out-of-range counters.

## Verification evidence

- Full suite: `PYTHONPATH=src python3 -m unittest discover -s tests -q` — **951 tests OK in 264.5 s, exit 0** (log: `/tmp/full_suite_task6.log`).
- Focused: `tests.test_vertical_14dof tests.test_vertical_equilibrium tests.test_vertical_14dof_compile tests.test_tire_vertical` — 100 tests OK. `tests.test_vertical_acceptance` — 17 tests OK. `git diff --check` clean.
- Independent oracles: reference-vehicle roll gradient 0.2905–0.2923 deg/g (oracle 0.289 ± 10 %) and front LLTD 0.477–0.481 (oracle 0.473 ± 0.02), with closed-form 0.28968 deg/g and 0.47399; braking front-axle transfer 825.10 N against the analytic 825.10 N; RK4 relaxation to a steady corner; finite-difference stability (max Re λ ≈ −4.86 s⁻¹); mirror symmetry; energy drift 2.3e-8 and 1.2e-8 against a 1e-6 gate; body-fixed hop set 16.63/17.61/17.61/17.82 Hz.
- Self-consistency checks (regression anchors, not independent validation): the level-body `gen[1]` identity, `block @ a == generalized_forces`, and Newton-then-residual equilibrium refinement.
- Mutation checks: the closure review caught 8 of 8 mutants, the hardening round extended them, and the acceptance and corner agents each ran their own.
- Production steady corner (compliant tires, measured roll-centre map; **not** acceptance values): roll 0.66–0.67 deg/g and front LLTD 0.486–0.489 at 12/15/25 m/s. Maximum feasible lateral acceleration is about 1.57 g at 12 m/s, 1.67 g at 15 m/s and 2.09 g at 25 m/s; a 3 g target raises `RuntimeError`.

## Ruled deviations and disclosures

See the ledger for the full `Ruling: … — … — cost if wrong` entries.

1. **Pitch mode outside the plan band.** The undamped pitch mode is 7.30 Hz against the plan's 3.4–4.7 Hz. The mass and stiffness matrices match the closed form to finite-difference noise, so this is not a model bug. The plan band assumes a quarter-car reference, and the pitch/roll inertias (radii 0.386 m and 0.193 m) are well below the geometry-implied values. The test asserts the pitch mode is outside the band. The inertia provenance should be confirmed with the data owner. Nothing was tuned.
2. **Anti scaling** uses `h_cg/L` with the design share instead of the brief's literal `h_s/L`. Total tire load transfer is unaffected (the pair is internal).
3. **Reference oracle tires** are stiffened by 1/300 on `r_2`, `r_3` instead of the brief's ×1e4, because larger stiffening makes the loaded-radius law non-monotone at low load.
4. **Planar reduction** passes at 1e-2 only with aero zeroed. With aero on, the vertical model books the drag couple `D·(h_s − h_cg)/L` that the planar quasi-static rule omits (gap `D·h_s/L`). Planar files stay unchanged per the plan.
5. **Inertance heave check** is gated at b = 0.3/0.5 kg, where the heave shift is below 0.2 %. At the brief's own 3/5 kg example it is 0.86 %, so the brief's example and tolerance are inconsistent.
6. **Gravity** is 9.80665 in the vertical modules and 9.81 in the planar conventions (about 0.43 N front-axle offset).
7. **Static solver at speed** has no longitudinal force, so its state is not stationary under the evaluator at speed > 0. The straight steady-corner solve is compared with it at 1e-6 only drag-free.
8. **Approximations** in the closure: one common `ay`/`ax` for all hubs, constant `h_s`, `r_1` and roll-centre height, no roll-centre lateral offset, and the unsprung pitch couple split half per wheel.
9. **Open, unexplained:** the production roll gradient is 16 % above the naive spring-in-series-with-tire estimate. It is not gated and nothing was tuned; a roll-moment budget at the solution is the natural follow-up.

## Task 7 handoff

Task 7 consumes `RateSystem(out.forcing, out.mass_matrix)`, embedding the 21×21 mechanical block into the 38×38 plant by state-name index map, and must not re-solve `rates`.
