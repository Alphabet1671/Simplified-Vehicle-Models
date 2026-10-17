# Task 6 mass + sprung-CG arm correction — scoped report

Scope: only the physical mass block, the inerter term, and sprung-CG arms. Not covered: roll-centre jacking, the steady-corner solver, LLTD, and Task 7. Task 6 is still incomplete and nothing is committed.

## Changes

- `vertical_14dof.py`
  - `sprung_wheel_positions(vehicle)` gives wheel (x, y) relative to the **sprung** CG. It is used for `z_body`, `z_dot`, `local_body_accel`, and the body-force moments. Yaw and contact kinematics keep the total-CG `wheel_positions`.
  - `vertical_mass_block(vehicle, jounce)` returns the physical 7×7 block for `q = [heave (+down), roll (right-down), pitch (nose-up), jounce ×4 (+compression)]`:
    `M = diag(m_s, I_roll, I_pitch, 0×4) + Σ m_ui c_i c_iᵀ + diag(0,0,0, b_i/MR_i²)`, with `c_i = [1, y_i, -x_sprung_i, -e_i]`.
  - The inerter is an ideal hub-to-chassis-point device with relative velocity `jounce_rate/MR`, per the `progress.md` ruling. Its kinetic energy `½ (b/MR²) j̇²` therefore adds only to the jounce diagonals and couples nothing to the body.
  - The evaluator builds the generalized force `Q` by virtual work. It collects body upward corner forces `B_i = spring + damper + anti` (plus aero and the existing RC-jacking term) and hub downward forces `H_i = m_ui g + B_i − tire_load`. It then solves `block · a = Q_eff` for the seven accelerated coordinates.
  - `Q = [Σ(m_s g + downforce − B) + ΣH, −Σ y B + jacking + Σ y H, Σ x B − downforce·arm − Σ x H, −H_i]`.
  - Variable motion ratio is supported through the Lagrange term. With `b_eff(j) = b/MR(j)²`, the forcing is `Q_j − ½ b_eff'(j) j̇²`, and the derivative comes from `MonotoneSpline.derivative` with no SciPy. With constant MR the term is exactly zero.
- `vertical_equilibrium.py`: the static solver uses `sprung_wheel_positions` for `wheel_vertical`, so static and evaluator coordinates are consistent.

## `VerticalEvaluation` semantics

- `rates` always holds the physical, **already-solved** derivatives (accelerations for `v_z`, `roll_rate`, `pitch_rate`, and the jounce rates). It is never a forcing vector.
- New field `forcing` (21): `mass_matrix @ rates = forcing`. New field `generalized_forces` (7): `Q` before the inertance velocity term.
- `mass_matrix is None` (all inertance 0): `forcing == rates`. `M0` was already solved inside the evaluator. `RateSystem(out.forcing).solve()` returns `rates`.
- Otherwise `mass_matrix` is the 21×21 mechanical block: identity on every row except indices 2, 3, 4, 17–20, which hold the 7×7 physical block `M0 + diag(b/MR²)`. `forcing` equals `rates` on all other rows, and on the seven vertical rows it holds the physical generalized force (N, N·m). Task 7 should use `RateSystem(out.forcing, out.mass_matrix)` and embed both by the state-name index map. It must not solve `rates` a second time.
- The zero-inertance explicit solve is identical in physics to the matrix case. It also differs from the old uncoupled formulas because the unsprung `m_u c cᵀ` coupling is now included, as the earlier report's derivation required.

## Tests (added in `tests/test_vertical_14dof.py::VerticalMassMatrixTests`)

- **Block structure**: the baseline block equals `diag + Σ m_u c cᵀ` and is SPD. The jounce diagonal is `m_u` at b=0.
- **Positive b**: the block stays SPD and symmetric. The difference from b=0 is exactly diag(b/MR²) on the jounce diagonals. The body rows and columns are untouched.
- **b=0 baseline**: `mass_matrix is None`, `forcing == rates`, and the `RateSystem` solve is exact.
- **`M0·a = Q`**: the test builds separate sprung-body and hub Newton–Euler equations from reported forces, with anti drive and brake cases and aero, and checks each against the solved accelerations. It then checks the virtual-work `Q` against `M0 a`. This runs with nonzero accelerations in all seven coordinates and no lateral force, so it avoids the unreviewed RC-jacking term.
- **`RateSystem` coherence with b>0**: `RateSystem(forcing, mass_matrix).solve() == rates`, non-vertical rows are untouched, and `block @ a == generalized_forces`.
- **Variable MR**: with a varying front MR spline, `forcing` on the jounce rows equals `Q − ½ b_eff'(j) j̇²` using a finite-difference `b_eff'`. Rear rows with constant MR carry no velocity term, and the `RateSystem` solve recovers `rates`.
- **Mirror symmetry**: the mass block transforms correctly under left/right reflection. The evaluator `mass_matrix` and rates mirror at nonzero b.
- **Static consistency**: with b>0 the static equilibrium at 0 m/s, and at 12 m/s with wheel speeds aligned to the loaded radii (zero tire force), gives zero `Q` and vertical rates in all seven coordinates to 1e-6.
- Existing tests were updated from total-CG arms to `sprung_wheel_positions`.

## Verification

- `PYTHONPATH=src python3 -m unittest tests.test_vertical_14dof tests.test_vertical_equilibrium -q`: 32 tests OK.
- `git diff --check`: clean. Untracked `tests/test_vertical_14dof.py` has no trailing whitespace.
- Full suite: `PYTHONPATH=src python3 -m unittest discover -s tests -q` ran 910 tests OK (296 s) on the mass snapshot before the final static-test tightening, which changed only a test. The parent's bash-1069 run is the authoritative one.

## Caveats and open items (not in scope)

- The RC-jacking sign and magnitude, the missing `−h_s ΣY` roll term, and the steady-corner/LLTD solver remain open. The `M0·a = Q` test deliberately uses straight-line states.
- Anti force still uses `h_s/L` as specified. The pitch/heave coupling between body and hub from anti is now consistent (paired ±).
- Inertance 0 is the default proposed value, so the production XML never builds a matrix. Positive-b behaviour is exercised only through test-built vehicles.
