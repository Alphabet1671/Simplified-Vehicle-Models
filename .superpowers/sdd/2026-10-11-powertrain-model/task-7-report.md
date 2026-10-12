# Task P7 report

## Status

Completed the powertrain/drivetrain assembly draft. The nested electric powertrain and four-wheel drivetrain compile independently, are resolved by the compiler, and are recorded in the manifest. The assembly reduction preserves the legacy drive-force contract.

## Fixed-point completion

`solve_shaft_torque_fixed_point` now performs explicit loss-map iterations with a documented under-relaxation factor. Each result reports the final map residual, convergence flag, and iterations used; reaching `max_iterations` returns `converged=False` instead of silently capping work. Regen reports both torque and battery charge-power limits plus the fixed-point diagnostics. Overspeed operation is rejected at the reviewed 18,600 rpm motor limit rather than clamped into an invalid operating point.

## Verification

- Focused: `PYTHONPATH=src python3 -m unittest tests.test_powertrain -v` — 8 tests passed.
- Full: `PYTHONPATH=src python3 -m unittest discover -s tests -q` — 178 tests passed.

## Notes

The fixed-point result is intentionally diagnostic: callers can distinguish a converged loss update from a max-iteration result and inspect its residual.

## Correction report

The P7 review corrections are complete. Regenerative transmission and inverter losses now use the reviewed reverse-direction convention (reciprocal efficiencies), and the returned regen force is capped by the battery charge-power limit. The 10 m/s, 80% SOC regression checks the physical battery-power inequality.

Vehicle assembly reduction now provides `powertrain-and-drivetrain` and reaches `drive-force-limit` through the registered `reduce_powertrain_and_drivetrain` adapter. The manifest records the provider and adapter chain. The redundant unreachable nested-assembly branch was removed.

Fixed-point nonconvergence now recomputes the residual at the under-relaxed torque that is returned. Tests cover adapter usage, directional charge-power behavior, fixed-point diagnostics, and existing compatibility and overspeed paths.

Correction verification:

- Focused: `PYTHONPATH=src python3 -m unittest tests.test_powertrain -v` — 8 tests passed.
- Full: `PYTHONPATH=src python3 -m unittest discover -s tests -q` — 178 tests passed.
