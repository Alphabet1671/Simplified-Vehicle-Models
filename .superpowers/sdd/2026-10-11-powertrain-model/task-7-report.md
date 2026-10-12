# Task P7 report

## Status

Completed the powertrain/drivetrain assembly draft. The nested electric powertrain and four-wheel drivetrain compile independently, are resolved by the compiler, and are recorded in the manifest. The assembly reduction preserves the legacy drive-force contract.

## Fixed-point completion

`solve_shaft_torque_fixed_point` now performs explicit loss-map iterations with a documented under-relaxation factor. Each result reports the final map residual, convergence flag, and iterations used; reaching `max_iterations` returns `converged=False` instead of silently capping work. Regen reports both torque and battery charge-power limits plus the fixed-point diagnostics. Overspeed operation is rejected at the reviewed 18,600 rpm motor limit rather than clamped into an invalid operating point.

## Verification

- Focused: `PYTHONPATH=src python3 -m unittest tests.test_powertrain -v` — 5 tests passed.
- Full: `PYTHONPATH=src python3 -m unittest discover -s tests -v` — 175 tests passed.

## Notes

The fixed-point result is intentionally diagnostic: callers can distinguish a converged loss update from a max-iteration result and inspect its residual.
