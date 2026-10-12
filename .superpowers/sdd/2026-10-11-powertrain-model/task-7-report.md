# Task P7 report

## Status

Completed the powertrain/drivetrain assembly draft. The nested electric powertrain and four-wheel drivetrain compile independently, are resolved by the compiler, and are recorded in the manifest. The assembly reduction preserves the legacy drive-force contract.

## Fixed-point completion

`solve_shaft_torque_fixed_point` now performs explicit loss-map iterations with a documented under-relaxation factor. Each result reports the final map residual, convergence flag, and iterations used; reaching `max_iterations` returns `converged=False` instead of silently capping work. Regen reports both torque and battery charge-power limits plus the fixed-point diagnostics. Overspeed operation is rejected at the reviewed 18,600 rpm motor limit rather than clamped into an invalid operating point.

## Verification

- Focused: `PYTHONPATH=src python3 -m unittest tests.test_powertrain tests.test_cross_fidelity -q` — 29 tests passed.
- Full: `PYTHONPATH=src python3 -m unittest discover -s tests -q` — 183 tests passed.

The final runtime correction feeds battery terminal-voltage sag back into the motor/loss iteration, reports the converged terminal voltage and residual, and computes recovered bus power after reverse-direction drivetrain and inverter losses. The full nested runtime and legacy lumped reduction are compared at the same 10 m/s, 80% SOC operating point for force, bus power, active constraint, storage current, and SOC rate.

The final correction preserves the underlying motor/storage active constraint in `BrakingResult` while reporting friction allocation separately. Standstill and forward motoring tests verify positive force, bus power, and charging-current direction.

## Notes

The fixed-point result is intentionally diagnostic: callers can distinguish a converged loss update from a max-iteration result and inspect its residual.

## Correction report

The P7 review corrections are complete. Regenerative transmission and inverter losses now use the reviewed reverse-direction convention (reciprocal efficiencies), and the returned regen force is capped by the battery charge-power limit. The 10 m/s, 80% SOC regression checks the physical battery-power inequality.

Vehicle assembly reduction now provides `powertrain-and-drivetrain` and reaches `drive-force-limit` through the registered `reduce_powertrain_and_drivetrain` adapter. The manifest records the provider and adapter chain. The redundant unreachable nested-assembly branch was removed.

Fixed-point nonconvergence now recomputes the residual at the under-relaxed torque that is returned. The assembled provider now retains nested motor, storage, and transmission runtime objects while remaining equal to the legacy contract. Motoring uses the coupled fixed-point/storage solve and reports bus current and SOC rate; regenerative braking is exposed through `resolve_braking` with motor and pack charge limits. Tests cover adapter usage, directional charge-power behavior, fixed-point diagnostics, compatibility, overspeed, braking, and legacy/full behavioral compatibility.

Correction verification:

- Focused: `PYTHONPATH=src python3 -m unittest tests.test_powertrain tests.test_cross_fidelity -q` — 29 tests passed.
- Full: `PYTHONPATH=src python3 -m unittest discover -s tests -q` — 183 tests passed.

The final runtime correction feeds battery terminal-voltage sag back into the motor/loss iteration, reports the converged terminal voltage and residual, and computes recovered bus power after reverse-direction drivetrain and inverter losses. The full nested runtime and legacy lumped reduction are compared at the same 10 m/s, 80% SOC operating point for force, bus power, active constraint, storage current, and SOC rate.

The final correction preserves the underlying motor/storage active constraint in `BrakingResult` while reporting friction allocation separately. Standstill and forward motoring tests verify positive force, bus power, and charging-current direction.
