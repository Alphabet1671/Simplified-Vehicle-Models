# P9 energy-coupled lap solve

The solver now exposes `solve_lap_with_energy` and `solve_stint` alongside the legacy `solve_lap`. Each energy result reports segment bus and shaft power, bus voltage, active longitudinal constraint, SOC trace, consumed/recovered/net energy, final SOC, minimum voltage, and an explicit outer-loop convergence flag and iteration count. `solve_stint` carries final SOC between laps and aggregates per-lap time and energy.

Regeneration is opt-in through `regen_enabled` (default `False`); disabled regeneration reports zero recovered energy. SOC is iterated through state-aware forward/backward sweeps, and traces use assembled motor/transmission operating points. The legacy lumped drivetrain path remains unchanged and continues to serve MIS regression validation.

Validation coverage is in `tests/test_energy_lap.py` (work-only behavior, outer-loop cap reporting, and multi-lap carry-over).