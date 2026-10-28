# tests

The unit and regression suite, run with `unittest`.

## Files

Test modules named `test_<topic>.py` each cover one topic and run under `unittest`. `test_gui_launchers.py` exercises direct venv launch from unrelated working directories. Maneuver result tests verify canonical RunRecord handoff to animator readers; video encoding is not a maneuver responsibility.

`test_evolution_benchmark.py` checks that a short planar rollout reports distance, RHS evaluations, and wall time per simulated second.

`test_evolution_cosim.py` checks fixed-rate controller stepping, state layouts, TV torque injection, deadlines and tick-refinement agreement.

`test_tire_gui_plotting.py` checks independent model display sweeps across loads and segments, fixed secondary slip, and input immutability.

`test_tire_gui_belt_speed.py` covers immutable measured-speed provenance, wheel/slip recovery guards and verified overrides. `test_tire_gui_ramps.py` covers repeated ramp extraction, physical condition grouping and measured-actuator alignment. `test_tire_gui_qt.py` includes X/Y response mapping and aligned replay overlays. `test_sim_gui_yaw_map.py` covers GUI candidate/cache identity validation and 14DoF planar reduction. `test_sim_gui_yaw_map_qt.py` covers cancellable Qt automatic-map preparation and process lifecycle. `test_yaw_map_parallel.py` covers ordered yaw evaluator batches, shared cache behavior, validation, worker deadlines/exceptions, and orphan cleanup after parent SIGKILL. `yaw_parallel_blocking.py` provides a top-level blocking worker fixture for deadline tests. `_yaw_parallel_spawn_smoke.py` compares real compiled serial and spawned yaw cells.

## Subdirectories

| Directory | Purpose |
|---|---|
| `fixtures/` | Data files that tests read. |
