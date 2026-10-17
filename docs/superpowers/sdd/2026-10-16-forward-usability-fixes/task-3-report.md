# Task 3 report: canonical evidence attempt

## Status at the first attempt (superseded)

The first bounded sweep produced no completed lap. One 7DoF candidate ran into the 300 s parent timeout (295 s solver budget) and ended `Timed_Out`, `lap_completed=false`, `usable=false`. It is retained at `docs/validation/iem24-murphy/forward-usability-sweep/planar/attempt_1792211261062379000/` for provenance only. Its summary records `planner=conservative` although the wrapper asked for `live`, so its planner provenance is inconsistent and it is not a tuning result.

## Correction

This report previously said solver progress reached the lap seam and restarted, and it concluded that a single-lap orchestration was required. That explanation was wrong. The run used non-canonical settings and just exceeded its time budget. The rewritten wrapper `tools/run/sweep_forward_steering.py` runs every candidate with the canonical settings, records them, and completes one lap in about 100 s (7DoF). No seam handling was needed.

## Follow-up

The sweeps, chosen gains and the evidence chains are recorded in `docs/validation/iem24-murphy/forward-usability-sweep/README.md` and the per-model chain directories. No acceptance threshold was weakened.
