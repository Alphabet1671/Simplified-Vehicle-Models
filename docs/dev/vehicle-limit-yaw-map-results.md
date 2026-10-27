# Vehicle-limit yaw map: IEM24 results

The ±1 g lateral grid is no longer the default. The map builder discovers signed support of the actual quasi-static model and accepts a map only after separate refinement and held-out checks.

## Physical limits

| Speed m/s | ax m/s² | directly feasible |ay| m/s² | directly infeasible |ay| m/s² |
|---:|---:|---:|---:|
| 5 | 0 | 15.53125 | 15.56250 |
| 15 | 0 | 16.75000 | 16.78125 |
| 25 | 0 | 18.68750 | 18.71875 |
| 33 | 0 | 20.56250 | 20.59375 |
| 33 | -6 | 22.56250 | 22.59375 |

Both signs were explicitly queried; symmetry was observed, not imposed. The entire declared ax range remains **−6 to +3 m/s²**; this is not a full longitudinal-envelope certification. Other ax domains can be built explicitly.

## Qualified finite-resolution final build

- Model-derived lateral search enclosure: ±77.41027 m/s², not an arbitrary g ceiling.
- Final lateral map extent: **±25.1875 m/s²**, including combined-acceleration boundary evidence.
- Final tensor: **79 speed ×76 ax ×428 ay = 2,569,712 nodes**.
- Maximum ay cell width:0.5 m/s², locally refined near boundaries; exact zero is retained.
- Refined direct boundary bracket width ≤0.05 m/s²; scan spacing0.5 m/s².
- 458 selective midpoint refinement rounds; approximately686s wall on this machine.
- Training:126,124 direct-coordinate probes and2,172 boundary comparisons; maximum conservative strict supported-knot distance **0.125 m/s²**.
- Held-out validation: **126,116 canonical HiGHS direct-coordinate probes**,2,158 boundary comparisons; maximum loss **0.25 m/s²**.
- No outer-domain truncation, unresolved LP, exhausted budget or wholly unsupported validation slice in this accepted build.

Construction uses an algebraically equivalent rear-only/no-TV polygon reduction; other drivetrains/TV modes fall back to canonical LP. Near numerically active boundaries the reduction also falls back to canonical LP. Tests compare1,900 points and both directional moments against HiGHS. Held-out validation always uses a separate canonical HiGHS evaluator, not the construction evaluator.

The tyre peak-envelope load table ends at3500N. Sampled feasible boundary loads reached2284N, below that ceiling; the table's clamp remains a disclosed model limitation.

## Track coverage after loading the final artifact

The shared preflight consumer successfully verified the artifact's current implementation/input identity and loaded it for both tracks at0.05m spacing:

| Track | Stations | Map holes | Lateral-axis saturation | Preflight warnings |
|---|---:|---:|---:|---|
| MIS |20,335|0|**0%** (previously79%)|none|
| FSC synthetic3m |15,353|0|**0%** (previously88%)|none|

This is coverage evidence, not a new lap solve or controller qualification. A coarse10m fixed-path probe loaded the accepted map successfully but hit the profile solver's initial candidate-seed budget; the substantially larger map needs separate graph-seed budgeting/performance work before claiming end-to-end planning readiness. The lowest sampled steady supported speeds rose to9.94m/s(MIS) and9.09m/s(FSC); these are preflight samples, not optimized profile speeds.

## Important negative evidence

An earlier internal-audit-only build passed but failed fresh holdouts (1.21875m/s² loss, five wholly unsupported faces). That acceptance was withdrawn. The production workflow now:

1. Refines using original-cell thirds.
2. Splits cells toward strict support, with measured-progress fallback for cross-axis masking.
3. Uses a fixed training guard band half the acceptance tolerance.
4. Freezes the map before validating original-cell quarter positions using independent HiGHS LPs.
5. Refuses an incomplete artifact rather than relaxing masks/tolerances.

This is **finite-resolution evidence, not a continuous feasibility proof**. Islands narrower than0.5m/s² can be missed. Nonzero yaw-demand feasibility and track/controller behavior remain separate checks. Legacy QSS/OCP files and tests were not changed.

## Reproduce

```sh
PYTHONPATH=src:. python3 tools/run/build_vehicle_yaw_map.py \
  --output runs/yaw_map_build/iem24-vehicle-limit.npz \
  --speed-points 12 --ax-points 9
```

Production CLI defaults provide3M map nodes,768refinement rounds,1200s,0.25m/s² acceptance loss and a0.125m/s² internal training target. Library settings deliberately retain smaller conservative budgets for small/test builds; an exhausted budget never qualifies.

The NPZ and adjacent JSON retain all axes, masks, moments, boundary brackets, training/validation evidence, settings, transitive input hashes, compiled-model identity and implementation hash. Consumer tools require identity-compatible settings and reject incomplete artifacts.

See [workflow details](vehicle-limit-yaw-map-workflow.md) for flags and scope.
