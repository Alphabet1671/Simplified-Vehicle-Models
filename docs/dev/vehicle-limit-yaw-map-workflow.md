# Vehicle-limit yaw-map workflow

The old ±9.80665 m/s² lateral grid is available only as explicit `--yaw-map-mode limited-grid` comparison mode. Driver, fixed-path probe and preflight default to the shared vehicle-limit builder. Incomplete builds are saved but cannot be used as accepted maps.

## What “vehicle limit” means

This is the current quasi-static no-TV LP model, with its tyre peak-force table, load transfer, aerodynamic loads, motor envelope and hydraulic brake bias. It is **not** a measured-vehicle limit or transient handling certification. The longitudinal domain remains explicitly declared (default -6 to +3 m/s²); lateral discovery covers that domain, not every physically possible longitudinal acceleration.

The search enclosure is `4*max(tabulated lateral tyre peak force)/vehicle mass`. It is finite because the existing peak-envelope interpolator clamps above its load-table range. For IEM24 the enclosure is 77.4103 m/s², deliberately much larger than achievable lateral acceleration. The tyre load table ends at 3500 N: the extrapolation/clamping behavior is a modeling limitation, not new experimental evidence.

Discovery scans both signs all the way to that enclosure. It does not stop at the first infeasible cell and does not assume a connected feasible region. Observed transitions are bracket-refined. Islands narrower than the configured scan spacing can still be missed; every artifact discloses that finite-resolution scope.

## Acceptance

1. Discover support over requested speed/ax knots and midpoint subdivisions with direct LP queries in both yaw objective directions.
2. Construct a rectangular strict-mask map with exact zero knots and a maximum lateral cell width. Never fill masked holes or broaden interpolation tolerances.
3. Independently scan signed lateral rays at cell-third speed/ax holdouts and refine their direct boundary brackets.
4. Compare directly feasible component/boundary endpoints to strict supported lateral knots at the same speed/ax. Wholly masked slices are explicitly unsupported, not assigned a fictitious small error.
5. Refine the worst responsible rectangular cells within declared budgets. Completion requires no outer truncation, no unresolved LPs or exhausted budgets, and loss within the declared tolerance.

This is a finite-resolution acceptance audit, not proof over the continuum. Boundary loss is conservative distance to a supported **knot**, which may overestimate distance to the supported interval. Full nonzero yaw demand must still be validated by the profile solver.

HiGHS status 2 means physical LP infeasibility. Other unsuccessful statuses and nonfinite optimal objectives are unresolved. A numerical failure must never create a physical mask boundary.

## Build

```sh
PYTHONPATH=src:. python3 tools/run/build_vehicle_yaw_map.py \
  --output runs/yaw_map_build/iem24.npz \
  --speed-points 12 --ax-points 9 --ax-min -6 --ax-max 3 \
  --ay-spacing .5 --scan-spacing .5 --boundary-tolerance .05 \
  --loss-tolerance .25 --max-probes 250000 --audit-probes 500000 \
  --max-nodes 200000 --refinement-rounds 64 --wall-seconds 600
```

Exit 0 means the finite-resolution audit completed; exit 4 means it did not. Inspect the adjacent JSON report before increasing resolution or budgets. Do not loosen the loss tolerance merely to get a success status.

The NPZ embeds axes, directional moments, strict mask, discovery evidence, audit evidence, settings, model/build identity and provenance. Input or implementation changes invalidate loading; array integrity is checked. Incomplete artifacts require an explicit diagnostic-only override at the library loader and are rejected by normal consumer tools.

## Use the same artifact

Consumers expose `--yaw-map` plus the same `--yaw-ay-spacing`, `--yaw-map-loss-tolerance`, `--yaw-map-max-nodes`, `--yaw-map-audit-probes`, `--yaw-map-refinement-rounds`, and `--yaw-map-wall-seconds`. Settings and initial speed/ax knots must match the build identity. A new default build is not silently substituted when a stale cache is supplied.

A shared physical-cell cache reuses direct LP moments across discovery, map construction and repeated audits.

## Direct IEM24 evidence

Signed direct discovery with 1 m/s² scans and 0.03125 m/s² brackets (1,984 probes, 1.8 s) found symmetric intervals at these sampled operating points:

| Speed m/s | ax m/s² | feasible |ay| boundary m/s² | infeasible side m/s² |
|---:|---:|---:|---:|
| 5 | 0 | 15.53125 | 15.5625 |
| 15 | 0 | 16.75000 | 16.78125 |
| 25 | 0 | 18.68750 | 18.71875 |
| 33 | 0 | 20.56250 | 20.59375 |
| 33 | -6 | 22.56250 | 22.59375 |

At ax=+3 and speeds25/33, no feasible component was observed; that must remain a masked operating region rather than being filled to make the rectangular map look complete.

Physical acceptance-build qualification and end-to-end preflight results are recorded separately when complete. Wider extent by itself is not acceptance.
