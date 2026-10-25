# Task 2 report — yaw-aware fixed-path profile

**Scope:** Phase 2 implementation of yaw-aware speed profiling. The `qss_*` OCP files were not modified.

## Implemented

- Added `solvers/yaw_aware_profile.py` with immutable `YawPath` and `YawAwareProfile` types.
- Added `curvature_rate_from_samples(distance, curvature, is_closed=...)`, requiring physical arclength starting at zero and supporting nonuniform open grids and duplicated-seam periodic grids.
- Added strict trilinear interpolation of Phase-1 `YawCapabilityMap` arrays. Interpolation rejects out-of-domain queries and any query whose nonzero-weight support includes an infeasible map node.
- Added `yaw_aware_profile(...)` with speed caps based on feasible speed/longitudinal-acceleration candidates, yaw-moment demand, repeated energy-based forward/backward transition projection, flying periodic and open standing-start boundaries, elapsed-time integration, and explicit non-certificate diagnostics.
- Added `to_speed_plan(path, profile)` projection for the existing forward driver. It requires exact physical-grid agreement and a converged profile; `SpeedPlan` remains a tracking request, not a capability proof.
- Added `tests/test_yaw_aware_profile.py` for input validation, finite-difference curvature rate, constant-radius/nonbinding yaw, a clothoid yaw cap, infeasible-map rejection, closure/standing semantics, and SpeedPlan conversion.

Standing starts explicitly reject maps whose speed grid starts above zero; no unsupported low-speed extension is inferred.

## Validation

Passed:

```text
PYTHONPATH=src python3 -m unittest tests.test_yaw_aware_profile tests.test_yaw_capability tests.test_line_trajectory tests.test_open_standing_start -q
Ran 41 tests in 19.318s — OK
```

## Limitations / contract notes

This is a sampled profile, not a continuous feasibility certificate. Segment feasibility is checked at a midpoint state; the descending discrete scans are conservative only relative to the sampled grid/candidates, not a formal global optimizer for nonmonotone maps. The clothoid fixture verifies the map-derived cap within its grid resolution. `YawPath` takes explicit physical distance; geometry-to-path station conversion remains a caller responsibility. The finite-difference utility is independent of `OffsetTrajectory`; callers must sample curvature consistently in path arclength and respect spline/geometry knots.
