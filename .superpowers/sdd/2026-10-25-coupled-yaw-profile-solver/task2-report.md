# Task 2 report — strict yaw-profile validation

Implemented the inherited contracts and strict-map refactor plus an independent profile validator. `validate_profile` recomputes each segment's midpoint speed, longitudinal/lateral acceleration, map support, and yaw-moment demand directly from q; it does not consume stored acceleration/certificates. It rejects nonfinite q, zero-zero edges, strict unsupported map queries, bad bounds and invalid flying seam speed. The object helper also compares the exact trapezoidal edge-time sum against stored elapsed/lap time within the requested scale tolerance.

Added closed curvature-rate seam agreement to `YawPath`; retained `_MapInterpolator` as an import alias from `yaw_profile_map`. Settings constructors already validate every declared count and tolerance, and tests now cover all fields.

Focused verification: `PYTHONPATH=src python3 -m unittest tests.test_yaw_profile_validation tests.test_yaw_aware_profile -q` — 17 tests passed.

The focused tests also exercise an exact grid face next to a masked corner and a nearby query with positive masked-corner weight; evidence reports the `1e-14` support threshold.
