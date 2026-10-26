# Planar-forward steering trial rejection

The steering-angle guard in `PlanarForwardPlant.evaluate` now raises
`TrialStateError` rather than `ValueError` when a finite trial state exceeds the
physical steering bound. `solve_forward.rhs` converts only `TrialStateError`
from implicit Newton/Radau trials to NaN so SciPy can reject the trial and reduce
its step; the physical bound is unchanged.

Malformed/nonfinite states remain `ValueError`, preserving the initial-state
input contract. Direct accepted-state replay still receives `TrialStateError`
if an actual accepted state violates steering bounds, so this change does not
mark a violating state valid.

Guarded verification:

```sh
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 120 -- \
  python3 -m unittest tests.test_planar_forward -q
```

Result: 10 tests passed in 0.821 seconds. The added test distinguishes retryable
finite steering overshoot from malformed nonfinite state rejection.
