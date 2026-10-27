# Interval-oracle local cut restriction

Transition interval construction now uses `searchsorted` to visit only speed,
longitudinal-acceleration, and lateral-acceleration knots whose mapped speed can
lie in the already bounded local `[ulo, uhi]` domain. Search bounds are expanded
outward by one ULP and the original strict `ulo < u < uhi` filter remains the
final inclusion rule. Thus no partition face in the legal interval is removed;
only knots whose mapping is outside it are skipped.

No support, interpolation, root, moment, or acceptance tolerance changed.

The line optimizer now stores accepted warm suggestions as q=speed² at reference
stations, interpolates q at new reference stations, clamps only numerical
negative q to zero, then returns `sqrt(q)` to the existing profile interface.
Closed seams remain explicitly equal. This preserves piecewise-linear q and its
associated segment acceleration more faithfully than speed interpolation; it is
still only a strictly revalidated suggestion.

Guarded verification:

```sh
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 120 -- \
  python3 -m unittest tests.test_yaw_line_budget tests.test_yaw_profile_transition -q
```

Result: 26 tests passed in 6.896 seconds; `git diff --check` passed.

The physical fine-grid profile re-run remains parent-owned.
