# Task 3 report — fold-guarded Corridor

## Implementation

Added `Corridor` and `corridor_from_track` in `src/simplified_vehicle_models/tracks/corridor.py`. Bounds use positive-right coordinates (`minimum = -left + margin`, `maximum = right - margin`), enforce the fold guard conservatively on both sides using absolute curvature, reject missing edges and invalid/empty corridors, and preserve provenance plus `is_conditional` (`provenance != "surveyed"`). Arrays are copied, validated and read-only. `is_periodic` preserves and checks the track closure flag alongside matching endpoint bounds. A zero-width corridor remains representable; `half_width` does not modify the actual one-sided bounds.

## TDD and tests

- RED: before implementation, `PYTHONPATH=src python3 -m unittest tests.test_corridor -v` failed as expected with `ModuleNotFoundError` for `tracks.corridor`.
- GREEN: final focused run `PYTHONPATH=src python3 -m unittest tests.test_corridor -v` — 10 passed.
- Full suite `PYTHONPATH=src python3 -m unittest discover -s tests -q` was started; it exceeded the 120-second foreground limit and continued as background job `bash-6155`. Output so far contains environment/cache warnings; final completion status remains to be collected.

## Self-review and concerns

- Conservative absolute-curvature clipping covers legacy tracks whose curvature direction may be unknown.
- Periodicity is not inferred from equal widths alone; the stored closure flag is required. The test exercises an open-flag track with identical endpoint bounds.
- Task 8 exact zero bounds remain caller-controlled; `half_width` reports a scalar only and does not imply symmetric shrink.
- Full-suite outcome remains pending; collect background job `bash-6155` before closing.
