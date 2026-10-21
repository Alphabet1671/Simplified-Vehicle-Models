# Task 2: reference-free multi-lap geometry

## Status

Implementation and focused validation complete. This work adds a reference-free detector, lap registration, robust clipping and arithmetic averaging; integrates selectable channels into the existing closed builder; and preserves legacy positional settings. Detector intervals share at most the exact crossing endpoint; their interiors do not overlap, so boundary samples do not create duplicate counted laps.

## Implementation

- Added `tools/track_gui/lap_average.py` with `detect_laps` and `average_laps`. Detection searches spatial returns under configurable travel/time/radius and heading constraints, chooses the closest point in the first return encounter, and reports lap boundaries. Processing filters gross lap lengths and shape residuals, cyclically registers loops using translation only (no north/east axis rotation), clips point residual outliers, computes a retained-sample arithmetic mean, and returns aligned overlap arrays and diagnostics.
- Added `geometry_from_arrays` in `tools/tracks/build_closed_track.py`; existing CLI defaults are unchanged. It creates closed geometry and shared geometry fields from arrays.
- Extended `ClosedSettings` only after existing positional fields. Default blank-unit trust applies only to the legacy default channel pair in the legacy path; selected GUI channels must explicitly opt into unknown units through `ChannelSelection` flags.
- Provenance carries channel names, mode, conversion scales/unknown-unit flags, time window, and relevant geometry/detection settings. GPS provenance does not claim velocity input. Multi-lap smoothing is performed once in the averaging helper; geometry construction then uses zero smoothing.

## Public API and diagnostics

`detect_laps(time, north, east, *, return_radius, minimum_travel, minimum_duration, maximum_duration, heading_tolerance)` returns `(lap_slices, diagnostics)`.

`average_laps(time, north, east, *, spacing, smoothing, return_radius, minimum_travel, minimum_duration, maximum_duration, deviation_limit, minimum_support)` returns `(north_nodes, east_nodes, diagnostics, overlap_points)`.

Diagnostic fields: `candidate_count` (spatial-return intervals passed by the detector), `detection_rejections` and `rejected_candidate_count` (heading-mismatch return attempts, not complete laps), `detection_stride`, `accepted_laps`, `rejected_laps` (complete intervals rejected by loop quality), `lap_lengths_m`, `lap_boundaries` (index and time boundaries), `clipped_samples`, `total_aligned_samples`, `overlap_support`, `residuals_m` (`median`, `p95`, `maximum`), and `registration`. Geometry fields: `lap_diagnostics`, `overlap_north`, `overlap_east`, and `provenance`. The prior 132 FSC count represented detector heading-mismatch attempts, not 132 complete rejected laps; final counts are recorded below.

## Verification

Focused command passed after fix round 1: `PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_laps tests.test_track_gui_channels tests.test_track_gui_closed tests.test_closed_track tests.test_track_tool_patches -q` — 56 tests, OK. Lap coverage includes unequal-shape arithmetic mean, local spike clipping, insufficient overlap-support refusal, requested support validation, incomplete tails, stationary/no-loop refusal, a geometry node cap, validation before SciPy import, and a single exact shared seam endpoint without positive-duration overlap.

## Fix round 1

Initial review found public-input validation gaps. `average_laps` now rejects non-finite or nonpositive spacing and deviation limits, negative or non-finite smoothing, and invalid support counts with actionable `ValueError` messages before array conversion. `detect_laps` now validates positive finite return radius, minimum travel and duration limits, ordered minimum/maximum duration, and heading tolerance in `(0, pi]` before importing SciPy. It also rejects non-vector channel arrays. Negative parameter tests patch the import only to prove validation runs first; they do not mock the valid averaging path. The normal arithmetic and real-fixture results are unchanged by this round.

MIS 2024 (`/Users/alphabet1671/Desktop/0615CompEndurance.mat`, SHA256 `091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7`): VNorth/VEast, velocity mode, 550–1050 s; return radius 25 m, minimum travel 150 m, duration 15–120 s, spacing 0.5 m, smoothing 1 m. Five loops accepted, measured loop lengths 1022.73, 1024.31, 1026.24, 1023.75, 1024.13 m; 21 point samples clipped; zero detector/quality rejections. Result length 1021.87 m, 2044 geometry nodes (2045 loader rows including duplicate endpoint), closure gap 0, winding -6.28699 rad (0.00381 rad error), minimum radius 4.77 m, minimum self-distance 14.31 m. `save_closed` wrote XML/CSV in a temporary output directory and `load_track` accepted the closed result; loaded length matched within one node spacing.

FSC 2026 (`/Users/alphabet1671/Dev/Simplified-Vehicle-Models/IEM24 Files/FSC-2026-May-Endurance.mat`, SHA256 `e160110c53e41c8a3419edafc30d84f6bf3bf442016ec7480c11c3d425e6f82b`): Latitude_MTI/Longitude_MTI, GPS mode, 720.9–1103.6 s; return radius 10 m, minimum travel 150 m, duration 18–35 s, spacing 0.1 m, smoothing 0.5 m. Fourteen loops accepted, lengths 290.57–305.08 m; zero complete loops were rejected by quality checks, while 133 heading-mismatch return attempts were rejected by detection; 3438 point samples clipped. Result length 288.43 m, 2884 geometry nodes (2885 loader rows including duplicate endpoint), closure gap 0, winding -6.28398 rad (0.00080 rad error), minimum radius 1.37 m, minimum self-distance 0.0032 m. Residuals are recorded in geometry diagnostics and embedded saved provenance. The very small minimum self-distance deserves review: this metric flags close geometry but does not establish a self-intersection. `save_closed` wrote XML/CSV in a temporary output directory and `load_track` accepted the closed result; loaded length matched within one node spacing. No visual inspection is claimed. This close-approach diagnostic warrants later geometry review; it is not itself a proof of intersection.

The MAT inputs were read only and were not modified or committed. Round-trip evidence covers geometry build, temporary save, and strict loader validation on both sources. We did not claim visual inspection, reverse/split tests here, or a full repository regression run; those remain in the broader plan.
