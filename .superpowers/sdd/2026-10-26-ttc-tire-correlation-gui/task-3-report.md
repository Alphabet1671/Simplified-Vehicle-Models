# Task 3 — steady automatic fitting report

## Scope and baseline

Implemented on `feature/ttc-tire-correlation-pyqt`, starting at `596e535b474eb94fb1b0aa6893cd9c159e07b669`. Read Task 3 brief and binding spec only. No repository XML or measured data were changed or staged.

## RED / GREEN evidence

- RED: `PYTHONPATH=src:. python3 -m unittest tests.test_tire_gui_fit -q` failed at import because `tools.tire_gui.fit` did not exist.
- GREEN: `PYTHONPATH=src:. python3 -m unittest tests.test_tire_gui_data tests.test_tire_gui_model tests.test_tire_gui_fit -v` — 18 tests passed.
- GREEN: `PYTHONPATH=src:. python3 -m unittest tests.test_tire_gui_data tests.test_tire_gui_model tests.test_tire_gui_fit tests.test_unitire_simple tests.test_unitire_transient -q` — 56 tests passed.
- Synthetic coverage exercises separate group RMS, empty-group rejection, zero residual at known parameters, single unlocked-parameter improvement with exact lock preservation, immutable seed, cancellation before prediction, NaN prediction rejection, boundary saturation reporting, and nonconvergence status with finite best candidate retention. Objective test uses exact expected result 8.25.
- GREEN correction: `PYTHONPATH=src:. python3 -m unittest tests.test_tire_gui_data tests.test_tire_gui_model tests.test_tire_gui_fit -v` — 22 tests passed.

## Implementation

- `tools/tire_gui/fit.py` implements force then trail stages using repository `predict_steady` and immutable `with_parameters` snapshots, with required configured finite bounds, logistic transforms, Nelder–Mead limits/tolerances, cancellation, scalar progress, per-parameter outcomes, dimensional RMSE/count metrics, and best finite candidate retention on nonconvergence. Fit starts can be clamped only in optimizer coordinates and generate a warning. Invalid model-domain trials are rejected as infeasible without changing the seed.
- `tools/tire_gui/contracts.py` adds `FitCancelled` and immutable-mapping `FitResult`.
- `tools/tire_gui/model.py` removes the optimizer-hot 501-load-point duplicate relaxation check: stiffness extrema are checked analytically at endpoints and real stationary points; positive scale/stiffness factors ensure relaxation length positivity across the domain. Candidate validation remains in place.
- Steady fitting explicitly preserves the snapshot transient coefficients as provisional and labels their provenance accordingly.

## Private acceptance

Private source: `private/ttc/unitire_simple_fit.mat` with local runs 8 and 72. Added `load_run(..., require_monotonic_time=False)` as explicit steady-only mode: it preserves the normalized time vector, all original channel rows and raw indices, and records a metadata warning. Default remains strict and rejects duplicate ET for transient/event work. No source row was removed. Reference fitData dimensions and selected arrays exactly match: lateral 1080, longitudinal 720, combined Fx 1440, combined Fy 1440, Mz 1080; max absolute difference for alpha/kappa/target arrays was zero in every group; belt-speed difference was zero. Force compatibility objective on both selection and fitData was 0.7960860084349208 (delta zero); trail normalized objective was 0.06309355126176772. Detailed private acceptance evidence is at ignored `tire_fits_out/task3_private_acceptance.txt`; no measurements or residual arrays are tracked.

## Known concerns

- SciPy and MATLAB optimizer trajectories need not match; the named compatibility behavior matches objective, bounded logistic transform, optimizer, iteration/evaluation limits, and tolerances.
- The fit entrypoint is core-only; UI orchestration must keep candidate application explicit and must not auto-apply nonconverged results.
- SciPy and MATLAB optimizer trajectories need not match; objective/data compatibility is verified, optimizer path parity is not claimed.

## Commit

Initial implementation: `008dca11e24bbc9d86b4f3bdf741267bc4906040`. Follow-up correction commit records explicit steady ingestion mode, private acceptance outcome, and robustness tests. The report is an intentional scoped force-add exception because the user expressly required this exact `.superpowers/...` destination; all other `.superpowers/` content remains ignored.
