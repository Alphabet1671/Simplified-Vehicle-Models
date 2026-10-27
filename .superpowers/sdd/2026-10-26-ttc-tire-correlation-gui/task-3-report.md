# Task 3 — steady automatic fitting report

## Scope and baseline

Implemented on `feature/ttc-tire-correlation-pyqt`, starting at `596e535b474eb94fb1b0aa6893cd9c159e07b669`. Read Task 3 brief and binding spec only. No repository XML or measured data were changed or staged.

## RED / GREEN evidence

- RED: `PYTHONPATH=src:. python3 -m unittest tests.test_tire_gui_fit -q` failed at import because `tools.tire_gui.fit` did not exist.
- GREEN: `PYTHONPATH=src:. python3 -m unittest tests.test_tire_gui_data tests.test_tire_gui_model tests.test_tire_gui_fit -v` — 18 tests passed.
- GREEN: `PYTHONPATH=src:. python3 -m unittest tests.test_tire_gui_data tests.test_tire_gui_model tests.test_tire_gui_fit tests.test_unitire_simple tests.test_unitire_transient -q` — 56 tests passed.
- Synthetic coverage exercises separate group RMS, empty-group rejection, zero residual at known parameters, single unlocked-parameter improvement with exact lock preservation, immutable seed, and cancellation before prediction. Objective test uses exact expected result 8.25.

## Implementation

- `tools/tire_gui/fit.py` implements force then trail stages using repository `predict_steady` and immutable `with_parameters` snapshots, with required configured finite bounds, logistic transforms, Nelder–Mead limits/tolerances, cancellation, scalar progress, per-parameter outcomes, dimensional RMSE/count metrics, and best finite candidate retention on nonconvergence. Fit starts can be clamped only in optimizer coordinates and generate a warning. Invalid model-domain trials are rejected as infeasible without changing the seed.
- `tools/tire_gui/contracts.py` adds `FitCancelled` and immutable-mapping `FitResult`.
- `tools/tire_gui/model.py` removes the optimizer-hot 501-load-point duplicate relaxation check: stiffness extrema are checked analytically at endpoints and real stationary points; positive scale/stiffness factors ensure relaxation length positivity across the domain. Candidate validation remains in place.
- Steady fitting explicitly preserves the snapshot transient coefficients as provisional and labels their provenance accordingly.

## Private acceptance

Private source: `private/ttc/unitire_simple_fit.mat`; reference fitData dimension inspection matched the stated counts: lateral 1080, longitudinal 720, combined 1440, Mz 1080. MAT `vBelt` was 11.1647222222 m/s. Comparison of measured selection arrays and objective was **blocked**: the local run-72 MAT has eight duplicate ET timestamps. The existing ingestion contract rejects nonmonotonic elapsed time (`ValueError: Elapsed-time channel is nonmonotonic`), appropriately preventing silently altered source data. No measured arrays, residuals, or comparisons were written into a report or tracked fixture.

## Known concerns

- SciPy and MATLAB optimizer trajectories need not match; the named compatibility behavior matches objective, bounded logistic transform, optimizer, iteration/evaluation limits, and tolerances.
- The fit entrypoint is core-only; UI orchestration must keep candidate application explicit and must not auto-apply nonconverged results.
- Local private numeric parity remains unverified until source duplicate-time handling can be reconciled with the source selection contract without modifying raw data.

## Commit

Pending at report creation; commit expected: `feat: implement staged steady tire correlation`.
