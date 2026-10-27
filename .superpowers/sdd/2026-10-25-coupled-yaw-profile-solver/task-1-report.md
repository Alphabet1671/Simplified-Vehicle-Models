# Task 1 report — Freeze reproductions and separate missing-zero effects

## Baseline

- Baseline revision before Task 1 commit: `4a4e4befefc115e50b4481839ba632145a714452`.
- The brief requested a verbose pre-change baseline run, but it was not captured in this work session; baseline test count/result is unavailable.

## Code and artifacts

- Added `anchored_axis` to the runner: retains requested linspace points and unions exact zero; strict ordering/finite/domain/count behavior is covered. Actual map axis lengths are emitted alongside requested base counts.
- Added runner axis tests for counts 5, 7, 9, and 13 and invalid bounds/counts.
- Added `tools/run/probe_coupled_yaw_profile.py` with raw-centerline and zero-offset geometry options, physical speed-domain/map construction, original or anchored axes, path/map arrays and hashes, settings/identity/timings, and artifact persistence before legacy profiling. Legacy failures are reported without fabricated segment evidence.
- Added deterministic synthetic lower-speed witness test checked edge-by-edge using the old strict oracle, plus its small JSON fixture and manifest.
- Physical-track probe fixtures were not generated. The manifest explicitly records this and makes no claim that anchoring removes any physical failure or that a coupling regression is established.

## Exact verification

- `PYTHONPATH=src:. python3 -m unittest tests.test_yaw_aware_profile tests.test_yaw_aware_forward_runner tests.test_probe_coupled_yaw_profile -q` — **22 tests passed**.
- Minimal no-physical-map CLI parse smoke: `PYTHONPATH=src:. python3 -c "from tools.run.probe_coupled_yaw_profile import parse_args; a=parse_args(['--track','unused.xml','--output','/tmp/yaw-probe-smoke.json','--geometry','raw-centerline']); print(a.geometry, a.speed_points, a.anchored)"` — **passed**, output `raw-centerline 12 False`.
- `git diff --check` — passed.
- Synthetic fixture SHA-256 verified as `ce57b45cdc4970dddcebd539813ead77b58dea9d2fdb60b5861eea83296fdc08`.

## Concerns / scope

- Physical reproduction runs, segment-level diagnostics, and physical fixture windows remain outstanding; Task 1 evidence is synthetic only. The probe cannot infer failed segment details from the current legacy profile API and explicitly records this limitation.
- The untracked plan file was excluded from the commit.
- Commit: `e3f6b03` (`test: freeze fine-grid yaw profile failures and anchor map axes`).
