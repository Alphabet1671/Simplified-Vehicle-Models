# Task 3 report — accepted maneuver evidence export

## Round 2 repair

Persisted accepted `SimulationResult.channels` as `channels.json`, included its SHA-256 in both artifact writers and integrity checks, and handled frozen Mapping channel rows during serialization. Added a validated evidence loader. Replaced the exporter API's naked manifest/result trust with a completed artifact directory: it validates via `load_artifact_evidence`, then reconstructs the record from integrity-checked NPZ arrays, commands CSV, and channels JSON only. It requires the exact compiled `14dof-simple` identity and canonical 38 vertical state names; stopped/failed outcomes are rejected. No plant instance/evaluation is used.

## Tests

Ran the requested command:

```text
PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_artifacts tests.test_maneuver_run_record -v
Ran 7 tests — OK
```

Tests create an actual artifact from a frozen-channel accepted result, export it through `FrameSource`, verify endpoints/position/speed, patch plant initialization/evaluation to fail if invoked, check channel tamper and stopped outcome rejection, and preserve existing artifact writer/legacy writer tests. `git diff --check` passed after cleanup.

## Commit

Round 2 repair committed as `1628149` (`fix: export only validated maneuver artifact evidence`) following initial commits `266c3c5` and `52172ec`.
