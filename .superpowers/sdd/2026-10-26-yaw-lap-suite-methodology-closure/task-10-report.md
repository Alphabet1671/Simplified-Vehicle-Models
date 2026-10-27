# Task 10 — Yaw tracking robustness and FSC lateral penalty

## Decision and scope

Implemented a versioned case matrix, strict case-input parsing, state-override admission helper, analytic synthetic fixture generator, and watchdog-covered parser/controller regressions. The physical replay and gain-selection gates are explicitly **inconclusive/not run**. No controller change was made and no FSC penalty-removal claim is made.

This conservative stopping point follows the supplied rulings: Task 7 has no convergence-supported selected grid; Task 8's physical basis matrix was not authorized; and Task 9 historical optimized replays are identity-inconclusive because plan source provenance is missing. Those historical metrics are not a fresh matched baseline. No baseline has been invented.

## Artifacts

- `docs/dev/yaw-lap-suite-closure/yaw-tracking-cases-v1.json`: versioned immutable planned matrix. It enumerates ten state perturbations (one factor at a time plus both combined signs), declared actuator-lag sensitivity bounds (independent steering/pedal multipliers 0.8 and 1.2, physical limits unchanged), controller sweep axes (preview 0.05/0.10/0.20 s; damping 0.08/0.12/0.16; gains 1.8 and 2.0; 1 m minimum preview floor), synthetic fixture labels, nominal gates and physical validation status.
- `tools/run/run_yaw_tracking_case.py`: single-case JSON parser, finite/known-field checks, initial-state override and synthetic fixture helpers. A physical invocation requires an immutable plan path plus SHA-256; it then deliberately fails closed because this worktree cannot bind that plan to source provenance, map, and track identity. It does not launch a misleading partial measurement.
- `tests/test_yaw_tracking_cases.py`: matrix schema, analytic S-bend/seam/pinch fixture shape, finite/unknown field checks, invalid lag rejection, strict no-clipping inadmissible state rejection, and a subprocess watchdog regression for the runner's fail-closed physical gate.
- Existing `tests/test_yaw_tracking_driver.py` was retained unchanged; controller behavior remains unchanged.

The synthetic path helper creates a periodic sinusoidal S-bend curvature and analytic curvature rate, a Gaussian width pinch, and identifies the maximum-|curvature-rate| seam index. It does not claim a completed forward closed-loop robustness test or a seam-invariant physical replay. Rotated-seam path/mapping equivalence remains an unmeasured physical gate.

## Results

Focused test command, with the process-group watchdog covering the entire test process:

```text
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 120 -- python3 -m unittest tests.test_yaw_tracking_cases tests.test_yaw_tracking_driver -q
Ran 15 tests in 0.110s — OK

After wiring the suite's explicit override/variation options, expanded regressions (including existing suite parser tests) also passed under a 150 s process watchdog:

```text
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 150 -- python3 -m unittest tests.test_yaw_lap_suite tests.test_yaw_tracking_cases tests.test_yaw_tracking_driver -q
Ran 21 tests in 0.497s — OK
```

The Task10-owned suite edits are limited to a one-replay JSON interface and strict pre-integration state rejection. Lag variation requests fail closed until actuator lag is represented as adjustable plant dynamics.
```

The watchdog regression launches one case process beneath `run_with_deadline.py` (20 s cap) and checks that missing immutable-plan binding exits nonzero with a specific diagnostic. No actual physical case was launched; consequently no physical duration budget was consumed.

No perturbation or actuator range is reported as passing the nominal tracking gates. They are planned, not measured. No short held-out closed-loop candidate sweep was measured; therefore no candidate is selected. The selection criterion (lower lateral RMS and no worse completion, speed RMS, or maximum lateral error) remains unapplied. Control effort and preview-horizon sensitivity remain unmeasured.

## Physical gates and limitations

MIS/FSC × fixed/optimized plan validation is `inconclusive_not_run`. Exact physical case identity cannot be established from Task 9 historical plans: their saved provenance does not contain an attested source revision/dirty-hash set. Task 7 and Task 8 do not authorize a substitute grid/matrix. The case runner currently refuses to represent an unbound replay as evidence. Thus completion, lateral RMS/max, speed RMS, sampled-center containment, seam-invariance, and actuator sensitivity outcomes are unavailable.

The parser and suite-runner override helper treat override values as deltas added to plant-layout lateral error, heading error, forward speed, and yaw-rate entries. The suite accepts an explicit `--initial-state-overrides` JSON dictionary for one replay and rejects nonfinite/layout-invalid values and inadmissible perturbed speed before integration; it never clips to an admissible value. `--plant-variation` validates its declared steering/pedal lag multipliers, then explicitly rejects nonempty variations because the current plant's lag parameters are not wired for sensitivity changes. The implementation is an interface/preflight and diagnostic fixture contribution, not a full physical case executor. A later authorized run must bind one saved plan, resolved controller, explicit initial-state override and plant variation to each invocation, enforce the outer watchdog, and report each case separately against the nominal gates.

## Controller and scientific claims

No reproducible supported improvement was selected, so `yaw_tracking_driver.py` was not modified. The present evidence does not establish FSC penalty removal, physical robustness, hardware uncertainty bounds, whole-footprint containment, or correspondence to Thommyppillai's trim-linearization/LQ-preview method. Follow-up work should first obtain provenance-bound plans and an authorized physical basis matrix, then execute the frozen sweep and held-out validations without per-track retuning. If that bounded gain experiment fails, specify the trim-linearization/LQ-preview follow-up rather than adding unvalidated gain scheduling.
