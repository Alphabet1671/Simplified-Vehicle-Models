# SDD ledger — plan: docs/superpowers/plans/2026-10-25-iem24-14dof-maneuver-animation.md

## Preflight interface scan

| Tasks | Shared file/interface | Finding |
|---|---|---|
| 1 → 2 | `VerticalManeuverPlant` used by runner selection | Clean: Task 1 defines adapter methods Task 2 consumes. |
| 1 → 3 | accepted `SimulationResult` / vertical state names | Clean: Task 1 supplies accepted outputs; Task 3 exports stored evidence only. |
| 2 → 5 | strict 14DoF definition and runner support | Clean: input definitions only execute after explicit selection. |
| 3 → 4 | `write_maneuver_run_record` | Clean: Task 3 provides the bridge's sole record writer. |
| 4 → 6 | completed artifact to video CLI | Clean: Task 6 consumes only successful completed artifacts. |
| 5 → 6 | six immutable JSON/CSV inputs | Clean: Task 6 executes each one independently. |
| Task 1 | vertical `s/n/heading_error` coordinate semantics | Ruling: use open straight reference so `s/n/heading_error` are world x/y/heading for animation; this excludes circular-reference geometry but preserves existing 14DoF physical evaluator. Cost if wrong: an adapter redesign is required before nonstraight geometry claims. |
| Task 5 | 1.0 g peak-ay requests and calibration metadata | Ruling: do not invent peak-ay calibration metadata; execute only after the actual calibration mechanism can produce matching evidence. Cost if wrong: requested peak-ay cases remain not runnable until calibration support is extended. |


Task 1: complete (commits cba7c05..dff6ba7, review clean)
Task 1: minor (deferred): accepted_outputs evaluates arbitrary supplied state; simulator records only accepted states, so no trial evidence is persisted. Final review must confirm this invariant.
Task 2: three prior implementation delegates were stopped after incomplete runner changes. The third reached a partial type-selection body but its smoke fixture violated schema (`calibration.artifact` cannot be a nested object) and its helper import was invalid (`VerticalVehicleParameters` is defined in `formulations.vertical_14dof`, not `parameters.vehicle`); the controller restored all Task 2 source/test changes. Ruling: reuse the verified type discriminator and flatten the simulation calibration directly at `calibration` (runner already supports `artifact=calibration` fallback). Cost if wrong: Task 2 test fixture or calibration persistence must be redesigned before all later work.
Task 2: fix round 1/5 (1 addressed, 0 open — `model_kind` is integrity-hashed and validated against compiled checked-in formulation identity; commits 5975b27..3e2290e)
Task 2: complete (commits dff6ba7..3e2290e, review clean)
Task 3: fix round 1/5 started — reviewer found accepted frozen channel mappings were discarded, 14DoF identity was only length-checked, and provenance accepted arbitrary digest mappings. First attempted amendment only added JSON normalization; it did not address the assigned findings. Implementer was reissued exact remaining requirements.
Task 3: fix round 1 review — canonical identity and MappingProxy channel support are addressed; provenance remains self-asserted and `stopped` outcome was accepted. Ruling: correct the plan seam by requiring a validated completed artifact directory for provenance (not a naked mapping), validated with `load_artifact` / manifest integrity, and persist accepted channels in artifacts so the future bridge can export without plant reevaluation. Cost if wrong: artifact schema must be redesigned again before video bridge can be trusted.
Task 3: fix round 2 — repaired: both writers persist channels.json with hashes and validated loading; exporter consumes completed artifact directories and trusted arrays/commands/channels only, enforcing compiled 14DoF identity, canonical states and completed outcome. Required focused suite passed (7 tests). See task-3-report.md; repair committed.
