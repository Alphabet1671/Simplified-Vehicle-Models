# Yaw-aware virtual-driver simulation and animation roadmap

**Prepared:** 2026-10-25. **Branch:** `feature/yaw-aware-virtual-driver` at `38cf592` when this roadmap was written.
**Status:** Implementation and diagnostic-execution roadmap. It is not a minimum-time, FMU-TV, measured-vehicle, or U3 qualification claim.

## Purpose

The yaw-aware driver now provides a physical yaw-capability map, line planner, fixed-path speed profile, and sampled forward-replay containment helper. This roadmap connects those components to the existing forward plant and animation exporter without modifying the established QSS OCP solver.

The target artifact chain is:

```text
edged signed track → yaw map → yaw-aware line + speed plan → forward simulation
→ optimized-path run record → animation MP4 or PNG sequence
```

## Stages

1. **Record-path adapter.** Convert the selected `YawAwareLinePlan` to physical-path `PathCoordinates`; run-record world pose must use this offset path, not the source centreline.
2. **Dedicated runner.** Add a `run_yaw_aware_forward_driver.py` diagnostic CLI. The existing centreline runner remains unchanged.
3. **MIS no-TV preflight and bounded run.** Use [the edged MIS track](../../models/tracks/2024_michigan_endurance_closed_edged.xml), a physical no-TV map, and the existing planar 7DoF forward plant first.
4. **MIS animation.** Write `run_record.npz`, dry-run the existing [animation exporter](../../tools/anim/export_animation.py), then write an MP4 with ffmpeg or a PNG sequence without it.
5. **FSC admission.** Only use an authoritative signed XY FSC track with independently supplied left/right edges. If no such model exists in the repository, record the block and stop rather than fabricating a course.
6. **Future TV/FM U run.** The controller-specific FMU harness remains shelved under [the FMU harness plan](../superpowers/plans/2026-10-25-fmu-yaw-capability-harness.md). A parameterized physical-TV map is not an FMU validation.

## Required disclosures for every diagnostic run

The runner must retain map/vehicle/track/plan identity, selected line metrics, solver outputs, sampled center-point edge clearance, and whether the forward lap reached the finish. It must disclose that:

- the trajectory is a numerical local result, not a globally minimum-time result;
- dense line containment and forward containment are not continuous footprint certificates;
- the sampled yaw map is quasi-static;
- regenerative TV is unavailable in the current physical map pending a charge-power model;
- a completed transient does not establish periodicity, convergence/refinement, closed-loop U3 qualification, real-vehicle validity, or FMU-TV behavior.

## FSC admission requirements

An FSC animation is blocked until all of the following are available:

| Required artifact | Reason |
|---|---|
| Source XML declaring a signed `closed-xy-profile` or `open-xy-profile` | The planner needs ordered, signed geometry and boundary semantics. |
| Referenced XY/curvature CSV | The forward plant and animation record need an authoritative spatial reference. |
| Left/right widths or edge coordinates in the source direction | The line optimizer must enforce a real corridor. |
| Closure/cut and seam convention | The forward run and path station map must agree on wrapping. |
| Provenance and import validation | Prevent an old OCP approximation from silently becoming physical source geometry. |

The initial repository scan finds MIS source tracks under `models/tracks/`, but no FSC track XML/CSV under that directory. The formal admission result is recorded separately after the scan is repeated as part of implementation.

## Execution plan

The task-level implementation plan is [2026-10-25-yaw-aware-driver-integration.md](../superpowers/plans/2026-10-25-yaw-aware-driver-integration.md).