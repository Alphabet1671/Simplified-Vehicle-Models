# Minimum lap time on a 2D track — stage 1 spec

**Status:** agreed 2026-10-20. This spec fixes the decisions argued in the
preceding design discussion. The implementation plan is
[minimum-lap-time-2d-track.md](2026-10-20-minimum-lap-time-2d-track.md).

**Goal:** minimum lap time on a track that carries real lateral edges, delivered
in stages so that each stage is usable on its own.

## Decisions

**D1. Stage 1 is edges plus a quasi-steady free-line solver.** It does not
contain transient dynamics. The review literature reports quasi-steady-state
free-trajectory optimisation as the cheap modern approach, and it produces the
two things the later transient stage needs: an optimal line and an optimistic
lap-time bound to compare against.

**D2. Track width lives beside the centreline, never inside it.** The geometry
CSV parser rejects any header other than the five existing columns, so width
must not be added there. Width is a separate `width` data role with its own CSV,
or a constant `<parameter>` fallback.

**D3. Width is a reference-point corridor.** The stored half-width is the
distance from the centreline to the edge *minus* the car half-width and a
declared margin. The solver never subtracts vehicle geometry and never
re-derives a body clearance. This keeps the model honest about what it can
certify.

**D4. Width data carries provenance.** Every width source is tagged `surveyed`,
`swept`, or `assumed`. A solver may run on any tag, but a result produced on an
`assumed` or `swept` corridor is reported as conditional and must say so in its
summary JSON. This is the guard against the failure mode where a corridor
constant quietly becomes the headline claim.

**D5. Stage 1 physics stays in one place.** The quasi-steady capability of the
vehicle is sampled from the existing NumPy point-mass physics
(`maximum_acceleration_limit`, `maximum_deceleration_limit`). CasADi is used
only to interpolate that sampled surface and to build the NLP. No tire or load
physics is re-implemented in CasADi in stage 1. That removes the single largest
maintenance risk identified in the discussion: two physics implementations that
drift apart.

**D6. Stage 1 formulation.** Distance domain `s ∈ [0, L]`. States `n, ψ, v`.
Controls `a` (longitudinal acceleration) and `ψ' = dψ/ds`. Corridor
`n_min(s) ≤ n ≤ n_max(s)`. Periodic boundary. Objective `∫ dt`. The curvature the
car actually experiences is `κ_v = (κ₀ + ψ') cos ψ / (1 − κ₀ n)`, and the
accelerations are bounded by the sampled capability surface at `(v, κ_v)`.
Trapezoid is not used; the transcription is Hermite–Simpson.

**D7. Stage 1 acceptance is five checks, and all five must pass.** Zero corridor
reproduces the existing centreline envelope; the free line is never slower than
the centreline; the line stays inside the corridor everywhere; the lap time is
mesh-converged; and rebuilding the path from the reported line and re-running the
existing solver on it reproduces the reported lap time.

**D8. Stages 2 and 3 are separate plans.** Stage 2 is the transient
(7DoF and 14DoF) minimum-time OCP. Stage 3 is certification and replay through
the forward driver. Neither starts until stage 1 closes its five checks, because
stage 1 is the harness that proves the transcription, the scaling and the IPOPT
configuration on a problem small enough to debug.

## Non-goals for stage 1

- No transient dynamics, no 14DoF, no suspension or tire-transient states.
- No forward-driver replay and no acceptance-gate integration.
- No track-width reconstruction from telemetry. Width is either supplied or
  declared constant with provenance `assumed`.
- No racing line export into `models/tracks/`. The rebuilt path is a stage 1
  artifact under the run directory, not a published track.
- No free 3D track. Elevation, camber and torsion are out of scope.

## Constraints inherited from the repository

- Full suite is `PYTHONPATH=src python3 -m unittest discover -s tests -q`.
- CasADi is optional and not on the default import path, so every module that
  imports it must do so inside a function or be excluded from default test
  discovery, matching how the retired OCP tests were handled.
- Existing track files, measured telemetry and archived validation artifacts are
  not modified.
