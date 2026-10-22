# TV-Aware Minimum-Lap-Time Solver Specification

**Status:** design for the next solver step, 2026-10-21.

**Goal:** one minimum-lap-time solver for a 2D track with width that can plan either
without a controller in the loop or with a torque-vectoring (TV) / drive-control
algorithm in the loop, without changing the objective or the track representation.

## 1. Non-negotiable requirements

- The objective remains minimum lap time on the 2D track, using the lateral
  corridor `(n_min(s), n_max(s))` from the width data.
- The same path/offset unknowns and the same speed-state transcription are used
  for open and closed tracks. Closure is a boundary operator, not a separate
  model.
- The controller is not silently ignored. Every result records whether the
  controller was absent, sampled algebraically, or exercised in a replay loop.
- A controller-aware result is accepted only after its own replay/feasibility
  check. No lap-time gain is claimed from a capability surface that was not
  replayed.
- Existing track XML/CSV data is immutable. Assumed widths and nonphysical test
  surfaces are tagged and cannot count as event-valid results.

## 2. Why the controller location matters

A TV algorithm changes the achievable longitudinal/lateral force and yaw moment
at a given speed and path curvature. It also changes the line because the
optimal trade between corner speed, exit acceleration, and lateral placement
depends on what the controller can deliver.

Three planning modes are therefore distinct:

| Mode | Controller location | What the OCP sees | Valid claim |
|---|---|---|---|
| `ideal` | none | existing vehicle capability surface | baseline ideal-allocation line |
| `allocation` | algebraic allocator sampled during surface generation | controller-limited `a_min, a_max, v_ceiling` at `(v, κ)` | quasi-static TV-aware line |
| `closed_loop` | stateful controller exercised by a forward replay | surface plus iteratively corrected local speed caps | line verified against that controller replay |

`allocation` is only valid when the controller is memoryless and its allocation
can be summarized by `(v, κ)`. `closed_loop` is required for a controller with
integrators, rate limits, wheel-slip state, estimator state, or path-history
dependence. A stateful controller must not be approximated as a static surface
and then called “in the loop.”

## 3. Unified path formulation

Use one consistent offset spline, not node values plus an independently
reconstructed Hermite–Simpson offset.

- `n(s)` is a cubic spline on the track distance. Closed tracks use a periodic
  basis; open tracks use a natural cubic spline with free endpoint values.
- At every node and collocation point, `n`, `n'`, and `n''` come from the same
  spline.
- `den = 1 - κ0(s) n(s)`.
- Heading error: `ψ = atan(n' / den)`.
- Path/vehicle curvature: differentiate `atan(n'/den)` with respect to `s`.
- The only dynamic state is speed `v(s)`; acceleration `a(s)` is the control.
- Lap time: `T = ∫ ds / (v cos ψ)`.
- Speed dynamics: `dv/ds = a den / (v cos ψ)`.
- Constraints at nodes and collocation points:
  - corridor: `n_min(s) ≤ n(s) ≤ n_max(s)`;
  - fold: `den ≥ fold_guard`;
  - capability: `a_min(v, κ) ≤ a ≤ a_max(v, κ)` and `v ≤ v_ceiling(κ)`;
  - curvature domain and any steering/kinematic limits.
- Closed tracks: periodic spline coefficients; open tracks: clamped spline and
  free endpoint offset (or an explicit endpoint policy recorded in settings).

This removes the mixed representation identified in the boundary
investigation. The spline resolution is independent of the speed mesh.

## 4. Controller and capability contracts

### 4.1 Controller query

A planning-facing controller answers one question: for a demanded operating
point, can it deliver the requested longitudinal acceleration without violating
its own allocation or actuator limits?

```python
@dataclass(frozen=True, slots=True)
class PathDemand:
    speed: float
    curvature: float
    longitudinal_acceleration: float
    lateral_acceleration: float
    yaw_rate: float
    steering_angle: float = 0.0

@dataclass(frozen=True, slots=True)
class ControlResponse:
    feasible: bool
    longitudinal_acceleration: float
    yaw_moment: float
    saturation: tuple[str, ...] = ()

class DriveControlPolicy(Protocol):
    name: str
    provenance_hash: str
    def evaluate(self, demand: PathDemand) -> ControlResponse: ...
```

`steering_angle` is optional because the quasi-steady line solver does not carry
a steering state. A controller that needs steering must derive it from the
steady-state handling model or expose a replay adapter; it must not assume zero
steering silently.

### 4.2 Capability backend

```python
class CapabilityBackend(Protocol):
    def sample(self, grid: CapabilityGrid) -> SampledCapability: ...
```

`SampledCapability` holds the arrays used by `build_ocp` plus provenance:
vehicle/model hash, controller policy name/hash, mode, curvature limit, and
sampling resolution.

- `IdealCapabilityBackend` wraps the current `build_capability_surface`.
- `AllocationCapabilityBackend` calls `DriveControlPolicy.evaluate` at each
  grid point and finds the feasible acceleration interval. It assumes a
  deterministic, monotone-in-`a` allocation; the assumption is tested and
  recorded.
- `ClosedLoopCorrector` does not create a surface directly. It replays the
  current line with the stateful controller and adds local speed caps for the
  next OCP solve.

### 4.3 Replay contract

A replay consumes the rebuilt `PathCoordinates` plus the controller and returns
distance, achieved speed, feasible flag, lap time, and the controller’s
saturation events. The reference adapter wraps the existing forward driver where
possible; a production TV adapter implements the same protocol.

### 4.4 TV integration seam

A real TV algorithm is an integration input, not something this solver may
invent. It must supply:

- a `DriveControlPolicy` implementation for the `allocation` mode, with a stable
  `provenance_hash` and no hidden state (or an explicit declaration that it is
  only valid in `closed_loop`);
- a replay adapter for `closed_loop` that resets controller state per attempt,
  drives the candidate path, and reports wheel/force saturation honestly;
- the controller’s actuator limits and any state initialization in the
  provenance record.

The reference policies in this plan are test doubles. A green mock-policy test
does not validate a physical TV algorithm or claim a TV lap-time benefit.

## 5. Closed-loop correction

1. Solve the spline OCP with the current capability surface.
2. Rebuild the path and replay it with the stateful controller.
3. If replay is feasible and its speed/time agrees with the OCP within
   tolerance, accept the result.
4. Otherwise, set a local speed cap
   `v_cap(s) = min(v_ocp(s), v_replay(s)) - margin` where replay is slower or
   infeasible, and re-solve. Never raise the cap above the OCP value to hide a
   controller limitation.
5. Stop after a configured number of iterations. A non-converged correction is
   reported as conditional/failed, not accepted.

The OCP gains an optional per-node `speed_cap` argument. This is a small,
auditable coupling to replay and preserves the minimum-time objective under the
controller’s demonstrated limits.

## 6. Provenance and acceptance

Every summary records:

- track source hashes, width provenance, and corridor margin;
- formulation/model, capability cache, mode, controller name/hash;
- spline type, control spacing, mesh ladder, solver tolerance;
- OCP lap time, replay lap time, maximum violations, iteration count;
- whether the controller was in the loop, and any saturated capability cells.

Acceptance gates:

- **Ideal parity:** ideal backend reproduces the existing solver on a synthetic
  circle/straight to `1e-6` before TV modes are trusted.
- **Allocation consistency:** `allocation` limits never exceed `ideal` at any
  cell; a mock policy with a known cap produces that cap.
- **Closed-loop replay:** final replay is feasible and within 1% of the OCP
  time, with no unreported saturation.
- **Geometry:** dense corridor containment between nodes, analytic derivative
  checks, and open/closed steady-circle parity.
- **Track matrix:** at least skidpad, acceleration, MIS, and FSC; failed
  convergence remains a reported failure.

## 7. Out of scope for this step

- A production TV calibration or a claim that a particular TV algorithm wins.
- Transient tire/thermal/energy effects beyond the existing quasi-steady model.
- Replacing the forward vehicle model with a full transient OCP.
- Changing existing track data or event-validity standards.

## 8. Risks

- A stateful controller may make the closed-loop fixed point oscillate; the
  iteration limit and cap monotonicity are required.
- A non-monotone allocation oracle can invalidate bisection; the sampler must
  detect and fall back to a scan.
- The spline path can be more sensitive near the corridor edge; fold and dense
  containment checks are mandatory.
- TV may reduce lap time on one track and increase it on another; results are
  per-configuration, not a universal “TV is faster” claim.
