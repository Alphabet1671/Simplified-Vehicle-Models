# Virtual-driver methodology comparison — current branch vs. two reference papers

> **Superseded recommendation (2026-10-25):** §6 below does not model yaw-moment capability, so it cannot reproduce the TV-vs-no-TV line-shape difference. The current recommendation is [virtual-driver proposal v2](virtual-driver-proposal-v2-2026-10-25.md). The paper analysis in §§1–5 still stands.

**Prepared:** 2026-10-25. **Branch / worktree:** `feature/minimum-time-line-2d-track` at `eb37c44` (main checkout).
**Question:** For a driver that navigates a 2D track with arbitrary left/right edges, which methodology gives the best algorithm efficiency and reproduction reliability, when the lap does not have to be the absolute minimum time?
**Type:** Engineering methodology review. This is not an MIS 2024 report or a qualification record. Statements about the papers come from their text. Statements about this repository come from its source and docs at the revision above; no solves were re-run for this review.

Papers reviewed:

- **[P1]** H. de Carvalho Pinheiro, S. Lubertino, M. Carello, *Path planning development for human-like virtual driver*, SAE 2023-36-0068 (local copy `~/Downloads/2023-36-0068.pdf`).
- **[P2]** M. Thommyppillai, S. Evangelou, R.S. Sharp, *Advances in the development of a virtual car driver*, Multibody Syst. Dyn. 22:245–267 (2009), DOI 10.1007/s11044-009-9164-4 (local copy `~/Downloads/s11044-009-9164-4.pdf`).

---

## 1. Executive summary

1. **The branch is solving a harder problem than the one stated.** Its main effort is a free-line, minimum-time direct-collocation OCP (CasADi/IPOPT) on top of a heavy qualification framework. After 355 commits ahead of `main` (2026-10-20 → 10-25), its own [status record](virtual-driver-all-track-status-2026-10-23.md) has **zero of six configurations qualified**. MIS runs time out, and the one FSC strict solve fails derivative-uncertainty gates at the ~1e-6 level. For a requirement of "a good, reproducible lap, not necessarily the minimum," most of that cost goes into guarantees nobody is asking for.
2. **Both papers decouple the problem; the branch's OCP does not.** Each paper solves one piece cheaply and makes no claim of global lap-time optimality:
   - [P1] makes a *path* by geometry: a parametric blend of the left and right edges plus a spline. It does no optimization at run time, and its five shape parameters are fitted offline.
   - [P2] *tracks* a given (x, y, t) path with offline linear-quadratic (LQ) optimal preview gains. At run time this costs one matrix–vector product per step.
3. **The repository already has a working decoupled pipeline.** The *forward-driver chain* is a centreline path, a point-mass forward/backward speed envelope, a `PreviewDriver`, and BDF integration. It records `usable = true` on MIS for both planar 7DoF (64.95 s lap) and vertical 14DoF (65.01 s lap) through cold→warm→refined→finer chains, with roughly 100–150 s of wall time per lap ([forward-usability-fixes](../validation/iem24-murphy/forward-usability-fixes/README.md)). Its main weakness for this problem is the **path**: it drives the centreline, so it does not use the edges.
4. **Recommendation:** Make a three-stage decoupled pipeline the primary product:
   - **Path:** edge-aware geometric racing line. Use a [P1]-style parametric line, or a minimum-curvature QP inside the existing corridor.
   - **Speed:** keep the existing point-mass forward/backward profile on that line.
   - **Tracking:** keep `PreviewDriver`, and optionally upgrade it toward [P2]'s LQ preview with gain scheduling.

   Keep the minimum-time OCP as an optional offline benchmark that reports how far a pipeline lap is from optimal. It should not sit on the critical path. §6 gives a concrete plan.

---

## 2. Current state of the worktree and its documentation

### 2.1 What the branch contains

| Area | Implementation | Documented status |
|---|---|---|
| Minimum-time line OCP (primary effort) | [`qss_spline_ocp.py`](../../src/simplified_vehicle_models/solvers/qss_spline_ocp.py) (1,436 lines): cubic-spline lateral offset n(s) in a normal-offset corridor; energy-state W = v² Hermite–Simpson transcription; signed capability chart; IPOPT. Plus [`qss_line_ocp.py`](../../src/simplified_vehicle_models/solvers/qss_line_ocp.py), [`capability_exchange.py`](../../src/simplified_vehicle_models/solvers/capability_exchange.py), [`adaptive_line.py`](../../src/simplified_vehicle_models/solvers/adaptive_line.py), [`closed_loop_line.py`](../../src/simplified_vehicle_models/solvers/closed_loop_line.py) | **HOLD, 0/6 configurations qualified** ([status](virtual-driver-all-track-status-2026-10-23.md)) |
| Independent validator | [`line_validation.py`](../../src/simplified_vehicle_models/solvers/line_validation.py): dense sampling, Richardson finite-difference speed derivatives, uncertainty propagation | FSC: 631,128 samples in 101 s; four 1e-6-level derivative-uncertainty gates fail |
| Orchestration / provenance | `minimum_time_run.py`, `run_manifest.py`, `convergence_policy.py`, `consumer_approval_record.py`, `refinement_comparison.py`, the matrix harness | The 12 most recent commits are all provenance, ledger or verifier hardening; about 100 of 355 commit subjects are "harden/record/provenance/verifier/schema"-type |
| Event / boundary | `standing_event.py`, `standing_straight_ocp.py`, new untracked `joint_event_contract.py` (an analytic constant-acceleration helper) | Restricted straight standing start only; U2 not qualified |
| Forward driver (secondary, older) | [`forward_driver.py`](../../src/simplified_vehicle_models/solvers/forward_driver.py): `build_speed_plan` (point-mass forward/backward envelope) plus `PreviewDriver` (curvature feedforward at a preview point, then heading/course/lateral/yaw feedback, pedal slew); `planar_forward.py`, `vertical_forward.py` | **`usable = true`** for MIS 7DoF and 14DoF chains ([README](../validation/iem24-murphy/forward-usability-fixes/README.md)) |
| Track edges | [`corridor.py`](../../src/simplified_vehicle_models/tracks/corridor.py): per-station `half_width_left/right` with a fold guard `1 − κn > 0`; legacy and signed-fitted policies | Arbitrary asymmetric edges are representable. The [gap audit](universal-2d-solver-gap-audit-2026-10-24.md) flags the legacy symmetric clamp and the untested width pinches |

### 2.2 Working-tree hygiene (observed, not changed)

- The modified files are the [roadmap](virtual-driver-all-track-research-roadmap.md) and the 10-23 plan. Untracked files include the [gap audit](universal-2d-solver-gap-audit-2026-10-24.md), the 10-24 plan and subplans, `joint_event_contract.py` and its test, and the input-reproduction note.
- Stray files sit at the repo root: `task-*.md` reports, `task_loaded_geometry_bound_probe.py`, a file named `x`, and a directory literally named `:.superpowers/`, which looks like a typo of `.superpowers/`. AGENTS.md puts task reports under `docs/superpowers/`. These should be moved or deleted before any commit.

### 2.3 What the documentation itself concludes

The [gap audit](universal-2d-solver-gap-audit-2026-10-24.md) and the [10-24 plan](../superpowers/plans/2026-10-24-universal-2d-solver.md) are candid about the following:

- The public runner *cannot* report an accepted result: basis convergence and the matched baseline are hard-wired as unavailable.
- The supported-track contract is implicit, and arbitrary cuts, initial poses and standing starts are unsupported.
- Closed-loop driving ("U3") is explicitly **deferred** and *not authorized* by the plan.

So the branch's documented deliverable is an *offline minimum-time planner*, not a driver. The driver the user describes is the deferred U3.

The roadmap (§7) also says *"No external paper has been assessed as part of this draft."* This review fills that gap for the two papers supplied.

---

## 3. The two papers in brief

### 3.1 [P1] Human-like path planning (SAE 2023-36-0068)

- **Goal:** reproduce a *human's* path, not a fast one. The fit metric is the mean point-wise distance to a driver-in-the-loop (DiL) driver's path (`Path_s`), not lap time.
- **Track input:** centreline plus width; left and right limits are built by perpendicular offset. Arbitrary edges substitute directly.
- **Vision model:** a 2D cone, 120° and 100 m, centred on the centreline and of fixed size. Two goal-selection logics: *First Curve* (aim at the next curve) and *Maximum Vision* (aim at the farthest visible point). They are blended by the **distance factor α ∈ [0,1]**.
- **Shape:** the target lateral position is a convex combination of the inner and outer limits, `β·RoadLimit_int + (1−β)·RoadLimit_ext`. **β₁** (widening, in-out) sets entry and exit; **β₂** (cutting) sets the mid-corner and apex.
- **Smoothing:** **s₁** and **s₂** set how gradual entry and exit are.
- **Path generation:** a Hybrid A* planner was tried and dropped as too expensive. Instead the path is a **4-point spline**: current position and heading, then goal position and heading. This follows Salvucci's ACT-R near/far-point model, **re-planned at 1 Hz**.
- **Calibration:** a brute-force grid of 5⁵ = 3,125 parameter sets. Best fit is about 0.95 m and 0.73 m mean distance to the two drivers, versus 2.90 m and 3.17 m for VI-Grade's VI-Driver.
- **Not covered:** speed or motion planning, actuation, closed-loop stability, and any check that the spline stays inside the edges. The θ / limit formulas are missing from the extracted text, and the evidence is one track (Calabogie) and two novice drivers.

### 3.2 [P2] Optimal linear preview control (Multibody Syst. Dyn. 2009)

- **Goal:** the tracking layer of a "practical virtual racing driver": fast computation that can still exploit the car.
- **Method:**
  - Linearize the full multibody car about a trim state.
  - Augment it with a shift-register model of the road preview (n points, x and y demands).
  - Solve one discrete algebraic Riccati equation (DARE) offline for the H₂/LQ gain `u = K z`. K₁ is state feedback; K₂ is preview gains on the shift-register states.
  - Inputs are throttle/brake and steer, with 2nd-order Butterworth filters standing in for driver bandwidth.
- **Run-time cost:** transform the preview points into the car frame each step, then do one matrix–vector product. NMPC is explicitly rejected as *"slow and extravagant"*, with no convergence guarantee.
- **Key properties:**
  - Preview gains decay to zero, so a finite horizon ("full preview", 500 points at 0.01 s in their example) captures nearly all the benefit.
  - Tightness is set by the Q/R weights.
  - Straight-running trims decouple lateral and longitudinal control; cornering trims couple them.
- **Results:** lane change with tracking error below 1.3 mm; a 140 m/250 m hairpin with braking at −0.04…0.16 m (x) and −0.04…0.06 m (y), using a single straight-running trim.
- **Not covered:** the reference *path and speed* (it must be supplied). Adaptation and gain scheduling across trims for limit handling are deferred to other work (refs [35, 36]). The car is a family saloon, and severity was deliberately limited.

---

## 4. How the current methodology maps to the papers

The driver task has four parts: **path**, **speed**, **tracking** and **verification**.

| Part | Current branch, primary (min-time OCP) | Current branch, forward-driver chain | [P1] | [P2] |
|---|---|---|---|---|
| Path | Optimized n(s) spline inside the corridor, coupled with speed | **Centreline** (no edge use) | Geometric edge blend (α, β, s) plus a 4-point spline at 1 Hz | Given |
| Speed | Optimized W(s) under the capability chart | Point-mass forward/backward envelope (live vehicle limits, lateral margin) | Not covered | Given (x, y as functions of time) |
| Tracking | None (U3 deferred) | `PreviewDriver`: single preview-point curvature feedforward plus hand-tuned gains | Not covered (the vision logic is the planner) | LQ preview, offline DARE, gain-scheduled per trim |
| Verification | Dense validator with 1e-6 tolerances, mesh/basis/seed convergence, manifests, approval records | Lateral error ≤ 0.25 m, periodicity, refinement tube, between-step interval certificate | Path distance to human | Frequency response, tracking error |
| Online cost | One large NLP per lap: FSC solve about 22 s, then 101 s validation; MIS times out | Speed plan O(N), then ODE integration | O(1) per 1 Hz replan | O(n_z) per step |
| Optimality claim | Local min-time (not certified) | None (conservative) | None (human-like) | Optimal *tracking* for a quadratic cost |

The branch's primary route is the only one of the four that **couples path and speed into one non-convex optimization**. That coupling is where almost all of its documented difficulties come from:

- seed dependence and timeouts,
- interpolated chart nonsmoothness,
- violations between collocation sites,
- derivative-uncertainty gates.

---

## 5. Comparative analysis

### 5.1 Algorithm efficiency

| Criterion | Min-time OCP (current) | Forward-driver chain (current) | [P1] geometric path | [P2] LQ preview |
|---|---|---|---|---|
| Problem class | Non-convex NLP with path inequality constraints | Two 1D sweeps, then ODE | Closed-form interpolation | Offline DARE, online linear algebra |
| Scaling with track length N | Superlinear in practice (sparse KKT plus more iterations); MIS exceeded its bounded budgets | Linear (sweeps) plus integration cost | Linear | Online independent of N; offline DARE ~O(n_z³) once per trim |
| Wall time, documented | FSC: about 22 s solve plus 101 s validation (5 free coefficients only); MIS: timeouts | MIS: about 100–150 s per lap for the 7DoF/14DoF plants. Dominated by the stiff plant, not the driver | Not reported; trivially cheap (the authors chose it *because* Hybrid A* was too costly) | Not reported; real-time by construction |
| Dependencies | CasADi, IPOPT, pinned venv | NumPy/SciPy | NumPy | NumPy/SciPy (`solve_discrete_are`) |
| Cost of re-running after a vehicle change | Rebuild the chart, then a full re-solve | Re-run the point-mass profile (fast) | None (geometry only) | Re-linearize and redo the DARE per trim (offline, seconds to minutes) |

Comments:

- **[P2]'s DARE size.** In [P2]'s example the augmented state z is the car states plus 2 × 500 preview states, so the DARE is about 1,000-dimensional. That is fine offline. It can be shrunk by using a coarser preview or by building a preview-gain table in distance rather than time.
- **[P1] is the cheapest path generator, but its 1 Hz replanning ties the path to the car's state.** The path then depends on the driver's history, and the planner must sit inside the simulation loop. For a pure "racing line" you can apply the same α/β/s blend *offline* along the whole track and skip the 1 Hz loop.
- **The current OCP spends most of its budget on verification, not optimization.** In the FSC run, validation took 4.5× as long as the solve. The 1e-6 (m/s)/m derivative gates are much tighter than any sensible lap-time tolerance; a 1e-3 relative lap-time target would be generous.

### 5.2 Reproduction reliability

Reliability here means two things: the same inputs give the same output, and a new track gives a valid output without manual rescue.

| Risk | Min-time OCP | Forward chain | [P1] | [P2] |
|---|---|---|---|---|
| Determinism | IPOPT is deterministic for fixed inputs, but the result depends on the seed, scaling, mesh, sites and runtime. The plan spends Tasks 1, 1A and 1B just on making those choices reproducible | Deterministic sweeps. The integrator is deterministic for a fixed rtol and step | Fully deterministic | Fully deterministic (closed-form gains) |
| Failure modes on a new track | Non-convergence, timeouts, local optima, between-site violations, chart-domain exits | Integrator rejection if the gains don't suit the track (for example the S-bend failure with lateral gain 3.0); path error above the gate | The spline can leave the edges (no constraint check), and the curve-detection logic can switch abruptly between goals | The linear gains lose quality near the friction limit; trim scheduling must be designed |
| Sensitivity to parameters | High (mesh, basis, seed, reserves, margins) | Medium (three steering gains, lateral margin) | Low to medium (five bounded, interpretable parameters) | Low (Q/R weights; preview length chosen by gain decay) |
| Arbitrary left/right edges | Supported in principle (corridor), but width pinches, asymmetry and cuts are open items in Task 3A | **Not used**: the centreline path ignores edges, apart from a lateral-error gate | **Native**: β is literally a blend of the two edges | N/A (tracks whatever path it is given) |
| Evidence of working | 0/6 qualified | MIS 7DoF/14DoF `usable = true` | Path fidelity on one track | Lane change and hairpin, with moderate severity |

**Reliability ranking for the stated goal:**
1. [P2]-style tracking and the existing forward chain: deterministic and already proven here.
2. [P1]-style geometry: deterministic, but needs an edge-containment check.
3. The coupled min-time OCP: most fragile.

### 5.3 Lap-time quality, under the relaxed requirement

- A **centreline** path (current forward chain) leaves real time on the table in every corner, because it throws away the available radius. This is the biggest cheap gain available.
- A **[P1]-style** line with β₁ near 0 (wide entry and exit) and β₂ near 1 (tight apex) gives an out-in-out line. It is human-like, though, not a fast line by construction. [P1] fitted it to *novice* drivers, so its tuned values (β₁ = 0.1, β₂ = 0.5) are not race settings and would need retuning against lap time.
- A **minimum-curvature QP** in the existing corridor gives a near-racing line. Minimize ∫κ² over n(s) with linearized curvature and `−w_left + margin ≤ n ≤ w_right − margin`; this is a convex, banded QP. It is the standard engineering compromise between geometric heuristics and full minimum time. Its single global optimum makes it **reproducible**, and it keeps the speed problem 1D.
- Measure the remaining gap, don't assume it. Run the existing OCP *once*, offline, on a track where it converges (for example FSC), and compare it with the pipeline lap. That is the right use of the OCP machinery.

### 5.4 Fit with what the branch already has

| Needed component | Already exists | Gap |
|---|---|---|
| Edge corridor with fold guard | `corridor.py`, signed-fitted policy | Width-pinch and breakpoint tests (already in Task 3A) |
| Offset-path geometry n(s) → XY, κ | `line_trajectory.py` (analytic reference/offset) | None for a fixed n(s) |
| Speed profile on an arbitrary path | `forward_driver._build_live_limit_speed_plan` → `forward_backward.solve_lap` | Feed it the *offset path's* curvature, not the centreline's |
| Tracking controller | `PreviewDriver`, `solve_forward`, planar and vertical plants | Optional: LQ preview / gain scheduling ([P2]) |
| Verification | Forward acceptance chain (periodicity, refinement tube, between-step) | Add an edge-containment check of the *driven* trajectory against the corridor |
| Path heuristic | — | New: [P1]-style blend and/or min-curvature QP (~a few hundred lines) |

---

## 6. Recommendation and suggested path

### 6.1 Primary pipeline (efficiency and reliability first)

1. **Line generator** (new, small, deterministic). Take the corridor edges and produce n(s). Two options:
   - **(a) Minimum-curvature QP** (default). Use SciPy, or OSQP if added, on the existing station grid. Add a safety margin of half the vehicle width plus a buffer, and optionally iterate 2–3 times with re-linearized curvature.
   - **(b) [P1]-style parametric line** (β₁, β₂, s₁, s₂ along detected corner segments). Use it when *human-like* behaviour is wanted, or as a cheap, explainable fallback.

   Both produce an n(s) that can be checked against the edges *exactly at every station before driving*. That check is what [P1] lacks; add it.
2. **Speed profile:** the existing live point-mass forward/backward solver on the line's curvature, with `lateral_margin` as the single knob for "how close to the limit".
3. **Tracking:** the existing `PreviewDriver` on the plant of choice. If tuning the gains per track becomes the reliability bottleneck, which the sweep history suggests it could, replace it with a [P2]-style LQ preview controller:
   - Offline DARE on a reduced planar model, for example a linear bicycle model plus steer filter, at a small grid of trims (speed × lateral acceleration).
   - Interpolate gains on line in that grid.
   - This swaps three hand-tuned gains for physically meaningful Q/R weights, with a principled preview length.
4. **Acceptance:** reuse the forward chain gates (lateral error, periodicity, refinement tube, between-step). Add an **edge-containment** check of the driven XY trajectory, including the vehicle footprint, and a **lap-time refinement** check at a tolerance that matches the relaxed requirement (for example ≤ 0.1 % across refinement levels).

### 6.2 Role of the existing min-time OCP

- **Benchmark only.** Run it offline where it converges to quantify the gap from optimal for the pipeline line. Report that gap; don't gate on it.
- Use the pipeline's line and speed as the **OCP seed**. That is a much better initializer than the current envelope guess and may by itself fix part of the documented convergence trouble.
- **Pause the qualification and provenance hardening work** (Tasks 1B and 6, consumer approval records, the comparison ledger) until a consumer actually asks for a bounded minimum-time claim. The 10-24 plan already says this governance is opt-in. The commit stream shows it is still consuming most of the effort.

### 6.3 Risks of the recommendation

- The decoupled pipeline is **not jointly optimal**, and how far it falls short depends on the track. Mitigation: measure the gap with the OCP benchmark (§6.2).
- The point-mass speed plan is steady-state. On a curvier racing line, transient limits may bite harder than on the centreline. Mitigation: the existing `lateral_margin` and the closed-loop correction pattern in `closed_loop_line.py` (lower the speed cap where the replay is slower) apply directly to the speed plan.
- [P2]'s linear gains degrade near saturation, so gain scheduling is needed for limit driving. [P2] defers the details to other publications. Keep `PreviewDriver` as the default until an LQ variant beats it on the same gates.
- [P1]'s vision-cone logic assumes the viewpoint is on the centreline and needs a corner-detection rule that the paper does not fully specify. Implement it offline along the whole track rather than in the loop.

---

## 7. Bottom line

| Method | Efficiency | Reproduction reliability | Edge use | Lap quality | Effort to deliver here |
|---|---|---|---|---|---|
| Current coupled min-time OCP | Low | Low (0/6 qualified, seed/mesh sensitive) | Yes (in principle) | Best (if converged) | High; still open |
| Current forward chain (centreline) | Medium (plant-dominated) | **High (usable on MIS 7DoF/14DoF)** | **No** | Conservative | Done |
| [P1] geometric path | **Very high** | High (deterministic; needs a containment check) | **Native** | Human-like, tunable | Low |
| [P2] LQ preview tracking | **Very high online** | **High** (closed-form gains) | N/A | Tracking only | Medium (linearization plus scheduling) |
| **Recommended:** min-curvature or [P1] line → point-mass speed → preview tracking | High | High | Yes | Near-racing line; gap measured against the OCP | Low–medium (reuses most existing code) |

For a driver that must navigate arbitrary left/right edges reliably and quickly, without the hard requirement of minimum lap time, the decoupled path → speed → tracking architecture is the better methodology. Both papers point to it, and most of it is already in this repository. The coupled minimum-time OCP should be demoted to an optional benchmark and seed consumer.
