# Task 4 report — signed transient capability

## Delivered interface

Added `simplified_vehicle_models.formulations.signed_capability` with the opt-in, versioned representation `signed_transient_normalized_v1`.

* `SignedTransientCapability` stores physical curvature, a normalized `u` axis and `u=(v-speed_floor)/(Vdom(k)-speed_floor)`, rather than falsely exposing normalized values as m/s speed axes.
* `domain_speed(k)`, `limits(v,k)`, and `validate_physical_query(...)` take physical SI queries and reject out-of-domain values. The chart uses Vdom defined by existence of **some signed feasible acceleration**, not an ax=0 steady hold.
* `signed_feasible_interval(...)` uses bounded scans plus inward endpoint root refinement on original NumPy `evaluate_equilibrium`; it rejects disconnected sampled acceleration sets and never fills no-feasible cells with zero.
* `build_signed_transient_capability(...)` samples signed speed-domain existence, applies declared endpoint/domain buffers, records actual model hash/settings/diagnostics, and validates reconstructed bilinear intervals on every cell interior and edge stencil. Any failed original-physics endpoint explicitly refuses the build. Sampling is evidence, not a continuum proof.
* `save_signed_transient_capability` writes a separate exclusive-create archive including representation, physical axes, source hash, settings and validation diagnostics. It cannot overwrite an archive. Legacy six-key capability caches and source XML/CSV are untouched.
* `symbolic_lookup` supplies the CasADi physical `(v,k)` lookup adapter. The spline OCP uses explicit representation dispatch; its physical state remains m/s. Legacy `build_ocp` explicitly rejects the signed type rather than silently treating `u` as a speed table. `line_validation` also dispatches signed physical limits.

## Physical regression results

Using the original compiled IEM24 planar point-mass view and `evaluate_equilibrium`:

| physical query | result |
|---|---|
| k=0, v=33.339080185 m/s | signed upper bound about -0.052829 m/s² (ax=0 infeasible) |
| k=0, v=35.067452826 m/s | signed upper bound about -4.051241 m/s² |
| k=-0.0606381562, v=17.495664306 m/s | no scanned feasible acceleration interval; excluded |
| same k, v=16.5 m/s | interval about [-2.42649, 0.47861] m/s² |

The focused live interval tests run in about 1.4 s. A diagnostic five-curvature/u-five live chart attempt (Vmax=20 m/s, scan=161, 35 speed-domain samples) took 6.7 s and correctly **refused**: at `k=-0.0455, u=0.25, v=5.0300478`, interpolated `[-12.27249, 3.77225]` had an infeasible original-physics endpoint. This is recorded as an honest limitation rather than a fake-valid broad chart. Further adaptive per-cell interval intersection/refinement is required before a broad real chart can be used in production.

## Tests

* `PYTHONPATH=src python3 -m unittest tests.test_signed_capability -v` — pass; CasADi parity skipped because CasADi is unavailable in this environment.
* `PYTHONPATH=src python3 -m unittest tests.test_point_mass_capability tests.test_qss_spline_ocp tests.test_line_validation -q` — pass (43 tests, 12 environment skips).
* Syntax compilation for modified modules passes.

## Limitations

The build-time endpoint/edge/interior stencil is conservative numerical certification only, not a formal continuum proof. The present builder refuses a demonstrated broad coarse chart instead of relaxing limits or manufacturing zero holds. No controller integration, TV formulation, source-data mutation, or full OCP matrix was added.

## Completion update — conservative real builder

The builder now constructs a usable live physical chart rather than stopping at vertex checks. For every cell it evaluates the original NumPy equilibrium at the complete construction edge/interior stencil, takes that cell's common inner signed interval, and propagates the requirement to all four shared vertices (lower bound by maximum, upper by minimum). It then revalidates exact bilinear reconstruction. Failure triggers bounded true-axis refinement and increasing domain backoff; it refuses with the offending domain/common-interval/revalidation probe after the limit. It never supplies an artificial zero hold.

### Measured narrow live chart

* Settings: `curvature=[-.01, 0, .01] 1/m`, `u=linspace(0,1,9)`, `speed_floor=.5 m/s`, `maximum_speed=35.067452826 m/s`, acceleration scan `[-30,10] m/s²`/161 points, 20 endpoint refinements, 61 speed-domain points, endpoint buffer `.002 m/s²`, domain backoff `.002 m/s`, and a 4×4 per-cell construction stencil.
* This completes in about 24 seconds including an independent test stencil. The chart metadata records actual axes/backoff/refinement, construction and reconstruction-validation stencil counts, distinct live interval evaluations, and mean conservative interval loss.
* An independent 37-by-19 `(u,k)` stencil (703 physical points, denser and different from construction) checks both reconstructed acceleration endpoints against original `evaluate_equilibrium`, including `u=1`, interior-`u`, edge-`k`, and interior-`k`; all pass. At the straight high-speed boundary it retains transient braking: approximately `[-15.2138, -4.0532] m/s²`, preserving the required negative maximum acceleration.
* The spline numeric forward/backward seed now dispatches signed charts through physical `.limits()` (and only clamps a seed query to its local domain); a CasADi `solve_spline_line` smoke test with a nonconstant-domain signed fixture passes. Legacy surfaces retain their old lookup path.
* Numeric/symbolic parity applies to valid physical `(v, k)` queries. Numeric `.limits()` rejects out-of-domain inputs. The symbolic lookup clamps only temporary rejected NLP iterates to avoid table extrapolation; explicit curvature and speed-domain constraint rows make that clamped value non-legal and it is not alternate capability semantics.

### Known limitation

The delivered live chart is intentionally narrow in curvature. A wider diagnostic around negative curved domain regions still discovers a **domain** failure at `k=-.05, u=1` when a linear interpolated Vdom (even after `.4 m/s` backoff) is `19.51896 m/s` with no feasible signed interval. That is correctly refused rather than inwardly fabricating acceleration bounds. Broader track coverage needs additional curvature knots and/or a domain representation that can conservatively bound this Vdom shape; it is not claimed here.

## Tests

With actual CasADi:

* `PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_signed_capability tests.test_qss_spline_ocp.SplineTranscriptionTest.test_signed_surface_solves_nonconstant_curvature_seed -v` — 8 tests pass.

## Commit

Pending final focused regression run and review.
