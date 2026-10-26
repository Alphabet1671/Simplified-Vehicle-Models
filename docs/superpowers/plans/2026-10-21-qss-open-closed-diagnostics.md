# QSS Open/Closed Diagnostics Implementation Plan

> **For agentic workers:** Use the session's `subagent-driven-development` or `executing-plans` skill to execute task-by-task with review checkpoints.

**Goal:** Distinguish endpoint/topology sensitivity from size and geometry effects by cutting MIS and FSC open without altering their sampled geometry, and rank alternatives that keep one solver.

**Architecture:** A read-only diagnostic builds in-memory track views: original closed, same-length cut-open, and shorter open prefix. It uses the same per-node Hermite–Simpson NLP, capability surface, fixed characteristic scaling, initial-guess policy, and strict IPOPT options in all cases. It records failed iterates as diagnostics, never accepted solutions. No source XML/CSV is rewritten or revalidated as a newly measured open track.

**Tech Stack:** Python unittest, dataclasses, NumPy, CasADi/IPOPT; existing track loader and QSS NLP builder.

**Spec:** User request to investigate one solver for both topologies and cut open both MIS and FSC to separate closure from length.

## Global Constraints

- Preserve source track data; do not merge or push.
- Keep production solver settings and result validation unchanged.
- A same-length cut retains the terminal cross-section (even though its position coincides with the first); no wrapping or endpoint equality is imposed on the open view.
- Open start speed is an explicitly reported rolling-start diagnostic, not an event standing start. No launch or replay acceptance claim.
- Primary matrix uses no reduced offset basis and zero open-only regularization to avoid basis/objective confounds.
- Short prefixes also change geometry: convergence cannot prove length alone is causal.
- A failed bounded optimization does not demonstrate mathematical infeasibility.

## Task 1: Immutable paired track views

Files: create `tools/diagnostics/compare_qss_track_boundaries.py`; create `tests/test_qss_track_boundaries.py`.

- [ ] Add tests for identical closed/cut-open geometry, width values, length and source bytes; verify source closure flags remain unchanged.
- [ ] Implement `track_view(track, *, is_closed, length=None)` using `dataclasses.replace`. Same-length cuts retain all cross-sections; prefixes slice existing nodes, reset segment lengths and consistently slice path and widths.
- [ ] Reject invalid prefix lengths and attempts to call cropped geometry closed.
- [ ] Run `PYTHONPATH=src python3 -m unittest tests.test_qss_track_boundaries -v`.

## Task 2: Reproducible bounded solve matrix

Files: extend diagnostic runner and tests.

- [ ] Add tests verifying equal mesh, initial-guess policy and objective settings across the primary pair; check failed results cannot be labelled accepted.
- [ ] Implement `solve_diagnostic` around `build_ocp`, `initial_guess`, `_decision_scaling` and CasADi IPOPT using production exact-Hessian/adaptive settings. Record status, iterations, constraint/bound/equality/defect residuals and IPOPT reported dual residual. Call `_validate_solver_result` before setting `accepted=true`.
- [ ] Build full closed/full open/prefix open cases for each source. Verify NPZ provenance against the selected formulation.
- [ ] Add CLI `--track` (repeatable), `--formulation`, `--surface`, `--output`, `--spacing`, `--maximum-iterations`, `--start-speed`, `--prefix-length`, `--fallback-half-width`, `--margin`. Write input SHA256s, settings and measured results to JSON. Protect source/reference paths from summary overwrites.
- [ ] Run tests and CLI help, then a six-case bounded matrix with 5 m spacing, 200 iterations, strict 1e-6 tolerance, 10 m/s open rolling start and 200 m prefixes.

## Task 3: Interpret and review

Files: developer note and diagnostic regression tests.

- [ ] Interpret closed/full-open convergence together; compare short prefixes separately without asserting pure length causality.
- [ ] If necessary rerun only an informative capped pair with a larger but shared budget.
- [ ] Record single-formulation alternatives: consistent spline geometry with analytic derivatives; full-state collocation with controlled capability smoothing; multiple shooting with shared boundary operator.
- [ ] Obtain independent experimental-design/code review and fix actionable findings.
- [ ] Run focused regression tests and source hash checks; leave objective active unless verification and review are genuinely complete.
