# SDD ledger — plan: docs/superpowers/plans/2026-10-26-lap-suite-line-convergence-tracking.md

Baseline868683e; existing unrelated report/plans preserved. Spec is direct user request plus plan acceptance criteria.

## Preflight interface/conflict scan

| Tasks | Shared interface/files | Finding/ruling |
|---|---|---|
|1 internal|profile seeds vs strict support|Map knots are optional proposals; strict edge/interval validation remains mandatory. Reserve room; no eviction of incumbent/previous graph nodes.|
|2 internal|line budgets/cache vs diagnostics|Return best validated incumbent with explicit budget status; cached exact evaluations are not new independent convergence evidence.|
|3 internal|convergence measurements|Separate stationarity/profile/mesh/control-basis statuses; same-grid repeats cannot certify mesh convergence.|
|4 internal|new driver mode vs existing PreviewDriver|Preserve compatibility default, explicitly select model-aware mode; no hidden speed cap.|
|1→2|CoupledProfileSettings/warm seeds|Additional initial seed controls must propagate via immutable settings; no map identity changes.|
|1→3|profile diagnostics/uncertainty|Seed omissions reported; interval component unresolved stays typed.|
|1→4|process deadline|All later physical runs use run_with_deadline wrapper≤1800s.|
|2→3|line result and budget reason|Convergence driver consumes validated result and treats budget stops inconclusive.|
|2→4|optimized geometry/speed plan|Controller follows actual selected line, not source centreline.|
|3→4|lap suite result|Completion, containment, convergence and tracking metrics stored separately.|

Ruling: initial optional seeds may be bounded/sampled without supporting every dense map knot—the graph still strictly evaluates actual edges and refinement keeps interval-component semantics. Reserve room rather than evict nodes already used by an incumbent; if mandatory witnesses exhaust budget, return visible uncertainty. Cost if wrong: lower resolution/longer refinement, never relaxed feasibility.
Ruling: proceed with staged implementation directly because user explicitly asked to resolve and carry on; no redundant execution-choice pause.
Tool limitation: subagent API exposes no model selector; use available agent configuration and focused briefs. Existing worker reused because child limit prevents spawning fresh agents.

## Progress

- Stage1 seed implementation delegated to25bd8bd4-fc97-4d1d-8375-3899bbd534b0; owns profile contracts/solver/tests.
- Parent implemented process-group hard deadline CLI,3 tests pass including grandchild termination at0.25s. Larger-than1800s limit rejected unless explicit approval flag.
