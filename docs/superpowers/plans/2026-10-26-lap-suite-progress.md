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

Ruling: flying seam layers use their dedicated anchor discretization, not the33-value interior baseline grid. Mandatory seam endpoints and all valid warm seam suggestions must be included before selecting optional uniform anchors; if mandatory seam union exceeds max_seam_anchors or station budget, return typed overflow. Unioning33baseline values into default32seam cap would make every closed solve impossible. Cost if wrong: seam resolution remains explicit/finite, not unsafe support.

## Progress

- Stage1 seed implementation delegated to25bd8bd4-fc97-4d1d-8375-3899bbd534b0; owns profile contracts/solver/tests.
- Parent implemented process-group hard deadline CLI,3 tests pass including grandchild termination at0.25s. Larger-than1800s limit rejected unless explicit approval flag.
- Seed reviewer Important positional settings compatibility fixed by appending fields. Parent caught standing optional nodes/domain snap filter/global optional budget issues; implementer fixes in progress.
- Physical accepted-map2m probe advanced past original seed overflow, internal validated lap61.48s, but public original-order check rejected2segments. Diagnosis: rotating by cumsum changes edge ds last bits, dense ax faces amplify. Ruling: preserve exact original physical segment lengths through rotation and use same lengths in edge checks/validator/timing, not broaden tolerance or disable map masks. Cost if wrong: broader YawPath interface requires regression coverage; physical semantics preserved.
- Probe serializer failed on ProfileValidation dataclass once; fixed with strict dataclass/enum/nonfinite handling and test.
- Accepted dense-map physical probes at2m now pass independent original-order validation: MIS61.4706s,v10.2302..25.2006,wall40.02s; FSC51.9168s,v9.3547..20.0714,wall27.95s. Both resolutionFalse; no fine-grid/lap-controller claims.
- Stage1 committed4d7454c after61focused tests9.66s; reviewer found mandatory warm seam removal. Fix b39132e adds dedicated seam policy, correct speed→q conversion, atomic matching anchors/global cap checks.59focused tests8.43s pass. Reviewer SPEC+QUALITY approved, no Important findings.
- Task1: complete (commits4f8f99f..b39132e, review clean). Stage1b cfb4b7a graph vector reductions approved initial review; second predecessor min-reduction removes last target loop.26tests6.77s pass; MIS2m total14.37s graph1.97s (was40s graph28). No physics/time change.
- Parent convergence contracts694d5d8:8tests distinct mesh/basis/profile/optimizer statuses; lap-plan artifact300a909:2tests integrity/identity+strict load. Suite fixedMIS/FSC2m saves/reloads reusable plans; replays never repeat optimizer.
- Stage2 5a62416: fixes seeds-beforewindows/globallegacycount/entryclock/private budget unwind.6behavioraltests4.55s+6originalfunctional157.9s pass guarded; reviewer SPEC+QUALITY APPROVED noImportant. Physical MIS40mcontrols2mgrid24eval900s line study underway.
- Task2 implementation reviewed; physical evidence pending. Stage3 MIS fixed2m61.47vs1m64.17 (+4.39%); FSC51.92vs54.34 (+4.67%); speed maxchanges5.83/6.19m/s. Convergence report correctly INCONCLUSIVE (coarse profile unresolved), not converged. Need finer/resolution controls, no false readiness.
- Tracking new explicit model-aware mode keeps PreviewDriver unchanged. Static drag/assembled drive/brake inversion+physicalfactory/steering-stop8tests. Baseline replay300s validation timeout; modelaware step-size failure evenpedal. MIS1m diagnostic steering-bound error now inwardstopguard; FSC1m integration reached validation but330s exhausted;850svalidation retryunder950souter underway. No controller successclaim.
- Actual MIS25control2mline search wall900s/5profilecalls selected61.4600 versusfixed61.4706, statusbudget_exceeded, unconverged. FourQPseeds geometryrejected. Parent diagnosed QP2*Ntrajectory fullstation quadrature repeated4blends; geometry-only+sharedlinearization fix underway with equivalence tests. No practical line improvement claimyet.
- Local oracle3409d11 fixes dense global-polynomial cancellation,44tests18.99s; physical0.5m now no numerical unresolved but300sfrontier refinementbudget; warmcoarse suggestions run underway. Resolution remains inconclusive.
- Reviewer approved Stage3/4 code+failure diagnostics noImportant; physical controller/convergence validation still unfinished.
