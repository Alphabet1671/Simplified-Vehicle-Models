# TTC Tire Correlation GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a native PyQt6 TTC correlation GUI with repository-compatible steady/transient UniTire auto-fitting, manual sliders, explicit cross-tire provenance and private data handling.

**Architecture:** Qt-free data, model editing, fitting and replay services under `tools/tire_gui/` drive thin Qt widgets. Every prediction uses the existing steady/transient runtime; immutable revisions isolate background fit candidates from manual edits. Export separate steady and transient XMLs, with private sessions and diagnostics.

**Tech Stack:** Python >=3.12, NumPy >=2.0, SciPy >=1.11, PyQt6 >=6.5, Matplotlib >=3.8, unittest.

**Spec:** [TTC tire correlation GUI design](../specs/2026-10-26-ttc-tire-correlation-gui.md). Read the full spec before execution; it contains investigated run identities, sample counts, candidate transient windows, equations, restrictions and UI behavior.

## Global Constraints

- Python >=3.12; keep Qt, SciPy and plotting optional for base-library users.
- The model is a deliberate hybrid: **43075 R20 lateral / 43100 R20 longitudinal proxy**.
- Do not change an existing vehicle's tire or overwrite the current fitted models as a side effect.
- No raw samples, fitData arrays, plots, or private manifests belong in Git. Do not use `git add -f`.
- Default all measured-data plots, sessions, caches, residuals, optimizer logs and reports to ignored `tire_fits_out/`; automated tests use synthetic data only.
- Use the repository steady and transient evaluators; do not port another force law into the GUI.
- Calendar acquisition date remains explicitly unknown; unavailable longitudinal dynamics cannot be presented as fitted.
- Native PyQt6 appearance, not a new theme or web frontend.

---

## Preparation already completed (not GUI implementation)

Local branch: `feature/ttc-tire-correlation-gui`. Existing unrelated edits remain untouched. [Ignore rules](../../../.gitignore) now cover `/private/ttc/` and `/tire_fits_out/`. Four TTC runs (7/8/9/72) plus the requested MAT seed are copied locally with checksum verification. Private manifest lives under the ignored root. There are no commits from this planning session. Do not stage the user's other pending files.

## File map and responsibility

All new implementation files below are proposed, not currently implemented.

| New path | Responsibility |
|---|---|
| `tools/tire_gui/__init__.py` | Qt-free package |
| `tools/tire_gui/contracts.py` | Immutable run, selection, event, revision and result records |
| `tools/tire_gui/data.py` | MAT loading, metadata, SI conversion and gap splitting |
| `tools/tire_gui/segments.py` | Exact upstream compatibility selection and explicit experimental selections |
| `tools/tire_gui/parameters.py` | Canonical XML names/units, parameter groups/bounds, MAT mapping |
| `tools/tire_gui/model.py` | Validated snapshots, undo/redo, predictions and stale-result checks |
| `tools/tire_gui/fit.py` | Two-stage steady compatibility objective and optimizer |
| `tools/tire_gui/events.py` | Transient candidate events, quality/identifiability gates |
| `tools/tire_gui/transient.py` | Runtime-based time integration and transient optimization |
| `tools/tire_gui/save.py` | Private sessions and validated XML save-as |
| `tools/tire_gui/app.py` | Optional dependency check, CLI and QApplication lifecycle |
| `tools/tire_gui/qt/__init__.py` | Qt UI package |
| `tools/tire_gui/qt/main_window.py` | Shared session, file actions, settings and tab wiring |
| `tools/tire_gui/qt/parameter_panel.py` | Reusable precise spinbox/slider/lock controls |
| `tools/tire_gui/qt/steady_tab.py` | Four steady plots and selectors |
| `tools/tire_gui/qt/transient_tab.py` | Events, co-timed traces and transient controls |
| `tools/tire_gui/qt/diagnostics_tab.py` | Metrics, convergence, provenance and exclusions |
| `tools/tire_gui/qt/worker.py` | QThread worker lifecycle, cancellation and revision-safe results |
| `tests/test_tire_gui_*.py` | Synthetic-only tests, split by task below |
| `docs/dev/tire-correlation-gui.md` | Launch, data privacy, fit workflow and limitations |

Modify [project extras](../../../pyproject.toml), [the documentation entrypoint](../../../README.md), and optionally add a short pointer in [the steady validation report](../../validation/unitire-simple.md) explaining that this is a new correlation tool, not retrospective validation of the existing fit. Do not change the model runtime or its XML schema in this project.

## Task 1: Typed private-data ingestion and exact compatibility selections

**Files:** create `contracts.py`, `data.py`, `segments.py`, `__init__.py`; create `tests/test_tire_gui_data.py`; modify [project extras](../../../pyproject.toml).

**Interfaces:** define the following public contracts. Arrays are copied and made nonwriteable at construction; metadata mappings are copied. `channels` uses SI keys `time`, `alpha`, `kappa`, `fz`, `fx`, `fy`, `mz`, `speed`, `omega`, `ia`, `pressure`; optional channels are omitted, not fabricated.

```python
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping
import numpy as np

@dataclass(frozen=True)
class RunData:
    run_id: int
    tire_id: str
    channels: Mapping[str, np.ndarray]
    metadata: Mapping[str, str]
    raw_indices: np.ndarray

@dataclass(frozen=True)
class SampleSet:
    alpha: np.ndarray
    kappa: np.ndarray
    fz: np.ndarray
    target: np.ndarray
    segment_ids: np.ndarray

@dataclass(frozen=True)
class SteadySelection:
    lateral: SampleSet
    longitudinal: SampleSet
    combined_fx: SampleSet
    combined_fy: SampleSet
    mz: SampleSet
    belt_speed: float

# data.py
# load_run(path: Path) -> RunData
# normalize_channels(raw: Mapping[str, np.ndarray], units: Mapping[str,str])
#     -> dict[str, np.ndarray]
# contiguous_slices(time: np.ndarray, maximum_gap: float = .02) -> tuple[slice,...]
# segments.py
# matlab_segment_indices(start: int, end: int, maximum_count: int = 180) -> np.ndarray
# compatibility_selection(lateral: RunData, longitudinal: RunData) -> SteadySelection
```

- [ ] Write failing tests for unit conversion and MATLAB inclusive/rounding semantics:

```python
class DataTests(unittest.TestCase):
    def test_matlab_indices(self):
        np.testing.assert_array_equal(matlab_segment_indices(1, 6, 3), [0, 3, 5])
    def test_gap_is_not_interpolated(self):
        blocks = contiguous_slices(np.array([0., .01, .02, 1.13, 1.14]))
        self.assertEqual([(s.start, s.stop) for s in blocks], [(0, 3), (3, 5)])
    def test_signs_and_units(self):
        raw = {'SA': np.array([3.]), 'FZ': np.array([-660.]),
               'V': np.array([36.]), 'N': np.array([60.]), 'SL': np.array([.1])}
        units = {'SA':'deg','FZ':'N','V':'kph','N':'rpm','SL':'dimensionless'}
        out = normalize_channels(raw, units)
        self.assertAlmostEqual(out['alpha'][0], -np.pi/60)
        self.assertEqual(out['fz'][0], 660.)
        self.assertEqual(out['speed'][0], 10.)
        self.assertAlmostEqual(out['omega'][0], 2*np.pi)
        self.assertEqual(out['kappa'][0], .1)
```

- [ ] Run `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_tire_gui_data.py' -v`; expect missing module/API failures.
- [ ] Implement MAT loading from channel descriptors, explicit errors for missing/ambiguous units or channels and invalid ET. The compatibility mode can read steady arrays without requiring ET, but transient processing cannot. Use the exact source tables in `UnitireFit_Simple_Auto.m:505–565`; preserve only IA=0 rows by default. MATLAB indexing implementation:

```python
count = end - start + 1
if start < 1 or count <= 0 or maximum_count < 2:
    raise ValueError('invalid one-based inclusive segment')
values = (np.arange(start, end + 1) if count <= maximum_count
          else np.unique(np.floor(np.linspace(start, end, maximum_count) + .5)))
return values.astype(np.int64) - 1
```

- [ ] Add optional extra without changing the base dependencies:

```toml
tire-gui = ["PyQt6>=6.5", "scipy>=1.11", "matplotlib>=3.8"]
```

- [ ] Extend synthetic tests to select tables from generated arrays of sufficient length: assert lateral/Mz 1080, longitudinal 720, combined 1440; reject short inputs rather than silently truncate. Assert two different tire identities survive selection provenance. Run the targeted suite green; verify `git check-ignore private/ttc/TTC/B2356run7.mat tire_fits_out/session.json` and that `git ls-files private/ttc tire_fits_out` is empty.
- [ ] Commit only this task's files and the reviewed ignore rules: `git add .gitignore pyproject.toml tools/tire_gui/__init__.py tools/tire_gui/contracts.py tools/tire_gui/data.py tools/tire_gui/segments.py tests/test_tire_gui_data.py && git commit -m "feat: add private TTC ingestion and compatibility segments"`.

## Task 2: Repository-model snapshots and precise parameter editing

**Files:** create `parameters.py`, `model.py`, `tests/test_tire_gui_model.py`; extend `contracts.py`.

**Interfaces:** `load_snapshot(steady_xml: Path, transient_xml: Path | None = None) -> ModelSnapshot`; `import_mat_seed(path: Path, template: ModelSnapshot) -> ModelSnapshot`; `with_parameter(snapshot, name: str, value: float) -> ModelSnapshot`; `predict_steady(snapshot, samples: SampleSet, belt_speed: float) -> TireForces`. Use existing `load_model`, `compile_unitire_simple`, `compile_unitire_transient`, `UniTireSimpleParameters` and `TransientTireParameters`.

```python
@dataclass(frozen=True)
class ModelSnapshot:
    revision: int
    steady: UniTireSimpleParameters
    transient: TransientTireParameters
    values: Mapping[str, float]  # canonical flat XML names, including transient
    provenance: Mapping[str, str]

@dataclass(frozen=True)
class ParameterSpec:
    name: str
    unit: str
    lower: float
    upper: float
    group: str
    default_locked: bool
```

- [ ] Write failing tests using the existing lab XML as an ordinary model fixture (no raw measurements). Example:

```python
class ModelTests(unittest.TestCase):
    def test_edit_is_immutable(self):
        old = load_snapshot(Path('models/tires/hoosier-43075-r20-simple.xml'))
        new = with_parameter(old, 'mu_0_y', old.values['mu_0_y'] + .1)
        self.assertEqual(new.revision, old.revision + 1)
        self.assertNotEqual(old.steady.mu_0_y, new.steady.mu_0_y)
    def test_bad_value_refused(self):
        old = load_snapshot(Path('models/tires/hoosier-43075-r20-simple.xml'))
        with self.assertRaises(ValueError):
            with_parameter(old, 'trail_lead_ratio', 1.1)
```

- [ ] Run model tests red. Implement canonical flatten/unflatten using the exporter field mapping in [the existing exporter](../../../tools/model_data/export_unitire_simple_tire.py#L52-L159); keep tuple coefficient order. Copy the mapping into a focused public adapter, not a dependency on private exporter internals. Do not require `fitReport` in imported MATs, and do not put fitData in a model snapshot.
- [ ] Recompile edits in memory through the existing model/compiler contract and check finite positive stiffness/relaxation over the selected operating load range. Preserve numerical floor parameters and lock unobservable coefficients by default. Implement compatibility omega as `abs((1+kappa)*belt_speed/max(polyval(radius, fz/f_z_rated),1e-4))` before calling `evaluate_unitire`.
- [ ] Add tests for all six polynomial vectors, aliases, full-precision round trips, MAT synthetic import with no report, direct evaluator/prediction equality and stale transient provenance after a steady edit. Run data and model tests green.
- [ ] Commit scoped files: `git add tools/tire_gui/parameters.py tools/tire_gui/model.py tools/tire_gui/contracts.py tests/test_tire_gui_model.py && git commit -m "feat: add validated tire parameter snapshots"`.

## Task 3: Faithful two-stage steady automatic fitting

**Files:** create `fit.py`, `tests/test_tire_gui_fit.py`; extend `contracts.py`.

**Interfaces:** `force_cost(residuals: tuple[np.ndarray, ...]) -> float`; `fit_steady(snapshot: ModelSnapshot, data: SteadySelection, locked: frozenset[str], cancel: Callable[[], bool], progress: Callable[[dict], None]) -> FitResult`. Define `FitCancelled(Exception)` in `contracts.py`. `FitResult` contains `source_revision: int`, `candidate: ModelSnapshot`, `metrics: Mapping[str,float]`, `converged: bool`, `warnings: tuple[str,...]`, and `status_by_parameter: Mapping[str,str]`. Locks are excluded from optimizer vectors, not overwritten afterward.

- [ ] Write objective tests first:

```python
class ObjectiveTests(unittest.TestCase):
    def test_separate_group_rms(self):
        self.assertEqual(force_cost(tuple(np.array([v]) for v in (1.,2.,3.,4.))), 8.25)
    def test_empty_group_is_explicit(self):
        with self.assertRaises(ValueError):
            force_cost((np.array([]),) * 4)
```

- [ ] Run fit tests red. Implement objective and logistic-bounded Nelder–Mead via SciPy `minimize(method='Nelder-Mead')`: maxiter 800, maxfev 4000, xatol/fatol 1e-5. Use `scipy.special.expit` for stable bound transforms. Preserve exact source force bounds: triplets lower `[0,-20000,10000]`, upper `[20000,20000,120000]`; e_1 [-.5,4]; each mu0 [.5,4], mus [.2,3], h [.02,4], vm [.02,3]. Trail pairs: d_x_0 [0,.3], d_e [-.1,.1], d_1/d_2 [0,8]. Do not silently clamp an imported seed in the live model; clamp only the optimizer start and disclose it.

```python
def force_cost(residuals):
    if len(residuals) != 4 or any(r.size == 0 for r in residuals):
        raise ValueError('four nonempty residual groups required')
    return sum(w * float(np.sqrt(np.mean(r*r)))
               for w, r in zip((1.,1.,.75,.75), residuals))
```

- [ ] Implement force stage then trail stage with existing runtime predictions and the spec's normalization. Cancellation checks precede each expensive prediction/objective call. Progress emits only scalar metrics and iteration counts; no samples are logged. Keep best finite candidate on nonconvergence, labeled not converged. Do not apply it automatically.
- [ ] Add synthetic tests: true parameters give zero residual; perturb one unlocked mu value and show improved objective, locks remain bitwise unchanged, cancellation raises `FitCancelled`, invalid/NaN prediction fails cleanly, boundary and nonconvergence flags are preserved. Fit each synthetic case with a small active subset rather than asserting recovery of 23 correlated parameters.
- [ ] Run the fit/data/model suites green. Local-only acceptance: compare reference MAT fitData selection and objective values to compatibility loading; report tolerances and differences privately. No TTC residual vector may become a tracked fixture.
- [ ] Commit: `git add tools/tire_gui/fit.py tools/tire_gui/contracts.py tests/test_tire_gui_fit.py && git commit -m "feat: implement staged steady tire correlation"`.

## Task 4: Time-domain replay and identifiable transient auto-fit

**Files:** create `events.py`, `transient.py`, `tests/test_tire_gui_transient.py`; extend `contracts.py`.

**Interfaces:** `candidate_events(run: RunData) -> tuple[TransientEvent,...]`; `simulate_event(snapshot: ModelSnapshot, event: TransientEvent, maximum_step: float) -> np.ndarray` (columns Fx,Fy,Mz, same rows as event input); `fit_transient(snapshot, events: tuple[TransientEvent,...], locked: frozenset[str], cancel, progress) -> FitResult`.

```python
@dataclass(frozen=True)
class TransientEvent:
    run: RunData
    start: int
    stop: int  # exclusive
    baseline_stop: int  # exclusive, within event
    approved: bool
    axis: str  # 'lateral' or 'longitudinal'
    exclusions: tuple[str, ...]
```

- [ ] Write failing gate tests using synthetic `RunData`:

```python
class EventTests(unittest.TestCase):
    def test_unapproved_event_cannot_fit(self):
        run = make_step_run()  # local test helper, defined below
        event = TransientEvent(run, 0, len(run.raw_indices), 20, False, 'lateral', ())
        seed = load_snapshot(Path('models/tires/hoosier-43075-r20-simple.xml'))
        with self.assertRaisesRegex(ValueError, 'approved'):
            fit_transient(seed, (event,), frozenset(), lambda: False, lambda p: None)

def make_step_run():
    t = np.arange(200) * .01
    z = np.zeros_like(t)
    channels = dict(time=t, alpha=np.where(t < .2, 0., .03),
                    kappa=z.copy(), fz=z+660, speed=z+1, omega=z+5,
                    ia=z.copy(), pressure=z+82.7, fx=z.copy(), fy=z.copy(), mz=z.copy())
    return RunData(0, 'synthetic', channels, {'source':'synthetic'}, np.arange(t.size))
```

- [ ] Run transient tests red. Candidate detection splits on ET gaps >.02 s and condition changes, uses an SA change >=1 degree over <=1 s as an initial **candidate** detector, and requires baseline >=.2 s, IA within .25 degree of zero, stable baseline load within 10%, speed within 10%, pressure within 3 kPa. These are editable screening defaults, not a claim about the protocol. Reject missing channels rather than infer stable conditions. Require explicit approval and record reasons for rejected events.
- [ ] Implement replay using `steady_rolling_state`, `evaluate_transient_unitire` and `solve_ivp`; interpolate commands only inside a contiguous event and split integration at command knots. Use measured V and N after verified SI conversion, never compatibility constant speed. Respect runtime speed floor (currently 0.5 m/s), even though historical module prose mentions 0.1. Reduce max step to demonstrate convergence; do not replace the state model with a generic fitted exponential.
- [ ] Fit lateral scale, then trail parameters; fit x scale only for accepted longitudinal excitation. Use bounds and normalization in the spec, fixed steady coefficients/k_cx/k_cy, per-event equal weighting, a bounded `least_squares` residual and finite-difference sensitivity. Mark SVD rank deficient when singular values fall below `max(s)*1e-6`; condition >1e6 and response time below two sample intervals are warning/failure gates, not confidence estimates. Freeze and label unidentified coefficients; manual editing stays available.
- [ ] Generate synthetic targets by `simulate_event` at known parameters, then test one-parameter recovery with a perturbed seed, no changes to locked/unexcited x scale, no integration across gaps, missing Mz blocks trail fit, and convergence when max step is halved. Add nonconstant-load replay and sign checks against the steady limit.
- [ ] Run transient and existing runtime tests green: `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test*unitire*.py' -v` plus the new transient test module. Locally review run-7 candidate windows from the spec; do not preapprove them from plateau times alone.
- [ ] Commit: `git add tools/tire_gui/events.py tools/tire_gui/transient.py tools/tire_gui/contracts.py tests/test_tire_gui_transient.py && git commit -m "feat: add gated transient tire correlation"`.

## Task 5: Safe paired XML export and private sessions

**Files:** create `save.py`, `tests/test_tire_gui_save.py`.

**Interfaces:** `save_session(snapshot: ModelSnapshot, output_dir: Path, stem: str, *, include_transient: bool, overwrite: bool = False) -> tuple[Path,...]`; `load_session(path: Path) -> ModelSnapshot`. Extend the session JSON with selections, locks, per-parameter fit status and fingerprints without changing the runtime XML schema. Keep measured arrays out of XML comments.

- [ ] Write failing tests that save to a temporary directory, compile both outputs and compare parameters:

```python
class SaveTests(unittest.TestCase):
    def test_save_refuses_collision(self):
        seed = load_snapshot(Path('models/tires/hoosier-43075-r20-simple.xml'))
        with tempfile.TemporaryDirectory() as folder:
            save_session(seed, Path(folder), 'candidate', include_transient=True)
            with self.assertRaises(FileExistsError):
                save_session(seed, Path(folder), 'candidate', include_transient=True)
    def test_path_escape_refused(self):
        seed = load_snapshot(Path('models/tires/hoosier-43075-r20-simple.xml'))
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                save_session(seed, Path(folder), '../escape', include_transient=False)
```

- [ ] Run save tests red. Serialize canonical name/unit/value rows, unique model IDs, mixed-tire provenance and provisional transient status. Reuse comment-preserving XML helpers where compatible; do not couple the tire service to vehicle editing state. Validate with actual compilers before publication. Save paired files in a temporary sibling directory; final bundle is a new directory named by `stem`. On failure retain existing bundle unchanged and clean up only verified temporary paths.
- [ ] Default GUI output to ignored `tire_fits_out/`; show a privacy warning on alternate locations and require explicit confirmation. Protect source XMLs and `models/` from accidental overwrite; public export of coefficient-only artifacts is a separate explicit user action after licensing review. `overwrite=True` requires the GUI confirmation and safe rollback; never broad-delete an output directory.
- [ ] Add tests for malformed IDs, units/full-precision round-trip, symlink escape, interrupted pair write, private manifest separation, stale steady fingerprint, unknown session version and default provisional x status. Run save suite green.
- [ ] Commit: `git add tools/tire_gui/save.py tests/test_tire_gui_save.py && git commit -m "feat: save private tire fits as validated XML bundles"`.

## Task 6: Native PyQt tabs, shared sliders and asynchronous fitting

**Files:** create `app.py`, all `qt/` files from the map, `tests/test_tire_gui_qt.py`.

**Interfaces:** `ParameterPanel.set_snapshot(snapshot)`, signal `parameterChanged(str,float)` and `locksChanged(object)`; `FitWorker` signals `progress(object)`, `finished(object)`, `failed(str)`, `cancelled()` and method `cancel()` backed by a thread-safe Event; `MainWindow.open_path(Path)` and `MainWindow.apply_candidate(FitResult) -> bool`.

- [ ] Write offscreen Qt tests first; skip only this module when optional dependencies are unavailable. Test a numeric edit rather than relying on screenshots:

```python
class QtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
    def test_stale_candidate_refused(self):
        window = MainWindow()
        window.open_path(Path('models/tires/hoosier-43075-r20-simple.xml'))
        old = window.snapshot
        window.set_parameter('mu_0_y', old.values['mu_0_y'] + .1)
        candidate = FitResult(old.revision, old, {}, True, (), {})
        self.assertFalse(window.apply_candidate(candidate))
        window.close()
```

- [ ] Run `QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_tire_gui_qt.py' -v`; expect missing UI/API failures. Implement the spec layout using sibling GUI conventions, not a stylesheet redesign.
- [ ] Bind every slider and numeric spinbox to the same `with_parameter` path, using signal blockers to prevent loops. Preserve full spinbox precision; slider maps an integer 0..10000 into the selected range. Debounce plotting only, not snapshot updates:

```python
value = lower + slider_value / 10000 * (upper - lower)
# QTimer(singleShot=True), interval 75 ms; restart on every edit.
# Nonpositive ranges cannot use logarithmic slider mapping.
```

- [ ] Add the four steady plots and transient elapsed-time/event plots using Matplotlib FigureCanvasQTAgg. Share load colors; label tire/axis, units, excluded samples and unknown date. Show candidate progress and apply/discard separately. Diagnostics includes fit status per coefficient, RMSE, convergence, holdout metrics and bounds/identifiability warnings.
- [ ] Put optimizers in QThread workers with immutable source snapshots. Emit Qt signals only; never manipulate widgets from workers. A cancel request sets an Event checked inside objectives. Shutdown cancels and waits without forcibly terminating threads; stale results cannot mutate the window. Disable duplicate-fit launch while one is active, but permit manual edits with revision invalidation.
- [ ] Implement CLI `PYTHONPATH=src:. python3 tools/tire_gui/app.py [steady.xml]`; add `TIRE_GUI_QUIT_AFTER_MS` for timed smoke-test exit, analogous to `VEHICLE_GUI_QUIT_AFTER_MS` in the existing vehicle launcher. Missing dependencies produce an installation hint (`python3 -m pip install -e '.[tire-gui]'`) without a traceback from importing Qt in core modules.
- [ ] Add tests for spinbox/slider consistency, locks, undo/redo, cancel-without-mutation, fresh candidate apply, settings restore, missing data, close while worker running, and separate transient tab with disabled unidentifiable x fitting. Smoke-launch offscreen; manually compare native appearance to the vehicle GUI and verify plots resize legibly. Store any measurement screenshots only in ignored output.
- [ ] Commit only UI/tests: `git add tools/tire_gui/app.py tools/tire_gui/qt tests/test_tire_gui_qt.py && git commit -m "feat: add native tire correlation GUI"`.

## Task 7: End-to-end verification, local TTC acceptance and user documentation

**Files:** create `tests/test_tire_gui_integration.py`, `docs/dev/tire-correlation-gui.md`; modify [README](../../../README.md) with a short linked entrypoint.

- [ ] Write a synthetic integration test for XML load → sample generation → steady fit → transient replay/fit → manual edit → paired save → compiler reload. Assert provenance and parameter states survive round-trip; a steady edit invalidates transient fitted status. Do not make the test depend on `/Users/...` or real MAT availability.
- [ ] Run the integration test red, implement only missing wiring in the owning modules, then run it green. Keep fixes scoped; do not change tire physics to improve correlation.
- [ ] Run all new tests and existing tire/model/GUI suites, then the full repository command:

```bash
QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_tire_gui*.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -q
git diff --check
git ls-files private/ttc tire_fits_out
```

Expected: new tests pass; no whitespace errors or tracked private data. Record unrelated baseline failures separately rather than claiming a passing full suite. A GUI test skip is not a GUI pass.

- [ ] Perform local-only TTC acceptance: load requested MAT seed (tire/fitData, no fitReport), verify compatibility sample counts, run both steady stages, inspect each dimensioned residual and proxy label, select and review run-7 transitions with their actual baseline/gaps/speed. Run lateral/trail fitting only if quality gates pass. Demonstrate unidentified longitudinal coefficient handling rather than inventing run 73. Save a private acceptance report with convergence/metrics and the unknown calendar-date disclosure.
- [ ] Write the user guide with exact install/launch commands, private data layout, initial seed choices, two-stage fit behavior, slider/lock/reset semantics, transient event approval and identifiability, separate XML output, unsupported camber behavior, source-vs-road scaling and export licensing review. Link it from README; keep README short.
- [ ] Commit the final scoped code/test/docs changes after reviewing `git diff --cached --name-only`; exclude private data and unrelated preexisting edits. Suggested message: `docs: document and verify tire correlation workflow`.

## Final review checklist

- [ ] All spec acceptance criteria map to Tasks 1–7; both auto-fitting and manual control exist on both physics tabs.
- [ ] Run 7 attribution, candidate windows, gaps and unavailable date are accurate; no unverified event is preapproved.
- [ ] Run 72's 43100 / 7-inch rim and combined-slip proxy are never mislabeled as 43075.
- [ ] MATLAB compatibility preserves objective, indices, units and sample counts; optimizer trajectory parity is not claimed.
- [ ] Every exported model compiles, runtime physics is unchanged, and original files remain untouched.
- [ ] Every existing-file mention in user-facing docs resolves to its intended source; new-file paths remain unambiguous.
- [ ] No TTC raw data, arrays, plots, hashes/manifests or fit sessions were staged inadvertently.

## Execution choice

This plan is ready for either subagent-driven task execution with reviews after each task, or inline execution with checkpoints after Tasks 1–3, 4–5 and 6–7. The planning deliverable does not claim that the GUI has been implemented or that a transient fit is validated.
