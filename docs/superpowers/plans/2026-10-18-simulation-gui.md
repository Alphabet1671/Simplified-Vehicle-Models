# Simulation GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a cross-platform PyQt6 GUI that loads a full vehicle model, shows one dropdown for each component, lets the user swap components and sweep them in a study, and runs each case into a run record.

**Architecture:**
- A Qt-free core in `tools/sim_gui/` does the work: it scans the model catalogue, holds the vehicle configuration, writes variant XML, builds the study cases and runs them.
- A thin Qt layer in `tools/sim_gui/qt/` shows the core.
- Each case runs in its own process through `tools/sim_gui/run_case.py`. That script patches the runner's model specification and calls the existing `child()` function of `tools/run/run_iem24_forward_driver.py`. The runner file does not change.
- Each case writes the standard `run_record.npz` (specification `docs/superpowers/specs/2026-10-16-run-result-file.md`). A `study.json` index lists all cases for the post-processing GUI, which comes later.

**Tech Stack:** Python 3.12+, PyQt6 ≥ 6.5 (6.11.0 is installed), numpy, scipy, `unittest`, `xml.etree.ElementTree`.

**Spec:** This plan is the spec. The run record format is in [the run result file specification](../specs/2026-10-16-run-result-file.md).

## Global Constraints

- Change nothing in `src/`. Do not edit `tools/run/run_iem24_forward_driver.py`, `forward_driver.py`, `vertical_forward.py` or `planar_forward.py`. Their hashes are pinned in `tests/test_vertical_14dof_evidence.py`.
- Only `tools/sim_gui/qt/`, `tools/sim_gui/app.py` and `tests/test_sim_gui_qt.py` import PyQt6. Every other new module imports only the standard library, numpy and the repository.
- Add PyQt6 as an optional extra: `gui = ["PyQt6>=6.5"]` in `pyproject.toml`. Do not add it to the core dependencies.
- Make it cross-platform. Use `pathlib`, `os.pathsep`, `sys.executable` and `QProcess` or `subprocess` with argument lists. Never use a shell. Do not use POSIX-only calls such as signals, `fork` or `timeout`.
- Start every case process with the repository root as its working directory, and with `PYTHONPATH = <root>/src` + `os.pathsep` + `<root>`.
- Always record. Every case passes `--record`.
- Write all study output under `runs/` in the repository root by default. Add `/runs/` to `.gitignore`.
- The runnable models are the keys of `runner.MODELS`: `planar-7dof` and `14dof-simple`. A point-mass formulation can be opened, but the GUI cannot run it. Show the message `no forward runner for point-mass; select planar-7dof or 14dof-simple`.
- Write JSON with `indent=2, sort_keys=True, allow_nan=False`. Write files atomically: write `<name>.tmp`, then `os.replace`.
- Write the docs in ASD-STE100 style.
- Run tests as `MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.<module> -v`. Qt tests set `QT_QPA_PLATFORM=offscreen`, and skip when PyQt6 is not installed.

## Fixed Decisions

| Topic | Decision |
|---|---|
| Location | Package `tools/sim_gui/`. Start it with `PYTHONPATH=src:. python3 tools/sim_gui/app.py`. |
| Input file | **Open Vehicle Model** accepts a `vehicle-formulation` XML or a `vehicle-assembly` XML. A formulation sets the default model from its subtype. An assembly defaults to `planar-7dof`. |
| Dropdowns | One dropdown for each component role of the assembly. Powertrain and drivetrain assemblies also get one dropdown for each child role, labelled `powertrain › power-plant`. That is two levels, no more. |
| Candidates | Scan all `*.xml` under `models/`, reading root attributes only. A candidate fits a slot when its descriptor capability equals the capability of the current component. When that capability is `None`, it must have the same `(type, subtype)` instead. **Browse…** adds a file from outside `models/`, under the same rule. |
| Other dropdowns | **Model**: `planar-7dof`, `14dof-simple`. **Track**: every `track-model/closed-xy-profile` under `models/tracks/` (the runner accepts only this subtype). |
| Parameters | Read-only. A table shows the parameters of the selected slot's component. The GUI does not edit parameters in this version. |
| Variant files | Write each case to `cases/case_NNN/model/`: `vehicle.xml`, `formulation.xml`, and `powertrain.xml` / `drivetrain.xml` when a child slot changed. Rewrite each `ref` and `path` as a POSIX path relative to the variant file, or as an absolute path when no relative path exists (for example, another drive on Windows). Set `id` to `<original id>.case-NNN`. Put the origin in a top XML comment. |
| Formulation | Copy the source formulation. Set its subtype to the selected model. Keep its `requires` list only when the subtype did not change. Otherwise drop it, so the default list applies. |
| Process model | The GUI is the parent and has no runner parent process. It makes `attempt_<time_ns>/`, starts `run_case.py --case-dir D --child-attempt A <runner args>` with one `QProcess`, sends stdout and stderr to `A/run.log`, stops the process at the timeout, and writes `A/parent.json` in the runner's format. This makes one process for each case, so Stop works on every platform. |
| Parallel cases | A spinbox from 1 to `os.cpu_count()`. Default: `max(1, min(4, os.cpu_count() // 2))`. |
| Progress | Read the JSON lines from the child's stdout. Progress = `rhs_time / duration`, clipped to [0, 1]. The child prints a line about every 5 s. |
| Sweep modes | **One-at-a-time**: change each dimension away from the baseline, one dimension per case, and add one baseline case. **Full factorial**: the Cartesian product. Dimensions: any slot, `model` and `track`. |
| Case limits | Warn above 50 cases. Refuse above 500. |
| Validation | **Build Cases** compiles each case's variant formulation with `compile_formulation` in a worker `QThread`. A case that fails gets the status `invalid` and the error text, and it does not run. |
| Run settings | Default preset "Reference lap": `--planner live --maximum-speed 60 --pedal-travel-time 0.175 --maximum-step 0.008 --rtol 1e-05 --method BDF --spacing 0.5 --duration 300`. Timeout default is 300 s for `planar-7dof` (also its maximum) and 3600 s for `14dof-simple`. The GUI does not support `--warm-start` in this version. |
| Statuses | `pending`, `invalid`, `running`, `completed`, `rejected`, `timed_out`, `cancelled`, `failed`. |
| Persistence | **Save Study** writes the definition to `<name>.study.json`. **Open Study** reads it back. `QSettings("Simplified-Vehicle-Models", "sim-gui")` stores the recent files and the window geometry. |
| Headless | `tools/sim_gui/run_study.py --definition X.study.json [--output-root runs] [--parallel N]` runs the same study without Qt. Use it for batches and tests. |

## Output Layout

```
runs/<study-name>_<YYYYmmdd-HHMMSS>/
  study.json                  # index, schema "sim-study-v1", rewritten at each status change
  definition.study.json       # copy of the definition that made this study
  cases/case_000/
    case.json                 # case id, label, model, track, choices, runner argv
    model/vehicle.xml, formulation.xml[, powertrain.xml, drivetrain.xml]
    attempt_<ns>/             # runner child output: run_record.npz, summary.json, trace.csv,
                              # pedal.csv, run.log, parent.json, source_snapshot/
```

`study.json`:

```json
{"schema": "sim-study-v1", "name": "...", "created_utc": "...", "repository_commit": "<sha or null>",
 "definition": "definition.study.json",
 "cases": [{"id": "case_000", "label": "baseline", "model": "planar-7dof",
            "track": "models/tracks/2024_michigan_endurance_closed.xml",
            "choices": {"tire": "models/tires/iem24_murphy_real_unitire.xml"},
            "directory": "cases/case_000", "status": "completed", "error": null,
            "attempt": "cases/case_000/attempt_1792...", "run_record": "cases/case_000/attempt_1792.../run_record.npz",
            "summary": {"status": "Duration_Limit", "lap_completed": false, "usable": false,
                        "lap_time": null, "simulated_seconds": 0.5, "elapsed_wall_seconds": 10.8}}]}
```

All paths in `study.json` are POSIX and relative to the study directory. Choice paths are relative to the repository root, or absolute when the file is outside the repository.

## File Map

| File | Responsibility |
|---|---|
| `tools/sim_gui/__init__.py` | `REPOSITORY_ROOT`, `child_environment()` |
| `tools/sim_gui/catalogue.py` | Scan model XML, find the candidates for a slot |
| `tools/sim_gui/vehicle.py` | `VehicleConfiguration`: slots, choices, swap, nested refresh |
| `tools/sim_gui/variant.py` | Write the variant XML for one case |
| `tools/sim_gui/settings.py` | `RunSettings` ⇄ runner argv, with validation |
| `tools/sim_gui/study.py` | Definition, sweep expansion, `study.json` index |
| `tools/sim_gui/execute.py` | Prepare a case, build its command, parse progress, finish a case |
| `tools/sim_gui/run_case.py` | Child entry point: patch `runner.MODELS`, call `runner.child` |
| `tools/sim_gui/run_study.py` | Headless study runner |
| `tools/sim_gui/qt/main_window.py` | Window, toolbar, docks, tabs |
| `tools/sim_gui/qt/model_panel.py` | Model, track and slot dropdowns; parameter table |
| `tools/sim_gui/qt/settings_tab.py` | Run settings form |
| `tools/sim_gui/qt/study_tab.py` | Sweep dimensions and case preview |
| `tools/sim_gui/qt/queue.py` | `CaseQueue` (QProcess pool) and the queue table |
| `tools/sim_gui/qt/workers.py` | `ValidationWorker` on a `QThread` |
| `tools/sim_gui/app.py` | `main()`: start the QApplication |
| `tests/test_sim_gui_core.py` | Tests for the Qt-free core |
| `tests/test_sim_gui_run.py` | One real 0.3 s case, run end to end |
| `tests/test_sim_gui_qt.py` | Offscreen widget tests |
| `docs/dev/simulation-gui.md` | User and developer doc |

---

### Task 1: Package skeleton and model catalogue

**Files:**
- Create: `tools/sim_gui/__init__.py`, `tools/sim_gui/catalogue.py`
- Modify: `pyproject.toml` (add the `gui` extra), `.gitignore` (add `/runs/`)
- Test: `tests/test_sim_gui_core.py`

**Interfaces:**
- Produces:
  - `REPOSITORY_ROOT: Path`
  - `child_environment() -> dict[str, str]`
  - `ModelEntry(path: Path, model_id: str, model_type: str, subtype: str, capability: str | None, kind: str)`, frozen
  - `scan_catalogue(root: Path = REPOSITORY_ROOT / "models") -> tuple[ModelEntry, ...]`
  - `read_entry(path: Path) -> ModelEntry`, which raises `ValueError`
  - `fits(slot_entry: ModelEntry, candidate: ModelEntry) -> bool`
  - `candidates(slot_entry, catalogue) -> tuple[ModelEntry, ...]`, sorted by path
  - `tracks(catalogue) -> tuple[ModelEntry, ...]`

- [ ] **Step 1: Write the failing tests.**

```python
class CatalogueTests(unittest.TestCase):
    def setUp(self):
        self.catalogue = scan_catalogue()
        models = REPOSITORY_ROOT / "models"
        # Key by models-relative path: several folders hold a file named iem24_murphy_real.xml.
        self.by_rel = {e.path.relative_to(models).as_posix(): e for e in self.catalogue}

    def rel(self, entries):
        return {e.path.relative_to(REPOSITORY_ROOT / "models").as_posix() for e in entries}

    def test_every_model_xml_is_listed(self):
        self.assertEqual(len(self.catalogue), len(list((REPOSITORY_ROOT / "models").rglob("*.xml"))))

    def test_tire_candidates_share_the_capability(self):
        slot = self.by_rel["tires/iem24_murphy_real_unitire.xml"]
        self.assertEqual(self.rel(candidates(slot, self.catalogue)),
                         {"tires/iem24_murphy_real_unitire.xml", "tires/hoosier-43075-r20-simple.xml"})

    def test_capability_none_matches_type_and_subtype(self):
        slot = self.by_rel["controllers/placeholder_2024_pedal_controller.xml"]
        self.assertEqual(self.rel(candidates(slot, self.catalogue)),
                         {"controllers/iem24_murphy_real.xml", "controllers/placeholder_2024_pedal_controller.xml"})

    def test_only_signed_tracks_are_offered(self):
        self.assertNotIn("tracks/2024_michigan_endurance_v11.xml", self.rel(tracks(self.catalogue)))

    def test_child_environment_puts_src_first(self):
        parts = child_environment()["PYTHONPATH"].split(os.pathsep)
        self.assertEqual(parts[:2], [str(REPOSITORY_ROOT / "src"), str(REPOSITORY_ROOT)])
```

- [ ] **Step 2: Run** `python3 -m unittest tests.test_sim_gui_core -v`. Expect `ModuleNotFoundError: tools.sim_gui`.

- [ ] **Step 3: Implement.**

```python
# tools/sim_gui/__init__.py
"""Simulation GUI: a Qt-free core and a thin PyQt6 layer (tools/sim_gui/qt)."""
from __future__ import annotations
import os
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]

def child_environment() -> dict[str, str]:
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join([str(REPOSITORY_ROOT / "src"), str(REPOSITORY_ROOT)])
    environment.setdefault("MPLCONFIGDIR", str(Path(os.environ.get("TMPDIR", "/tmp")) / "mplcfg"))
    return environment
```

```python
# tools/sim_gui/catalogue.py (essential parts)
def read_entry(path):
    path = Path(path).resolve()
    try:
        root = ElementTree.parse(path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"cannot read {path}: {error}") from error
    if root.tag != "model":
        raise ValueError(f"{path}: the root element must be model")
    descriptor = component_descriptor(root.get("type", ""), root.get("subtype", ""))  # ValueError if unknown
    return ModelEntry(path, root.get("id", ""), descriptor.model_type, descriptor.subtype,
                      descriptor.capability, descriptor.kind)

def scan_catalogue(root=REPOSITORY_ROOT / "models"):
    entries = []
    for path in sorted(Path(root).rglob("*.xml")):
        try:
            entries.append(read_entry(path))
        except ValueError:
            continue          # skip; the Model panel log shows the skipped files
    return tuple(entries)

def fits(slot_entry, candidate):
    if slot_entry.capability is not None:
        return candidate.capability == slot_entry.capability
    return (candidate.model_type, candidate.subtype) == (slot_entry.model_type, slot_entry.subtype)
```

`scan_catalogue` also returns the skipped files through a second function, `skipped_files(root) -> tuple[tuple[Path, str], ...]`. The GUI log uses it. `tracks()` keeps the entries with `model_type == "track-model"` and `subtype == "closed-xy-profile"`.

- [ ] **Step 4: Run the tests.** Expect PASS.
- [ ] **Step 5: Commit** `feat: add simulation GUI model catalogue`.

---

### Task 2: Vehicle configuration

**Files:** Create `tools/sim_gui/vehicle.py`. Test: `tests/test_sim_gui_core.py`.

**Interfaces:**
- Consumes: `ModelEntry`, `read_entry`, `fits` (Task 1).
- Produces:
  - `Slot(key: str, label: str, parent: str | None)`. The key is `"tire"`, or `"powertrain/power-plant"` for a child.
  - `VehicleConfiguration`, frozen, with the fields `source: Path` (the file that was opened), `formulation: Path | None`, `assembly: Path`, `model: str`, `choices: dict[str, Path]` (slot key → component XML).
  - `VehicleConfiguration.open(path) -> VehicleConfiguration`
  - `.slots() -> tuple[Slot, ...]`, in assembly order, with each child right after its parent
  - `.with_choice(key, path) -> VehicleConfiguration`, which raises `ValueError` when `fits` is false. When the key is a parent, it replaces that parent's child choices with the children of the new assembly.
  - `.with_model(name) -> VehicleConfiguration`, which raises `ValueError` for a name not in `runner.MODELS`.
  - `.changed_from(baseline) -> tuple[str, ...]`
  - `.parameters(key) -> list[tuple[str, float, str]]`

- [ ] **Step 1: Write the failing tests.**

```python
FORMULATION_14 = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
PLANAR_FORMULATION = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"

class VehicleTests(unittest.TestCase):
    def test_open_formulation_lists_roles_and_children(self):
        configuration = VehicleConfiguration.open(FORMULATION_14)
        self.assertEqual(configuration.model, "14dof-simple")
        self.assertEqual([s.key for s in configuration.slots()],
            ["tire", "aero", "powertrain", "powertrain/power-plant", "powertrain/energy-storage",
             "drivetrain", "drivetrain/transmission", "drivetrain/controller",
             "suspension", "damper-front", "damper-rear", "tire-transient"])

    def test_open_assembly_defaults_to_planar(self):
        configuration = VehicleConfiguration.open(REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml")
        self.assertIsNone(configuration.formulation)
        self.assertEqual(configuration.model, "planar-7dof")

    def test_swap_rejects_a_wrong_capability(self):
        configuration = VehicleConfiguration.open(FORMULATION_14)
        with self.assertRaises(ValueError):
            configuration.with_choice("tire", REPOSITORY_ROOT / "models/aero/murphy_2024.xml")

    def test_swapping_a_parent_refreshes_its_children(self):
        configuration = VehicleConfiguration.open(FORMULATION_14).with_choice(
            "powertrain", REPOSITORY_ROOT / "models/powertrains/murphy_2024.xml")
        child = ElementTree.parse(REPOSITORY_ROOT / "models/powertrains/murphy_2024.xml").find(
            "./components/component[@role='power-plant']").get("ref")
        expected = (REPOSITORY_ROOT / "models/powertrains" / child).resolve()
        self.assertEqual(configuration.choices["powertrain/power-plant"], expected)
        self.assertEqual(configuration.changed_from(VehicleConfiguration.open(FORMULATION_14))[0], "powertrain")

    def test_point_mass_formulation_opens_but_names_no_runner(self):
        configuration = VehicleConfiguration.open(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_point_mass.xml")
        self.assertEqual(configuration.model, "point-mass")
        self.assertFalse(configuration.runnable)
```

- [ ] **Step 2: Run the tests.** Expect `ImportError`.
- [ ] **Step 3: Implement.**
  - Parse with `ElementTree`. Resolve each `ref` against the parent file's folder.
  - A formulation must have exactly one `<source ref>`, which gives the assembly.
  - `model` = the formulation subtype, or `"planar-7dof"` for an assembly.
  - `runnable` = `model in runner.MODELS`. Import the runner lazily: `importlib.import_module("tools.run.run_iem24_forward_driver")`.
  - Children come only from roles whose entry has `model_type` `powertrain-assembly` or `drivetrain-assembly`.
  - `with_choice` checks `fits(read_entry(current), read_entry(new))`.
  - `parameters(key)` reads `./parameters/parameter` with name, value and unit.
- [ ] **Step 4: Run the tests.** Expect PASS.
- [ ] **Step 5: Commit** `feat: add vehicle configuration with component swaps`.

---

### Task 3: Variant XML writer

**Files:** Create `tools/sim_gui/variant.py`. Test: `tests/test_sim_gui_core.py`.

**Interfaces:**
- Consumes: `VehicleConfiguration` (Task 2).
- Produces: `write_variant(configuration, directory: Path, case_id: str) -> VariantFiles(formulation: Path, vehicle: Path)`.

- [ ] **Step 1: Write the failing tests.**

```python
class VariantTests(unittest.TestCase):
    def test_baseline_variant_compiles_to_the_same_components(self):
        configuration = VehicleConfiguration.open(FORMULATION_14)
        with tempfile.TemporaryDirectory() as d:
            files = write_variant(configuration, Path(d), "case_000")
            base = compile_formulation(FORMULATION_14).manifest
            case = compile_formulation(files.formulation).manifest
            self.assertEqual([(c.role, c.path) for c in case.loaded_components if c.role != "powertrain_drivetrain"],
                             [(c.role, c.path) for c in base.loaded_components if c.role != "powertrain_drivetrain"])

    def test_tire_swap_reaches_the_compiled_model(self):
        hoosier = REPOSITORY_ROOT / "models/tires/hoosier-43075-r20-simple.xml"
        configuration = VehicleConfiguration.open(PLANAR_FORMULATION).with_choice("tire", hoosier)
        with tempfile.TemporaryDirectory() as d:
            files = write_variant(configuration, Path(d), "case_001")
            roles = {c.role: c.path for c in compile_formulation(files.formulation).manifest.loaded_components}
            self.assertEqual(roles["tire"], hoosier.resolve())
            self.assertEqual(ElementTree.parse(files.vehicle).getroot().get("id"), "iem24-murphy-real-planar.case-001")

    def test_changed_child_writes_a_nested_assembly(self):
        configuration = VehicleConfiguration.open(PLANAR_FORMULATION).with_choice(
            "powertrain/power-plant", REPOSITORY_ROOT / "models/power-plants/placeholder_2024_motor.xml")
        with tempfile.TemporaryDirectory() as d:
            write_variant(configuration, Path(d), "case_002")
            self.assertTrue((Path(d) / "powertrain.xml").is_file())
            self.assertFalse((Path(d) / "drivetrain.xml").exists())

    def test_model_change_drops_the_requires_list(self):
        configuration = VehicleConfiguration.open(FORMULATION_14).with_model("planar-7dof")
        with tempfile.TemporaryDirectory() as d:
            root = ElementTree.parse(write_variant(configuration, Path(d), "case_003").formulation).getroot()
            self.assertEqual(root.get("subtype"), "planar-7dof")
            self.assertEqual(root.findall("requires"), [])
```

- [ ] **Step 2: Run the tests.** Expect `ImportError`.
- [ ] **Step 3: Implement.**

```python
def _reference(target: Path, directory: Path) -> str:
    try:
        return Path(os.path.relpath(target, directory)).as_posix()
    except ValueError:                      # another drive on Windows
        return target.resolve().as_posix()

def _rewrite(source: Path, directory: Path, case_id: str, replace: dict[str, Path]) -> ElementTree.ElementTree:
    tree = ElementTree.parse(source)
    root = tree.getroot()
    root.set("id", f"{root.get('id')}.{case_id.replace('_', '-')}")
    for element in root.iter():
        for attribute in ("ref", "path"):
            value = element.get(attribute)
            if value is None:
                continue
            target = replace.get(element.get("role")) if element.tag == "component" else None
            element.set(attribute, _reference(target or (source.parent / value).resolve(), directory))
    return tree
```

- Write order: the nested assemblies (only when a child key changed), then `vehicle.xml` (it points at the nested files), then `formulation.xml`.
- Start each file with `<!-- Generated by tools/sim_gui for <case_id> from <source path>. Do not edit. -->`, written before the root element.
- Use `tree.write(path, encoding="UTF-8", xml_declaration=True)`.
- For an assembly input, make the formulation from scratch: `<model schema-version="1.0" id="<assembly id>.<case>-formulation" type="vehicle-formulation" subtype="<model>"><source ref="vehicle.xml"/></model>`.

- [ ] **Step 4: Run the tests.** Expect PASS.
- [ ] **Step 5: Commit** `feat: write per-case variant vehicle XML`.

---

### Task 4: Run settings

**Files:** Create `tools/sim_gui/settings.py`. Test: `tests/test_sim_gui_core.py`.

**Interfaces:**
- Produces:
  - `RunSettings`, a frozen dataclass with these fields: `duration=300.0`, `timeout_seconds=300.0`, `maximum_speed=60.0`, `lateral_acceleration: float | None = None`, `pedal_travel_time=0.175`, `preview_seconds=0.15`, `heading_gain=1.5`, `lateral_gain=1.5`, `maximum_step=0.008`, `rtol=1e-5`, `method="BDF"`, `spacing=0.5`, `planner="live"`, `record_precision="float32"`, `quasi_static_attitude=False`.
  - `RunSettings.for_model(model) -> RunSettings`, which sets the timeout to 300 or 3600.
  - `runner_argv(settings, model, track: Path) -> list[str]`, which always contains `--record`.
  - `validate(settings, model, track) -> None`, which calls `runner.parse_args(argv)` and turns `SystemExit` into `ValueError` with the argparse message (capture stderr with `contextlib.redirect_stderr`).
  - `to_json()` and `from_json()`.

- [ ] **Step 1: Write the tests.**
  - The reference preset argv equals the command in `docs/dev/animation-export.md` step 1, minus `--output-root` and with `--duration 300`.
  - `validate` raises `ValueError` that contains `wall limit` for `planar-7dof` with a timeout of 301.
  - `quasi_static_attitude=True` with `14dof-simple` raises `ValueError("quasi-static attitude applies to planar-7dof only")`.
  - A JSON round trip gives the same object.
- [ ] **Step 2: Run the tests.** Expect failures.
- [ ] **Step 3: Implement.** Omit `--lateral-acceleration` when it is `None`. Format floats with `repr`.
- [ ] **Step 4: Run the tests.** Expect PASS.
- [ ] **Step 5: Commit** `feat: map GUI run settings to runner arguments`.

---

### Task 5: Study definition and sweep expansion

**Files:** Create `tools/sim_gui/study.py`. Test: `tests/test_sim_gui_core.py`.

**Interfaces:**
- Consumes: `VehicleConfiguration`, `RunSettings`.
- Produces:
  - `Dimension(key: str, values: tuple[str, ...])`. The key is a slot key, `"model"` or `"track"`. Each value is a repository-relative POSIX path or a model name.
  - `StudyDefinition(name: str, source: str, track: str, model: str, choices: dict[str, str], settings: RunSettings, mode: Literal["one-at-a-time", "full-factorial"], dimensions: tuple[Dimension, ...])`, with `to_json` and `from_json`.
  - `CaseSpec(id: str, label: str, model: str, track: str, choices: dict[str, str])`
  - `expand(definition) -> tuple[CaseSpec, ...]`
  - `MAXIMUM_CASES = 500`, `WARNING_CASES = 50`, `class TooManyCases(ValueError)`
  - `StudyIndex`, with `create(directory, definition, cases)`, `load(directory)`, `update(case_id, **fields)` (atomic rewrite) and `cases`.

- [ ] **Step 1: Write the tests.**
  - One-at-a-time with dimensions `tire:[a,b]` and `model:[planar-7dof,14dof-simple]` gives 1 + 2 + 2 = 5 cases. A value equal to the baseline is skipped, so the result is 3 when the baseline is `a` and `planar-7dof`.
  - Full factorial over the same dimensions gives 4 cases.
  - `case_000` is the baseline in one-at-a-time mode.
  - Labels have the form `tire=hoosier-43075-r20-simple, model=14dof-simple`. The baseline label is `baseline`.
  - A parent dimension (`powertrain`) gives cases whose child choices are the new assembly's children. Use `VehicleConfiguration.with_choice`.
  - 501 cases raise `TooManyCases`.
  - `StudyIndex.update` keeps every other case unchanged and leaves no `.tmp` file behind.
- [ ] **Step 2: Run the tests.** Expect failures.
- [ ] **Step 3: Implement.**
  - Use `itertools.product` for the full factorial.
  - Case ids are `case_{index:03d}`.
  - `create` writes `study.json` and `definition.study.json`.
  - Write `repository_commit` from `git rev-parse HEAD`, run with `subprocess.run([...], capture_output=True, cwd=REPOSITORY_ROOT)`. Write `null` when that fails.
- [ ] **Step 4: Run the tests.** Expect PASS.
- [ ] **Step 5: Commit** `feat: add study definition and sweep expansion`.

---

### Task 6: Case child entry and execution core

**Files:** Create `tools/sim_gui/run_case.py` and `tools/sim_gui/execute.py`. Test: `tests/test_sim_gui_core.py`, `tests/test_sim_gui_run.py`.

**Interfaces:**
- Consumes: Tasks 3–5.
- Produces:
  - `prepare_case(study_dir, definition, case: CaseSpec) -> PreparedCase(directory, formulation, vehicle, argv)`. It writes `case.json` and the variant files.
  - `validate_case(prepared) -> str | None`. It calls `compile_formulation`, and returns the error text or `None`.
  - `new_attempt(prepared) -> Path`, which makes `attempt_<time_ns>/` with `exist_ok=False`.
  - `child_command(prepared, attempt) -> list[str]` = `[sys.executable, "-u", <run_case.py>, "--case-dir", D, *argv, "--child-attempt", A]`.
  - `parse_progress(line: str, duration: float) -> float | None`
  - `finish_case(prepared, attempt, *, return_code: int | None, timed_out: bool, cancelled: bool, wall_seconds: float, command) -> dict`. It writes `parent.json` and returns the `study.json` case fields (status, attempt, run_record, summary).

`run_case.py`:

```python
"""Run one GUI study case: the runner child with the case's variant vehicle."""
from __future__ import annotations
import argparse, dataclasses, json, sys
from pathlib import Path

def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--case-dir", type=Path, required=True)
    known, rest = pre.parse_known_args(argv)
    case = json.loads((known.case_dir / "case.json").read_text())
    import tools.run.run_iem24_forward_driver as runner
    args = runner.parse_args(rest)
    if args.child_attempt is None:
        raise SystemExit("run_case.py runs only as a child: pass --child-attempt")
    spec = runner.MODELS[args.model]
    runner.MODELS[args.model] = dataclasses.replace(
        spec, formulation=runner.canonical_path(known.case_dir / case["formulation"]),
        vehicle=runner.canonical_path(known.case_dir / case["vehicle"]))
    return runner.child(args)

if __name__ == "__main__":
    raise SystemExit(main())
```

A probe on 2026-10-18 tested this pattern. A 0.3 s planar case with the Hoosier tire returned code 0 in 11 s. Its record manifest named the variant formulation, the variant vehicle and the Hoosier tire XML as sources.

Status mapping in `finish_case`:

| Condition | Status |
|---|---|
| `cancelled` | `cancelled` |
| `timed_out` | `timed_out` |
| return code 0 and `summary.json` exists | `completed` |
| return code 2 | `rejected` |
| anything else | `failed` |

`parent.json` has the same keys as the runner's `main()`: `status` (`Child_Completed` / `Child_Rejected` / `Timed_Out` / `Cancelled`), `return_code` (124 for a timeout, `None` for a cancel), `total_wall_seconds`, `within_wall_limit`, `timeout_seconds`, `requested_wall_seconds`, `elapsed_wall_seconds`, `solver_wall_budget_seconds`, `model`, `command`, `attempt`.

- [ ] **Step 1: Write the unit tests** (`test_sim_gui_core.py`).
  - `parse_progress('{"rhs_time": 150.0, "wall_seconds": 3}', 300.0) == 0.5`.
  - A non-JSON line returns `None`. A value above the duration clips to 1.0.
  - `finish_case` with each condition gives the status in the table.
  - `child_command` contains `--record` once and ends with `--child-attempt`, `A`.
- [ ] **Step 2: Write the integration test** (`test_sim_gui_run.py`). Run one case with these settings: planar, Hoosier tire, `duration=0.3`, `timeout_seconds=120`. Run it with `subprocess.run(child_command(...), cwd=REPOSITORY_ROOT, env=child_environment(), timeout=180)`. Check:
  - the return code is 0;
  - `load_run_record(record, root=REPOSITORY_ROOT)` passes (V10 source check included);
  - `record.manifest.sources` contains `models/tires/hoosier-43075-r20-simple.xml`.

  Put the study under `REPOSITORY_ROOT / "runs" / "_test"` so the source paths are relative. Delete it in `tearDown` with `shutil.rmtree`, and assert first that the path ends with `runs/_test`.
- [ ] **Step 3: Run both tests.** Expect failures.
- [ ] **Step 4: Implement.** Then run both tests. Expect PASS. The integration test takes about 12 s.
- [ ] **Step 5: Commit** `feat: run GUI cases through the forward-driver child`.

---

### Task 7: Headless study runner

**Files:** Create `tools/sim_gui/run_study.py`. Test: `tests/test_sim_gui_run.py`.

**Interfaces:**
- Consumes: Tasks 5–6.
- Produces: `run_study(definition, output_root=REPOSITORY_ROOT / "runs", parallel=1, on_update=None) -> Path` (the study directory), and a CLI with `--definition`, `--output-root` and `--parallel`.

- [ ] **Step 1: Write the test.** Run a definition with two one-at-a-time cases (baseline and Hoosier, 0.3 s each) with `parallel=2`. Check that `study.json` lists two `completed` cases and that each `run_record` file exists.
- [ ] **Step 2: Run the test.** Expect a failure.
- [ ] **Step 3: Implement.**
  - Prepare and validate every case. Write `invalid` and skip the invalid ones.
  - Run the rest with `concurrent.futures.ThreadPoolExecutor(parallel)`. Each worker uses `subprocess.Popen(..., stdout=log, stderr=STDOUT, cwd=root, env=child_environment())` and `wait(timeout=settings.timeout_seconds)`. On `TimeoutExpired`, call `kill()` and then `wait()`.
  - Update `StudyIndex` with a lock.
  - Exit 0 when every valid case is `completed`, and 1 otherwise.
- [ ] **Step 4: Run the test.** Expect PASS.
- [ ] **Step 5: Commit** `feat: add headless study runner`.

---

### Task 8: Qt model panel and settings tab

**Files:** Create `tools/sim_gui/qt/__init__.py`, `qt/model_panel.py`, `qt/settings_tab.py`. Test: `tests/test_sim_gui_qt.py`.

**Interfaces:**
- Consumes: Tasks 1, 2, 4.
- Produces:
  - `ModelPanel(QWidget)`, with the signals `configurationChanged(object)` and `slotSelected(str)`, and the methods `set_configuration(cfg)`, `configuration()`, `track()` and `baseline()`.
  - `SettingsTab(QWidget)`, with the signal `settingsChanged(object)` and the methods `settings()`, `set_settings(s)` and `set_model(name)`.

The `ModelPanel` layout, from top to bottom:
1. A source path label (elided in the middle, with the full path as its tooltip).
2. A `QFormLayout`: **Model** combo, **Track** combo, then one row for each slot. Each slot row is a `QComboBox` with the candidate stem as its text and the path as its data, a **Browse…** `QToolButton`, and a **Reset** `QToolButton` that is enabled only when the slot changed. Child rows indent by 16 px. A changed slot's combo uses a bold font.
3. A **Parameters** `QTableWidget` (Name, Value, Unit), read-only, for the slot that has focus.

Behaviour:
- A combo change calls `with_choice`. On `ValueError`, show a `QMessageBox.warning` and restore the old index with `blockSignals`.
- A parent change rebuilds that parent's child rows.
- When the configuration is not runnable, show the point-mass message in a red label and emit nothing that can run.

`SettingsTab` uses a `QFormLayout` with these widgets:
- `QDoubleSpinBox` for each float, with the ranges from `runner.parse_args` (positive). Use decimals: 3 for seconds and m/s, 6 for `rtol` and `maximum_step`.
- A **Lateral cap** checkbox with a spinbox.
- `QComboBox` for method, planner and precision.
- A **Quasi-static attitude** checkbox, enabled only for `planar-7dof`.
- A **Preset: Reference lap** button.
- `set_model` sets the timeout maximum (300 for planar, `1e6` for 14DoF) and the default.

- [ ] **Step 1: Write the offscreen tests.**
  - The tire combo holds two items.
  - Selecting the Hoosier tire emits a configuration whose `choices["tire"]` is the Hoosier path, and makes the font bold.
  - A powertrain swap rebuilds the child combos.
  - The quasi-static checkbox is disabled after `set_model("14dof-simple")`.
  - The planar timeout maximum is 300.

  Start the module with:

  ```python
  os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
  try:
      from PyQt6.QtWidgets import QApplication
  except ImportError:
      QApplication = None
  @unittest.skipIf(QApplication is None, "PyQt6 is not installed")
  ```

  Make one `QApplication.instance() or QApplication([])` in `setUpModule`.
- [ ] **Step 2: Run the tests.** Expect failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run the tests.** Expect PASS.
- [ ] **Step 5: Commit** `feat: add Qt model panel and run settings tab`.

---

### Task 9: Study tab, case queue and validation worker

**Files:** Create `qt/study_tab.py`, `qt/queue.py`, `qt/workers.py`. Test: `tests/test_sim_gui_qt.py`.

**Interfaces:**
- Consumes: Tasks 5–8.
- Produces:
  - `StudyTab(QWidget)`, with `definition(base) -> StudyDefinition` and `set_definition(d)`.
  - `ValidationWorker(QObject)`, with `run()` and the signals `progress(int, int)`, `caseReady(object, object)` (`PreparedCase`, error or `None`) and `finished()`.
  - `CaseQueue(QObject)`, with `start(prepared_cases, parallel, timeout)` and `stop()`, and the signals `caseStarted(str)`, `caseProgress(str, float)`, `caseFinished(str, dict)` and `allFinished()`.
  - `QueueTable(QTableWidget)`, with the columns #, Label, Status, Progress (`QProgressBar` cell widget), Wall s, Lap time s and Record.

`StudyTab` layout:
- Mode radio buttons **One-at-a-time** and **Full factorial**.
- A dimension list (`QListWidget`). **Add dimension** gives a menu of the slot keys, `model` and `track`. **Remove** deletes the selected dimension.
- For the selected dimension, a checklist (`QListWidget` with checkable items) of its candidates, or the model names, or the tracks.
- A **Case preview** table (id, label). It updates on every change, and shows the case count in red above 50.
- **Study name** `QLineEdit`, default `study`, limited to the regular expression `[A-Za-z0-9_-]{1,40}`.

`CaseQueue`:
- Keep one `QProcess` for each running case. Use `setWorkingDirectory(str(REPOSITORY_ROOT))`, `setProcessEnvironment` built from `child_environment()`, and `setProcessChannelMode(MergedChannels)`.
- Connect `readyReadStandardOutput` to a handler. It appends the bytes to `run.log` and sends each complete line to `parse_progress`.
- Use a single-shot `QTimer` for each case with the timeout in milliseconds. On timeout, call `kill()`.
- When a slot is free, start the next pending case.
- `stop()` marks the pending cases `cancelled`, calls `kill()` on the running ones, and records them as `cancelled`.
- `finished(int, QProcess.ExitStatus)` calls `finish_case`, then `StudyIndex.update`.

- [ ] **Step 1: Write the offscreen tests.**
  - The preview count is 3 for the one-at-a-time example of Task 5.
  - Test `CaseQueue` with a fake case whose command is `[sys.executable, "-c", "print('{\"rhs_time\": 1.0}'); import time; time.sleep(30)"]`. Inject the command through the `command_factory` constructor argument, which defaults to `child_command`. Call `stop()` after the first `caseProgress`. Check that `caseFinished` reports `cancelled` within 5 s, using a `QEventLoop` with a `QTimer` guard.
  - A timeout of 0.5 s gives `timed_out`.
- [ ] **Step 2: Run the tests.** Expect failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run the tests.** Expect PASS.
- [ ] **Step 5: Commit** `feat: add study tab and process-pool case queue`.

---

### Task 10: Main window and entry point

**Files:** Create `qt/main_window.py` and `tools/sim_gui/app.py`. Test: `tests/test_sim_gui_qt.py`.

**Interfaces:**
- Consumes: Tasks 8–9.
- Produces: `MainWindow(QMainWindow)` and `app.main(argv=None) -> int`. `app.py` accepts an optional positional model XML path to open at start.

Window layout:
- **Toolbar and File menu:** Open Vehicle Model… (`Ctrl+O`), Open Study… , Save Study… (`Ctrl+S`), Build Cases, Run (`F5`), Stop, Open Output Folder (`QDesktopServices.openUrl(QUrl.fromLocalFile(...))`), and Quit.
- **Left dock:** `ModelPanel`.
- **Centre:** a `QTabWidget` with the tabs **Run Settings**, **Study**, **Queue** and **Log**. The Log tab is a read-only `QPlainTextEdit` that shows the selected case's `run.log`. It refreshes with a 1 s `QTimer` while that case runs, and keeps the last 2000 lines.
- **Status bar:** the counts of cases by status.

Rules:
- **Run** without a study builds a single baseline case.
- **Run** is disabled while a study runs or when the configuration is not runnable.
- **Build Cases** runs the `ValidationWorker` with a `QProgressDialog`. It asks for confirmation above 50 cases, and refuses above 500.
- On close during a run, ask "Stop the running cases and quit?". **Yes** calls `CaseQueue.stop()` and waits for `allFinished` (at most 10 s).
- Keep the window geometry and the last 10 opened files in `QSettings`.
- Call `QApplication.setApplicationName("Simulation GUI")`. Qt 6 handles high-DPI scaling by default.

- [ ] **Step 1: Write the offscreen test.** Start `MainWindow`, call `open_model(FORMULATION)`, build one case, run it with `duration=0.3`, and wait at most 120 s for `allFinished`. Check that the queue table shows `completed` and that the record file exists. Use `runs/_test` as the output root and clean it up as in Task 6.
- [ ] **Step 2: Run the test.** Expect a failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run the test.** Expect PASS.
- [ ] **Step 5: Start the GUI by hand** with `PYTHONPATH=src:. python3 tools/sim_gui/app.py models/formulations/iem24_murphy_real_14dof_simple.xml`. Check that the dropdowns are filled, a swap makes the font bold, and the Stop button stops a case.
- [ ] **Step 6: Commit** `feat: add simulation GUI main window`.

---

### Task 11: Documentation and full suite

**Files:**
- Create: `docs/dev/simulation-gui.md`
- Modify: `docs/dev/tools-layout.md` (add the row `` `tools/sim_gui/` | Run simulations from a GUI: component swaps, sweeps, run records. See [simulation-gui.md](simulation-gui.md) ``), and `README.md` (add one link line in the tools list, if a tools list exists).

- [ ] **Step 1: Write `docs/dev/simulation-gui.md`** in ASD-STE100 style. Give it these sections:
  - Install (`pip install -e .[gui]`, or `pip install "PyQt6>=6.5"`).
  - Start the GUI.
  - Load a vehicle model.
  - Swap a component, with the candidate rule.
  - Run a study, with the two modes and the case limits.
  - Output layout and the `study.json` schema.
  - The headless runner.
  - Limits: no parameter editing, no warm start, no point-mass runner, and planar runs capped at 300 s.
  - Hand-off: the post-processing GUI reads `study.json` and each `run_record.npz`. For an animation now, use `tools/anim/export_animation.py --record <path>`.
- [ ] **Step 2: Run the full suite.**
  `MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest discover -s tests -q > /tmp/full_suite.log 2>&1; grep -nE "^(Ran |OK|FAILED|FAIL:|ERROR:)" /tmp/full_suite.log`
  Expect `OK`. The suite takes about 7 minutes.
- [ ] **Step 3: Commit** `docs: document the simulation GUI`.

---

## Out of Scope (later work)

- The post-processing GUI: graphs, reports and animations from `study.json`.
- Editing parameters, and sweeping parameter values.
- Warm-start chains and multi-segment runs.
- A forward runner for the point-mass model.
