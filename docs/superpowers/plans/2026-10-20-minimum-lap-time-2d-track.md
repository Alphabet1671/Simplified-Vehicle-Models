# Minimum-Time Line on a 2D Track (Stage 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the track model real lateral edges and deliver a quasi-steady free-line minimum-time solver whose output is the optimal racing line, its speed profile, and a lap time verified against the existing centreline solver.

**Architecture:** Track width becomes a provenance-tagged `width` data role on `TrackProfile`, exposed as a fold-guarded `Corridor` in path-offset coordinates. The vehicle's quasi-steady capability is sampled once from the existing NumPy point-mass physics into a `(v, κ) → (a_min, a_max, v_ceiling)` surface cached on disk. CasADi interpolates that surface inside a Hermite–Simpson direct-collocation NLP in the distance domain with states `(n, ψ, v)` and controls `(a, ψ')`, solved by IPOPT over a coarse-to-fine mesh ladder. The line is converted back to a `PathCoordinates` and checked by re-running the existing solver on it.

**Tech Stack:** Python 3.13, NumPy, SciPy, CasADi (optional import) with IPOPT, `unittest`, XML model files, CSV data files.

**Spec:** [2026-10-20-minimum-lap-time-2d-track-spec.md](2026-10-20-minimum-lap-time-2d-track-spec.md). Read it first; every constraint below traces to a decision `D1`–`D8` in that file.

## Global Constraints

- Do **not** modify any existing track XML or CSV. Archive summaries record source hashes for `models/tracks/2024_michigan_endurance_closed.xml`, `..._multilap_closed.xml`, `..._v11.xml`, `fsae_2026_acceleration.xml`, `fsae_2026_skidpad.xml` and their data files. Editing one invalidates published provenance. New width data goes in **new** files; new edged tracks are **new** XML files that reference the existing geometry CSV unchanged.
- The geometry CSV header must stay exactly `distance,x_north,y_east,heading_rad,curvature`. `_read_geometry_csv` rejects any other header, so width must never become a column there (`D2`).
- Stored half-widths are **reference-point** clearances: edge distance minus car half-width minus margin. No task subtracts vehicle geometry (`D3`).
- Every width source carries `width_provenance` ∈ `{surveyed, swept, assumed}`; a missing or unknown tag is an error, not a default (`D4`).
- Stage 1 physics stays in NumPy. No tyre, load, or powertrain equation may be re-implemented in CasADi (`D5`).
- CasADi must not be imported at module scope in `src/`. Import it inside the functions that need it, and guard the solver tests with `unittest.skipUnless(_has_casadi(), ...)`.
- **Lateral sign convention, derived not assumed.** `x` north, `y` east, `z` down is right handed, so positive curvature is a right turn. `ṅ = vx sinψ + vy cosψ` grows `n` for an eastward (rightward) velocity when heading north, so **`n` is positive to the right of travel**, and the offset-curve curvature is `κ = κ₀/(1 − κ₀ n)`. Tasks 3, 5 and 7 each contain a test that fails if this is inverted; if any of them disagrees, fix the source, never the test.
- SI units throughout: metres, radians, radians per metre, metres per second, metres per second squared, seconds.
- Run `PYTHONPATH=src python3 -m unittest discover -s tests -q` before every commit.

## File Structure

| Path | Responsibility |
|---|---|
| `src/simplified_vehicle_models/tracks/types.py` | `TrackProfile` width fields and provenance validation |
| `src/simplified_vehicle_models/tracks/profile.py` | Parses the `width` data role and the constant fallback |
| `src/simplified_vehicle_models/tracks/corridor.py` | **new** — `Corridor`, `corridor_from_track`, fold guard |
| `src/simplified_vehicle_models/tracks/racing_line.py` | **new** — `offset_path`, `path_from_solution` |
| `src/simplified_vehicle_models/formulations/point_mass_capability.py` | **new** — sampled capability surface, hash, save/load |
| `src/simplified_vehicle_models/solvers/qss_line_ocp.py` | **new** — CasADi transcription, mesh, solve, ladder |
| `tools/tracks/build_width_profile.py` | **new** — writes a width CSV |
| `tools/model_data/build_capability_surface.py` | **new** — builds and caches the surface |
| `tools/run/solve_minimum_time_line.py` | **new** — mesh ladder run and artifacts |
| `tools/gates/check_minimum_time_line.py` | **new** — the five stage 1 acceptance checks |
| `tests/test_track_width.py`, `tests/test_corridor.py`, `tests/test_point_mass_capability.py`, `tests/test_qss_line_ocp.py`, `tests/test_racing_line.py`, `tests/test_minimum_time_line_gate.py` | **new** |
| `docs/dev/minimum-time-line.md` | **new** — how to run and what the numbers mean |

---

### Task 1: Track width in the track model

**Files:**
- Modify: `src/simplified_vehicle_models/tracks/types.py`
- Modify: `src/simplified_vehicle_models/tracks/profile.py`
- Test: `tests/test_track_width.py`

**Interfaces:**
- Produces: `TrackProfile.half_width_right`, `.half_width_left`, `.width_provenance`, `.has_edges`; `WIDTH_PROVENANCE`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_track_width.py`:

```python
import csv
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track


def _write_circle_track(
    directory: Path,
    *,
    radius: float = 25.0,
    nodes: int = 240,
    direction: float = 1.0,
    half_width: float | None = None,
    width_rows: int | None = None,
    provenance: str | None = "assumed",
    side: str = "both",
) -> Path:
    """Write a closed circular track with an optional width source.

    ``direction`` is +1 for a right-turning circle (positive curvature) and -1
    for a left-turning one.
    """

    distance = np.linspace(0.0, 2.0 * math.pi * radius, nodes)
    angle = direction * distance / radius
    north = radius * np.sin(angle)
    east = radius * (1.0 - np.cos(angle))
    heading = angle.copy()
    curvature = np.full(nodes, direction / radius)

    with (directory / "centreline.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["distance", "x_north", "y_east", "heading_rad", "curvature"])
        for row in zip(distance, north, east, heading, curvature):
            writer.writerow([f"{value:.17g}" for value in row])

    parameters = [
        '<parameter name="is_closed" value="true" />',
        '<parameter name="terminal_segment_is_duplicate" value="true" />',
        '<parameter name="maximum_radius" value="300" unit="m" />',
    ]
    data = ['<data ref="centreline.csv" />']
    if half_width is not None:
        parameters.append(f'<parameter name="half_width" value="{half_width}" unit="m" />')
        parameters.append(
            f'<parameter name="width_provenance" value="{provenance or ""}" />'
        )
    if width_rows is not None:
        with (directory / "width.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["distance", "half_width_left", "half_width_right"])
            for index in range(width_rows):
                left = 3.0 if side in ("both", "left") else 1.0
                right = 3.0 if side in ("both", "right") else 1.0
                writer.writerow([f"{distance[index]:.17g}", f"{left}", f"{right}"])
        data.append('<data role="width" ref="width.csv" />')
        parameters.append('<parameter name="width_provenance" value="surveyed" />')

    xml = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model schema-version="1.0" id="circle" type="track-model" '
        'subtype="closed-xy-profile">\n'
        "  <metadata><title>Synthetic circle</title></metadata>\n"
        f"  {' '.join(data)}\n"
        "  <parameters>\n"
        f"    {' '.join(parameters)}\n"
        "  </parameters>\n"
        "</model>\n"
    )
    path = directory / "track.xml"
    path.write_text(xml, encoding="utf-8")
    return path


class WidthParameterTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def test_constant_width_is_broadcast_to_the_whole_grid(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        self.assertTrue(track.has_edges)
        self.assertEqual(track.width_provenance, "assumed")
        np.testing.assert_allclose(track.half_width_right, 2.25)
        np.testing.assert_allclose(track.half_width_left, 2.25)
        self.assertEqual(track.half_width_right.shape, track.distance.shape)

    def test_absent_width_leaves_edges_undefined(self) -> None:
        track = load_track(_write_circle_track(self.directory))
        self.assertFalse(track.has_edges)
        self.assertIsNone(track.half_width_right)
        self.assertIsNone(track.width_provenance)

    def test_width_data_role_is_read_per_node(self) -> None:
        track = load_track(
            _write_circle_track(self.directory, width_rows=240, side="left")
        )
        self.assertEqual(track.width_provenance, "surveyed")
        np.testing.assert_allclose(track.half_width_left, 3.0)
        np.testing.assert_allclose(track.half_width_right, 1.0)

    def test_width_data_row_count_must_match_the_grid(self) -> None:
        with self.assertRaises(ValueError):
            load_track(_write_circle_track(self.directory, width_rows=100))

    def test_missing_provenance_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            load_track(
                _write_circle_track(self.directory, half_width=2.25, provenance=None)
            )

    def test_unknown_provenance_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            load_track(
                _write_circle_track(self.directory, half_width=2.25, provenance="guessed")
            )

    def test_nonpositive_half_width_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            load_track(_write_circle_track(self.directory, half_width=0.0))
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `PYTHONPATH=src python3 -m unittest tests.test_track_width -v`
Expected: `AttributeError: 'TrackProfile' object has no attribute 'has_edges'` on the first test; the provenance and row-count tests fail because no `ValueError` is raised.

- [ ] **Step 3: Add the width fields**

In `src/simplified_vehicle_models/tracks/types.py`, above `TrackProfile`:

```python
#: How a lateral corridor was obtained.  ``surveyed`` means the boundary was
#: measured independently of the driver.  ``swept`` means it is the union of
#: driven vehicle envelopes and is therefore an inner bound on the drivable
#: area.  ``assumed`` means it is a declared constant used for sensitivity
#: only.  A result produced from a non-surveyed corridor is conditional.
WIDTH_PROVENANCE = ("surveyed", "swept", "assumed")
```

Replace the field block of `TrackProfile` with:

```python
    distance: FloatArray
    segment_length: FloatArray
    radius: FloatArray
    curvature: FloatArray
    is_closed: bool
    title: str
    path_coordinates: PathCoordinates | None = None
    half_width_right: FloatArray | None = None
    half_width_left: FloatArray | None = None
    width_provenance: str | None = None

    def __post_init__(self) -> None:
        left, right = self.half_width_left, self.half_width_right
        if (left is None) != (right is None):
            raise ValueError("both half widths are required together")
        if left is None:
            if self.width_provenance is not None:
                raise ValueError("width provenance requires width data")
            return
        if self.width_provenance not in WIDTH_PROVENANCE:
            raise ValueError(
                f"width provenance must be one of {WIDTH_PROVENANCE}, "
                f"not {self.width_provenance!r}"
            )
        for name in ("half_width_left", "half_width_right"):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != self.distance.shape:
                raise ValueError(f"{name} must match the distance grid")
            if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
                raise ValueError(f"{name} must be positive and finite")
            values.setflags(write=False)
            object.__setattr__(self, name, values)

    @property
    def has_edges(self) -> bool:
        """Return true when the track carries a lateral corridor."""

        return self.half_width_left is not None
```

Add `import numpy as np` at the top of `types.py` if absent.

- [ ] **Step 4: Parse the width role**

In `src/simplified_vehicle_models/tracks/profile.py`, after `_data_path`:

```python
def _role_data_path(
    root: ElementTree.Element, role: str, source_path: Path
) -> Path | None:
    """Return the data reference for one named role, or None when absent."""

    for element in root.findall("./data"):
        if element.get("role") != role:
            continue
        reference = element.get("ref")
        if not reference:
            raise ValueError(f"{source_path}: data role {role!r} has no reference")
        return (source_path.parent / reference).resolve()
    return None


def _read_width_csv(path: Path) -> tuple[FloatArray, FloatArray, FloatArray]:
    """Read a per-node lateral corridor table.

    Columns are distance, left half width, and right half width in metres.
    Positive lateral offset is to the right of the direction of travel.
    """

    expected = ["distance", "half_width_left", "half_width_right"]
    rows: list[tuple[float, float, float]] = []
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames != expected:
                raise ValueError(f"{path}: width columns must be {', '.join(expected)}")
            for row_number, row in enumerate(reader, start=2):
                try:
                    rows.append(
                        (
                            float(row["distance"]),
                            float(row["half_width_left"]),
                            float(row["half_width_right"]),
                        )
                    )
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError(
                        f"{path}: row {row_number} contains invalid numeric data"
                    ) from error
    except OSError as error:
        raise ValueError(f"cannot read width data {path}: {error}") from error
    values = np.asarray(rows, dtype=float)
    return values[:, 0], values[:, 1], values[:, 2]


def _width_fields(
    root: ElementTree.Element, source_path: Path, distance: FloatArray
) -> dict[str, object]:
    """Return the width constructor fields for one track."""

    parameters = _read_parameters(root, source_path)
    data_path = _role_data_path(root, "width", source_path)
    constant = parameters.get("half_width")
    if data_path is not None and constant is not None:
        raise ValueError(
            f"{source_path}: declare the corridor as data or as a parameter, not both"
        )
    if data_path is None and constant is None:
        return {}
    provenance = parameters.get("width_provenance")
    if provenance is None:
        raise ValueError(f"{source_path}: width data needs a width_provenance")
    if data_path is not None:
        width_distance, left, right = _read_width_csv(data_path)
        if width_distance.shape != distance.shape or not np.allclose(
            width_distance, distance, rtol=0.0, atol=1.0e-9
        ):
            raise ValueError(
                f"{data_path}: width distance grid must match the geometry grid"
            )
    else:
        half_width = _parse_positive_float(constant, "half_width", source_path)
        left = np.full(distance.shape, half_width)
        right = np.full(distance.shape, half_width)
    return {
        "half_width_left": left,
        "half_width_right": right,
        "width_provenance": provenance.strip(),
    }
```

Then in `_load_closed_xy` and `_load_open_xy`, splat the width fields into the returned `TrackProfile`:

```python
    return TrackProfile(
        distance=distance,
        segment_length=physical_segment_length,
        radius=magnitude,
        curvature=curvature,
        is_closed=is_closed,
        title=_title(root, source_path),
        path_coordinates=coordinates,
        **_width_fields(root, source_path, distance),
    )
```

Leave `_load_distance_radius` unchanged: a legacy radius profile has no geometry to offset from.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `PYTHONPATH=src python3 -m unittest tests.test_track_width -v`
Expected: 7 tests, PASS.

- [ ] **Step 6: Run the full suite**

Run: `PYTHONPATH=src python3 -m unittest discover -s tests -q`
Expected: PASS. Existing positional `TrackProfile(...)` constructions keep working because the new fields default to `None`.

- [ ] **Step 7: Commit**

```bash
git add src/simplified_vehicle_models/tracks/types.py \
        src/simplified_vehicle_models/tracks/profile.py \
        tests/test_track_width.py
git commit -m "feat: carry a provenance-tagged lateral corridor on the track profile"
```

---

### Task 2: Width source tool and the assumed MIS corridor

**Files:**
- Create: `tools/tracks/build_width_profile.py`
- Create: `models/tracks/2024_michigan_endurance_closed_width.csv`
- Create: `models/tracks/2024_michigan_endurance_closed_edged.xml`
- Test: `tests/test_track_width.py` (append)

**Interfaces:**
- Produces: `write_width_profile(track, destination, *, half_width=None, half_width_left=None, half_width_right=None, provenance="assumed") -> Path`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_track_width.py`:

```python
from tools.tracks.build_width_profile import write_width_profile


class WidthProfileToolTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def test_symmetric_width_writes_the_expected_header_and_rows(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        destination = self.directory / "written.csv"
        write_width_profile(source, destination, half_width=2.25, provenance="assumed")
        lines = destination.read_text(encoding="utf-8").splitlines()
        self.assertEqual(lines[0], "distance,half_width_left,half_width_right")
        self.assertEqual(len(lines), source.distance.size + 1)
        left, right = (float(value) for value in lines[1].split(",")[1:])
        self.assertAlmostEqual(left, 2.25)
        self.assertAlmostEqual(right, 2.25)

    def test_asymmetric_width_is_written_per_side(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        destination = self.directory / "asymmetric.csv"
        write_width_profile(
            source,
            destination,
            half_width_left=2.0,
            half_width_right=3.0,
            provenance="assumed",
        )
        first = destination.read_text(encoding="utf-8").splitlines()[1].split(",")
        self.assertAlmostEqual(float(first[1]), 2.0)
        self.assertAlmostEqual(float(first[2]), 3.0)

    def test_both_a_half_width_and_per_side_widths_is_rejected(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        with self.assertRaises(ValueError):
            write_width_profile(
                source,
                self.directory / "bad.csv",
                half_width=2.0,
                half_width_left=2.0,
                provenance="assumed",
            )

    def test_unknown_provenance_is_rejected(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        with self.assertRaises(ValueError):
            write_width_profile(
                source, self.directory / "bad.csv", half_width=2.0, provenance="guessed"
            )

    def test_the_written_file_loads_back_through_the_track_loader(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        write_width_profile(
            source, self.directory / "width.csv", half_width=1.75, provenance="assumed"
        )
        track_dir = self.directory
        xml = (
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<model schema-version="1.0" id="round-trip" type="track-model" '
            'subtype="closed-xy-profile">\n'
            '  <metadata><title>Round trip</title></metadata>\n'
            '  <data ref="centreline.csv" />\n'
            '  <data role="width" ref="width.csv" />\n'
            '  <parameters>\n'
            '    <parameter name="is_closed" value="true" />\n'
            '    <parameter name="terminal_segment_is_duplicate" value="true" />\n'
            '    <parameter name="maximum_radius" value="300" unit="m" />\n'
            '    <parameter name="width_provenance" value="assumed" />\n'
            '  </parameters>\n'
            '</model>\n'
        )
        (track_dir / "round_trip.xml").write_text(xml, encoding="utf-8")
        restored = load_track(track_dir / "round_trip.xml")
        np.testing.assert_allclose(restored.half_width_left, 1.75)
        np.testing.assert_allclose(restored.half_width_right, 1.75)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_track_width.WidthProfileToolTest -v`
Expected: `ModuleNotFoundError: No module named 'tools.tracks.build_width_profile'`.

- [ ] **Step 3: Write the tool**

Create `tools/tracks/build_width_profile.py`:

```python
"""Write a lateral corridor table for a track.

The corridor is a reference-point clearance: the stored half width is the
distance from the centreline to the edge minus the car half width and any
declared margin.  The solver never subtracts vehicle geometry.

Positive lateral offset is to the right of the direction of travel.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import WIDTH_PROVENANCE


def write_width_profile(
    track,
    destination: str | Path,
    *,
    half_width: float | None = None,
    half_width_left: float | None = None,
    half_width_right: float | None = None,
    provenance: str = "assumed",
) -> Path:
    """Write ``distance, half_width_left, half_width_right`` for a track."""

    if provenance not in WIDTH_PROVENANCE:
        raise ValueError(f"provenance must be one of {WIDTH_PROVENANCE}")
    symmetric = half_width is not None
    per_side = half_width_left is not None or half_width_right is not None
    if symmetric and per_side:
        raise ValueError("declare either half_width or the two sides, not both")
    if not symmetric and not per_side:
        raise ValueError("declare half_width or both sides")
    if symmetric:
        left = np.full(track.distance.shape, float(half_width))
        right = np.full(track.distance.shape, float(half_width))
    else:
        if half_width_left is None or half_width_right is None:
            raise ValueError("both sides are required")
        left = np.full(track.distance.shape, float(half_width_left))
        right = np.full(track.distance.shape, float(half_width_right))
    if not (np.all(np.isfinite(left)) and np.all(np.isfinite(right))):
        raise ValueError("half widths must be finite")
    if np.any(left <= 0.0) or np.any(right <= 0.0):
        raise ValueError("half widths must be positive")

    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["distance", "half_width_left", "half_width_right"])
        for index, distance in enumerate(track.distance):
            writer.writerow(
                [f"{distance:.17g}", f"{left[index]:.17g}", f"{right[index]:.17g}"]
            )
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--half-width", type=float, default=None)
    parser.add_argument("--half-width-left", type=float, default=None)
    parser.add_argument("--half-width-right", type=float, default=None)
    parser.add_argument("--provenance", default="assumed", choices=WIDTH_PROVENANCE)
    arguments = parser.parse_args(argv)
    track = load_track(arguments.track)
    path = write_width_profile(
        track,
        arguments.output,
        half_width=arguments.half_width,
        half_width_left=arguments.half_width_left,
        half_width_right=arguments.half_width_right,
        provenance=arguments.provenance,
    )
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src python3 -m unittest tests.test_track_width.WidthProfileToolTest -v`
Expected: 5 tests, PASS.

- [ ] **Step 5: Generate the assumed MIS corridor**

The 2.25 m figure is a **declared assumption** for a wide FSAE course with the car half-width already removed. It is not a measurement and must be reported as `assumed` everywhere.

```bash
PYTHONPATH=src python3 -m tools.tracks.build_width_profile \
  --track models/tracks/2024_michigan_endurance_closed.xml \
  --output models/tracks/2024_michigan_endurance_closed_width.csv \
  --half-width 2.25 \
  --provenance assumed
```

Create `models/tracks/2024_michigan_endurance_closed_edged.xml`. It reuses the existing geometry CSV unchanged and adds only a `width` role:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<model schema-version="1.0" id="2024-michigan-endurance-closed-edged" type="track-model" subtype="closed-xy-profile">
    <!-- Same geometry as 2024_michigan_endurance_closed.xml. That file is not
         edited, so its recorded source hash stays valid. Only a lateral
         corridor is added, and that corridor is ASSUMED, not surveyed. -->
    <metadata>
        <title>2024 Michigan Endurance Closed (assumed corridor)</title>
        <source sha256="091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7">0615CompEndurance.mat</source>
    </metadata>
    <data ref="2024_michigan_endurance_closed.csv" />
    <data role="width" ref="2024_michigan_endurance_closed_width.csv" />
    <parameters>
        <parameter name="is_closed" value="true" />
        <parameter name="terminal_segment_is_duplicate" value="true" />
        <parameter name="node_spacing" value="0.5" unit="m" />
        <parameter name="smoothing_length" value="2" unit="m" />
        <parameter name="maximum_radius" value="300" unit="m" />
        <parameter name="width_provenance" value="assumed" />
        <parameter name="half_width_note" value="declared 2.25 m reference-point half width; not measured" />
    </parameters>
</model>
```

- [ ] **Step 6: Verify the edged track loads and the originals are untouched**

```bash
PYTHONPATH=src python3 -c "
from simplified_vehicle_models.tracks.profile import load_track
track = load_track('models/tracks/2024_michigan_endurance_closed_edged.xml')
print(track.has_edges, track.width_provenance, track.half_width_right.min(), track.half_width_right.max(), track.lap_length)
"
git diff --stat models/tracks/2024_michigan_endurance_closed.xml models/tracks/2024_michigan_endurance_closed.csv
```

Expected: `True assumed 2.25 2.25 1016.70064959`, and `git diff --stat` prints nothing.

- [ ] **Step 7: Commit**

```bash
git add tools/tracks/build_width_profile.py tests/test_track_width.py \
        models/tracks/2024_michigan_endurance_closed_width.csv \
        models/tracks/2024_michigan_endurance_closed_edged.xml
git commit -m "feat: add a corridor writer and an assumed-corridor MIS track"
```

---

### Task 3: Corridor bounds with the fold guard

**Files:**
- Create: `src/simplified_vehicle_models/tracks/corridor.py`
- Test: `tests/test_corridor.py`

**Interfaces:**
- Produces: `Corridor(distance, minimum, maximum, provenance)` with `.half_width`, `.is_periodic`; `corridor_from_track(track, *, margin=0.0, fold_guard=0.2) -> Corridor`.
- Guarantees `minimum <= 0 <= maximum` and `minimum <= maximum`, so Task 5's box constraints are always non-empty.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_corridor.py`:

```python
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


class CorridorTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def test_symmetric_corridor_uses_the_right_positive_convention(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track)
        np.testing.assert_allclose(corridor.maximum, 2.25)
        np.testing.assert_allclose(corridor.minimum, -2.25)
        self.assertEqual(corridor.provenance, "assumed")

    def test_narrow_right_side_maps_to_the_positive_bound(self) -> None:
        track = load_track(
            _write_circle_track(self.directory, width_rows=240, side="left")
        )
        corridor = corridor_from_track(track)
        np.testing.assert_allclose(corridor.maximum, 1.0)
        np.testing.assert_allclose(corridor.minimum, -3.0)

    def test_margin_shrinks_both_sides(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track, margin=0.25)
        np.testing.assert_allclose(corridor.maximum, 2.0)
        np.testing.assert_allclose(corridor.minimum, -2.0)

    def test_zero_corridor_is_representable(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track, margin=2.25)
        np.testing.assert_allclose(corridor.maximum, 0.0)
        np.testing.assert_allclose(corridor.minimum, 0.0)

    def test_margin_wider_than_the_corridor_is_rejected(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        with self.assertRaises(ValueError):
            corridor_from_track(track, margin=3.0)

    def test_fold_guard_clips_a_tight_corner(self) -> None:
        # A 6 m radius corner with a 4 m corridor would fold: 1 - kappa*n <= 0.
        track = load_track(
            _write_circle_track(self.directory, radius=6.0, half_width=4.0)
        )
        corridor = corridor_from_track(track, fold_guard=0.2, margin=0.0)
        fold = 1.0 - track.curvature * corridor.maximum
        self.assertGreaterEqual(float(np.min(fold)), 0.2 - 1.0e-12)
        self.assertLess(float(np.max(corridor.maximum)), 4.0)

    def test_a_track_without_edges_is_rejected(self) -> None:
        track = load_track(_write_circle_track(self.directory))
        with self.assertRaises(ValueError):
            corridor_from_track(track)

    def test_arrays_are_read_only(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track)
        with self.assertRaises(ValueError):
            corridor.maximum[0] = 0.0
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_corridor -v`
Expected: `ModuleNotFoundError: No module named 'simplified_vehicle_models.tracks.corridor'`.

- [ ] **Step 3: Write the corridor module**

Create `src/simplified_vehicle_models/tracks/corridor.py`:

```python
"""Express a track's lateral edges in path-offset coordinates.

The offset ``n`` is positive to the right of the direction of travel.  That
follows from the repository frame: ``x`` north, ``y`` east, ``z`` down is right
handed, positive curvature is a right turn, and ``n_dot = vx sin(psi) +
vy cos(psi)`` grows for a rightward velocity component.  The curvature of an
offset path is therefore ``kappa_0 / (1 - kappa_0 * n)``, and ``1 - kappa_0 * n``
must stay positive, which is the fold guard.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class Corridor:
    """Store the admissible lateral offset at every path node."""

    distance: FloatArray
    minimum: FloatArray
    maximum: FloatArray
    provenance: str

    @property
    def half_width(self) -> float:
        """Return the narrowest half width in the corridor."""

        return float(min(-self.minimum.min(), self.maximum.min()))

    @property
    def is_periodic(self) -> bool:
        """Return true when the first and last nodes agree."""

        return bool(self.minimum[0] == self.minimum[-1])


def corridor_from_track(
    track, *, margin: float = 0.0, fold_guard: float = 0.2
) -> Corridor:
    """Return the corridor in path-offset coordinates, clipped by the fold guard."""

    if not getattr(track, "has_edges", False):
        raise ValueError("the track carries no lateral corridor")
    if not np.isfinite(margin) or margin < 0.0:
        raise ValueError("margin must be finite and nonnegative")
    if not np.isfinite(fold_guard) or not 0.0 < fold_guard < 1.0:
        raise ValueError("fold_guard must lie strictly between 0 and 1")

    left = np.asarray(track.half_width_left, dtype=float)
    right = np.asarray(track.half_width_right, dtype=float)
    if margin > float(min(left.min(), right.min())):
        raise ValueError("margin removes the whole corridor")

    # Positive offset is to the right, so the right half width is the upper bound.
    maximum = right - margin
    minimum = -left + margin

    # The offset curvature kappa_0 / (1 - kappa_0 * n) blows up as the
    # denominator reaches zero, so each side is clipped where it would fold.
    magnitude = np.abs(np.asarray(track.curvature, dtype=float))
    limit = np.where(
        magnitude > 0.0, (1.0 - fold_guard) / np.maximum(magnitude, 1.0e-12), np.inf
    )
    maximum = np.minimum(maximum, limit)
    minimum = np.maximum(minimum, -limit)

    if np.any(minimum > 0.0) or np.any(maximum < 0.0):
        raise ValueError("the corridor does not contain the centreline")
    if np.any(minimum > maximum):
        raise ValueError("the corridor is empty")

    for array in (maximum, minimum):
        array.setflags(write=False)
    return Corridor(
        distance=np.asarray(track.distance, dtype=float),
        minimum=minimum,
        maximum=maximum,
        provenance=str(track.width_provenance),
    )
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src python3 -m unittest tests.test_corridor -v`
Expected: 8 tests, PASS.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/tracks/corridor.py tests/test_corridor.py
git commit -m "feat: express track edges as a fold-guarded lateral corridor"
```

---

### Task 4: Sampled quasi-steady capability surface

**Files:**
- Create: `src/simplified_vehicle_models/formulations/point_mass_capability.py`
- Create: `tools/model_data/build_capability_surface.py`
- Test: `tests/test_point_mass_capability.py`

**Interfaces:**
- Produces: `CapabilitySurface(speed, curvature, acceleration_max, acceleration_min, speed_ceiling, provenance_hash)`; `build_capability_surface(vehicle, *, curvature_limit, ...)`; `capability_hash(vehicle)`; `save_capability_surface`; `load_capability_surface`.
- Shape contract consumed by Task 5: `acceleration_max[i, j]` is the limit at `(speed[i], curvature[j])`; `speed_ceiling[j]` belongs to `curvature[j]`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_point_mass_capability.py`:

```python
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import (
    acceleration_limits,
    point_mass_view_of_planar,
)
from simplified_vehicle_models.formulations.point_mass_capability import (
    build_capability_surface,
    capability_hash,
    load_capability_surface,
    save_capability_surface,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"


class CapabilitySurfaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = point_mass_view_of_planar(
            compile_formulation(str(FORMULATION)).model
        )
        cls.surface = build_capability_surface(
            cls.vehicle,
            curvature_limit=0.25,
            speed_points=41,
            curvature_points=21,
        )

    def test_axes_are_sorted_and_symmetric(self) -> None:
        self.assertTrue(np.all(np.diff(self.surface.speed) > 0.0))
        self.assertTrue(np.all(np.diff(self.surface.curvature) > 0.0))
        self.assertAlmostEqual(float(self.surface.curvature[10]), 0.0)

    def test_limits_reproduce_the_numpy_physics_at_grid_nodes(self) -> None:
        for row in (0, 13, 40):
            for column in (0, 10, 20):
                speed = float(self.surface.speed[row])
                curvature = float(self.surface.curvature[column])
                limits = acceleration_limits(self.vehicle, speed, curvature)
                expected_max = limits.maximum_acceleration
                expected_min = -limits.maximum_deceleration
                self.assertAlmostEqual(
                    float(self.surface.acceleration_max[row, column]),
                    expected_max,
                    delta=1.0e-9 * max(1.0, abs(expected_max)),
                )
                self.assertAlmostEqual(
                    float(self.surface.acceleration_min[row, column]),
                    expected_min,
                    delta=1.0e-9 * max(1.0, abs(expected_min)),
                )

    def test_speed_ceiling_is_where_acceleration_reaches_zero(self) -> None:
        for column in (0, 13, 20):
            ceiling = float(self.surface.speed_ceiling[column])
            curvature = float(self.surface.curvature[column])
            below = acceleration_limits(self.vehicle, ceiling - 0.05, curvature)
            self.assertGreaterEqual(below.maximum_acceleration, -1.0e-3)

    def test_hash_is_stable_and_hex(self) -> None:
        first = capability_hash(self.vehicle)
        self.assertEqual(first, capability_hash(self.vehicle))
        self.assertEqual(len(first), 64)
        int(first, 16)

    def test_round_trip_preserves_every_array(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "surface.npz"
            save_capability_surface(self.surface, path)
            restored = load_capability_surface(path)
        np.testing.assert_allclose(restored.speed, self.surface.speed)
        np.testing.assert_allclose(restored.curvature, self.surface.curvature)
        np.testing.assert_allclose(
            restored.acceleration_max, self.surface.acceleration_max
        )
        np.testing.assert_allclose(
            restored.acceleration_min, self.surface.acceleration_min
        )
        np.testing.assert_allclose(restored.speed_ceiling, self.surface.speed_ceiling)
        self.assertEqual(restored.provenance_hash, self.surface.provenance_hash)

    def test_rejects_a_bad_curvature_limit(self) -> None:
        with self.assertRaises(ValueError):
            build_capability_surface(self.vehicle, curvature_limit=0.0)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_point_mass_capability -v`
Expected: `ModuleNotFoundError: No module named 'simplified_vehicle_models.formulations.point_mass_capability'`.

- [ ] **Step 3: Write the capability module**

Create `src/simplified_vehicle_models/formulations/point_mass_capability.py`:

```python
"""Sample the quasi-steady capability of the point-mass vehicle onto a grid.

The grid is the only vehicle physics the stage 1 optimal control sees.  Every
value comes from the NumPy point-mass equilibrium solver, so the physics stays
single-sourced; CasADi only interpolates this table.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.formulations.point_mass import (
    acceleration_limits,
    maximum_acceleration_limit,
    maximum_vehicle_speed,
)


@dataclass(frozen=True, slots=True)
class CapabilitySurface:
    """Store the sampled acceleration limits and the cornering ceiling."""

    speed: FloatArray
    curvature: FloatArray
    acceleration_max: FloatArray
    acceleration_min: FloatArray
    speed_ceiling: FloatArray
    provenance_hash: str

    def __post_init__(self) -> None:
        for name in (
            "speed",
            "curvature",
            "acceleration_max",
            "acceleration_min",
            "speed_ceiling",
        ):
            values = np.array(getattr(self, name), dtype=float, copy=True)
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{name} must be finite")
            values.setflags(write=False)
            object.__setattr__(self, name, values)
        if self.acceleration_max.shape != (self.speed.size, self.curvature.size):
            raise ValueError("acceleration_max must have shape (speed, curvature)")
        if self.acceleration_min.shape != self.acceleration_max.shape:
            raise ValueError("acceleration_min must match acceleration_max")
        if self.speed_ceiling.shape != self.curvature.shape:
            raise ValueError("speed_ceiling must match the curvature axis")


def capability_hash(vehicle) -> str:
    """Return a stable hash of the scalar vehicle parameters this surface uses."""

    payload = "|".join(
        f"{name}={float(getattr(vehicle, name)):.12g}"
        for name in sorted(dir(vehicle))
        if not name.startswith("_")
        and isinstance(getattr(vehicle, name, None), (int, float))
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _ceiling_speed(vehicle, curvature: float, maximum: float, tolerance: float) -> float:
    """Return the steady-speed ceiling for one signed curvature."""

    def residual(speed: float) -> float:
        return maximum_acceleration_limit(vehicle, speed, curvature)

    if residual(maximum) >= 0.0:
        return maximum
    low = 1.0e-3
    if residual(low) < 0.0:
        return low
    return brentq(residual, low, maximum, xtol=tolerance, rtol=1.0e-12)


def build_capability_surface(
    vehicle,
    *,
    curvature_limit: float,
    speed_points: int = 121,
    curvature_points: int = 81,
    minimum_speed: float = 0.5,
    tolerance: float = 1.0e-4,
) -> CapabilitySurface:
    """Sample the acceleration envelope and the cornering ceiling."""

    if not math.isfinite(curvature_limit) or curvature_limit <= 0.0:
        raise ValueError("curvature_limit must be finite and positive")
    if speed_points < 3 or curvature_points < 3:
        raise ValueError("at least three samples per axis are required")

    top = min(float(maximum_vehicle_speed(vehicle)), 60.0)
    speed = np.linspace(minimum_speed, top, speed_points)
    curvature = np.linspace(-curvature_limit, curvature_limit, curvature_points)
    acceleration_max = np.empty((speed.size, curvature.size))
    acceleration_min = np.empty((speed.size, curvature.size))
    speed_ceiling = np.empty(curvature.size)

    for column, value in enumerate(curvature):
        for row, velocity in enumerate(speed):
            limits = acceleration_limits(vehicle, float(velocity), float(value))
            acceleration_max[row, column] = limits.maximum_acceleration
            acceleration_min[row, column] = -limits.maximum_deceleration
        speed_ceiling[column] = _ceiling_speed(vehicle, float(value), top, tolerance)

    return CapabilitySurface(
        speed=speed,
        curvature=curvature,
        acceleration_max=acceleration_max,
        acceleration_min=acceleration_min,
        speed_ceiling=speed_ceiling,
        provenance_hash=capability_hash(vehicle),
    )


def save_capability_surface(surface: CapabilitySurface, path: str | Path) -> Path:
    """Write a surface to a compressed NumPy archive."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        destination,
        speed=surface.speed,
        curvature=surface.curvature,
        acceleration_max=surface.acceleration_max,
        acceleration_min=surface.acceleration_min,
        speed_ceiling=surface.speed_ceiling,
        provenance_hash=np.asarray(surface.provenance_hash),
    )
    return destination


def load_capability_surface(path: str | Path) -> CapabilitySurface:
    """Read a surface written by :func:`save_capability_surface`."""

    with np.load(Path(path), allow_pickle=False) as data:
        return CapabilitySurface(
            speed=data["speed"],
            curvature=data["curvature"],
            acceleration_max=data["acceleration_max"],
            acceleration_min=data["acceleration_min"],
            speed_ceiling=data["speed_ceiling"],
            provenance_hash=str(data["provenance_hash"]),
        )
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `PYTHONPATH=src python3 -m unittest tests.test_point_mass_capability -v`
Expected: 6 tests, PASS. The grid-node reproduction test is load-bearing: it is the only thing preventing the surface from drifting away from the physics it is sampled from.

- [ ] **Step 5: Write the cache tool**

Create `tools/model_data/build_capability_surface.py`:

```python
"""Build and cache the quasi-steady capability surface for a formulation."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.point_mass_capability import (
    build_capability_surface,
    capability_hash,
    save_capability_surface,
)
from simplified_vehicle_models.tracks.profile import load_track


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formulation", required=True, type=Path)
    parser.add_argument("--track", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--curvature-margin", type=float, default=1.5)
    arguments = parser.parse_args(argv)

    vehicle = point_mass_view_of_planar(
        compile_formulation(str(arguments.formulation)).model
    )
    track = load_track(arguments.track)
    peak = float(abs(track.curvature).max())
    surface = build_capability_surface(
        vehicle, curvature_limit=arguments.curvature_margin * peak
    )
    path = save_capability_surface(surface, arguments.output)
    print(f"wrote {path} hash={capability_hash(vehicle)} peak_curvature={peak:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 6: Build the MIS surface and record the cost**

```bash
PYTHONPATH=src python3 -m tools.model_data.build_capability_surface \
  --formulation models/formulations/iem24_murphy_real_planar_7dof.xml \
  --track models/tracks/2024_michigan_endurance_closed_edged.xml \
  --output runs/capability/iem24_murphy_7dof_mis.npz
```

Expected: a few hundred kilobytes. Budget 5–20 minutes; every grid point is a root solve against the NumPy equilibrium. Record the wall time in the commit message. If it exceeds 20 minutes, halve `curvature_points` to 41 and rerun.

- [ ] **Step 7: Commit**

```bash
git add src/simplified_vehicle_models/formulations/point_mass_capability.py \
        tools/model_data/build_capability_surface.py \
        tests/test_point_mass_capability.py
git commit -m "feat: sample the point-mass capability onto a cached surface"
```

Do not commit `runs/`; it is a working directory.

---

### Task 5: Hermite–Simpson quasi-steady free-line transcription

**Files:**
- Create: `src/simplified_vehicle_models/solvers/qss_line_ocp.py`
- Test: `tests/test_qss_line_ocp.py`

**Interfaces:**
- Consumes: `Corridor` (Task 3), `CapabilitySurface` (Task 4).
- Produces: `QssSettings`, `QssSolution`, `build_mesh(track, spacing)`, `build_ocp(track, corridor, surface, mesh, settings)`, `initial_guess(track, corridor, surface, mesh, settings)`, `solve_line(track, corridor, surface, mesh, settings, *, seed=None)`.

**Contract that later tasks depend on.** The decision vector is **grouped by variable, not interleaved**:

```text
[ n_0..n_N | psi_0..psi_N | v_0..v_N | a_0..a_N | dpsi_0..dpsi_N | a_mid_0.. | dpsi_mid_0.. ]
```

size `5(N+1) + 2N`. The state vector is ordered `(n, ψ, v)`, so the rate vector is ordered `(dn/ds, dψ/ds, dv/ds)`. Both orderings are asserted in Step 5.

Equations, with `κ₀` the centreline curvature and `n` positive to the right:

```text
denominator   = 1 - κ₀ n
dn/ds         = tan(ψ) · denominator
dψ/ds         = control
dv/ds         = a · denominator / (v cos ψ)
dt/ds         = denominator / (v cos ψ)          objective
κ_vehicle     = (κ₀ + dψ/ds) cos ψ / denominator
```

- [ ] **Step 1: Write the failing tests**

Create `tests/test_qss_line_ocp.py`:

```python
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import (
    CapabilitySurface,
)
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


def _has_casadi() -> bool:
    try:
        import casadi  # noqa: F401
    except ImportError:
        return False
    return True


CASADI = _has_casadi()


def _flat_surface(ceiling: float, *, acceleration: float = 3.0,
                  deceleration: float = 8.0) -> CapabilitySurface:
    """Return a synthetic surface with a constant envelope."""

    speed = np.linspace(0.5, 40.0, 81)
    curvature = np.linspace(-0.3, 0.3, 61)
    return CapabilitySurface(
        speed=speed,
        curvature=curvature,
        acceleration_max=np.full((speed.size, curvature.size), acceleration),
        acceleration_min=np.full((speed.size, curvature.size), -deceleration),
        speed_ceiling=np.full(curvature.size, ceiling),
        provenance_hash="synthetic",
    )


@unittest.skipUnless(CASADI, "CasADi is not installed")
class TranscriptionTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def _track(self, **kwargs):
        defaults = {"radius": 40.0, "nodes": 360, "half_width": 3.0}
        defaults.update(kwargs)
        return load_track(_write_circle_track(self.directory, **defaults))

    def test_mesh_ends_on_the_lap_length(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh

        track = self._track()
        mesh = build_mesh(track, spacing=2.5)
        self.assertAlmostEqual(float(mesh[0]), 0.0)
        self.assertAlmostEqual(float(mesh[-1]), track.lap_length, places=9)
        self.assertTrue(np.all(np.diff(mesh) > 0.0))

    def test_decision_layout_matches_the_declared_contract(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import (
            build_mesh,
            build_ocp,
            QssSettings,
        )

        track = self._track()
        mesh = build_mesh(track, spacing=4.0)
        corridor = corridor_from_track(track)
        built = build_ocp(track, corridor, _flat_surface(25.0), mesh, QssSettings())
        count = mesh.size - 1
        self.assertEqual(built["nlp"]["x"].shape[0], 5 * (count + 1) + 2 * count)

    def test_zero_corridor_recovers_the_steady_circle_time(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import (
            build_mesh,
            solve_line,
            QssSettings,
        )

        radius = 40.0
        track = self._track(radius=radius, half_width=2.25)
        corridor = corridor_from_track(track, margin=2.25)
        solution = solve_line(
            track,
            corridor,
            _flat_surface(20.0),
            build_mesh(track, 2.0),
            QssSettings(maximum_iterations=4000),
        )
        self.assertEqual(solution.status, "Solve_Succeeded")
        expected = 2.0 * math.pi * radius / 20.0
        self.assertAlmostEqual(solution.lap_time, expected, delta=2.0e-3 * expected)
        self.assertLess(float(np.max(np.abs(solution.offset))), 1.0e-9)

    def test_right_turn_free_line_hugs_the_positive_edge(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import (
            build_mesh,
            solve_line,
            QssSettings,
        )

        radius, half_width = 40.0, 3.0
        track = self._track(radius=radius, half_width=half_width, direction=1.0)
        corridor = corridor_from_track(track)
        solution = solve_line(
            track,
            corridor,
            _flat_surface(25.0),
            build_mesh(track, 2.0),
            QssSettings(maximum_iterations=6000),
        )
        self.assertEqual(solution.status, "Solve_Succeeded")
        self.assertGreater(float(np.min(solution.offset)), 0.9 * half_width)
        expected = 2.0 * math.pi * (radius - half_width) / 25.0
        self.assertAlmostEqual(solution.lap_time, expected, delta=5.0e-3 * expected)

    def test_left_turn_free_line_hugs_the_negative_edge(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import (
            build_mesh,
            solve_line,
            QssSettings,
        )

        radius, half_width = 40.0, 3.0
        track = self._track(radius=radius, half_width=half_width, direction=-1.0)
        corridor = corridor_from_track(track)
        solution = solve_line(
            track,
            corridor,
            _flat_surface(25.0),
            build_mesh(track, 2.0),
            QssSettings(maximum_iterations=6000),
        )
        self.assertEqual(solution.status, "Solve_Succeeded")
        self.assertLess(float(np.max(solution.offset)), -0.9 * half_width)
        expected = 2.0 * math.pi * (radius - half_width) / 25.0
        self.assertAlmostEqual(solution.lap_time, expected, delta=5.0e-3 * expected)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_qss_line_ocp -v`
Expected: `ModuleNotFoundError: No module named 'simplified_vehicle_models.solvers.qss_line_ocp'`, or all skipped if CasADi is absent. Install it for development with:

```bash
python3 -m pip install --target /tmp/casadi_pkgs casadi
```

- [ ] **Step 3: Write the transcription**

Create `src/simplified_vehicle_models/solvers/qss_line_ocp.py`:

```python
"""Minimum-time quasi-steady racing line in the distance domain.

States are the lateral offset, the heading error, and the speed.  Controls are
the longitudinal acceleration and the rate of change of heading error with
respect to the centreline arc length.  The vehicle's acceleration limits come
from a sampled capability surface, so this module contains no vehicle physics.

The decision vector is grouped by variable, not interleaved:
    [n | psi | v | a | dpsi/ds | a_mid | dpsi/ds_mid]
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class QssSettings:
    """Store the transcription and solver settings."""

    fold_guard: float = 0.2
    maximum_heading: float = 0.6
    maximum_curvature_rate: float = 0.5
    minimum_speed: float = 0.5
    regularisation: float = 1.0e-6
    tolerance: float = 1.0e-6
    maximum_iterations: int = 3000


@dataclass(frozen=True, slots=True)
class QssSolution:
    """Store one accepted transcription solution."""

    distance: FloatArray
    offset: FloatArray
    heading_error: FloatArray
    speed: FloatArray
    acceleration: FloatArray
    curvature_rate: FloatArray
    lap_time: float
    status: str
    iterations: int
    maximum_violation: float


def build_mesh(track, spacing: float) -> FloatArray:
    """Return a uniform mesh that ends exactly on the lap length."""

    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("spacing must be finite and positive")
    length = float(track.lap_length)
    count = max(int(math.ceil(length / spacing)), 4)
    return np.linspace(0.0, length, count + 1)


def periodic_sample(values, distance, positions) -> FloatArray:
    """Sample a periodic array defined on ``distance`` at arbitrary positions."""

    values = np.asarray(values, dtype=float)
    distance = np.asarray(distance, dtype=float)
    extended = np.r_[values, values[0]]
    nodes = np.r_[distance, distance[-1] + (distance[1] - distance[0])]
    return np.interp(np.asarray(positions, dtype=float), nodes, extended)


def _element_curvature(track, mesh) -> FloatArray:
    """Return the centreline curvature at the mesh nodes."""

    return periodic_sample(track.curvature, track.distance, mesh)


def build_ocp(track, corridor, surface, mesh, settings: QssSettings):
    """Build the CasADi NLP and return it with its bound vectors."""

    import casadi as ca

    mesh = np.asarray(mesh, dtype=float)
    count = mesh.size - 1
    nodes = count + 1

    acceleration_max = ca.interpolant(
        "a_max", "linear", [surface.speed, surface.curvature], surface.acceleration_max
    )
    acceleration_min = ca.interpolant(
        "a_min", "linear", [surface.speed, surface.curvature], surface.acceleration_min
    )
    ceiling = ca.interpolant(
        "v_ceiling", "linear", [surface.curvature], surface.speed_ceiling
    )

    kappa0 = _element_curvature(track, mesh)
    n_lower = periodic_sample(corridor.minimum, track.distance, mesh)
    n_upper = periodic_sample(corridor.maximum, track.distance, mesh)

    offsets = [ca.MX.sym(f"n_{index}") for index in range(nodes)]
    headings = [ca.MX.sym(f"psi_{index}") for index in range(nodes)]
    speeds = [ca.MX.sym(f"v_{index}") for index in range(nodes)]
    accelerations = [ca.MX.sym(f"a_{index}") for index in range(nodes)]
    rates_psi = [ca.MX.sym(f"k_{index}") for index in range(nodes)]
    mid_accelerations = [ca.MX.sym(f"am_{index}") for index in range(count)]
    mid_rates = [ca.MX.sym(f"km_{index}") for index in range(count)]

    vector = ca.vertcat(
        *offsets,
        *headings,
        *speeds,
        *accelerations,
        *rates_psi,
        *mid_accelerations,
        *mid_rates,
    )

    acceleration_box = float(
        max(surface.acceleration_max.max(), -surface.acceleration_min.min())
    )
    lower = np.concatenate(
        [
            n_lower,
            np.full(nodes, -settings.maximum_heading),
            np.full(nodes, settings.minimum_speed),
            np.full(nodes, -acceleration_box),
            np.full(nodes, -settings.maximum_curvature_rate),
            np.full(count, -acceleration_box),
            np.full(count, -settings.maximum_curvature_rate),
        ]
    )
    upper = np.concatenate(
        [
            n_upper,
            np.full(nodes, settings.maximum_heading),
            np.full(nodes, float(surface.speed[-1])),
            np.full(nodes, acceleration_box),
            np.full(nodes, settings.maximum_curvature_rate),
            np.full(count, acceleration_box),
            np.full(count, settings.maximum_curvature_rate),
        ]
    )

    def rates(state, control, curvature0):
        """Return ``(dn/ds, dpsi/ds, dv/ds)`` for one point."""

        offset, heading, speed = state
        acceleration, rate_psi = control
        denominator = 1.0 - curvature0 * offset
        return ca.vertcat(
            ca.tan(heading) * denominator,
            rate_psi,
            acceleration * denominator / (speed * ca.cos(heading)),
        )

    def vehicle_curvature(offset, heading, rate_psi, curvature0):
        return (curvature0 + rate_psi) * ca.cos(heading) / (1.0 - curvature0 * offset)

    def time_rate(offset, heading, speed, curvature0):
        return (1.0 - curvature0 * offset) / (speed * ca.cos(heading))

    constraints = []
    constraint_lower: list[float] = []
    constraint_upper: list[float] = []

    def add(expression, low, high):
        constraints.append(expression)
        constraint_lower.append(low)
        constraint_upper.append(high)

    def add_limits(offset, heading, speed, acceleration, rate_psi, curvature0):
        curvature = vehicle_curvature(offset, heading, rate_psi, curvature0)
        add(curvature, float(surface.curvature[0]), float(surface.curvature[-1]))
        add(acceleration - acceleration_min(speed, curvature), 0.0, float("inf"))
        add(acceleration_max(speed, curvature) - acceleration, 0.0, float("inf"))
        add(ceiling(curvature) - speed, 0.0, float("inf"))
        add(1.0 - curvature0 * offset, settings.fold_guard, float("inf"))

    objective = settings.regularisation * ca.sumsqr(ca.vertcat(*rates_psi))
    for index in range(count):
        step = float(mesh[index + 1] - mesh[index])
        middle_position = 0.5 * (mesh[index] + mesh[index + 1])
        kappa_middle = float(_element_curvature(track, np.asarray([middle_position]))[0])
        kappa_start = float(kappa0[index])
        kappa_end = float(kappa0[index + 1])

        start_state = ca.vertcat(offsets[index], headings[index], speeds[index])
        end_state = ca.vertcat(
            offsets[index + 1], headings[index + 1], speeds[index + 1]
        )
        start_rate = rates(
            (offsets[index], headings[index], speeds[index]),
            (accelerations[index], rates_psi[index]),
            kappa_start,
        )
        end_rate = rates(
            (offsets[index + 1], headings[index + 1], speeds[index + 1]),
            (accelerations[index + 1], rates_psi[index + 1]),
            kappa_end,
        )
        middle_state = 0.5 * (start_state + end_state) + step / 8.0 * (
            start_rate - end_rate
        )
        middle_rate = rates(
            (middle_state[0], middle_state[1], middle_state[2]),
            (mid_accelerations[index], mid_rates[index]),
            kappa_middle,
        )

        defect = end_state - start_state - step / 6.0 * (
            start_rate + 4.0 * middle_rate + end_rate
        )
        for row in range(3):
            add(defect[row], 0.0, 0.0)

        objective += step / 6.0 * (
            time_rate(offsets[index], headings[index], speeds[index], kappa_start)
            + 4.0
            * time_rate(
                middle_state[0], middle_state[1], middle_state[2], kappa_middle
            )
            + time_rate(
                offsets[index + 1], headings[index + 1], speeds[index + 1], kappa_end
            )
        )

        add_limits(
            offsets[index], headings[index], speeds[index],
            accelerations[index], rates_psi[index], kappa_start,
        )
        add_limits(
            middle_state[0], middle_state[1], middle_state[2],
            mid_accelerations[index], mid_rates[index], kappa_middle,
        )
        add_limits(
            offsets[index + 1], headings[index + 1], speeds[index + 1],
            accelerations[index + 1], rates_psi[index + 1], kappa_end,
        )

    add(offsets[0] - offsets[-1], 0.0, 0.0)
    add(headings[0] - headings[-1], 0.0, 0.0)
    add(speeds[0] - speeds[-1], 0.0, 0.0)

    return {
        "nlp": {"x": vector, "f": objective, "g": ca.vertcat(*constraints)},
        "x_lower": lower,
        "x_upper": upper,
        "g_lower": np.asarray(constraint_lower, dtype=float),
        "g_upper": np.asarray(constraint_upper, dtype=float),
        "mesh": mesh,
        "count": count,
    }
```

- [ ] **Step 4: Write the seed, the solver, and the unpacking**

Append to `src/simplified_vehicle_models/solvers/qss_line_ocp.py`:

```python
def unpack(values, count: int) -> dict[str, FloatArray]:
    """Split the grouped decision vector into named node arrays."""

    nodes = count + 1
    expected = 5 * nodes + 2 * count
    flat = np.asarray(values, dtype=float).ravel()
    if flat.size != expected:
        raise ValueError(
            f"decision vector has {flat.size} entries, layout needs {expected}"
        )
    blocks = {}
    names = ("offset", "heading", "speed", "acceleration", "curvature_rate")
    for index, name in enumerate(names):
        blocks[name] = flat[index * nodes:(index + 1) * nodes]
    tail = flat[5 * nodes:]
    blocks["mid_acceleration"] = tail[:count]
    blocks["mid_curvature_rate"] = tail[count:]
    return blocks


def initial_guess(track, corridor, surface, mesh, settings: QssSettings) -> FloatArray:
    """Return a flat starting vector inside every bound.

    The offset is the corridor midpoint, the heading error is zero, and the
    speed is a fraction of the cornering ceiling on the centreline.
    """

    mesh = np.asarray(mesh, dtype=float)
    count = mesh.size - 1
    offsets = 0.5 * (
        periodic_sample(corridor.minimum, track.distance, mesh)
        + periodic_sample(corridor.maximum, track.distance, mesh)
    )
    centre_curvature = np.clip(
        _element_curvature(track, mesh),
        float(surface.curvature[0]),
        float(surface.curvature[-1]),
    )
    ceiling = np.interp(
        centre_curvature, surface.curvature, surface.speed_ceiling
    )
    speeds = np.maximum(
        0.6 * float(np.min(ceiling)), settings.minimum_speed + 0.1
    ) * np.ones(mesh.size)
    headings = np.zeros(mesh.size)
    accelerations = np.zeros(mesh.size)
    curvature_rates = np.zeros(mesh.size)
    mids = np.zeros(count)
    return np.concatenate(
        [offsets, headings, speeds, accelerations, curvature_rates, mids, mids]
    )


def solve_line(track, corridor, surface, mesh, settings: QssSettings,
               *, seed: FloatArray | None = None) -> QssSolution:
    """Solve the free-line minimum-time problem on one given mesh."""

    import casadi as ca

    built = build_ocp(track, corridor, surface, mesh, settings)
    count = built["count"]
    nodes = count + 1
    if seed is None:
        vector = initial_guess(track, corridor, surface, mesh, settings)
    else:
        vector = np.asarray(seed, dtype=float).ravel()
        if vector.size != built["nlp"]["x"].shape[0]:
            raise ValueError("seed does not match the mesh")
        vector = np.clip(vector, built["x_lower"], built["x_upper"])

    options = {
        "ipopt.tol": settings.tolerance,
        "ipopt.constr_viol_tol": settings.tolerance,
        "ipopt.max_iter": settings.maximum_iterations,
        "ipopt.hessian_approximation": "exact",
        "ipopt.nlp_scaling_method": "gradient-based",
        "ipopt.print_level": 0,
        "print_time": False,
    }
    solver = ca.nlpsol("qss_line", "ipopt", built["nlp"], options)
    result = solver(
        x0=vector,
        lbx=built["x_lower"],
        ubx=built["x_upper"],
        lbg=built["g_lower"],
        ubg=built["g_upper"],
    )
    values = np.asarray(result["x"], dtype=float).ravel()
    constraints = np.asarray(result["g"], dtype=float).ravel()
    violation = float(
        np.max(
            np.maximum(
                np.maximum(built["g_lower"] - constraints, 0.0),
                constraints - built["g_upper"],
            )
        )
    )
    stats = solver.stats()
    blocks = unpack(values, count)
    assert blocks["offset"].size == nodes
    return QssSolution(
        distance=np.asarray(mesh, dtype=float),
        offset=blocks["offset"],
        heading_error=blocks["heading"],
        speed=blocks["speed"],
        acceleration=blocks["acceleration"],
        curvature_rate=blocks["curvature_rate"],
        lap_time=float(result["f"]),
        status=str(stats["return_status"]),
        iterations=int(stats["iter_count"]),
        maximum_violation=violation,
    )


def project_seed(coarse: QssSolution, mesh) -> FloatArray:
    """Interpolate a coarse solution onto a new mesh, clipped by its bounds later."""

    mesh = np.asarray(mesh, dtype=float)
    count = mesh.size - 1

    def closed(values):
        values = np.asarray(values, dtype=float)
        return np.r_[values, values[0]]

    source = np.r_[coarse.distance]
    if source.size != coarse.offset.size:
        raise ValueError("solution distance does not match its node count")
    offsets = np.interp(mesh, source, closed(coarse.offset))
    headings = np.interp(mesh, source, closed(coarse.heading_error))
    speeds = np.interp(mesh, source, closed(coarse.speed))
    accelerations = np.interp(mesh, source, closed(coarse.acceleration))
    curvature_rates = np.interp(mesh, source, closed(coarse.curvature_rate))
    mids = np.zeros(count)
    return np.concatenate(
        [offsets, headings, speeds, accelerations, curvature_rates, mids, mids]
    )
```

`np.interp` on `source` that ends at the lap length: positions equal to the lap length return `closed(...)[-1]`, which is the first node value, matching periodicity.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_qss_line_ocp -v`
Expected: 5 tests, PASS.

If `test_right_turn_free_line_hugs_the_positive_edge` selects the negative edge, the bounds are grouped in the wrong order — check that `n_lower`/`n_upper` occupy the **first** block of `lower`/`upper`, not an interleaved block. If `test_zero_corridor_recovers_the_steady_circle_time` diverges, check that `rates` returns `(dn, dψ, dv)` and not `(dn, dv, dψ)`.

- [ ] **Step 6: Commit**

```bash
git add src/simplified_vehicle_models/solvers/qss_line_ocp.py tests/test_qss_line_ocp.py
git commit -m "feat: transcribe the quasi-steady free-line minimum-time problem"
```

---

### Task 6: Mesh ladder solve

**Files:**
- Modify: `src/simplified_vehicle_models/solvers/qss_line_ocp.py` (add `solve_ladder`)
- Test: `tests/test_qss_line_ocp.py` (append)

**Interfaces:**
- Produces: `solve_ladder(track, corridor, surface, settings, spacings=(5.0, 2.0, 1.0), *, seed=None) -> list[QssSolution]`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_qss_line_ocp.py`:

```python
@unittest.skipUnless(CASADI, "CasADi is not installed")
class LadderTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def test_ladder_converges_and_is_mesh_independent(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import (
            QssSettings,
            solve_ladder,
        )

        track = load_track(
            _write_circle_track(
                self.directory, radius=40.0, nodes=360, half_width=3.0
            )
        )
        corridor = corridor_from_track(track)
        stages = solve_ladder(
            track,
            corridor,
            _flat_surface(25.0),
            QssSettings(maximum_iterations=6000),
            spacings=(4.0, 2.0, 1.0),
        )
        self.assertEqual(len(stages), 3)
        for stage in stages:
            self.assertEqual(stage.status, "Solve_Succeeded")
        coarse, finest = stages[-2], stages[-1]
        self.assertAlmostEqual(
            finest.lap_time, coarse.lap_time, delta=5.0e-3 * coarse.lap_time
        )
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_qss_line_ocp.LadderTest -v`
Expected: `AttributeError: module ... has no attribute 'solve_ladder'`.

- [ ] **Step 3: Implement the ladder**

Append to `src/simplified_vehicle_models/solvers/qss_line_ocp.py`:

```python
def solve_ladder(track, corridor, surface, settings: QssSettings,
                 spacings: tuple[float, ...] = (5.0, 2.0, 1.0),
                 *, seed: FloatArray | None = None) -> list[QssSolution]:
    """Solve coarse to fine, warm starting each stage from the one before.

    Each stage builds its own mesh, and the previous solution is projected onto
    that mesh before the solve.  The projection interpolates the state and
    control arrays only; this transcription has no algebraic variables, so no
    consistency re-solve is needed and no mismatched seed can reach the solver.
    """

    if not spacings:
        raise ValueError("at least one mesh spacing is required")
    stages: list[QssSolution] = []
    current = seed
    for spacing in spacings:
        mesh = build_mesh(track, spacing)
        solution = solve_line(track, corridor, surface, mesh, settings, seed=current)
        if solution.status != "Solve_Succeeded":
            raise RuntimeError(
                f"stage at {spacing} m failed with {solution.status} "
                f"after {solution.iterations} iterations"
            )
        stages.append(solution)
        current = project_seed(solution, mesh)
    return stages
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_qss_line_ocp -v`
Expected: 6 tests, PASS.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/solvers/qss_line_ocp.py tests/test_qss_line_ocp.py
git commit -m "feat: solve the free-line problem on a coarse-to-fine mesh ladder"
```

---

### Task 7: Racing line back to a path

**Files:**
- Create: `src/simplified_vehicle_models/tracks/racing_line.py`
- Test: `tests/test_racing_line.py`

**Interfaces:**
- Produces: `offset_path(coordinates, offset) -> PathCoordinates`; `path_from_solution(track, solution, *, smoothing=0.0) -> PathCoordinates`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_racing_line.py`:

```python
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.racing_line import offset_path
from tests.test_track_width import _write_circle_track

#: A 720-node circle resolves the discrete curvature to about 1e-5 relative.
NODES = 720


class OffsetPathTest(unittest.TestCase):
    def test_positive_offset_on_a_right_hand_circle_is_an_inside_circle(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            radius = 40.0
            track = load_track(
                _write_circle_track(
                    Path(name), radius=radius, nodes=NODES, half_width=3.0
                )
            )
            path = offset_path(
                track.path_coordinates, np.full(track.distance.shape, 3.0)
            )
            np.testing.assert_allclose(
                path.curvature, 1.0 / (radius - 3.0), rtol=1.0e-4
            )
            self.assertLess(path.lap_length, track.lap_length)

    def test_negative_offset_widens_the_radius(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            radius = 40.0
            track = load_track(
                _write_circle_track(
                    Path(name), radius=radius, nodes=NODES, half_width=3.0
                )
            )
            path = offset_path(
                track.path_coordinates, np.full(track.distance.shape, -2.0)
            )
            np.testing.assert_allclose(
                path.curvature, 1.0 / (radius + 2.0), rtol=1.0e-4
            )

    def test_zero_offset_reproduces_the_centreline_exactly(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            track = load_track(
                _write_circle_track(Path(name), radius=40.0, nodes=NODES, half_width=3.0)
            )
            path = offset_path(
                track.path_coordinates, np.zeros(track.distance.shape)
            )
            np.testing.assert_allclose(
                path.x_north, track.path_coordinates.x_north
            )
            np.testing.assert_allclose(path.y_east, track.path_coordinates.y_east)
            np.testing.assert_allclose(path.curvature, track.curvature, rtol=1.0e-4)

    def test_offset_length_must_match(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            track = load_track(
                _write_circle_track(Path(name), radius=40.0, nodes=NODES, half_width=3.0)
            )
            with self.assertRaises(ValueError):
                offset_path(track.path_coordinates, np.zeros(3))
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_racing_line -v`
Expected: `ModuleNotFoundError: No module named 'simplified_vehicle_models.tracks.racing_line'`.

- [ ] **Step 3: Write the module**

Create `src/simplified_vehicle_models/tracks/racing_line.py`:

```python
"""Turn a lateral offset profile into a drivable signed path.

The offset is positive to the right of the direction of travel, matching the
plant's ``n``.  In the repository frame (``x`` north, ``y`` east, ``z`` down,
right handed) the unit tangent at heading ``theta`` is ``(cos theta, sin
theta)`` and the right normal is ``(-sin theta, cos theta)``.
"""

from __future__ import annotations

import numpy as np

from simplified_vehicle_models.tracks.types import PathCoordinates


def _periodic_derivative(values, distance):
    """Return d(values)/d(distance) for a closed path whose last node repeats the first.

    A plain ``np.gradient`` is wrong at the seam: it takes a one-sided difference
    across the duplicated node and returns half the true curvature there.  The
    wrapped central difference below is correct at every node, including the seam.
    """

    length = float(distance[-1])
    unique = np.asarray(distance, dtype=float)[:-1]
    nodes = np.asarray(values, dtype=float)[:-1]
    forward = np.diff(np.r_[unique, unique[0] + length])
    backward = np.r_[forward[-1], forward[:-1]]
    derivative = (np.roll(nodes, -1) - np.roll(nodes, 1)) / (forward + backward)
    return np.r_[derivative, derivative[0]]


def offset_path(coordinates: PathCoordinates, offset) -> PathCoordinates:
    """Return the path displaced laterally by ``offset`` at every node."""

    values = np.asarray(offset, dtype=float)
    if values.shape != coordinates.distance.shape:
        raise ValueError("offset must match the path node count")
    if not np.all(np.isfinite(values)):
        raise ValueError("offset must be finite")

    heading = np.asarray(coordinates.heading, dtype=float)
    north = np.asarray(coordinates.x_north, dtype=float)
    east = np.asarray(coordinates.y_east, dtype=float)
    x = north - values * np.sin(heading)
    y = east + values * np.cos(heading)

    step = np.hypot(np.diff(x), np.diff(y))
    distance = np.r_[0.0, np.cumsum(step)]

    if coordinates.is_closed:
        tangent_north = _periodic_derivative(x, distance)
        tangent_east = _periodic_derivative(y, distance)
    else:
        tangent_north = np.gradient(x, distance)
        tangent_east = np.gradient(y, distance)

    angles = np.arctan2(tangent_east, tangent_north)
    new_heading = np.unwrap(angles)
    if coordinates.is_closed:
        # Wrap the angular difference into (-pi, pi] before dividing, so the
        # seam needs no special case and the tangent angle can never jump.
        delta = np.roll(angles, -1) - np.roll(angles, 1)
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        length = float(distance[-1])
        unique = np.asarray(distance, dtype=float)[:-1]
        forward = np.diff(np.r_[unique, unique[0] + length])
        backward = np.r_[forward[-1], forward[:-1]]
        curvature = delta / (forward + backward)
        curvature = np.r_[curvature, curvature[0]]
    else:
        curvature = np.gradient(new_heading, distance)

    for array in (x, y, new_heading, curvature, distance):
        array.setflags(write=False)
    return PathCoordinates(
        distance=distance,
        x_north=x,
        y_east=y,
        heading=new_heading,
        curvature=curvature,
        is_closed=coordinates.is_closed,
    )


def _periodic_smooth(values, sigma_samples: float):
    """Return a periodic Gaussian moving average of ``values``."""

    values = np.asarray(values, dtype=float)
    half = int(max(1, round(4.0 * sigma_samples)))
    taps = np.arange(-half, half + 1)
    kernel = np.exp(-0.5 * (taps / sigma_samples) ** 2)
    kernel /= kernel.sum()
    padded = np.r_[values[-half:], values, values[:half]]
    return np.convolve(padded, kernel, mode="valid")


def path_from_solution(track, solution, *, smoothing: float = 0.0) -> PathCoordinates:
    """Return the racing line of one solution as a signed path.

    ``smoothing`` applies a periodic Gaussian average to the offset before the
    displacement.  Use it only when the raw line carries curvature noise that
    the mesh, not the physics, produced.
    """

    from simplified_vehicle_models.solvers.qss_line_ocp import periodic_sample

    offset = periodic_sample(
        np.asarray(solution.offset, dtype=float)[:-1],
        np.asarray(solution.distance, dtype=float)[:-1],
        np.asarray(track.distance, dtype=float),
    )
    if smoothing > 0.0:
        spacing = float(track.lap_length) / float(track.distance.size - 1)
        offset = _periodic_smooth(offset[:-1], smoothing / spacing)
        offset = np.r_[offset, offset[0]]
    return offset_path(track.path_coordinates, offset)
```

`periodic_sample` receives the solution's first `N` nodes because its last node repeats the first; passing both would create a duplicate interval. The rebuild then resamples onto the track's own node grid, so the reported path always lives on the geometry grid.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `PYTHONPATH=src python3 -m unittest tests.test_racing_line -v`
Expected: 4 tests, PASS. The first three are the definitive lateral-sign check: `corridor_from_track`, the transcription and the path rebuild must all agree, and if the inside-line test fails while the transcription tests pass, the fault is here.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/tracks/racing_line.py tests/test_racing_line.py
git commit -m "feat: rebuild a signed path from a lateral offset profile"
```

---

### Task 8: End-to-end run, the five acceptance checks, and documentation

**Files:**
- Create: `tools/run/solve_minimum_time_line.py`
- Create: `tools/gates/check_minimum_time_line.py`
- Create: `docs/dev/minimum-time-line.md`
- Test: `tests/test_minimum_time_line_gate.py`

**Interfaces:**
- Consumes: everything from Tasks 1–7.
- Produces: a run directory with `summary.json`, `line.csv`, `racing_line.csv`, `racing_line_track.xml`, `racing_line_track.csv`.

- [ ] **Step 1: Write the failing gate test**

Create `tests/test_minimum_time_line_gate.py`:

```python
import unittest

import numpy as np

from tools.gates.check_minimum_time_line import (
    check_corridor_containment,
    check_free_line_not_slower,
    check_mesh_convergence,
    check_replay_agreement,
    check_zero_corridor_matches_envelope,
)


class GateCheckTest(unittest.TestCase):
    def test_zero_corridor_check_uses_relative_tolerance(self) -> None:
        self.assertTrue(
            check_zero_corridor_matches_envelope(
                open_loop=61.14, ocp=61.30, tolerance=0.01
            )
        )
        self.assertFalse(
            check_zero_corridor_matches_envelope(
                open_loop=61.14, ocp=65.00, tolerance=0.01
            )
        )

    def test_free_line_check_rejects_a_slower_line(self) -> None:
        self.assertTrue(check_free_line_not_slower(centreline=61.14, free=58.90))
        self.assertFalse(check_free_line_not_slower(centreline=61.14, free=61.60))

    def test_containment_check_tests_both_edges(self) -> None:
        minimum = np.array([-1.0, -2.0, -1.0])
        maximum = np.array([1.0, 2.0, 1.0])
        self.assertTrue(
            check_corridor_containment(np.array([0.0, 1.9, -0.9]), minimum, maximum)
        )
        self.assertFalse(
            check_corridor_containment(np.array([0.0, 2.1, -0.9]), minimum, maximum)
        )

    def test_mesh_convergence_check_uses_relative_change(self) -> None:
        self.assertTrue(check_mesh_convergence(coarse=58.90, fine=58.85, tolerance=0.005))
        self.assertFalse(check_mesh_convergence(coarse=58.90, fine=59.40, tolerance=0.005))

    def test_replay_check_compares_lap_times(self) -> None:
        self.assertTrue(check_replay_agreement(ocp=58.85, replay=58.80, tolerance=0.01))
        self.assertFalse(check_replay_agreement(ocp=58.85, replay=59.60, tolerance=0.01))
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_minimum_time_line_gate -v`
Expected: `ModuleNotFoundError: No module named 'tools.gates.check_minimum_time_line'`.

- [ ] **Step 3: Write the gate module**

Create `tools/gates/check_minimum_time_line.py`:

```python
"""Stage 1 acceptance checks for the minimum-time line.

Each check is a pure function of measured values so it can be unit tested.  The
run tool computes the values and reports all five.
"""

from __future__ import annotations

import numpy as np


def check_zero_corridor_matches_envelope(*, open_loop: float, ocp: float,
                                         tolerance: float) -> bool:
    """A collapsed corridor must reproduce the centreline envelope."""

    return abs(ocp - open_loop) <= tolerance * open_loop


def check_free_line_not_slower(*, centreline: float, free: float) -> bool:
    """The free line must not be slower than the centreline."""

    return free <= centreline


def check_corridor_containment(offset, minimum, maximum) -> bool:
    """Every reported offset must lie inside the corridor."""

    values = np.asarray(offset, dtype=float)
    return bool(
        np.all(values >= np.asarray(minimum, dtype=float) - 1.0e-6)
        and np.all(values <= np.asarray(maximum, dtype=float) + 1.0e-6)
    )


def check_mesh_convergence(*, coarse: float, fine: float, tolerance: float) -> bool:
    """Two mesh levels must agree to within a relative tolerance."""

    return abs(fine - coarse) <= tolerance * coarse


def check_replay_agreement(*, ocp: float, replay: float, tolerance: float) -> bool:
    """The independent solver on the rebuilt path must reproduce the OCP lap."""

    return abs(replay - ocp) <= tolerance * ocp
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src python3 -m unittest tests.test_minimum_time_line_gate -v`
Expected: 5 tests, PASS.

- [ ] **Step 5: Write the run tool**

Create `tools/run/solve_minimum_time_line.py`. It must, in order:

1. Load the edged track and build `corridor = corridor_from_track(track, margin=arguments.margin)`.
2. Compile the formulation, build `point_mass_view_of_planar`, and load the surface; abort with an explicit message if `surface.provenance_hash != capability_hash(vehicle)`.
3. Run `solve_lap(vehicle, track, LapMode.FLYING)` on the **edged track** and record `lap.lap_time` as `centreline_envelope_s`.
4. Run `solve_line` with `corridor_from_track(track, margin=corridor.half_width)` to collapse the corridor for the zero-corridor control, and record `ocp_zero_corridor_s`.
5. Run `solve_ladder(track, corridor, surface, settings, spacings=(5.0, 2.0, 1.0))`, keep the finest stage, and record `ocp_free_line_s`, the per-stage iterations and wall times, and `maximum_violation`.
6. Rebuild the path with `path_from_solution`, write `racing_line_track.xml` and `racing_line_track.csv` into the run directory, load it with `load_track`, and run `solve_lap` on it to obtain `replay_lap_s`.
7. Evaluate the five checks. Tolerances: 1 % for the zero-corridor and replay checks, 0.5 % for mesh convergence, and the containment check against the corridor resampled onto the solution mesh with `periodic_sample`.
8. Write `summary.json`, print a table, and exit 1 if any check is false. Set `conditional_result` to `track.width_provenance != "surveyed"`.

`summary.json` shape:

```json
{
  "track": "models/tracks/2024_michigan_endurance_closed_edged.xml",
  "corridor_provenance": "assumed",
  "conditional_result": true,
  "margin_m": 0.1,
  "mesh_spacing_m": 1.0,
  "centreline_envelope_s": 0.0,
  "ocp_zero_corridor_s": 0.0,
  "ocp_free_line_s": 0.0,
  "replay_lap_s": 0.0,
  "free_line_gain_s": 0.0,
  "free_line_gain_percent": 0.0,
  "stages": [{"spacing_m": 5.0, "iterations": 0, "wall_seconds": 0.0, "lap_time_s": 0.0}],
  "maximum_violation": 0.0,
  "checks": {
    "zero_corridor_matches_envelope": false,
    "free_line_not_slower": false,
    "corridor_containment": false,
    "mesh_convergence": false,
    "replay_agreement": false
  }
}
```

- [ ] **Step 6: Run the full stage 1 pipeline**

```bash
PYTHONPATH=src:/tmp/casadi_pkgs python3 -m tools.run.solve_minimum_time_line \
  --track models/tracks/2024_michigan_endurance_closed_edged.xml \
  --formulation models/formulations/iem24_murphy_real_planar_7dof.xml \
  --surface runs/capability/iem24_murphy_7dof_mis.npz \
  --output runs/minimum_time_line/stage1 \
  --margin 0.10
```

Expected: all five checks `true`, `conditional_result: true`, and a free-line gain between 2 % and 12 %. Record the actual numbers for Step 7.

Diagnostics if a check fails:
- `corridor_containment` fails: the mesh sampling of the corridor is off by one node, or `periodic_sample` is being handed the duplicated final node.
- `zero_corridor_matches_envelope` fails: the surface and `solve_lap` disagree about the speed ceiling; re-check Task 4 Step 4.
- `replay_agreement` fails: the path rebuild sign is wrong; re-check Task 7 Step 4.
- `mesh_convergence` fails: raise `QssSettings.regularisation` by 10× and rerun; if it still fails, the mesh needs to be graded rather than uniform, which is a stage 2 change.

- [ ] **Step 7: Write the developer documentation**

Create `docs/dev/minimum-time-line.md` in the ASD-STE100 style used by [forward-driver.md](forward-driver.md): purpose, files, terms, how the solver works, how to run it, known limits. It must state:

- The corridor is `assumed` unless the track says `surveyed`, and a result on an assumed corridor is conditional.
- What the solver does not model: transient dynamics, sideslip, suspension, 14DoF states.
- The five acceptance checks and how to read `summary.json`.
- The measured stage 1 numbers from Step 6, labelled with the corridor provenance.
- That this is stage 1 of three, and that stage 2 (transient OCP) and stage 3 (certification) are separate plans.

- [ ] **Step 8: Run the full suite and commit**

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -q
git add tools/run/solve_minimum_time_line.py tools/gates/check_minimum_time_line.py \
        docs/dev/minimum-time-line.md tests/test_minimum_time_line_gate.py
git commit -m "feat: run, gate and document the stage 1 minimum-time line"
```

---

## Stage 2 and Stage 3 roadmap

Do not start either until Task 8's five checks pass. Each gets its own plan file.

**Stage 2 — transient minimum-time OCP.** Two decisions are already made and must be carried in:

- The transient model must be a *reduced* transient model. Wheel spin is quasi-steady with the `|T| ≤ Re·μ·Fz` traction bound; tyre relaxation and trail are algebraic on straights and states in transitions. The full 14DoF at lap scale is not affordable: its mean integrator step in the acceptance chain is 0.61 ms, which is 1 cm of travel at 15 m/s, so a lap-scale collocation mesh cannot resolve it.
- Multiple shooting on the existing plant was judged the better fit for corridor compliance, because `dense_domain` already bounds every state over every interval and can certify `n_min ≤ n(s) ≤ n_max` *between* nodes. The first task of the stage 2 plan is therefore a decision record comparing (a) CasADi collocation with a symbolic plant and (b) multiple shooting on a CasADi-wrapped plant, both measured on the same 1 m problem against stage 1's answer.

**Stage 3 — certification and replay.** Feed the racing line and speed profile to `PreviewDriver` and `solve_forward`, run the existing refinement and between-step chains, and report the driven lap against the OCP lap. This is where the corridor replaces the hard-coded 0.25 m path-tracking tolerance as the physical gate.

---

## Self-review

**Spec coverage.** D1 is the scope (Tasks 1–8 are quasi-steady only). D2 is Task 1's separate data role plus the global constraint on the geometry header. D3 is Task 2's docstring and the global constraint. D4 is Task 1's provenance validation plus Task 8's `conditional_result`. D5 is Task 4, which samples NumPy physics and re-implements nothing. D6 is Task 5. D7 is Task 8's five checks, each unit-tested in Step 1. D8 is the roadmap.

**Defects found and fixed during this review.** The first draft had a state/rate ordering mismatch (`(dn, dv, dψ)` against a `(n, ψ, v)` state); an interleaved decision vector against a grouped `unpack`; a `np.resize` seed projection that would have silently corrupted warm starts; and a corridor validator that rejected the zero-width corridor the acceptance check needs. All four are corrected above, and Task 5 Step 5 names the two that would otherwise appear as mysterious solver failures.

Two further defects were found by running the plan's own code before handing it over, and both are now fixed:

- The test fixture wrote track XML without a `<metadata><title>`, which the loader rejects, so Task 1 would have failed on its first run. Both fixtures now carry a title. Verified: a right-turning and a left-turning circle both load with exactly `2π·40 m` and correctly signed curvature.
- `offset_path` differentiated with `np.gradient`, which takes a one-sided difference across the duplicated seam node and returned **half** the true curvature there (relative error 0.5), failing Task 7's own test. It now uses `_periodic_derivative` and a wrap-safe angular difference; measured relative curvature error is 3.2e-6 at 720 nodes and 2.9e-5 at 240 nodes, comfortably inside the 1e-4 tolerance. Verified for offsets of +3 m, −2 m and 0 m, with the heading spanning exactly 0 to 2π.

**Placeholder scan.** No step says "handle edge cases", "similar to Task N", or "implement later". Task 8 Step 5 specifies the run tool as an ordered contract plus the exact JSON shape rather than a full listing, because its body is pure orchestration of Task 1–7 interfaces that are all defined above; every non-obvious operation it performs (collapsing the corridor, resampling for the containment check, writing the rebuilt track) names the exact existing function to call.

**Type consistency.** `Corridor.minimum`/`.maximum` are used with the same names in Tasks 3, 5 and 8. `CapabilitySurface.speed_ceiling` is indexed by curvature in Task 4 and interpolated by curvature in Task 5. `QssSolution.offset/heading_error/speed/acceleration/curvature_rate/lap_time/distance` are produced in Task 5, consumed by `project_seed` in Task 6 and `path_from_solution` in Task 7. `periodic_sample` is defined once in Task 5 and imported by Task 7.

**Known softness.** Two steps depend on measured behaviour that cannot be known before running: the capability-surface build time (Task 4 Step 6, with a documented fallback) and the free-line gain range (Task 8 Step 6, with failure triage). They are budgets, not asserted facts.
