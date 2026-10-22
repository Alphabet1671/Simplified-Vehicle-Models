# TV-Aware Minimum-Lap-Time Solver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `subagent-driven-development` (recommended) or `executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the 2D-width minimum-lap-time solver so the racing line and speed profile can be optimized with no controller, with an algebraic TV allocator sampled into the capability surface, or with a stateful TV controller exercised by closed-loop replay.

**Architecture:** One consistent offset-spline path formulation replaces node offsets plus an independently reconstructed Hermite–Simpson offset. Speed remains the only differential state. Controller behavior enters through a `CapabilityBackend`: ideal physics, an algebraic allocation oracle sampled at `(v, κ)`, or a closed-loop replay correction that adds per-node speed caps. Topology changes only the spline boundary operator and periodic speed constraint.

**Tech Stack:** Python 3.13, NumPy, SciPy (`CubicSpline`, `brentq`), CasADi/IPOPT, `unittest`, existing XML track/vehicle models.

**Spec:** [2026-10-21-tv-aware-minimum-lap-spec.md](../specs/2026-10-21-tv-aware-minimum-lap-spec.md). Read it before starting. This plan implements its modes, contracts, and gates.

## Global Constraints

- The lap-time objective is exactly `T = ∫ ds / (v cos ψ)`; do not replace it with a distance or energy objective.
- The 2D lateral corridor is mandatory. A centerline-only run is a diagnostic, never an accepted result.
- Existing track XML/CSV files are immutable. New tests use in-memory views or temporary fixtures.
- `ideal` mode must reproduce the current production solver before either controller mode is trusted.
- `allocation` mode is only for a deterministic, memoryless policy. A policy with state must use `closed_loop`.
- Never call a stateful controller “in the loop” by precomputing a static surface.
- Never hide controller saturation. Record it in the result and tighten caps only downward.
- Do not relax the `1e-6` solver tolerance or claim event validity from a diagnostic run.
- CasADi stays an optional import inside the functions that need it.
- Do not merge or push.

## File Structure

| Path | Responsibility |
|---|---|
| `src/simplified_vehicle_models/controls/planning.py` | **new** — controller query/response protocol and provenance |
| `src/simplified_vehicle_models/formulations/capability_backend.py` | **new** — grid, sampled capability, ideal backend |
| `src/simplified_vehicle_models/formulations/controller_capability.py` | **new** — allocation-aware surface sampling |
| `src/simplified_vehicle_models/tracks/offset_spline.py` | **new** — periodic/open offset basis and derivatives |
| `src/simplified_vehicle_models/solvers/qss_spline_ocp.py` | **new** — consistent spline + speed-state OCP |
| `src/simplified_vehicle_models/solvers/closed_loop_line.py` | **new** — replay correction and cap iteration |
| `tools/run/solve_minimum_time_line.py` | controller mode selection and provenance |
| `tests/test_control_planning.py` | **new** — policy contract/provenance tests |
| `tests/test_capability_backend.py` | **new** — ideal/allocation sampling tests |
| `tests/test_offset_spline.py` | **new** — basis/derivative/boundary tests |
| `tests/test_qss_spline_ocp.py` | **new** — spline OCP parity and geometry tests |
| `tests/test_closed_loop_line.py` | **new** — correction/cap tests |
| `docs/dev/tv-aware-minimum-lap.md` | **new** — measured results and limitations |

---

### Task 1: Controller planning contract and provenance

**Files:**
- Create: `src/simplified_vehicle_models/controls/planning.py`
- Create: `tests/test_control_planning.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `PathDemand(speed, curvature, longitudinal_acceleration, lateral_acceleration, yaw_rate, steering_angle=0.0)`
  - `ControlResponse(feasible, longitudinal_acceleration, yaw_moment, saturation=())`
  - `DriveControlPolicy` protocol with `name: str`, `provenance_hash: str`, `evaluate(demand) -> ControlResponse`
  - `controller_hash(policy) -> str`, deterministic over declared parameters.

- [ ] **Step 1: Write the failing test**

```python
import math
import unittest

from simplified_vehicle_models.controls.planning import (
    ControlResponse, PathDemand, controller_hash,
)


class _ExamplePolicy:
    name = 'example'
    provenance_hash = 'example-v1'

    def __init__(self, cap):
        self.cap = cap

    def evaluate(self, demand):
        feasible = demand.longitudinal_acceleration <= self.cap
        return ControlResponse(feasible, min(demand.longitudinal_acceleration, self.cap), 0.0)


class PlanningContractTest(unittest.TestCase):
    def test_demand_requires_finite_positive_speed_and_finite_values(self):
        with self.assertRaises(ValueError):
            PathDemand(0.0, 0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            PathDemand(10.0, float('nan'), 0.0, 0.0, 0.0)

    def test_controller_hash_is_stable_and_parameter_sensitive(self):
        a = _ExamplePolicy(2.0)
        b = _ExamplePolicy(2.0)
        c = _ExamplePolicy(1.0)
        self.assertEqual(controller_hash(a), controller_hash(b))
        self.assertNotEqual(controller_hash(a), controller_hash(c))

    def test_response_reports_saturation_without_changing_sign(self):
        response = _ExamplePolicy(2.0).evaluate(PathDemand(20.0, 0.01, 5.0, 4.0, 0.2))
        self.assertFalse(response.feasible)
        self.assertEqual(response.longitudinal_acceleration, 2.0)
        self.assertEqual(controller_hash(_ExamplePolicy(2.0)),
                         controller_hash(_ExamplePolicy(2.0)))
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_control_planning -v`

Expected: import error for `simplified_vehicle_models.controls.planning`.

- [ ] **Step 3: Implement the contract**

```python
"""Controller-facing planning contract; no vehicle equations live here."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from typing import Protocol


@dataclass(frozen=True, slots=True)
class PathDemand:
    speed: float
    curvature: float
    longitudinal_acceleration: float
    lateral_acceleration: float
    yaw_rate: float
    steering_angle: float = 0.0

    def __post_init__(self):
        values = (self.speed, self.curvature, self.longitudinal_acceleration,
                  self.lateral_acceleration, self.yaw_rate, self.steering_angle)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('path demand values must be finite')
        if self.speed <= 0.0:
            raise ValueError('path demand speed must be positive')


@dataclass(frozen=True, slots=True)
class ControlResponse:
    feasible: bool
    longitudinal_acceleration: float
    yaw_moment: float
    saturation: tuple[str, ...] = ()

    def __post_init__(self):
        if not (math.isfinite(self.longitudinal_acceleration) and math.isfinite(self.yaw_moment)):
            raise ValueError('control response values must be finite')
        if not isinstance(self.feasible, bool):
            raise ValueError('feasible must be boolean')
        if not all(isinstance(tag, str) for tag in self.saturation):
            raise ValueError('saturation tags must be strings')


class DriveControlPolicy(Protocol):
    name: str
    provenance_hash: str

    def evaluate(self, demand: PathDemand) -> ControlResponse: ...


def controller_hash(policy) -> str:
    """Hash the declared controller identity and its public parameters."""
    payload = {
        'name': policy.name,
        'provenance': policy.provenance_hash,
        'parameters': sorted(
            (name, repr(value)) for name, value in vars(policy).items()
            if not name.startswith('_') and name not in ('name', 'provenance_hash')
        ),
    }
    return hashlib.sha256(repr(payload).encode()).hexdigest()
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src python3 -m unittest tests.test_control_planning -v`

Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/controls/planning.py tests/test_control_planning.py
git commit -m "Add controller planning contract"
```

---

### Task 2: Ideal capability backend with explicit provenance

**Files:**
- Create: `src/simplified_vehicle_models/formulations/capability_backend.py`
- Create: `tests/test_capability_backend.py`

**Interfaces:**
- Consumes: `CapabilitySurface`, `build_capability_surface`, `capability_hash`.
- Produces:
  - `CapabilityGrid(speed, curvature)`
  - `SampledCapability(surface, vehicle_hash, controller_hash, mode, version)`
  - `IdealCapabilityBackend(vehicle).sample(grid, curvature_limit) -> SampledCapability`

- [ ] **Step 1: Write the failing test**

```python
import unittest
from unittest.mock import patch
import numpy as np

from simplified_vehicle_models.formulations.capability_backend import (
    CapabilityGrid, IdealCapabilityBackend,
)
from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface


class IdealBackendTest(unittest.TestCase):
    def test_backend_wraps_existing_builder_and_hash(self):
        vehicle = object()
        surface = CapabilitySurface(
            np.array([1.0, 2.0]), np.array([-1.0, 1.0]),
            np.ones((2, 2)), -np.ones((2, 2)), np.array([5.0, 5.0]), 'vehicle-hash')
        with patch('simplified_vehicle_models.formulations.capability_backend.'
                   'build_capability_surface', return_value=surface) as build:
            sampled = IdealCapabilityBackend(vehicle).sample(
                CapabilityGrid(surface.speed, surface.curvature), curvature_limit=1.0)
        build.assert_called_once()
        self.assertEqual(sampled.mode, 'ideal')
        self.assertEqual(sampled.vehicle_hash, 'vehicle-hash')
        self.assertEqual(sampled.controller_hash, '')
        self.assertIs(sampled.surface, surface)

    def test_grid_requires_increasing_finite_axes(self):
        with self.assertRaises(ValueError):
            CapabilityGrid(np.array([1.0, 1.0]), np.array([-1.0, 1.0]))
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_capability_backend -v`

Expected: import error.

- [ ] **Step 3: Implement the backend**

```python
"""Capability backends used by the line OCP; controller behavior is supplied."""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import (
    CapabilitySurface, build_capability_surface,
)


@dataclass(frozen=True, slots=True)
class CapabilityGrid:
    speed: np.ndarray
    curvature: np.ndarray

    def __post_init__(self):
        for name in ('speed', 'curvature'):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.ndim != 1 or value.size < 3 or not np.all(np.isfinite(value)):
                raise ValueError(f'{name} must be a finite one-dimensional grid')
            if np.any(np.diff(value) <= 0.0):
                raise ValueError(f'{name} must increase')
            value.setflags(write=False)
            object.__setattr__(self, name, value)


@dataclass(frozen=True, slots=True)
class SampledCapability:
    surface: CapabilitySurface
    vehicle_hash: str
    controller_hash: str
    mode: str
    version: str = '1'
    metadata: dict = field(default_factory=dict)


class IdealCapabilityBackend:
    mode = 'ideal'

    def __init__(self, vehicle):
        self.vehicle = vehicle

    def sample(self, grid: CapabilityGrid, *, curvature_limit: float) -> SampledCapability:
        surface = build_capability_surface(
            self.vehicle, curvature_limit=curvature_limit,
            speed_points=grid.speed.size, curvature_points=grid.curvature.size)
        return SampledCapability(surface, surface.provenance_hash, '', self.mode)
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src python3 -m unittest tests.test_capability_backend -v`

Expected: 2 tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/formulations/capability_backend.py tests/test_capability_backend.py
git commit -m "Add ideal capability backend"
```

---

### Task 3: Algebraic TV allocation capability backend

**Files:**
- Create: `src/simplified_vehicle_models/formulations/controller_capability.py`
- Extend: `tests/test_capability_backend.py`

**Interfaces:**
- Consumes: `DriveControlPolicy`, `PathDemand`, `SampledCapability`, `CapabilityGrid`.
- Produces:
  - `AllocationCapabilityBackend(vehicle, policy, scan_points=17, bisection_steps=12).sample(grid, curvature_limit) -> SampledCapability`
  - `feasible_interval(policy, speed, curvature) -> tuple[float, float] | None`
  - A monotonicity flag in `SampledCapability` metadata; if detected false, the backend falls back to the best feasible scan sample and records `non_monotone`.

- [ ] **Step 1: Write the failing test**

```python
def _ideal_surface(grid):
    shape = (grid.speed.size, grid.curvature.size)
    return CapabilitySurface(
        grid.speed, grid.curvature, np.full(shape, 5.0), np.full(shape, -8.0),
        np.full(grid.curvature.size, 20.0), 'vehicle-hash')


class _CapPolicy:
    name = 'cap-at-two'
    provenance_hash = 'cap-at-two-v1'

    def evaluate(self, demand):
        feasible = demand.longitudinal_acceleration <= 2.0
        return ControlResponse(feasible, min(demand.longitudinal_acceleration, 2.0), 0.0)


class AllocationBackendTest(unittest.TestCase):
    def test_known_cap_appears_in_the_sampled_surface(self):
        vehicle = object()
        backend = AllocationCapabilityBackend(vehicle, _CapPolicy())
        grid = CapabilityGrid(np.array([1.0, 5.0, 10.0]), np.array([-0.1, 0.0, 0.1]))
        with patch.object(AllocationCapabilityBackend, '_ideal',
                          return_value=_ideal_surface(grid)):
            sampled = backend.sample(grid, curvature_limit=0.1)
        self.assertEqual(sampled.mode, 'allocation')
        self.assertNotEqual(sampled.controller_hash, '')
        self.assertTrue(np.all(sampled.surface.acceleration_max <= 2.0 + 1e-9))
        self.assertTrue(np.any(np.isclose(sampled.surface.acceleration_max, 2.0)))

    def test_non_monotone_policy_is_flagged(self):
        class BumpPolicy:
            name = 'bump'
            provenance_hash = 'bump-v1'
            def evaluate(self, demand):
                ok = demand.longitudinal_acceleration <= 1.0 or demand.longitudinal_acceleration >= 3.0
                return ControlResponse(ok, demand.longitudinal_acceleration, 0.0)
        backend = AllocationCapabilityBackend(object(), BumpPolicy())
        grid = CapabilityGrid(np.array([1.0, 5.0]), np.array([-0.1, 0.1]))
        sampled = backend.sample(grid, curvature_limit=0.1,
                                 ideal_surface=_ideal_surface(grid))
        self.assertTrue(sampled.metadata['non_monotone'])
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_capability_backend -v`

Expected: import error.

- [ ] **Step 3: Implement the sampler**

```python
"""Controller-aware capability sampling; the policy is an oracle, not a plant."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from simplified_vehicle_models.controls.planning import PathDemand, controller_hash
from simplified_vehicle_models.formulations.capability_backend import (
    CapabilityGrid, SampledCapability,
)
from simplified_vehicle_models.formulations.point_mass_capability import (
    CapabilitySurface, build_capability_surface,
)


@dataclass(frozen=True, slots=True)
class AllocationCapabilityBackend:
    vehicle: object
    policy: object
    scan_points: int = 17
    bisection_steps: int = 12
    mode: str = 'allocation'

    def _ideal(self, grid, curvature_limit):
        return build_capability_surface(
            self.vehicle, curvature_limit=curvature_limit,
            speed_points=grid.speed.size, curvature_points=grid.curvature.size)

    def _demand(self, speed, curvature, ax):
        return PathDemand(speed, curvature, ax, speed * speed * curvature, speed * curvature)

    def feasible_interval(self, speed, curvature, ideal_lo, ideal_hi):
        grid = np.linspace(ideal_lo, ideal_hi, self.scan_points)
        flags = np.array([self.policy.evaluate(self._demand(speed, curvature, ax)).feasible
                          for ax in grid])
        if not flags.any():
            return None
        if flags.all():
            return float(ideal_lo), float(ideal_hi), False
        # Monotone feasibility is assumed for the boundary refinement; verify
        # the sampled set and fall back to an isolated feasible sample if not.
        indices = np.flatnonzero(flags)
        contiguous = np.all(np.diff(indices) == 1)
        if not contiguous:
            return float(grid[indices[-1]]), float(grid[indices[-1]]), True

        def refine(feasible, infeasible):
            lo, hi = float(infeasible), float(feasible)
            for _ in range(self.bisection_steps):
                mid = 0.5 * (lo + hi)
                if self.policy.evaluate(self._demand(speed, curvature, mid)).feasible:
                    hi = mid
                else:
                    lo = mid
            return hi

        lower = float(grid[indices[0]])
        upper = float(grid[indices[-1]])
        if indices[0] > 0:
            lower = refine(lower, grid[indices[0] - 1])
        if indices[-1] + 1 < grid.size:
            upper = refine(upper, grid[indices[-1] + 1])
        return lower, upper, False

    def sample(self, grid, *, curvature_limit, ideal_surface=None, **_):
        if ideal_surface is None:
            ideal_surface = self._ideal(grid, curvature_limit)
        amax = np.empty_like(ideal_surface.acceleration_max)
        amin = np.empty_like(ideal_surface.acceleration_min)
        ceiling = np.empty_like(ideal_surface.speed_ceiling)
        flags = {'non_monotone': False}
        for j, curvature in enumerate(grid.curvature):
            for i, speed in enumerate(grid.speed):
                interval = self.feasible_interval(
                    float(speed), float(curvature),
                    float(ideal_surface.acceleration_min[i, j]),
                    float(ideal_surface.acceleration_max[i, j]))
                if interval is None:
                    amin[i, j] = 0.0; amax[i, j] = 0.0
                else:
                    amin[i, j], amax[i, j] = interval[0], interval[1]
                    flags['non_monotone'] = flags['non_monotone'] or interval[2]
            ceiling[j] = 0.0
            for i in range(grid.speed.size - 1, -1, -1):
                if amax[i, j] > amin[i, j]:
                    ceiling[j] = grid.speed[i]
                    break
            ceiling[j] = min(float(ideal_surface.speed_ceiling[j]), float(ceiling[j]))
        surface = CapabilitySurface(grid.speed, grid.curvature, amax, amin, ceiling,
                                    ideal_surface.provenance_hash)
        return SampledCapability(surface, ideal_surface.provenance_hash,
                                 controller_hash(self.policy), self.mode,
                                 metadata=flags)
```

Add `metadata` to `SampledCapability`; update Task 2 dataclass to `metadata: dict = field(default_factory=dict)`.

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src python3 -m unittest tests.test_capability_backend -v`

Expected: 4 tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/formulations/capability_backend.py \
  src/simplified_vehicle_models/formulations/controller_capability.py \
  tests/test_capability_backend.py
git commit -m "Add allocation-aware capability backend"
```

---

### Task 4: Periodic and open offset spline basis

**Files:**
- Create: `src/simplified_vehicle_models/tracks/offset_spline.py`
- Create: `tests/test_offset_spline.py`

**Interfaces:**
- Consumes: a `TrackProfile` with `distance`, `lap_length`, `is_closed`.
- Produces:
  - `OffsetSpline(track, control_spacing)` with `control_distance`, `control_count`, `is_closed`.
  - `.basis(distance) -> (values, first, second)` matrices of shape `(len(distance), control_count)`.
  - Closed: periodic cubic spline through control values; open: natural cubic spline with free endpoint values.
  - `.evaluate(coefficients, distance) -> (n, n', n'')`.

- [ ] **Step 1: Write the failing test**

```python
import unittest
import numpy as np

from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track
import tempfile
from pathlib import Path


class OffsetSplineTest(unittest.TestCase):
    def test_basis_partitions_unity_and_derivatives_match_finite_differences(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            spline = OffsetSpline(track, control_spacing=10.0)
            positions = np.linspace(0.0, track.lap_length, 37)
            values, first, second = spline.basis(positions)
            np.testing.assert_allclose(values.sum(axis=1), 1.0, atol=1e-12)
            step = 1e-5
            plus = spline.evaluate(np.arange(spline.control_count, dtype=float), positions + step)
            minus = spline.evaluate(np.arange(spline.control_count, dtype=float), positions - step)
            np.testing.assert_allclose(first, (plus[0] - minus[0]) / (2 * step), atol=1e-7)
            np.testing.assert_allclose(second, (plus[1] - minus[1]) / (2 * step), atol=1e-5)

    def test_periodic_spline_has_equal_seam_values_and_derivatives(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            spline = OffsetSpline(track, control_spacing=10.0)
            coefficients = np.sin(np.linspace(0.0, 2 * np.pi, spline.control_count))
            start = spline.evaluate(coefficients, np.array([0.0]))
            end = spline.evaluate(coefficients, np.array([track.lap_length]))
            np.testing.assert_allclose(start[0], end[0], atol=1e-10)
            np.testing.assert_allclose(start[1], end[1], atol=1e-10)

    def test_open_spline_retains_free_endpoint_values(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            object.__setattr__(track, 'is_closed', False)
            spline = OffsetSpline(track, control_spacing=10.0)
            n, _, _ = spline.evaluate(np.linspace(-1.0, 1.0, spline.control_count),
                                      np.array([0.0, track.lap_length]))
            self.assertNotAlmostEqual(float(n[0]), float(n[1]))
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_offset_spline -v`

Expected: import error.

- [ ] **Step 3: Implement the spline basis**

```python
"""Cubic offset basis with periodic and open boundary operators."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from scipy.interpolate import CubicSpline


@dataclass(frozen=True, slots=True)
class OffsetSpline:
    track: object
    control_spacing: float

    def __post_init__(self):
        if not np.isfinite(self.control_spacing) or self.control_spacing <= 0.0:
            raise ValueError('control_spacing must be finite and positive')
        count = max(4, int(round(self.track.lap_length / self.control_spacing)))
        if self.track.is_closed:
            count = max(4, count)
        positions = np.linspace(0.0, self.track.lap_length, count, endpoint=False)
        if not self.track.is_closed:
            positions = np.linspace(0.0, self.track.lap_length, count, endpoint=True)
        object.__setattr__(self, 'control_distance', positions)
        object.__setattr__(self, 'control_count', positions.size)
        object.__setattr__(self, 'is_closed', bool(self.track.is_closed))

    def _spline(self, coefficients):
        coefficients = np.asarray(coefficients, dtype=float)
        if coefficients.shape != (self.control_count,):
            raise ValueError('coefficients must match control_count')
        if self.is_closed:
            # Duplicate the first control value at the period end so CubicSpline's
            # periodic condition operates on a genuinely periodic table.
            positions = np.r_[self.control_distance, self.track.lap_length]
            values = np.r_[coefficients, coefficients[0]]
            return CubicSpline(positions, values, bc_type='periodic')
        return CubicSpline(self.control_distance, coefficients, bc_type='natural')

    def basis(self, distance):
        positions = np.asarray(distance, dtype=float)
        values = np.eye(self.control_count)
        columns = [self._spline(values[:, j])(positions) for j in range(self.control_count)]
        first = [self._spline(values[:, j])(positions, 1) for j in range(self.control_count)]
        second = [self._spline(values[:, j])(positions, 2) for j in range(self.control_count)]
        return np.column_stack(columns), np.column_stack(first), np.column_stack(second)

    def evaluate(self, coefficients, distance):
        spline = self._spline(coefficients)
        positions = np.asarray(distance, dtype=float)
        return spline(positions), spline(positions, 1), spline(positions, 2)
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src python3 -m unittest tests.test_offset_spline -v`

Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/tracks/offset_spline.py tests/test_offset_spline.py
git commit -m "Add periodic and open offset spline"
```

---

### Task 5: Consistent spline OCP with speed as the only state

**Files:**
- Create: `src/simplified_vehicle_models/solvers/qss_spline_ocp.py`
- Create: `tests/test_qss_spline_ocp.py`

**Interfaces:**
- Consumes: `OffsetSpline`, `CapabilitySurface`, `Corridor`, `QssSettings`, `build_mesh`.
- Produces:
  - `SplineSolution(distance, offset, heading_error, curvature, speed, acceleration, lap_time, status, iterations, maximum_violation, coefficients)`
  - `build_spline_ocp(track, corridor, surface, mesh, spline, settings, speed_cap=None)`
  - `solve_spline_line(track, corridor, surface, mesh, spline, settings, *, seed=None, speed_cap=None)` with the same status/tolerance contract as `solve_line`.
  - Decision layout `[coefficients | speed nodes | acceleration nodes | midpoint acceleration]`.

- [ ] **Step 1: Write the failing test**

```python
import unittest
import numpy as np
from tests.test_track_width import _write_circle_track
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.corridor import Corridor, corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
import tempfile
from pathlib import Path


def _surface():
    speed = np.linspace(0.5, 40.0, 81)
    curvature = np.linspace(-0.3, 0.3, 61)
    return CapabilitySurface(speed, curvature, np.full((81, 61), 3.0),
                             np.full((81, 61), -8.0), np.full(61, 25.0), 'synthetic')


class SplineOcpTest(unittest.TestCase):
    def test_zero_corridor_open_circle_matches_capability_envelope(self):
        from dataclasses import replace
        from simplified_vehicle_models.solvers.qss_line_ocp import capability_envelope_time
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            view = replace(track, is_closed=False)
            base = corridor_from_track(view)
            corridor = Corridor(base.distance, np.zeros_like(base.minimum),
                                np.zeros_like(base.maximum), base.provenance, False)
            mesh = build_mesh(view, 2.0)
            spline = OffsetSpline(view, control_spacing=10.0)
            settings = QssSettings(maximum_iterations=2000, start_speed=0.5)
            solution = solve_spline_line(view, corridor, _surface(), mesh, spline, settings)
            self.assertEqual(solution.status, 'Solve_Succeeded')
            expected = capability_envelope_time(view, _surface(), settings)
            self.assertAlmostEqual(solution.lap_time, expected, delta=2e-3)

    def test_open_and_closed_use_the_same_state_layout(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            for closed in (True, False):
                view = track if closed else __import__('dataclasses').replace(
                    track, is_closed=False)
                corridor = corridor_from_track(view, half_width=3.0)
                spline = OffsetSpline(view, control_spacing=10.0)
                solution = solve_spline_line(view, corridor, _surface(), build_mesh(view, 3.0),
                                             spline, QssSettings(maximum_iterations=2000))
                self.assertEqual(solution.status, 'Solve_Succeeded')
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_qss_spline_ocp -v`

Expected: import error.

- [ ] **Step 3: Implement the spline OCP**

Use the existing `qss_line_ocp` scaling, tolerance ladder, `_validate_solver_result`, and capability interpolants. Define these exact helpers in `qss_spline_ocp.py`:

```python
def _curvature_pair(track, positions):
    k0 = track_sample(track.curvature, track.distance, positions,
                      closed=bool(track.is_closed), period=track.lap_length)
    dk = np.gradient(track.curvature, track.distance)
    dk0 = track_sample(dk, track.distance, positions,
                       closed=bool(track.is_closed), period=track.lap_length)
    return k0, dk0


def _geometry(track, spline, coefficients, positions):
    n, np_, npp = spline.evaluate(coefficients, positions)
    k0, dk0 = _curvature_pair(track, positions)
    den = 1.0 - k0 * n
    u = np_ / den
    psi = np.arctan(u)
    den_p = -dk0 * n - k0 * np_
    q = ((npp * den - np_ * den_p) / (den * den)) / (1.0 + u * u)
    kappa = (k0 + q) * np.cos(psi) / den
    return n, psi, q, kappa, den
```

- Objective: `h/6 * (1/(v0 cosψ0) + 4/(vm cosψm) + 1/(v1 cosψ1))`.
- Defect: `v1 - v0 - h/6*(f0 + 4 fm + f1)` with
  `f = a*den/(v*cosψ)`.
- Constraints at each node and midpoint: capability, ceiling, fold, curvature;
  corridor at nodes, midpoint, and quarter points using the spline.
- Closed: `v[0] == v[-1]` and periodic spline. Open: `v[0] == max(start_speed, minimum_speed)`, free terminal speed.
- Initial guess: coefficients from `np.linalg.lstsq(spline.basis(mesh)[0], zeros)`; speeds from the existing open/closed envelope; accelerations from a finite difference of the envelope.
- Scaling: `[coefficients | speed | acceleration | midpoint acceleration]` in that order. Use coefficients near zero and physical speed scale 10 m/s, acceleration scale 5 m/s².
- Add `speed_cap=None`: if provided, add `v[i] ≤ cap[i]` and `vm ≤ interpolated cap` constraints.

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_qss_spline_ocp -v`

Expected: 2 tests pass; zero-corridor circle time within `2e-3 s`.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/solvers/qss_spline_ocp.py tests/test_qss_spline_ocp.py
git commit -m "Add consistent spline minimum-time OCP"
```

---

### Task 6: Closed-loop controller correction

**Files:**
- Create: `src/simplified_vehicle_models/solvers/closed_loop_line.py`
- Create: `tests/test_closed_loop_line.py`

**Interfaces:**
- Consumes: `solve_spline_line`, `path_from_solution`.
- Produces:
  - `ReplaySample(distance, speed, feasible, lap_time, saturation=())`
  - `ReplayProtocol(path, controller) -> ReplaySample`
  - `correct_to_replay(track, corridor, surface, mesh, spline, settings, controller, replay, *, tolerance=0.01, maximum_rounds=5)`.
  - Result records `rounds`, `replay_time`, `speed_caps`, `cap_margin`, and `accepted`.

- [ ] **Step 1: Write the failing test**

```python
import unittest
import numpy as np
from dataclasses import replace
from tests.test_track_width import _write_circle_track
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
from simplified_vehicle_models.solvers.closed_loop_line import (
    ReplaySample, correct_to_replay,
)
from tests.test_qss_spline_ocp import _surface
from pathlib import Path
import tempfile


class ClosedLoopCorrectionTest(unittest.TestCase):
    def test_caps_are_lowered_when_replay_is_infeasible(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            corridor = corridor_from_track(track)
            spline = OffsetSpline(track, control_spacing=10.0)
            mesh = build_mesh(track, 3.0)
            calls = []

            def replay(path, controller):
                calls.append(path.distance.copy())
                return ReplaySample(path.distance, np.full(path.distance.size, 8.0), False,
                                    float(path.lap_length / 8.0), ('speed',))

            result = correct_to_replay(track, corridor, _surface(), mesh, spline,
                                       QssSettings(maximum_iterations=2000), None, replay,
                                       maximum_rounds=2)
            self.assertEqual(result.rounds, 2)
            self.assertFalse(result.accepted)
            self.assertGreaterEqual(len(calls), 2)
            self.assertLess(result.speed_caps[-1], result.speed_caps[0])
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_closed_loop_line -v`

Expected: import error.

- [ ] **Step 3: Implement the correction loop**

```python
@dataclass(frozen=True, slots=True)
class ReplaySample:
    distance: np.ndarray
    speed: np.ndarray
    feasible: bool
    lap_time: float
    saturation: tuple[str, ...] = ()


def _cap_from_replay(mesh, replay, ocp_speed, margin):
    replay_speed = np.interp(mesh, replay.distance, replay.speed)
    return np.minimum(ocp_speed, replay_speed) - margin
```

Algorithm:

1. Solve the spline OCP with `speed_cap=None`.
2. Build the path with `path_from_solution`.
3. Call `replay(path, controller)`.
4. If feasible and `abs(replay_time - ocp_time)/ocp_time <= tolerance`, accept.
5. Otherwise set `cap = min(ocp_speed, interp(replay_speed)) - margin` with
   `margin = 0.1 m/s`, never above the previous cap; re-solve with `speed_cap=cap`.
6. Stop at `maximum_rounds`; accepted only if the final replay is feasible and
   within tolerance. Record every round.

Use `solve_spline_line(..., speed_cap=cap)` and use `replay.lap_time` directly;
the replay adapter owns its integration method and open/closed start treatment.
The correction loop must not infer a replay time from a possibly infeasible
speed trace.

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_closed_loop_line -v`

Expected: 1 test passes.

- [ ] **Step 5: Commit**

```bash
git add src/simplified_vehicle_models/solvers/closed_loop_line.py tests/test_closed_loop_line.py
git commit -m "Add closed-loop line correction"
```

---

### Task 7: Runner modes, provenance, and comparison report

**Files:**
- Modify: `tools/run/solve_minimum_time_line.py`
- Create: `tests/test_tv_runner.py`

**Interfaces:**
- Consumes: all previous tasks.
- Produces:
  - CLI `--control-mode ideal|allocation|closed_loop` and `--control-policy NAME`.
  - `--offset-spline` mode (or make spline the default after Task 5 parity).
  - Summary fields `control_mode`, `controller_hash`, `capability_mode`, `replay_time_s`, `rounds`, `accepted`.
  - A/B summary writer that runs the same track/corridor/mesh under all three modes and reports line offset RMS, lap time, replay time, and saturation.

- [ ] **Step 1: Write the failing test**

```python
import unittest
from unittest.mock import patch
from pathlib import Path
from tools.run import solve_minimum_time_line as runner


class RunnerModeTest(unittest.TestCase):
    def test_control_mode_is_required_to_be_known(self):
        with patch('sys.argv', ['runner', '--track', 'x', '--formulation', 'y',
                                '--surface', 'z', '--output', 'out',
                                '--control-mode', 'not-a-mode']):
            with self.assertRaises(SystemExit):
                runner.main()

    def test_ideal_and_allocation_are_distinct_summary_modes(self):
        # Build a tiny fake summary through the internal helper rather than
        # running a full track.
        from tools.run.solve_minimum_time_line import _control_summary
        ideal = _control_summary('ideal', '', 'ideal')
        allocation = _control_summary('allocation', 'hash', 'allocation')
        self.assertEqual(ideal['control_mode'], 'ideal')
        self.assertNotEqual(ideal['control_mode'], allocation['control_mode'])
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=src python3 -m unittest tests.test_tv_runner -v`

Expected: CLI rejects the mode but `_control_summary` does not exist.

- [ ] **Step 3: Implement the runner integration**

Add:

```python
def _control_summary(mode, controller_hash, capability_mode):
    return {
        'control_mode': mode,
        'controller_hash': controller_hash,
        'capability_mode': capability_mode,
    }
```

- Parse and validate `--control-mode` and `--control-policy` before creating output.
- `ideal`: use `IdealCapabilityBackend`; preserve current surface cache behavior.
- `allocation`: load the policy from the registry, sample with `AllocationCapabilityBackend`, write a cache whose filename includes the controller hash, and use that surface.
- `closed_loop`: build the ideal/allocation surface, solve, then call `correct_to_replay` with the configured replay adapter. If no replay adapter is configured, exit with a clear configuration error; never silently accept an OCP-only result.
- Summary must include the controller hash and mode; a mismatch between requested and recorded mode is an error.
- Add `--compare-control-modes` that runs ideal/allocation/closed_loop on the same track and writes a comparison JSON. It must not overwrite an existing output.

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH=src python3 -m unittest tests.test_tv_runner -v`

Expected: 2 tests pass.

- [ ] **Step 5: Commit**

```bash
git add tools/run/solve_minimum_time_line.py tests/test_tv_runner.py
git commit -m "Add controller modes to the lap-time runner"
```

---

### Task 8: Acceptance matrix and documentation

**Files:**
- Create: `docs/dev/tv-aware-minimum-lap.md`
- Extend: `tests/test_qss_spline_ocp.py`, `tests/test_closed_loop_line.py`
- Modify: `docs/dev/minimum-time-line.md`

**Interfaces:**
- Consumes: all previous tasks.
- Produces: a measured comparison and a decision record, not a new acceptance standard.

- [ ] **Step 1: Add regression gates**

Add tests that assert:

```python
def test_allocation_never_exceeds_ideal(self):
    grid = CapabilityGrid(np.linspace(1.0, 30.0, 5), np.linspace(-0.2, 0.2, 5))
    ideal = _ideal_surface(grid)
    sampled = AllocationCapabilityBackend(object(), _CapPolicy()).sample(
        grid, curvature_limit=0.2, ideal_surface=ideal)
    self.assertTrue(np.all(sampled.surface.acceleration_max <= ideal.acceleration_max + 1e-9))
    self.assertTrue(np.all(sampled.surface.acceleration_min >= ideal.acceleration_min - 1e-9))


def test_closed_loop_rejects_a_replay_that_never_becomes_feasible(self):
    with tempfile.TemporaryDirectory() as directory:
        track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                               half_width=3.0))
        corridor = corridor_from_track(track)
        spline = OffsetSpline(track, control_spacing=10.0)
        mesh = build_mesh(track, 3.0)

        def always_infeasible(path, controller):
            return ReplaySample(path.distance, np.full(path.distance.size, 8.0), False,
                                float(path.lap_length / 8.0), ('speed',))

        result = correct_to_replay(track, corridor, _surface(), mesh, spline,
                                   QssSettings(maximum_iterations=2000), None,
                                   always_infeasible, maximum_rounds=3)
        self.assertFalse(result.accepted)
        self.assertEqual(result.rounds, 3)


def test_spline_solution_respects_dense_corridor(self):
    positions = np.linspace(0.0, track.lap_length, 25 * (mesh.size - 1) + 1)
    offset = spline.evaluate(solution.coefficients, positions)[0]
    lower = track_sample(corridor.minimum, corridor.distance, positions,
                         closed=track.is_closed, period=track.lap_length)
    upper = track_sample(corridor.maximum, corridor.distance, positions,
                         closed=track.is_closed, period=track.lap_length)
    self.assertTrue(np.all(offset >= lower - 1e-6))
    self.assertTrue(np.all(offset <= upper + 1e-6))
```

- [ ] **Step 2: Run the full focused suite**

Run: `PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_control_planning tests.test_capability_backend tests.test_offset_spline tests.test_qss_spline_ocp tests.test_closed_loop_line tests.test_tv_runner tests.test_track_width tests.test_corridor tests.test_minimum_time_line_gate -v`

Expected: all tests pass.

- [ ] **Step 3: Run the real-track matrix**

For each of MIS, skidpad, acceleration, and FSC, run `ideal`, `allocation`, and
`closed_loop` with the same corridor and mesh. Use the existing capability cache
only for `ideal`; allocation cache must include the controller hash. Save outputs
to new paths (the runner refuses overwrite).

Required report fields per mode: OCP time, replay time, maximum violation, mesh
convergence, line RMS against ideal, and saturation events. A mode that fails
convergence is reported as failed; do not substitute another mode’s result.

- [ ] **Step 4: Write the decision record**

`docs/dev/tv-aware-minimum-lap.md` must state:

- which modes were actually run;
- the controller name/hash and whether it was algebraic or stateful;
- per-track differences between ideal and controller-aware lines;
- whether controller-aware replay improved, matched, or worsened lap time;
- all assumed widths/surfaces and all unresolved convergence failures;
- a clear statement that a lap-time gain is conditional on the controller and
  replay, not a universal TV benefit.

- [ ] **Step 5: Commit**

```bash
git add docs/dev/tv-aware-minimum-lap.md docs/dev/minimum-time-line.md \
  tests/test_qss_spline_ocp.py tests/test_closed_loop_line.py
git commit -m "Document TV-aware solver acceptance"
```

---

## Self-review checklist

- [ ] Every task ends in a passing test or an explicit measured artifact.
- [ ] `ideal` parity is established before any controller mode is trusted.
- [ ] The spline path is the only offset representation used by the OCP; no
      node-offset/HS mismatch remains in the new path.
- [ ] Algebraic TV is sampled, stateful TV is replayed; the two are never
      conflated.
- [ ] Every result records controller mode/hash and replay status.
- [ ] The 2D corridor is enforced at nodes, midpoints, and dense samples.
- [ ] No track source data, tolerance, or acceptance standard is changed.
