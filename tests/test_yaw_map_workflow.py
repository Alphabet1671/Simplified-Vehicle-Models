import unittest
from dataclasses import dataclass, replace

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.formulations.yaw_map_domain import (
    ProbeResult, ProbeStatus, YawMapDomainSettings,
)
from simplified_vehicle_models.formulations.yaw_map_workflow import (
    YawMapWorkflowSettings, build_yaw_map_workflow,
)
from simplified_vehicle_models.parameters.tire import UniTirePeakEnvelope


@dataclass
class FakeVehicle:
    total_mass: float
    tire: object


def vehicle():
    return FakeVehicle(40., UniTirePeakEnvelope(
        vertical_loads=np.array([0., 500., 1000.]),
        longitudinal_peak_force=np.array([0., 100., 100.]),
        lateral_peak_force=np.array([0., 120., 120.]),
        effective_radius=.2, reference_speed=3.,
    ))


def evaluator(limit, *, unresolved=False):
    def evaluate(speed, ax, ay):
        if unresolved and speed == 5. and ax == 0. and ay == 0.:
            return ProbeResult(ProbeStatus.UNRESOLVED, "synthetic numerical failure")
        return ProbeResult(ProbeStatus.FEASIBLE if abs(ay) <= limit(speed, ax) else ProbeStatus.INFEASIBLE)
    return evaluate


def synthetic_builder(limit):
    def build(_vehicle, grid, _tv_limits, *, yaw_inertia):
        shape = (len(grid.speed), len(grid.longitudinal_acceleration), len(grid.lateral_acceleration))
        mask = np.zeros(shape, dtype=bool)
        into = np.zeros(shape)
        out = np.zeros(shape)
        for i, speed in enumerate(grid.speed):
            for j, ax in enumerate(grid.longitudinal_acceleration):
                for k, ay in enumerate(grid.lateral_acceleration):
                    if abs(ay) <= limit(float(speed), float(ax)):
                        mask[i, j, k] = True
                        into[i, j, k] = 10. + ay
                        out[i, j, k] = 20. - ay
        return YawCapabilityMap(grid.speed, grid.longitudinal_acceleration, grid.lateral_acceleration,
                                into, out, mask, yaw_inertia, "physical_no_tv_v1", "synthetic")
    return build


class YawMapWorkflowTest(unittest.TestCase):
    def settings(self, **kwargs):
        discovery = YawMapDomainSettings(scan_spacing=.5, bracket_tolerance=.125,
                                         probe_budget=20_000,
                                         speed_ax_audit_subdivisions=0,
                                         lateral_interpolation_loss_tolerance=.6)
        values = dict(discovery=discovery, maximum_lateral_cell_width=.5,
                      maximum_map_nodes=10_000, maximum_audit_probes=10_000,
                      maximum_refinement_rounds=3)
        values.update(kwargs)
        return YawMapWorkflowSettings(**values)

    def test_builds_signed_nonuniform_axis_with_zero_and_maximum_cell_width(self):
        limit = lambda speed, ax: 2. + .1 * speed - .2 * ax
        result = build_yaw_map_workflow(vehicle(), [0., 10.], [-1., 0., 2.], evaluator(limit),
                                        yaw_inertia=1., settings=self.settings(),
                                        map_builder=synthetic_builder(limit))
        axis = result.capability.lateral_acceleration
        self.assertTrue(np.any(axis == 0.))
        self.assertLessEqual(float(np.max(np.diff(axis))), .5 + 1e-12)
        self.assertLess(axis[0], -2.)
        self.assertGreater(axis[-1], 2.)
        self.assertFalse(result.complete)
        self.assertTrue(result.audit.complete)
        self.assertFalse(result.validation_audit.complete)
        self.assertIn("not continuous", result.finite_resolution_disclosure)
        self.assertEqual(result.capability.yaw_moment_into_turn.shape,
                         result.capability.feasible_mask.shape)

    def test_singleton_axis_cannot_qualify_without_independent_holdout(self):
        result = build_yaw_map_workflow(vehicle(), [5.], [0.], evaluator(lambda _v, _a: 2.),
                                        yaw_inertia=1., settings=self.settings(),
                                        map_builder=synthetic_builder(lambda _v, _a: 2.))
        self.assertFalse(result.complete)
        self.assertIn("singleton", result.finite_resolution_disclosure)

    def test_unresolved_map_builder_returns_inspectable_incomplete_result(self):
        from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityNumericalError
        def failing_builder(_vehicle, _grid, _limits, *, yaw_inertia):
            raise YawCapabilityNumericalError("synthetic map-cell numerical failure")
        result = build_yaw_map_workflow(vehicle(), [0., 10.], [0., 1.], evaluator(lambda _v, _a: 2.),
                                        yaw_inertia=1., settings=self.settings(), map_builder=failing_builder)
        self.assertFalse(result.complete)
        self.assertEqual(result.capability.feasible_mask.shape,
                         (2, 2, len(result.capability.lateral_acceleration)))
        self.assertFalse(np.any(result.capability.feasible_mask))
        self.assertEqual(result.audit.unresolved[0].status, ProbeStatus.UNRESOLVED)
        self.assertIn("map-cell", result.audit.unresolved[0].detail)

    def test_outer_truncation_is_explicitly_incomplete_and_refines(self):
        # The map grid builder lies about its outer feasible faces while direct
        # holdouts remain feasible.  The audit must reject it rather than claim
        # a qualified map.
        limit = lambda _speed, _ax: 10.
        result = build_yaw_map_workflow(vehicle(), [5.], [0.], evaluator(limit), yaw_inertia=1.,
                                        settings=self.settings(maximum_refinement_rounds=1),
                                        map_builder=synthetic_builder(lambda _v, _a: 2.))
        self.assertFalse(result.complete)
        self.assertGreater(len(result.audit.interpolation_losses), 0)
        self.assertGreaterEqual(result.refinement_rounds, 0)

    def test_refined_holdout_boundary_detects_exterior_gap_missed_by_cell_thirds(self):
        # Direct support ends at 2.24.  With a 0.5 grid, old third-cell points
        # could stop at 2.166 and miss the outer gap; the refined feasible
        # boundary end must be tested and reject this deliberately truncated map.
        limit = lambda _speed, _ax: 2.24
        settings = self.settings(maximum_refinement_rounds=1)
        settings = replace(settings, discovery=replace(settings.discovery, lateral_interpolation_loss_tolerance=.1))
        result = build_yaw_map_workflow(vehicle(), [5.], [0.], evaluator(limit), yaw_inertia=1.,
                                        settings=settings, map_builder=synthetic_builder(lambda _v, _a: 2.0))
        self.assertFalse(result.complete)
        self.assertTrue(any(abs(probe.lateral_acceleration) > 2.1
                            for probe in result.audit.interpolation_losses))

    def test_strict_mask_interpolation_loss_is_recorded_not_hull_filled(self):
        # A boundary falls inside cells; directly feasible holdouts can lack all
        # feasible interpolation corners.  The map mask stays strict and audit
        # evidence makes the tolerance decision visible.
        limit = lambda _speed, _ax: 1.15
        # The synthetic physical map deliberately supports only nodes at |ay| <=
        # 0.8 while direct LP support reaches 1.15, creating strict-corner loss.
        strict = self.settings(maximum_refinement_rounds=1)
        strict = replace(strict, discovery=replace(strict.discovery, lateral_interpolation_loss_tolerance=.01))
        result = build_yaw_map_workflow(vehicle(), [5.], [0.], evaluator(limit), yaw_inertia=1., 
                                        settings=strict, map_builder=synthetic_builder(lambda _v, _a: .8))
        self.assertFalse(result.complete)
        self.assertGreater(len(result.audit.interpolation_losses), 0)
        self.assertFalse(np.all(result.capability.feasible_mask))

    def test_fully_masked_speed_ax_holdout_has_json_safe_null_loss(self):
        # No strict support at the holdout v/ax must not pass just because ay
        # spacing is fine; use a null distance plus explicit status, not Infinity.
        limit = lambda speed, _ax: 1. if speed < 5. else 2.
        result = build_yaw_map_workflow(vehicle(), [0., 10.], [0.], evaluator(limit), yaw_inertia=1.,
                                        settings=self.settings(maximum_refinement_rounds=1),
                                        map_builder=synthetic_builder(lambda speed, _ax: 1. if speed in (0., 10.) else -1.))
        self.assertFalse(result.complete)
        self.assertIsNone(result.audit.maximum_interpolation_loss)
        self.assertTrue(any(probe.full_slice_unsupported and probe.loss_distance is None
                            for probe in result.audit.interpolation_losses))

    def test_infeasible_holdouts_do_not_trigger_lateral_support_scans(self):
        calls = []
        def probe(speed, ax, ay):
            calls.append((speed, ax, ay))
            return ProbeResult(ProbeStatus.INFEASIBLE)
        result = build_yaw_map_workflow(vehicle(), [5.], [0.], probe, yaw_inertia=1.,
                                        settings=self.settings(maximum_refinement_rounds=1),
                                        map_builder=synthetic_builder(lambda _v, _a: -1.))
        # Infeasible holdouts bypass map evidence entirely; they cannot be map
        # losses.  Audit still directly tests each finite holdout coordinate.
        self.assertFalse(result.audit.interpolation_losses)
        self.assertTrue(all(probe.map_reason is None for probe in result.audit.probes))
        self.assertGreater(result.audit.direct_probe_count, 0)
        self.assertEqual(len(calls), (result.domain.diagnostics.probe_count
                         + result.audit.direct_probe_count + result.validation_audit.direct_probe_count))

    def test_node_budget_never_allocates_oversized_grid(self):
        seen = []
        def builder(_vehicle, grid, _limits, *, yaw_inertia):
            seen.append(len(grid.speed) * len(grid.longitudinal_acceleration) * len(grid.lateral_acceleration))
            return synthetic_builder(lambda _v, _a: 2.)(_vehicle, grid, _limits, yaw_inertia=yaw_inertia)
        result = build_yaw_map_workflow(vehicle(), [0., 10.], [0., 1.], evaluator(lambda _v, _a: 2.),
                                        yaw_inertia=1., settings=self.settings(maximum_map_nodes=1), map_builder=builder)
        self.assertFalse(result.complete)
        self.assertEqual(seen, [1])

    def test_outer_refinement_extends_ay_only_to_model_bound(self):
        from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityGrid
        from simplified_vehicle_models.formulations.yaw_map_workflow import AuditProbe, YawMapCompletionAudit, _refine_axes
        from simplified_vehicle_models.solvers.yaw_profile_map import MapQueryReason
        capability = synthetic_builder(lambda _v, _a: 1.)(vehicle(),
            YawCapabilityGrid([0., 10.], [0., 1.], [-1., 0., 1.]), None, yaw_inertia=1.)
        probe = AuditProbe(5., .5, 2.5, ProbeStatus.FEASIBLE, MapQueryReason.OUT_OF_DOMAIN, .5)
        audit = YawMapCompletionAudit((), 0, 10, False, False, (probe,), (), (), (), .5, .1)
        _, _, ay = _refine_axes(capability, audit, self.settings(), lateral_bound=2.)
        self.assertEqual(ay[-1], 2.)
        self.assertLessEqual(max(np.diff(ay)), .5 + 1e-12)

    def test_refinement_selects_one_worst_face_not_all_holdout_axes(self):
        from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityGrid
        from simplified_vehicle_models.formulations.yaw_map_workflow import AuditProbe, YawMapCompletionAudit, _refine_axes
        from simplified_vehicle_models.solvers.yaw_profile_map import MapQueryReason
        capability = synthetic_builder(lambda _v, _a: 1.)(vehicle(),
            YawCapabilityGrid([0., 10.], [0., 1.], [-1., 0., 1.]), None, yaw_inertia=1.)
        losses = tuple(AuditProbe(speed, ax, .8, ProbeStatus.FEASIBLE,
                                  MapQueryReason.MASKED_SUPPORT, .2)
                       for speed in (10./3., 20./3.) for ax in (1./3., 2./3.))
        audit = YawMapCompletionAudit((), 0, 10, False, False, (), losses, (), (), .2, .1)
        speed, ax, ay = _refine_axes(capability, audit, self.settings())
        self.assertEqual(len(speed), 3)
        self.assertEqual(len(ax), 3)
        self.assertIn(5., speed)
        self.assertIn(.5, ax)
        self.assertGreater(len(ay), 3)

    def test_refinement_node_budget_retains_last_usable_map(self):
        seen = []
        def builder(_vehicle, grid, _limits, *, yaw_inertia):
            seen.append((len(grid.speed), len(grid.longitudinal_acceleration), len(grid.lateral_acceleration)))
            return synthetic_builder(lambda _v, _a: 1.)(_vehicle, grid, _limits, yaw_inertia=yaw_inertia)
        result = build_yaw_map_workflow(vehicle(), [5.], [0.], evaluator(lambda _v, _a: 2.), yaw_inertia=1.,
                                        settings=self.settings(maximum_map_nodes=10, maximum_refinement_rounds=2),
                                        map_builder=builder)
        self.assertFalse(result.complete)
        self.assertGreaterEqual(len(seen), 1)
        self.assertEqual(result.capability.feasible_mask.shape, seen[0])
        self.assertGreater(len(result.audit.interpolation_losses), 0)
        self.assertTrue(result.validation_audit.budget_exhausted or not result.complete)

    def test_unresolved_direct_lp_and_audit_budget_are_never_physical_infeasible(self):
        limit = lambda _speed, _ax: 2.
        unresolved = build_yaw_map_workflow(vehicle(), [5.], [0.], evaluator(limit, unresolved=True),
                                            yaw_inertia=1., settings=self.settings(),
                                            map_builder=synthetic_builder(limit))
        self.assertFalse(unresolved.complete)
        self.assertFalse(unresolved.domain.diagnostics.complete)
        constrained = self.settings(maximum_audit_probes=1)
        budgeted = build_yaw_map_workflow(vehicle(), [5.], [0.], evaluator(limit), yaw_inertia=1.,
                                          settings=constrained, map_builder=synthetic_builder(limit))
        self.assertFalse(budgeted.complete)
        self.assertTrue(budgeted.audit.budget_exhausted)

    def test_physical_smoke_uses_real_builder_when_scipy_is_available(self):
        try:
            from simplified_vehicle_models.compiler.compile import compile_formulation
            from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
        except ImportError:
            self.skipTest("physical formulation dependencies unavailable")
        from pathlib import Path
        root = Path(__file__).resolve().parents[1]
        model = compile_formulation(str(root / "models/formulations/iem24_murphy_real_planar_7dof.xml")).model
        physical_vehicle = point_mass_view_of_planar(model)
        # Keep the physical test intentionally tiny; direct adapter integration
        # is supplied by yaw_capability in the parent task.
        from simplified_vehicle_models.formulations.yaw_capability import direct_yaw_probe
        settings = YawMapWorkflowSettings(
            discovery=YawMapDomainSettings(scan_spacing=2., bracket_tolerance=.5, probe_budget=200,
                                            speed_ax_audit_subdivisions=0,
                                            lateral_interpolation_loss_tolerance=3.),
            maximum_lateral_cell_width=2., maximum_map_nodes=100, maximum_audit_probes=100,
            maximum_refinement_rounds=1, time_budget_seconds=60.,
        )
        result = build_yaw_map_workflow(physical_vehicle, [5.], [0.],
                                        lambda v, x, y: direct_yaw_probe(physical_vehicle, v, x, y),
                                        yaw_inertia=model.yaw_inertia, settings=settings)
        self.assertGreaterEqual(len(result.capability.lateral_acceleration), 3)
        self.assertEqual(result.capability.feasible_mask.shape,
                         result.capability.yaw_moment_out_of_turn.shape)


if __name__ == "__main__":
    unittest.main()
