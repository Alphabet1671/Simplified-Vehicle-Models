"""Shared map options for the driver, fixed-path probe, and preflight."""
from dataclasses import asdict
from pathlib import Path
import numpy as np
from simplified_vehicle_models.formulations.yaw_map_artifact import (
    build_identity, json_values, load_yaw_map, save_yaw_map,
)


class IncompleteYawMapError(ValueError):
    def __init__(self, result):
        self.result = result
        super().__init__('vehicle-limit yaw-map audit incomplete; inspect map-build evidence, increase resolution/budgets, or load a completed artifact')


def add_yaw_map_options(parser):
    parser.add_argument('--yaw-map', type=Path, help='load a completed, identity-checked vehicle-limit map artifact')
    parser.add_argument('--yaw-map-mode', choices=('vehicle-limit', 'limited-grid'), default='vehicle-limit',
                        help='limited-grid is explicitly truncated legacy comparison mode')
    parser.add_argument('--yaw-ay-spacing', type=float, default=.5)
    parser.add_argument('--yaw-map-ax-min', type=float, default=-6.)
    parser.add_argument('--yaw-map-ax-max', type=float, default=3.)
    parser.add_argument('--yaw-map-scan-spacing', type=float, default=.5)
    parser.add_argument('--yaw-map-boundary-tolerance', type=float, default=.05)
    parser.add_argument('--yaw-map-max-probes', type=int, default=250000)
    parser.add_argument('--yaw-map-loss-tolerance', type=float, default=.25)
    parser.add_argument('--yaw-map-wall-seconds', type=float, default=600.)
    parser.add_argument('--yaw-map-max-nodes', type=int, default=500000)
    parser.add_argument('--yaw-map-audit-probes', type=int, default=500000)
    parser.add_argument('--yaw-map-refinement-rounds', type=int, default=128)


def resolve_yaw_map(vehicle, yaw_inertia, speeds, ax, args, *, limited_ay=None, save_path=None):
    """Build/load by the same declared settings, rejecting incomplete audits."""
    from simplified_vehicle_models.formulations.yaw_capability import (
        YawCapabilityGrid, build_physical_yaw_map, direct_yaw_probe,
    )
    from simplified_vehicle_models.formulations.yaw_map_domain import YawMapDomainSettings
    from simplified_vehicle_models.formulations.yaw_map_workflow import YawMapWorkflowSettings, build_yaw_map_workflow
    mode = getattr(args, 'yaw_map_mode', 'vehicle-limit')
    if mode == 'limited-grid':
        if getattr(args, 'yaw_map', None): raise ValueError('--yaw-map is incompatible with limited-grid mode')
        if limited_ay is None: raise ValueError('limited-grid requires an explicit lateral axis')
        cap = build_physical_yaw_map(vehicle, YawCapabilityGrid(speeds, ax, limited_ay), yaw_inertia=yaw_inertia)
        return cap, {'complete':False, 'mode':mode, 'scope':'explicit limited-domain legacy comparison; not vehicle-limit coverage'}
    settings = YawMapWorkflowSettings(
        discovery=YawMapDomainSettings(probe_budget=getattr(args, 'yaw_map_max_probes', 250000),
            scan_spacing=getattr(args, 'yaw_map_scan_spacing', .5),
            bracket_tolerance=getattr(args, 'yaw_map_boundary_tolerance', .05),
            lateral_interpolation_loss_tolerance=getattr(args, 'yaw_map_loss_tolerance', .25)),
        maximum_lateral_cell_width=getattr(args, 'yaw_ay_spacing', .5),
        maximum_map_nodes=getattr(args, 'yaw_map_max_nodes', 500000),
        maximum_audit_probes=getattr(args, 'yaw_map_audit_probes', 500000),
        maximum_refinement_rounds=getattr(args, 'yaw_map_refinement_rounds', 128),
        time_budget_seconds=getattr(args, 'yaw_map_wall_seconds', 600.))
    identity = build_identity(vehicle, yaw_inertia, settings, speeds, ax)
    if getattr(args, 'yaw_map', None):
        cap, metadata = load_yaw_map(args.yaw_map, expected_identity=identity)
        return cap, metadata['report']
    from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
    evaluator = PhysicalYawEvaluator(vehicle, time_budget_seconds=settings.time_budget_seconds, use_reduced=True)
    validation = PhysicalYawEvaluator(vehicle, time_budget_seconds=settings.time_budget_seconds)
    result = build_yaw_map_workflow(vehicle, speeds, ax, evaluator,
        yaw_inertia=yaw_inertia, settings=settings, map_builder=evaluator.build,
        validation_evaluator=validation)
    report = {k:json_values(v) for k,v in asdict(result).items() if k != 'capability'}
    report.update(mode=mode, declared_ax_domain_mps2=[float(min(ax)),float(max(ax))], settings=json_values(settings))
    if save_path is not None:
        from simplified_vehicle_models.formulations.yaw_map_artifact import source_provenance
        formulation = Path(__file__).resolve().parents[2] / 'models/formulations/iem24_murphy_real_planar_7dof.xml'
        save_yaw_map(save_path, result.capability, report=report, identity=identity,
                     provenance={'source_sha256':source_provenance(formulation),
                                 'identity_policy':'compiled vehicle parameters and builder implementation are authoritative for cache identity'})
    if not result.complete: raise IncompleteYawMapError(result)
    return result.capability, report
