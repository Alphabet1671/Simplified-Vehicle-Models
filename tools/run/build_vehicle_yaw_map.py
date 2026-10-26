"""Discover and audit vehicle-limit yaw-map coverage, saving reproducible evidence.

A nonzero exit means the saved map failed the declared finite-resolution audit;
use its report to refine the build rather than silently using a truncated map.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import subprocess
import time
import numpy as np

from simplified_vehicle_models.formulations.yaw_map_artifact import (
    build_identity, json_values, save_yaw_map, source_provenance,
)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--formulation', type=Path, default=Path('models/formulations/iem24_murphy_real_planar_7dof.xml'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--speed-points', type=int, default=12)
    p.add_argument('--ax-points', type=int, default=9)
    p.add_argument('--ax-min', type=float, default=-6.)
    p.add_argument('--ax-max', type=float, default=3.)
    p.add_argument('--ay-spacing', type=float, default=.5)
    p.add_argument('--scan-spacing', type=float, default=.5)
    p.add_argument('--boundary-tolerance', type=float, default=.05)
    p.add_argument('--loss-tolerance', type=float, default=.25)
    p.add_argument('--max-nodes', type=int, default=30000)
    p.add_argument('--max-probes', type=int, default=250000)
    p.add_argument('--refinement-rounds', type=int, default=32)
    p.add_argument('--audit-probes', type=int, default=500000)
    p.add_argument('--wall-seconds', type=float, default=600.)
    return p


def main(argv=None):
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from simplified_vehicle_models.formulations.point_mass_capability import capability_hash
    from simplified_vehicle_models.formulations.yaw_capability import direct_yaw_probe
    from simplified_vehicle_models.formulations.yaw_map_domain import YawMapDomainSettings
    from simplified_vehicle_models.formulations.yaw_map_workflow import YawMapWorkflowSettings, build_yaw_map_workflow
    from tools.run.run_yaw_aware_forward_driver import anchored_axis, physical_map_speed_limit
    p = parser(); args = p.parse_args(argv)
    if args.output.exists(): p.error('output already exists; choose a fresh artifact path')
    if args.speed_points < 2 or args.ax_points < 2: p.error('at least two speed and ax knots are required')
    domain_settings = YawMapDomainSettings(scan_spacing=args.scan_spacing, bracket_tolerance=args.boundary_tolerance,
        probe_budget=args.max_probes, lateral_interpolation_loss_tolerance=args.loss_tolerance)
    settings = YawMapWorkflowSettings(discovery=domain_settings, maximum_lateral_cell_width=args.ay_spacing,
        maximum_map_nodes=args.max_nodes, maximum_audit_probes=args.audit_probes,
        maximum_refinement_rounds=args.refinement_rounds, time_budget_seconds=args.wall_seconds)
    compiled = compile_formulation(str(args.formulation)).model
    vehicle = point_mass_view_of_planar(compiled)
    speed = np.linspace(0., physical_map_speed_limit(vehicle), args.speed_points)
    ax = anchored_axis(args.ax_min, args.ax_max, args.ax_points)
    identity = build_identity(vehicle, compiled.yaw_inertia, settings, speed, ax)
    started = time.monotonic()
    from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
    evaluator = PhysicalYawEvaluator(vehicle, time_budget_seconds=settings.time_budget_seconds)
    result = build_yaw_map_workflow(vehicle, speed, ax, evaluator,
        yaw_inertia=compiled.yaw_inertia, settings=settings, map_builder=evaluator.build)
    report = {k: json_values(v) for k, v in asdict(result).items() if k != 'capability'}
    report['wall_seconds'] = time.monotonic() - started
    report['declared_ax_domain_mps2'] = [args.ax_min, args.ax_max]
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True, check=False).stdout.strip()
    save_yaw_map(args.output, result.capability, report=report, identity=identity,
        provenance={'formulation': str(args.formulation), 'source_sha256': source_provenance(args.formulation),
                    'compiled_vehicle_sha256': capability_hash(vehicle),
                    'git_revision': revision,
                    'identity_policy':'compiled vehicle parameters and builder implementation are authoritative for cache identity; XML bytes retained for provenance',
                    'settings': asdict(settings), 'scope': 'quasi-static model, finite-resolution lateral audit; declared ax domain only'})
    summary = {'complete': result.complete, 'artifact': str(args.output),
               'shape': list(result.capability.feasible_mask.shape),
               'ay_extent_mps2': result.capability.lateral_acceleration[[0,-1]].tolist(),
               'maximum_interpolation_loss_mps2': result.audit.maximum_interpolation_loss,
               'wall_seconds': report['wall_seconds']}
    args.output.with_suffix('.json').write_text(json.dumps(json_values(report), indent=2, allow_nan=False)+'\n')
    print(json.dumps(json_values(summary), indent=2, allow_nan=False))
    return 0 if result.complete else 4


if __name__ == '__main__':
    raise SystemExit(main())
