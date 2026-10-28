"""Discover and audit vehicle-limit yaw-map coverage, saving reproducible evidence.

Exit 0 means the map passed finite-resolution audit; exit 4 means a saved,
inspectable map/report is incomplete; other nonzero exits indicate build errors.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback


def _positive_workers(value):
    try:
        workers = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError('workers must be a positive integer')
    if str(workers) != str(value) or workers < 1:
        raise argparse.ArgumentTypeError('workers must be a positive integer')
    return workers


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
    p.add_argument('--max-nodes', type=int, default=3000000)
    p.add_argument('--max-probes', type=int, default=250000)
    p.add_argument('--refinement-rounds', type=int, default=768)
    p.add_argument('--audit-probes', type=int, default=500000)
    p.add_argument('--wall-seconds', type=float, default=1200.)
    p.add_argument('--workers', type=_positive_workers, default=1)
    return p


def _emit(event, **fields):
    print(json.dumps({'timestamp': __import__('datetime').datetime.now().astimezone().isoformat(),
                      'event': event, **fields}, allow_nan=False), flush=True)


def _write_failure(output, args, started, phase, error):
    report_path = output.with_suffix('.json')
    payload = {'status': 'error', 'complete': False, 'phase': phase,
               'error_type': type(error).__name__, 'error': str(error),
               'traceback': traceback.format_exc(), 'workers': args.workers,
               'wall_seconds': time.monotonic() - started,
               'artifact': str(output), 'timestamp': __import__('datetime').datetime.now().astimezone().isoformat()}
    if report_path.exists():
        report_path = output.with_name(output.stem + '.failure.json')
    report_path.write_text(json.dumps(payload, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    _emit('error', report=str(report_path), phase=phase, error_type=payload['error_type'], error=str(error))


def main(argv=None):
    args = parser().parse_args(argv)
    if args.output.exists():
        parser().error('output already exists; choose a fresh artifact path')
    if args.speed_points < 2 or args.ax_points < 2:
        parser().error('at least two speed and ax knots are required')
    started = time.monotonic()
    phase = 'startup'
    _emit('startup', command=[sys.executable, '-u', '-m', 'tools.run.build_vehicle_yaw_map', *(argv or sys.argv[1:])],
          formulation=str(args.formulation), output=str(args.output), workers=args.workers,
          cpu_count=os.cpu_count() or 1, wall_seconds=args.wall_seconds)
    try:
        phase = 'imports'
        from dataclasses import asdict
        import numpy as np
        from simplified_vehicle_models.formulations.yaw_map_artifact import (
            build_identity, json_values, save_yaw_map, source_provenance,
        )
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
        from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
        from simplified_vehicle_models.formulations.point_mass_capability import capability_hash
        from simplified_vehicle_models.formulations.yaw_map_domain import YawMapDomainSettings
        from simplified_vehicle_models.formulations.yaw_map_workflow import YawMapWorkflowSettings, build_yaw_map_workflow
        from simplified_vehicle_models.formulations.yaw_map_parallel import ParallelYawEvaluator
        from tools.run.run_yaw_aware_forward_driver import anchored_axis, physical_map_speed_limit
        phase = 'compile'
        _emit('phase', phase=phase)
        compiled = compile_formulation(str(args.formulation)).model
        if isinstance(compiled, VerticalVehicleParameters):
            compiled = compiled.planar
        vehicle = point_mass_view_of_planar(compiled)
        speed = np.linspace(0., physical_map_speed_limit(vehicle), args.speed_points)
        ax = anchored_axis(args.ax_min, args.ax_max, args.ax_points)
        domain_settings = YawMapDomainSettings(scan_spacing=args.scan_spacing, bracket_tolerance=args.boundary_tolerance,
            probe_budget=args.max_probes, lateral_interpolation_loss_tolerance=args.loss_tolerance)
        settings = YawMapWorkflowSettings(discovery=domain_settings, maximum_lateral_cell_width=args.ay_spacing,
            maximum_map_nodes=args.max_nodes, maximum_audit_probes=args.audit_probes,
            maximum_refinement_rounds=args.refinement_rounds, time_budget_seconds=args.wall_seconds)
        identity = build_identity(vehicle, compiled.yaw_inertia, settings, speed, ax)
        remaining = max(0., args.wall_seconds - (time.monotonic() - started))
        phase = 'training-and-validation'
        _emit('phase', phase=phase, deadline_seconds=args.wall_seconds, workers=args.workers)
        # Distinct evaluators/caches; validation runs after training, sharing the
        # workflow's absolute deadline without overlapping process pools.
        def workflow_progress(event, fields):
            _emit('progress', phase=event, **fields)
        with ParallelYawEvaluator(vehicle, workers=args.workers, time_budget_seconds=remaining,
                                  use_reduced=True) as evaluator:
            result = build_yaw_map_workflow(vehicle, speed, ax, evaluator,
                yaw_inertia=compiled.yaw_inertia, settings=settings, map_builder=evaluator.build,
                progress_callback=workflow_progress)
        phase = 'save'
        _emit('phase', phase=phase, complete=bool(result.complete))
        report = {k: json_values(v) for k, v in asdict(result).items() if k != 'capability'}
        report['wall_seconds'] = time.monotonic() - started
        report['workers'] = args.workers
        report['declared_ax_domain_mps2'] = [args.ax_min, args.ax_max]
        revision = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True, check=False).stdout.strip()
        save_yaw_map(args.output, result.capability, report=report, identity=identity,
            provenance={'formulation': str(args.formulation), 'source_sha256': source_provenance(args.formulation),
                        'compiled_vehicle_sha256': capability_hash(vehicle), 'git_revision': revision,
                        'workers': args.workers,
                        'identity_policy':'compiled vehicle parameters and builder implementation are authoritative for cache identity; workers are execution metadata, not physical identity; XML bytes retained for provenance',
                        'settings': asdict(settings), 'scope': 'quasi-static model, finite-resolution lateral audit; declared ax domain only'})
        summary = {'status': 'complete' if result.complete else 'incomplete', 'complete': result.complete,
                   'artifact': str(args.output), 'shape': list(result.capability.feasible_mask.shape),
                   'ay_extent_mps2': result.capability.lateral_acceleration[[0,-1]].tolist(),
                   'training_interpolation_loss_mps2': result.audit.maximum_interpolation_loss,
                   'validation_interpolation_loss_mps2': result.validation_audit.maximum_interpolation_loss,
                   'validation_complete': result.validation_audit.complete,
                   'workers': args.workers, 'wall_seconds': report['wall_seconds']}
        args.output.with_suffix('.json').write_text(json.dumps(json_values(report), indent=2, allow_nan=False)+'\n', encoding='utf-8')
        _emit('terminal', **summary)
        return 0 if result.complete else 4
    except BaseException as error:
        try:
            _write_failure(args.output, args, started, phase, error)
        except Exception as report_error:
            _emit('error', phase=phase, error=str(error), report_error=str(report_error))
        if isinstance(error, KeyboardInterrupt):
            return 130
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
