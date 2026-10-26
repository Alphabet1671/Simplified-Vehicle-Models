"""Bounded, non-mutating closed/cut-open QSS comparisons.

Run with PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m
 tools.diagnostics.compare_qss_track_boundaries --help.
Results are numerical diagnostics, not event acceptance or infeasibility proofs.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.solvers.qss_line_ocp import (
    QssSettings, build_mesh, build_ocp, _decision_scaling, _validate_solver_result,
)


def track_view(track, *, is_closed: bool, length: float | None = None):
    """Cut topology in memory, retaining samples; optionally slice an open prefix.

    Full open cuts retain the final cross-section even if it is at the same XY
    position as the first. No open-XML geometry reinterpretation is attempted.
    """
    if length is not None:
        if not math.isfinite(length) or not 0.0 < length <= track.lap_length:
            raise ValueError('prefix length must be positive and no longer than track')
        if is_closed and length < track.lap_length:
            raise ValueError('a cropped track cannot be declared closed')
    count = track.distance.size if length is None else int(np.searchsorted(track.distance, length, side='right'))
    if count < 2:
        raise ValueError('prefix must retain at least two nodes')
    fields = {name: getattr(track, name)[:count] for name in ('distance', 'radius', 'curvature')}
    segments = np.diff(fields['distance'])
    segments.setflags(write=False)
    path = track.path_coordinates
    if path is not None:
        path = replace(path, is_closed=is_closed, **{
            name: getattr(path, name)[:count]
            for name in ('distance', 'x_north', 'y_east', 'heading', 'curvature')
        })
    for name in ('half_width_left', 'half_width_right'):
        values = getattr(track, name)
        fields[name] = None if values is None else values[:count]
    return replace(track, **fields, segment_length=segments, is_closed=is_closed, path_coordinates=path)


def make_cases(track, prefix_length: float):
    return [
        ('closed_full', track_view(track, is_closed=True)),
        ('open_full', track_view(track, is_closed=False)),
        ('open_prefix', track_view(track, is_closed=False, length=prefix_length)),
    ]


def common_seed(mesh, speed: float):
    """Identical per-node seed for either topology; no envelope/basis confound."""
    nodes = len(mesh)
    return np.concatenate([np.zeros(nodes), np.zeros(nodes), np.full(nodes, speed),
                           np.zeros(nodes), np.zeros(nodes), np.zeros(nodes - 1), np.zeros(nodes - 1)])


def _fallback_constraint_rows(built, constraint_count):
    """Name the current per-node builder rows, with deliberately strict guards.

    ``build_ocp`` currently does not expose row metadata.  Keep this fallback
    coupled to its append order rather than silently accepting a different NLP:
    per element it appends three HS defects, a midpoint-speed denominator guard,
    two quarter-point corridor rows, one midpoint corridor row, and three sets
    of five physical-limit rows.  Both topologies append three final boundary
    rows, but their meanings differ.
    """
    count = int(built['count'])
    if built.get('offset_basis') is not None or int(built.get('control_count', count + 1)) != count + 1:
        raise ValueError('residual fallback only supports the per-node offset layout')
    rows_per_element = 22
    boundary_rows = 3
    if constraint_count != rows_per_element * count + boundary_rows:
        raise ValueError('per-node constraint layout changed; update diagnostic row mapping')

    labels, hs_indices = [], []
    limit_names = ('curvature_domain', 'capability_acceleration_min',
                   'capability_acceleration_max', 'speed_ceiling', 'fold_guard')
    for element in range(count):
        prefix = f'element_{element}'
        for state in ('offset', 'heading', 'speed'):
            hs_indices.append(len(labels))
            labels.append(f'{prefix}:hs_{state}')
        labels.extend((f'{prefix}:midpoint_speed_domain',
                       f'{prefix}:interior_corridor_t25',
                       f'{prefix}:interior_corridor_t75',
                       f'{prefix}:interior_corridor_midpoint'))
        for sample in ('node_start', 'midpoint', 'node_end'):
            labels.extend(f'{prefix}:{sample}:{limit}' for limit in limit_names)
    if bool(built.get('is_closed', False)):
        labels.extend(('closed_boundary_offset', 'closed_boundary_heading',
                       'closed_boundary_speed'))
    else:
        labels.extend(('open_boundary_start_heading', 'open_boundary_end_heading',
                       'open_boundary_start_speed'))
    return labels, np.asarray(hs_indices, dtype=int)


def _constraint_rows(built, constraint_count):
    """Prefer optional builder row labels; otherwise use the guarded fallback."""
    metadata = built.get('constraint_row_metadata')
    if metadata is None:
        layout = built.get('constraint_layout')
        metadata = layout.get('row_labels') if isinstance(layout, dict) else None
    if metadata is None:
        return _fallback_constraint_rows(built, constraint_count)
    if len(metadata) != constraint_count or not all(isinstance(label, str) for label in metadata):
        raise ValueError('builder constraint row metadata does not match constraints')
    labels = list(metadata)
    hs_indices = np.asarray([index for index, label in enumerate(labels)
                             if ':hs_' in label], dtype=int)
    if hs_indices.size == 0:
        raise ValueError('builder constraint row metadata has no HS defect rows')
    return labels, hs_indices


def residuals(values, constraints, built):
    gv = np.maximum(np.maximum(built['g_lower'] - constraints, 0.0), constraints - built['g_upper'])
    xv = np.maximum(np.maximum(built['x_lower'] - values, 0.0), values - built['x_upper'])
    eq = built['g_lower'] == built['g_upper']
    if constraints.size != built['g_lower'].size or constraints.size != built['g_upper'].size:
        raise ValueError('constraint values and bounds have inconsistent sizes')
    labels, defect_indices = _constraint_rows(built, constraints.size)
    worst = int(np.argmax(gv))
    return dict(constraint_violation=float(np.max(gv)), bound_violation=float(np.max(xv)),
                equality_violation=float(np.max(gv[eq])), worst_constraint=labels[worst],
                hs_defect_violation=float(np.max(gv[defect_indices])))


def synthetic_capability_control():
    """Return an explicitly nonphysical smooth control, never a vehicle calibration."""
    from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
    return CapabilitySurface(np.array([0.5, 40.0]), np.array([-1.0, 1.0]),
                             np.full((2, 2), 3.0), np.full((2, 2), -8.0),
                             np.full(2, 10.0), 'NONPHYSICAL_CONSTANT_CONTROL')


def solve_diagnostic(track, corridor, surface, settings, spacing, *, speed_cap=None):
    """Run production NLP/options with a bounded budget and retain failed residuals."""
    import casadi as ca

    if settings.offset_control_spacing is not None:
        raise ValueError('primary diagnostic requires the shared per-node layout')
    mesh = build_mesh(track, spacing)
    built = dict(build_ocp(track, corridor, surface, mesh, settings), is_closed=bool(track.is_closed))
    if speed_cap is not None:
        if not np.isfinite(speed_cap) or speed_cap < max(settings.minimum_speed, settings.start_speed):
            raise ValueError('speed cap must be finite and at least the start speed')
        built['x_upper'][2 * mesh.size:3 * mesh.size] = np.minimum(
            built['x_upper'][2 * mesh.size:3 * mesh.size], speed_cap)
    center, scale = _decision_scaling(mesh.size)
    seed = common_seed(mesh, settings.start_speed)
    seed = np.clip(seed, built['x_lower'], built['x_upper'])
    physical = built['nlp']['x']
    scaled = ca.MX.sym('boundary_scaled', seed.size)
    substitution = ca.DM(center) + ca.DM(scale) * scaled
    nlp = dict(x=scaled, f=ca.substitute(built['nlp']['f'], physical, substitution),
               g=ca.substitute(built['nlp']['g'], physical, substitution))
    opts = {'ipopt.tol': settings.tolerance, 'ipopt.constr_viol_tol': settings.tolerance,
            'ipopt.max_iter': settings.maximum_iterations, 'ipopt.hessian_approximation': 'exact',
            'ipopt.nlp_scaling_method': 'gradient-based', 'ipopt.mu_strategy': 'adaptive',
            'ipopt.print_level': 0, 'print_time': False}
    solver = ca.nlpsol('boundary_compare', 'ipopt', nlp, opts)
    started = time.perf_counter()
    result = solver(x0=(seed-center)/scale, lbx=(built['x_lower']-center)/scale,
                    ubx=(built['x_upper']-center)/scale,
                    lbg=built['g_lower'], ubg=built['g_upper'])
    stats = solver.stats()
    values = center + scale * np.asarray(result['x']).ravel()
    constraints = np.asarray(result['g']).ravel()
    objective = float(result['f'])
    metrics = residuals(values, constraints, built)
    status = str(stats['return_status'])
    accepted = False
    rejection = None
    try:
        _validate_solver_result(status, values, constraints, objective,
                                metrics['constraint_violation'], settings.tolerance,
                                built['x_lower'], built['x_upper'])
        accepted = True
    except RuntimeError as error:
        rejection = str(error)
    iterations = stats.get('iterations', {})
    dual = iterations.get('inf_du', [])
    return dict(status=status, accepted=accepted, rejection=rejection,
                iterations=int(stats['iter_count']), wall_seconds=time.perf_counter()-started,
                objective_s=objective if np.isfinite(objective) else None,
                last_reported_dual_infeasibility=float(dual[-1]) if len(dual) else None,
                nodes=mesh.size, variables=seed.size, constraints=constraints.size,
                is_closed=bool(track.is_closed), length_m=track.lap_length,
                seed_sha256=hashlib.sha256(seed.tobytes()).hexdigest(), **metrics)


def source_assets(source):
    source = Path(source).resolve()
    root = ET.parse(source).getroot()
    return [source] + [(source.parent / item.get('ref')).resolve()
                       for item in root.findall('./data') if item.get('ref')]


def run(args):
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from simplified_vehicle_models.formulations.point_mass_capability import load_capability_surface, capability_hash

    if Path(args.output).exists() or Path(args.output).is_symlink():
        raise FileExistsError('diagnostic output already exists; select a new output path')
    assets = [asset for source in args.track for asset in source_assets(source)]
    assets += [Path(args.surface).resolve(), Path(args.formulation).resolve()]
    output = Path(args.output).resolve()
    for asset in assets:
        if output == asset or (output.exists() and asset.exists() and output.samefile(asset)):
            raise ValueError('diagnostic output would overwrite an input asset')
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in assets}
    vehicle = point_mass_view_of_planar(compile_formulation(str(args.formulation)).model)
    surface = load_capability_surface(args.surface)
    if surface.provenance_hash != capability_hash(vehicle):
        raise ValueError('capability provenance does not match formulation')
    control_surface = bool(getattr(args, 'synthetic_capability_control', False))
    if control_surface:
        surface = synthetic_capability_control()
    speed_cap = getattr(args, 'speed_cap', None)
    settings = QssSettings(tolerance=1e-6, maximum_iterations=args.maximum_iterations,
                           start_speed=args.start_speed, open_regularisation=0.0)
    report = dict(settings=asdict(settings), spacing_m=args.spacing,
                  prefix_requested_m=args.prefix_length, margin_m=args.margin,
                  fallback_half_width_m=args.fallback_half_width,
                  capability_mode='NONPHYSICAL_CONSTANT_CONTROL' if control_surface else 'production',
                  diagnostic_speed_cap_mps=speed_cap,
                  caveats=['Rolling-start diagnostic, not event acceptance.',
                           'Opening changes endpoint constraints and periodic sampling.',
                           'Prefixes change both size and sampled geometry.',
                           'Failed bounded convergence is not proof of infeasibility.'],
                  input_sha256=hashes, results=[])
    output.parent.mkdir(parents=True, exist_ok=True)
    def save(*, initial=False):
        with output.open('x' if initial else 'w', encoding='utf-8') as handle:
            handle.write(json.dumps(report, indent=2, allow_nan=False) + '\n')
    save(initial=True)
    for source in args.track:
        original = load_track(source)
        for case, view in make_cases(original, args.prefix_length):
            print(f'{source}: {case}, L={view.lap_length:.3f} m', flush=True)
            corridor = corridor_from_track(view, margin=args.margin,
                        half_width=None if view.has_edges else args.fallback_half_width)
            try:
                result = solve_diagnostic(view, corridor, surface, settings, args.spacing, speed_cap=speed_cap)
            except Exception as error:
                result = dict(accepted=False, status='diagnostic_exception', error=f'{type(error).__name__}: {error}')
            result.update(track=str(source), case=case, corridor_provenance=corridor.provenance)
            report['results'].append(result)
            save()
            print(json.dumps(result, allow_nan=False), flush=True)
    report['source_data_unchanged'] = all(hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
                                          for path, digest in hashes.items())
    save()
    if not report['source_data_unchanged']:
        raise RuntimeError('input assets changed during comparison')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track', type=Path, action='append', required=True)
    parser.add_argument('--formulation', type=Path, required=True)
    parser.add_argument('--surface', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--spacing', type=float, default=5.0)
    parser.add_argument('--maximum-iterations', type=int, default=200)
    parser.add_argument('--start-speed', type=float, default=10.0)
    parser.add_argument('--prefix-length', type=float, default=200.0)
    parser.add_argument('--fallback-half-width', type=float, default=3.0)
    parser.add_argument('--margin', type=float, default=0.1)
    parser.add_argument('--synthetic-capability-control', action='store_true',
                        help='NONPHYSICAL constant capability control; never acceptance')
    parser.add_argument('--speed-cap', type=float, default=None,
                        help='optional diagnostic node-speed cap, not production settings')
    args = parser.parse_args(argv)
    if not np.isfinite(args.start_speed) or args.start_speed < 0.5:
        parser.error('start speed must be finite and at least 0.5 m/s')
    run(args)
    return 0  # Running a failed diagnostic is not the same as passing acceptance.


if __name__ == '__main__':
    raise SystemExit(main())
