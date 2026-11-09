"""Bounded full-plant current-state physical-inverse stopping diagnostic.

This tests only the operational v=0.01 m/s event from 3 m/s on a straight
positive-speed reference. It does not claim rest, generic low-speed support,
ABS, or production controller qualification.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

from simplified_vehicle_models.formulations.planar_7dof import evaluate_normal_loads, aerodynamic_forces
from simplified_vehicle_models.tires.unitire_simple import loaded_radius, evaluate_unitire
from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
from simplified_vehicle_models.solvers.vertical_forward import _drivetrain_context

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
from simplified_vehicle_models.solvers.forward_driver import DriverCommand, SpeedPlan
from simplified_vehicle_models.solvers.vertical_forward import (
    VerticalForwardPlant, WHEEL_SPEED_INDEX, SOC_INDEX,
)

ROOT = Path(__file__).resolve().parents[2]
FORMULATION = ROOT / 'models/formulations/iem24_murphy_real_14dof_simple.xml'
HYDRAULIC = ROOT / 'models/vehicles/iem24_murphy_real_planar.xml'
EVENT_SPEED_MPS = .01
INITIAL_SPEED_MPS = 3.
TARGETS = (.7, 1.)
MAX_TIME_S = 10.


def _rhs_factory(plant, predictor, maximum_force, target, *, controlled):
    def request(state):
        context = plant.brake_authority_context(state, predictor=predictor)
        result = predictor.request_current(context, target)
        return float(result.pedal_force_n), result

    def rhs(t, z):
        state = z[:-1] if controlled else z
        requested, _ = request(state)
        if controlled:
            pedal_position = float(np.clip(z[-1], -1., 1.))
            applied = max(-pedal_position, 0.) * maximum_force
            command = DriverCommand(max(pedal_position, 0.), applied, 0.)
            rates = plant.evaluate(t, state, command).solve()
            pedal_rate = float(np.clip((-requested / maximum_force - pedal_position) / .02,
                                       -1. / .175, 1. / .175))
            return np.r_[rates, pedal_rate]
        return plant.evaluate(t, state, DriverCommand(0., requested, 0.)).solve()
    return rhs, request


def _jacobian(rhs):
    def jac(t, y):
        base = rhs(t, y)
        out = np.empty((len(y), len(y)))
        for j in range(len(y)):
            step = 1e-7 * max(abs(float(y[j])), 1.)
            trial = y.copy(); trial[j] += step
            try:
                out[:, j] = (rhs(t, trial) - base) / step
            except ValueError:
                trial = y.copy(); trial[j] -= step
                out[:, j] = (base - rhs(t, trial)) / step
        return out
    return jac


def _event(t, y):
    return float(y[3]) - EVENT_SPEED_MPS
_event.terminal = True
_event.direction = -1


def _run(model, target, pedal_start, *, method='BDF', rtol=1e-5, atol=1e-7,
         max_step=.01, controlled=True, trace=None):
    # Positive-speed plan is deliberately open and far from its endpoint; it is
    # a reference only, not a terminal-speed clamp or stop target.
    plan = SpeedPlan(np.array([0., 100.]), np.array([3., 3.]), np.zeros(2), False)
    plant = VerticalForwardPlant(model, plan)
    predictor = BrakeAuthorityPredictor(model.planar, maximum_abs_curvature_inv_m=0.,
                                         minimum_speed_mps=.005, maximum_speed_mps=40.)
    y0 = plant.initial_state(INITIAL_SPEED_MPS)
    maximum = float(model.planar.maximum_pedal_force)
    rhs, request = _rhs_factory(plant, predictor, maximum, target, controlled=controlled)
    yinit = np.r_[y0, float(pedal_start)] if controlled else y0.copy()
    started = time.perf_counter()
    sol = solve_ivp(rhs, (0., MAX_TIME_S), yinit, method=method, rtol=rtol,
                    atol=atol, max_step=max_step, jac=_jacobian(rhs) if method == 'BDF' else None,
                    events=_event)
    wall = time.perf_counter() - started
    if not sol.success:
        raise RuntimeError(sol.message)
    states = sol.y[:len(y0)].T
    pedal = sol.y[-1] if controlled else np.full(len(sol.t), -target)
    rows = []
    for t, state, p in zip(sol.t, states, pedal):
        requested, pred = request(state)
        applied = max(-float(p), 0.) * maximum if controlled else requested
        ctx = plant.brake_authority_context(state, predictor=predictor)
        rows.append((requested, applied, pred.deceleration_mps2,
                     pred.limiting_constraint, ctx.normal_loads_n, ctx.wheel_radii_m))
    requested = np.asarray([r[0] for r in rows]); applied = np.asarray([r[1] for r in rows])
    event_crossed = bool(len(sol.t_events[0]))
    terminal = states[-1]
    wheels = terminal[list(WHEEL_SPEED_INDEX)]
    loads = np.asarray([r[4] for r in rows], dtype=float)
    radii = np.asarray([r[5] for r in rows], dtype=float)
    actual = np.asarray([ -plant.evaluate(float(t), state, DriverCommand(0., float(a), 0.)).solve()[3]
                          for t, state, a in zip(sol.t, states, applied)])
    truth = sol.t >= .5  # explicitly excludes the documented initial foot/pedal transient
    error = actual[truth] - target if np.any(truth) else np.array([])
    admission = bool(np.all(np.isfinite(sol.y)) and np.all(np.isfinite(requested))
                     and np.all(np.isfinite(applied)) and np.all(wheels > 0.)
                     and np.all(loads > 0.) and event_crossed)
    row = {
        'target_deceleration_mps2': target, 'initial_pedal_position': pedal_start,
        'law': 'BrakeAuthorityPredictor.request_current(plant.brake_authority_context(state))',
        'predictor_domain_mps': [.005, 40.], 'method': method, 'rtol': rtol,
        'atol': atol, 'max_step_s': max_step, 'success': bool(sol.success),
        'event_crossed': event_crossed, 'event_speed_mps': float(terminal[3]),
        'event_time_s': float(sol.t[-1]), 'event_station_m': float(terminal[0]),
        'terminal_wheel_speeds_rad_s': wheels.tolist(), 'terminal_soc': float(terminal[SOC_INDEX]),
        'minimum_contact_N': float(loads.min()), 'contact_loads_terminal_N': loads[-1].tolist(),
        'loaded_radii_terminal_m': radii[-1].tolist(), 'requested_force_initial_N': float(requested[0]),
        'requested_force_terminal_N': float(requested[-1]), 'applied_force_initial_N': float(applied[0]),
        'applied_force_terminal_N': float(applied[-1]), 'actual_deceleration_min_mps2': float(actual.min()),
        'actual_deceleration_max_mps2': float(actual.max()),
        'truth_gate_after_foot_delay_s': .5,
        'truth_gate_max_abs_error_mps2': float(np.max(np.abs(error))) if error.size else None,
        'truth_gate_mean_abs_error_mps2': float(np.mean(np.abs(error))) if error.size else None,
        'truth_gate_threshold_mps2': max(.25, .05 * target),
        'truth_gate_pass': bool(error.size and np.max(np.abs(error)) <= max(.25, .05 * target)),
        'authority_limit_initial': rows[0][3], 'authority_limit_terminal': rows[-1][3],
        'all_finite': bool(np.all(np.isfinite(sol.y))), 'admission_pass': admission,
        'samples': len(sol.t), 'nfev': sol.nfev, 'njev': sol.njev, 'nlu': sol.nlu,
        'wall_s': wall, 'terminal_is_operational_event_not_rest': True,
    }
    if trace is not None:
        trace.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(trace, time_s=sol.t, state=states, pedal_position=pedal,
                            requested_brake_force_N=requested, applied_brake_force_N=applied,
                            actual_deceleration_mps2=actual, predicted_deceleration_mps2=np.array([r[2] for r in rows]),
                            normal_load_N=loads, loaded_radius_m=radii, event_speed_mps=EVENT_SPEED_MPS)
    return row


def stop_distance_metrics(predicted_m, actual_m, *, max_error_fraction=.05):
    """Validate positive-distance forecast error against actual event station."""
    predicted = float(predicted_m); actual = float(actual_m)
    if not np.isfinite(predicted) or not np.isfinite(actual) or predicted <= 0. or actual <= 0.:
        raise ValueError('predicted and actual stop distances must be finite and positive')
    if not np.isfinite(max_error_fraction) or max_error_fraction <= 0.:
        raise ValueError('max_error_fraction must be finite and positive')
    signed = predicted - actual
    fraction = abs(signed) / actual
    return {'predicted_distance_m': predicted, 'actual_event_distance_m': actual,
            'signed_error_m': signed, 'absolute_error_m': abs(signed),
            'relative_error_fraction_actual_denominator': fraction,
            'threshold_fraction': float(max_error_fraction), 'pass': fraction <= max_error_fraction}


def forecast_stop(model, target, initial_pedal, initial_soc):
    """Reduced offline forecast: public inverse + exact pedal law + 3 load iterations."""
    vehicle = model.planar
    predictor = BrakeAuthorityPredictor(vehicle, maximum_abs_curvature_inv_m=0.,
                                         minimum_speed_mps=.005, maximum_speed_mps=40.)
    driven, gearing, efficiency = _drivetrain_context(vehicle)

    def rhs(t, y):
        speed, pedal, station = map(float, y)
        pedal = float(np.clip(pedal, -1., 1.))
        acceleration = 0.
        for _ in range(3):
            loads = evaluate_normal_loads(vehicle, speed, acceleration, 0.)
            loads = np.maximum(np.asarray(loads, dtype=float), 1.)
            radii = loaded_radius(vehicle.tire, loads)
            # Use current quasi-static loads/radii and the public state-aware inverse.
            context = predictor_context(speed, loads, radii, initial_soc)
            requested = predictor.request_current(context, target).pedal_force_n
            applied = max(-pedal, 0.) * vehicle.maximum_pedal_force
            command = DriverCommand(max(pedal, 0.), applied, 0.)
            power = evaluate_forward_powertrain(vehicle, driven, gearing, efficiency,
                                                speed / radii, initial_soc, command)
            tire = evaluate_unitire(vehicle.tire, np.zeros(4), np.zeros(4), loads, speed / radii)
            _, drag = aerodynamic_forces(vehicle, speed)
            wheel_force = float(np.sum((power.wheel_torques + tire.rolling_resistance_moment) / radii))
            eff_mass = vehicle.total_mass + vehicle.wheel_inertia * float(np.sum(1. / radii**2))
            acceleration = (wheel_force - drag) / eff_mass
        pedal_rate = float(np.clip((-requested / vehicle.maximum_pedal_force - pedal) / .02,
                                   -1. / .175, 1. / .175))
        return np.array([acceleration, pedal_rate, speed])

    def event(t, y): return y[0] - EVENT_SPEED_MPS
    event.terminal = True; event.direction = -1
    initial = np.array([INITIAL_SPEED_MPS, float(initial_pedal), 0.])
    started = time.perf_counter()
    # Explicit RK keeps internal trial stages close to the positive event and
    # avoids BDF Newton iterates probing below the predictor's declared floor.
    sol = solve_ivp(rhs, (0., MAX_TIME_S), initial, method='DOP853', rtol=1e-8, atol=1e-10,
                    max_step=.001, events=event)
    runtime = time.perf_counter() - started
    if not sol.success: raise RuntimeError(sol.message)
    return {'success': bool(sol.success), 'event_reached': bool(len(sol.t_events[0])),
            'event_speed_mps': float(sol.y[0, -1]), 'event_time_s': float(sol.t[-1]),
            'predicted_distance_m': float(sol.y[2, -1]), 'initial_speed_mps': INITIAL_SPEED_MPS,
            'initial_pedal_position': float(initial_pedal), 'initial_state_coordinates': {'speed_mps': INITIAL_SPEED_MPS, 'station_m': 0., 'pedal_position': float(initial_pedal)}, 'initial_soc': float(initial_soc),
            'target_deceleration_mps2': float(target), 'iterations_per_rhs': 3,
            'wall_s': runtime, 'nfev': sol.nfev}


def predictor_context(speed, loads, radii, soc):
    # Import here so the forecast remains a small harness-facing API consumer.
    from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
    return BrakeAuthorityContext(float(speed), 0., tuple(float(x) for x in radii),
                                 tuple(float(x) for x in loads), soc=float(np.clip(soc, 0., 1.)))


def parse_metrics(payload):
    data = json.loads(payload)
    if not isinstance(data, dict) or not isinstance(data.get('rows'), list):
        raise ValueError('expected object with rows array')
    return data


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--trace-dir', type=Path, default=ROOT / '.superpowers/sdd/implementation-plan')
    parser.add_argument('--distance-audit', action='store_true',
                        help='forecast against existing stop-controller traces; do not rerun full plant')
    args = parser.parse_args(argv)
    model = compile_formulation(str(FORMULATION)).model
    if args.distance_audit:
        results_path = args.trace_dir / 'stop-controller-results.json'
        actual = parse_metrics(results_path.read_text())
        forecasts = []
        for actual_row in actual['rows']:
            target = float(actual_row['target_deceleration_mps2'])
            pedal = float(actual_row['initial_pedal_position'])
            trace_path = args.trace_dir / actual_row['trace_file']
            with np.load(trace_path) as trace:
                initial_state = trace['state'][0]
                initial_soc = float(initial_state[SOC_INDEX])
            pred = forecast_stop(model, target, pedal, initial_soc)
            comparison = stop_distance_metrics(pred['predicted_distance_m'],
                                                actual_row['event_station_m'])
            forecasts.append({**pred, **comparison,
                              'trace_file': trace_path.name,
                              'actual_event_time_s': actual_row['event_time_s'],
                              'actual_source_hashes': {
                                  'formulation_sha256': actual['formulation_sha256'],
                                  'hydraulic_sha256': actual['hydraulic_sha256']}})
        audit = {'scope': 'offline reduced distance forecast compared to existing actual full-plant traces',
                 'operation': 'public API request_current inverse is evaluated at current RHS speed/load context; caller iterates quasi-static load/radius balance three times',
                 'maximum_predicted_to_actual_relative_error_fraction': max(r['relative_error_fraction_actual_denominator'] for r in forecasts),
                 'maximum_solver_wall_s_per_forecast': max(r['wall_s'] for r in forecasts),
                 'actual_full_plant_rerun': False,
                 'actual_trace_results_sha256': hashlib.sha256(results_path.read_bytes()).hexdigest(),
                 'formulation_sha256': hashlib.sha256(FORMULATION.read_bytes()).hexdigest(),
                 'hydraulic_sha256': hashlib.sha256(HYDRAULIC.read_bytes()).hexdigest(),
                 'event_speed_mps': EVENT_SPEED_MPS,
                 'forecast': 'public current-state inverse; actual signed pedal law; 3 quasi-static load/radius iterations; full forward powertrain closure for positive drive phase; tire rolling, aero drag and wheel inertia included',
                 'distance_error_denominator': 'actual event station',
                 'predeclared_relative_error_limit': .05,
                 'rows': forecasts,
                 'pass': len(forecasts) == 4 and all(r['event_reached'] and r['pass'] for r in forecasts)}
        output_path = args.output or args.trace_dir / 'stop-controller-distance-audit.json'
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(audit, indent=2) + '\n')
        print(json.dumps(audit, indent=2))
        return 0 if audit['pass'] else 1
    maximum = float(model.planar.maximum_pedal_force)
    args.output = args.output or ROOT / '.superpowers/sdd/implementation-plan/stop-controller-results.json'
    rows = []
    for target in TARGETS:
        for pedal in (0., .3):
            tag = f'stop-controller-{target:g}-{pedal:g}'
            try:
                row = _run(model, target, pedal, trace=args.trace_dir / f'{tag}.npz')
                row['trace_file'] = f'{tag}.npz'
            except Exception as exc:
                row = {'target_deceleration_mps2': target, 'initial_pedal_position': pedal,
                       'admission_pass': False, 'causal_status': f'{type(exc).__name__}: {exc}'}
            rows.append(row)
            print(f"target={target:g} initial_pedal={pedal:g} pass={row.get('admission_pass')} "+
                  f"status={row.get('causal_status', row.get('event_station_m'))}", flush=True)
    output = {
        'scope': 'two-target full-plant stop-controller diagnostic; not Task 4 completion',
        'formulation': FORMULATION.name, 'formulation_sha256': hashlib.sha256(FORMULATION.read_bytes()).hexdigest(),
        'hydraulic_sha256': hashlib.sha256(HYDRAULIC.read_bytes()).hexdigest(),
        'initial_speed_mps': INITIAL_SPEED_MPS, 'event_speed_mps': EVENT_SPEED_MPS,
        'speed_plan': {'kind': 'open constant positive reference', 'speeds_mps': [3., 3.], 'stations_m': [0., 100.]},
        'maximum_pedal_force_N': maximum, 'regen_enabled': bool(model.planar.controller.regen_enabled),
        'predictor_low_speed_extension': {'minimum_speed_mps': .005, 'explicitly_validated_extension': True,
                                           'generic_zero_speed_claim': False},
        'actuator': 'same full-plant pedal state and signed pedal law; initial pedal 0 or +0.3',
        'jacobian': 'BDF forward FD step 1e-7*max(abs(state_j),1), negative one-sided fallback on ValueError',
        'rows': rows, 'pass': len(rows) == 4 and all(r.get('admission_pass') and r.get('truth_gate_pass') for r in rows),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps(output, indent=2))
    return 0 if output['pass'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
