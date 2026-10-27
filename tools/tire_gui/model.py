"""Immutable repository-compiled tire model snapshots and evaluator boundary."""
from hashlib import sha256
from pathlib import Path
from typing import Mapping

import numpy as np

from simplified_vehicle_models.xml.loader import load_model
from simplified_vehicle_models.tires.unitire_simple import compile_unitire_simple, evaluate_unitire, loaded_radius
from simplified_vehicle_models.tires.unitire_transient import TransientTireParameters, compile_unitire_transient
from .contracts import ModelSnapshot, SampleSet
from .parameters import CANONICAL_STEADY_NAMES, TRANSIENT_NAMES, flatten_steady, unflatten_steady


def _fingerprint(values):
    payload = '\n'.join(f'{k}={values[k].hex()}' for k in sorted(CANONICAL_STEADY_NAMES))
    return sha256(payload.encode()).hexdigest()

def _make_snapshot(steady, transient, revision, provenance, statuses=None, load_domain=(0.0,1500.0)):
    values = flatten_steady(steady)
    values.update({n: float(getattr(transient, n)) for n in TRANSIENT_NAMES})
    for name, coefficients in (('k_x', steady.k_x), ('k_y', steady.k_y)):
        polynomial = np.poly1d(coefficients)
        candidates = [load_domain[0], load_domain[1]]
        derivative = polynomial.deriv()
        if derivative.order > 0:
            candidates.extend(float(root.real) * steady.f_z_rated for root in derivative.r if abs(root.imag) < 1e-12 and load_domain[0] <= root.real * steady.f_z_rated <= load_domain[1])
        for load in candidates:
            stiffness = float(polynomial(load / steady.f_z_rated))
            if not np.isfinite(stiffness) or stiffness <= 0:
                raise ValueError(f'{name} slip stiffness must be finite and positive over load domain at {load:g} N: {stiffness}')
    for load in np.linspace(load_domain[0], load_domain[1], 501):
        sigma_x, sigma_y = (transient.longitudinal_length_scale * np.polyval(steady.k_x, load / steady.f_z_rated) / max(steady.k_cx, steady.stiff_eps), transient.lateral_length_scale * np.polyval(steady.k_y, load / steady.f_z_rated) / max(steady.k_cy, steady.stiff_eps))
        if not (np.isfinite(sigma_x) and np.isfinite(sigma_y) and sigma_x > 0 and sigma_y > 0):
            raise ValueError(f'relaxation lengths must be finite and positive at {load:g} N')
    return ModelSnapshot(revision, steady, transient, values, provenance, statuses or {}, _fingerprint(values), load_domain)

def load_snapshot(steady_xml: Path, transient_xml: Path | None = None, load_domain: tuple[float, float] = (0.0, 1500.0)) -> ModelSnapshot:
    if len(load_domain) != 2 or not np.all(np.isfinite(load_domain)) or load_domain[0] < 0 or load_domain[1] <= load_domain[0]:
        raise ValueError('load_domain must be finite, nonnegative, and increasing')
    steady = compile_unitire_simple(load_model(steady_xml))
    if transient_xml is None:
        transient = TransientTireParameters(1., 1., 1., .25)
        transient_source = 'provisional default (not fitted)'
    else:
        transient = compile_unitire_transient(load_model(transient_xml))
        transient_source = f'XML:{Path(transient_xml).name} (provisional unless fit evidence is supplied)'
    return _make_snapshot(steady, transient, 0, {'seed_kind':'repository lab XML','steady_source':str(Path(steady_xml)),'transient_source':transient_source,'transient_fit_status':'provisional'}, {n:'repository lab XML' for n in (*CANONICAL_STEADY_NAMES,*TRANSIENT_NAMES)}, load_domain)

def with_parameters(snapshot: ModelSnapshot, changes: Mapping[str, float]) -> ModelSnapshot:
    """Construct one validated atomic candidate; does not alter simple edit API."""
    if not changes: return snapshot
    values = dict(snapshot.values)
    for name, raw in changes.items():
        if name not in values: raise ValueError(f'unknown tire parameter {name!r}')
        value = float(raw)
        if not np.isfinite(value): raise ValueError(f'{name} must be finite')
        if name == 'trail_lead_ratio' and not 0 <= value <= 1: raise ValueError('trail_lead_ratio must be from zero through one')
        if name in {'longitudinal_length_scale','lateral_length_scale','trail_pole_ratio'} and value <= 0: raise ValueError(f'{name} must be positive')
        values[name] = value
    steady_values = {n: values[n] for n in CANONICAL_STEADY_NAMES}
    steady = unflatten_steady(steady_values, snapshot.steady)
    transient = TransientTireParameters(*(values[n] for n in TRANSIENT_NAMES))
    provenance = dict(snapshot.provenance); statuses = dict(snapshot.parameter_status)
    changed_steady = any(n in CANONICAL_STEADY_NAMES for n in changes)
    if changed_steady:
        provenance['transient_fit_status'] = 'stale'
        statuses.update({n: 'edited' for n in changes})
        statuses.update({n: 'stale' for n in TRANSIENT_NAMES})
    else: statuses.update({n:'edited' for n in changes})
    return _make_snapshot(steady, transient, snapshot.revision+1, provenance, statuses, snapshot.load_domain)

def with_parameter(snapshot: ModelSnapshot, name: str, value: float) -> ModelSnapshot:
    return with_parameters(snapshot, {name:value})

def predict_steady(snapshot: ModelSnapshot, samples: SampleSet, belt_speed: float):
    if not np.isfinite(belt_speed) or belt_speed <= 0: raise ValueError('belt_speed must be finite and positive')
    radius = loaded_radius(snapshot.steady, samples.fz)
    omega = np.abs((1.0 + samples.kappa) * belt_speed / np.maximum(radius, 1e-4))
    return evaluate_unitire(snapshot.steady, samples.alpha, samples.kappa, samples.fz, omega)

def import_mat_seed(path: Path, template: ModelSnapshot) -> ModelSnapshot:
    """Import supported steady coefficients from a MATLAB tire struct; fitReport is optional."""
    try: from scipy.io import loadmat
    except ImportError as exc: raise RuntimeError('MAT import requires scipy') from exc
    mat = loadmat(Path(path), squeeze_me=True, struct_as_record=False)
    if 'tire' not in mat: raise ValueError('MAT seed is missing tire structure')
    tire = mat['tire']; values = dict(template.values)
    scalar_names = {'Fz_rated':'f_z_rated','Kcx':'k_cx','Kcy':'k_cy','E1':'e_1','mu0x':'mu_0_x','musx':'mu_s_x','hx':'h_x','vmx':'v_m_x','mu0y':'mu_0_y','musy':'mu_s_y','hy':'h_y','vmy':'v_m_y','K11':'k_11','K12':'k_12','K13':'k_13','K21':'k_21','K22':'k_22','K23':'k_23','MxR1':'m_x_r_1','MxR2':'m_x_r_2','MxR3':'m_x_r_3','R1':'r_1','R2':'r_2','R3':'r_3','Fy_shift0':'f_y_shift_0','Fy_shiftFz':'f_y_shift_fz','KRl0':'k_rl_0','KRl1':'k_rl_1','KRl2':'k_rl_2','frr0':'f_rr_0','frr1':'f_rr_1','frr2':'f_rr_2','hrr':'h_rr','omega_cr':'omega_cr','omega_eps':'omega_eps','vroll_min':'vroll_min','force_eps':'force_eps','stiff_eps':'stiff_eps','slip_eps':'slip_eps'}
    for source, target in scalar_names.items():
        if not hasattr(tire, source): raise ValueError(f'MAT tire is missing {source}')
        values[target] = float(np.asarray(getattr(tire, source)).reshape(()))
    for source, target, degree in [('Kx','k_x',2),('Ky','k_y',2),('Dx0','d_x_0',1),('De','d_e',1),('D1','d_1',1),('D2','d_2',1)]:
        coefficients = np.asarray(getattr(tire, source), dtype=float).reshape(-1)
        if len(coefficients) != degree+1: raise ValueError(f'MAT {source} must have {degree+1} coefficients')
        for i, coefficient in enumerate(coefficients): values[f'{target}_{degree-i}'] = float(coefficient)
    steady = unflatten_steady({n:values[n] for n in CANONICAL_STEADY_NAMES}, template.steady)
    provenance = dict(template.provenance); provenance.update(seed_kind='requested MATLAB fit', steady_source=str(Path(path)), transient_fit_status='provisional')
    statuses = dict(template.parameter_status); statuses.update({n:'requested MATLAB fit' for n in CANONICAL_STEADY_NAMES})
    return _make_snapshot(steady, template.transient, template.revision+1, provenance, statuses, template.load_domain)
