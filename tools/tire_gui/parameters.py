"""Public canonical mapping for flattened UniTire XML parameters."""
from dataclasses import dataclass, fields

from simplified_vehicle_models.parameters.tire import UniTireSimpleParameters

VECTOR_DEGREES = {'k_x': (2, 1, 0), 'k_y': (2, 1, 0), 'd_x_0': (1, 0), 'd_e': (1, 0), 'd_1': (1, 0), 'd_2': (1, 0)}
VECTOR_FIELDS = {f'{name}_{degree}': (name, index) for name, degrees in VECTOR_DEGREES.items() for index, degree in enumerate(degrees)}
TRANSIENT_NAMES = ('longitudinal_length_scale', 'lateral_length_scale', 'trail_pole_ratio', 'trail_lead_ratio')
STEADY_SCALARS = tuple(f.name for f in fields(UniTireSimpleParameters) if f.name not in {*VECTOR_DEGREES, 'effective_radius'})
CANONICAL_STEADY_NAMES = (*STEADY_SCALARS, *VECTOR_FIELDS)

@dataclass(frozen=True)
class ParameterSpec:
    name: str
    unit: str
    lower: float
    upper: float
    group: str
    default_locked: bool

UNITS = {
    'f_z_rated':'N','k_cx':'N/m','k_cy':'N/m','k_x':'N','k_y':'N','d_x_0':'m','d_e':'m','d_1':'1','d_2':'1',
    'e_1':'1','mu_0_x':'1','mu_s_x':'1','h_x':'1','v_m_x':'m/s','mu_0_y':'1','mu_s_y':'1','h_y':'1','v_m_y':'m/s',
    'k_11':'N*m','k_12':'N*m','k_13':'N*m','k_21':'N*m','k_22':'N*m','k_23':'N*m','m_x_r_1':'N*m','m_x_r_2':'N*m','m_x_r_3':'N*m',
    'r_1':'m','r_2':'m','r_3':'m','f_y_shift_0':'N','f_y_shift_fz':'N','k_rl_0':'m/N^2','k_rl_1':'m/N^2','k_rl_2':'m/N^2',
    'f_rr_0':'1','f_rr_1':'1','f_rr_2':'1','h_rr':'1','omega_cr':'rad/s','omega_eps':'rad/s','vroll_min':'m/s','force_eps':'N','stiff_eps':'1','slip_eps':'1',
    **{n:'1' for n in TRANSIENT_NAMES}}

def flatten_steady(parameters):
    values = {name: float(getattr(parameters, name)) for name in STEADY_SCALARS}
    for flat, (vector, index) in VECTOR_FIELDS.items(): values[flat] = float(getattr(parameters, vector)[index])
    return values

def unflatten_steady(values, template):
    kwargs = {name: float(values[name]) for name in STEADY_SCALARS}
    for vector, degrees in VECTOR_DEGREES.items(): kwargs[vector] = tuple(float(values[f'{vector}_{d}']) for d in degrees)
    kwargs['effective_radius'] = template.effective_radius
    return type(template)(**kwargs)

def specs():
    result = []
    for name in (*CANONICAL_STEADY_NAMES, *TRANSIENT_NAMES):
        group = name.split('_')[0]
        locked = name in {'k_cx','k_cy','f_z_rated','r_1','r_2','r_3','force_eps','stiff_eps','slip_eps','omega_cr','omega_eps','vroll_min','k_11','k_12','k_13','k_21','k_22','k_23','longitudinal_length_scale'}
        result.append(ParameterSpec(name, UNITS[name], float('-inf'), float('inf'), group, locked))
    return tuple(result)
