"""Public canonical mapping for flattened UniTire XML parameters."""
from dataclasses import dataclass, fields

from simplified_vehicle_models.parameters.tire import UniTireSimpleParameters

VECTOR_DEGREES = {'k_x': (2, 1, 0), 'k_y': (2, 1, 0), 'd_x_0': (1, 0), 'd_e': (1, 0), 'd_1': (1, 0), 'd_2': (1, 0)}
VECTOR_FIELDS = {f'{name}_{degree}': (name, index) for name, degrees in VECTOR_DEGREES.items() for index, degree in enumerate(degrees)}
TRANSIENT_NAMES = ('longitudinal_length_scale', 'lateral_length_scale', 'trail_pole_ratio', 'trail_lead_ratio')
STEADY_SCALARS = tuple(f.name for f in fields(UniTireSimpleParameters) if f.name not in {*VECTOR_DEGREES, 'effective_radius'})
CANONICAL_STEADY_NAMES = (*STEADY_SCALARS, *VECTOR_FIELDS)

# MATLAB source struct fields mapped to the canonical XML/parameter API. These
# tables are intentionally independent from exporter-private implementation data.
MAT_SCALAR_FIELDS = {
    'Fz_rated':'f_z_rated','Kcx':'k_cx','Kcy':'k_cy','E1':'e_1','mu0x':'mu_0_x','musx':'mu_s_x','hx':'h_x','vmx':'v_m_x',
    'mu0y':'mu_0_y','musy':'mu_s_y','hy':'h_y','vmy':'v_m_y','K11':'k_11','K12':'k_12','K13':'k_13','K21':'k_21','K22':'k_22','K23':'k_23',
    'MxR1':'m_x_r_1','MxR2':'m_x_r_2','MxR3':'m_x_r_3','R1':'r_1','R2':'r_2','R3':'r_3','Fy_shift0':'f_y_shift_0','Fy_shiftFz':'f_y_shift_fz',
    'KRl0':'k_rl_0','KRl1':'k_rl_1','KRl2':'k_rl_2','frr0':'f_rr_0','frr1':'f_rr_1','frr2':'f_rr_2','hrr':'h_rr','omega_cr':'omega_cr',
    'omega_eps':'omega_eps','vroll_min':'vroll_min','force_eps':'force_eps','stiff_eps':'stiff_eps','slip_eps':'slip_eps'}
MAT_VECTOR_FIELDS = {'Kx':'k_x','Ky':'k_y','Dx0':'d_x_0','De':'d_e','D1':'d_1','D2':'d_2'}

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
