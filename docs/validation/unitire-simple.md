# Simplified steady-state UniTire validation

## Scope

This report covers the Python port of `unitire_simple_solve.m` and the export
of the fitted `43075 R20` parameter set. It covers no other model.

The port targets Model 2, the planar rigid vehicle. It supplies the
`steady-combined-slip` capability. The friction ellipse supplies only
`tire-force-envelope`, and the compiler refuses to derive the steady
capability from the envelope.

## Sources

| Role | File | SHA-256 |
|---|---|---|
| Solver | `~/Dev/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model/unitire_simple_solve.m` | `6959fce78741a3bde7189b9dbfa20c881a6304216f35c2e9400f1ecf09e15f1e` |
| Parameters | `~/Dev/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model/Hoosier_R20_Combined_Simple.mat` | `ed46ac5c5eb19ceeca359a10a5c4b279b26e468dec00bec229df3033848f1500` |

The Python artifact is `models/tires/hoosier-43075-r20-simple.xml`. It carries
both hashes in its header comments. `tools/export_unitire_simple_tire.py`
regenerates it, so the export is reproducible and not hand copied.

The fitted source is the `43075 R20` tire at 12 psi on an 8 inch rim, fitted
against TTC `B2356` runs 8 and 72, as the MATLAB fit script records. The
comparison script `TireModelComparison.m` selects this same parameter file as
its simplified UniTire source. The other file `unitire_simple_fit.mat` is a
different fit and is not used.

## Reference vector method

Reference vectors come from the reviewed MATLAB function itself, not from a
second Python implementation and not from the fit data. MATLAB R2024b ran the
source function over 29 operating points:

- a pure lateral sweep from `-0.3` to `+0.3` rad;
- a pure longitudinal sweep from `-0.2` to `+0.2` slip ratio;
- four combined-slip points;
- a load sweep from `0` to `1500` N, including zero load;
- a hub-speed sweep from `0.05` to `200` rad/s, including below the floor.

The fixture is `tests/fixtures/hoosier_43075_r20_reference.csv`. It records the
source hashes and the MATLAB release. Values use `%.17g`, so each value round
trips through a float64.

## Agreement

The test compares 17 output columns across all 29 rows. The comparison covers
the forces, the three moments, both radii, both friction values, the pneumatic
trail, the relaxation length, and the source intermediates `sx`, `sy`, `phi`,
`f_bar` and `lambda`. The intermediates are compared so that an error cannot
cancel between them and the final forces.

Measured agreement:

| Quantity | Worst case |
|---|---|
| Absolute difference | `4.5e-13` on the lateral force, whose scale is about `2400` N |
| Relative difference | `1.6e-15` on the aligning moment |

Several columns agree exactly. The test asserts `rtol=1e-12` and `atol=1e-11`,
which leaves a large margin over the measured difference while still failing on
any change of behavior.

## Axis and sign convention

The vehicle frame is SAE J670: `+x` forward, `+y` right, `+z` down. The source
tire function uses the same lateral and vertical sense. The tire boundary is
therefore an identity in the lateral and vertical directions, and the port
needs no lateral sign conversion.

Verified behavior:

- A positive slip angle gives a positive lateral force, which acts to the right.
- A positive slip angle gives a negative aligning moment, as the source states.
- A positive slip ratio gives a positive longitudinal force.
- Negating the slip angle negates the lateral force and the aligning moment, and
  leaves the longitudinal force unchanged.

## Parameter units

The MATLAB file carries no unit table. The XML declares a unit for every
parameter, derived from the structure of the source:

| Group | Unit | Derivation |
|---|---|---|
| `k_x`, `k_y` | N | `phi = K*S/F` is dimensionless, so the stiffness carries force units |
| `k_cx`, `k_cy` | N/m | `Xc = Fx/Kcx` is a length |
| `d_x_0`, `d_e`, `r_1`, `r_2`, `r_3` | m | lengths in the source |
| `v_m_x`, `v_m_y`, `vroll_min`, `omega_cr`, `omega_eps` | m/s or rad/s | rates and speeds |
| `f_z_rated` | N | the load normalization anchor |
| `force_eps` | N | a force floor |
| `mu_0_*`, `mu_s_*`, `h_*`, `e_1`, `d_1`, `d_2`, `f_rr_*`, `h_rr` | 1 | dimensionless factors |

Two limits are recorded rather than hidden:

1. The source `Mx2` overturning term mixes a linear term with a cubed term, so
   it is dimensionally inhomogeneous. The six `k_1*` and `k_2*` coefficients and
   the three `m_x_r_*` coefficients are zero in this fit, so the term vanishes.
   The XML declares the linear-term unit and records this limit.
2. `stiff_eps` is one numerical floor that the source applies to stiffness terms
   with different units. It is dimensionless in the source, and the XML records
   that.

Neither limit affects the fitted behavior, because both parameter groups are
zero. Both must be resolved before a fit that uses them is accepted.

## Operating domain

- Zero load returns zero forces and zero moments without a NaN. The friction
  law still evaluates, because the source divides by a force floor.
- A hub speed below the rolling-speed floor uses the floor. The source clamps
  `Vroll` to `vroll_min`, which is `0.1` m/s in this fit.
- The friction law reaches the peak value at zero slip speed and the sliding
  value at large slip speed.
- The model has no inclination angle, no camber, and no relaxation state. The
  returned relaxation length is a diagnostic only.

## Not covered

- The fit accuracy against the TTC measurements. This report checks the port
  against the fitted model. It does not re-derive the fit, and it does not claim
  an agreement with the raw tire test data.
- The upstream `43075` provenance of the TTC runs. The fit script and the
  comparison script both name the tire as `43075 R20`, and this report repeats
  that name; it does not independently confirm the run identity.
- Camber, inclination, transient relaxation, and combined-slip behavior outside
  the fitted load range.
- Any vehicle-level result. The tire is not yet connected to a vehicle model.

## Reproduction

```bash
PYTHONPATH=src python -m unittest tests.test_unitire_simple -v
```

Regenerate the parameter artifact after a refit:

```bash
python3 tools/export_unitire_simple_tire.py \
  --output models/tires/hoosier-43075-r20-simple.xml
```

Regenerate the reference fixture with the committed MATLAB driver:

```matlab
addpath('tools');
generate_unitire_reference( ...
    'tests/fixtures/hoosier_43075_r20_reference.csv', ...
    '/path/to/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model');
```

The committed driver reproduces the committed fixture byte for byte. The
comparison ignores the fixture comment lines and compares the remaining 30
lines.
