# Caster/steering-axis jacking in the simplified 14DoF model — investigation

## Recommendation

Implement steering-induced wheel-center vertical motion as a **kinematic steering-axis coupling** in the front wheel/jounce kinematics, then let the existing tire, suspension and 7-DOF vertical equations produce the load transfer. Do not add a hand-tuned vertical force beside the existing roll-centre jacking force. Preserve the current 14DoF coordinates and use virtual work so force signs, action/reaction and energy are consistent.

“Caster jacking” is used ambiguously. There are three effects to keep separate:

1. **Steering-axis geometric lift/drop:** rotating a wheel around a tilted steering axis moves its center/contact assembly vertically. The front-view kingpin/steering-axis inclination (KPI/SAI) contributes, as can side-view caster, depending on steering-axis ground-intercept offsets and the point whose height is tracked. For a fixed steering axis and wheel-center offset, a first-order geometric curve is `h(δ)=b sinδ+a(1−cosδ)`, where signed geometry lengths `a,b` must be derived from the actual 3-D steering-axis sweep. An even-only curve follows only when `b=0`.
2. **Force-induced steering-axis moments:** tire forces and aligning moment create steer torque, approximately through mechanical trail, pneumatic trail and scrub/offset arms. These can influence vertical loads only through compliant/kinematic coupling (or a steering DOF and its constraints); they are not themselves a vertical force.
3. **Suspension roll-centre jacking:** already modeled in `vertical_14dof.py` from lateral link force and roll-centre height. It is a different mechanism and should not be counted twice.

A common back-of-envelope decomposition is `h(δ) ≈ t_C sin(κ)(1 − cos δ) + r_S sin(σ)(1 − cos δ)` where `t_C` is mechanical caster trail, `κ` is caster, `r_S` is signed scrub offset, and `σ` is SAI/KPI. This even-only decomposition is *not generally exact*: rotating an offset wheel center around a tilted axis also produces a `sinδ` component, with a sign that can reverse between mirrored corners. The current steering command is positive-right and parallel at both front wheels (Ackermann offsets are zero), but same signed steer does **not** establish same-signed lift unless the actual wheel-center offset geometry does. Do not infer even parity or zero roll from caster/KPI angles alone. Determine both terms from the steering-axis sweep.

A distinct force-induced approximation sometimes discussed is a vertical force proportional to lateral tire force and steering-axis inclination (e.g. `F_y tan κ` and/or `F_y tan σ`), with convention-specific signs. Treat it separately from geometric lift, and do not add both a derived vertical force and the same geometry-induced displacement in the compliance path. Use only with explicit force-path derivation and validation against the steering-axis kinematics.

## Existing code path

`evaluate_vertical_dynamics` in `src/simplified_vehicle_models/formulations/vertical_14dof.py` has the right mechanics framework:

- Body corner vertical position is `heave + y*roll − x*pitch`; hub-down position is that body position minus wheel jounce (jounce positive in compression).
- Tire vertical deflection/load is computed from hub motion.
- Steered wheel angles are commanded steering plus suspension toe, and those same angles feed contact kinematics, tire slip/forces and body-force rotation.
- The model already assembles suspension, roll-centre jacking, anti-dive/squat and unsprung load transfer into body/jounce generalized forces, then solves the coupled heave/roll/pitch/jounce mass block.

Vehicle parameters already store `caster_angle` and `kingpin_inclination_angle`, but grep found them only in the XML/compiler/parameter type; they currently have no dynamics effect. The current values are 0.08135 rad caster and 0.12448 rad KPI; they are not enough to determine lift because trail/offset and exact axis conventions are missing. The corner suspension geometry has no steering-axis sweep, scrub radius, mechanical trail, or jounce-steer data (the current jounce-steer curve is an explicit zero baseline). Existing roll-centre jacking is an internal force pair: its body and hub effects are deliberately balanced, while hub-only unsprung terms act on jounce rows.

## Recommended equations and implementation shape

Preferred interface: augment the front suspension/steering kinematics with a per-wheel steering-axis vertical displacement function `h_i(δ_i, jounce_i)` and its derivatives. It must represent **wheel-center/hub displacement relative to chassis**, not tire compression or a guessed vertical force. Use actual 3-D hardpoint geometry if available; calculate finite steer sweeps at fixed jounce and fit a smooth, bounded function over the permitted steer range. Preserve left/right mirror symmetry using their own signed local geometry and actual road-wheel angles (Ackermann can make them unequal). A generic tilted steering-axis rotation can also produce an **odd** lift/drop component: for steering-axis unit vector `u` and wheel-center offset `r`, Rodrigues rotation gives `h(δ) = (u × r)_z sinδ + [u_z(u·r) − r_z](1−cosδ)`. The even-only approximation is justified only when the measured odd coefficient vanishes. Mirror signs and actual steering-angle conventions must be tested with the 3-D sweep.

There is a second plausible implementation if the intended effect is the *force path* through a side-view caster axis rather than steering-induced kinematic lift: model the body/hub vertical component of the longitudinal steering-axis/link force as an equal-and-opposite internal pair, in the same force-pair architecture as anti-dive and roll-centre jacking. In the repo's sign convention, `body_vertical_corner` is upward force on the body and `hub_force` downward force on the hub, so a derived signed `D_c` enters both arrays equally (body heave/roll/pitch cancellation, jounce reaction retained). For a specific side-view convention with `x` forward, `z` up, and positive caster defined as the steering-axis top rearward, a preliminary derivation gives the body-up component `C_i = −tan(caster) * F_x` using the front tire longitudinal force in the steering-axis frame: traction (`F_x > 0`) gives body down, while braking (`F_x < 0`) gives body lift/anti-dive. In the evaluator's sign convention, put this signed `C_i` in both `body_vertical_corner` and `hub_force`, as for the other internal vertical pairs; it cancels from body heave/roll/pitch generalized rows and remains in the hub/jounce balance. This is not a settled vehicle-specific result until the car's caster sign and force axis are confirmed. Using rotated body longitudinal force `bfx` is only a side-view approximation at steer; using `link_longitudinal` requires a derivation that avoids double-counting unsprung inertia. The term is **not** interchangeable with geometric lift, is not a complete trail/scrub model, and must not be combined with geometric displacement without proving distinct effects. Choose a formulation from an explicit physical free-body/kinematic derivation.

With hub-down coordinate `z_h = z_body − jounce`, a positive-up kinematic lift `h_i` gives `z_h = z_body − jounce − h_i(δ_i)`. Thus hub-down rate includes `−h_δ δdot` when steering is dynamic. For quasi-static prescribed steering, displacement alone suffices; no steering-rate term is needed. Dynamic use also needs `h'' δdot²+h' δddot` as a known unsprung acceleration term. The key requirement is to apply the kinematic constraint consistently to the virtual-work/Jacobian formulation: the existing generalized hub velocity coefficient is `c_i=[1, y_i, −x_i, −1]`. Extend the point velocity/Jacobian with the steer-dependent term and derive generalized forces from that Jacobian, rather than patching only heave or jounce force rows. If using a compliant tire/suspension model, the wheel force that results from the displacement then naturally redistributes heave, roll and individual loads.

A simple quasi-static test can prescribe `δ` and solve the vertical equilibrium with `h_i(δ)` included in hub position. This captures suspension/tire compliance and load redistribution with the present DOFs; do not simultaneously apply an equivalent `F_z ≈ k_eff h` force. The transient forward solver supplies steering rate but not steering acceleration; it now rejects nonzero-rate commands if nonzero steering-lift coefficients are enabled rather than silently omitting the required `h' δddot` inertial term. Fixed steering with calibrated lift remains supported; extending forward commands to provide a smooth steering acceleration is needed before steer-ramp runs. If steering torque compliance is a later goal, it requires steering DOF(s), steering stiffness/damping/ratio and a torque balance; this is a separate scope.

### Minimal reduced-order fallback

The implemented optional curve accepts two signed coefficients per front corner, `h_i(δ)=b_i sinδ+a_i(1−cosδ)`, with `h_i(0)=0`. In the suspension-kinematics XML the optional meter-unit parameters are `steering_lift_sine_left_front`, `steering_lift_cosine_left_front`, `steering_lift_sine_right_front`, and `steering_lift_cosine_right_front`; omit them for zero lift. `b` captures lift/drop odd in steering angle and `a` captures the even component. They default to zero when absent, which keeps legacy models byte-identical. The IEM24 front corners carry values derived from the steering-axis hardpoints (from real car data) by `tools/model_data/derive_steering_lift.py`; the rear carries none. Calibrate/sign the values from measured or derived wheel-center lift versus steer, not by tuning a vertical-force gain against lap time. The sine and cosine coefficients may capture both caster and KPI geometric effects, but only when empirically or geometrically defined; do not infer exact coefficients from the current caster/KPI values alone. Rear wheels default to zero unless they actually steer.

## Vehicle-specific information still needed

Required before a physically interpretable implementation:

- Steering-axis direction in vehicle coordinates for each front corner, not just scalar caster/KPI angles (explicit signs and reference convention).
- Steering-axis ground intersection/offset relative to tire contact patch and wheel center at nominal ride height: mechanical trail and scrub radius, plus wheel radius/contact geometry.
- Steering arms/Ackermann road-wheel angle convention and steering range.
- How wheel-center height varies with steering angle at fixed suspension position; preferably CAD hardpoint sweep or measured wheel-center/ride-height change, at multiple jounce positions if practical.
- Whether steering has compliance/play and whether the input to this model is rack angle, mean road-wheel angle or individual road-wheel angles.

The current vehicle has caster `0.08135 rad` (4.661 deg), KPI `0.12448 rad` (7.132 deg), steer ratio 5.3962, and maximum road-wheel angle 0.5236 rad (30 deg); the steering table is parallel (zero Ackermann offsets), and positive road-wheel angle is right turn. These angles are not enough to determine lift because required offsets and exact axis conventions are missing. At 30 deg, `1−cosδ≈0.134`; that gives only a dimensionless multiplier, not a lift without a justified effective geometry length. Do not assume a projection length or add angles as force coefficients. Scrub radius and mechanical caster trail are not separately represented in the current model.

A force-pair implementation additionally needs an unambiguous side-view free-body definition: which longitudinal force is transmitted along the steering axis, whether it is tire-frame or body-frame, and how unsprung inertia is partitioned. One preliminary derivation suggests `D_c ≈ −tan(caster) H_x` under a particular positive-caster convention, but it must be re-derived against the model's own axis orientation and checked with front braking/drive before implementation. It cannot be treated as the same effect as the steering-angle-dependent geometric lift.

## Validation plan

1. **Geometry unit tests:** zero steer gives exactly zero added height; mirrored geometry obeys CAD-derived sine/cosine symmetry; sweep matches source hardpoints across steer/jounce bounds; derivatives are continuous and finite. Confirm whether lift reverses across left/right and ±steer from the actual axis offsets.
2. **Virtual-work tests:** finite-difference displacement Jacobian matches analytic Jacobian; generalized work `Q·qdot` equals contact/hub work; no hidden energy source for a static steering angle. Check action/reaction and signs for positive/negative steer, caster, KPI, and each side independently.
3. **Zero-effect regression:** with coefficient/curve zero, all evaluator outputs match current baseline exactly; existing equilibrium and acceptance suites remain unchanged.
4. **Static steering sweep:** solve 14DoF vertical equilibrium at zero speed for ±steer positions. Report four tire loads, heave, roll, pitch and jounce vs steer; total vertical load remains conserved (no gravity change), and mirror symmetry holds. Compare to CAD or measured wheel scales/ride heights.
5. **Dynamic steering sweep:** apply slow steering ramps and reversals at zero speed to isolate geometric jacking, then repeat with lateral tire forces. Check no artificial jounce-rate spikes, stable solver behavior and load recovery. Separate caster-axis geometry from roll-centre and unsprung inertial jacking in tests.
6. **Sensitivity/uncertainty:** bound the result over uncertainty in trail, scrub and axis angles; document whether effect is material at the car's steering range before adding it to accepted baselines.

## External references

These are authoritative textbook/catalog references for vehicle dynamics and suspension/steering geometry, not direct validation of the reduced equation. Exact steering-axis kinematics should be derived from the car's geometry.

- Thomas D. Gillespie, *Fundamentals of Vehicle Dynamics*, SAE R-114: <https://www.sae.org/publications/books/content/r-114/>
- William F. Milliken and Douglas L. Milliken, *Race Car Vehicle Dynamics*, SAE R-456: <https://www.sae.org/publications/books/content/r-456/>
- Hans B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed.: <https://www.sciencedirect.com/book/9780080970165/tyre-and-vehicle-dynamics>
- SAE technical paper search for primary research on caster/jacking and steering geometry: <https://saemobilus.sae.org/search/?query=caster%20jacking>
- Secondary terminology checks: <https://en.wikipedia.org/wiki/Caster_angle> and <https://en.wikipedia.org/wiki/Scrub_radius> (not used to establish a vehicle-specific equation).

## IEM24 front values (from real car data)

The hardpoints are confirmed by scans and are treated as accurate. Rotating the upright about the upper-to-lower outer ball-joint line and tracking the wheel centre gives, over ±30° of road-wheel steer, a two-term fit with a worst residual of about 3 µm:

| Corner | `b` (sine) [mm] | `a` (cosine) [mm] |
|---|---|---|
| Front left | +1.527 | −6.560 |
| Front right | −1.527 | −6.560 |

The axis has ~4.5° caster and ~7.5° KPI, matching the tabulated caster and KPI. The ground-plane intercept of the axis reproduces the tabulated mechanical trail (0.1 in) and scrub radius (0.6 in), which cross-checks the points. The resulting wheel-centre height change is small, at most about 1.6 mm at full lock, and it is mostly a drop of the wheel centre on the inside wheel (right-front in a right turn, −1.6 mm at full lock; left-front is within ±0.2 mm). The sine term makes the two fronts differ, so it acts as a small roll/diagonal input. Lift moves only the tire deflection; no extra force is added. At the tire stiffness this raises the inside-front load by about 130 N at full lock, 74 N at 20° and 29 N at 10°, with the opposite sign on the other front wheel at small angles (−13 N at 10°).

Limitations:

- Only the steering-axis rotation of the wheel centre at design ride height is modelled. Jounce dependence, compliance and the sub-millimetre camber-induced contact-patch shift are neglected.
- Separate bench measurements of corner weight versus steering angle exist for an unknown car and possibly a previous-year car. They are not used to fit or validate these values, because they need not describe this car. They remain a qualitative check only.
- The forward plant supplies steering rate but not steering acceleration. The omitted inertial term `m_u h' δ̈` is below about 1 N at these magnitudes.

## Bump steer (from real car data)

The jounce-steer curves are derived from the same hardpoints with `tools/model_data/derive_bump_steer.py` (see `docs/model-data/provenance.md`). Front toe-in stays within -0.205° to +0.005° and rear within -0.103° to +0.026° over the full travel, so bump steer is a second-order effect next to the steering-lift term above.
