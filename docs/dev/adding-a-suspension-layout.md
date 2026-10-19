# Adding a suspension layout

A suspension layout is a force network behind the shared kinematics/network seam.
The compiled object is `CompiledSuspension(kinematics, network, layout)` and the
network is evaluated in wheel order **FL, FR, RL, RR**.

## Assembly contract

Register a `suspension-assembly` subtype with `capability="suspension-forces"`.
A nesting assembly declares its child roles in `child_roles`; the compiler resolves
those children and passes a role-to-value mapping to the compile function.

Current layouts are:

| Subtype | Child roles | Meaning |
|---|---|---|
| `independent-corner` | `kinematics`, `spring`, `arb`, `damper-front`, `damper-rear` | Per-corner springs/dampers plus axle ARBs and optional inerter. |
| `axle-roll-heave` | front/rear heave and roll spring/damper roles plus `kinematics` | Stateless per-axle heave/roll modes. |
| `four-mode` | heave, roll, pitch and warp spring/damper roles plus `kinematics` | Stateless four-mode transform. |
| `hydraulic` | `kinematics`, `spring`, `circuit`, `damper-front`, `damper-rear` | Corner springs with a stateful hydraulic circuit. |

A child component should expose a small validated value object. Large circuit or
lookup data belongs in a pinned NPZ; XML carries scalar settings and the SHA-256.

## Force-network interface

Implement `initial_state(z_static)`, `forces(jounce, jounce_rate, state)`,
`static_forces(jounce, state)`, `static_check`, `inertia_block`,
`inertial_forcing`, and `with_kinematics`. Stateless layouts use `n_states = 0`
and return an empty `ds_dt`; stateful layouts return `NetworkForces.ds_dt` in the
same order as their `state_names`.

The evaluator converts the four wheel forces into generalized body and jounce
forces by virtual work. Keep this layer wheel-referred and do not duplicate body
force transformations in a layout.

## Sign and units

* SI units throughout; force is N, jounce is m, velocity is m/s, pressure is Pa,
  hydraulic flow is m³/s and fluid area is m².
* Jounce is positive in compression. Corner force is positive in compression.
* Positive damper velocity is compression.
* A cylinder area is positive when compression delivers fluid into its node. A
  rod-side chamber can therefore use a negative signed area.
* For independent corners, damper tables see shaft velocity `jounce_rate / MR`.
  For modal layouts, tables see wheel-referred mode velocity with **no** motion
  ratio. This is a deliberate convention and changes table lookup versus the
  independent layout.
* The hydraulic layout stores one gas fraction per accumulator. Orifice lines are
  algebraic; they have no integrated flow state or line volume.

## Modal and hydraulic limitations

Modal decoupling is exact only for linear elements. The shipped modal examples use
linear mode springs and therefore their per-axle ride frequencies and corner
 damping ratios are diagonal approximations. A nonlinear bump stop or other
 nonlinear mode element would not remain exactly decoupled.

The hydraulic example is a sizing example, not a fit to measured hydraulic data.
Its circuit graph and proposed values are pinned in `models/data/`; a future layout
can reuse the loader with a different graph and accumulator set.

## Verification checklist

1. Add focused force, static-load, state-domain and reduction-oracle tests.
2. Check zero-jounce static forces against `kinematics.static_sprung_load`.
3. Run equilibrium and a short forward integration with zero domain counters.
4. Add the layout to setup/report tools only through network metadata, not hard-coded
   `.front`, `.rear` or `.dampers` reaches.
5. Run the full suite and `git diff --check`.
