# Shared forward-driver compatibility decision

## Decision

The preview-driver architecture is compatible with all three higher-fidelity models in the [master plan](<../../project-master-plan.md>), provided the driver observes named physical signals and the plant owns its complete state and implicit force equations. Proceed with an opt-in transient planar prototype. This is architectural compatibility, not implemented or benchmarked support for the two larger plants.

The current [registry](<../../../src/simplified_vehicle_models/compiler/registry.py>) implements only point-mass and planar formulations. Body 10DoF and full 14DoF remain future milestones. Do not invent those plants or count a synthetic integration test as their physical validation.

| Plant | Complete dynamic content | Required numerical contract | Driver reuse |
|---|---|---|---|
| Planar 7DoF | Body longitudinal/lateral velocity, yaw rate, four wheel speeds; pose/progress and energy states in addition | Local quasi-static load/force consistency, wheel inertia retained | Same prescribed-path preview and speed feedback |
| Body 10DoF | Six body DOF and four wheel speeds; massless suspension with spring, damping and inertance | State-dependent effective mass matrix or implicit residual; consistent static initialization | Same observations/commands; separately tuned gains and speed profile |
| Full 14DoF | Six body DOF, four unsprung vertical DOF, four wheel speeds, at least eight tire internal states | Coupled mass matrix, stiff relaxation, contact events and tire state reset policy | Same observations/commands; plant owns camber, contact and all tire states |

References: master-plan sections 3.2–3.4, 8, 9.1, 10, 14.2–14.4. The full-model state is about 32 entries before controls/energy; mechanical DOF must not be used as array length. No driver code may truncate plant states to seven entries.

## Scope boundaries

- All simulations follow the supplied XY path. No racing-line or target-line optimization: explicitly outside master-plan scope.
- Backward propagation applies only to a speed envelope. Every dynamic state advances forward in time. Never splice segments by speed alone.
- The plant interface exposes an implicit rate system `M(t,y,u) * ydot = f(t,y,u)`; identity mass is optional. The solver handles nonsingular mass systems numerically, not by manually reducing every model equation. Algebraic closure remains a checked plant operation.
- A singular DAE/contact formulation needs a dedicated consistent-initialization and DAE/event backend. The initial prototype must reject unsupported/singular systems, not silently pseudoinvert them. This does not prevent future integration through the same driver signals.
- The shared driver consumes progress, lateral/heading error, longitudinal/lateral speed, yaw rate and steering state. Course-error feedback includes body sideslip so the driver does not force zero body heading error during a turn. It commands normalized throttle, physical brake-pedal force and steering rate. Plants translate commands through their actual drivetrain and actuator physics.
- Rate-limited steering is a dynamic state. Future controller filters/integrals and tire states belong in explicit layouts and periodicity metadata; no hidden controller memory during integrator evaluations.
- Use the existing NumPy planar tire/load physics. Keep four wheel-speed states, battery SOC and physical pedal mapping; no tire-force clipping, hidden ABS, or traction controller.
- A conservative kinematic speed envelope is permitted for the first prototype only. It is a requested speed plan, not a demonstrated live tire capability map or minimum-time result. Forward validation can reject it. Production planning should reuse the point-mass live-limit solver with recorded adapter provenance.
- Prototype results distinguish completion, tracking/physical checks, periodicity, resolution, and runtime. A finite initial-value lap is not automatically a periodic hot lap. Old OCP usability rules and historical results remain untouched.

## Wheel inertia review

The user-authorized single-agent workflow requested provider `openai-codex`, model `gpt-6-luna`. Its returned investigation confirmed the current OCP removes wheel inertia, whereas the NumPy evaluator retains it. The XML currently used a 12 kg estimate. Apply the requested solid-cylinder assumption instead:

`J = 0.5 * 10 * (16 * 0.0254 / 2)**2 = 0.2064512 kg*m^2`.

Width is 8 inches but does not enter axial inertia for fixed total mass. This is a declared modeling assumption, not a measured inertia or a claim that all real unsprung mass rotates. Existing archived runs predate this correction and must not be described as same-parameter comparisons.

The coupled RHS evaluates slip from the current wheel speed and body contact velocity, solves loads/tire forces, and returns both body and wheel accelerations. The integrator advances all states together. Wheel spin follows `(net wheel torque - effective radius * longitudinal force + signed rolling moment) / J`. Do not update wheel speed outside the integrator or substitute torque balance.

## First experiment and acceptance

Build and test a generic mass-system driver harness; demonstrate that additional inertance/relaxation-like states survive integration in synthetic contract tests. These are not 10DoF/14DoF vehicle implementations. Then integrate the actual planar model on a bounded maneuver and the actual track under a 300 s parent cap. Preserve failures and timing diagnostics. Check local convergence, path error, wheel speeds, actuator bounds, electrical power and resolution. Report periodicity and accuracy refinement as separate unproven gates until measured. Do not promise sub-five-minute valid hot-lap performance from architecture alone.

Implementation is additive on the existing feature branch. Preserve unrelated working-tree modifications, measured telemetry, previous OCP code and artifacts. No commits are requested.
