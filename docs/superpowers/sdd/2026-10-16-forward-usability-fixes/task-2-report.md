# Task 2 report: vertical dense state-domain certificate

Implemented the 14DoF vertical dense-domain certificate and model-aware runner/gate integration.

## Scope

The certificate checks only finite interpolant-wide state-domain inequalities: SOC bounds, forward speed > 0.25 m/s, wheel speeds >= 0, steering physical bound, each corner's physical suspension jounce travel, curvature path denominator >= 0.5, and path progress > 0.25. Curvature and trigonometric interval calculations reuse `planar_dense_certificate` helpers.

The result explicitly states that it is not a full between-step certificate and does not validate continuous tables, dampers, tires, loads, powertrain closure, or the true ODE trajectory. Pedal overshoot is diagnostic only when the augmented 39-state layout is supplied.

## Integration

The vertical runner writes the dense-domain result for `14dof-simple`; provenance now includes the new certificate module and dense-domain helper. The gate dispatches from `summary.model`, reconstructs the track plan from recorded settings, reloads dense bounds, and independently recomputes the vertical state-domain result (it does not trust the stored summary flag). It never imports planar load-closure/audit code for vertical runs, and leaves `between_step_validation_complete` false because the available vertical evidence is not a full continuous certificate.

No canonical laps or driver-tuning evidence were run.
