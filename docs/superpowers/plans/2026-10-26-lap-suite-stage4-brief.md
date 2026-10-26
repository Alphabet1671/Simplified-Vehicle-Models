# Stage4 brief: model-aware yaw tracking and replay metrics

Implement after earlier profile/line stages validated. Prefer new solvers/yaw_tracking_driver.py to preserve PreviewDriver's public default behavior and older tests. New class can subclass PreviewDriver with appended immutable fields or compose it while preserving command/pedal protocol.

No hidden speed cap or force-full-throttle policy. Desired acceleration=profile feedforward+speed feedback. Model-aware actuator inversion: required longitudinal force=mass*acceleration+aero_drag(speed); positive force maps through assembled motor force limit (rear topology shares from existing point_mass helper, initial SOC explicit if observation lacks SOC), negative force invert actual brake_torques pedal curve/bias/hydraulic pressure/deadband by bisection. Tyre slip/actuator limits still handled by plant; static feedforward not claim transient exactness. Brake governor remains, no simultaneous throttle/brake. Cache static force mapping across speed grid only with disclosed interpolation/error; no repeated expensive compiler operations in command.

Lateral controller needs evidence: preview path curvature, course error heading+atan2(vy,vx), speed-dependent lookahead, bounded steering/yaw damping. Keep physically consistent sign and saturations. Diagnose prior near-seam excursion from recorded error/curvature; test seam continuity (SpeedPlan wraps), straight error correction, mirrored turns, actuator saturation and finite outputs. Do not tune solely one track then claim generality.

Compute time-weighted metrics using actual sample times: speed error RMS/max, lateral RMS/max, heading/course error, throttle fraction>=.95, brake usage, steering/rate saturation, sampled centre corridor violation, lap completion. Footprint and between-step containment not implied. Publish controller settings/provenance in record.

Suite replay loads validated persisted plan rather than repeating line optimization. Baseline and new controller replay same geometry/profile/initial conditions, compare metrics. Each run outer process watchdog<=1800s and total replay wall budget<1500s. Use at least synthetic controller tests plus MIS/FSC short/flying-lap evidence if possible. If uncompleted report cause/trace, not success.

No OCP/physical map edits; existing PreviewDriver callers retain behavior. Report docs/superpowers/plans/2026-10-26-lap-suite-stage4-report.md. No subagents.
