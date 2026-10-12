# P8 point-mass integration report

## Dispatch

The point-mass formulation now consumes the compiled `drive-force-limit` capability. A
`FullDrivetrainRuntime` dispatches through `maximum_drive_force`, including the motor
map, inverter, storage voltage, and transmission coupling. A plain
`DrivetrainParameters` retains the original scalar torque/power reduction for fast
legacy runs and MIS compatibility.

The motor-map interface uses rpm. Transmission wheel/motor speeds are angular speeds
in rad/s, so assembled drive and regen paths convert with `rpm = rad/s * 60/(2*pi)`.
The scalar speed-limit check applies the same conversion.

Coupled drive voltage updates are explicitly bounded to two iterations and the torque
bracket is bounded to twelve bisection steps. This prevents repeated point-mass
bracketing from becoming an unbounded nested solve; returned diagnostics still expose
convergence and residual information.

## Focused coverage

- assembled vehicle compilation produces `FullDrivetrainRuntime`;
- assembled point-mass drive dispatch agrees with `maximum_drive_force`;
- assembled regen reports torque and charge-power limits through
  `maximum_regen_force`;
- legacy scalar dispatch remains selectable and uses the original force reduction;
- rad/s-to-rpm operating-point conversion is covered by the powertrain tests.

## Concerns

The full assembled path is more expensive than the legacy scalar path because it
solves the voltage/storage coupling. It is bounded, but MIS and cross-fidelity lap
runs remain materially slower than legacy-only runs. P9 should own state-of-charge
iteration rather than adding another hidden loop in point-mass dispatch.
