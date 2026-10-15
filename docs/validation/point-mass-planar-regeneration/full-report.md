# Historical Regenerative Full-Vehicle Comparison — Planar Result Unvalidated

> **Not validation evidence:** The planar 7-DOF solve did not converge (`Maximum_Iterations_Exceeded` at 500 iterations). Its 43.9124 s objective, traces, and derived metrics are illustrative only and must not be treated as a valid lap-time result. The point-mass result converged independently. This comparison is retained as a historical regenerative-model study.

Matched placeholder-vehicle runs on the 2024 Michigan Endurance closed track.

## Results

| Metric | Point mass | Planar 7-DOF |
|---|---:|---:|
| Lap time (s) | 48.2327 | 43.9124 |
| Initial SOC | 0.9617 | 0.9617 |
| Final SOC | 0.9270 | 0.9097 |
| Consumed energy (kWh) | 0.3294 | 0.4018 |
| Recovered energy (kWh) | 0.0001 | 0.0015 |
| Net energy (kWh) | 0.3293 | 0.4003 |
| Peak discharge power (kW) | 70.7602 | 76.6925 |
| Peak regeneration power (kW; negative) | -0.0600 | -25.4515 |

Convergence: point mass **True**; planar **False** (Maximum_Iterations_Exceeded). Iterations: 3 / 500.

## Trace definitions

All figures share track distance in km and traces are sampled at the common 1 m segment midpoints. Planar node traces are averaged across adjacent nodes; point-mass acceleration/load/power traces are segment-valued. DC power is signed (discharge positive, accepted regeneration negative). Point-mass realized radius is the centerline radius; planar realized radius is ground speed divided by absolute yaw rate, with straight-line infinity clipped to 1000 m only in the figure. Wheel ordering is FL, FR, RL, RR.

Figures: `ax.png`, `ay.png`, `velocity.png`, `dc-power.png`, `cornering-radius.png`, `fz-wheels.png`
