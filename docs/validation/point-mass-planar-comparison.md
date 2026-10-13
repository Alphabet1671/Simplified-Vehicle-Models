# Point-Mass vs Planar Lap Comparison

## Lap-time result

Both solvers used the placeholder 2024 Formula vehicle and the closed Michigan endurance track. On the 1 m / 1,018-node comparison mesh (1,016.701 m lap):

| Solver | Lap time | Status | Speed range | Iterations |
| --- | ---: | --- | ---: | ---: |
| Point mass + energy-coupled SOC | 45.73984 s | converged | 12.109–35.270 m/s | 2 profile sweeps, 3 SOC iterations |
| Planar 7-DOF OCP | 43.90662 s | `Solve_Succeeded` | 12.188–31.516 m/s | 296 IPOPT iterations |

The point-mass result is **1.83321 s (4.18%) slower**. The point-mass model is lower fidelity: it has no yaw/sideslip, steering, wheel-speed, or wheel-level force states. Its tire is a pure-force envelope sampled from UniTire, while the planar model uses the full simplified combined-slip tire law. Treat this as a model comparison, not a same-fidelity solver contest.

## UniTire-derived grip and load response

Point-mass pure-slip peaks were sampled at 15 m/s using 601-point sweeps over slip angle 0–0.6 rad and slip ratio 0–1.5, then refined locally with a parabolic fit. A 71-node load table spans 0–3,500 N; force capacity is interpolated by wheel load. No tire coefficients from the 2025 Matlab lap simulation were used.

| Normal load | Longitudinal peak | \(\mu_x\) | Lateral peak | \(\mu_y\) |
| ---: | ---: | ---: | ---: | ---: |
| 250 N | 576.50 N | 2.3060 | 595.09 N | 2.3804 |
| 660 N (rated) | 1,508.49 N | 2.2856 | 1,571.03 N | 2.3804 |
| 1,500 N | 3,448.91 N | 2.2993 | 3,570.51 N | 2.3803 |
| 2,500 N | 5,787.62 N | 2.3150 | 5,950.85 N | 2.3803 |

At this reference speed, lateral peak grip is almost load-independent over the sampled range, while longitudinal peak grip varies weakly and non-monotonically. The lookup preserves this sampled UniTire behavior instead of imposing the old linear load-sensitivity coefficients.

## Compute time

One isolated same-process timing sample (single trial, not a median):

| Work | Wall time |
| --- | ---: |
| Point-mass profile + battery/SOC coupling | 30.60 s |
| Planar OCP with equilibrium seed | 197.34 s |
| Input/model/lookup setup | 0.021 s |

For this sample, point mass was **6.45× faster** than the planar solve. The point-mass count is forward/backward profile sweeps plus outer energy iterations; it is not directly comparable to IPOPT iterations. Timings are machine- and load-dependent and should be repeated before treating the ratio as a stable benchmark.

## Warm-start experiment

Mapped the point-mass speed/time trajectory to the planar mesh; retained the planar equilibrium seed's sideslip and steering (which point mass cannot supply), estimated pedal guesses from the speed slope, and invoked the planar local algebraic projection. With a 500-iteration cap, IPOPT returned `Maximum_Iterations_Exceeded` after 500 iterations. The result failed feasibility checks (maximum algebraic residual 5.22e4, dynamics residual 28.99, inequality violation 4.06); its 129.61 s objective is **not a valid lap time**. The converged cold/equilibrium-seed run needed 296 iterations, so this trial saved no iterations. Do not use this mapping as a production warm start yet; a useful bridge needs a dynamically consistent reconstruction of planar body and wheel states, not speed interpolation alone.

## Model-mapping notes and reproduction

- Source vehicle formulation: `models/formulations/placeholder_planar_7dof.xml`; assembly: `models/vehicles/placeholder_2024_formula.xml`.
- Source track: `models/tracks/2024_michigan_endurance_closed.xml`; point mass uses the magnitude of sampled curvature for radius, while planar retains the signed XY path.
- The assembly has a standalone transmission ratio of 10.9, but the smooth motor envelope used by the planar equations specifies 11.589. The point-mass reduction explicitly uses 11.589 to match the ratio used in the planar formulation.
- Point-mass battery/SOC coupling was enabled; regeneration was disabled. The point-mass final SOC was 0.91073. The point-mass fixed-radius and pure-force tire reductions remain additional approximations.
- Planar settings: exact sparse Hessian, 1 m mesh, 0.05 m lateral corridor, 0.2 rad heading bound, and steering-rate limit \(\pi\) rad/s. It converged with algebraic residual 2.38e-7, dynamics residual 1.24e-10, and inequality violation below 1e-8.

Reproduce the lap-time and warm-start run with:

```bash
PYTHONPATH=src:/tmp/casadi_pkgs python3 tools/compare_planar_point_mass.py --spacing 1 --iterations 500 --output /tmp/point-mass-planar.json
```

The machine-readable summary is in [`point-mass-planar-comparison.json`](point-mass-planar-comparison.json). The UniTire adapter, point-mass/planar seed bridge, and comparison runner are in `src/simplified_vehicle_models/tires/unitire_envelope.py`, `src/simplified_vehicle_models/solvers/point_mass_seed.py`, and `tools/compare_planar_point_mass.py`.
