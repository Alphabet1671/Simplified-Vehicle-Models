# IEM24 Murphy comparison (historical OCP claim unverified)

> **Verification status:** The planar 7-DOF OCP API has been retired from the
> current tree and is not supported. The saved 20 m / 52-node `Solve_Succeeded`
> result (53.967 s) is historical only, not a validated prediction. Its coarse
> geometry is invalid as documented in `docs/validation/planar-ocp.md`; solver
> convergence and saved trace metrics do not establish a usable lap. Accordingly,
> this artifact is not a current three-way comparison: only measured telemetry
> and the point-mass result remain usable here. Historical OCP data and plots are
> retained for provenance, but all OCP-derived metrics below are unverified.

Measured telemetry remains unchanged. The pack-voltage panel plots terminal-voltage change from each trace's own lap start, so all three start at zero and the mid-stint measured pack voltage offset is removed.

The point-mass run uses the real IEM24 parameter set and measured rear-drive controller configuration. The 7DoF run uses the same mass, CG, LLTD, aero, rear-only topology, battery initial SOC, and 10.5 N·m torque cap. It requires a combined-slip tire and smooth motor envelope, so its tester-fitted UniTire uses provisional surface-mu scaling informed by measured peak deceleration/cornering (not a tire re-fit), while its smooth motor envelope approximates the measured map. Parallel steering, 3.5 in/rev Narco rack travel, and 65% front brake bias are user-confirmed; steering-arm/rate and hydraulic/pedal details remain provisional. The virtual brake request is extended only as needed to reach the current hydraulic pressure cap.

- Measured flying laps: 10; median lap time 71.685 s.
- Point-mass lap: 77.561 s (converged: True).
- Planar 7DoF OCP: **historical/unverified**; the saved 53.967 s / `Solve_Succeeded` entry is not a usable lap-time claim because its 20 m geometry is invalid.

![Three-way measured, point-mass and planar 7DoF overlay](comparison_three_way.png)

The per-signal metrics and shared traces remain archived in `comparison_three_way_summary.json` and `comparison_three_way.csv`; planar OCP entries in both are historical/unverified and must not be interpreted as current comparison evidence.
