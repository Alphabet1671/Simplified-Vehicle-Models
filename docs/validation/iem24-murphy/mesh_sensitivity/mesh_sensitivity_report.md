# IEM24 planar 7DoF mesh sensitivity

All runs use the same real-parameter IEM24 formulation and solver settings; only distance mesh spacing varies. A lap time is valid only when `converged` is true. Only converged runs at spacing <=0.5 m and runtime <=300 s are usable. No usable 0.5 m reference run was found; coarser results are diagnostic only.

| Mesh (m) | Nodes | Runtime (s) | Usable | Status | Iterations | Lap time (s) | Δ vs 0.5 m (s) | Max dynamics residual | Max inequality violation |
|---:|---:|---:|:---:|:---|---:|---:|---:|---:|---:|
| 0.05 | — | 300.04 | False | Timed_Out (not converged) | — | — | — | — | — |
| 0.1 | — | 300.07 | False | Timed_Out (not converged) | — | — | — | — | — |
| 0.5 | — | 300.04 | False | Timed_Out (not converged) | — | — | — | — | — |
| 1 | 1018 | — | False | Maximum_Iterations_Exceeded (not converged) | 8000 | — | — | 2.386e+01 | 5.226e-01 |
| 2 | 509 | — | False | Maximum_Iterations_Exceeded (not converged) | 8000 | — | — | 4.427e-01 | 5.712e-02 |
| 5 | 204 | — | False | Infeasible_Problem_Detected (not converged) | 5557 | — | — | 5.044e-08 | 4.307e-02 |

**No usable result.** No requested spacing produced a converged lap inside the 300 s cap, so the usable set is empty and no lap time is reported. The measured cause is the arithmetic budget, not a convergence defect: at 0.5 m the NLP has 38,644 variables and costs about 1.35 s per IPOPT iteration plus about 26 s of setup, so 300 s buys roughly 200 iterations, while the 20 m rung needs 2,226 and the 1 m and 2 m runs exhaust 8,000. Only MUMPS is available as a linear solver here; MA57, MA27, and Pardiso all return `Invalid_Option`. Full diagnosis and preserved attempts: `.superpowers/sdd/2026-10-16-iem24-mesh-sensitivity/task-4-report.md`.

Per-mesh CSV/summary/log files are stored in their `spacing_*` subdirectories.
