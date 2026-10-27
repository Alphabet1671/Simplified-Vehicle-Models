# original

Acceptance runs of the 7-DoF model on the original track.

## Subdirectories

| Directory | Purpose |
|---|---|
| `between_step_acceptance/` | Between-step certificate runs, one subdirectory per start type. |
| `closure_acceptance/` | Attempts of the lap-closure acceptance check. |
| `final_acceptance/` | Attempts of the first final acceptance round. |
| `final_acceptance_run2/` | Attempts of the second final acceptance round. |
| `final_acceptance_run3/` | Attempts of the third final acceptance round. |
| `final_acceptance_run4/` | Attempts of the fourth final acceptance round. |
| `final_acceptance_run5/` | Attempts of the fifth final acceptance round, one subdirectory per refinement level. |
| `final_acceptance_run6/` | Attempts of the sixth final acceptance round, one subdirectory per refinement level. |
| `refinement_acceptance/` | Attempts of the refinement acceptance check. |

## Attempt directories

Holds 4 `attempt_<timestamp>/` directories. Each one is a single recorded solver attempt. Heavy traces and logs inside them are not tracked by git.
