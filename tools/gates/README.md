# gates

Acceptance-gate checkers.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `certify_iem24_forward_between_step.py` | Decide whether a refined forward lap earns ``between_step_validation_complete``. |
| `check_14dof_simple_gates.py` | Evaluate the Task 9 vertical 14DoF acceptance gates from finished attempts. |
| `check_iem24_forward_refinement.py` | Compare two independent forward-driver attempts on a shared physical distance grid. |
| `check_iem24_forward_refinement_vertical.py` | Compare two 14DoF forward-driver attempts on a shared physical distance grid. |
| `check_iem24_forward_tube.py` | Estimate an empirical error tube from three refinement levels of one lap. |
| `check_iem24_forward_tube_v2.py` | NON-BINDING tube diagnostic v2 (protocol: docs/validation/iem24-murphy/ seven_dof_track_pair/tube_v2_protocol.md, frozen before any fourth level). |
| `check_iem24_forward_tube_vertical.py` | Empirical refinement tube for the vertical 14DoF, from three refinement levels. |
| `check_minimum_time_line.py` | Stage 1 acceptance checks for the minimum-time line. |
