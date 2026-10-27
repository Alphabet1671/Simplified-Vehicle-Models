# suspension

Suspension components and the force-network seam.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `components.py` | Register the suspension model components: the table-based damper and the corner suspension. |
| `corner.py` | Corner-table loader of the vertical (14DoF) rung. |
| `damper_table.py` | N-valve damper force table: monotone velocity spline x multilinear clicks. |
| `elements.py` | Compiled values of the force-element components a suspension layout nests. |
| `errors.py` | Exceptions shared by the suspension subpackage. |
| `hydraulic_components.py` | Register the hydraulic suspension components. |
| `kinematics.py` | Suspension kinematics: the per-wheel geometry of a vertical (14DoF) car. |
| `modal_components.py` | Register the modal suspension assemblies and their mode-rate spring. |
| `monotone_spline.py` | Fritsch-Carlson monotone cubic Hermite spline (NumPy only). |
| `network.py` | The force-network seam between a suspension layout and the vertical evaluator. |
| `quasi_static.py` | Solve the quasi-static vertical tire loads of a four-wheel vehicle. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `layouts/` | One suspension layout implementation per layout. |
