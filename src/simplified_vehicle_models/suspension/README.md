# suspension

Suspension components and the force-network seam.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `components.py` | Register the suspension model components: the table-based damper and the corner suspension. |
| `corner.py` | Corner-table loader of the vertical (14DoF) rung; jounce knot ranges are descriptive design support, while spring and jounce-geometry tails extrapolate by their documented endpoint policies. |
| `damper_table.py` | N-valve damper force table: zero-origin normalization, Akima velocity interpolation, and multilinear clicks. |
| `elements.py` | Compiled values of the force-element components a suspension layout nests. |
| `errors.py` | Exceptions shared by the suspension subpackage. |
| `hydraulic_components.py` | Register the hydraulic suspension components. |
| `kinematics.py` | Suspension kinematics: the per-wheel geometry of a vertical (14DoF) car. |
| `modal_components.py` | Register the modal suspension assemblies and their mode-rate spring. |
| `monotone_spline.py` | Fritsch-Carlson monotone cubic Hermite spline (NumPy only) with strict, linear, or constant explicit extrapolation. |
| `physical.py` | Physical shaft linear damper and spring/damper/inertance strut force elements. |
| `physical_components.py` | XML compilers and registrations for physical dampers, struts and additive assemblies. |
| `layouts/parallel.py` | Additive physical corner and axle-mode network with signed shaft diagnostics. |
| `network.py` | The force-network seam between a suspension layout and the vertical evaluator. |
| `quasi_static.py` | Solve the quasi-static vertical tire loads of a four-wheel vehicle. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `layouts/` | One suspension layout implementation per layout. |
