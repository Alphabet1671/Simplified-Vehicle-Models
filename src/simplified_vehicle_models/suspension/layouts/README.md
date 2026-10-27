# layouts

One suspension layout implementation per layout.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Suspension layouts: one force-network implementation per layout. |
| `axle_roll_heave.py` | Axle roll/heave layout: a decoupled heave and roll spring and damper per axle. |
| `four_mode.py` | Four-mode layout: heave, roll, pitch and warp springs and dampers. |
| `hydraulic.py` | Hydraulically interconnected layout: corner springs plus gas-orifice cylinder circuits. |
| `independent_corner.py` | Independent-corner layout: a spring and a damper per wheel and one ARB per axle. |
| `modal.py` | Stateless modal force network and the two mode matrices built on it. |
