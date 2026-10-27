# maneuvers

Open-loop maneuver definitions and controllers.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Qt-free open-loop maneuver definition and command contracts. |
| `analysis.py` | Evidence-only analysis for completed or partial maneuver simulations. |
| `calibration.py` | Frozen pre-run peak lateral-acceleration amplitude calibration. |
| `csv_source.py` | Strict, immutable-source CSV playback with canonical SI commands. |
| `pid.py` | Qt-free sampled PID controller with filtered measurement derivative. |
| `program.py` | Sampled maneuver controller composition with explicit channel ownership. |
| `schema.py` | Strict serialization and validation for versioned maneuver definitions. |
| `types.py` | Qt-free value types for maneuver control. |
| `waveforms.py` | Pure analytical steering waveform sources. |
