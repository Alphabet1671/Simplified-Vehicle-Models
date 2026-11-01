# tires

Tire models.

`ttc-r20-unitire-transient.xml` preserves the user's self-contained TTC parameter artifact byte-for-byte. The current `main`-based compiler expects separate steady and transient components; this artifact is not selected by the default vehicle graph or directly supported as a self-contained transient tire.

## Files

| File | Contents |
|---|---|
| `correlated_comp_2024.xml` | `correlated-comp-2024` model, type `tire-model`, subtype `load-sensitive-friction-ellipse`. |
| `ttc-r20-unitire-transient.xml` | User-supplied self-contained TTC R20 steady-plus-transient parameter artifact; retained for provenance, not wired into the current compiler's split-component graph. |
| `hoosier-43075-r20-simple.xml` | `hoosier-43075-r20-simple` model, type `tire-model`, subtype `unitire-simple`. |
| `iem24_murphy_real_tire.xml` | `iem24-murphy-real-tire` model, type `tire-model`, subtype `load-sensitive-friction-ellipse`. |
| `iem24_murphy_real_unitire.xml` | `iem24-murphy-real-unitire` model, type `tire-model`, subtype `unitire-simple`. |
| `iem24_murphy_real_unitire_transient.xml` | `iem24-murphy-real-unitire-transient` model, type `tire-model`, subtype `unitire-transient-simple`. |
