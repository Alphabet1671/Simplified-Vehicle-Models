# tires

Tire models.

`ttc-r20-unitire-transient.xml` preserves the user's self-contained TTC parameter artifact byte-for-byte. The current compiler expects separate steady and transient components. The artifact is retained byte-for-byte as provenance; its full parameter block is split into `ttc-r20-unitire-steady.xml` and `ttc-r20-unitire-transient-companion.xml`, because the strict transient component schema accepts only four scalar fields. Neither companion changes the default vehicle graph.

## Files

| File | Contents |
|---|---|
| `correlated_comp_2024.xml` | `correlated-comp-2024` model, type `tire-model`, subtype `load-sensitive-friction-ellipse`. |
| `ttc-r20-unitire-transient.xml` | User-supplied self-contained TTC R20 steady-plus-transient parameter artifact; retained byte-for-byte as provenance, not directly accepted as a split component by the current closed-schema transient compiler. |
| `ttc-r20-unitire-steady.xml` | Exact transcription of the TTC artifact's 59 steady parameters for the `unitire-simple` compiler; no empirical validation claim. |
| `ttc-r20-unitire-transient-companion.xml` | Exact four-scalar transient companion copied from the TTC artifact for the current closed-schema transient compiler. |
| `hoosier-43075-r20-simple.xml` | `hoosier-43075-r20-simple` model, type `tire-model`, subtype `unitire-simple`. |
| `iem24_murphy_real_tire.xml` | `iem24-murphy-real-tire` model, type `tire-model`, subtype `load-sensitive-friction-ellipse`. |
| `iem24_murphy_real_unitire.xml` | `iem24-murphy-real-unitire` model, type `tire-model`, subtype `unitire-simple`. |
| `iem24_murphy_real_unitire_transient.xml` | `iem24-murphy-real-unitire-transient` model, type `tire-model`, subtype `unitire-transient-simple`. |
