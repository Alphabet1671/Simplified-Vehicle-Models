# compiler

The XML model compiler: registry, capabilities, reduction adapters.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `adapters.py` | Register the reduction adapters and resolve one capability request. |
| `capabilities.py` | Declare the capability names and their Protocols. |
| `compile.py` | Compile one formulation against one assembly. |
| `manifest.py` | Record what the compiler loaded, used, reduced, and ignored. |
| `registry.py` | Register the model types and look up one descriptor by type and subtype. |
