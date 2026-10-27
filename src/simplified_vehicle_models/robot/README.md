# robot

Fake-only robot execution contract.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Gated fake-only robot execution contract. |
| `contract.py` | Vendor-neutral robot execution contract; no physical transport implementation. |
| `fake.py` | Deterministic fake backend used only for contract and fault testing. |
| `session.py` | Safety-gated robot session state machine for an externally supplied backend. |
