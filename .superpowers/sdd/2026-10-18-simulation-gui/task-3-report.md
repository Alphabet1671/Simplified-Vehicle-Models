# Task 3 report: Variant XML writer

## Scope
Implemented only Task 3 on `feature/simulation-gui`:

- `tools/model_data/xml_references.py`
  - `relative_reference` resolves both target and directory before `relpath`, including macOS `/tmp` symlink behavior.
  - `rewrite_references` rewrites XML `ref` and `path` attributes and supports role replacements.
- `tools/sim_gui/variant.py`
  - `VariantFiles` and `write_variant`.
  - Writes per-case vehicle and formulation XML, nested assembly copies for changed descendants, and provenance comments.
  - Rewrites changed component references to copied files and leaves unchanged references anchored to original files.
  - Appends `.case-id` (underscores converted to hyphens) to every copied XML model id.
  - Preserves formulation `requires` only when the subtype is unchanged; creates formulations from scratch for assembly input.

## TDD
Added variant tests to `tests/test_sim_gui_core.py`, first verified the expected `ModuleNotFoundError`, then implemented the writer. The tests cover baseline compilation, tire replacement reaching the compiler, nested assembly output, and dropping `requires` on model change. Existing core tests remain passing.

## Verification
Command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_sim_gui_core -v
```

Result: **15 tests passed**.

The baseline and tire tests compile files written into an unresolved `TemporaryDirectory()` path, exercising the required path-resolution behavior. The real compiler manifest records component roles and paths; `powertrain_drivetrain` is the synthetic nested provider and is excluded from the baseline comparison as specified.

## Notes / concerns

- The current compiler registry does not expose a straightforward runnable fixture for a two-level nested vehicle tree, so no additional deep compile fixture was introduced beyond the existing arbitrary-depth vehicle configuration test. The writer itself recursively handles arbitrary nesting and writes deepest copies first.
- Existing unrelated worktree modifications were left untouched.
