# Vertical 14DoF model implementation plan

> Historical engineering plan retained for project context. Suspension dimensions and setup values in this plan are from real car data; sensitive source workbook filenames, locations, hashes, cell addresses, and sheet-specific details are intentionally omitted. Refer to the maintained model-data provenance and validation documents for the current implementation record.

## Scope

This plan describes the vertical 14-DoF model architecture, corner suspension model, damper and suspension import workflows, tire transient model, validation gates, and lap-evidence process. All imported car-specific values should be described only as **from real car data** in Git-tracked documentation.

## Key implementation decisions

- Keep the independent-corner suspension as the production acceptance model; preserve its corner spring, bump-stop, anti-roll-bar, and damper paths.
- Keep the axle roll/heave modal setup as a separate exploratory model, with 450 lb/in heave and 350 lb/in roll springs front and rear. The rear roll mode includes its stated anti-roll contribution.
- Use the existing corner damper table as the heave damper. The roll damper is a provisional proxy until measured roll-damper characteristics are supplied.
- Apply the real-car motion ratios to modal shaft velocities and forces through the mode-coordinate Jacobian. The independent-corner model and its acceptance evidence remain unchanged.
- Distinguish measured vehicle data from proposed assumptions, user decisions, and model limitations. When documentation attributes a value to a private source, write only **from real car data**.
- Preserve deterministic model generation and test the imported values against a neutral, non-sensitive fixture. Never record source workbook paths, sheet names, cell references, or hashes in Git.

## Current status

The detailed implementation and current validation status are maintained in `docs/model-data/provenance.md` and `docs/validation/14dof-simple-parameters.md`. This plan is archival and intentionally excludes private source details.
