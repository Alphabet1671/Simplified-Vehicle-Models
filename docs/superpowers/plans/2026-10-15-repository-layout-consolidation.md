# Repository Layout Consolidation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep full documentation and generated reports under `docs/`, ignore LLM working directories, and categorize the tools without old-path wrappers.

**Architecture:** Preserve a short root README and AGENTS pointer (explicit user preference); move the model-data provenance and tracked agent task reports into `docs/`. Group the flat `tools` namespace into domain subpackages with no compatibility shims, update imports and reproducibility commands, but preserve frozen acceptance checker bytes and historical JSON records. Keep previously ignored scratch material local and never move the separate Git worktree.

**Tech Stack:** Python 3.12+, stdlib unittest, Git, Markdown.

**Spec:** User request in conversation: "reorganize the repo's tools and documentations ... all documentation in /docs/ ... generated reports there ... all the LLM agent working folders to gitignore"; clarified: retain short README.md/AGENTS.md entrypoints; categorize tool scripts without old-path wrappers.

## Global Constraints

- Leave `docs/validation/**/attempt_*/` historical machine records unchanged; hashes and recorded original script paths are evidence.
- Do not modify the bytes of frozen `tools/check_iem24_forward_tube.py`; its SHA-256 is pinned by tests and acceptance records.
- Do not move or delete `.worktrees/point-mass`; it is a separate Git worktree.
- Keep full generated MIS reports under `docs/validation/` and ensure `tests/test_mis_report_standard.py` passes.
- Preserve previously ignored `.superpowers/` scratch files locally. Move only the 21 tracked SDD reports into `docs/superpowers/sdd/`.
- Do not modify source-hashed solver/model files or rewrite acceptance results.

---

### Task 1: Ignore agent workspaces and relocate documentation

**Files:** Modify `.gitignore`, `README.md`, `AGENTS.md`, `docs/standards/mis-2024-report-standard.md`, `docs/validation/powertrain.md`, `tests/test_energy_lap.py`; move `models/data/provenance.md` to `docs/model-data/provenance.md`; move tracked `.superpowers/sdd/**/*.md` to `docs/superpowers/sdd/`; create `tests/test_repository_layout.py`.

**Interfaces:** Documentation roots `docs/`, `docs/superpowers/sdd/`; no executable interfaces change.

- [ ] **Step 1: Add a failing repository-layout test.** Assert `models/data/provenance.md` is absent and all tracked `.md` paths except root README/AGENTS are under `docs/`. Assert `.superpowers/` and `.worktrees/` are ignored via `git check-ignore`.
- [ ] **Step 2: Run `PYTHONPATH=src:. python3 -m unittest tests.test_repository_layout -q` and observe failure.**
- [ ] **Step 3: Extend `.gitignore` with `.superpowers/`, `.worktrees/`, known agent dirs, cache dirs, and heavy `docs/validation/**/attempt_*/` artifact patterns without ignoring summary/certificates; use `git mv` for tracked Markdown and provenance, adjust references; replace root README/AGENTS with short navigation and live commands.**
- [ ] **Step 4: Run layout test and `git diff --check`; verify only previously tracked SDD logs move, while ignored scratch and `.worktrees/` remain untouched.**
- [ ] **Step 5: Commit this coherent docs and ignore migration.**

### Task 2: Categorize importable tool scripts

**Files:** Move `tools/*.py` and `tools/*.m` to `tools/{data,tracks,model_data,run,gates,reports,diagnostics}/`; create each category `__init__.py`; adjust path depth in moved scripts; update imports in `tools/`, `tests/`, `examples/`, README/AGENTS, `docs/standards/`, `docs/dev/` and report templates. Preserve `check_iem24_forward_tube.py` byte-for-byte.

**Interfaces:** New module spelling `tools.<category>.<basename>` and CLI `python -m tools.<category>.<basename>` from repository root with `PYTHONPATH=src:.`; old tool script paths are removed by user choice.

- [ ] **Step 1: Add a failing test in `tests/test_repository_layout.py` that lists every category and asserts old top-level Python tools no longer exist and moved commands support `--help` where applicable.**
- [ ] **Step 2: Run layout test to see missing categories fail.**
- [ ] **Step 3: Use `git mv` to categorize scripts; change cross-tool imports and path assumptions. Leave the frozen v1 checker content untouched and update its test path, not its hash. Ensure standalone CLI execution works via `python -m` and direct scripts when promised in docs.**
- [ ] **Step 4: Update code and docs references only for executable commands, source path constants, and generated report templates. Preserve historical command lines inside attempt JSON/old reports, adding a mapping note rather than silently rewriting evidence.**
- [ ] **Step 5: Run targeted tests, `--help` smoke checks for representative categories, and `git diff --check`. Commit tool migration.**

### Task 3: Audit generated report placement and full regression

**Files:** Create `docs/dev/tools-layout.md`; update generated report command text only if independently reproducible; keep report PNG/CSV/JSON paths under `docs/validation/`.

**Interfaces:** Report output defaults remain rooted at `docs/validation/`; tool commands use the new module names.

- [ ] **Step 1: Write a layout test that discovers generated report defaults and checks that no tracked report Markdown or PNG outside root entrypoints lives outside `docs/`.**
- [ ] **Step 2: Run the test and fix violations without changing model or acceptance certificate content.**
- [ ] **Step 3: Run `PYTHONPATH=src:. python3 -m unittest discover -s tests -q`, verify pinned SHA-256 and validation link targets, and inspect changed report commands.**
- [ ] **Step 4: Commit adjustments and push the review branch; report test count, paths, and any deliberately preserved historical references.**
