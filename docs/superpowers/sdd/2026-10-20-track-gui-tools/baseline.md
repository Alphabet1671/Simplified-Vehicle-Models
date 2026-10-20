# Track GUI extension regression baseline

## Source

The unchanged local main checkout was at `f634d21`. The feature branch starts from this commit. No source changes were present in main during the baseline run.

## Full test command

```bash
PYTHONPATH=src MPLCONFIGDIR=/tmp/mplcfg QT_QPA_PLATFORM=offscreen \
  python3 -m unittest discover -s tests -q
```

Result: **1,817 tests in 505.692 s; 20 failures, 9 errors, 2 skipped.** Exit code: 1.

The 29 normalized failure/error entries match the previous merged-main baseline exactly. There are zero added entries and zero removed entries. Normalize the absolute repository path to `<root>` before comparison.

The existing failures concern animation render fixtures and vertical-model evidence/domain checks. They are not track GUI test failures. The final branch test run must be compared with these named entries, not described as an entirely passing suite.

The raw local log is `/tmp/track-tools-main-baseline.log`. The normalized entry list is `/tmp/track-tools-main-baseline-failures.txt`. These temporary files are evidence for this local run, not repository dependencies.

## Existing focused track GUI tests

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen \
  python3 -m unittest \
  tests.test_track_gui_design tests.test_track_gui_export \
  tests.test_track_gui_closed tests.test_track_gui_widgets \
  tests.test_track_gui_qt tests.test_track_gui_docs \
  tests.test_track_tool_patches -q
```

Result: **65 tests in 0.745 s, OK.** Exit code: 0. The offscreen Qt plugin prints the existing font-alias and `propagateSizeHints` notices. This run is not a manual desktop check.

## Private fixture identity

| Fixture | Bytes | SHA256 |
|---|---:|---|
| MIS June 2024, `0615CompEndurance.mat` | 334,513,192 | `091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7` |
| FSC May 2026, `FSC-2026-May-Endurance.mat` | 654,292,592 | `e160110c53e41c8a3419edafc30d84f6bf3bf442016ec7480c11c3d425e6f82b` |

The MAT files remain private and untracked. Fixture acceptance results will be recorded separately after all import and export operations are available.
