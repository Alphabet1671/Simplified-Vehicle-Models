# Task 7 Report: Animation GUI Guide and Documentation

## Status
Implemented the animation GUI guide, documentation index entry, and focused documentation regression tests.

## Files
- `docs/dev/animation-gui.md`
- `docs/dev/tools-layout.md`
- `tests/test_anim_gui_docs.py`

## Implementation
- Documented installation, startup commands, record and study selection, exporter options, progress/cancellation behavior, output naming, preview behavior, tests, and known limits.
- Added the `tools/anim_gui/` category row to the tools layout with a link to the guide.
- Added the exact brief-specified tests for the startup command, tools-layout row and guide link, and preservation of `/videos/` in `.gitignore`.
- Preserved the existing `/videos/` ignore entry unchanged.

## Focused verification
```text
$ MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_docs -v
 test_the_default_output_folder_is_ignored (tests.test_anim_gui_docs.AnimGuiDocsTests.test_the_default_output_folder_is_ignored) ... ok
 test_the_guide_names_the_start_command (tests.test_anim_gui_docs.AnimGuiDocsTests.test_the_guide_names_the_start_command) ... ok
 test_tools_layout_lists_the_package_and_links_the_guide (tests.test_anim_gui_docs.AnimGuiDocsTests.test_tools_layout_lists_the_package_and_links_the_guide) ... ok

----------------------------------------------------------------------
Ran 3 tests in 0.001s

OK
```

`git diff --check` passed. No unrelated files were staged.

## Review fix round 1
- Corrected the editable-install command to `python -m pip install -e '.[gui]'`.
- Clarified that the GUI asks before replacing an existing output, while non-empty PNG folders are refused rather than replaced.

Focused verification after the review fixes:
```text
$ MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_docs -v
 test_the_default_output_folder_is_ignored (tests.test_anim_gui_docs.AnimGuiDocsTests.test_the_default_output_folder_is_ignored) ... ok
 test_the_guide_names_the_start_command (tests.test_anim_gui_docs.AnimGuiDocsTests.test_the_guide_names_the_start_command) ... ok
 test_tools_layout_lists_the_package_and_links_the_guide (tests.test_anim_gui_docs.AnimGuiDocsTests.test_tools_layout_lists_the_package_and_links_the_guide) ... ok

----------------------------------------------------------------------
Ran 3 tests in 0.001s

OK
```

`git diff --check` passed after the review fixes.
