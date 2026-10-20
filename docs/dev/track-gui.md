# Track GUI

The track GUI makes track models for the simulators. It has two modes. In the first mode you design an open course from sections. In the second mode you build a closed track from MAT telemetry. The GUI writes new files and can replace an existing file only after the mode's overwrite checks and confirmations.

## Install

Install the GUI extra from the repository root:

```bash
python -m pip install -e '.[gui]'
```

The closed-track mode needs SciPy. Install the forward extra:

```bash
python -m pip install -e '.[forward]'
```

Without SciPy the GUI still starts. The closed-track tab shows a message and its Build button is off.

## Start the GUI

Start the application from the repository root. The optional argument is a saved design file:

```bash
PYTHONPATH=src:. python3 tools/track_gui/app.py
PYTHONPATH=src:. python3 tools/track_gui/app.py my_design.json
```

## Design an open course

Use the tab **Design an open course**. The section table has one row for each section. The types are `straight`, `arc`, `arc-angle` and `clothoid`. A right turn has a positive curvature. Each type reads only some columns. The GUI greys the cells that it ignores.

| Type | Columns it reads |
|---|---|
| `straight` | Length |
| `arc` | Length, Curvature |
| `arc-angle` | Radius, Angle |
| `clothoid` | Curvature, End curvature, Rate |

Select a preset and **Load preset** to start from the committed acceleration or skidpad course. The presets build the same course as `tools/tracks/build_fsae_dynamic_tracks.py`. The build settings are the node spacing, the start heading, the start position, the curvature-jump switch and the jump ramp length.

The two plots update after each edit. The first plot shows the centerline. The second plot shows the curvature against the distance. Both mark the section boundaries. The line under the table shows the length, the node count, the minimum radius, the heading change and the number of curvature jumps. When the design is not valid, the line shows the reason and **Save track** is off.

**Save design…** writes the sections and settings to a JSON file. **Open design…** reads it again. Saving inside `models/` needs **Allow writing inside models/**. If the target exists, the GUI asks before it replaces the file, even when **Overwrite existing files** is checked.

## Build a closed track from telemetry

Use the tab **Closed track from telemetry**. Select the MAT file, the lap start and end times, the node spacing and the smoothing length. The maximum-radius value is saved into the output XML metadata; it is not passed to the geometry builder and does not constrain the built geometry. Select **Build**. The build runs in a background thread, so the window stays usable. The result box lists the length, the node count, the closure gap, the winding error, the minimum radius, the minimum self-distance and the largest smoothing move. Check these figures before you save.

## Import MAT telemetry

Choose north/X and east/Y channels explicitly. Velocity channels use m/s (or a supported velocity unit); position channels use metres; GPS channels use latitude and longitude in degrees. The GUI synchronizes selected channels over their common time interval. GPS coordinates become local north/east metres. Do not assume that empty-looking channels contain data. In the FSC fixture, Latitude_MTI and Longitude_MTI contain GPS; several plausible Pos*/GPS_* names are empty or nearly empty. For FSC vehicle-frame velocity checks, velY_MTI is north and velX_MTI is east. For MIS velocity, VNorth is north and VEast is east.

Multi-lap mode detects spatial returns. It does not require a lap counter. Select a window that contains complete moving laps. A constant lap counter or unreliable lap trigger is not a reliable boundary source. The detector applies return-radius, travel, duration and heading limits, then rejects stopped, pit or dissimilar laps. A window with too few supported laps fails rather than creating a track. Check accepted and rejected lap counts, time boundaries, point clipping, registration residuals and overlap support before saving. Smoothing and spacing affect the resulting geometry; do not treat example circuit lengths as targets.

A reproducible private-fixture acceptance run uses explicit paths:

```bash
PYTHONPATH=src:. python3 tools/track_gui/validate_track_imports.py \\
  --mis /path/to/0615CompEndurance.mat \\
  --fsc "/path/to/FSC-2026-May-Endurance.mat" \\
  --output /tmp/track-import-acceptance
```

The command writes `acceptance.json` and one overlap PNG for each source. It does not discover private files by default. Set `TRACK_GUI_MIS_MAT` and `TRACK_GUI_FSC_MAT` to run the optional actual-data unit tests. Missing private data makes those tests skip; a selected CLI path that does not exist is an error.

## View, reverse and cut a track

The existing-track viewer loads repository track XML. Signed XY tracks show the map, start/direction and curvature. Legacy unsigned distance/radius tracks have no recoverable XY map and show an explicit unsigned-geometry notice; reversal still works without inventing a turn direction. Reverse changes distance order and direction while retaining a closed start. Double reversal should restore geometry, with headings compared modulo 2π. Closed tracks can be split at editable distances or into equal sections. Use the cut controls to add, edit or remove cut distances. Check every section endpoint and the start/finish wrap before saving. Each reconstructed section must remain within the 0.02 m source-position tolerance and pass the strict track loader. Section export can fail if the geometry cannot meet both checks, if it reaches the 100,000-node resource cap, or if the target set fails a safety check. Do not weaken loader validation to force an export.

## Save a track

The save bar has a folder, a file name, a model id and a title. The GUI writes `<file name>.xml` and `<file name>.csv`. It writes them in a temporary folder first, reads them back with `load_track`, and only then moves them to the folder. If the read-back fails, nothing reaches the folder.

The GUI applies these rules:

- The default folder is `tracks_out/` in the repository root. Git ignores it.
- A folder inside `models/` is refused unless you tick **Allow writing inside models/**.
- In open-course mode, an existing file is refused unless you tick **Overwrite existing files**. A generated file also needs the second confirmation shown by the GUI; this confirmation is separate from the overwrite checkbox.
- In closed-track mode, an existing file is refused unless you tick **Overwrite existing files**. A generated file also needs the second confirmation shown by the GUI; this confirmation is separate from the overwrite checkbox.
- A file that carries a `Generated by tools/...` comment is treated as generated. The message names the tool that normally writes it.
- The file name has 1 to 40 letters, numbers, `_` or `-`. The model id has 1 to 80 letters, digits, `.`, `_` or `-`, and starts with a letter or a digit. The title is one line of 1 to 120 characters.

The simulation GUI lists only `closed-xy-profile` tracks. An open course does not appear in its track list. This is intended. To use a closed track there, save it inside `models/`.

## Shared code

The package `tools/gui_common/` holds the code that the track, vehicle and animation GUIs share: the name check, the atomic file writes, the `models/` protection and `ProcessJob`, a cancelable child process. Only `tools/gui_common/qt/` imports PyQt6.

## Tests

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest \
  tests.test_gui_common tests.test_gui_common_qt tests.test_track_tool_patches \
  tests.test_track_gui_design tests.test_track_gui_export tests.test_track_gui_closed \
  tests.test_track_gui_channels tests.test_track_gui_laps tests.test_track_gui_ops \
  tests.test_track_gui_real_data tests.test_track_gui_widgets tests.test_track_gui_qt \
  tests.test_track_gui_tools_qt tests.test_track_gui_docs -v
```

Qt tests skip when PyQt6 is not installed. The closed-track tests skip when SciPy is not installed. Real-data acceptance tests skip with a path hint when either private MAT fixture is absent; set `TRACK_GUI_MIS_MAT` and `TRACK_GUI_FSC_MAT` to use alternate local copies. Those tests also skip when SciPy or matplotlib is not installed. The environment variable `TRACK_GUI_QUIT_AFTER_MS` closes the window after that many milliseconds. A smoke test uses it.

## Limits

- Both modes can overwrite an existing track only when **Overwrite existing files** is checked. Generated tracks also need the second confirmation from the GUI.
- The GUI does not import a CSV centerline.
- Windows and Linux are not tested. Only macOS was used.
