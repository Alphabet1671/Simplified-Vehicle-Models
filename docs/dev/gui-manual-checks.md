# Manual checks for the track, vehicle and animation GUIs

The unit tests run the GUIs offscreen. They do not show a window. Do these checks once on a desktop before you trust a GUI for real work. Each check takes a few minutes.

Do the checks from the repository root. Install the GUI extra first:

```bash
python -m pip install -e '.[gui]'
```

Write down the result of each step. When a step fails, record the step number, what you saw and the terminal output.

## 1. Track GUI

Guide: [track-gui.md](track-gui.md).

1. Start the GUI:

   ```bash
   PYTHONPATH=src:. python3 tools/track_gui/app.py
   ```

2. Load the preset **Skidpad (rule geometry)**.
3. Check that both plots show the figure-eight. The first plot is the centerline. The second plot is the curvature against the distance.
4. Set the folder to `tracks_out/` and the file name to `skidpad_check`. Select **Save track**.
5. Open `tracks_out/skidpad_check.xml` in a text editor. Check that it holds `subtype="open-xy-profile"`.
6. Select **Save track** again with the same name. Check that the GUI refuses, or asks before it replaces the file.
7. Set the folder to `models/tracks/` and select **Save track**. Check that the GUI refuses unless **Allow writing inside models/** is ticked. Do not tick it.
8. Optional, needs SciPy: build a closed track from MAT telemetry. Check that the plots update and that the build can be cancelled.
9. Close the window. Delete the test files:

   ```bash
   rm tracks_out/skidpad_check.*
   ```

## 2. Vehicle GUI

Guide: [vehicle-gui.md](vehicle-gui.md).

1. Start the GUI with the planar 7DoF formulation:

   ```bash
   PYTHONPATH=src:. python3 tools/vehicle_gui/app.py models/formulations/iem24_murphy_real_planar_7dof.xml
   ```

2. Select the slot `assembly`. Change `total_mass` to `290`.
3. Check that the verdict stays green and that the slot is marked `edited`.
4. Type a text value, such as `abc`, into a number cell. Check that the GUI refuses it.
5. Select a generated slot, such as the suspension springs. Check that the table is read-only and names the generator tool.
6. Save as `check_vehicle` into `vehicles_out/`.
7. Start the simulation GUI and open `vehicles_out/check_vehicle__formulation.xml`. Check that it loads. See [simulation-gui.md](simulation-gui.md).
8. Check that `git status --short models/` prints nothing. The vehicle GUI must not change `models/`.
9. Close both windows. Delete the test files:

   ```bash
   rm vehicles_out/check_vehicle__*
   ```

## 3. Animation GUI

Guide: [animation-gui.md](animation-gui.md). The export needs `ffmpeg` on `PATH`.

1. Make a short run record with the planar 7DoF runner. Use `planar-7dof`, not `14dof-simple`: the `14dof-simple` runner has a known import error on `main`.

   ```bash
   PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py \
       --model planar-7dof --track models/tracks/2024_michigan_endurance_closed.xml \
       --planner live --maximum-speed 60.0 --pedal-travel-time 0.175 \
       --maximum-step 0.008 --rtol 1e-05 --method BDF --spacing 0.5 \
       --timeout-seconds 900 --output-root /tmp/runs/planar --record
   ```

2. Start the GUI with the record:

   ```bash
   PYTHONPATH=src:. python3 tools/anim_gui/animation_gui_app.py /tmp/runs/planar/<attempt>/run_record.npz
   ```

3. Check that the panel shows the frame count and the windows that it will draw.
4. Select the still-frame preview. Check that a PNG frame is shown.
5. Tick **Render only a time range** and set 0 s to 2 s. Select **Export**.
6. Check that the progress bar moves and that `videos/<name>.mp4` plays in a video player.
7. Start a second export to the same file. Check that the GUI asks before it replaces the video. Accept, then select **Cancel**.
8. Check that no `.partial.mp4` file stays in `videos/`, and that the first video still plays.
9. Close the window. Delete the test video in `videos/`.

## Report the result

When all steps pass, the GUIs are ready for use. When a step fails, open an issue with the step number and the record from that step.
