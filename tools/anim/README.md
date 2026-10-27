# anim

Animation export from a run record to video.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Animation export for the vehicle models: run record in, video file out. |
| `encode.py` | Frame writers: one ffmpeg pipe, a PNG sequence, and a null writer for tests. |
| `export_animation.py` | Export a run record as a 120 fps video (or a PNG sequence). |
| `glyphs.py` | Screen-space glyph geometry for the animation panels (NumPy only). |
| `layout.py` | Which windows exist and where they go (pure; no matplotlib). |
| `panels.py` | The four animation windows (matplotlib artists, updated in place). |
| `render.py` | Figure assembly and the frame loop (matplotlib Agg, one canvas for the whole run). |
| `resample.py` | Frame access for the animation exporter. |
| `scene.py` | Orthographic projection of the vehicle scene (NumPy only, no matplotlib). |
| `stl.py` | STL reading and the placeholder meshes of the vehicle scene (NumPy only). |
| `style.py` | Look of the animation: one frozen dataclass, no matplotlib import. |
