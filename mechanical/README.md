# SmartRover Mechanical CAD

Parametric `build123d` CAD sources for the SmartRover unibody chassis, optional camera brace, and 2-piece wheels (outputs live in [`3d-printing/`](../3d-printing/)).

- [`cad/params.py`](cad/params.py) — Shared PCB, battery, motor, wheel, fastener, and clearance dimensions.
- [`cad/parts.py`](cad/parts.py) — Printable part builders (`UnibodyChassis`, optional `CameraBrace`, closed-hub `WheelRim`, and `WheelTire` TPU).
- [`cad/verify_cad.py`](cad/verify_cad.py) — Automated checks for solid validity, `Z = 0` seating, overhangs, collisions, 180 × 180 mm bed fit, ToF FoV, and fasteners.
- [`cad/export_and_render.py`](cad/export_and_render.py) — Exports individual STLs/STEPs, assembled/exploded STEPs, and the compact rigid print plate.
- [`cad/render_blender.py`](cad/render_blender.py) — Headless Blender Cycles renderer for [`3d-printing/renders/`](../3d-printing/renders/).

## Commands

Requires [`uv`](https://docs.astral.sh/uv/) (provisions Python 3.12 and locked `build123d` deps from [`pyproject.toml`](pyproject.toml) / [`uv.lock`](uv.lock)). Run from the repo root:

```bash
make cad-verify   # Check geometry, clearances, print plate, and collisions
make cad-export   # Regenerate STLs/STEPs into 3d-printing/
make cad-render   # Re-export and render previews (requires Blender; override with BLENDER=...)
```
