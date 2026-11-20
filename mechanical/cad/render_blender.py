"""Headless Blender script to render studio-quality previews of SmartRover.

Usage:
    blender --background --python mechanical/cad/render_blender.py -- <preview_dir> <output_dir>
"""

import sys
from pathlib import Path
import bpy
from mathutils import Vector

CAD_DIR = Path(__file__).resolve().parent
REPO_ROOT = CAD_DIR.parent.parent
sys.path.insert(0, str(CAD_DIR))
import params as p

# Material palette: (Red, Green, Blue, Alpha, Roughness, Metallic)
# Stealth Black & Silk Silver livery: PLA Black unibody & optional brace, Silk+ Silver 5-spoke rims, Black TPU tires
MATERIALS = {
    "frame_dark": (0.035, 0.038, 0.042, 1.0, 0.42, 0.0),    # PLA Black unibody chassis & optional camera brace
    "rim_silver": (0.76, 0.79, 0.82, 1.0, 0.18, 0.65),      # PLA Silk+ Silver 5-spoke rims
    "tire_black": (0.022, 0.022, 0.025, 1.0, 0.85, 0.0),    # Matte Black TPU 95A chevron tires
    "pcb_green": (0.025, 0.25, 0.095, 1.0, 0.30, 0.05),     # FR4 solder mask PCB & Arducam
    "battery_blue": (0.10, 0.24, 0.52, 1.0, 0.40, 0.10),    # 1S LiPo pack (visible in exploded view)
    "sensor_dark": (0.05, 0.055, 0.065, 1.0, 0.38, 0.05),   # Black M12 lens & VL53L0X breakout
    "motor_metal": (0.78, 0.80, 0.83, 1.0, 0.22, 0.85),     # Nickel can motors & shafts
}


def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = 64
    scene.cycles.use_denoising = True
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 860
    try:
        scene.view_settings.look = "AgX - Medium High Contrast"
    except TypeError:
        pass

    world = bpy.data.worlds.new("World")
    scene.world = world
    if world.node_tree is None:
        world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.90, 0.92, 0.95, 1.0)
    bg.inputs[1].default_value = 0.70


def make_mat(name, params):
    r, g, b, a, rough, metallic = params
    mat = bpy.data.materials.new(name=name)
    if mat.node_tree is None:
        mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (r, g, b, a)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metallic
    return mat


def import_stl_group(mesh_dir: Path):
    for name, params in MATERIALS.items():
        stl_path = mesh_dir / f"{name}.stl"
        if not stl_path.exists():
            continue
        bpy.ops.wm.stl_import(filepath=str(stl_path))
        obj = bpy.context.selected_objects[0]
        obj.name = name
        obj.data.materials.append(make_mat(name, params))


def setup_studio(center: Vector):
    # Studio seamless ground floor aligned to wheel contact plane
    bpy.ops.mesh.primitive_plane_add(size=1200, location=(center.x, center.y, -p.GROUND_CLEARANCE - 0.05))
    plane = bpy.context.active_object
    plane.data.materials.append(make_mat("floor", (0.86, 0.88, 0.90, 1.0, 0.65, 0.0)))

    # 3-Point studio lighting (Key, Fill, Rim)
    lights = [
        ("Key", (center.x + 140, center.y + 160, center.z + 180), 450000, 140),
        ("Fill", (center.x - 160, center.y + 80, center.z + 110), 220000, 180),
        ("Rim", (center.x - 60, center.y - 170, center.z + 130), 260000, 140),
    ]
    for lname, loc, energy, size in lights:
        ldata = bpy.data.lights.new(name=lname, type="AREA")
        ldata.energy = energy
        ldata.size = size
        lobj = bpy.data.objects.new(name=lname, object_data=ldata)
        bpy.context.collection.objects.link(lobj)
        lobj.location = loc
        direction = center - lobj.location
        lobj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def setup_camera(name: str, cam_loc: Vector, target: Vector, lens: float = 48.0):
    cdata = bpy.data.cameras.new(name)
    cdata.lens = lens
    cobj = bpy.data.objects.new(name, cdata)
    bpy.context.collection.objects.link(cobj)
    cobj.location = cam_loc
    direction = target - cam_loc
    cobj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cobj


def render_view(cam_loc: Vector, target: Vector, out_path: Path, lens: float = 48.0, name: str = "Cam"):
    setup_camera(name, cam_loc, target, lens)
    bpy.context.scene.render.filepath = str(out_path)
    bpy.ops.render.render(write_still=True)


def main():
    if "--" in sys.argv:
        args = sys.argv[sys.argv.index("--") + 1 :]
    else:
        args = []

    preview_root = Path(args[0]) if len(args) > 0 else Path("/tmp/smartrover_preview")
    out_dir = Path(args[1]) if len(args) > 1 else REPO_ROOT / "3d-printing/renders"
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Assembled View (Front 3/4 with optional camera brace) + local interactive Studio .blend
    if (preview_root / "assembled").exists():
        print(f"Rendering assembled view into {out_dir}...")
        reset_scene()
        import_stl_group(preview_root / "assembled")
        center = Vector((0.0, 2.0, 15.0))
        setup_studio(center)
        render_view(
            Vector((160.0, 180.0, 118.0)),
            center,
            out_dir / "cad_assembled_front.png",
            lens=48.0,
            name="StudioCamera_Front",
        )

        # Save local interactive .blend file (gitignored) for optional GUI inspection
        blend_path = REPO_ROOT / "mechanical/SmartRover_Studio.blend"
        blend_path.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))
        print(f"Interactive Blender project saved to {blend_path}")

    # 2. Exploded Unibody View (shows single chassis, empty battery bay, battery, PCB, wheels & optional brace)
    if (preview_root / "exploded").exists():
        print(f"Rendering exploded view into {out_dir}...")
        reset_scene()
        import_stl_group(preview_root / "exploded")
        center_exp = Vector((0.0, 4.0, 56.0))
        setup_studio(center_exp)
        render_view(Vector((280.0, 305.0, 225.0)), center_exp, out_dir / "cad_exploded.png", lens=45.0)

    print(f"Renders saved to {out_dir}")


if __name__ == "__main__":
    main()
