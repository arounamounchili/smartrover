"""Build, validate, export (STL + STEP + Assembled STEP + PrintPlate), and prep Blender previews."""

import argparse
import math
import sys
from pathlib import Path

from build123d import (
    Axis,
    Box,
    BuildPart,
    BuildSketch,
    Color,
    Compound,
    Cylinder,
    Locations,
    Plane,
    RectangleRounded,
    export_step,
    export_stl,
    extrude,
)

CAD_DIR = Path(__file__).resolve().parent
REPO_ROOT = CAD_DIR.parent.parent
OUT_DIR = REPO_ROOT / "3d-printing"
sys.path.insert(0, str(CAD_DIR))

import params as p
import parts


def _center_xy_drop_z(shape):
    bb = shape.bounding_box()
    cx = (bb.min.X + bb.max.X) / 2.0
    cy = (bb.min.Y + bb.max.Y) / 2.0
    return shape.translate((-cx, -cy, -bb.min.Z))


def make_print_oriented_parts():
    """Return dict of {name: part_in_print_orientation} (all flat on Z=0, zero supports)."""
    chassis = _center_xy_drop_z(parts.build_unibody_chassis())
    rim = _center_xy_drop_z(parts.build_wheel_rim().rotate(Axis.X, 180))
    cam_brace = _center_xy_drop_z(parts.build_camera_brace())
    tire = _center_xy_drop_z(parts.build_wheel_tire())

    return {
        "01_unibody_chassis": chassis,
        "02_wheel_rim_x4": rim,
        "03_camera_brace_optional": cam_brace,
        "04_wheel_tire_tpu_x4": tire,
    }


def _build_motors_ghost(x_face: float):
    """Build 4x reference 10 mm motors with flush grommet flanges and Ø2.0 mm shafts seated at +-x_face."""
    motor_len = 20.0
    shaft_len = 4.5
    with BuildPart() as motors:
        for sx in (-1, 1):
            for ay in (p.WHEELBASE_FRONT_Y, p.WHEELBASE_REAR_Y):
                with Locations((sx * (x_face - motor_len / 2.0), ay, p.AXLE_Z)):
                    Cylinder(radius=4.98, height=motor_len, rotation=(0, 90, 0))
                with Locations((sx * (x_face - p.MOTOR_GROMMET_FLANGE_DEPTH / 2.0), ay, p.AXLE_Z)):
                    Cylinder(radius=6.25, height=p.MOTOR_GROMMET_FLANGE_DEPTH - 0.1, rotation=(0, 90, 0))
                with Locations((sx * (x_face + shaft_len / 2.0), ay, p.AXLE_Z)):
                    Cylinder(radius=1.0, height=shaft_len, rotation=(0, 90, 0))
    return motors.part


def make_hardware_ghosts():
    """Real-scale PCB (with under-PCB pins), 1S LiPo battery, Arducam UC-367, VL53L0X, Qwiic cable, and 4 motors."""
    pcb_top = p.PCB_SEAT_Z + p.PCB_THICKNESS  # 21.6 mm

    with BuildPart() as pcb:
        # 45 x 80 x 1.6 mm FR4 board
        with Locations((0, 0, p.PCB_SEAT_Z)):
            with BuildSketch(Plane.XY):
                RectangleRounded(p.PCB_WIDTH, p.PCB_LENGTH, p.PCB_CORNER_RADIUS)
            extrude(amount=p.PCB_THICKNESS)
        # Vertical USB-C (J101)
        with Locations((p.USBC_X, p.USBC_Y, pcb_top + 3.5)):
            Box(9.0, 7.5, 7.0)
        # ESP32-S3-MINI-1
        with Locations((-11.74, 0.0, pcb_top + 1.2)):
            Box(15.4, 20.5, 2.4)
        # J201 4-pin vertical UART header + under-PCB solder pins (2.6 mm below PCB)
        with Locations((p.UART_X, p.UART_Y, pcb_top + 4.2)):
            Box(10.5, 2.6, 8.4)
        with Locations((p.UART_X, p.UART_Y, p.PCB_SEAT_Z - 1.3)):
            Box(10.2, 2.2, 2.6)
        # S201 RESET & S202 BOOT tactile buttons
        with Locations((p.RESET_BTN_X, p.RESET_BTN_Y, pcb_top + 1.1)):
            Box(4.5, 4.5, 2.2)
        with Locations((p.BOOT_BTN_X, p.BOOT_BTN_Y, pcb_top + 1.1)):
            Box(4.5, 4.5, 2.2)
        # J309 8-pin Arducam female socket + under-PCB solder pins (2.6 mm below PCB)
        with Locations((0.01, p.CAM_HEADER_Y, pcb_top + p.CAM_SOCKET_HEIGHT / 2.0)):
            Box(20.32, 2.54, p.CAM_SOCKET_HEIGHT)
        with Locations((0.01, p.CAM_HEADER_Y, p.PCB_SEAT_Z - 1.3)):
            Box(19.0, 2.2, 2.6)
        # S101 right-side SPDT power switch + under-PCB pins (down to Z = 15.6 mm)
        with Locations((p.SWITCH_X, p.SWITCH_CENTER_Y, 22.8)):
            Box(6.4, 12.6, 14.4)
        # J102 right-side JST-PH battery header + mated plug + under-PCB pins
        with Locations((p.BATT_CONN_X, p.BATT_CONN_Y, pcb_top + 4.4)):
            Box(5.5, 6.0, 8.8)
        with Locations((19.21, p.BATT_CONN_Y, p.PCB_SEAT_Z - 1.4)):
            Box(1.5, 3.4, 2.8)
        # J310 right-side SM04B I2C header + mated SHR-04V-S-B JST-SH plug
        with Locations((20.8, p.TOF_CONN_Y, pcb_top + 1.6)):
            Box(6.6, 6.8, 3.2)

    with BuildPart() as battery:
        # Exact 1S LiPo pack (34 x 60 x 12 mm) seated cleanly inside the 35 x 62 mm battery bay
        with Locations((0, 0.0, p.FLOOR_THICK + 0.1 + p.BATT_GHOST_H / 2.0)):
            Box(p.BATT_GHOST_W, p.BATT_GHOST_L, p.BATT_GHOST_H)

    with BuildPart() as j310_cable:
        # 1. Battery JST-PH lead routing from battery bay (+X) out into concealed right J102 pocket
        with Locations((20.5, p.BATT_CONN_Y, p.FLOOR_THICK + 12.2)):
            Box(7.0, 4.0, 2.2)
        # 2. Continuous 4-wire Qwiic cable sweep: J310 plug exit -> open right gutter -> transverse turn -> VL53L0X right Qwiic port
        # Exit bend out of J310 (+X into right gutter at X=26.1)
        with Locations((25.8, p.TOF_CONN_Y, 21.0)):
            Box(3.4, 4.4, 3.2)
        # Drop & run forward along right gutter (above front-right motor bore Z>=16.8)
        with Locations((26.1, 40.0, 19.4)):
            Box(3.4, 28.0, 3.0)
        # Transverse turn behind 4.0 mm solid front bumper wall (X=16.8..26.1, Y=52.9)
        with Locations((21.4, 52.9, 19.4)):
            Box(10.8, 3.2, 3.0)
        # Drop vertically into right VL53L0X Qwiic bay (X=16.8, Y=53.0, Z=12.0..19.4)
        with Locations((16.8, 53.0, 15.8)):
            Box(3.4, 3.2, 7.6)
        # 3. 4x Motor wire pairs routing up through open-top edge chimneys to PCB top pads J301..J308
        for sx in (-1, 1):
            # Front FL / FR motor leads (chimney X in [+-3.6, +-9.6], Y=39.5..46.5 -> straight onto J301..J304 @ X=+-4.5)
            with Locations((sx * 5.2, 43.0, 16.0)):
                Box(2.4, 2.6, 12.6)
            with Locations((sx * 5.2, 40.6, 22.1)):
                Box(2.4, 5.2, 0.8)
            # Rear RL / RR motor leads (chimney X in [+-3.6, +-9.6], Y=-47.0..-40.0 -> straight onto J305..J308 @ X=+-4.5)
            with Locations((sx * 5.2, -43.5, 16.0)):
                Box(2.4, 2.6, 12.6)
            with Locations((sx * 5.2, -40.8, 22.1)):
                Box(2.4, 5.6, 0.8)

    with BuildPart() as cam_pcb:
        # Exact UC-367 Arducam PCB (24.33 x 34.09 x 1.74 mm) centered at (0.46, +25.0)
        with Locations((p.CAM_PCB_X, p.CAM_PCB_Y, p.CAM_CENTER_Z)):
            Box(24.33, 1.74, p.CAM_PCB_HEIGHT)

    with BuildPart() as cam_lens:
        # Black M12 lens housing + barrel facing +Y front
        with Locations((p.CAM_PCB_X, p.CAM_PCB_Y + 3.0, p.CAM_BOTTOM_Z + 21.0)):
            Box(15.0, 5.0, 15.0)
        with Locations((p.CAM_PCB_X, p.CAM_PCB_Y + 10.0, p.CAM_BOTTOM_Z + 21.0)):
            Cylinder(radius=7.5, height=14.0, rotation=(90, 0, 0))
        # Real UC-367 Rev.F rear aluminum FPGA heatsink (~15 x 15 x 6.5 mm protruding -Y)
        with Locations((p.CAM_PCB_X, p.CAM_PCB_Y - 0.87 - 3.25, p.CAM_BOTTOM_Z + 12.0)):
            Box(15.0, 6.5, 15.0)
        # Real UC-367 Rev.F rear M2 pan-head lens-mount screws (18 mm pitch, Ø4.2 x 2.5 mm protruding -Y)
        for sx in (-1, 1):
            with Locations((p.CAM_PCB_X + sx * 9.0, p.CAM_PCB_Y - 0.87 - 1.25, p.CAM_BOTTOM_Z + 21.0)):
                Cylinder(radius=2.1, height=2.5, rotation=(90, 0, 0))

    with BuildPart() as tof:
        with Locations(parts.tof_local_to_global(0.0, p.TOF_BOARD_THICK / 2.0, 0.0)):
            Box(
                p.TOF_BOARD_WIDTH,
                p.TOF_BOARD_THICK,
                p.TOF_BOARD_HEIGHT,
                rotation=(p.TOF_TILT_DEG, 0, 0),
            )

    motors_part = _build_motors_ghost(p.CHASSIS_WIDTH / 2.0)

    return pcb.part, battery.part, cam_pcb.part, cam_lens.part, tof.part, j310_cable.part, motors_part


def build_assembled_parts(exploded: bool = False) -> dict:
    """Build and position all printable chassis and wheel parts in the rover frame."""
    exp = 16.0 if exploded else 0.0

    assembled = {
        "chassis": parts.build_unibody_chassis(),
        "cam_brace": parts.build_camera_brace().translate((0, 0, exp * 5.6)),
    }

    rim_base = parts.build_wheel_rim()
    tire_base = parts.build_wheel_tire()

    x_mount = p.CHASSIS_WIDTH / 2.0 + exp * 2.1
    for lbl, sx, ay in (
        ("FL", -1, p.WHEELBASE_FRONT_Y),
        ("FR", 1, p.WHEELBASE_FRONT_Y),
        ("RL", -1, p.WHEELBASE_REAR_Y),
        ("RR", 1, p.WHEELBASE_REAR_Y),
    ):
        rot_y = 90 if sx == 1 else -90
        assembled[f"rim_{lbl}"] = rim_base.rotate(Axis.Y, rot_y).translate((sx * x_mount, ay, p.AXLE_Z))
        assembled[f"tire_{lbl}"] = tire_base.rotate(Axis.Y, rot_y).translate(
            (sx * (x_mount + p.WHEEL_STANDOFF + exp * 1.1), ay, p.AXLE_Z)
        )

    return assembled


def make_assembled_compound(exploded: bool = False):
    """Build complete colored Compound and Blender STL groups for the SmartRover assembly."""
    exp = 16.0 if exploded else 0.0
    asm = build_assembled_parts(exploded=exploded)

    asm["chassis"].label = "01_UnibodyChassis"
    asm["chassis"].color = Color(0.14, 0.15, 0.17)
    asm["cam_brace"].label = "03_CameraBrace_Optional"
    asm["cam_brace"].color = Color(0.16, 0.17, 0.19)
    children = [asm["chassis"], asm["cam_brace"]]

    wheel_rims, wheel_tires = [], []
    for idx, lbl in enumerate(("FL", "FR", "RL", "RR"), start=1):
        r = asm[f"rim_{lbl}"]
        r.label = f"02_WheelRim_{idx}"
        r.color = Color(0.82, 0.85, 0.88)
        wheel_rims.append(r)

        t = asm[f"tire_{lbl}"]
        t.label = f"04_WheelTire_TPU_{idx}"
        t.color = Color(0.06, 0.06, 0.07)
        wheel_tires.append(t)

    children.extend(wheel_rims + wheel_tires)

    pcb_ghost, batt_ghost, cam_pcb_ghost, cam_lens_ghost, tof_ghost, j310_cable, motor_ghosts = (
        make_hardware_ghosts()
    )
    if exploded:
        # Lift battery cleanly ABOVE chassis top rim (Z=26..38) so the empty 35x62 mm bay is clearly visible beneath it
        batt_ghost = batt_ghost.translate((0, 0, exp * 1.60))
        j310_cable = j310_cable.translate((0, 0, exp * 2.25))
        pcb_ghost = pcb_ghost.translate((0, 0, exp * 3.10))
        cam_pcb_ghost = cam_pcb_ghost.translate((0, 0, exp * 3.10))
        cam_lens_ghost = cam_lens_ghost.translate((0, 0, exp * 3.10))
        tilt_rad = math.radians(p.TOF_TILT_DEG)
        tof_ghost = tof_ghost.translate(
            (0, exp * 1.35 * math.cos(tilt_rad), exp * 1.35 * math.sin(tilt_rad))
        )
        motor_ghosts = _build_motors_ghost(p.CHASSIS_WIDTH / 2.0 + exp * 1.15)

    pcb_ghost.label = "REF_MainPCB"
    pcb_ghost.color = Color(0.05, 0.40, 0.16)
    batt_ghost.label = "REF_Battery_1S"
    batt_ghost.color = Color(0.18, 0.32, 0.60)
    cam_pcb_ghost.label = "REF_Arducam_PCB"
    cam_pcb_ghost.color = Color(0.05, 0.40, 0.16)
    cam_lens_ghost.label = "REF_Arducam_Lens"
    cam_lens_ghost.color = Color(0.08, 0.08, 0.09)
    tof_ghost.label = "REF_ToF_Sensor"
    tof_ghost.color = Color(0.15, 0.16, 0.20)
    j310_cable.label = "REF_J310_Cable"
    j310_cable.color = Color(0.85, 0.65, 0.15)
    motor_ghosts.label = "REF_Motors_10mm"
    motor_ghosts.color = Color(0.75, 0.78, 0.82)
    children.extend([pcb_ghost, batt_ghost, cam_pcb_ghost, cam_lens_ghost, tof_ghost, j310_cable, motor_ghosts])

    groups = {
        "frame_dark": [asm["chassis"], asm["cam_brace"]],
        "rim_silver": wheel_rims,
        "tire_black": wheel_tires,
        "pcb_green": [pcb_ghost, cam_pcb_ghost],
        "battery_blue": [batt_ghost],
        # Omit blocky wire-envelope collision proxies from Blender studio renders (still verified in verify_cad.py & STEP)
        "sensor_dark": [cam_lens_ghost, tof_ghost],
        "motor_metal": [motor_ghosts],
    }

    return Compound(children=children, label="SmartRover"), groups


def make_print_plate(print_parts: dict) -> Compound:
    """Arrange all rigid PLA/PETG parts cleanly on one compact <=180 x 180 mm plate (excludes TPU tires)."""
    placements = [
        ("01_unibody_chassis", (-18.0, 0.0, 0.0), None, 0),
        ("02_wheel_rim_x4", (34.0, 46.5, 0.0), None, 0),
        ("02_wheel_rim_x4", (34.0, 15.5, 0.0), None, 0),
        ("02_wheel_rim_x4", (34.0, -15.5, 0.0), None, 0),
        ("02_wheel_rim_x4", (34.0, -46.5, 0.0), None, 0),
        ("03_camera_brace_optional", (66.0, 0.0, 0.0), Axis.Z, 90),
    ]
    plate_items = []
    for name, pos, rot_ax, rot_ang in placements:
        part = print_parts[name]
        if rot_ax is not None:
            part = part.rotate(rot_ax, rot_ang)
        plate_items.append(part.translate(pos))
    return Compound(children=plate_items, label="SmartRover_RigidPrintPlate")


def export_preview_stls(preview_dir: Path, groups: dict):
    preview_dir.mkdir(parents=True, exist_ok=True)
    for path in preview_dir.glob("*.stl"):
        path.unlink()
    for gname, shapes in groups.items():
        comp = Compound(children=shapes)
        export_stl(comp, str(preview_dir / f"{gname}.stl"))


def main():
    parser = argparse.ArgumentParser(description="Export SmartRover STL/STEP models and print plate.")
    parser.add_argument(
        "--preview-dir",
        type=Path,
        default=None,
        help="Optional directory to also export grouped STLs for Blender studio rendering.",
    )
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    step_dir = OUT_DIR / "step"
    stl_dir = OUT_DIR / "stl"
    step_dir.mkdir(exist_ok=True)
    stl_dir.mkdir(exist_ok=True)

    # Remove previous exports before regenerating
    for path in list(step_dir.glob("*.step")) + list(stl_dir.glob("*.stl")):
        path.unlink()

    print("Building print-oriented SmartRover parts...")
    print_parts = make_print_oriented_parts()
    for name, solid in print_parts.items():
        bb = solid.bounding_box()
        sz = bb.size
        valid = solid.is_valid
        print(f"  {name:26s} | valid={valid} | size=({sz.X:5.1f} x {sz.Y:5.1f} x {sz.Z:5.1f}) mm | Zmin={bb.min.Z:.2f}")
        export_stl(solid, str(stl_dir / f"{name}.stl"))
        export_step(solid, str(step_dir / f"{name}.step"))

    # Full assembled CAD + exploded view + compact <=180x180 mm rigid print plate
    asm_comp, asm_groups = make_assembled_compound(exploded=False)
    exp_comp, exp_groups = make_assembled_compound(exploded=True)
    plate = make_print_plate(print_parts)

    export_step(asm_comp, str(OUT_DIR / "SmartRover_Assembled.step"))
    export_step(exp_comp, str(OUT_DIR / "SmartRover_Exploded.step"))
    export_step(plate, str(OUT_DIR / "SmartRover_RigidPrintPlate.step"))
    export_stl(plate, str(OUT_DIR / "SmartRover_RigidPrintPlate.stl"))

    if args.preview_dir is not None:
        export_preview_stls(args.preview_dir / "assembled", asm_groups)
        export_preview_stls(args.preview_dir / "exploded", exp_groups)
    print("Export complete.")


if __name__ == "__main__":
    main()
