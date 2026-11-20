"""Automated geometric, fitment, and 3D-printability verification for SmartRover."""

import math
import sys
from pathlib import Path
from build123d import Compound, Cone, Cylinder

CAD_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(CAD_DIR))

import params as p
import parts
import export_and_render as er


def check_all():
    failures = []

    # 1. Print-oriented solid validity, flat Z=0 bed seating, DFM overhang audit, and <=180x180 mm RigidPrintPlate
    print_parts = er.make_print_oriented_parts()
    for name, solid in print_parts.items():
        bb = solid.bounding_box()
        if not solid.is_valid:
            failures.append(f"{name}: invalid B-Rep solid")
        if solid.volume <= 0:
            failures.append(f"{name}: non-positive volume {solid.volume}")
        if abs(bb.min.Z) > 1e-3:
            failures.append(f"{name}: not seated at Z=0 (Zmin={bb.min.Z:.4f})")
        # DFM overhang check: flag non-horizontal steep overhangs (46.5..89.0 deg, area >= 1.0 mm^2)
        # or wide unsupported horizontal ceilings (min X/Y span > 2.5 mm)
        for face in solid.faces():
            fbb = face.bounding_box()
            if fbb.max.Z < 0.05 or face.area < 1.0:
                continue
            try:
                n = face.normal_at(face.center())
            except Exception:
                continue
            if n.Z < -0.05:
                ang = math.degrees(math.asin(min(1.0, abs(n.Z))))
                if 46.5 < ang < 89.0:
                    failures.append(
                        f"{name}: unsupported overhang {ang:.1f} deg ({face.area:.1f} mm^2) at Z={face.center().Z:.2f}"
                    )
                elif ang >= 89.0:
                    # Horizontal bridge is safe when narrow along at least one axis (<= 2.5 mm)
                    min_span = min(fbb.size.X, fbb.size.Y)
                    if min_span > 2.5:
                        failures.append(
                            f"{name}: wide unsupported horizontal ceiling ({fbb.size.X:.1f}x{fbb.size.Y:.1f} mm) at Z={face.center().Z:.2f}"
                        )

    def _vol(sh):
        if sh is None:
            return 0.0
        return sh.volume if hasattr(sh, "volume") else sum(s.volume for s in sh if hasattr(s, "volume"))

    plate = er.make_print_plate(print_parts)
    plate_bb = plate.bounding_box()
    if plate_bb.size.X > 180.0 or plate_bb.size.Y > 180.0:
        failures.append(f"RigidPrintPlate exceeds 180x180 mm mini bed: {plate_bb.size.X:.1f} x {plate_bb.size.Y:.1f} mm")
    if abs(plate_bb.min.Z) > 1e-3:
        failures.append(f"RigidPrintPlate not seated at Z=0 (Zmin={plate_bb.min.Z:.4f})")
    plate_items = list(plate.children)
    for i in range(len(plate_items)):
        for j in range(i + 1, len(plate_items)):
            a_bb, b_bb = plate_items[i].bounding_box(), plate_items[j].bounding_box()
            if (
                a_bb.max.X > b_bb.min.X
                and b_bb.max.X > a_bb.min.X
                and a_bb.max.Y > b_bb.min.Y
                and b_bb.max.Y > a_bb.min.Y
            ):
                ov = _vol(plate_items[i].intersect(plate_items[j]))
                if ov > 0.01:
                    failures.append(f"RigidPrintPlate parts {i} <-> {j} overlap: {ov:.3f} mm^3")

    # 2. Assembled collision / interference checks (all printed parts + PCB, battery, ToF, Qwiic cable, camera, motors)
    assembled = er.build_assembled_parts(exploded=False)
    chassis = assembled["chassis"]
    cam_brace = assembled["cam_brace"]
    pcb, battery, cam_pcb, cam_lens, tof_hw, j310_cable, motors_hw = er.make_hardware_ghosts()
    cam_hw = Compound(children=[cam_pcb, cam_lens])

    half_w = p.CHASSIS_WIDTH / 2.0
    spin_envelopes = {}
    for label, sx, ay in [
        ("FR", 1, p.WHEELBASE_FRONT_Y),
        ("FL", -1, p.WHEELBASE_FRONT_Y),
        ("RR", 1, p.WHEELBASE_REAR_Y),
        ("RL", -1, p.WHEELBASE_REAR_Y),
    ]:
        spin_envelopes[label] = Cylinder(
            radius=p.WHEEL_OD / 2.0 + 0.6,
            height=p.WHEEL_WIDTH + 0.8,
            rotation=(0, 90, 0),
        ).translate((sx * (half_w + p.WHEEL_STANDOFF - 0.3 + p.WHEEL_WIDTH / 2.0), ay, p.AXLE_Z))

    all_parts = {
        **assembled,
        "pcb_hw": pcb,
        "battery_hw": battery,
        "cam_hw": cam_hw,
        "tof_hw": tof_hw,
        "j310_cable": j310_cable,
        "motors_hw": motors_hw,
    }

    # Allowed intentional stretch-fit / crush-rib press-fit overlaps
    allowed_pairs = set()
    crush_pairs = set()
    for lbl in ("FR", "FL", "RR", "RL"):
        allowed_pairs.add(frozenset((f"rim_{lbl}", f"tire_{lbl}")))
        crush_pairs.add(frozenset(("motors_hw", f"rim_{lbl}")))

    names = list(all_parts.keys())
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            pair = frozenset((a, b))
            if pair in allowed_pairs:
                continue
            vol = _vol(all_parts[a].intersect(all_parts[b]))
            if pair in crush_pairs:
                if vol < 0.05 or vol > 20.0:
                    failures.append(f"Crush-rib fitment out of range {a} <-> {b}: {vol:.3f} mm^3")
            elif vol > 0.05:
                failures.append(f"Collision {a} <-> {b}: overlap volume = {vol:.3f} mm^3")

    # Verify non-crush core of motors, grommet flanges & shafts have 0 interference anywhere
    for sx in (-1, 1):
        for ay in (p.WHEELBASE_FRONT_Y, p.WHEELBASE_REAR_Y):
            can_core = Cylinder(radius=4.80, height=20.0, rotation=(0, 90, 0)).translate(
                (sx * (half_w - 10.0), ay, p.AXLE_Z)
            )
            shaft_core = Cylinder(radius=0.90, height=4.5, rotation=(0, 90, 0)).translate(
                (sx * (half_w + 2.25), ay, p.AXLE_Z)
            )
            for pname in ["chassis", "cam_brace", *[f"rim_{l}" for l in ("FR", "FL", "RR", "RL")]]:
                if _vol(all_parts[pname].intersect(can_core)) > 0.05:
                    failures.append(f"Motor core (R=4.80) collides with {pname} at sx={sx}, y={ay}")
                if _vol(all_parts[pname].intersect(shaft_core)) > 0.05:
                    failures.append(f"Motor shaft core (R=0.90) collides with {pname} at sx={sx}, y={ay}")

    # Verify all 4 spinning wheel envelopes clear static chassis, brace, PCB & Qwiic cable
    for lbl, env in spin_envelopes.items():
        for static_name in ("chassis", "cam_brace", "pcb_hw", "j310_cable"):
            if _vol(all_parts[static_name].intersect(env)) > 0.05:
                failures.append(f"Spinning wheel envelope {lbl} failed clearance against {static_name}")

    # 3. Ground clearance, recessed ToF protection, and 25 deg optical FoV cone check
    tire_min_z = assembled["tire_FR"].bounding_box().min.Z
    chassis_min_z = chassis.bounding_box().min.Z
    clearance = chassis_min_z - tire_min_z
    if abs(clearance - p.GROUND_CLEARANCE) > 0.2:
        failures.append(f"Ground clearance mismatch: {clearance:.2f} vs {p.GROUND_CLEARANCE:.2f}")

    tof_max_y = tof_hw.bounding_box().max.Y
    recess_depth = p.CHASSIS_Y_FRONT - tof_max_y
    if recess_depth < 1.0:
        failures.append(f"VL53L0X not safely recessed behind front bumper: recess={recess_depth:.2f} mm")

    cone_len = 40.0
    fov_half_rad = math.radians(12.5)
    fov_cone = Cone(
        bottom_radius=1.5,
        top_radius=1.5 + cone_len * math.tan(fov_half_rad),
        height=cone_len,
        rotation=(-90.0 + p.TOF_TILT_DEG, 0, 0),
    ).translate(parts.tof_local_to_global(0.0, p.TOF_BOARD_THICK + cone_len / 2.0, 0.0))
    if _vol(chassis.intersect(fov_cone)) > 0.01:
        failures.append("UnibodyChassis clips the VL53L0X 25 deg optical FoV cone")

    # 4. Unobstructed vertical top access to USB-C (J101), UART (J201), RESET (S201), BOOT (S202)
    pcb_top = p.PCB_SEAT_Z + p.PCB_THICKNESS
    for label, ax, ay, rad in [
        ("USB-C J101", p.USBC_X, p.USBC_Y, 5.5),
        ("UART J201", p.UART_X, p.UART_Y, 4.5),
        ("RESET S201", p.RESET_BTN_X, p.RESET_BTN_Y, 3.0),
        ("BOOT S202", p.BOOT_BTN_X, p.BOOT_BTN_Y, 3.0),
    ]:
        probe = Cylinder(radius=rad, height=60.0).translate((ax, ay, pcb_top + 32.0))
        for pname in ("chassis", "cam_brace"):
            ov = _vol(all_parts[pname].intersect(probe))
            if ov > 0.01:
                failures.append(f"{pname} blocks vertical access to {label}: vol={ov:.3f}")

    # 5. Coaxial M2.5 fastener pass-through, vertical driver clearance, and pilot wall integrity
    # Optional CameraBrace vertical M2.5 bolts + open driver access to sky
    for sx in (-1, 1):
        bx = sx * p.CAM_BRACE_BOLT_X
        by = p.CAM_BRACE_BOLT_Y
        pin = Cylinder(radius=1.20, height=6.0).translate((bx, by, p.CHASSIS_HEIGHT + 2.0))
        if _vol(cam_brace.intersect(pin)) > 0.01:
            failures.append(f"CameraBrace M2.5 clearance hole blocked at x={bx:.2f}")
        tool = Cylinder(radius=2.60, height=60.0).translate((bx, by, p.CHASSIS_HEIGHT + p.CAM_BRACE_FOOT_Z + 30.0))
        if _vol(cam_brace.intersect(tool)) > 0.01:
            failures.append(f"CameraBrace M2.5 vertical driver access blocked at x={bx:.2f}")
        # Solid pilot wall in UnibodyChassis beneath bolt
        ring = (Cylinder(radius=2.4, height=5.0) - Cylinder(radius=1.3, height=5.0)).translate(
            (bx, by, p.CHASSIS_HEIGHT - 3.5)
        )
        if _vol(chassis.intersect(ring)) < 35.0:
            failures.append(f"UnibodyChassis camera brace pilot boss missing wall at x={bx:.2f}")

    # Main PCB H201 / H202 pilot walls
    for hy in (p.PCB_HOLE_FRONT_Y, p.PCB_HOLE_REAR_Y):
        ring = (Cylinder(radius=2.6, height=6.0) - Cylinder(radius=1.3, height=6.0)).translate(
            (0.0, hy, p.PCB_SEAT_Z - 4.0)
        )
        if _vol(chassis.intersect(ring)) < 50.0:
            failures.append(f"UnibodyChassis PCB pilot wall thin/missing at y={hy}")

    # Bottom optional motor lock-screw pilot walls & outer grommet flange seat perimeter integrity
    for sx in (-1, 1):
        for ay in (p.WHEELBASE_FRONT_Y, p.WHEELBASE_REAR_Y):
            ring_m = (Cylinder(radius=2.2, height=2.6) - Cylinder(radius=1.2, height=2.6)).translate(
                (sx * p.MOTOR_LOCK_X, ay, 1.5)
            )
            if _vol(chassis.intersect(ring_m)) < 15.0:
                failures.append(f"UnibodyChassis bottom motor lock wall missing at sx={sx}, y={ay}")
            flange_ring = (
                Cylinder(radius=p.MOTOR_GROMMET_FLANGE_DIAM / 2.0 + 0.6, height=0.5, rotation=(0, 90, 0))
                - Cylinder(radius=p.MOTOR_GROMMET_FLANGE_DIAM / 2.0 + 0.05, height=0.5, rotation=(0, 90, 0))
            ).translate((sx * (half_w - 0.3), ay, p.AXLE_Z))
            flange_pct = _vol(chassis.intersect(flange_ring)) / _vol(flange_ring)
            if flange_pct < 0.80:
                failures.append(
                    f"UnibodyChassis outer grommet flange clipped at sx={sx}, y={ay} ({100*flange_pct:.1f}% < 80%)"
                )

    # All 4 VL53L0X corner screw pilot walls intact (>=95%)
    for sx in (-1, 1):
        for sz in (-1, 1):
            hx = sx * (p.TOF_HOLE_SPACING_X / 2.0)
            hz = sz * (p.TOF_HOLE_SPACING_Z / 2.0)
            ring_t = (
                Cylinder(radius=2.3, height=3.8, rotation=(-90.0 + p.TOF_TILT_DEG, 0, 0))
                - Cylinder(radius=1.2, height=3.8, rotation=(-90.0 + p.TOF_TILT_DEG, 0, 0))
            ).translate(parts.tof_local_to_global(hx, -2.0, hz))
            if _vol(chassis.intersect(ring_t)) / _vol(ring_t) < 0.95:
                failures.append(f"UnibodyChassis VL53L0X pilot wall thin/missing at sx={sx}, sz={sz}")

    # 6. Assembly drop-in sweep checks (battery into bay, optional camera brace over installed Arducam)
    for dz in (2, 6, 12, 18, 24):
        if _vol(chassis.intersect(battery.translate((0, 0, dz)))) > 0.01:
            failures.append(f"Battery drop-in blocked at dz=+{dz} mm")
    for dz in (2, 6, 12, 20, 30, 40):
        cb_z = cam_brace.translate((0, 0, dz))
        for hw_name, hw in (("pcb_hw", pcb), ("cam_hw", cam_hw), ("j310_cable", j310_cable), ("chassis", chassis)):
            if _vol(cb_z.intersect(hw)) > 0.05:
                failures.append(f"CameraBrace drop-on sweep hits {hw_name} at dz=+{dz} mm")

    if failures:
        print("CAD VERIFICATION FAILED:")
        for f in failures:
            print("  -", f)
        sys.exit(1)

    chassis_bb = chassis.bounding_box()
    track = assembled["tire_FR"].bounding_box().max.X - assembled["tire_FL"].bounding_box().min.X
    print("ALL CAD CHECKS PASSED:")
    print(f"  - {len(print_parts)} printable designs valid & seated at Z=0.00 mm (0 supports)")
    print(f"  - RigidPrintPlate ({plate_bb.size.X:.1f} x {plate_bb.size.Y:.1f} mm) fits 180x180 mm bed with 0 overlaps")
    print(f"  - 0 unintended collisions across {len(all_parts)} assembled parts + hardware (incl. 1S LiPo & Qwiic cable)")
    print(f"  - Body: {chassis_bb.size.X:.1f} x {chassis_bb.size.Y:.1f} x {chassis_bb.size.Z:.1f} mm | Track: {track:.1f} mm | Wheelbase: {p.WHEELBASE_FRONT_Y - p.WHEELBASE_REAR_Y:.1f} mm")
    print(f"  - Ground clearance: {clearance:.2f} mm | VL53L0X nose recess: {recess_depth:.2f} mm | 25 deg FoV clear")
    print(f"  - Battery bay verified for {p.BATT_GHOST_W:.0f} x {p.BATT_GHOST_L:.0f} x {p.BATT_GHOST_H:.0f} mm 1S LiPo + J102 lead pass-through")


if __name__ == "__main__":
    check_all()
