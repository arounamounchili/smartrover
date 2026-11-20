"""Parametric build123d modules for SmartRover."""

import math
from build123d import (
    Axis,
    Box,
    BuildPart,
    BuildSketch,
    Cone,
    Cylinder,
    GeomType,
    Locations,
    Mode,
    Plane,
    PolarLocations,
    Polygon,
    Rectangle,
    RectangleRounded,
    chamfer,
    extrude,
    loft,
)
import params as p


def tof_local_to_global(x_l: float, y_l: float, z_l: float) -> tuple[float, float, float]:
    """Map local ToF board coordinates (y_l=0 at tilted PCB back-face, +y_l along optical axis) to rover frame."""
    y_seat = p.CHASSIS_Y_FRONT - p.TOF_SEAT_BACK_OFFSET_Y
    z_seat = p.TOF_CENTER_Z
    rad = math.radians(p.TOF_TILT_DEG)
    return (
        x_l,
        y_seat + y_l * math.cos(rad) - z_l * math.sin(rad),
        z_seat + y_l * math.sin(rad) + z_l * math.cos(rad),
    )


def _teardrop_points(cy: float, cz: float, radius: float, flat_top_w: float = 2.0, n_arc: int = 36) -> list[tuple[float, float]]:
    """Return 2D (Y, Z) polygon points for a horizontal bore with a 45-deg teardrop top."""
    # Lower 270 deg of the bore circle (135 deg -> 270 deg -> 405 deg)
    pts = []
    start_deg = 135.0
    end_deg = 405.0
    for i in range(n_arc + 1):
        ang = math.radians(start_deg + (end_deg - start_deg) * i / n_arc)
        pts.append((cy + radius * math.cos(ang), cz + radius * math.sin(ang)))
    # 45-deg roof lines from the top tangents meeting at a flat bridge of width flat_top_w
    t_xy = radius * math.sqrt(0.5)
    half_flat = min(flat_top_w / 2.0, t_xy * 0.85)
    peak_dz = t_xy + (t_xy - half_flat)
    pts.append((cy + half_flat, cz + peak_dz))
    pts.append((cy - half_flat, cz + peak_dz))
    return pts


def build_unibody_chassis():
    """Single-piece unibody chassis with battery bay, teardrop motor bores,
    +10 deg recessed VL53L0X pocket, open-top Qwiic wire gutter, and LED channels.
    """
    half_w = p.CHASSIS_WIDTH / 2.0  # 31.0 mm
    chassis_len = p.CHASSIS_Y_FRONT - p.CHASSIS_Y_REAR  # 115.5 mm
    chassis_yc = (p.CHASSIS_Y_FRONT + p.CHASSIS_Y_REAR) / 2.0  # +2.25 mm

    with BuildPart() as part:
        # 1. Main body block
        with Locations((0, chassis_yc, p.CHASSIS_HEIGHT / 2.0)):
            Box(p.CHASSIS_WIDTH, chassis_len, p.CHASSIS_HEIGHT)

        # 2. 45-deg outer chamfers (all self-supporting when printed flat on Z=0)
        # Vertical outer corners (3.8 mm front, 3.2 mm rear to preserve the rear grommet flange seat)
        front_vert_corners = [
            e for e in part.edges().filter_by(Axis.Z)
            if abs(abs(e.center().X) - half_w) < 0.1 and e.center().Y > 0
        ]
        rear_vert_corners = [
            e for e in part.edges().filter_by(Axis.Z)
            if abs(abs(e.center().X) - half_w) < 0.1 and e.center().Y < 0
        ]
        if front_vert_corners:
            chamfer(front_vert_corners, length=3.8)
        if rear_vert_corners:
            chamfer(rear_vert_corners, length=3.2)

        # Lower front and rear skid chamfers
        front_bottom = [
            e for e in part.edges()
            if abs(e.center().Y - p.CHASSIS_Y_FRONT) < 0.1 and abs(e.center().Z) < 0.1
        ]
        if front_bottom:
            chamfer(front_bottom, length=3.0)

        rear_bottom = [
            e for e in part.edges()
            if abs(e.center().Y - p.CHASSIS_Y_REAR) < 0.1 and abs(e.center().Z) < 0.1
        ]
        if rear_bottom:
            chamfer(rear_bottom, length=4.0)

        # Top outer perimeter chamfer
        top_outer = [
            e for e in part.edges()
            if abs(e.center().Z - p.CHASSIS_HEIGHT) < 0.1
        ]
        if top_outer:
            chamfer(top_outer, length=1.2)

        # 45-deg beveled side recess between wheels (Z=3.8..11.4, below S101/J102 pocket)
        for sx in (-1, 1):
            crease_x_out = sx * (half_w + 0.5)
            crease_x_in = sx * (half_w - 1.3)
            with BuildSketch(Plane.YZ.offset(crease_x_out)):
                Polygon((-23.0, 3.8), (23.0, 3.8), (23.0, 11.4), (-23.0, 11.4))
            with BuildSketch(Plane.YZ.offset(crease_x_in)):
                Polygon((-21.2, 5.6), (21.2, 5.6), (21.2, 9.6), (-21.2, 9.6))
            loft(mode=Mode.SUBTRACT)

        # 3. Upper PCB cradle pocket (45.7 x 80.7 mm, R2.8 corners; PCB top sits 0.4 mm below rim)
        with Locations((0, 0, p.PCB_SEAT_Z)):
            with BuildSketch(Plane.XY):
                RectangleRounded(
                    p.PCB_WIDTH + 0.7,
                    p.PCB_LENGTH + 0.7,
                    p.PCB_CORNER_RADIUS + 0.3,
                )
            extrude(amount=p.CHASSIS_HEIGHT - p.PCB_SEAT_Z + 1.0, mode=Mode.SUBTRACT)

        # 4. Central 1S LiPo battery bay (35 x 62 mm between front/rear PCB bulkheads)
        bay_h = p.PCB_SEAT_Z - p.FLOOR_THICK
        with Locations((0, 0.0, p.FLOOR_THICK + bay_h / 2.0)):
            Box(p.BATT_BAY_W, p.BATT_BAY_L, bay_h + 0.5, mode=Mode.SUBTRACT)

        # Under-PCB clearance recess for J202 Tag-Connect guide pins (X=15.2..20.3, Y=12.2..19.8)
        with Locations((18.0, 16.0, 17.6)):
            Box(7.0, 9.5, 5.5, mode=Mode.SUBTRACT)

        # Floor ventilation and battery strap slots
        for vy in (-18.0, -6.0, 6.0, 18.0):
            with Locations((0, vy, p.FLOOR_THICK / 2.0)):
                Box(24.0, 4.2, p.FLOOR_THICK + 1.0, mode=Mode.SUBTRACT)

        # 5. Right-side S101 (power switch) and J102 (top-entry JST-PH battery header) pocket
        # Leaves 6.7 mm beside the PCB edge (X=22.5..29.2) for routing the 6.0x4.5 mm JST-PH plug
        with Locations((23.1, -2.5, 17.75)):
            Box(12.2, 24.0, 9.5, mode=Mode.SUBTRACT)
        # Outer rim notch opposite J102 (Y=-13.8..-5.4) for connector access
        with Locations((30.1, -9.6, 20.2)):
            Box(3.0, 8.4, 4.6, mode=Mode.SUBTRACT)

        # 6. Right-side J310 (SM04B I2C/ToF) plug bay and open-top Qwiic cable gutter
        # 6a. Plug and bend bay beside J310 (Y=21.5..31.5, X=22.5..29.2, Z>=15.6)
        with Locations((25.85, 26.5, 19.05)):
            Box(6.7, 10.0, 6.9, mode=Mode.SUBTRACT)
        # 6b. Outer rim notch beside J310 for plug insertion
        with Locations((30.1, 26.5, 20.6)):
            Box(3.0, 8.2, 3.8, mode=Mode.SUBTRACT)
        # 6c. Longitudinal open-top gutter (X=23.0..29.2, width 6.2 mm > 5.8 mm JST-SH plug)
        gutter_w = p.GUTTER_X_OUT - p.GUTTER_X_IN
        gutter_xc = (p.GUTTER_X_IN + p.GUTTER_X_OUT) / 2.0
        gutter_y_start = 26.0
        gutter_y_end = 56.0
        gutter_h = p.CHASSIS_HEIGHT - p.GUTTER_FLOOR_Z + 1.0
        with Locations((gutter_xc, (gutter_y_start + gutter_y_end) / 2.0, p.GUTTER_FLOOR_Z + gutter_h / 2.0)):
            Box(gutter_w, gutter_y_end - gutter_y_start, gutter_h, mode=Mode.SUBTRACT)
        # 6d. Transverse front wire trench behind 4.0 mm front wall (X=13.6..29.2, Y=49.8..56.0)
        with Locations((21.4, 52.9, p.GUTTER_FLOOR_Z + gutter_h / 2.0)):
            Box(15.6, 6.2, gutter_h, mode=Mode.SUBTRACT)
        # 6e. Vertical exit shaft (6.4 x 6.0 mm) from right VL53L0X Qwiic bay into transverse trench
        with Locations((16.8, 53.0, 16.5)):
            Box(6.4, 6.0, 12.0, mode=Mode.SUBTRACT)

        # 7. +10 deg upward-tilted recessed front VL53L0X STEMMA QT pocket
        # Tilts backward as Z rises, so the pocket prints upright without supports
        with Locations(tof_local_to_global(0.0, 4.0, 0.0)):
            Box(
                p.TOF_BOARD_WIDTH + 0.8,
                8.0,
                p.TOF_BOARD_HEIGHT + 0.8,
                rotation=(p.TOF_TILT_DEG, 0, 0),
                mode=Mode.SUBTRACT,
            )

        # Left and right Qwiic connector bays with 45-deg self-supporting roofs (X=+-18.3..+-12.6)
        for sx in (-1, 1):
            qwiic_pts = [
                (sx * 12.6, 6.4),
                (sx * 18.3, 6.4),
                (sx * 18.3, 15.0),
                (sx * 12.6, 20.7),
            ]
            with BuildSketch(Plane.XZ.offset(-52.2)):
                Polygon(*qwiic_pts)
            extrude(amount=-9.0, mode=Mode.SUBTRACT)  # extrudes along +Y from Y=52.2 through Y=61.2

        # 45-deg upper brow relief above the ToF pocket (avoids top overhang and widens FoV)
        brow_pts = [
            (53.8, 19.6),
            (61.0, 19.6),
            (61.0, 23.0),
            (50.4, 23.0),
        ]
        with BuildSketch(Plane.YZ):
            Polygon(*brow_pts)
        extrude(amount=(p.TOF_BOARD_WIDTH + 0.8) / 2.0, both=True, mode=Mode.SUBTRACT)

        # 4x M2.5 blind corner pilots normal to the +10 deg seat (20.32 x 12.70 mm spacing)
        # 4.2 mm depth behind seat leaves >=1.6 mm wall ahead of the front motor bore
        for sx in (-1, 1):
            for sz in (-1, 1):
                hx = sx * (p.TOF_HOLE_SPACING_X / 2.0)
                hz = sz * (p.TOF_HOLE_SPACING_Z / 2.0)
                with Locations(tof_local_to_global(hx, -1.8, hz)):
                    Cylinder(
                        radius=p.M25_PILOT_DIAM / 2.0,
                        height=4.8,
                        rotation=(-90.0 + p.TOF_TILT_DEG, 0, 0),
                        mode=Mode.SUBTRACT,
                    )

        # 8. 4x Horizontal teardrop motor bores and flush outer rubber-grommet counterbores
        bore_r = p.MOTOR_DIAM / 2.0
        flange_r = p.MOTOR_GROMMET_FLANGE_DIAM / 2.0
        for sx in (-1, 1):
            for ay in (p.WHEELBASE_FRONT_Y, p.WHEELBASE_REAR_Y):
                # Main teardrop bore from outer wall inward (depth = MOTOR_BORE_DEPTH)
                bore_pts = _teardrop_points(ay, p.AXLE_Z, bore_r, flat_top_w=1.4)
                with BuildSketch(Plane.YZ.offset(sx * (half_w + 0.5))):
                    Polygon(*bore_pts)
                extrude(amount=-sx * (p.MOTOR_BORE_DEPTH + 0.5), mode=Mode.SUBTRACT)

                # Flush outer counterbore for QUARKZMAN 12.6 mm grommet flange (stops inward wheel-press load)
                flange_pts = _teardrop_points(ay, p.AXLE_Z, flange_r, flat_top_w=1.8)
                with BuildSketch(Plane.YZ.offset(sx * (half_w + 0.5))):
                    Polygon(*flange_pts)
                extrude(amount=-sx * (p.MOTOR_GROMMET_FLANGE_DEPTH + 0.5), mode=Mode.SUBTRACT)

                # 0.6 mm x 45 deg lead-in cone behind the flange seat
                with Locations((sx * (half_w - p.MOTOR_GROMMET_FLANGE_DEPTH - 0.3), ay, p.AXLE_Z)):
                    Cone(
                        bottom_radius=bore_r + 0.6,
                        top_radius=bore_r,
                        height=0.6,
                        rotation=(0, -90 * sx, 0),
                        mode=Mode.SUBTRACT,
                    )

                # Optional bottom M2.5 x 4 mm radial lock-screw pilot (for bare 10 mm motors)
                with Locations((sx * p.MOTOR_LOCK_X, ay, 2.3)):
                    Cylinder(radius=p.M25_PILOT_DIAM / 2.0, height=5.0, mode=Mode.SUBTRACT)

        # 9. Center motor-wire cavities (45-deg peaked ceiling) and open-top edge chimneys to J301..J308
        for ay in (p.WHEELBASE_FRONT_Y, p.WHEELBASE_REAR_Y):
            cavity_pts = [
                (ay - 4.2, p.AXLE_Z - 3.6),
                (ay + 4.2, p.AXLE_Z - 3.6),
                (ay + 4.2, p.AXLE_Z + 1.8),
                (ay, p.AXLE_Z + 6.0),
                (ay - 4.2, p.AXLE_Z + 1.8),
            ]
            with BuildSketch(Plane.YZ):
                Polygon(*cavity_pts)
            extrude(amount=8.0, both=True, mode=Mode.SUBTRACT)

        for sx in (-1, 1):
            # Open-top motor wire chimneys (X in [+-3.6, +-9.6], Z=7.5..22.5) outside the PCB Y=+-40 edges,
            # aligned with the motor rear exits (Y=+43.5 / -44.5) and top SMD pads J301..J308 (X=+-4.5)
            with Locations((sx * 6.6, 43.0, 15.0)):
                Box(6.0, 7.0, 15.0, mode=Mode.SUBTRACT)
            with Locations((sx * 6.6, -43.25, 15.0)):
                Box(6.0, 7.5, 15.0, mode=Mode.SUBTRACT)

        # 10. Front headlight (D201/D202) and rear taillight (D203/D204) open-top light channels
        for sx in (-1, 1):
            f_y_len = p.CHASSIS_Y_FRONT - p.LED_FRONT_Y + 2.0
            with Locations((sx * p.LED_FRONT_X, (p.LED_FRONT_Y + p.CHASSIS_Y_FRONT) / 2.0 + 0.5, 21.4)):
                Box(3.8, f_y_len, 2.0, mode=Mode.SUBTRACT)
            r_y_len = p.LED_REAR_Y - p.CHASSIS_Y_REAR + 2.0
            with Locations((sx * p.LED_REAR_X, (p.LED_REAR_Y + p.CHASSIS_Y_REAR) / 2.0 - 0.5, 21.4)):
                Box(3.8, r_y_len, 2.0, mode=Mode.SUBTRACT)

        # 11. M2.5 self-tapping pilot holes
        # Main PCB H201 / H202 into front and rear bulkheads (10 mm deep)
        for hy in (p.PCB_HOLE_FRONT_Y, p.PCB_HOLE_REAR_Y):
            with Locations((0, hy, p.PCB_SEAT_Z - 5.0)):
                Cylinder(radius=p.M25_PILOT_DIAM / 2.0, height=10.5, mode=Mode.SUBTRACT)

        # Optional camera brace M2.5 pilots on left and right top rims (8.5 mm deep)
        for sx in (-1, 1):
            with Locations((sx * p.CAM_BRACE_BOLT_X, p.CAM_BRACE_BOLT_Y, p.CHASSIS_HEIGHT - 4.25)):
                Cylinder(radius=p.M25_PILOT_DIAM / 2.0, height=8.8, mode=Mode.SUBTRACT)

    return part.part


def build_camera_brace():
    """Optional open-back bolt-on stabilizer for the Arducam Mini 5MP Plus (UC-367).

    Bridges over the top edge of the PCB so the rear FPGA heatsink and M2 lens-mount screws stay
    unobstructed, with 45-deg screw-relief cutouts in the rear groove lips. Drops straight over an
    installed camera and mounts with 2x M2.5 x 8 mm screws (prints upright without supports).
    """
    z_base = p.CHASSIS_HEIGHT                      # 22.0 mm
    z_foot_top = z_base + p.CAM_BRACE_FOOT_Z       # 26.0 mm
    z_arm_top = 38.2                               # 12.2 mm loft keeps arm overhangs <= 43.5 deg
    z_bridge_bot = p.CAM_TOP_Z + 0.6               # 62.2 mm (0.6 mm above Arducam PCB top)
    z_top = p.CAM_BRACE_TOP_Z                      # 71.5 mm

    # Side jaw X bounds centered on Arducam PCB X (+0.46 mm)
    cam_cx = p.CAM_PCB_X
    half_cam = p.CAM_PCB_WIDTH / 2.0               # 12.6 mm
    jaw_x_l_out = cam_cx - half_cam - 3.2          # -15.34 mm
    jaw_x_l_in = cam_cx - half_cam + 1.2           # -10.94 mm (>3.8 mm clear of M12 lens)
    jaw_x_r_in = cam_cx + half_cam - 1.2           # +11.86 mm
    jaw_x_r_out = cam_cx + half_cam + 3.2          # +16.26 mm

    jaw_y_rear = p.CAM_PCB_Y - 3.2                 # 21.8 mm (2.1 mm rear lip wall behind 2.2 mm groove)
    jaw_y_front = p.CAM_PCB_Y + 2.8                # 27.8 mm (1.7 mm front lip wall)
    jaw_yc = (jaw_y_rear + jaw_y_front) / 2.0      # 24.8 mm
    jaw_dy = jaw_y_front - jaw_y_rear              # 6.0 mm

    with BuildPart() as part:
        # 1. Left and right mounting feet on the unibody rim (Y in [11.0, 21.5], clears S101 and J310)
        for sx in (-1, 1):
            with Locations((sx * 26.9, 16.25, z_base + p.CAM_BRACE_FOOT_Z / 2.0)):
                Box(7.6, 10.5, p.CAM_BRACE_FOOT_Z)

        # 2. Swept arms (<=43.5 deg overhang) lofting from forward half of feet to vertical side jaws
        arm_specs = [
            (-26.5, -23.1, jaw_x_l_out, jaw_x_l_in),
            (23.1, 26.5, jaw_x_r_in, jaw_x_r_out),
        ]
        for bx0, bx1, tx0, tx1 in arm_specs:
            b_xc, b_w = (bx0 + bx1) / 2.0, bx1 - bx0
            t_xc, t_w = (tx0 + tx1) / 2.0, tx1 - tx0
            with BuildSketch(Plane.XY.offset(z_foot_top - 0.5)):
                with Locations((b_xc, 19.6)):
                    Rectangle(b_w, 3.8)
            with BuildSketch(Plane.XY.offset(z_arm_top)):
                with Locations((t_xc, jaw_yc)):
                    Rectangle(t_w, jaw_dy)
            loft()

            # Vertical side jaw pillar (Z = 38.2 .. z_top)
            with Locations((t_xc, jaw_yc, (z_arm_top + z_top) / 2.0)):
                Box(t_w, jaw_dy, z_top - z_arm_top)

        # 3. Vaulted top crossbar above the PCB (Z = 62.2..71.5 mm, 2.4 mm deep on PCB centerline)
        # 42.4-deg underside narrows the horizontal bridge span to 10.4 mm
        bridge_pts = [
            (jaw_x_l_out, z_bridge_bot),
            (jaw_x_l_in, z_bridge_bot),
            (cam_cx - 5.2, z_bridge_bot + 6.8),
            (cam_cx + 5.2, z_bridge_bot + 6.8),
            (jaw_x_r_in, z_bridge_bot),
            (jaw_x_r_out, z_bridge_bot),
            (jaw_x_r_out, z_top),
            (jaw_x_l_out, z_top),
        ]
        with BuildSketch(Plane.XZ.offset(-(p.CAM_PCB_Y - 1.2))):
            Polygon(*bridge_pts)
        extrude(amount=-2.4)

        # Top outer edge chamfers
        top_x_edges = [
            e for e in part.edges().filter_by(Axis.X)
            if abs(e.center().Z - z_top) < 0.2
        ]
        if top_x_edges:
            chamfer(top_x_edges, length=0.8)

        # 4. Vertical open-bottom PCB slide grooves (Y=25.0, width 2.2 mm)
        groove_z0 = z_arm_top - 1.5
        groove_z1 = z_bridge_bot
        groove_zc = (groove_z0 + groove_z1) / 2.0
        groove_h = groove_z1 - groove_z0
        for edge_x in (cam_cx - half_cam, cam_cx + half_cam):
            with Locations((edge_x, p.CAM_PCB_Y, groove_zc)):
                Box(3.0, p.CAM_PCB_THICK, groove_h, mode=Mode.SUBTRACT)

        # 5. Rear M2 lens-mount screw relief cutouts in rear groove lips (Z = 43.0..53.5 mm, 45-deg top slope)
        for sx in (-1, 1):
            x_in = cam_cx + sx * (half_cam - 2.0)
            x_out = cam_cx + sx * (half_cam + 0.2)
            relief_pts = [
                (x_in, 43.0),
                (x_out, 43.0),
                (x_out, 53.5),
                (x_in, 55.7),
            ]
            with BuildSketch(Plane.XZ.offset(-(jaw_y_rear - 0.5))):
                Polygon(*relief_pts)
            extrude(amount=-(p.CAM_PCB_Y - jaw_y_rear + 0.5), mode=Mode.SUBTRACT)

        # 6. Open-top vertical M2.5 clearance holes through feet at (+-27.0, 14.5)
        for sx in (-1, 1):
            with Locations((sx * p.CAM_BRACE_BOLT_X, p.CAM_BRACE_BOLT_Y, z_base + p.CAM_BRACE_FOOT_Z / 2.0)):
                Cylinder(radius=p.M25_CLEAR_DIAM / 2.0, height=p.CAM_BRACE_FOOT_Z + 2.0, mode=Mode.SUBTRACT)

    return part.part


def build_wheel_rim():
    """Closed-hub 5-spoke rim (PLA/PETG) with Ø2.16 mm crush-ribbed shaft bore."""
    rim_r = p.WHEEL_RIM_OD / 2.0                         # 12.3 mm barrel
    inner_flange_r = p.WHEEL_RIM_INNER_FLANGE_OD / 2.0   # 13.0 mm chassis-side backstop flange
    outer_bead_r = p.WHEEL_RIM_OUTER_BEAD_OD / 2.0       # 12.55 mm spoke-side retention bead
    w = p.WHEEL_WIDTH                                    # 8.0 mm
    standoff = p.WHEEL_STANDOFF                          # 1.2 mm

    with BuildPart() as part:
        # Outer retention bead (spoke side) with chamfered entry ramp
        with Locations((0, 0, standoff + w - 0.5)):
            Cylinder(radius=outer_bead_r, height=1.0)
        outer_bead_edge = [
            e for e in part.edges().filter_by(GeomType.CIRCLE)
            if abs(e.center().Z - (standoff + w)) < 0.05 and abs(e.radius - outer_bead_r) < 0.05
        ]
        if outer_bead_edge:
            chamfer(outer_bead_edge, length=p.WHEEL_RIM_LIP_CHAMFER)

        # Inner backstop flange (chassis side) with 45-deg conical transition
        with Locations((0, 0, standoff + p.WHEEL_RIM_LIP_WIDTH / 2.0)):
            Cylinder(radius=inner_flange_r, height=p.WHEEL_RIM_LIP_WIDTH)
        with Locations((0, 0, standoff + p.WHEEL_RIM_LIP_WIDTH + 0.35)):
            Cone(bottom_radius=inner_flange_r, top_radius=rim_r, height=0.7)

        # Main cylindrical rim barrel
        with Locations((0, 0, standoff + w / 2.0)):
            Cylinder(radius=rim_r, height=w)

        # Hollow inner ring behind spokes (leaves 2.0 mm spoke face on build plate)
        with Locations((0, 0, standoff + (w - 2.2) / 2.0)):
            Cylinder(radius=rim_r - 2.0, height=w - 1.8, mode=Mode.SUBTRACT)

        # 5 wedge windows in outer face to form 5 spokes
        with Locations((0, 0, standoff + w - 1.5)):
            with BuildSketch(Plane.XY):
                with PolarLocations(radius=7.4, count=5):
                    Polygon((-1.75, -3.1), (2.05, -3.1), (1.25, 2.85), (-1.25, 2.85))
            extrude(amount=4.0, both=True, mode=Mode.SUBTRACT)

        # Stepped motor shaft hub: Ø5.5 mm standoff nose (Z=0..1.2) flaring at 45 deg to Ø8.6 mm hub
        with Locations((0, 0, standoff / 2.0)):
            Cylinder(radius=2.75, height=standoff)
        with Locations((0, 0, standoff + 0.775)):
            Cone(bottom_radius=2.75, top_radius=4.3, height=1.55)
        with Locations((0, 0, (standoff + 1.55 + standoff + w) / 2.0)):
            Cylinder(radius=4.3, height=w - 1.55)

        # Flat closed outer hub (4.0 mm septum above bore); 5.2 mm deep Ø2.16 mm shaft bore + 0.4 mm lead-in
        bore_depth = 5.2
        with Locations((0, 0, bore_depth / 2.0)):
            Cylinder(radius=p.SHAFT_BORE_DIAM / 2.0, height=bore_depth, mode=Mode.SUBTRACT)
        with Locations((0, 0, 0.2)):
            Cone(bottom_radius=1.48, top_radius=p.SHAFT_BORE_DIAM / 2.0, height=0.4, mode=Mode.SUBTRACT)

        r_s_grip = p.SHAFT_CRUSH_GRIP_DIAM / 2.0
        r_s_out = p.SHAFT_BORE_DIAM / 2.0 + 0.15
        r_s_mid = (r_s_grip + r_s_out) / 2.0
        s_rib_rad = r_s_out - r_s_grip
        # 3 crush ribs (Ø1.90 mm grip) merging into the bore roof at Z=5.2
        for ang in (0.0, 120.0, 240.0):
            rad = math.radians(ang)
            with Locations((r_s_mid * math.cos(rad), r_s_mid * math.sin(rad), 2.9)):
                Box(s_rib_rad, 0.55, 4.6, rotation=(0, 0, ang))

    return part.part


def build_wheel_tire():
    """Flexible TPU 95A chevron tire for the Ø34 x 8 mm wheel."""
    outer_r = p.WHEEL_OD / 2.0                                              # 17.0 mm
    inner_r = (p.WHEEL_RIM_OD - p.TIRE_INTERFERENCE) / 2.0                  # 12.225 mm
    base_r = inner_r + p.TIRE_CARCASS_THICK                                 # 14.225 mm
    inner_ch_r = (p.WHEEL_RIM_INNER_FLANGE_OD - p.TIRE_INTERFERENCE) / 2.0  # 12.925 mm
    outer_ch_r = (p.WHEEL_RIM_OUTER_BEAD_OD - p.TIRE_INTERFERENCE) / 2.0    # 12.475 mm
    lip_w = p.WHEEL_RIM_LIP_WIDTH                                           # 1.2 mm
    lead_in = p.TIRE_LEAD_IN                                                # 0.80 mm
    w = p.WHEEL_WIDTH                                                       # 8.0 mm

    with BuildPart() as part:
        # Tire carcass
        with Locations((0, 0, w / 2.0)):
            Cylinder(radius=base_r, height=w)

        # 18 directional chevron tread lugs extending from carcass to outer_r
        n_lugs = 18
        r_lug = (base_r + outer_r) / 2.0
        for i in range(n_lugs):
            ang = i * (360.0 / n_lugs)
            rad = math.radians(ang)
            with Locations((r_lug * math.cos(rad), r_lug * math.sin(rad), w / 2.0)):
                Box(3.1, (outer_r - base_r) + 2.0, w + 2.0, rotation=(18, 0, ang))

        # Trim lugs to tire width Z=[0, w] and outer radius <= outer_r
        with Locations((0, 0, w / 2.0)):
            Cylinder(radius=outer_r, height=w, mode=Mode.INTERSECT)

        # 1. Main barrel bore (stretch fit on rim barrel)
        with Locations((0, 0, w / 2.0)):
            Cylinder(radius=inner_r, height=w + 2.0, mode=Mode.SUBTRACT)

        # 2. Inner backstop flange channel (chassis side)
        with Locations((0, 0, lip_w / 2.0)):
            Cylinder(radius=inner_ch_r, height=lip_w, mode=Mode.SUBTRACT)
        with Locations((0, 0, lip_w + 0.35)):
            Cone(bottom_radius=inner_ch_r, top_radius=inner_r, height=0.7, mode=Mode.SUBTRACT)

        # 3. Outer retention bead channel (spoke side)
        with Locations((0, 0, w - 0.5)):
            Cylinder(radius=outer_ch_r, height=1.0, mode=Mode.SUBTRACT)
        with Locations((0, 0, w - 1.0 - 0.25)):
            Cone(bottom_radius=inner_r, top_radius=outer_ch_r, height=0.5, mode=Mode.SUBTRACT)

        # 4. 45-deg lead-in chamfers on both mouths for slip-on assembly
        with Locations((0, 0, lead_in / 2.0)):
            Cone(bottom_radius=inner_ch_r + lead_in, top_radius=inner_ch_r, height=lead_in, mode=Mode.SUBTRACT)
        with Locations((0, 0, w - lead_in / 2.0)):
            Cone(bottom_radius=outer_ch_r, top_radius=outer_ch_r + lead_in, height=lead_in, mode=Mode.SUBTRACT)

    return part.part
