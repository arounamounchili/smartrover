"""Parametric dimensions for the SmartRover unibody chassis and wheels (in millimeters).

Coordinate convention (assembled rover):
  X = 0 : longitudinal centerline (-X = Left, +X = Right)
  Y = 0 : main PCB center (+Y = Front, -Y = Rear)
  Z = 0 : bottom face of UnibodyChassis (wheels extend to Z = -GROUND_CLEARANCE)
"""

# --- Hardware Envelope (from hardware/SmartRover.kicad_pcb) ---
PCB_WIDTH = 45.0
PCB_LENGTH = 80.0
PCB_THICKNESS = 1.6
PCB_CORNER_RADIUS = 2.5
PCB_SEAT_Z = 20.0               # 2.2 mm floor + 17.8 mm bay (~15 mm clear below solder pins)

# M2.5 PCB mounting holes (H201 front, H202 rear)
PCB_HOLE_FRONT_Y = 34.925
PCB_HOLE_REAR_Y = -35.00
M25_PILOT_DIAM = 2.2            # self-tapping M2.5 in PLA/PETG (or 3.2 for heat-set insert)
M25_CLEAR_DIAM = 2.75

# Key PCB component locations (rover-centered coordinates from SmartRover.kicad_pcb)
LED_FRONT_X = 18.48
LED_FRONT_Y = 38.75
LED_REAR_X = 18.48
LED_REAR_Y = -38.75

USBC_X = -6.41                  # J101 vertical USB-C
USBC_Y = -23.22
UART_X = -10.61                 # J201 4-pin vertical UART header
UART_Y = 10.32
RESET_BTN_X = -13.85            # S201 tactile RESET
RESET_BTN_Y = 16.08
BOOT_BTN_X = -13.87             # S202 tactile BOOT
BOOT_BTN_Y = -16.08
SWITCH_X = 17.91                # S101 right-side SPDT power switch (body Y in [-4.4, +8.3])
SWITCH_CENTER_Y = 2.00
BATT_CONN_X = 19.76             # J102 right-side JST-PH battery header
BATT_CONN_Y = -9.60
TOF_CONN_Y = 26.30              # J310 right-side SM04B JST-SH I2C/ToF header

# Arducam Mini UC-367 (24.33 x 34.09 x 1.74 mm PCB, offset forward of J309 @ Y=+20.0)
CAM_HEADER_Y = 20.00
CAM_PCB_X = 0.46
CAM_PCB_Y = 25.00
CAM_PCB_WIDTH = 25.2            # 24.33 mm PCB + 0.87 mm total slide clearance
CAM_PCB_HEIGHT = 34.2
CAM_PCB_THICK = 2.2             # vertical slide groove width for 1.74 mm Arducam PCB
CAM_SOCKET_HEIGHT = 5.8         # height above PCB top (male pins seated into J309 female header)
CAM_BOTTOM_Z = PCB_SEAT_Z + PCB_THICKNESS + CAM_SOCKET_HEIGHT  # 27.4 mm
CAM_CENTER_Z = CAM_BOTTOM_Z + CAM_PCB_HEIGHT / 2.0             # 44.5 mm
CAM_TOP_Z = CAM_BOTTOM_Z + CAM_PCB_HEIGHT                      # 61.6 mm

# Front ToF breakout mounting (Adafruit VL53L0X STEMMA QT PID 3317)
# Board is 25.40 x 17.78 mm (1.00 x 0.70 in) with 20.32 x 12.70 mm M2.5 corner holes.
# Tilted +10 deg upward (leans back as Z rises -> zero overhangs upright) and recessed behind nose lip.
TOF_HOLE_SPACING_X = 20.32
TOF_HOLE_SPACING_Z = 12.70
TOF_BOARD_WIDTH = 25.40
TOF_BOARD_HEIGHT = 17.78
TOF_BOARD_THICK = 1.60
TOF_CENTER_Z = 11.2
TOF_SEAT_BACK_OFFSET_Y = 4.6    # tilted back-plane 4.6 mm behind CHASSIS_Y_FRONT (~1.5 mm protective recess)
TOF_TILT_DEG = 10.0             # +10 deg upward pitch eliminates floor-cone reflections

# Motors & rubber grommet mounting (QUARKZMAN 10 mm OD x 6 mm ID x 11 mm flanged grommet or bare 10 mm motors)
MOTOR_DIAM = 10.15              # press-fit bore for 10.0 mm rubber grommet / bare 10 mm motor can
MOTOR_BORE_DEPTH = 24.5         # per-side bore depth from outer wall inward (fits motors up to 26 mm long)
MOTOR_GROMMET_FLANGE_DIAM = 12.80  # flush outer counterbore for grommet flange (resists wheel press force)
MOTOR_GROMMET_FLANGE_DEPTH = 1.35
MOTOR_LOCK_X = 25.0             # optional bottom M2.5 x 4 mm radial lock-screw X position
SHAFT_BORE_DIAM = 2.16          # main bore + 3 crush ribs (1.90 mm grip) for 2.0 mm motor shaft
SHAFT_CRUSH_GRIP_DIAM = 1.90

# --- Compact Unibody Stance & Wheel Proportions ---
GROUND_CLEARANCE = 8.0
WHEEL_OD = 34.0
WHEEL_RIM_OD = 24.6             # outer barrel OD under TPU tire
WHEEL_RIM_INNER_FLANGE_OD = 26.0   # chassis-side solid backstop flange
WHEEL_RIM_OUTER_BEAD_OD = 25.1     # spoke-side low-profile retention bead (0.25 mm radial height)
WHEEL_RIM_LIP_WIDTH = 1.2
WHEEL_RIM_LIP_CHAMFER = 0.60
WHEEL_WIDTH = 8.0
WHEEL_STANDOFF = 1.2            # minimal standoff so short 4-5 mm motor shafts engage deeply
TIRE_CARCASS_THICK = 2.0        # compliant 2.0 mm TPU carcass wall
TIRE_INTERFERENCE = 0.15        # 0.15 mm stretch fit on barrel
TIRE_LEAD_IN = 0.80             # 45 deg entry ramp on tire mouths

WHEELBASE_FRONT_Y = 43.5
WHEELBASE_REAR_Y = -44.5
AXLE_Z = (WHEEL_OD / 2.0) - GROUND_CLEARANCE  # 17.0 - 8.0 = 9.0 mm

# Single-piece Unibody Chassis dimensions (62.0 x 115.5 x 22.0 mm)
CHASSIS_WIDTH = 62.0            # X in [-31.0, +31.0]
CHASSIS_Y_FRONT = 60.0          # integrates +10 deg recessed ToF nose & headlight slits
CHASSIS_Y_REAR = -55.5          # clears rear outer grommet flange (-50.9) + 3.2 mm corner bevel cleanly
CHASSIS_HEIGHT = 22.0           # top rim wraps 0.4 mm above PCB top (21.6 mm)
FLOOR_THICK = 2.2

# Under-PCB 1S LiPo battery bay (fits up to 34 x 60 x 13.5 mm packs, e.g. 603048 / 803048 / 103450)
BATT_BAY_W = 35.0               # 34 mm max battery + 1.0 mm slide clearance
BATT_BAY_L = 62.0               # clear rectangular span between H201 (+34.9) & H202 (-35.0) standoffs
BATT_GHOST_W = 34.0
BATT_GHOST_L = 60.0
BATT_GHOST_H = 12.0             # verified 12 mm thick LiPo with >= 3 mm clearance to under-PCB solder pins

# Right-side open-top Qwiic cable gutter (J310 -> front recessed ToF bay)
GUTTER_X_IN = 23.0              # 0.5 mm past PCB edge (+22.5)
GUTTER_X_OUT = 29.2             # 6.2 mm wide (>5.8 mm JST-SH connector head), 1.8 mm solid outer wall
GUTTER_FLOOR_Z = 16.8           # clears front-right teardrop motor bore top (Z=15.48) by 1.32 mm solid roof

# Optional slim bolt-on Camera Brace interface (on solid top deck behind J310)
CAM_BRACE_BOLT_X = 27.0
CAM_BRACE_BOLT_Y = 14.5         # clears S101 (Y<=+8.3) & J310 bay (Y>=+22.0) on both sides
CAM_BRACE_FOOT_Z = 4.0
CAM_BRACE_TOP_Z = 71.5          # vaulted top arch above Arducam PCB; keeps rear heatsink and M2 screws open
