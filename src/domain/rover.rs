//! Control loop coordinator combining safety supervision and drive kinematics.

use crate::domain::kinematics::{mix_arcade_drive, slew_limit};
use crate::domain::ports::{
    DriveCommand, MotorPort, RoverMode, SensorFrame, StatusPort, TelemetrySnapshot, WheelOutput,
};
use crate::domain::safety::SafetySupervisor;

pub(crate) struct RoverController {
    safety: SafetySupervisor,
    current_left_duty: i32,
    current_right_duty: i32,
    tick_count: u32,
}

impl RoverController {
    pub(crate) const fn new() -> Self {
        Self {
            safety: SafetySupervisor::new(),
            current_left_duty: 0,
            current_right_duty: 0,
            tick_count: 0,
        }
    }

    pub(crate) fn step(
        &mut self,
        sensors: SensorFrame,
        command: DriveCommand,
        ticks_since_last_cmd: u32,
        motors: &mut impl MotorPort,
        indicators: &mut impl StatusPort,
    ) -> TelemetrySnapshot {
        self.tick_count = self.tick_count.wrapping_add(1);

        let safety_eval = self
            .safety
            .evaluate(&sensors, command, ticks_since_last_cmd);

        let wheel_output = if safety_eval.safe_command.brake {
            self.current_left_duty = 0;
            self.current_right_duty = 0;
            WheelOutput::brake()
        } else if matches!(
            safety_eval.override_mode,
            Some(RoverMode::Tipped | RoverMode::LowBattery | RoverMode::Disconnected)
        ) {
            self.current_left_duty = 0;
            self.current_right_duty = 0;
            WheelOutput::coast()
        } else {
            let (target_left, target_right) = mix_arcade_drive(
                safety_eval.safe_command.throttle,
                safety_eval.safe_command.steering,
                safety_eval.safe_command.gear,
            );
            self.current_left_duty = slew_limit(self.current_left_duty, target_left);
            self.current_right_duty = slew_limit(self.current_right_duty, target_right);
            WheelOutput {
                left_duty: self.current_left_duty,
                right_duty: self.current_right_duty,
                active_brake: false,
            }
        };

        let mode = if let Some(override_mode) = safety_eval.override_mode {
            override_mode
        } else if wheel_output.left_duty == 0 && wheel_output.right_duty == 0 {
            RoverMode::Idle
        } else {
            RoverMode::ManualDrive
        };

        motors.apply(wheel_output);
        indicators.indicate(mode, wheel_output, self.tick_count);

        TelemetrySnapshot {
            mode,
            adas_intervening: safety_eval.adas_intervened,
            left_duty: wheel_output.left_duty,
            right_duty: wheel_output.right_duty,
            distance_mm: sensors.distance_mm,
            battery_pct: sensors.battery_pct,
            tof_healthy: sensors.tof_healthy,
            imu_healthy: sensors.imu_healthy,
        }
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use crate::domain::ports::SpeedGear;

    #[derive(Default)]
    struct MockMotors {
        last_output: Option<WheelOutput>,
    }

    impl MotorPort for MockMotors {
        fn apply(&mut self, output: WheelOutput) {
            self.last_output = Some(output);
        }
    }

    #[derive(Default)]
    struct MockLeds {
        last_mode: Option<RoverMode>,
        last_wheel_output: Option<WheelOutput>,
    }

    impl StatusPort for MockLeds {
        fn indicate(&mut self, mode: RoverMode, wheel_output: WheelOutput, _tick: u32) {
            self.last_mode = Some(mode);
            self.last_wheel_output = Some(wheel_output);
        }
    }

    #[test]
    fn test_controller_manual_drive_idle_and_ebrake() {
        let mut controller = RoverController::new();
        let mut motors = MockMotors::default();
        let mut leds = MockLeds::default();
        let sensors = SensorFrame::default();

        let idle = controller.step(sensors, DriveCommand::default(), 0, &mut motors, &mut leds);
        assert_eq!(idle.mode, RoverMode::Idle);

        let cmd = DriveCommand {
            throttle: 1.0,
            steering: 0.0,
            gear: SpeedGear::Sport,
            brake: false,
            adas_enabled: true,
        };
        let snap = controller.step(sensors, cmd, 0, &mut motors, &mut leds);
        assert_eq!(snap.mode, RoverMode::ManualDrive);
        assert!(motors.last_output.unwrap().left_duty > 0);
        assert_eq!(leds.last_mode, Some(RoverMode::ManualDrive));

        let brake_cmd = DriveCommand { brake: true, ..cmd };
        let snap = controller.step(sensors, brake_cmd, 0, &mut motors, &mut leds);
        assert_eq!(snap.mode, RoverMode::Braking);
        assert_eq!(motors.last_output.unwrap(), WheelOutput::brake());
    }

    #[test]
    fn test_end_to_end_driving_lifecycle_integration() {
        let mut controller = RoverController::new();
        let mut motors = MockMotors::default();
        let mut leds = MockLeds::default();
        let sensors = SensorFrame::default();

        let snap = controller.step(
            sensors,
            DriveCommand::default(),
            100,
            &mut motors,
            &mut leds,
        );
        assert_eq!(snap.mode, RoverMode::Disconnected);
        assert_eq!(motors.last_output.unwrap(), WheelOutput::coast());

        let full_fwd = DriveCommand {
            throttle: 1.0,
            steering: 0.0,
            gear: SpeedGear::Sport,
            brake: false,
            adas_enabled: true,
        };
        let s1 = controller.step(sensors, full_fwd, 0, &mut motors, &mut leds);
        let s2 = controller.step(sensors, full_fwd, 0, &mut motors, &mut leds);
        let s3 = controller.step(sensors, full_fwd, 0, &mut motors, &mut leds);
        let s4 = controller.step(sensors, full_fwd, 0, &mut motors, &mut leds);
        assert_eq!(s1.left_duty, 250);
        assert_eq!(s2.left_duty, 500);
        assert_eq!(s3.left_duty, 750);
        assert_eq!(s4.left_duty, 1000);
        assert_eq!(s4.mode, RoverMode::ManualDrive);

        let wall_sensors = SensorFrame {
            distance_mm: 100,
            ..sensors
        };
        let _ = controller.step(wall_sensors, full_fwd, 0, &mut motors, &mut leds);
        let s_adas = controller.step(wall_sensors, full_fwd, 0, &mut motors, &mut leds);
        assert_eq!(s_adas.mode, RoverMode::AdasBraking);
        assert!(s_adas.adas_intervening);
        assert_eq!(motors.last_output.unwrap(), WheelOutput::brake());
    }

    #[test]
    fn test_tipped_and_low_battery_cut_duty_immediately_and_ramp_on_recovery() {
        let mut controller = RoverController::new();
        let mut motors = MockMotors::default();
        let mut leds = MockLeds::default();
        let ok_sensors = SensorFrame::default();
        let full_fwd = DriveCommand {
            throttle: 1.0,
            steering: 0.0,
            gear: SpeedGear::Sport,
            brake: false,
            adas_enabled: true,
        };

        // Ramp up to 1000 duty over 4 ticks
        for _ in 0..4 {
            let _ = controller.step(ok_sensors, full_fwd, 0, &mut motors, &mut leds);
        }
        assert_eq!(motors.last_output.unwrap().left_duty, 1000);

        // Tipped while moving: immediate coast to 0 (no slew-down)
        let freefall = SensorFrame {
            accel_g: (0.0, 0.0, 0.1),
            ..ok_sensors
        };
        let tipped = controller.step(freefall, full_fwd, 0, &mut motors, &mut leds);
        assert_eq!(tipped.mode, RoverMode::Tipped);
        assert_eq!(tipped.left_duty, 0);
        assert_eq!(tipped.right_duty, 0);
        assert_eq!(motors.last_output.unwrap(), WheelOutput::coast());

        // Recover upright with neutral inputs, then drive again: must slew up from 0
        let _ = controller.step(
            ok_sensors,
            DriveCommand::default(),
            0,
            &mut motors,
            &mut leds,
        );
        let r1 = controller.step(ok_sensors, full_fwd, 0, &mut motors, &mut leds);
        let r2 = controller.step(ok_sensors, full_fwd, 0, &mut motors, &mut leds);
        assert_eq!(r1.mode, RoverMode::ManualDrive);
        assert_eq!(r1.left_duty, 250);
        assert_eq!(r2.left_duty, 500);

        // LowBattery while moving at 500 duty: immediate coast to 0
        let low_batt = SensorFrame {
            battery_pct: 10,
            ..ok_sensors
        };
        let batt_cut = controller.step(low_batt, full_fwd, 0, &mut motors, &mut leds);
        assert_eq!(batt_cut.mode, RoverMode::LowBattery);
        assert_eq!(batt_cut.left_duty, 0);
        assert_eq!(batt_cut.right_duty, 0);
        assert_eq!(motors.last_output.unwrap(), WheelOutput::coast());

        // Recover battery >= 20%: must slew up from 0
        let recovered_batt = SensorFrame {
            battery_pct: 25,
            ..ok_sensors
        };
        let b1 = controller.step(recovered_batt, full_fwd, 0, &mut motors, &mut leds);
        assert_eq!(b1.mode, RoverMode::ManualDrive);
        assert_eq!(b1.left_duty, 250);
    }

    #[test]
    fn test_disconnect_recovery_ramps_up_from_zero_via_slew_limiter() {
        let mut controller = RoverController::new();
        let mut motors = MockMotors::default();
        let mut leds = MockLeds::default();
        let sensors = SensorFrame::default();
        let full_fwd = DriveCommand {
            throttle: 1.0,
            steering: 0.0,
            gear: SpeedGear::Sport,
            brake: false,
            adas_enabled: true,
        };

        let _ = controller.step(sensors, full_fwd, 0, &mut motors, &mut leds);
        let s2 = controller.step(sensors, full_fwd, 0, &mut motors, &mut leds);
        assert_eq!(s2.left_duty, 500);

        let disc = controller.step(sensors, full_fwd, 12, &mut motors, &mut leds);
        assert_eq!(disc.mode, RoverMode::Disconnected);
        assert_eq!(motors.last_output.unwrap(), WheelOutput::coast());

        let rec1 = controller.step(sensors, full_fwd, 0, &mut motors, &mut leds);
        let rec2 = controller.step(sensors, full_fwd, 0, &mut motors, &mut leds);
        assert_eq!(rec1.mode, RoverMode::ManualDrive);
        assert_eq!(rec1.left_duty, 250);
        assert_eq!(rec2.left_duty, 500);
    }

    #[test]
    fn test_adas_braking_with_pivot_steering_rotates_without_brake() {
        let mut controller = RoverController::new();
        let mut motors = MockMotors::default();
        let mut leds = MockLeds::default();
        let wall = SensorFrame {
            distance_mm: 100,
            ..Default::default()
        };
        let pivot_fwd = DriveCommand {
            throttle: 0.8,
            steering: 0.9,
            gear: SpeedGear::Sport,
            brake: false,
            adas_enabled: true,
        };

        let _ = controller.step(wall, pivot_fwd, 0, &mut motors, &mut leds);
        let snap = controller.step(wall, pivot_fwd, 0, &mut motors, &mut leds);

        assert_eq!(snap.mode, RoverMode::AdasBraking);
        assert!(snap.adas_intervening);
        let out = motors.last_output.unwrap();
        assert!(!out.active_brake);
        assert!(out.left_duty > 0);
        assert!(out.right_duty < 0);
        assert_eq!(out.left_duty, -out.right_duty);
    }
}
