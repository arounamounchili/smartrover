//! Domain data types and hardware abstraction traits.

use crate::config::control::TOF_CLEAR_DISTANCE_MM;

/// Speed scaling presets for manual driving.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum SpeedGear {
    Crawl = 1,
    Normal = 2,
    Sport = 3,
}

impl SpeedGear {
    pub(crate) const fn from_u8(val: u8) -> Self {
        match val {
            1 => Self::Crawl,
            3 => Self::Sport,
            _ => Self::Normal,
        }
    }

    pub(crate) const fn power_scale(self) -> f32 {
        match self {
            Self::Crawl => 0.45,
            Self::Normal => 0.75,
            Self::Sport => 1.00,
        }
    }
}

/// High-level operational state of the rover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum RoverMode {
    Idle = 0,
    ManualDrive = 1,
    Braking = 2,
    AdasBraking = 3,
    Disconnected = 4,
    Tipped = 5,
    LowBattery = 6,
}

impl RoverMode {
    pub(crate) const fn from_u8(val: u8) -> Self {
        match val {
            0 => Self::Idle,
            1 => Self::ManualDrive,
            2 => Self::Braking,
            3 => Self::AdasBraking,
            4 => Self::Disconnected,
            5 => Self::Tipped,
            6 => Self::LowBattery,
            _ => Self::Disconnected,
        }
    }
}

/// Normalized teleoperation command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DriveCommand {
    /// Forward/reverse demand in `[-1.0, 1.0]`.
    pub(crate) throttle: f32,
    /// Left/right turn demand in `[-1.0, 1.0]`.
    pub(crate) steering: f32,
    pub(crate) gear: SpeedGear,
    pub(crate) brake: bool,
    pub(crate) adas_enabled: bool,
}

impl Default for DriveCommand {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            steering: 0.0,
            gear: SpeedGear::Normal,
            brake: false,
            adas_enabled: true,
        }
    }
}

/// Sensor readings sampled at the start of a control tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SensorFrame {
    pub(crate) distance_mm: u16,
    pub(crate) battery_pct: i32,
    pub(crate) accel_g: (f32, f32, f32),
    pub(crate) tof_healthy: bool,
    pub(crate) imu_healthy: bool,
}

impl Default for SensorFrame {
    fn default() -> Self {
        Self {
            distance_mm: TOF_CLEAR_DISTANCE_MM,
            battery_pct: 100,
            accel_g: (0.0, 0.0, 1.0),
            tof_healthy: true,
            imu_healthy: true,
        }
    }
}

/// Target H-bridge output state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct WheelOutput {
    /// Left motor duty cycle in `-1000..=1000`.
    pub(crate) left_duty: i32,
    /// Right motor duty cycle in `-1000..=1000`.
    pub(crate) right_duty: i32,
    pub(crate) active_brake: bool,
}

impl WheelOutput {
    pub(crate) const fn coast() -> Self {
        Self {
            left_duty: 0,
            right_duty: 0,
            active_brake: false,
        }
    }

    pub(crate) const fn brake() -> Self {
        Self {
            left_duty: 0,
            right_duty: 0,
            active_brake: true,
        }
    }
}

/// Status snapshot published after each control tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct TelemetrySnapshot {
    pub(crate) mode: RoverMode,
    pub(crate) adas_intervening: bool,
    pub(crate) left_duty: i32,
    pub(crate) right_duty: i32,
    pub(crate) distance_mm: u16,
    pub(crate) battery_pct: i32,
    pub(crate) tof_healthy: bool,
    pub(crate) imu_healthy: bool,
}

/// Differential motor driver interface.
pub(crate) trait MotorPort {
    fn apply(&mut self, output: WheelOutput);
}

/// Status LED indicator interface.
pub(crate) trait StatusPort {
    fn indicate(&mut self, mode: RoverMode, wheel_output: WheelOutput, tick: u32);
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_speed_gear_conversions_and_scaling() {
        assert_eq!(SpeedGear::from_u8(1), SpeedGear::Crawl);
        assert_eq!(SpeedGear::from_u8(2), SpeedGear::Normal);
        assert_eq!(SpeedGear::from_u8(3), SpeedGear::Sport);
        assert_eq!(SpeedGear::from_u8(99), SpeedGear::Normal);

        assert!((SpeedGear::Crawl.power_scale() - 0.45).abs() < 1e-5);
        assert!((SpeedGear::Normal.power_scale() - 0.75).abs() < 1e-5);
        assert!((SpeedGear::Sport.power_scale() - 1.00).abs() < 1e-5);

        assert_eq!(RoverMode::from_u8(0), RoverMode::Idle);
        assert_eq!(RoverMode::from_u8(6), RoverMode::LowBattery);
        assert_eq!(RoverMode::from_u8(99), RoverMode::Disconnected);
    }

    #[test]
    fn test_default_ports_and_wheel_outputs() {
        let cmd = DriveCommand::default();
        assert_eq!(cmd.throttle, 0.0);
        assert_eq!(cmd.steering, 0.0);
        assert_eq!(cmd.gear, SpeedGear::Normal);
        assert!(!cmd.brake);
        assert!(cmd.adas_enabled);

        let sf = SensorFrame::default();
        assert_eq!(sf.distance_mm, TOF_CLEAR_DISTANCE_MM);
        assert_eq!(sf.battery_pct, 100);
        assert_eq!(sf.accel_g, (0.0, 0.0, 1.0));

        assert_eq!(
            WheelOutput::coast(),
            WheelOutput {
                left_duty: 0,
                right_duty: 0,
                active_brake: false
            }
        );
        assert_eq!(
            WheelOutput::brake(),
            WheelOutput {
                left_duty: 0,
                right_duty: 0,
                active_brake: true
            }
        );
    }
}
