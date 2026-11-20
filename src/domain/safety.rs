//! Safety and driver-assistance checks for manual teleoperation.

use crate::config::control::{
    ADAS_CONFIRM_TICKS, ADAS_MIN_THROTTLE_SCALE, ADAS_SLOW_DISTANCE_MM, ADAS_STOP_DISTANCE_MM,
    ADAS_STOP_RELEASE_MM, FREEFALL_MIN_G_SQ, LOW_BATT_CUTOFF_PCT, LOW_BATT_RECOVER_PCT,
    MIN_UPRIGHT_AZ_G, STICK_DEADZONE, TILT_CUTOFF_G, TILT_HYSTERESIS_TICKS, TOF_FAULT_MAX_THROTTLE,
    WATCHDOG_TIMEOUT_TICKS,
};
use crate::domain::kinematics::sanitize_unit;
use crate::domain::ports::{DriveCommand, RoverMode, SensorFrame};

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SafetyEvaluation {
    pub(crate) override_mode: Option<RoverMode>,
    pub(crate) safe_command: DriveCommand,
    pub(crate) adas_intervened: bool,
}

pub(crate) struct SafetySupervisor {
    tilt_ticks: u8,
    obstacle_ticks: u8,
    tipped_latched: bool,
    low_batt_latched: bool,
    adas_stop_latched: bool,
}

const fn stop_override(mode: RoverMode, cmd: DriveCommand) -> SafetyEvaluation {
    SafetyEvaluation {
        override_mode: Some(mode),
        safe_command: DriveCommand {
            throttle: 0.0,
            steering: 0.0,
            brake: false,
            ..cmd
        },
        adas_intervened: false,
    }
}

impl SafetySupervisor {
    pub(crate) const fn new() -> Self {
        Self {
            tilt_ticks: 0,
            obstacle_ticks: 0,
            tipped_latched: false,
            low_batt_latched: false,
            adas_stop_latched: false,
        }
    }

    pub(crate) fn evaluate(
        &mut self,
        sensors: &SensorFrame,
        cmd: DriveCommand,
        ticks_since_last_cmd: u32,
    ) -> SafetyEvaluation {
        let mut safe_cmd = DriveCommand {
            throttle: sanitize_unit(cmd.throttle),
            steering: sanitize_unit(cmd.steering),
            ..cmd
        };

        let (ax, ay, az) = sensors.accel_g;

        // Free-fall, tilt, and inversion cutoff (only update from valid IMU readings)
        if sensors.imu_healthy && ax.is_finite() && ay.is_finite() && az.is_finite() {
            let total_g_sq = ax * ax + ay * ay + az * az;
            if total_g_sq < FREEFALL_MIN_G_SQ {
                self.tipped_latched = true;
            }

            if ax.abs() > TILT_CUTOFF_G || ay.abs() > TILT_CUTOFF_G || az < MIN_UPRIGHT_AZ_G {
                self.tilt_ticks = self.tilt_ticks.saturating_add(1);
                if self.tilt_ticks >= TILT_HYSTERESIS_TICKS {
                    self.tipped_latched = true;
                }
            } else {
                self.tilt_ticks = 0;
                if self.tipped_latched
                    && total_g_sq >= FREEFALL_MIN_G_SQ
                    && safe_cmd.throttle.abs() < STICK_DEADZONE
                    && safe_cmd.steering.abs() < STICK_DEADZONE
                {
                    self.tipped_latched = false;
                }
            }
        } else {
            self.tilt_ticks = 0;
        }

        if self.tipped_latched {
            return stop_override(RoverMode::Tipped, safe_cmd);
        }

        // Low-battery cutoff with hysteresis to avoid load-sag chatter
        if sensors.battery_pct < LOW_BATT_CUTOFF_PCT {
            self.low_batt_latched = true;
        } else if sensors.battery_pct >= LOW_BATT_RECOVER_PCT {
            self.low_batt_latched = false;
        }

        if self.low_batt_latched {
            return stop_override(RoverMode::LowBattery, safe_cmd);
        }

        // Command watchdog
        if ticks_since_last_cmd >= WATCHDOG_TIMEOUT_TICKS {
            return stop_override(RoverMode::Disconnected, safe_cmd);
        }

        // Manual brake
        if safe_cmd.brake {
            return SafetyEvaluation {
                override_mode: Some(RoverMode::Braking),
                safe_command: DriveCommand {
                    throttle: 0.0,
                    steering: 0.0,
                    brake: true,
                    ..safe_cmd
                },
                adas_intervened: false,
            };
        }

        // Forward obstacle limiting and ToF fault speed limit
        let mut adas_intervened = false;
        let mut override_mode = None;

        if !sensors.tof_healthy {
            self.obstacle_ticks = 0;
            self.adas_stop_latched = false;
            if safe_cmd.adas_enabled && safe_cmd.throttle > TOF_FAULT_MAX_THROTTLE {
                safe_cmd.throttle = TOF_FAULT_MAX_THROTTLE;
                adas_intervened = true;
            }
        } else {
            if sensors.distance_mm <= ADAS_STOP_DISTANCE_MM {
                self.obstacle_ticks = self.obstacle_ticks.saturating_add(1);
                if self.obstacle_ticks >= ADAS_CONFIRM_TICKS {
                    self.adas_stop_latched = true;
                }
            } else if sensors.distance_mm > ADAS_STOP_RELEASE_MM {
                self.obstacle_ticks = 0;
                self.adas_stop_latched = false;
            } else if !self.adas_stop_latched {
                self.obstacle_ticks = 0;
            }

            if safe_cmd.adas_enabled && safe_cmd.throttle > STICK_DEADZONE {
                if self.adas_stop_latched {
                    safe_cmd.throttle = 0.0;
                    if safe_cmd.steering.abs() < STICK_DEADZONE {
                        safe_cmd.brake = true;
                    }
                    adas_intervened = true;
                    override_mode = Some(RoverMode::AdasBraking);
                } else if sensors.distance_mm < ADAS_SLOW_DISTANCE_MM {
                    let range = (ADAS_SLOW_DISTANCE_MM - ADAS_STOP_DISTANCE_MM) as f32;
                    let margin = sensors.distance_mm.saturating_sub(ADAS_STOP_DISTANCE_MM) as f32;
                    let max_forward = (margin / range).clamp(ADAS_MIN_THROTTLE_SCALE, 1.0);
                    if safe_cmd.throttle > max_forward {
                        safe_cmd.throttle = max_forward;
                        adas_intervened = true;
                    }
                }
            }
        }

        SafetyEvaluation {
            override_mode,
            safe_command: safe_cmd,
            adas_intervened,
        }
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use crate::domain::ports::SpeedGear;

    #[test]
    fn test_watchdog_stops_rover_on_disconnect() {
        let mut sup = SafetySupervisor::new();
        let sensors = SensorFrame::default();
        let cmd = DriveCommand {
            throttle: 1.0,
            steering: 0.0,
            gear: SpeedGear::Sport,
            brake: false,
            adas_enabled: true,
        };

        let eval = sup.evaluate(&sensors, cmd, WATCHDOG_TIMEOUT_TICKS + 1);
        assert_eq!(eval.override_mode, Some(RoverMode::Disconnected));
        assert_eq!(eval.safe_command.throttle, 0.0);
    }

    #[test]
    fn test_adas_blocks_forward_but_allows_reverse_and_turning() {
        let mut sup = SafetySupervisor::new();
        let blocked_sensors = SensorFrame {
            distance_mm: 120,
            ..Default::default()
        };

        let forward_cmd = DriveCommand {
            throttle: 0.8,
            steering: 0.0,
            gear: SpeedGear::Normal,
            brake: false,
            adas_enabled: true,
        };

        let _ = sup.evaluate(&blocked_sensors, forward_cmd, 0);
        let eval_fwd = sup.evaluate(&blocked_sensors, forward_cmd, 0);
        assert_eq!(eval_fwd.override_mode, Some(RoverMode::AdasBraking));
        assert_eq!(eval_fwd.safe_command.throttle, 0.0);
        assert!(eval_fwd.adas_intervened);

        let reverse_cmd = DriveCommand {
            throttle: -0.7,
            ..forward_cmd
        };
        let eval_rev = sup.evaluate(&blocked_sensors, reverse_cmd, 0);
        assert_eq!(eval_rev.override_mode, None);
        assert_eq!(eval_rev.safe_command.throttle, -0.7);

        let pivot_cmd = DriveCommand {
            throttle: 0.8,
            steering: 0.9,
            ..forward_cmd
        };
        let eval_pivot = sup.evaluate(&blocked_sensors, pivot_cmd, 0);
        assert_eq!(eval_pivot.override_mode, Some(RoverMode::AdasBraking));
        assert_eq!(eval_pivot.safe_command.throttle, 0.0);
        assert_eq!(eval_pivot.safe_command.steering, 0.9);
        assert!(!eval_pivot.safe_command.brake);
    }

    #[test]
    fn test_adas_proportional_slow_zone() {
        let mut sup = SafetySupervisor::new();
        let slow_zone_sensors = SensorFrame {
            distance_mm: (ADAS_STOP_DISTANCE_MM + ADAS_SLOW_DISTANCE_MM) / 2,
            ..Default::default()
        };
        let cmd = DriveCommand {
            throttle: 1.0,
            steering: 0.0,
            gear: SpeedGear::Sport,
            brake: false,
            adas_enabled: true,
        };

        let eval = sup.evaluate(&slow_zone_sensors, cmd, 0);
        assert_eq!(eval.override_mode, None);
        assert!(eval.adas_intervened);
        assert!((eval.safe_command.throttle - 0.5).abs() < 0.02);
    }

    #[test]
    fn test_freefall_and_tilt_rollover_cutoffs_and_auto_recovery() {
        let mut sup = SafetySupervisor::new();
        let cmd = DriveCommand {
            throttle: 0.8,
            ..Default::default()
        };

        let freefall_sensors = SensorFrame {
            accel_g: (0.0, 0.0, 0.1),
            ..Default::default()
        };
        let eval = sup.evaluate(&freefall_sensors, cmd, 0);
        assert_eq!(eval.override_mode, Some(RoverMode::Tipped));
        assert_eq!(eval.safe_command.throttle, 0.0);

        let normal_sensors = SensorFrame::default();
        let idle_cmd = DriveCommand::default();
        let recovered = sup.evaluate(&normal_sensors, idle_cmd, 0);
        assert_eq!(recovered.override_mode, None);

        let tilted_sensors = SensorFrame {
            accel_g: (0.85, 0.0, 0.5),
            ..Default::default()
        };
        for _ in 0..(TILT_HYSTERESIS_TICKS - 1) {
            let e = sup.evaluate(&tilted_sensors, cmd, 0);
            assert_eq!(e.override_mode, None);
        }
        let latched = sup.evaluate(&tilted_sensors, cmd, 0);
        assert_eq!(latched.override_mode, Some(RoverMode::Tipped));
    }

    #[test]
    fn test_low_battery_cutoff_and_manual_ebrake() {
        let mut sup = SafetySupervisor::new();
        let cmd = DriveCommand {
            throttle: 0.9,
            ..Default::default()
        };

        let low_batt = SensorFrame {
            battery_pct: LOW_BATT_CUTOFF_PCT - 1,
            ..Default::default()
        };
        let eval_batt = sup.evaluate(&low_batt, cmd, 0);
        assert_eq!(eval_batt.override_mode, Some(RoverMode::LowBattery));
        assert_eq!(eval_batt.safe_command.throttle, 0.0);

        let brake_cmd = DriveCommand { brake: true, ..cmd };
        let eval_brake = sup.evaluate(&SensorFrame::default(), brake_cmd, 0);
        assert_eq!(eval_brake.override_mode, Some(RoverMode::Braking));
        assert!(eval_brake.safe_command.brake);
    }

    #[test]
    fn test_upside_down_inversion_latches_and_stays_latched() {
        let mut sup = SafetySupervisor::new();
        let inverted = SensorFrame {
            accel_g: (0.0, 0.0, -1.0),
            ..Default::default()
        };
        let idle = DriveCommand::default();

        for _ in 0..(TILT_HYSTERESIS_TICKS - 1) {
            let e = sup.evaluate(&inverted, idle, 0);
            assert_eq!(e.override_mode, None);
        }
        let latched = sup.evaluate(&inverted, idle, 0);
        assert_eq!(latched.override_mode, Some(RoverMode::Tipped));

        // Even with neutral sticks, staying upside down must keep Tipped latched
        let still_inverted = sup.evaluate(&inverted, idle, 0);
        assert_eq!(still_inverted.override_mode, Some(RoverMode::Tipped));

        // Upright + neutral inputs recovers
        let upright = sup.evaluate(&SensorFrame::default(), idle, 0);
        assert_eq!(upright.override_mode, None);
    }

    #[test]
    fn test_low_battery_hysteresis_prevents_sag_chatter() {
        let mut sup = SafetySupervisor::new();
        let cmd = DriveCommand {
            throttle: 0.8,
            ..Default::default()
        };

        // Drop below cutoff (15%)
        let low = SensorFrame {
            battery_pct: LOW_BATT_CUTOFF_PCT - 1,
            ..Default::default()
        };
        assert_eq!(
            sup.evaluate(&low, cmd, 0).override_mode,
            Some(RoverMode::LowBattery)
        );

        // Rebound to 17% (between 15% and 20% recover threshold) stays latched
        let rebound = SensorFrame {
            battery_pct: LOW_BATT_CUTOFF_PCT + 2,
            ..Default::default()
        };
        let still_latched = sup.evaluate(&rebound, cmd, 0);
        assert_eq!(still_latched.override_mode, Some(RoverMode::LowBattery));
        assert_eq!(still_latched.safe_command.throttle, 0.0);

        // Recover at >= 20%
        let recovered = SensorFrame {
            battery_pct: LOW_BATT_RECOVER_PCT,
            ..Default::default()
        };
        assert_eq!(sup.evaluate(&recovered, cmd, 0).override_mode, None);
    }

    #[test]
    fn test_adas_tick_one_stop_zone_clamp_and_tof_fault_limit() {
        let mut sup = SafetySupervisor::new();
        let cmd = DriveCommand {
            throttle: 1.0,
            ..Default::default()
        };

        // Tick 1 inside <= ADAS_STOP_DISTANCE_MM (140 mm) must already clamp throttle to 0.20 while confirming
        let stop_zone = SensorFrame {
            distance_mm: 100,
            ..Default::default()
        };
        let tick1 = sup.evaluate(&stop_zone, cmd, 0);
        assert_eq!(tick1.override_mode, None);
        assert!(tick1.adas_intervened);
        assert!((tick1.safe_command.throttle - ADAS_MIN_THROTTLE_SCALE).abs() < 1e-4);

        let tick2 = sup.evaluate(&stop_zone, cmd, 0);
        assert_eq!(tick2.override_mode, Some(RoverMode::AdasBraking));
        assert_eq!(tick2.safe_command.throttle, 0.0);

        // Unhealthy ToF with ADAS enabled caps forward speed to Crawl limit
        let mut sup2 = SafetySupervisor::new();
        let faulted = SensorFrame {
            tof_healthy: false,
            ..Default::default()
        };
        let eval_fault = sup2.evaluate(&faulted, cmd, 0);
        assert!(eval_fault.adas_intervened);
        assert!((eval_fault.safe_command.throttle - TOF_FAULT_MAX_THROTTLE).abs() < 1e-4);
    }

    #[test]
    fn test_imu_fault_does_not_clear_tipped_latch() {
        let mut sup = SafetySupervisor::new();
        let freefall = SensorFrame {
            accel_g: (0.0, 0.0, 0.1),
            ..Default::default()
        };
        assert_eq!(
            sup.evaluate(&freefall, DriveCommand::default(), 0)
                .override_mode,
            Some(RoverMode::Tipped)
        );

        let imu_lost = SensorFrame {
            imu_healthy: false,
            accel_g: (0.0, 0.0, 1.0),
            ..Default::default()
        };
        assert_eq!(
            sup.evaluate(&imu_lost, DriveCommand::default(), 0)
                .override_mode,
            Some(RoverMode::Tipped)
        );
    }

    #[test]
    fn test_watchdog_exact_timeout_boundary() {
        let mut sup = SafetySupervisor::new();
        let sensors = SensorFrame::default();
        let cmd = DriveCommand {
            throttle: 0.7,
            steering: -0.3,
            ..Default::default()
        };

        let just_before = sup.evaluate(&sensors, cmd, WATCHDOG_TIMEOUT_TICKS - 1);
        assert_eq!(just_before.override_mode, None);
        assert!((just_before.safe_command.throttle - 0.7).abs() < 1e-5);

        let at_timeout = sup.evaluate(&sensors, cmd, WATCHDOG_TIMEOUT_TICKS);
        assert_eq!(at_timeout.override_mode, Some(RoverMode::Disconnected));
        assert_eq!(at_timeout.safe_command.throttle, 0.0);
        assert_eq!(at_timeout.safe_command.steering, 0.0);
        assert!(!at_timeout.safe_command.brake);
    }

    #[test]
    fn test_low_battery_exact_cutoff_boundary() {
        let mut sup = SafetySupervisor::new();
        let cmd = DriveCommand {
            throttle: 0.8,
            ..Default::default()
        };

        let at_limit = SensorFrame {
            battery_pct: LOW_BATT_CUTOFF_PCT,
            ..Default::default()
        };
        let eval_ok = sup.evaluate(&at_limit, cmd, 0);
        assert_eq!(eval_ok.override_mode, None);
        assert!((eval_ok.safe_command.throttle - 0.8).abs() < 1e-5);

        let below_limit = SensorFrame {
            battery_pct: LOW_BATT_CUTOFF_PCT - 1,
            ..Default::default()
        };
        let eval_low = sup.evaluate(&below_limit, cmd, 0);
        assert_eq!(eval_low.override_mode, Some(RoverMode::LowBattery));
        assert_eq!(eval_low.safe_command.throttle, 0.0);
        assert_eq!(eval_low.safe_command.steering, 0.0);
    }

    #[test]
    fn test_tilt_recovery_requires_neutral_sticks() {
        let mut sup = SafetySupervisor::new();
        let freefall = SensorFrame {
            accel_g: (0.0, 0.0, 0.1),
            ..Default::default()
        };
        assert_eq!(
            sup.evaluate(&freefall, DriveCommand::default(), 0)
                .override_mode,
            Some(RoverMode::Tipped)
        );

        let upright = SensorFrame::default();
        let held_throttle = DriveCommand {
            throttle: STICK_DEADZONE + 0.05,
            ..Default::default()
        };
        let still_tipped_thr = sup.evaluate(&upright, held_throttle, 0);
        assert_eq!(still_tipped_thr.override_mode, Some(RoverMode::Tipped));
        assert_eq!(still_tipped_thr.safe_command.throttle, 0.0);

        let held_steer = DriveCommand {
            steering: -(STICK_DEADZONE + 0.05),
            ..Default::default()
        };
        let still_tipped_str = sup.evaluate(&upright, held_steer, 0);
        assert_eq!(still_tipped_str.override_mode, Some(RoverMode::Tipped));
        assert_eq!(still_tipped_str.safe_command.steering, 0.0);

        let recovered = sup.evaluate(&upright, DriveCommand::default(), 0);
        assert_eq!(recovered.override_mode, None);
    }

    #[test]
    fn test_adas_disabled_bypasses_obstacle_and_tof_fault_limits() {
        let mut sup = SafetySupervisor::new();
        let cmd = DriveCommand {
            throttle: 1.0,
            adas_enabled: false,
            ..Default::default()
        };

        let wall = SensorFrame {
            distance_mm: 50,
            ..Default::default()
        };
        let _ = sup.evaluate(&wall, cmd, 0);
        let eval_wall = sup.evaluate(&wall, cmd, 0);
        assert_eq!(eval_wall.override_mode, None);
        assert!(!eval_wall.adas_intervened);
        assert_eq!(eval_wall.safe_command.throttle, 1.0);

        let tof_fault = SensorFrame {
            tof_healthy: false,
            ..Default::default()
        };
        let eval_fault = sup.evaluate(&tof_fault, cmd, 0);
        assert_eq!(eval_fault.override_mode, None);
        assert!(!eval_fault.adas_intervened);
        assert_eq!(eval_fault.safe_command.throttle, 1.0);
    }

    #[test]
    fn test_adas_single_tick_obstacle_spike_resets_counter() {
        let mut sup = SafetySupervisor::new();
        let cmd = DriveCommand {
            throttle: 1.0,
            ..Default::default()
        };
        let close = SensorFrame {
            distance_mm: 100,
            ..Default::default()
        };
        let clear = SensorFrame {
            distance_mm: 600,
            ..Default::default()
        };

        // Tick 1 close: slows down, not yet AdasBraking
        let t1 = sup.evaluate(&close, cmd, 0);
        assert_eq!(t1.override_mode, None);
        assert!(t1.adas_intervened);

        // Tick 2 clear: resets obstacle_ticks to 0
        let t2 = sup.evaluate(&clear, cmd, 0);
        assert_eq!(t2.override_mode, None);
        assert!(!t2.adas_intervened);

        // Tick 3 close again: must restart confirmation count at 1
        let t3 = sup.evaluate(&close, cmd, 0);
        assert_eq!(t3.override_mode, None);
        assert!((t3.safe_command.throttle - ADAS_MIN_THROTTLE_SCALE).abs() < 1e-4);

        // Tick 4 consecutive close: now confirms AdasBraking
        let t4 = sup.evaluate(&close, cmd, 0);
        assert_eq!(t4.override_mode, Some(RoverMode::AdasBraking));
        assert_eq!(t4.safe_command.throttle, 0.0);
    }

    #[test]
    fn test_adas_stop_hysteresis_prevents_boundary_chatter() {
        let mut sup = SafetySupervisor::new();
        let cmd = DriveCommand {
            throttle: 0.8,
            ..Default::default()
        };

        let stop_zone = SensorFrame {
            distance_mm: ADAS_STOP_DISTANCE_MM,
            ..Default::default()
        };
        let _ = sup.evaluate(&stop_zone, cmd, 0);
        let latched = sup.evaluate(&stop_zone, cmd, 0);
        assert_eq!(latched.override_mode, Some(RoverMode::AdasBraking));
        assert_eq!(latched.safe_command.throttle, 0.0);

        // Jitter inside the hysteresis band (141..=165 mm) keeps AdasBraking latched
        let jitter = SensorFrame {
            distance_mm: ADAS_STOP_RELEASE_MM,
            ..Default::default()
        };
        let still_stopped = sup.evaluate(&jitter, cmd, 0);
        assert_eq!(still_stopped.override_mode, Some(RoverMode::AdasBraking));
        assert_eq!(still_stopped.safe_command.throttle, 0.0);

        // Moving past ADAS_STOP_RELEASE_MM releases back into slow zone
        let released = SensorFrame {
            distance_mm: ADAS_STOP_RELEASE_MM + 1,
            ..Default::default()
        };
        let eval_released = sup.evaluate(&released, cmd, 0);
        assert_eq!(eval_released.override_mode, None);
        assert!(eval_released.adas_intervened);
        assert!(eval_released.safe_command.throttle > 0.0);
    }

    #[test]
    fn test_adas_ignores_stick_drift_within_deadzone() {
        let mut sup = SafetySupervisor::new();
        let wall = SensorFrame {
            distance_mm: 80,
            ..Default::default()
        };
        let drift_cmd = DriveCommand {
            throttle: STICK_DEADZONE - 0.01,
            ..Default::default()
        };

        for _ in 0..4 {
            let eval = sup.evaluate(&wall, drift_cmd, 0);
            assert_eq!(eval.override_mode, None);
            assert!(!eval.adas_intervened);
            assert!(!eval.safe_command.brake);
        }
    }

    #[test]
    fn test_non_finite_accel_inputs_do_not_trigger_or_clear_tipped_latch() {
        let mut sup = SafetySupervisor::new();
        let nan_accel = SensorFrame {
            accel_g: (f32::NAN, 0.0, 1.0),
            ..Default::default()
        };
        let inf_accel = SensorFrame {
            accel_g: (0.0, f32::INFINITY, 1.0),
            ..Default::default()
        };

        assert_eq!(
            sup.evaluate(&nan_accel, DriveCommand::default(), 0)
                .override_mode,
            None
        );
        assert_eq!(
            sup.evaluate(&inf_accel, DriveCommand::default(), 0)
                .override_mode,
            None
        );

        let freefall = SensorFrame {
            accel_g: (0.0, 0.0, 0.1),
            ..Default::default()
        };
        assert_eq!(
            sup.evaluate(&freefall, DriveCommand::default(), 0)
                .override_mode,
            Some(RoverMode::Tipped)
        );
        assert_eq!(
            sup.evaluate(&nan_accel, DriveCommand::default(), 0)
                .override_mode,
            Some(RoverMode::Tipped)
        );
    }

    #[test]
    fn test_simultaneous_faults_follow_strict_priority_order() {
        let mut sup = SafetySupervisor::new();
        let brake_cmd = DriveCommand {
            throttle: 0.9,
            brake: true,
            ..Default::default()
        };

        // 1. Tipped > LowBattery > Disconnected > Braking > AdasBraking
        let all_faults = SensorFrame {
            accel_g: (0.0, 0.0, 0.1),
            battery_pct: LOW_BATT_CUTOFF_PCT - 5,
            distance_mm: 50,
            ..Default::default()
        };
        let eval_tipped = sup.evaluate(&all_faults, brake_cmd, WATCHDOG_TIMEOUT_TICKS + 5);
        assert_eq!(eval_tipped.override_mode, Some(RoverMode::Tipped));
        assert!(!eval_tipped.safe_command.brake);

        // 2. Clear Tipped -> LowBattery takes precedence over Disconnected and Braking
        let mut sup2 = SafetySupervisor::new();
        let batt_and_wall = SensorFrame {
            battery_pct: LOW_BATT_CUTOFF_PCT - 5,
            distance_mm: 50,
            ..Default::default()
        };
        let eval_batt = sup2.evaluate(&batt_and_wall, brake_cmd, WATCHDOG_TIMEOUT_TICKS + 5);
        assert_eq!(eval_batt.override_mode, Some(RoverMode::LowBattery));
        assert!(!eval_batt.safe_command.brake);

        // 3. Healthy battery -> Disconnected takes precedence over Braking
        let mut sup3 = SafetySupervisor::new();
        let eval_disc = sup3.evaluate(
            &SensorFrame::default(),
            brake_cmd,
            WATCHDOG_TIMEOUT_TICKS + 5,
        );
        assert_eq!(eval_disc.override_mode, Some(RoverMode::Disconnected));
        assert!(!eval_disc.safe_command.brake);

        // 4. Connected + Braking takes precedence over ADAS obstacle
        let wall = SensorFrame {
            distance_mm: 50,
            ..Default::default()
        };
        let eval_brake = sup3.evaluate(&wall, brake_cmd, 0);
        assert_eq!(eval_brake.override_mode, Some(RoverMode::Braking));
        assert!(eval_brake.safe_command.brake);
        assert!(!eval_brake.adas_intervened);
    }
}
