//! Atomic state shared across control, sensor, and network tasks.

use crate::config::control::TOF_CLEAR_DISTANCE_MM;
use crate::domain::kinematics::sanitize_unit;
use crate::domain::ports::{DriveCommand, RoverMode, SensorFrame, SpeedGear, TelemetrySnapshot};
use core::sync::atomic::{AtomicBool, AtomicI8, AtomicI32, AtomicU16, AtomicU32, Ordering, fence};

pub struct SharedRoverState {
    packed_cmd: AtomicU32,
    cmd_seq: AtomicU32,

    sensor_seq: AtomicU32,
    distance_mm: AtomicU16,
    tof_healthy: AtomicBool,
    imu_healthy: AtomicBool,
    battery_pct: AtomicI32,
    accel_x_mg: AtomicI32,
    accel_y_mg: AtomicI32,
    accel_z_mg: AtomicI32,

    telem_seq: AtomicU32,
    packed_telem_a: AtomicU32,
    packed_telem_b: AtomicU32,
    rssi_dbm: AtomicI8,
}

impl Default for SharedRoverState {
    fn default() -> Self {
        Self::new()
    }
}

impl SharedRoverState {
    pub const fn new() -> Self {
        let initial_cmd = Self::encode_command(DriveCommand {
            throttle: 0.0,
            steering: 0.0,
            gear: SpeedGear::Normal,
            brake: false,
            adas_enabled: true,
        });
        let (telem_a, telem_b) = Self::encode_telemetry(&TelemetrySnapshot {
            mode: RoverMode::Disconnected,
            adas_intervening: false,
            left_duty: 0,
            right_duty: 0,
            distance_mm: TOF_CLEAR_DISTANCE_MM,
            battery_pct: 100,
            tof_healthy: true,
            imu_healthy: true,
        });

        Self {
            packed_cmd: AtomicU32::new(initial_cmd),
            cmd_seq: AtomicU32::new(0),

            sensor_seq: AtomicU32::new(0),
            distance_mm: AtomicU16::new(TOF_CLEAR_DISTANCE_MM),
            tof_healthy: AtomicBool::new(true),
            imu_healthy: AtomicBool::new(true),
            battery_pct: AtomicI32::new(100),
            accel_x_mg: AtomicI32::new(0),
            accel_y_mg: AtomicI32::new(0),
            accel_z_mg: AtomicI32::new(1000),

            telem_seq: AtomicU32::new(0),
            packed_telem_a: AtomicU32::new(telem_a),
            packed_telem_b: AtomicU32::new(telem_b),
            rssi_dbm: AtomicI8::new(0),
        }
    }

    const fn encode_command(cmd: DriveCommand) -> u32 {
        let t = ((sanitize_unit(cmd.throttle) * 1000.0) as i32 + 1000) as u32;
        let s = ((sanitize_unit(cmd.steering) * 1000.0) as i32 + 1000) as u32;
        let g = (cmd.gear as u8 as u32) & 0x3;
        let b = (cmd.brake as u32) & 0x1;
        let a = (cmd.adas_enabled as u32) & 0x1;
        (t & 0x7FF) | ((s & 0x7FF) << 11) | (g << 22) | (b << 24) | (a << 25)
    }

    fn decode_command(packed: u32) -> DriveCommand {
        let t_milli = (packed & 0x7FF) as i32 - 1000;
        let s_milli = ((packed >> 11) & 0x7FF) as i32 - 1000;
        let gear = SpeedGear::from_u8(((packed >> 22) & 0x3) as u8);
        let brake = ((packed >> 24) & 0x1) != 0;
        let adas_enabled = ((packed >> 25) & 0x1) != 0;

        DriveCommand {
            throttle: (t_milli.clamp(-1000, 1000) as f32) / 1000.0,
            steering: (s_milli.clamp(-1000, 1000) as f32) / 1000.0,
            gear,
            brake,
            adas_enabled,
        }
    }

    const fn encode_telemetry(snap: &TelemetrySnapshot) -> (u32, u32) {
        let l = (clamp_i32(snap.left_duty, -1000, 1000) + 1000) as u32;
        let r = (clamp_i32(snap.right_duty, -1000, 1000) + 1000) as u32;
        let mode = (snap.mode as u8 as u32) & 0x7;
        let adas = (snap.adas_intervening as u32) & 0x1;
        let tof = (snap.tof_healthy as u32) & 0x1;
        let imu = (snap.imu_healthy as u32) & 0x1;
        let word_a = (l & 0x7FF)
            | ((r & 0x7FF) << 11)
            | (mode << 22)
            | (adas << 25)
            | (tof << 26)
            | (imu << 27);

        let dist = snap.distance_mm as u32;
        let batt = clamp_i32(snap.battery_pct, 0, 100) as u32;
        let word_b = dist | (batt << 16);
        (word_a, word_b)
    }

    fn decode_telemetry(word_a: u32, word_b: u32) -> TelemetrySnapshot {
        let left_duty = (word_a & 0x7FF) as i32 - 1000;
        let right_duty = ((word_a >> 11) & 0x7FF) as i32 - 1000;
        let mode = RoverMode::from_u8(((word_a >> 22) & 0x7) as u8);
        let adas_intervening = ((word_a >> 25) & 0x1) != 0;
        let tof_healthy = ((word_a >> 26) & 0x1) != 0;
        let imu_healthy = ((word_a >> 27) & 0x1) != 0;

        let distance_mm = (word_b & 0xFFFF) as u16;
        let battery_pct = ((word_b >> 16) & 0x7F) as i32;

        TelemetrySnapshot {
            mode,
            adas_intervening,
            left_duty,
            right_duty,
            distance_mm,
            battery_pct,
            tof_healthy,
            imu_healthy,
        }
    }

    pub(crate) fn publish_command(&self, cmd: DriveCommand) {
        let packed = Self::encode_command(cmd);
        self.packed_cmd.store(packed, Ordering::Release);
        self.cmd_seq.fetch_add(1, Ordering::Release);
    }

    pub(crate) fn publish_rssi(&self, rssi_dbm: i8) {
        self.rssi_dbm.store(rssi_dbm, Ordering::Relaxed);
    }

    pub(crate) fn rssi_dbm(&self) -> i8 {
        self.rssi_dbm.load(Ordering::Relaxed)
    }

    pub(crate) fn publish_imu_and_tof(
        &self,
        dist_mm: u16,
        ax: f32,
        ay: f32,
        az: f32,
        tof_healthy: bool,
        imu_healthy: bool,
    ) {
        self.sensor_seq.fetch_add(1, Ordering::AcqRel);
        fence(Ordering::Release);
        self.distance_mm.store(dist_mm, Ordering::Relaxed);
        self.tof_healthy.store(tof_healthy, Ordering::Relaxed);
        self.imu_healthy.store(imu_healthy, Ordering::Relaxed);
        self.accel_x_mg
            .store((ax * 1000.0) as i32, Ordering::Relaxed);
        self.accel_y_mg
            .store((ay * 1000.0) as i32, Ordering::Relaxed);
        self.accel_z_mg
            .store((az * 1000.0) as i32, Ordering::Relaxed);
        self.sensor_seq.fetch_add(1, Ordering::Release);
    }

    pub(crate) fn publish_battery(&self, pct: i32) {
        self.battery_pct.store(pct.clamp(0, 100), Ordering::Relaxed);
    }

    /// Loads the latest command together with its monotonic sequence counter.
    pub(crate) fn load_command_with_seq(&self) -> (DriveCommand, u32) {
        let seq = self.cmd_seq.load(Ordering::Acquire);
        let packed = self.packed_cmd.load(Ordering::Acquire);
        (Self::decode_command(packed), seq)
    }

    pub(crate) fn load_sensor_frame(&self) -> SensorFrame {
        loop {
            let s1 = self.sensor_seq.load(Ordering::Acquire);
            if s1 & 1 != 0 {
                core::hint::spin_loop();
                continue;
            }
            let distance_mm = self.distance_mm.load(Ordering::Relaxed);
            let tof_healthy = self.tof_healthy.load(Ordering::Relaxed);
            let imu_healthy = self.imu_healthy.load(Ordering::Relaxed);
            let ax = self.accel_x_mg.load(Ordering::Relaxed) as f32 / 1000.0;
            let ay = self.accel_y_mg.load(Ordering::Relaxed) as f32 / 1000.0;
            let az = self.accel_z_mg.load(Ordering::Relaxed) as f32 / 1000.0;
            let battery_pct = self.battery_pct.load(Ordering::Relaxed);
            fence(Ordering::Acquire);
            if self.sensor_seq.load(Ordering::Acquire) == s1 {
                return SensorFrame {
                    distance_mm,
                    battery_pct,
                    accel_g: (ax, ay, az),
                    tof_healthy,
                    imu_healthy,
                };
            }
        }
    }

    pub(crate) fn publish_telemetry(&self, snap: &TelemetrySnapshot) {
        let (word_a, word_b) = Self::encode_telemetry(snap);
        self.telem_seq.fetch_add(1, Ordering::AcqRel);
        fence(Ordering::Release);
        self.packed_telem_a.store(word_a, Ordering::Relaxed);
        self.packed_telem_b.store(word_b, Ordering::Relaxed);
        self.telem_seq.fetch_add(1, Ordering::Release);
    }

    pub(crate) fn read_telemetry(&self) -> TelemetrySnapshot {
        loop {
            let s1 = self.telem_seq.load(Ordering::Acquire);
            if s1 & 1 != 0 {
                core::hint::spin_loop();
                continue;
            }
            let word_a = self.packed_telem_a.load(Ordering::Relaxed);
            let word_b = self.packed_telem_b.load(Ordering::Relaxed);
            fence(Ordering::Acquire);
            if self.telem_seq.load(Ordering::Acquire) == s1 {
                return Self::decode_telemetry(word_a, word_b);
            }
        }
    }
}

const fn clamp_i32(val: i32, min: i32, max: i32) -> i32 {
    if val < min {
        min
    } else if val > max {
        max
    } else {
        val
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_shared_rover_state_command_and_seq_roundtrip() {
        let state = SharedRoverState::default();

        let (_, initial_seq) = state.load_command_with_seq();
        assert_eq!(initial_seq, 0);

        let cmd = DriveCommand {
            throttle: 0.75,
            steering: -0.5,
            gear: SpeedGear::Sport,
            brake: true,
            adas_enabled: false,
        };
        state.publish_command(cmd);

        let (loaded, seq1) = state.load_command_with_seq();
        assert_eq!(seq1, 1);
        assert!((loaded.throttle - 0.75).abs() < 1e-3);
        assert!((loaded.steering + 0.5).abs() < 1e-3);
        assert_eq!(loaded.gear, SpeedGear::Sport);
        assert!(loaded.brake);
        assert!(!loaded.adas_enabled);

        let (_, seq1_again) = state.load_command_with_seq();
        assert_eq!(seq1_again, 1);
    }

    #[test]
    fn test_shared_rover_state_sensors_and_telemetry() {
        let state = SharedRoverState::new();

        state.publish_imu_and_tof(340, 0.12, -0.25, 0.98, true, false);
        state.publish_battery(82);
        assert_eq!(state.rssi_dbm(), 0);
        state.publish_rssi(-58);
        assert_eq!(state.rssi_dbm(), -58);

        let sf = state.load_sensor_frame();
        assert_eq!(sf.distance_mm, 340);
        assert_eq!(sf.battery_pct, 82);
        assert!(sf.tof_healthy);
        assert!(!sf.imu_healthy);
        assert!((sf.accel_g.0 - 0.12).abs() < 1e-3);
        assert!((sf.accel_g.1 + 0.25).abs() < 1e-3);
        assert!((sf.accel_g.2 - 0.98).abs() < 1e-3);

        for mode in [
            RoverMode::Idle,
            RoverMode::ManualDrive,
            RoverMode::Braking,
            RoverMode::AdasBraking,
            RoverMode::Disconnected,
            RoverMode::Tipped,
            RoverMode::LowBattery,
        ] {
            let snap = TelemetrySnapshot {
                mode,
                adas_intervening: true,
                left_duty: 400,
                right_duty: -400,
                distance_mm: 340,
                battery_pct: 82,
                tof_healthy: false,
                imu_healthy: true,
            };
            state.publish_telemetry(&snap);
            let read_back = state.read_telemetry();
            assert_eq!(read_back.mode, mode);
            assert!(read_back.adas_intervening);
            assert_eq!(read_back.left_duty, 400);
            assert_eq!(read_back.right_duty, -400);
            assert_eq!(read_back.distance_mm, 340);
            assert_eq!(read_back.battery_pct, 82);
            assert!(!read_back.tof_healthy);
            assert!(read_back.imu_healthy);
        }
    }

    #[test]
    fn test_packed_command_sanitizes_non_finite_and_out_of_range() {
        let state = SharedRoverState::new();
        state.publish_command(DriveCommand {
            throttle: f32::NAN,
            steering: f32::INFINITY,
            gear: SpeedGear::Normal,
            brake: false,
            adas_enabled: true,
        });
        let (decoded, seq1) = state.load_command_with_seq();
        assert_eq!(seq1, 1);
        assert_eq!(decoded.throttle, 0.0);
        assert_eq!(decoded.steering, 0.0);

        state.publish_command(DriveCommand {
            throttle: 2.5,
            steering: -3.0,
            gear: SpeedGear::Crawl,
            brake: false,
            adas_enabled: true,
        });
        let (clamped, seq2) = state.load_command_with_seq();
        assert_eq!(seq2, 2);
        assert!((clamped.throttle - 1.0).abs() < 1e-3);
        assert!((clamped.steering + 1.0).abs() < 1e-3);
    }

    #[test]
    fn test_corrupted_telemetry_mode_bits_fail_safe() {
        // Mode bits 7 (binary 111) sit outside valid enum discriminants 0..=6.
        let corrupted_word_a = 7u32 << 22;
        let decoded = SharedRoverState::decode_telemetry(corrupted_word_a, 0);
        assert_eq!(decoded.mode, RoverMode::Disconnected);
    }

    #[test]
    fn test_concurrent_publish_and_load_never_corrupts_or_drops_seq_advance() {
        extern crate std;
        use std::sync::Arc;
        use std::thread;

        let state = Arc::new(SharedRoverState::new());
        let writer_state = Arc::clone(&state);

        let writer = thread::spawn(move || {
            for i in 0..2000i32 {
                let norm = ((i % 201) - 100) as f32 / 100.0;
                writer_state.publish_command(DriveCommand {
                    throttle: norm,
                    steering: -norm,
                    gear: SpeedGear::Sport,
                    brake: (i % 2) == 0,
                    adas_enabled: (i % 3) != 0,
                });
            }
        });

        let mut last_seq = 0;
        for _ in 0..2000 {
            let (cmd, seq) = state.load_command_with_seq();
            assert!(seq >= last_seq);
            last_seq = seq;
            assert!(cmd.throttle.is_finite());
            assert!(cmd.steering.is_finite());
            // Producer sets throttle = -steering, so packed round-trip must preserve symmetry.
            assert!((cmd.throttle + cmd.steering).abs() < 2e-3);
        }

        writer.join().unwrap();
        let seq_before = state.load_command_with_seq().1;
        state.publish_command(DriveCommand::default());
        let (_, seq_after) = state.load_command_with_seq();
        assert_eq!(seq_after, seq_before + 1);
    }

    #[test]
    fn test_concurrent_imu_and_tof_seqlock_never_tears() {
        extern crate std;
        use std::sync::atomic::{AtomicBool, Ordering as StdOrdering};
        use std::sync::{Arc, Barrier};
        use std::thread;

        let state = Arc::new(SharedRoverState::new());
        let done = Arc::new(AtomicBool::new(false));
        let barrier = Arc::new(Barrier::new(2));

        let writer_state = Arc::clone(&state);
        let writer_done = Arc::clone(&done);
        let writer_barrier = Arc::clone(&barrier);

        let writer = thread::spawn(move || {
            writer_barrier.wait();
            for i in 1..=10_000u32 {
                let n = (i % 500) + 100; // 100..=599 (distinct from initial 1000)
                let ax = (n as f32) / 1000.0;
                let ay = -ax;
                let az = ((1000 + n) as f32) / 1000.0;
                let tof_ok = n.is_multiple_of(2);
                let imu_ok = !tof_ok;
                writer_state.publish_imu_and_tof(n as u16, ax, ay, az, tof_ok, imu_ok);
            }
            writer_done.store(true, StdOrdering::Release);
        });

        barrier.wait();
        let mut reads = 0u32;
        while !done.load(StdOrdering::Acquire) || reads < 10_000 {
            let sf = state.load_sensor_frame();
            if sf.distance_mm == TOF_CLEAR_DISTANCE_MM {
                // Initial state before first write
                assert_eq!(sf.accel_g, (0.0, 0.0, 1.0));
                assert!(sf.tof_healthy);
                assert!(sf.imu_healthy);
            } else {
                let n = sf.distance_mm as i32;
                assert!((100..=599).contains(&n), "unexpected dist: {}", n);
                let ax_in = (n as f32) / 1000.0;
                let az_in = ((1000 + n) as f32) / 1000.0;
                let expected_ax = ((ax_in * 1000.0) as i32) as f32 / 1000.0;
                let expected_ay = ((-ax_in * 1000.0) as i32) as f32 / 1000.0;
                let expected_az = ((az_in * 1000.0) as i32) as f32 / 1000.0;
                assert!((sf.accel_g.0 - expected_ax).abs() < 1e-6);
                assert!((sf.accel_g.1 - expected_ay).abs() < 1e-6);
                assert!((sf.accel_g.2 - expected_az).abs() < 1e-6);
                assert_eq!(sf.tof_healthy, n % 2 == 0);
                assert_eq!(sf.imu_healthy, n % 2 != 0);
            }
            reads += 1;
        }

        writer.join().unwrap();
    }

    #[test]
    fn test_concurrent_telemetry_seqlock_never_tears() {
        extern crate std;
        use std::sync::atomic::{AtomicBool, Ordering as StdOrdering};
        use std::sync::{Arc, Barrier};
        use std::thread;

        let state = Arc::new(SharedRoverState::new());
        let done = Arc::new(AtomicBool::new(false));
        let barrier = Arc::new(Barrier::new(2));

        let writer_state = Arc::clone(&state);
        let writer_done = Arc::clone(&done);
        let writer_barrier = Arc::clone(&barrier);

        let writer = thread::spawn(move || {
            writer_barrier.wait();
            for i in 1..=10_000i32 {
                let n = (i % 500) + 100; // 100..=599 (distinct from initial 0 / 1000)
                writer_state.publish_telemetry(&TelemetrySnapshot {
                    mode: RoverMode::from_u8((n % 7) as u8),
                    adas_intervening: n % 2 == 0,
                    left_duty: n,
                    right_duty: -n,
                    distance_mm: n as u16,
                    battery_pct: n % 101,
                    tof_healthy: n % 3 == 0,
                    imu_healthy: n % 3 != 0,
                });
            }
            writer_done.store(true, StdOrdering::Release);
        });

        barrier.wait();
        let mut reads = 0u32;
        while !done.load(StdOrdering::Acquire) || reads < 10_000 {
            let snap = state.read_telemetry();
            if snap.distance_mm == TOF_CLEAR_DISTANCE_MM {
                assert_eq!(snap.mode, RoverMode::Disconnected);
                assert_eq!(snap.left_duty, 0);
                assert_eq!(snap.right_duty, 0);
                assert_eq!(snap.battery_pct, 100);
                assert!(!snap.adas_intervening);
                assert!(snap.tof_healthy);
                assert!(snap.imu_healthy);
            } else {
                let n = snap.distance_mm as i32;
                assert!((100..=599).contains(&n), "unexpected dist: {}", n);
                assert_eq!(snap.left_duty, n);
                assert_eq!(snap.right_duty, -n);
                assert_eq!(snap.battery_pct, n % 101);
                assert_eq!(snap.mode, RoverMode::from_u8((n % 7) as u8));
                assert_eq!(snap.adas_intervening, n % 2 == 0);
                assert_eq!(snap.tof_healthy, n % 3 == 0);
                assert_eq!(snap.imu_healthy, n % 3 != 0);
            }
            reads += 1;
        }

        writer.join().unwrap();
    }

    #[test]
    fn test_battery_and_telemetry_clamping_and_negative_accel_roundtrip() {
        let state = SharedRoverState::default();

        // publish_battery clamps below 0 and above 100
        state.publish_battery(-5);
        assert_eq!(state.load_sensor_frame().battery_pct, 0);
        state.publish_battery(150);
        assert_eq!(state.load_sensor_frame().battery_pct, 100);
        state.publish_battery(0);
        assert_eq!(state.load_sensor_frame().battery_pct, 0);

        // Negative acceleration roundtrip
        state.publish_imu_and_tof(42, -1.25, -0.5, -2.0, true, true);
        let sensor = state.load_sensor_frame();
        assert_eq!(sensor.distance_mm, 42);
        assert!((sensor.accel_g.0 + 1.25).abs() < 1e-3);
        assert!((sensor.accel_g.1 + 0.5).abs() < 1e-3);
        assert!((sensor.accel_g.2 + 2.0).abs() < 1e-3);

        // encode_telemetry / decode_telemetry duty clamping (-1500/1500 -> -1000/1000)
        // and battery clamping (-5/150 -> 0/100), plus u16::MAX distance boundary
        state.publish_telemetry(&TelemetrySnapshot {
            mode: RoverMode::Tipped,
            adas_intervening: true,
            left_duty: -1500,
            right_duty: 1500,
            distance_mm: u16::MAX,
            battery_pct: -5,
            tof_healthy: false,
            imu_healthy: true,
        });
        let t1 = state.read_telemetry();
        assert_eq!(t1.mode, RoverMode::Tipped);
        assert!(t1.adas_intervening);
        assert_eq!(t1.left_duty, -1000);
        assert_eq!(t1.right_duty, 1000);
        assert_eq!(t1.distance_mm, u16::MAX);
        assert_eq!(t1.battery_pct, 0);
        assert!(!t1.tof_healthy);
        assert!(t1.imu_healthy);

        state.publish_telemetry(&TelemetrySnapshot {
            mode: RoverMode::ManualDrive,
            adas_intervening: false,
            left_duty: 1500,
            right_duty: -1500,
            distance_mm: 0,
            battery_pct: 150,
            tof_healthy: true,
            imu_healthy: false,
        });
        let t2 = state.read_telemetry();
        assert_eq!(t2.left_duty, 1000);
        assert_eq!(t2.right_duty, -1000);
        assert_eq!(t2.distance_mm, 0);
        assert_eq!(t2.battery_pct, 100);
    }
}
