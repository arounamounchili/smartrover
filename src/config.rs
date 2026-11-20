//! Centralized configuration for SmartRover domain and hardware parameters.

/// HTTP server port for the FPV Cockpit & Gamepad control API.
#[cfg(target_arch = "xtensa")]
pub(crate) const WEB_PORT: u16 = 80;

/// Maximum JPEG frame size accepted by the camera pipeline and HTTP streamer (20 KB).
/// OV5642 QVGA @ quantization `0x0C` produces ~10–15 KB JPEGs, leaving generous headroom
/// while keeping the double-buffered `SharedFrame` compact (40 KB total SRAM).
#[cfg(target_arch = "xtensa")]
pub(crate) const MAX_JPEG_FRAME_BYTES: usize = 20480;

/// Number of concurrent HTTP server worker tasks (`1` control keep-alive + `1` stream + `1` spare/preemption).
#[cfg(target_arch = "xtensa")]
pub const WEB_WORKER_COUNT: usize = 3;

/// Network and SoftAP defaults when `WIFI_SSID` / `WIFI_PASS` are not provided at build time.
#[cfg(target_arch = "xtensa")]
pub mod network {
    /// Default SoftAP SSID when built without `WIFI_SSID`.
    pub const DEFAULT_AP_SSID: &str = "SmartRover_AP";

    /// Default SoftAP WPA2 passphrase (must be >= 8 characters for WPA2-PSK).
    pub const DEFAULT_AP_PASS: &str = "smartrover123";

    /// Static IPv4 address of the rover in SoftAP mode.
    pub const AP_IP_OCTETS: [u8; 4] = [192, 168, 4, 1];

    /// First DHCP lease host octet assigned in SoftAP mode (`192.168.4.100`).
    pub(crate) const AP_DHCP_POOL_START_OCTET: u8 = 100;

    /// Wi-Fi SoftAP RF channel (`1..=13`).
    pub const AP_CHANNEL: u8 = 6;

    /// Delay in milliseconds between STA reconnection attempts.
    pub(crate) const STA_RETRY_DELAY_MS: u64 = 5000;

    /// Interval between RSSI samples while connected in STA mode.
    pub(crate) const RSSI_POLL_INTERVAL_MS: u64 = 2000;
}

/// Pure domain parameters for Gamepad teleoperation & ADAS safety.
pub(crate) mod control {
    /// Control loop execution period in milliseconds (20 Hz).
    #[cfg(target_arch = "xtensa")]
    pub(crate) const TICK_RATE_MS: u64 = 50;

    /// Thumbstick deadzone (0.0 to 1.0) to eliminate controller stick drift.
    pub(crate) const STICK_DEADZONE: f32 = 0.08;

    /// Cubic exponential factor (0.0 = linear, 1.0 = full cubic) for fine low-speed control.
    pub(crate) const STICK_EXPO: f32 = 0.35;

    /// Maximum PWM duty change per 50ms tick (0..1000 scale) to prevent brownout current spikes.
    pub(crate) const PWM_SLEW_RATE_PER_TICK: i32 = 250;

    /// Number of 50ms ticks without a controller packet before the Dead-Man's Switch stops the rover (12 ticks = 600ms).
    pub(crate) const WATCHDOG_TIMEOUT_TICKS: u32 = 12;

    /// Forward distance (mm) at which ADAS blocks forward throttle completely (while allowing reverse/pivot).
    pub(crate) const ADAS_STOP_DISTANCE_MM: u16 = 140;

    /// Forward distance (mm) required to release a latched ADAS stop (hysteresis against ToF jitter).
    pub(crate) const ADAS_STOP_RELEASE_MM: u16 = 165;

    /// Forward distance (mm) below which ADAS progressively scales down forward throttle.
    pub(crate) const ADAS_SLOW_DISTANCE_MM: u16 = 320;

    /// ToF distance ceiling (mm) representing clear / out-of-range space (VL53L0X max specified range).
    pub(crate) const TOF_CLEAR_DISTANCE_MM: u16 = 2000;

    /// Minimum slow-zone throttle ceiling applied before full stop confirmation.
    pub(crate) const ADAS_MIN_THROTTLE_SCALE: f32 = 0.20;

    /// Consecutive ticks required to confirm an ADAS obstacle stop (filters single-tick ToF noise).
    pub(crate) const ADAS_CONFIRM_TICKS: u8 = 2;

    /// Maximum forward throttle allowed when `adas_enabled` is true and the ToF sensor is unhealthy.
    pub(crate) const TOF_FAULT_MAX_THROTTLE: f32 = 0.45;

    /// Minimum battery percentage before low-voltage cutoff protects the LiPo cell.
    pub(crate) const LOW_BATT_CUTOFF_PCT: i32 = 15;

    /// Battery percentage required to recover from low-voltage cutoff (hysteresis).
    pub(crate) const LOW_BATT_RECOVER_PCT: i32 = 20;

    /// Lateral/longitudinal acceleration threshold (in G) indicating the rover has tipped over.
    pub(crate) const TILT_CUTOFF_G: f32 = 0.68;

    /// Minimum upright Z-axis gravity (in G) below which the rover is considered inverted/tipped.
    pub(crate) const MIN_UPRIGHT_AZ_G: f32 = 0.50;

    /// Consecutive ticks above `TILT_CUTOFF_G` (or below `MIN_UPRIGHT_AZ_G`) required to latch rollover protection.
    pub(crate) const TILT_HYSTERESIS_TICKS: u8 = 8;

    /// Minimum total gravity vector squared (g^2) before free-fall is detected (0.4^2 = 0.16).
    pub(crate) const FREEFALL_MIN_G_SQ: f32 = 0.16;
}

/// LiPo battery voltage divider calibration, SoC curve, and smoothing constants.
pub(crate) mod battery {
    /// Upper voltage divider resistor `R201` (`+LIPO` to `GPIO1`) in ohms (10 kΩ, 1%).
    const DIVIDER_R_TOP_OHM: f32 = 10_000.0;
    /// Lower voltage divider resistor `R202` (`GPIO1` to `GND`) in ohms (10 kΩ, 1%).
    const DIVIDER_R_BOTTOM_OHM: f32 = 10_000.0;
    /// Physical voltage divider ratio `(R201 + R202) / R202 = 2.0`.
    pub(crate) const DIVIDER_RATIO: f32 =
        (DIVIDER_R_TOP_OHM + DIVIDER_R_BOTTOM_OHM) / DIVIDER_R_BOTTOM_OHM;
    /// Exponential moving average smoothing factor for ADC millivolt readings (`0.0 < alpha <= 1.0`).
    pub(crate) const ADC_EMA_ALPHA: f32 = 0.25;
    /// Number of consecutive ADC samples averaged (after 1 warm-up read and min/max trim).
    #[cfg(target_arch = "xtensa")]
    pub(crate) const ADC_SAMPLE_COUNT: usize = 16;
    /// Piecewise-linear 1S LiPo voltage-to-SoC lookup table `(voltage_v, soc_pct)`.
    /// Keeps `LOW_BATT_CUTOFF_PCT` (15%) at 3.55 V safely above the 3.3 V buck dropout.
    pub(crate) const LIPO_SOC_TABLE: [(f32, f32); 10] = [
        (3.30, 0.0),
        (3.45, 7.0),
        (3.55, 15.0),
        (3.70, 25.0),
        (3.77, 40.0),
        (3.85, 58.0),
        (3.95, 75.0),
        (4.05, 87.0),
        (4.12, 95.0),
        (4.20, 100.0),
    ];
}
