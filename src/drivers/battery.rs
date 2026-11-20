//! LiPo battery monitor on `ADC1` / `GPIO1` using `esp-hal`'s factory curve calibration (`AdcCalCurve`).

#[cfg(target_arch = "xtensa")]
use esp_hal::{
    Blocking,
    analog::adc::{Adc, AdcCalCurve, AdcPin},
    peripherals::{ADC1, GPIO1},
};

/// Curve-calibrated `ADC1` battery sense pin (`GPIO1` @ `11 dB` attenuation).
#[cfg(target_arch = "xtensa")]
pub type BatteryPin = AdcPin<GPIO1<'static>, ADC1<'static>, AdcCalCurve<ADC1<'static>>>;

/// Computes the trimmed mean of `samples` by discarding the single minimum and maximum values
/// when `samples.len() >= 3`.
fn trimmed_mean_u16(samples: &[u16]) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }
    if samples.len() < 3 {
        let sum: u32 = samples.iter().map(|&s| s as u32).sum();
        return sum as f32 / samples.len() as f32;
    }
    let mut sum = 0u32;
    let mut min_v = u16::MAX;
    let mut max_v = 0u16;
    for &s in samples {
        sum += s as u32;
        if s < min_v {
            min_v = s;
        }
        if s > max_v {
            max_v = s;
        }
    }
    let trimmed_sum = sum - (min_v as u32) - (max_v as u32);
    trimmed_sum as f32 / (samples.len() - 2) as f32
}

/// Exponential moving average smoother for battery ADC millivolt readings.
struct BatteryFilter {
    alpha: f32,
    ema_mv: Option<f32>,
}

impl Default for BatteryFilter {
    fn default() -> Self {
        Self::new(crate::config::battery::ADC_EMA_ALPHA)
    }
}

impl BatteryFilter {
    const fn new(alpha: f32) -> Self {
        Self {
            alpha,
            ema_mv: None,
        }
    }

    fn update(&mut self, sample_mv: u16) -> u16 {
        let s = sample_mv as f32;
        let next = match self.ema_mv {
            Some(prev) => (self.alpha * s) + ((1.0 - self.alpha) * prev),
            None => s,
        };
        self.ema_mv = Some(next);
        (next + 0.5) as u16
    }
}

/// Monitors 1S LiPo battery state-of-charge via the `R201`/`R202` voltage divider.
pub(crate) struct BatteryMonitor {
    #[cfg(target_arch = "xtensa")]
    filter: BatteryFilter,
}

impl BatteryMonitor {
    #[cfg(target_arch = "xtensa")]
    pub(crate) const fn new() -> Self {
        Self {
            filter: BatteryFilter::new(crate::config::battery::ADC_EMA_ALPHA),
        }
    }

    #[cfg(target_arch = "xtensa")]
    pub(crate) fn read_voltage_filtered(
        &mut self,
        adc: &mut Adc<'static, ADC1<'static>, Blocking>,
        pin: &mut BatteryPin,
    ) -> (f32, i32) {
        use crate::config::battery::ADC_SAMPLE_COUNT;

        // Discard 1 warm-up conversion so the SAR S&H capacitor settles across the 10k/10k divider.
        let _ = adc.read_blocking(pin);

        let mut samples = [0u16; ADC_SAMPLE_COUNT];
        for slot in &mut samples {
            *slot = adc.read_blocking(pin);
        }

        let calibrated_mv = trimmed_mean_u16(&samples);
        let smoothed_mv = self.filter.update((calibrated_mv + 0.5) as u16);
        Self::convert_adc_mv(smoothed_mv)
    }

    /// Converts calibrated pin millivolts (`adc_mv`) into `(battery_volts, percentage)`.
    fn convert_adc_mv(adc_mv: u16) -> (f32, i32) {
        let v_batt = (adc_mv as f32 / 1000.0) * crate::config::battery::DIVIDER_RATIO;
        let pct = Self::calculate_percentage(v_batt);
        (v_batt, pct)
    }

    /// Converts battery voltage to clamped state-of-charge percentage (`0..=100`) using a
    /// piecewise-linear 1S LiPo discharge curve (`LIPO_SOC_TABLE`).
    fn calculate_percentage(voltage: f32) -> i32 {
        use crate::config::battery::LIPO_SOC_TABLE;

        if !voltage.is_finite() {
            return 0;
        }
        if voltage <= LIPO_SOC_TABLE[0].0 {
            return 0;
        }
        let last = LIPO_SOC_TABLE[LIPO_SOC_TABLE.len() - 1];
        if voltage >= last.0 {
            return 100;
        }

        for pair in LIPO_SOC_TABLE.windows(2) {
            let (v0, p0) = pair[0];
            let (v1, p1) = pair[1];
            if voltage <= v1 {
                let frac = (voltage - v0) / (v1 - v0);
                let pct = p0 + frac * (p1 - p0);
                return ((pct + 0.5) as i32).clamp(0, 100);
            }
        }
        100
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_battery_percentage_and_adc_conversion() {
        assert_eq!(BatteryMonitor::calculate_percentage(4.2), 100);
        assert_eq!(BatteryMonitor::calculate_percentage(3.3), 0);
        assert_eq!(BatteryMonitor::calculate_percentage(3.55), 15);
        assert_eq!(BatteryMonitor::calculate_percentage(3.95), 75);
        assert_eq!(BatteryMonitor::calculate_percentage(4.091), 92);
        assert_eq!(BatteryMonitor::calculate_percentage(4.5), 100);
        assert_eq!(BatteryMonitor::calculate_percentage(2.5), 0);
        assert_eq!(BatteryMonitor::calculate_percentage(f32::NAN), 0);

        // Ideal 10k / 10k divider: 2063 mV at GPIO1 -> 4.126 V -> 95%
        let (v, pct) = BatteryMonitor::convert_adc_mv(2063);
        assert!((v - 4.126).abs() < 1e-3);
        assert_eq!(pct, 95);

        let mut filter = BatteryFilter::default();
        assert_eq!(filter.update(2000), 2000);
        // Single transient dip should be smoothed rather than dropping straight to 1400mV
        assert_eq!(filter.update(1400), 1850);
    }

    #[test]
    fn test_battery_filter_convergence_and_adc_extremes() {
        let (v_min, pct_min) = BatteryMonitor::convert_adc_mv(0);
        assert_eq!(v_min, 0.0);
        assert_eq!(pct_min, 0);

        let (v_max, pct_max) = BatteryMonitor::convert_adc_mv(u16::MAX);
        assert!((v_max - 131.07).abs() < 1e-2);
        assert_eq!(pct_max, 100);

        // Physical 2:1 divider thresholds (1650mV -> 3.30V -> 0%, 2100mV -> 4.20V -> 100%)
        let (v_empty, pct_empty) = BatteryMonitor::convert_adc_mv(1650);
        assert!((v_empty - 3.3).abs() < 1e-4);
        assert_eq!(pct_empty, 0);
        let (v_full, pct_full) = BatteryMonitor::convert_adc_mv(2100);
        assert!((v_full - 4.2).abs() < 1e-4);
        assert_eq!(pct_full, 100);

        // Monotonicity across 2.8 V .. 4.3 V
        let mut prev_pct = 0;
        for mv in 1400..=2150 {
            let (_, pct) = BatteryMonitor::convert_adc_mv(mv);
            assert!(pct >= prev_pct, "non-monotonic at mv={mv}");
            prev_pct = pct;
        }

        // Rounding behavior (+0.5 before truncation)
        let mut half_filter = BatteryFilter::new(0.5);
        assert_eq!(half_filter.update(1000), 1000);
        assert_eq!(half_filter.update(1001), 1001);

        let mut quarter_filter = BatteryFilter::new(0.25);
        assert_eq!(quarter_filter.update(1000), 1000);
        assert_eq!(quarter_filter.update(1001), 1000);

        // Multi-sample convergence toward steady step input
        let mut conv_filter = BatteryFilter::new(0.25);
        assert_eq!(conv_filter.update(2000), 2000);
        let mut last = 2000;
        for _ in 0..20 {
            let next = conv_filter.update(1600);
            assert!(next <= last);
            last = next;
        }
        assert!((last as i32 - 1600).abs() <= 2);
    }

    #[test]
    fn test_trimmed_mean_drops_outliers() {
        let samples = [100, 2000, 2002, 2001, 1999, 2000, 3900];
        assert!((trimmed_mean_u16(&samples) - 2000.4).abs() < 1e-3);
        assert_eq!(trimmed_mean_u16(&[]), 0.0);
        assert_eq!(trimmed_mean_u16(&[1800, 1802]), 1801.0);
    }
}
