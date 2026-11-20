//! ST VL53L0X Time-of-Flight laser ranging sensor driver (`I2C0`).

use crate::config::control::TOF_CLEAR_DISTANCE_MM;
#[cfg(target_arch = "xtensa")]
use embassy_time::{Duration, Timer};
#[cfg(target_arch = "xtensa")]
use esp_hal::{Blocking, i2c::master::I2c};

/// Error types for the VL53L0X sensor.
#[cfg(target_arch = "xtensa")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Vl53l0xError {
    I2cError,
    Timeout,
    InvalidModelId(u8),
    BufferTooLarge,
}

/// VL53L0X default 7-bit I2C slave address.
#[cfg(target_arch = "xtensa")]
const VL53L0X_I2C_ADDR: u8 = 0x29;

/// Expected VL53L0X identification register (`0xC0`) value.
#[cfg(target_arch = "xtensa")]
const VL53L0X_MODEL_ID: u8 = 0xEE;

/// Internal device range status (`(RESULT_RANGE_STATUS & 0x78) >> 3`) indicating a valid,
/// error-free measurement (`VL53L0X_DEVICEERROR_RANGECOMPLETE_NO_ERROR`).
const VL53L0X_RANGECOMPLETE_INTERNAL_STATUS: u8 = 11;

/// Minimum physically valid ToF target distance (mm) per ST VL53L0X datasheet.
const MIN_VALID_DISTANCE_MM: u16 = 20;

/// Minimum return signal rate in FP9.7 format (`0x0050` = 0.625 MCPS) to reject weak
/// grazing floor reflections. Applied once after ST's default tuning register table.
#[cfg(target_arch = "xtensa")]
const MIN_RETURN_SIGNAL_RATE_FP9_7: u16 = 0x0050;

#[cfg(target_arch = "xtensa")]
mod reg {
    pub(super) const SYSRANGE_START: u8 = 0x00;
    pub(super) const SYSTEM_SEQUENCE_CONFIG: u8 = 0x01;
    pub(super) const SYSTEM_INTERRUPT_CONFIG_GPIO: u8 = 0x0A;
    pub(super) const SYSTEM_INTERRUPT_CLEAR: u8 = 0x0B;
    pub(super) const RESULT_INTERRUPT_STATUS: u8 = 0x13;
    pub(super) const RESULT_RANGE_STATUS: u8 = 0x14;
    pub(super) const FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT: u8 = 0x44;
    pub(super) const DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD: u8 = 0x4E;
    pub(super) const DYNAMIC_SPAD_REF_EN_START_OFFSET: u8 = 0x4F;
    pub(super) const MSRC_CONFIG_CONTROL: u8 = 0x60;
    pub(super) const GPIO_HV_MUX_ACTIVE_HIGH: u8 = 0x84;
    pub(super) const VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV: u8 = 0x89;
    pub(super) const GLOBAL_CONFIG_SPAD_ENABLES_REF_0: u8 = 0xB0;
    pub(super) const GLOBAL_CONFIG_REF_EN_START_SELECT: u8 = 0xB6;
}

/// Validates both `RESULT_RANGE_STATUS` (`0x14`) and the 16-bit distance register (`0x1E..=0x1F`).
///
/// When the VL53L0X looks across a floor into free space, grazing floor reflections or weak
/// returns trigger `Signal Fail` (`9`) or `Phase Fail` (`4`/`6`) in `RESULT_RANGE_STATUS` while
/// leaving phantom `100..250 mm` numbers in `0x1E..=0x1F`. Only status `11` (`RANGECOMPLETE`)
/// within `20..=TOF_CLEAR_DISTANCE_MM` is trusted as a real obstacle; all other readings map to
/// `TOF_CLEAR_DISTANCE_MM` (clear).
const fn sanitize_range_result(range_status: u8, raw_be: [u8; 2]) -> u16 {
    let internal_status = (range_status & 0x78) >> 3;
    if internal_status != VL53L0X_RANGECOMPLETE_INTERNAL_STATUS {
        return TOF_CLEAR_DISTANCE_MM;
    }
    let dist = u16::from_be_bytes(raw_be);
    if dist < MIN_VALID_DISTANCE_MM || dist > TOF_CLEAR_DISTANCE_MM {
        TOF_CLEAR_DISTANCE_MM
    } else {
        dist
    }
}

/// 3-sample median filter for VL53L0X readings to reject single-frame ToF spikes.
#[derive(Debug, Clone, Copy)]
pub(crate) struct DistanceFilter {
    window: [u16; 3],
    count: u8,
    next_idx: usize,
}

impl DistanceFilter {
    pub(crate) const fn new() -> Self {
        Self {
            window: [0; 3],
            count: 0,
            next_idx: 0,
        }
    }

    pub(crate) fn reset(&mut self) {
        self.count = 0;
        self.next_idx = 0;
    }

    pub(crate) fn update(&mut self, sample_mm: u16) -> u16 {
        self.window[self.next_idx] = sample_mm;
        self.next_idx = (self.next_idx + 1) % 3;
        if self.count < 3 {
            self.count += 1;
        }
        match self.count {
            1 => self.window[0],
            2 => self.window[0].min(self.window[1]),
            _ => median3(self.window[0], self.window[1], self.window[2]),
        }
    }
}

const fn median3(a: u16, b: u16, c: u16) -> u16 {
    if a <= b {
        if b <= c {
            b
        } else if a <= c {
            c
        } else {
            a
        }
    } else if a <= c {
        a
    } else if b <= c {
        c
    } else {
        b
    }
}

/// Driver for the VL53L0X Time-of-Flight (ToF) distance sensor.
#[cfg(target_arch = "xtensa")]
pub(crate) struct Vl53l0x {
    stop_variable: u8,
}

#[cfg(target_arch = "xtensa")]
impl Vl53l0x {
    pub(crate) const fn new() -> Self {
        Self { stop_variable: 0 }
    }

    fn write_reg(i2c: &mut I2c<'_, Blocking>, reg: u8, val: u8) -> Result<(), Vl53l0xError> {
        i2c.write(VL53L0X_I2C_ADDR, &[reg, val])
            .map_err(|_| Vl53l0xError::I2cError)
    }

    fn read_reg(i2c: &mut I2c<'_, Blocking>, reg: u8) -> Result<u8, Vl53l0xError> {
        let mut buf = [0u8];
        i2c.write_read(VL53L0X_I2C_ADDR, &[reg], &mut buf)
            .map_err(|_| Vl53l0xError::I2cError)?;
        Ok(buf[0])
    }

    fn write_multi(i2c: &mut I2c<'_, Blocking>, reg: u8, data: &[u8]) -> Result<(), Vl53l0xError> {
        if data.len() > 15 {
            return Err(Vl53l0xError::BufferTooLarge);
        }
        let mut buf = [0u8; 16];
        buf[0] = reg;
        buf[1..1 + data.len()].copy_from_slice(data);
        i2c.write(VL53L0X_I2C_ADDR, &buf[..1 + data.len()])
            .map_err(|_| Vl53l0xError::I2cError)
    }

    fn read_multi(
        i2c: &mut I2c<'_, Blocking>,
        reg: u8,
        data: &mut [u8],
    ) -> Result<(), Vl53l0xError> {
        i2c.write_read(VL53L0X_I2C_ADDR, &[reg], data)
            .map_err(|_| Vl53l0xError::I2cError)
    }

    async fn get_spad_info(i2c: &mut I2c<'_, Blocking>) -> Result<(u8, u8), Vl53l0xError> {
        Self::write_reg(i2c, 0x80, 0x01)?;
        Self::write_reg(i2c, 0xFF, 0x01)?;
        Self::write_reg(i2c, 0x00, 0x00)?;
        Self::write_reg(i2c, 0xFF, 0x06)?;
        let tmp83 = Self::read_reg(i2c, 0x83)?;
        Self::write_reg(i2c, 0x83, tmp83 | 0x04)?;
        Self::write_reg(i2c, 0xFF, 0x07)?;
        Self::write_reg(i2c, 0x81, 0x01)?;
        Self::write_reg(i2c, 0x80, 0x01)?;
        Self::write_reg(i2c, 0x94, 0x6b)?;
        Self::write_reg(i2c, 0x83, 0x00)?;

        let mut timeout = 0;
        while Self::read_reg(i2c, 0x83)? == 0x00 {
            Timer::after(Duration::from_millis(5)).await;
            timeout += 1;
            if timeout > 100 {
                return Err(Vl53l0xError::Timeout);
            }
        }

        Self::write_reg(i2c, 0x83, 0x01)?;
        let tmp = Self::read_reg(i2c, 0x92)?;
        let count = tmp & 0x7f;
        let type_is_aperture = (tmp >> 7) & 0x01;

        Self::write_reg(i2c, 0x81, 0x00)?;
        Self::write_reg(i2c, 0xFF, 0x06)?;
        let tmp83 = Self::read_reg(i2c, 0x83)?;
        Self::write_reg(i2c, 0x83, tmp83 & !0x04)?;
        Self::write_reg(i2c, 0xFF, 0x01)?;
        Self::write_reg(i2c, 0x00, 0x01)?;
        Self::write_reg(i2c, 0xFF, 0x00)?;
        Self::write_reg(i2c, 0x80, 0x00)?;

        Ok((count, type_is_aperture))
    }

    pub(crate) async fn init(&mut self, i2c: &mut I2c<'_, Blocking>) -> Result<(), Vl53l0xError> {
        // Ensure Bank 0 is selected before probing/re-initializing (recovers if a prior
        // I2C failure aborted mid-Bank-1 sequence on an already-powered sensor without XSHUT).
        Self::write_reg(i2c, 0xFF, 0x00)?;
        Self::write_reg(i2c, 0x80, 0x00)?;

        let model_id = Self::read_reg(i2c, 0xC0)?;
        if model_id != VL53L0X_MODEL_ID {
            return Err(Vl53l0xError::InvalidModelId(model_id));
        }

        // Stop any continuous ranging left active across a warm MCU reset or I2C recovery.
        Self::write_reg(i2c, reg::SYSRANGE_START, 0x00)?;
        Self::write_reg(i2c, reg::SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        let vhv = Self::read_reg(i2c, reg::VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV)?;
        Self::write_reg(i2c, reg::VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv | 0x01)?;

        Self::write_reg(i2c, 0x88, 0x00)?;
        Self::write_reg(i2c, 0x80, 0x01)?;
        Self::write_reg(i2c, 0xFF, 0x01)?;
        Self::write_reg(i2c, 0x00, 0x00)?;
        let stop_var = Self::read_reg(i2c, 0x91)?;
        if stop_var != 0 {
            self.stop_variable = stop_var;
        }
        Self::write_reg(i2c, 0x00, 0x01)?;
        Self::write_reg(i2c, 0xFF, 0x00)?;
        Self::write_reg(i2c, 0x80, 0x00)?;

        let msrc = Self::read_reg(i2c, reg::MSRC_CONFIG_CONTROL)?;
        Self::write_reg(i2c, reg::MSRC_CONFIG_CONTROL, msrc | 0x12)?;

        Self::write_reg(i2c, reg::SYSTEM_SEQUENCE_CONFIG, 0xFF)?;

        let (spad_count, spad_type_is_aperture) = Self::get_spad_info(i2c).await?;
        let mut ref_spad_map = [0u8; 6];
        Self::read_multi(
            i2c,
            reg::GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
            &mut ref_spad_map,
        )?;

        Self::write_reg(i2c, 0xFF, 0x01)?;
        Self::write_reg(i2c, reg::DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)?;
        Self::write_reg(i2c, reg::DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)?;
        Self::write_reg(i2c, 0xFF, 0x00)?;
        Self::write_reg(i2c, reg::GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)?;

        let first_spad_to_enable = if spad_type_is_aperture != 0 { 12 } else { 0 };
        let mut spads_enabled = 0;
        for i in 0..48 {
            if i < first_spad_to_enable || spads_enabled == spad_count {
                ref_spad_map[i / 8] &= !(1 << (i % 8));
            } else if (ref_spad_map[i / 8] >> (i % 8)) & 0x1 > 0 {
                spads_enabled += 1;
            }
        }
        Self::write_multi(i2c, reg::GLOBAL_CONFIG_SPAD_ENABLES_REF_0, &ref_spad_map)?;

        for &(reg, val) in VL53L0X_TUNING_REGS {
            Self::write_reg(i2c, reg, val)?;
        }

        // Re-enable Bank 1 ranging core (0x00 = 0x01) after tuning before returning to Bank 0.
        Self::write_reg(i2c, 0xFF, 0x01)?;
        Self::write_reg(i2c, 0x8E, 0x01)?;
        Self::write_reg(i2c, 0x00, 0x01)?;
        Self::write_reg(i2c, 0xFF, 0x00)?;
        Self::write_reg(i2c, 0x80, 0x00)?;

        // Apply custom return-signal-rate floor once after ST's tuning table so it is not shadowed.
        let [rate_hi, rate_lo] = MIN_RETURN_SIGNAL_RATE_FP9_7.to_be_bytes();
        Self::write_reg(
            i2c,
            reg::FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
            rate_hi,
        )?;
        Self::write_reg(
            i2c,
            reg::FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT + 1,
            rate_lo,
        )?;

        Self::write_reg(i2c, reg::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)?;
        let gpio_hv = Self::read_reg(i2c, reg::GPIO_HV_MUX_ACTIVE_HIGH)?;
        Self::write_reg(i2c, reg::GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv & !0x10)?;
        Self::write_reg(i2c, reg::SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        Self::write_reg(i2c, reg::SYSTEM_SEQUENCE_CONFIG, 0x01)?;
        Self::perform_single_ref_calibration(i2c, 0x40).await?;

        Self::write_reg(i2c, reg::SYSTEM_SEQUENCE_CONFIG, 0x02)?;
        Self::perform_single_ref_calibration(i2c, 0x00).await?;

        Self::write_reg(i2c, reg::SYSTEM_SEQUENCE_CONFIG, 0xE8)?;

        self.start_continuous(i2c)?;
        Timer::after(Duration::from_millis(10)).await;
        Ok(())
    }

    async fn perform_single_ref_calibration(
        i2c: &mut I2c<'_, Blocking>,
        vhv_init_byte: u8,
    ) -> Result<(), Vl53l0xError> {
        Self::write_reg(i2c, reg::SYSRANGE_START, 0x01 | vhv_init_byte)?;

        let mut timeout = 0;
        while (Self::read_reg(i2c, reg::RESULT_INTERRUPT_STATUS)? & 0x07) == 0 {
            Timer::after(Duration::from_millis(5)).await;
            timeout += 1;
            if timeout > 100 {
                return Err(Vl53l0xError::Timeout);
            }
        }

        Self::write_reg(i2c, reg::SYSTEM_INTERRUPT_CLEAR, 0x01)?;
        Self::write_reg(i2c, reg::SYSRANGE_START, 0x00)
    }

    /// Starts continuous back-to-back ranging so 20 Hz reads avoid 30–60 ms single-shot stalls.
    fn start_continuous(&self, i2c: &mut I2c<'_, Blocking>) -> Result<(), Vl53l0xError> {
        Self::write_reg(i2c, 0x80, 0x01)?;
        Self::write_reg(i2c, 0xFF, 0x01)?;
        Self::write_reg(i2c, 0x00, 0x00)?;
        Self::write_reg(i2c, 0x91, self.stop_variable)?;
        Self::write_reg(i2c, 0x00, 0x01)?;
        Self::write_reg(i2c, 0xFF, 0x00)?;
        Self::write_reg(i2c, 0x80, 0x00)?;
        Self::write_reg(i2c, reg::SYSRANGE_START, 0x02)
    }

    /// Non-blocking poll of continuous ranging results. Returns `Ok(Some(mm))` when a new
    /// measurement is ready, or `Ok(None)` if the current integration is still in progress.
    pub(crate) fn poll_distance(
        &self,
        i2c: &mut I2c<'_, Blocking>,
    ) -> Result<Option<u16>, Vl53l0xError> {
        let st = Self::read_reg(i2c, reg::RESULT_INTERRUPT_STATUS)?;
        if (st & 0x07) == 0 {
            return Ok(None);
        }

        // Read the 12-byte result block (0x14..=0x1F) so both RESULT_RANGE_STATUS (buf[0])
        // and the 16-bit distance register (buf[10..=11] at 0x1E..=0x1F) are validated together.
        let mut buf = [0u8; 12];
        Self::read_multi(i2c, reg::RESULT_RANGE_STATUS, &mut buf)?;
        Self::write_reg(i2c, reg::SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        Ok(Some(sanitize_range_result(buf[0], [buf[10], buf[11]])))
    }
}

#[cfg(target_arch = "xtensa")]
const VL53L0X_TUNING_REGS: &[(u8, u8)] = &[
    (0xFF, 0x01),
    (0x00, 0x00),
    (0xFF, 0x00),
    (0x09, 0x00),
    (0x10, 0x00),
    (0x11, 0x00),
    (0x24, 0x01),
    (0x25, 0xFF),
    (0x75, 0x00),
    (0xFF, 0x01),
    (0x4E, 0x2C),
    (0x48, 0x00),
    (0x30, 0x20),
    (0xFF, 0x00),
    (0x30, 0x09),
    (0x54, 0x00),
    (0x31, 0x04),
    (0x32, 0x03),
    (0x40, 0x83),
    (0x46, 0x25),
    (0x60, 0x00),
    (0x27, 0x00),
    (0x50, 0x06),
    (0x51, 0x00),
    (0x52, 0x96),
    (0x56, 0x08),
    (0x57, 0x30),
    (0x61, 0x00),
    (0x62, 0x00),
    (0x64, 0x00),
    (0x65, 0x00),
    (0x66, 0xA0),
    (0xFF, 0x01),
    (0x22, 0x32),
    (0x47, 0x14),
    (0x49, 0xFF),
    (0x4A, 0x00),
    (0xFF, 0x00),
    (0x7A, 0x0A),
    (0x7B, 0x00),
    (0x78, 0x21),
    (0xFF, 0x01),
    (0x23, 0x34),
    (0x42, 0x00),
    (0x44, 0xFF),
    (0x45, 0x26),
    (0x46, 0x05),
    (0x40, 0x40),
    (0x0E, 0x06),
    (0x20, 0x1A),
    (0x43, 0x40),
    (0xFF, 0x00),
    (0x34, 0x03),
    (0x35, 0x44),
    (0xFF, 0x01),
    (0x31, 0x04),
    (0x4B, 0x09),
    (0x4C, 0x05),
    (0x4D, 0x04),
    (0xFF, 0x00),
    (0x44, 0x00),
    (0x45, 0x20),
    (0x47, 0x08),
    (0x48, 0x28),
    (0x67, 0x00),
    (0x70, 0x04),
    (0x71, 0x01),
    (0x72, 0xFE),
    (0x76, 0x00),
    (0x77, 0x00),
    (0xFF, 0x01),
    (0x0D, 0x01),
    (0xFF, 0x00),
    (0x80, 0x01),
    (0x01, 0xF8),
];

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_vl53l0x_range_sanitization_boundaries() {
        let valid_status = VL53L0X_RANGECOMPLETE_INTERNAL_STATUS << 3; // 0x58

        // Normal reading: 350mm (0x015E)
        assert_eq!(
            sanitize_range_result(valid_status, 350u16.to_be_bytes()),
            350
        );

        // Out-of-range (0mm, sub-20mm crosstalk, >2000mm, 8190mm, u16::MAX) -> TOF_CLEAR_DISTANCE_MM
        assert_eq!(
            sanitize_range_result(valid_status, [0, 0]),
            TOF_CLEAR_DISTANCE_MM
        );
        assert_eq!(
            sanitize_range_result(valid_status, 1u16.to_be_bytes()),
            TOF_CLEAR_DISTANCE_MM
        );
        assert_eq!(
            sanitize_range_result(valid_status, (MIN_VALID_DISTANCE_MM - 1).to_be_bytes()),
            TOF_CLEAR_DISTANCE_MM
        );
        assert_eq!(
            sanitize_range_result(valid_status, MIN_VALID_DISTANCE_MM.to_be_bytes()),
            MIN_VALID_DISTANCE_MM
        );
        assert_eq!(
            sanitize_range_result(valid_status, TOF_CLEAR_DISTANCE_MM.to_be_bytes()),
            TOF_CLEAR_DISTANCE_MM
        );
        assert_eq!(
            sanitize_range_result(valid_status, (TOF_CLEAR_DISTANCE_MM + 1).to_be_bytes()),
            TOF_CLEAR_DISTANCE_MM
        );
        assert_eq!(
            sanitize_range_result(valid_status, 8190u16.to_be_bytes()),
            TOF_CLEAR_DISTANCE_MM
        );
        assert_eq!(
            sanitize_range_result(valid_status, u16::MAX.to_be_bytes()),
            TOF_CLEAR_DISTANCE_MM
        );
    }

    #[test]
    fn test_vl53l0x_range_status_filters_phantom_floor_reflections() {
        let valid_status = VL53L0X_RANGECOMPLETE_INTERNAL_STATUS << 3; // 0x58
        let signal_fail_status = 9u8 << 3; // Signal rate below 0.625 MCPS threshold
        let phase_fail_status = 4u8 << 3; // Phase out-of-bounds / wrap failure

        // Valid status (11) with a real 240mm obstacle is preserved
        assert_eq!(
            sanitize_range_result(valid_status, 240u16.to_be_bytes()),
            240
        );

        // Phantom 140mm or 180mm floor readings during Signal Fail or Phase Fail map to clear
        assert_eq!(
            sanitize_range_result(signal_fail_status, 140u16.to_be_bytes()),
            TOF_CLEAR_DISTANCE_MM
        );
        assert_eq!(
            sanitize_range_result(phase_fail_status, 180u16.to_be_bytes()),
            TOF_CLEAR_DISTANCE_MM
        );

        // Sub-20mm coverglass/aperture crosstalk with valid status also maps to clear
        assert_eq!(
            sanitize_range_result(valid_status, 12u16.to_be_bytes()),
            TOF_CLEAR_DISTANCE_MM
        );
        assert_eq!(sanitize_range_result(valid_status, 20u16.to_be_bytes()), 20);
    }

    #[test]
    fn test_distance_filter_rejects_single_sample_spikes_and_resets() {
        let mut filter = DistanceFilter::new();

        // Warm-up: 1st sample passes through, 2nd uses conservative min
        assert_eq!(filter.update(800), 800);
        assert_eq!(filter.update(820), 800);

        // Single-tick close spike is rejected by 3-sample median
        assert_eq!(filter.update(120), 800);
        assert_eq!(filter.update(810), 810);
        assert_eq!(filter.update(815), 810);
        assert_eq!(filter.update(820), 815);

        // Sustained real obstacle passes through on the 2nd consecutive close reading
        assert_eq!(filter.update(130), 815);
        assert_eq!(filter.update(125), 130);
        assert_eq!(filter.update(125), 125);

        // Reset discards prior history after sensor recovery
        filter.reset();
        assert_eq!(filter.update(900), 900);
    }
}
