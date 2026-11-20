//! Bosch BMI323 6-axis IMU driver (`I2C0`).

#[cfg(target_arch = "xtensa")]
use embassy_time::{Duration, Timer};
#[cfg(target_arch = "xtensa")]
use esp_hal::{Blocking, i2c::master::I2c};

/// Error type for the BMI323 IMU.
#[cfg(target_arch = "xtensa")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Bmi323Error {
    I2cError,
    InvalidChipId(u8),
}

/// BMI323 I2C 7-bit address (`SDO` tied to `GND`).
#[cfg(target_arch = "xtensa")]
const BMI323_I2C_ADDR: u8 = 0x68;

/// Expected BMI323 `CHIP_ID` (register `0x00`, LSB).
const BMI323_CHIP_ID: u8 = 0x43;

/// Decodes a 4-byte BMI323 `CHIP_ID` I2C response (`2 dummy bytes + 16-bit ID`).
const fn decode_chip_id(buf: [u8; 4]) -> u8 {
    buf[2]
}

/// Decodes an 8-byte BMI323 accelerometer register burst (`0x03..0x0A`, with 2 dummy status bytes)
/// into `(ax_g, ay_g, az_g)` at +/-2g sensitivity (16384 LSB/g).
fn decode_accel_g(buf: &[u8; 8]) -> (f32, f32, f32) {
    let ax = i16::from_le_bytes([buf[2], buf[3]]) as f32 / 16384.0;
    let ay = i16::from_le_bytes([buf[4], buf[5]]) as f32 / 16384.0;
    let az = i16::from_le_bytes([buf[6], buf[7]]) as f32 / 16384.0;
    (ax, ay, az)
}

/// Driver for the BMI323 Inertial Measurement Unit (IMU).
#[cfg(target_arch = "xtensa")]
pub(crate) struct Bmi323;

#[cfg(target_arch = "xtensa")]
impl Bmi323 {
    pub(crate) const fn new() -> Self {
        Self
    }

    pub(crate) async fn init(&self, i2c: &mut I2c<'_, Blocking>) -> Result<(), Bmi323Error> {
        let mut id_buf = [0u8; 4];
        i2c.write_read(BMI323_I2C_ADDR, &[0x00], &mut id_buf)
            .map_err(|_| Bmi323Error::I2cError)?;
        let chip_id = decode_chip_id(id_buf);
        if chip_id != BMI323_CHIP_ID {
            return Err(Bmi323Error::InvalidChipId(chip_id));
        }

        // Soft reset
        i2c.write(BMI323_I2C_ADDR, &[0x7E, 0xAF, 0xDE])
            .map_err(|_| Bmi323Error::I2cError)?;
        Timer::after(Duration::from_millis(50)).await;

        // Configure Accelerometer (Reg 0x20): 100Hz, Normal Mode
        i2c.write(BMI323_I2C_ADDR, &[0x20, 0x08, 0x40])
            .map_err(|_| Bmi323Error::I2cError)?;

        Ok(())
    }

    pub(crate) fn read_accel(
        &self,
        i2c: &mut I2c<'_, Blocking>,
    ) -> Result<(f32, f32, f32), Bmi323Error> {
        let mut buf = [0u8; 8];
        i2c.write_read(BMI323_I2C_ADDR, &[0x03], &mut buf)
            .map_err(|_| Bmi323Error::I2cError)?;
        Ok(decode_accel_g(&buf))
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn test_bmi323_chip_id_and_accel_decoding() {
        // 16384 LSB = +1.0g on Z, -8192 LSB = -0.5g on X, +8192 LSB = +0.5g on Y
        let x_bytes = (-8192i16).to_le_bytes();
        let y_bytes = 8192i16.to_le_bytes();
        let z_bytes = 16384i16.to_le_bytes();
        let raw = [
            0x00, 0x00, x_bytes[0], x_bytes[1], y_bytes[0], y_bytes[1], z_bytes[0], z_bytes[1],
        ];

        let (ax, ay, az) = decode_accel_g(&raw);
        assert!((ax + 0.5).abs() < 1e-4);
        assert!((ay - 0.5).abs() < 1e-4);
        assert!((az - 1.0).abs() < 1e-4);

        assert_eq!(
            decode_chip_id([0x00, 0x00, BMI323_CHIP_ID, 0x00]),
            BMI323_CHIP_ID
        );
    }

    #[test]
    fn test_bmi323_accel_extremes_and_chip_id_mismatch() {
        // Zero acceleration
        assert_eq!(decode_accel_g(&[0u8; 8]), (0.0, 0.0, 0.0));

        // i16::MIN (-2.0g) and i16::MAX (~+1.999939g)
        let min_b = i16::MIN.to_le_bytes();
        let max_b = i16::MAX.to_le_bytes();
        let rep_b = 4096i16.to_le_bytes(); // +0.25g
        let buf = [
            0xAA, 0xBB, min_b[0], min_b[1], max_b[0], max_b[1], rep_b[0], rep_b[1],
        ];
        let (ax, ay, az) = decode_accel_g(&buf);
        assert_eq!(ax, -2.0);
        assert!((ay - (32767.0 / 16384.0)).abs() < 1e-6);
        assert_eq!(az, 0.25);

        // Chip ID mismatch values
        assert_ne!(decode_chip_id([0x00, 0x00, 0x00, 0x43]), BMI323_CHIP_ID);
        assert_ne!(decode_chip_id([0x00, 0x00, 0xFF, 0x00]), BMI323_CHIP_ID);
        assert_eq!(decode_chip_id([0xFF, 0xFF, 0x42, 0x00]), 0x42);
    }
}
