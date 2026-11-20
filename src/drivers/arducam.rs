//! Arducam Mini 5MP Plus (`OV5642`) camera driver (`SPI2` + `I2C0`).

use esp_hal::{Blocking, delay::Delay, gpio::Output, i2c::master::I2c, spi::master::Spi};

/// Error types for the Arducam Mini 5MP Plus (`OV5642`) driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArducamError {
    SpiError,
    SpiProbeFailed,
    I2cError,
    InvalidSensorId(u16),
}

/// SPI FIFO burst-read chunk size kept on stack (non-async helper keeps task arena compact).
const SPI_BURST_CHUNK_BYTES: usize = 2048;

pub struct Arducam {
    cs_pin: Output<'static>,
    delay: Delay,
}

impl Arducam {
    pub fn new(cs: Output<'static>) -> Self {
        Self {
            cs_pin: cs,
            delay: Delay::new(),
        }
    }

    pub fn init(
        &mut self,
        i2c: &mut I2c<'static, Blocking>,
        spi: &mut Spi<'static, Blocking>,
    ) -> Result<(), ArducamError> {
        self.write_spi_reg(spi, 0x00, 0x55)?;
        if self.read_spi_reg(spi, 0x00)? != 0x55 {
            return Err(ArducamError::SpiProbeFailed);
        }

        self.write_spi_reg(spi, 0x06, 0x05)?;
        self.delay.delay_millis(100);

        self.write_spi_reg(spi, 0x01, 0x01)?;

        let vid = Self::read_sensor_reg(i2c, 0x300A)?;
        let pid = Self::read_sensor_reg(i2c, 0x300B)?;
        let chip_id = ((vid as u16) << 8) | (pid as u16);
        if chip_id != 0x5642 {
            return Err(ArducamError::InvalidSensorId(chip_id));
        }

        self.init_jpeg_320x240(i2c, spi)?;

        Ok(())
    }

    pub(crate) fn start_capture(
        &mut self,
        spi: &mut Spi<'static, Blocking>,
    ) -> Result<(), ArducamError> {
        self.write_spi_reg(spi, 0x04, 0x01)?; // FIFO_CLEAR
        self.write_spi_reg(spi, 0x04, 0x00)?;
        self.write_spi_reg(spi, 0x04, 0x02) // FIFO_START
    }

    pub(crate) fn flush_fifo(
        &mut self,
        spi: &mut Spi<'static, Blocking>,
    ) -> Result<(), ArducamError> {
        self.write_spi_reg(spi, 0x04, 0x01)?; // FIFO_CLEAR
        self.write_spi_reg(spi, 0x04, 0x00)
    }

    pub(crate) fn clear_fifo_flag(
        &mut self,
        spi: &mut Spi<'static, Blocking>,
    ) -> Result<(), ArducamError> {
        self.write_spi_reg(spi, 0x41, 0x01)
    }

    pub(crate) fn is_capture_done(
        &mut self,
        spi: &mut Spi<'static, Blocking>,
    ) -> Result<bool, ArducamError> {
        Ok((self.read_spi_reg(spi, 0x41)? & 0x08) != 0)
    }

    pub(crate) fn get_fifo_length(
        &mut self,
        spi: &mut Spi<'static, Blocking>,
    ) -> Result<u32, ArducamError> {
        let l1 = self.read_spi_reg(spi, 0x42)? as u32;
        let l2 = self.read_spi_reg(spi, 0x43)? as u32;
        let l3 = (self.read_spi_reg(spi, 0x44)? & 0x7F) as u32;
        Ok((l3 << 16) | (l2 << 8) | l1)
    }

    /// Synchronous SPI FIFO burst read (`0x3C`) using a 2 KB stack staging buffer so the
    /// Embassy async task arena stays compact and the 8 MHz burst runs without timer yields.
    pub(crate) fn read_fifo_burst<const N: usize>(
        &mut self,
        spi: &mut Spi<'static, Blocking>,
        dst: &mut heapless::Vec<u8, N>,
        read_len: usize,
    ) -> bool {
        dst.clear();
        self.cs_pin.set_low();
        let mut burst_ok = spi.write(&[0x3C]).is_ok(); // BURST_READ

        if burst_ok {
            let mut chunk = [0u8; SPI_BURST_CHUNK_BYTES];
            for offset in (0..read_len).step_by(SPI_BURST_CHUNK_BYTES) {
                let chunk_len = (read_len - offset).min(SPI_BURST_CHUNK_BYTES);
                if spi.read(&mut chunk[..chunk_len]).is_err()
                    || dst.extend_from_slice(&chunk[..chunk_len]).is_err()
                {
                    burst_ok = false;
                    break;
                }
            }
        }

        self.cs_pin.set_high();
        burst_ok
    }

    fn write_spi_reg(
        &mut self,
        spi: &mut Spi<'static, Blocking>,
        reg: u8,
        data: u8,
    ) -> Result<(), ArducamError> {
        self.cs_pin.set_low();
        self.delay.delay_micros(1);
        let res = spi.write(&[reg | 0x80, data]);
        self.delay.delay_micros(1);
        self.cs_pin.set_high();
        self.delay.delay_micros(1);
        res.map_err(|_| ArducamError::SpiError)
    }

    fn read_spi_reg(
        &mut self,
        spi: &mut Spi<'static, Blocking>,
        reg: u8,
    ) -> Result<u8, ArducamError> {
        let mut buf = [reg & 0x7F, 0x00];
        self.cs_pin.set_low();
        self.delay.delay_micros(1);
        let res = spi.transfer(&mut buf);
        self.delay.delay_micros(1);
        self.cs_pin.set_high();
        self.delay.delay_micros(1);
        res.map_err(|_| ArducamError::SpiError)?;
        Ok(buf[1])
    }

    fn read_sensor_reg(i2c: &mut I2c<'static, Blocking>, reg: u16) -> Result<u8, ArducamError> {
        let addr = reg.to_be_bytes();
        let mut buf = [0u8; 1];
        i2c.write_read(0x3C, &addr, &mut buf)
            .map_err(|_| ArducamError::I2cError)?;
        Ok(buf[0])
    }

    fn write_sensor_reg(
        i2c: &mut I2c<'static, Blocking>,
        reg: u16,
        data: u8,
    ) -> Result<(), ArducamError> {
        let addr = reg.to_be_bytes();
        i2c.write(0x3C, &[addr[0], addr[1], data])
            .map_err(|_| ArducamError::I2cError)
    }

    fn init_jpeg_320x240(
        &mut self,
        i2c: &mut I2c<'static, Blocking>,
        spi: &mut Spi<'static, Blocking>,
    ) -> Result<(), ArducamError> {
        Self::write_sensor_reg(i2c, 0x3008, 0x80)?;
        self.delay.delay_millis(100);

        for &(reg, val) in OV5642_QVGA_PREVIEW {
            Self::write_sensor_reg(i2c, reg, val)?;
        }
        self.delay.delay_millis(100);

        for &(reg, val) in OV5642_JPEG_CAPTURE_QSXGA {
            Self::write_sensor_reg(i2c, reg, val)?;
        }

        for &(reg, val) in OV5642_320X240_JPEG {
            Self::write_sensor_reg(i2c, reg, val)?;
        }
        self.delay.delay_millis(100);

        Self::write_sensor_reg(i2c, 0x3818, 0xa8)?;
        Self::write_sensor_reg(i2c, 0x3621, 0x10)?;
        Self::write_sensor_reg(i2c, 0x3801, 0xb0)?;
        // OV5642 quantization scale 0x0C keeps QVGA JPEGs compact (~10-15 KB)
        // for higher Wi-Fi streaming frame rate and ample headroom inside MAX_JPEG_FRAME_BYTES.
        Self::write_sensor_reg(i2c, 0x4407, 0x0C)?;
        Self::write_sensor_reg(i2c, 0x5888, 0x00)?;
        Self::write_sensor_reg(i2c, 0x5000, 0xFF)?;
        self.write_spi_reg(spi, 0x03, 0x02)?;
        Ok(())
    }
}

const OV5642_QVGA_PREVIEW: &[(u16, u8)] = &[
    (0x3103, 0x93),
    (0x3008, 0x82),
    (0x3017, 0x7f),
    (0x3018, 0xfc),
    (0x3810, 0xc2),
    (0x3615, 0xf0),
    (0x3000, 0x00),
    (0x3001, 0x00),
    (0x3002, 0x5c),
    (0x3003, 0x00),
    (0x3004, 0xff),
    (0x3005, 0xff),
    (0x3006, 0x43),
    (0x3007, 0x37),
    (0x3011, 0x08),
    (0x3010, 0x10),
    (0x460c, 0x22),
    (0x3815, 0x04),
    (0x370c, 0xa0),
    (0x3602, 0xfc),
    (0x3612, 0xff),
    (0x3634, 0xc0),
    (0x3613, 0x00),
    (0x3605, 0x7c),
    (0x3621, 0x09),
    (0x3622, 0x60),
    (0x3604, 0x40),
    (0x3603, 0x27),
    (0x4000, 0x21),
    (0x401d, 0x22),
    (0x3600, 0x54),
    (0x3605, 0x04),
    (0x3606, 0x3f),
    (0x3c01, 0x80),
    (0x5000, 0x4f),
    (0x5020, 0x04),
    (0x5181, 0x79),
];
const OV5642_320X240_JPEG: &[(u16, u8)] = &[
    (0x3800, 0x01),
    (0x3801, 0xa8),
    (0x3802, 0x00),
    (0x3803, 0x0a),
    (0x3804, 0x0a),
    (0x3805, 0x20),
    (0x3806, 0x07),
    (0x3807, 0x98),
    (0x3808, 0x01),
    (0x3809, 0x40),
    (0x380a, 0x00),
    (0x380b, 0xf0),
    (0x380c, 0x0c),
    (0x380d, 0x80),
    (0x380e, 0x07),
    (0x380f, 0xd0),
    (0x5001, 0x7f),
    (0x5680, 0x00),
    (0x5681, 0x00),
    (0x5682, 0x0a),
    (0x5683, 0x20),
    (0x5684, 0x00),
    (0x5685, 0x00),
    (0x5686, 0x07),
    (0x5687, 0x98),
    (0x3801, 0xb0),
];
const OV5642_JPEG_CAPTURE_QSXGA: &[(u16, u8)] = &[
    (0x3503, 0x07),
    (0x3000, 0x00),
    (0x3001, 0x00),
    (0x3002, 0x00),
    (0x3003, 0x00),
    (0x3005, 0xff),
    (0x3006, 0xff),
    (0x3007, 0x3f),
    (0x350c, 0x07),
    (0x350d, 0xd0),
    (0x3602, 0xe4),
    (0x3612, 0xac),
    (0x3613, 0x44),
    (0x3621, 0x27),
    (0x3622, 0x08),
    (0x3623, 0x22),
    (0x3604, 0x60),
    (0x3705, 0xda),
    (0x370a, 0x80),
    (0x3801, 0x8a),
    (0x3803, 0x0a),
    (0x3804, 0x0a),
    (0x3805, 0x20),
    (0x3806, 0x07),
    (0x3807, 0x98),
    (0x3808, 0x0a),
    (0x3809, 0x20),
    (0x380a, 0x07),
    (0x380b, 0x98),
    (0x380c, 0x0c),
    (0x380d, 0x80),
    (0x380e, 0x07),
    (0x380f, 0xd0),
    (0x3810, 0xc2),
    (0x3815, 0x44),
    (0x3818, 0xc8),
    (0x3824, 0x01),
    (0x3827, 0x0a),
    (0x3a00, 0x78),
    (0x3a0d, 0x10),
    (0x3a0e, 0x0d),
    (0x3a10, 0x32),
    (0x3a1b, 0x3c),
    (0x3a1e, 0x32),
    (0x3a11, 0x80),
    (0x3a1f, 0x20),
    (0x460b, 0x35),
    (0x471d, 0x00),
    (0x4713, 0x03),
    (0x471c, 0x50),
    (0x5682, 0x0a),
    (0x5683, 0x20),
    (0x5686, 0x07),
    (0x5687, 0x98),
    (0x5001, 0x4f),
    (0x589b, 0x00),
    (0x589a, 0xc0),
    (0x4407, 0x08),
    (0x3002, 0x0c),
    (0x3002, 0x00),
    (0x3503, 0x00),
    (0x5025, 0x80),
];
