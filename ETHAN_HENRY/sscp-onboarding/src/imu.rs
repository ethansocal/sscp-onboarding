//! Minimal blocking SPI driver for the ST ISM330DLC 6-axis IMU.
//!
//! Register map and scaling values come from the ISM330DLC datasheet (DocID 031040).

use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{self, Spi, mode::Master};

/// Expected value of the WHO_AM_I register.
pub const WHO_AM_I_VALUE: u8 = 0x6A;

mod reg {
    pub const INT1_CTRL: u8 = 0x0D;
    pub const INT2_CTRL: u8 = 0x0E;
    pub const WHO_AM_I: u8 = 0x0F;
    pub const CTRL1_XL: u8 = 0x10;
    pub const CTRL2_G: u8 = 0x11;
    pub const CTRL3_C: u8 = 0x12;
    pub const CTRL4_C: u8 = 0x13;
    pub const OUT_TEMP_L: u8 = 0x20;
}

/// Set on the address byte to make an SPI read instead of a write.
const READ: u8 = 0x80;

// CTRL3_C bits
const CTRL3_SW_RESET: u8 = 1 << 0;
const CTRL3_IF_INC: u8 = 1 << 2;
const CTRL3_BDU: u8 = 1 << 6;
// CTRL4_C bits
const CTRL4_I2C_DISABLE: u8 = 1 << 2;

/// 104 Hz output data rate (ODR bits [7:4] = 0100) for both sensors.
const ODR_104HZ: u8 = 0b0100 << 4;
/// Accelerometer full scale ±4 g (FS_XL bits [3:2] = 10).
const FS_XL_4G: u8 = 0b10 << 2;
/// Gyroscope full scale ±500 dps (FS_G bits [3:2] = 01).
const FS_G_500DPS: u8 = 0b01 << 2;

/// Sensitivity for ±4 g, in g per LSB (0.122 mg/LSB).
const ACCEL_G_PER_LSB: f32 = 0.122e-3;
/// Sensitivity for ±500 dps, in dps per LSB (17.50 mdps/LSB).
const GYRO_DPS_PER_LSB: f32 = 17.5e-3;

#[derive(Debug, defmt::Format)]
pub enum Error {
    Spi(spi::Error),
    /// WHO_AM_I returned something other than [`WHO_AM_I_VALUE`].
    BadWhoAmI(u8),
}

impl From<spi::Error> for Error {
    fn from(e: spi::Error) -> Self {
        Error::Spi(e)
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct Reading {
    /// Temperature in °C.
    pub temp_c: f32,
    /// Angular rate in degrees per second, [x, y, z].
    pub gyro_dps: [f32; 3],
    /// Acceleration in g, [x, y, z].
    pub accel_g: [f32; 3],
}

pub struct Ism330dlc<'d> {
    spi: Spi<'d, Blocking, Master>,
    cs: Output<'d>,
}

impl<'d> Ism330dlc<'d> {
    /// `spi` must be configured for SPI mode 3 at ≤10 MHz. `cs` should start high.
    pub fn new(spi: Spi<'d, Blocking, Master>, cs: Output<'d>) -> Self {
        Self { spi, cs }
    }

    /// Resets the IMU, checks WHO_AM_I and starts both sensors at 104 Hz.
    /// INT1 is routed to accelerometer data-ready, INT2 to gyroscope data-ready.
    pub fn init(&mut self, delay: &mut cortex_m::delay::Delay) -> Result<(), Error> {
        // Disable the I2C interface first so SDA/SCL activity can't confuse it.
        self.write_reg(reg::CTRL4_C, CTRL4_I2C_DISABLE)?;

        let id = self.read_reg(reg::WHO_AM_I)?;
        if id != WHO_AM_I_VALUE {
            return Err(Error::BadWhoAmI(id));
        }

        self.write_reg(reg::CTRL3_C, CTRL3_SW_RESET)?;
        delay.delay_ms(10);
        // The reset clears CTRL4_C, so disable I2C again.
        self.write_reg(reg::CTRL4_C, CTRL4_I2C_DISABLE)?;
        // Block data update keeps the low/high bytes of a sample consistent;
        // address auto-increment allows burst reads.
        self.write_reg(reg::CTRL3_C, CTRL3_BDU | CTRL3_IF_INC)?;

        self.write_reg(reg::CTRL1_XL, ODR_104HZ | FS_XL_4G)?;
        self.write_reg(reg::CTRL2_G, ODR_104HZ | FS_G_500DPS)?;

        self.write_reg(reg::INT1_CTRL, 1 << 0)?; // INT1_DRDY_XL
        self.write_reg(reg::INT2_CTRL, 1 << 1)?; // INT2_DRDY_G
        Ok(())
    }

    /// Burst-reads temperature, gyroscope and accelerometer (0x20..=0x2D).
    pub fn read(&mut self) -> Result<Reading, Error> {
        let mut raw = [0u8; 14];
        self.read_regs(reg::OUT_TEMP_L, &mut raw)?;

        let word = |i: usize| i16::from_le_bytes([raw[i], raw[i + 1]]) as f32;
        Ok(Reading {
            // 256 LSB/°C, 0 LSB = 25 °C
            temp_c: 25.0 + word(0) / 256.0,
            gyro_dps: [2, 4, 6].map(|i| word(i) * GYRO_DPS_PER_LSB),
            accel_g: [8, 10, 12].map(|i| word(i) * ACCEL_G_PER_LSB),
        })
    }

    pub fn read_reg(&mut self, addr: u8) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        self.read_regs(addr, &mut buf)?;
        Ok(buf[0])
    }

    pub fn write_reg(&mut self, addr: u8, value: u8) -> Result<(), Error> {
        self.cs.set_low();
        let res = self.spi.blocking_write(&[addr & !READ, value]);
        self.cs.set_high();
        Ok(res?)
    }

    fn read_regs(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error> {
        self.cs.set_low();
        let res = self
            .spi
            .blocking_write(&[addr | READ])
            .and_then(|()| self.spi.blocking_transfer_in_place(buf));
        self.cs.set_high();
        Ok(res?)
    }
}
