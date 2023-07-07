use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Read, Write, WriteRead},
};

#[derive(Debug)]
pub enum Error<E> {
    I2C(E),
    InvalidChipId(u8),
}

pub const STATUS_BASE: u8 = 0x00;
pub const STATUS_HWID: u8 = 0x01;
pub const STATUS_VERSION: u8 = 0x02;
pub const STATUS_SWRST: u8 = 0x7F;

pub const RESET_VALUE: u8 = 0xFF;

pub const ENCODER_BASE: u8 = 0x11;
pub const ENCODER_STATUS: u8 = 0x00;
pub const ENCODER_INTENSET: u8 = 0x10;
pub const ENCODER_INTENCLR: u8 = 0x20;
pub const ENCODER_POSITION: u8 = 0x30;
pub const ENCODER_DELTA: u8 = 0x40;

pub const CODE_SWRST: [u8; 3] = [STATUS_BASE, STATUS_SWRST, RESET_VALUE];
pub const CODE_CHIPID: [u8; 2] = [STATUS_BASE, STATUS_HWID];
pub const CODE_VERSION: [u8; 2] = [STATUS_BASE, STATUS_VERSION];
pub const CODE_POSITION: [u8; 2] = [ENCODER_BASE, ENCODER_POSITION];
pub const CODE_INTENSET: [u8; 3] = [ENCODER_BASE, ENCODER_INTENSET, 0x01];
pub const CODE_INTENCLR: [u8; 3] = [ENCODER_BASE, ENCODER_INTENCLR, 0x01];

pub struct Encoder<I> {
    i2c: I,
    address: u8,
}

impl<I, E> Encoder<I>
where
    I: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>,
{
    pub fn new(i2c: I, address: u8) -> Self {
        Self { i2c, address }
    }

    pub fn init(&mut self, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        self.sw_reset(delay)?;
        let chip_id = self.chip_id(delay)?;
        if chip_id != 0x55 {
            return Err(Error::InvalidChipId(chip_id));
        }
        Ok(())
    }

    pub fn sw_reset(&mut self, delay: &mut dyn DelayMs<u16>) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &CODE_SWRST)
            .map_err(Error::I2C)?;
        delay.delay_ms(500);
        Ok(())
    }

    pub fn chip_id(&mut self, delay: &mut dyn DelayMs<u16>) -> Result<u8, Error<E>> {
        let mut buf = [0; 1];
        self.i2c
            .write(self.address, &CODE_CHIPID)
            .map_err(Error::I2C)?;
        delay.delay_ms(8);
        self.i2c.read(self.address, &mut buf).map_err(Error::I2C)?;
        Ok(buf[0])
    }

    pub fn get_version(&mut self, delay: &mut dyn DelayMs<u16>) -> Result<u32, Error<E>> {
        self.get_raw_version(delay).map(|v| (v >> 16) & 0xFFFF)
    }

    pub fn get_raw_version(&mut self, delay: &mut dyn DelayMs<u16>) -> Result<u32, Error<E>> {
        let mut buf = [0; 4];
        self.i2c
            .write(self.address, &CODE_VERSION)
            .map_err(Error::I2C)?;
        delay.delay_ms(8);
        self.i2c.read(self.address, &mut buf).map_err(Error::I2C)?;

        Ok(u32::from_be_bytes(buf))
    }

    pub fn get_position(&mut self, delay: &mut dyn DelayMs<u16>) -> Result<i32, Error<E>> {
        let mut buf = [0; 4];
        self.i2c
            .write(self.address, &CODE_POSITION)
            .map_err(Error::I2C)?;
        delay.delay_ms(8);
        self.i2c.read(self.address, &mut buf).map_err(Error::I2C)?;

        Ok(i32::from_be_bytes(buf))
    }

    pub fn enable_interrupt(&mut self) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &CODE_INTENSET)
            .map_err(Error::I2C)
    }

    pub fn disable_interrupt(&mut self) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &CODE_INTENCLR)
            .map_err(Error::I2C)
    }

    pub fn set_pixel(&mut self, color: (u8, u8, u8)) -> Result<(), Error<E>> {
        let tx = [0x0E, 0x01, 0x06];
        self.i2c.write(self.address, &tx).map_err(Error::I2C)?;
        let tx = [0x0E, 0x03, 0x00, 0x03];
        self.i2c.write(self.address, &tx).map_err(Error::I2C)?;
        let tx = [0x0E, 0x04, 0x00, 0x00, color.0, color.1, color.2];
        self.i2c.write(self.address, &tx).map_err(Error::I2C)?;
        let tx = [0x0E, 0x05];
        self.i2c.write(self.address, &tx).map_err(Error::I2C)?;

        Ok(())
    }
}
