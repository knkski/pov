//! Controls an apa102 LED strip

#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use embedded_hal::spi::MODE_3;
use f3::hal::{delay::Delay, prelude::*, spi::Spi, stm32f30x, i2c::I2c};
use f3::Lsm303dlhc;
use cortex_m_semihosting::hprintln;

const NUM_LEDS: i32 = 142;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f30x::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let freq = 1.mhz();
    let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze(&mut flash.acr);

    let mut _delay = Delay::new(cp.SYST, clocks);

    // Set up SPI
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let mut spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        MODE_3,
        1.khz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);
    let mut lsm303dlhc = Lsm303dlhc::new(i2c).unwrap();

    let mut position = 0;
    let mut old_accel = (0i16, 0i16, 0i16);

    loop {
        let accel = lsm303dlhc.accel().unwrap();
        let delta = ((accel.x - old_accel.0).abs(), (accel.y - old_accel.1).abs(), (accel.z - old_accel.2).abs());
        old_accel = (accel.x, accel.y, accel.z);
        let max_accel = delta.0.abs().max(delta.1.abs()).max(delta.2.abs()) / 2048;
        let first_byte = 0xE1 + (max_accel as u8);

        // Write 32 0's to signal start of frame
        spi.write(&[0, 0, 0, 0]).unwrap();

        // For each LED, write 3 1's and 5 brightness bits, followed by
        // a byte each of BGR.
        for i in (0..NUM_LEDS).rev() {
            match ((position + i) / 6) % 3 {
                0 => spi.write(&[first_byte, 0x00, 0x00, 0xFF]).unwrap(),
                1 => spi.write(&[first_byte, 0x00, 0xFF, 0x00]).unwrap(),
                _ => spi.write(&[first_byte, 0xFF, 0x00, 0x00]).unwrap(),
            }
        }

        // Follow up with at least N/2 1's.
        spi.write(&[0xFF, 0xFF, 0xFF, 0xFF]).unwrap();
        spi.write(&[0xFF, 0xFF, 0xFF, 0xFF]).unwrap();
        spi.write(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
            .unwrap();
        position = (position + 1) % NUM_LEDS;
    }
}
