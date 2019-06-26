//! Controls an apa102 LED strip

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use embedded_hal::spi::MODE_3;
use f3::hal::{delay::Delay, prelude::*, spi::Spi, stm32f30x};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f30x::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let freq = 32.mhz();
    let clocks = rcc.cfgr.sysclk(freq).pclk1(freq).freeze(&mut flash.acr);

    let mut _delay = Delay::new(cp.SYST, clocks);

    // Set up SPI
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let mut spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        MODE_3,
        32.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut position = 0;
    loop {
        // Write 32 0's to signal start of frame
        spi.write(&[0, 0, 0, 0]).unwrap();

        // Go back and forth
        let pos = (144i32 - ((position + 144) % 288)).abs();

        // For each LED, write 3 1's and 5 brightness bits, followed by
        // a byte each of BGR.
        for i in 0..144 {
            if (i - 1..i + 1).contains(&pos) {
                spi.write(&[0xE1, 0x00, 0x00, 0xFF]).unwrap();
            } else {
                spi.write(&[0xE1, 0x00, 0x00, 0x00]).unwrap();
            }
        }

        // Follow up with at least N/2 1's.
        spi.write(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
            .unwrap();
        position = (position + 1) % 288;
    }
}
