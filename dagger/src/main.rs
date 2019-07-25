#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_semihosting;
extern crate feather_m0 as hal;
#[cfg(not(feature = "use_semihosting"))]
extern crate panic_halt;
use smart_leds::RGB8;

use core::borrow::BorrowMut;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::spi_master;
use hal::{entry, CorePeripherals, Peripherals};
use tinybmp::{Bmp, Header, FileType};

#[entry]
fn main() -> ! {
    let bmp = Bmp::from_slice(include_bytes!("mario.bmp")).expect("Failed to parse image");
    let data = bmp.image_data();

    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    let mut pins = hal::Pins::new(peripherals.PORT);


    let mut spi = spi_master(
        &mut clocks,
        1.khz(),
        peripherals.SERCOM4,
        peripherals.PM.borrow_mut(),
        pins.sck,
        pins.mosi,
        pins.miso,
        &mut pins.port,
    );

    let mut column = 0;
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let bpp = bmp.bpp() / 8;

    loop {
        spi.write(&[0, 0, 0, 0]).unwrap();

        for i in 0..4 {
            spi.write(&[0xFF, 0x00, 0x00, 0x00]).unwrap();
        }

        for i in 0..bmp.height() {
            for _ in 0..4 {
                let offset = (i * bmp.width() * bpp + column * bpp) as usize;
                spi.write(&[0xFF]).unwrap();
                spi.write(&data[offset..offset+3]).unwrap();
            }
        }

        for i in 0..4 {
            spi.write(&[0xFF, 0x00, 0x00, 0x00]).unwrap();
        }


        for i in 0..4 {
            spi.write(&[0xFF, 0x00, 0x00, 0x00]).unwrap();
        }

        for i in (0..bmp.height()).rev() {
            for _ in 0..4 {
                let offset = (i * bmp.width() * bpp + column * bpp) as usize;
                spi.write(&[0xFF]).unwrap();
                spi.write(&data[offset..offset+3]).unwrap();
            }
        }

        for i in 0..4 {
            spi.write(&[0xFF, 0x00, 0x00, 0x00]).unwrap();
        }

        spi.write(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]).unwrap();

        column = (column + 1) % bmp.width();

        delay.delay_ms(16u16)
    }
}
