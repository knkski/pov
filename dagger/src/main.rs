#![no_std]
#![no_main]
#![feature(clamp)]

extern crate feather_m0 as hal;
#[cfg(not(feature = "use_semihosting"))]
extern crate panic_halt;

use core::borrow::BorrowMut;
use core::cell::Cell;
use cortex_m::interrupt::{free, Mutex};
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::entry;
use hal::gpio::{Pa12, Pb10, Pb11, PfD};
use hal::pac::{interrupt, CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::sercom::{SPIMaster4, Sercom4Pad0, Sercom4Pad2, Sercom4Pad3};
use hal::spi_master;
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use tinybmp::Bmp;

/// Convenience definition
type SPI4 = SPIMaster4<Sercom4Pad0<Pa12<PfD>>, Sercom4Pad2<Pb10<PfD>>, Sercom4Pad3<Pb11<PfD>>>;

/// Flag that each function can check to see if it should stop
static STOP: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

#[interrupt]
fn SERCOM0() -> ! {
    free(|cs| STOP.borrow(cs).set(true));
}

// First 3 bits are for the protocol, only last 5 bits determine
// the 32 levels of brightness
const BRIGHTNESS: u8 = 0b11111111;

fn wheel(pos: u8) -> [u8; 3] {
    match pos {
        0..=84 => [255 - 3 * pos, 3 * pos, 0],
        85..=169 => [0, 255 - 3 * (pos - 85), 3 * (pos - 85)],
        170..=255 => [3 * (pos - 170), 0, 255 - 3 * (pos - 170)],
    }
}

fn light_show(spi: &mut SPI4, delay: &mut Delay) -> () {
    let seed: [u8; 32] = [0; 32];
    let mut rng = ChaCha8Rng::from_seed(seed);
    let mut i = 0;
    let mut on = true;
    let mut dot = 0;

    loop {
        spi.write(&[0, 0, 0, 0]).unwrap();

        let color = wheel(i);
        if on {
            for led in 0..144 {
                if (led % 7i32 - dot).abs() < 2 {
                    let opposite = ((i as u16 + 127) % 255) as u8;
                    let color = wheel(opposite);
                    spi.write(&[BRIGHTNESS, color[0], color[1], color[2]])
                        .unwrap();
                } else {
                    let crackle = rng.gen_range(-8, 8);
                    spi.write(&[
                        BRIGHTNESS,
                        (color[0] as i16 + crackle).clamp(0, 255) as u8,
                        (color[1] as i16 + crackle).clamp(0, 255) as u8,
                        (color[2] as i16 + crackle).clamp(0, 255) as u8,
                    ])
                    .unwrap();
                }
            }
        } else {
            for _ in 0..144 {
                spi.write(&[BRIGHTNESS, 0x00, 0x00, 0x00]).unwrap();
            }
        }

        spi.write(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]).unwrap();

        i = (i + 1) % 255;
        dot = (dot + 1) % 7;
        on = !on;

        if free(|cs| STOP.borrow(cs).get()) {
            return;
        }

        delay.delay_ms(16u16);
    }
}

fn image_show(spi: &mut SPI4, _delay: &mut Delay) -> () {
    let bmp = Bmp::from_slice(include_bytes!("ghosts.bmp")).expect("Failed to parse image");
    let data = bmp.image_data();

    let mut column = 0;
    let bpp = bmp.bpp() / 8;

    loop {
        spi.write(&[0, 0, 0, 0]).unwrap();

        for i in 0..bmp.height() {
            let offset = (i * bmp.width() * bpp + column * bpp) as usize;
            spi.write(&[0xFF]).unwrap();
            spi.write(&data[offset..offset + 3]).unwrap();
        }

        for i in (0..bmp.height()).rev() {
            let offset = (i * bmp.width() * bpp + column * bpp) as usize;
            spi.write(&[0xFF]).unwrap();
            spi.write(&data[offset..offset + 3]).unwrap();
        }

        spi.write(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]).unwrap();

        column = (column + 1) % bmp.width();

        if free(|cs| STOP.borrow(cs).get()) {
            return;
        }
    }
}

#[entry]
fn main() -> ! {
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
        1500.khz(),
        peripherals.SERCOM4,
        peripherals.PM.borrow_mut(),
        pins.sck,
        pins.mosi,
        pins.miso,
        &mut pins.port,
    );

    let mut delay = Delay::new(core.SYST, &mut clocks);

    let mut fn_idx = 0;

    loop {
        match fn_idx % 2 {
            0 => light_show(&mut spi, &mut delay),
            1 => image_show(&mut spi, &mut delay),
            _ => unreachable!(),
        }

        fn_idx += 1;
        fn_idx %= 2;
        free(|cs| STOP.borrow(cs).set(false));
    }
}
