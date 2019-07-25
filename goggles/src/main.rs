//! Controls an apa102 LED strip

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use f3::hal::{delay::Delay, prelude::*, spi::Spi, stm32f30x, gpio::gpioa::{PA5, PA6, PA7}, gpio::AF5};
use smart_leds::RGB8;
use ws2812_spi as ws2812;
use nb::block;

fn wheel(pos: u8, brightness: u8) -> RGB8 {
    match pos {
        0..=84 => RGB8 {
            r: (255 - 3 * pos) / brightness,
            g: 3 * pos / brightness,
            b: 0,
        },
        85..=170 => RGB8 {
            r: 0,
            g: (255 - 3 * (pos - 85)) / brightness,
            b: 3 * (pos - 85) / brightness,
        },
        170..=255 => RGB8 {
            r: 3 * (pos - 170) / brightness,
            g: 0,
            b: (255 - 3 * (pos - 170)) / brightness,
        }
    }
}

// Write a single byte for ws2812 devices
fn write_byte(spi: &mut Spi<stm32f30x::SPI1, (PA5<AF5>, PA6<AF5>, PA7<AF5>)>, mut data: u8) {
    let mut serial_bits: u32 = 0;
    for _ in 0..3 {
        let bit = data & 0x80;
        let pattern = if bit == 0x80 { 0b110 } else { 0b100 };
        serial_bits = pattern | (serial_bits << 3);
        data <<= 1;
    }
    block!(spi.send((serial_bits >> 1) as u8)).unwrap();
    // Split this up to have a bit more lenient timing
    for _ in 3..8 {
        let bit = data & 0x80;
        let pattern = if bit == 0x80 { 0b110 } else { 0b100 };
        serial_bits = pattern | (serial_bits << 3);
        data <<= 1;
    }
    // Some implementations (stm32f0xx-hal) want a matching read
    // We don't want to block so we just hope it's ok this way
    spi.read().ok();
    block!(spi.send((serial_bits >> 8) as u8)).unwrap();
    spi.read().ok();
    block!(spi.send(serial_bits as u8)).unwrap();
    spi.read().ok();
}


#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f30x::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiof = dp.GPIOF.split(&mut rcc.ahb);

    gpiof
        .pf9
        .into_open_drain_output(&mut gpiof.moder, &mut gpiof.otyper);

    let clocks = rcc
        .cfgr
        .sysclk(72.mhz())
        .pclk1(3.mhz())
        .freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks);

    // Set up SPI

    // Set up SPI
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let mut spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        ws2812::MODE,
        3.mhz(),
        clocks,
        &mut rcc.apb2,
    );



    let mut data: [RGB8; 32] = [RGB8::default(); 32];
    let mut i = 0u8;

    loop {
        for j in 0..32 {
            let brightness = 0xFFu8 >> 16 - ((j + i) % 16);
            data[j as usize] = wheel(i, brightness);
        }

        for item in data.iter() {
            write_byte(&mut spi, item.g);
            write_byte(&mut spi, item.r);
            write_byte(&mut spi, item.b);
            write_byte(&mut spi, 0);
        }
        for _ in 0..20 {
            block!(spi.send(0)).unwrap();
            spi.read().unwrap();
        }

        delay.delay_ms(50u16);

        i += 1;
        i %= 255;
    }
}
