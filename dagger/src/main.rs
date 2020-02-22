#![no_std]
#![no_main]
#![feature(clamp)]

extern crate feather_m4 as hal;
extern crate panic_semihosting;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::entry;
use hal::gpio::{Pa17, Pb22, Pb23, PfC};
use hal::pac::{CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::sercom::{SPIMaster1, Sercom1Pad1, Sercom1Pad2, Sercom1Pad3};
use hal::time::KiloHertz;
use hal::{i2c_master, spi_master};
use tinybmp::Bmp;

type SPI1 = SPIMaster1<Sercom1Pad2<Pb22<PfC>>, Sercom1Pad3<Pb23<PfC>>, Sercom1Pad1<Pa17<PfC>>>;
type I2C2 = hal::sercom::I2CMaster2<
    hal::sercom::Sercom2Pad0<hal::gpio::Pa12<hal::gpio::PfC>>,
    hal::sercom::Sercom2Pad1<hal::gpio::Pa13<hal::gpio::PfC>>,
>;

const BRIGHTNESS: u8 = 0b11100001;

fn wheel(pos: u8) -> [u8; 3] {
    match pos {
        0..=84 => [255 - 3 * pos, 3 * pos, 0],
        85..=169 => [0, 255 - 3 * (pos - 85), 3 * (pos - 85)],
        170..=255 => [3 * (pos - 170), 0, 255 - 3 * (pos - 170)],
    }
}

fn light_cycle(spi: &mut SPI1, delay: &mut Delay) -> ! {
    let mut i = 0;

    loop {
        spi.write(&[0, 0, 0, 0]).unwrap();
        let color = wheel(i);
        for _ in 0..144 {
            spi.write(&[BRIGHTNESS, color[0], color[1], color[2]])
                .unwrap();
        }
        i = (i + 1) % 255;

        delay.delay_ms(16u16);
    }
}

fn image_show(spi: &mut SPI1, _delay: &mut Delay) -> ! {
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
    }
}

fn axial(spi: &mut SPI1, delay: &mut Delay, imu: &mut bno055::Bno055<I2C2>) -> ! {
    loop {
        let quat: mint::Quaternion<f32> = imu.quaternion().unwrap();
        // let val = (quat.s as u32).clamp(0, 255) as u8;
				let val = 255u8;
        spi.write(&[0, 0, 0, 0]).unwrap();
        for _ in 0..144 {
            spi.write(&[BRIGHTNESS, val, val, val]).unwrap();
        }

        delay.delay_ms(16u16);
    }
}

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

    let mut pins = hal::Pins::new(peripherals.PORT);
    let mut delay = Delay::new(core.SYST, &mut clocks);
    let mut spi = spi_master(
        &mut clocks,
        1500.khz(),
        peripherals.SERCOM1,
        &mut peripherals.MCLK,
        pins.sck,
        pins.mosi,
        pins.miso,
        &mut pins.port,
    );

    let i2c = i2c_master(
        &mut clocks,
        KiloHertz(400),
        peripherals.SERCOM2,
        &mut peripherals.MCLK,
        pins.sda,
        pins.scl,
        &mut pins.port,
    );
    let mut imu = bno055::Bno055::new(i2c);
    imu.init(&mut delay).unwrap();
    imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
        .unwrap();
    while !imu.is_fully_calibrated().unwrap() {}
		let calib = imu.calibration_profile(&mut delay).unwrap();
		// mcu.nvram_write(BNO055_CALIB_ADDR, calib.as_bytes(), BNO055_CALIB_SIZE).unwrap();


    axial(&mut spi, &mut delay, &mut imu);
    // light_cycle(&mut spi, &mut delay);
    // image_show(&mut spi, &mut delay);
    // loop {
    // red_led.set_high().unwrap();
    // delay.delay_ms(1000u16);
    // red_led.set_low().unwrap();
    // delay.delay_ms(1000u16);
    // }
}
