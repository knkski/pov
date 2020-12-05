#![no_main]
#![no_std]

mod logger;
mod timer;

#[allow(unused_imports)]
use panic_itm;

use crate::timer::Timer;
use bno055::Bno055;
use cortex_m::peripheral::ITM;
use embedded_hal::blocking::spi::Write;
use hal::{clocks, gpio::Level, prelude::*, saadc};
use heapless::{consts::U64, Vec};
use nrf52840_hal as hal;
use nrf52840_pac as pac;
use rtfm::app;

// Will be used
fn wheel(pos: u8) -> [u8; 3] {
    match pos {
        0..=84 => [255 - 3 * pos, 3 * pos, 0],
        85..=169 => [0, 255 - 3 * (pos - 85), 3 * (pos - 85)],
        170..=255 => [3 * (pos - 170), 0, 255 - 3 * (pos - 170)],
    }
}

const NUM_DOTS: u32 = 144;
const MAX_PRESSURE: i16 = 12800;
const TIMER1_FREQ: u32 = 16_000;
const TIMER2_FREQ: u32 = 16_000;
const TIMER3_FREQ: u32 = 16_000;

fn mode_rgb_all(spi: &mut hal::Spim<pac::SPIM0>, counter: &mut u8, intensity: &mut u8) {
    Write::write(&mut *spi, &[0, 0, 0, 0]).unwrap();
    let color = wheel(*counter);
    for _ in 0..NUM_DOTS {
        // Write::write(&mut *spi, &[0xFF, color[0], color[1], color[2]]).unwrap();
        Write::write(&mut *spi, &[0xFF, 0xFF, 0xFF, 0xFF]).unwrap();
    }
    *counter = counter.wrapping_add(*intensity.max(&mut 1));
}

fn mode_rgb_per(spi: &mut hal::Spim<pac::SPIM0>, counter: &mut u8, intensity: &mut u8) {
    Write::write(&mut *spi, &[0, 0, 0, 0]).unwrap();
    for i in 0..NUM_DOTS {
        let color = wheel(i as u8 + *counter);
        Write::write(&mut *spi, &[0xFF, color[0], color[1], color[2]]).unwrap();
    }
    *counter = counter.wrapping_add(*intensity.max(&mut 1));
}

fn mode_switch(spi: &mut hal::Spim<pac::SPIM0>) {
    Write::write(&mut *spi, &[0, 0, 0, 0]).unwrap();
    for _ in 0..NUM_DOTS {
        Write::write(&mut *spi, &[0xFF, 0, 0, 0xFF]).unwrap();
    }
}

pub enum Mode {
    RGBAll,
    RGBPer,
}

impl Mode {
    pub fn next(&mut self) {
        *self = match self {
            Mode::RGBAll => Mode::RGBPer,
            Mode::RGBPer => Mode::RGBAll,
        }
    }
}

#[app(device = nrf52840_pac, peripherals = true)]
const APP: () = {
    struct Resources<'a> {
        timer1: pac::TIMER1,
        timer2: pac::TIMER2,
        timer3: pac::TIMER3,
        itm: ITM,
        log_consumer: bbqueue::Consumer<'static, logger::LogBufferSize>,
        mode: Mode,

        /// The number of cycles currently elapsed since last mode switch,
        /// where a cycle is a single tick of TIMER2
        mode_switch_cycles: Option<u8>,
        spi: hal::spim::Spim<pac::SPIM0>,
        bno055: Bno055<hal::twim::Twim<hal::pac::TWIM1>>,
        counter: u8,
        accel_samples: Vec<f32, U64>,
        analog: hal::Saadc,
        pressure: hal::gpio::p0::P0_04<hal::gpio::Input<hal::gpio::Floating>>,
        pressure_level: u8,
        capacitive: hal::gpio::p0::P0_05<hal::gpio::Input<hal::gpio::Floating>>,
        capacitive_level: u8,
        is_analog_pressed: bool,
        temp: i8,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut timer0 = cx.device.TIMER0;
        timer0.init();
        let log_consumer = logger::init(timer0);

        // Configure to use external clocks, and start them
        let _clocks = hal::clocks::Clocks::new(cx.device.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_external(clocks::LfOscConfiguration::NoExternalNoBypass)
            .start_lfclk();

        // Set up timers
        let mut timer1 = cx.device.TIMER1;
        timer1.init();
        timer1.fire_at(1, TIMER1_FREQ);

        let mut timer2 = cx.device.TIMER2;
        timer2.init();
        timer2.fire_at(1, TIMER2_FREQ);

        let mut timer3 = cx.device.TIMER3;
        timer3.init();
        timer3.fire_at(1, TIMER3_FREQ);

        // Set up SPI for dot star light strip
        let port0 = hal::gpio::p0::Parts::new(cx.device.P0);
        let spiclk = port0.p0_14.into_push_pull_output(Level::Low).degrade();
        let spimosi = port0.p0_13.into_push_pull_output(Level::Low).degrade();
        let pins = hal::spim::Pins {
            sck: spiclk,
            miso: None,
            mosi: Some(spimosi),
        };
        let spi = hal::spim::Spim::new(
            cx.device.SPIM0,
            pins,
            hal::spim::Frequency::M1,
            hal::spim::MODE_0,
            0,
        );

        // Set up bno055
        let scl = port0.p0_11.into_floating_input().degrade();
        let sda = port0.p0_12.into_floating_input().degrade();
        let pins = hal::twim::Pins { scl, sda };
        let i2c = hal::twim::Twim::new(cx.device.TWIM1, pins, hal::twim::Frequency::K100);
        let mut bno055 = bno055::Bno055::new(i2c).with_alternative_address();
        let mut delay = hal::delay::Delay::new(cx.core.SYST);
        bno055.init(&mut delay).unwrap();
        bno055
            .set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
            .unwrap();

        // We're all set up, hand off control back to RTFM
        init::LateResources {
            timer1,
            timer2,
            timer3,
            itm: cx.core.ITM,
            log_consumer,
            mode: Mode::RGBAll,
            mode_switch_cycles: None,
            spi,
            bno055,
            counter: 0,
            accel_samples: Vec::new(),
            analog: saadc::Saadc::new(cx.device.SAADC, saadc::SaadcConfig::default()),
            pressure: port0.p0_04,
            pressure_level: 0,
            capacitive: port0.p0_05,
            capacitive_level: 0,
            is_analog_pressed: false,
            temp: 0,
        }
    }

    #[task(binds = TIMER1, resources = [
        timer1,
        bno055,
        accel_samples,
        analog,
        pressure,
        pressure_level,
        capacitive,
        capacitive_level,
        is_analog_pressed,
        mode,
        mode_switch_cycles
    ])]
    fn timer1(cx: timer1::Context) {
        let timer = cx.resources.timer1;
        let analog = cx.resources.analog;
        let pressure = cx.resources.pressure;
        let cycles = cx.resources.mode_switch_cycles;

        timer.ack_compare_event(1);

        if let Ok(c) = analog.read(&mut *cx.resources.capacitive) {
            *cx.resources.capacitive_level = c.max(0).min(255) as u8;
        }

        if let Ok(p) = analog.read(&mut *pressure) {
            let clamped = p.max(0).min(MAX_PRESSURE) as u32;
            *cx.resources.pressure_level = (clamped * 31 / (MAX_PRESSURE as u32)) as u8;
        }

        match (
            cx.resources.capacitive_level,
            cx.resources.is_analog_pressed,
        ) {
            (0...100, ap) => {
                *ap = false;
            }
            (_, ap @ false) => {
                *ap = true;
                *cycles = Some(0);
                cx.resources.mode.next();
            }
            (_, ap @ true) => {
                *ap = true;
            }
        }

        let _ = timer.fire_at(1, TIMER1_FREQ);
    }

    #[task(binds = TIMER2, resources = [timer2, spi, counter, analog, pressure_level, mode, mode_switch_cycles])]
    fn timer2(cx: timer2::Context) {
        let timer = cx.resources.timer2;
        timer.ack_compare_event(1);

        let mode = cx.resources.mode;
        let cycles = cx.resources.mode_switch_cycles;

        match (cycles, mode) {
            (None, Mode::RGBAll) => mode_rgb_all(
                cx.resources.spi,
                cx.resources.counter,
                cx.resources.pressure_level,
            ),
            (None, Mode::RGBPer) => mode_rgb_per(
                cx.resources.spi,
                cx.resources.counter,
                cx.resources.pressure_level,
            ),
            (Some(c @ 0...32), _) => {
                mode_switch(cx.resources.spi);
                *c += 1;
            }
            (c, _) => {
                *c = None;
            }
        }

        let _ = timer.fire_at(1, TIMER2_FREQ);
    }

    #[task(binds = TIMER3, resources = [timer3, bno055, temp])]
    fn timer3(cx: timer3::Context) {
        let timer = cx.resources.timer3;

        timer.ack_compare_event(1);

        *cx.resources.temp = cx.resources.bno055.temperature().unwrap();

        let _ = timer.fire_at(1, TIMER3_FREQ);
    }

    #[idle(resources = [itm, log_consumer])]
    fn idle(cx: idle::Context) -> ! {
        let itm_port = &mut cx.resources.itm.stim[0];
        loop {
            while let Ok(grant) = cx.resources.log_consumer.read() {
                let length = grant.buf().len();
                for chunk in grant.buf().chunks(256) {
                    cortex_m::itm::write_all(itm_port, chunk);
                }
                grant.release(length);
            }
        }
    }

    extern "C" {
        fn PDM();
        fn QDEC();
    }
};
