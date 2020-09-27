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
use hal::{
    clocks,
    gpio::{Floating, Input, Level, Pin},
    prelude::*,
    saadc,
};
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


const NUM_DOTS: u32 = 72;
const MAX_PRESSURE: i16 = 12800;

#[app(device = nrf52840_pac, peripherals = true)]
const APP: () = {
    struct Resources {
        timer1: pac::TIMER1,
        timer2: pac::TIMER2,
        timer3: pac::TIMER3,
        itm: ITM,
        log_consumer: bbqueue::Consumer<'static, logger::LogBufferSize>,
        led: hal::gpio::p1::P1_15<hal::gpio::Output<hal::gpio::PushPull>>,
        led_on: bool,
        spi: hal::spim::Spim<pac::SPIM0>,
        bno055: Bno055<hal::twim::Twim<hal::pac::TWIM1>>,
        counter: u8,
        accel_samples: Vec<f32, U64>,
        analog: hal::Saadc,
        analog_pin: hal::gpio::p0::P0_04<hal::gpio::Input<hal::gpio::Floating>>,
        intensity: u8,
    }

    #[init]
    fn init(mut cx: init::Context) -> init::LateResources {
        let mut timer0 = cx.device.TIMER0;
        timer0.init();
        let log_consumer = logger::init(timer0);
        log::info!("STARTING UP!");

        // Configure to use external clocks, and start them
        let _clocks = hal::clocks::Clocks::new(cx.device.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_external(clocks::LfOscConfiguration::NoExternalNoBypass)
            .start_lfclk();

        // Set up timers
        let mut timer1 = cx.device.TIMER1;
        timer1.init();
        timer1.fire_at(1, 16_000);

        let mut timer2 = cx.device.TIMER2;
        timer2.init();
        timer2.fire_at(1, 100_000);

        let mut timer3 = cx.device.TIMER3;
        timer3.init();
        timer3.fire_at(1, 100_000);

        // Set up LED pin for flashing on and off
        let pins = hal::gpio::p1::Parts::new(cx.device.P1);
        let mut led = pins.p1_15.into_push_pull_output(hal::gpio::Level::Low);
        led.set_high().unwrap();

        // Set up SPI for dot star light strip
        let mut port0 = hal::gpio::p0::Parts::new(cx.device.P0);
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
        let mut analog = saadc::Saadc::new(cx.device.SAADC, saadc::SaadcConfig::default());
        let mut analog_pin = port0.p0_04;
        log::info!("PIN: {}", analog.read(&mut analog_pin).unwrap());

        let itm = &mut cx.core.ITM.stim[0];

        // Set up bno055
        let scl = port0.p0_11.into_floating_input().degrade();
        let sda = port0.p0_12.into_floating_input().degrade();
        let pins = hal::twim::Pins { scl, sda };
        let i2c = hal::twim::Twim::new(cx.device.TWIM1, pins, hal::twim::Frequency::K100);
        let mut bno055 = bno055::Bno055::new(i2c).with_alternative_address();
        let mut delay = hal::delay::Delay::new(cx.core.SYST);
        bno055.init(&mut delay).unwrap_or_else(|err| {
            cortex_m::iprintln!(itm, "Couldn't initialize BNO055: {:?}", err);
            panic!("{:?}", err);
        });
        bno055
            .set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
            .unwrap_or_else(|err| {
                cortex_m::iprintln!(itm, "Couldn't set bno055 mode: {:?}", err);
                panic!("{:?}", err);
            });

        // We're all set up, hand off control back to RTFM
        init::LateResources {
            timer1,
            timer2,
            timer3,
            itm: cx.core.ITM,
            log_consumer,
            led,
            led_on: true,
            spi,
            bno055,
            counter: 0,
            accel_samples: Vec::new(),
            analog,
            analog_pin,
            intensity: 255,
        }
    }

    #[task(binds = TIMER1, resources = [timer1, bno055, accel_samples, analog, analog_pin, intensity])]
    fn timer1(cx: timer1::Context) {
        let timer = cx.resources.timer1;
        let analog = cx.resources.analog;
        let analog_pin = cx.resources.analog_pin;
        let pressure = analog.read(&mut *analog_pin);

        timer.ack_compare_event(1);
        if let Ok(p) = pressure {
            let clamped = p.max(0).min(MAX_PRESSURE) as u32;
            *cx.resources.intensity = (clamped * 31 / (MAX_PRESSURE as u32)) as u8;
            log::info!("INTENSITY: {}", *cx.resources.intensity);
        }

        /* match cx.resources.bno055.quaternion() {
         *     Ok(q) => {
         *         *cx.resources.color = (
         *             (q.v.x * 127. + 128.) as u8,
         *             (q.v.y * 127. + 128.) as u8,
         *             (q.v.z * 127. + 128.) as u8,
         *             0x7F,
         *         );
         *     }
         *     Err(err) => {
         *         log::error!("{:?}", err);
         *     }
         * }; */

        match cx.resources.bno055.linear_acceleration() {
            Ok(a) => {
                if a.x > 5. {
                    log::info!("a.x: {}", a.x);
                }
            }
            Err(err) => log::error!("ACCEL ERROR: {:?}", err),
        }

        // if cx.resources.accel_samples.len() >= 64 {
        // log::info!("SAMPLES: {:?}", cx.resources.accel_samples);
        // cx.resources.accel_samples.clear();
        // }

        // let temp = cx.resources.bno055.temperature().unwrap();
        let _ = timer.fire_at(1, 16_000);
    }

    #[task(binds = TIMER2, resources = [timer2, spi, counter, analog, intensity])]
    fn timer2(cx: timer2::Context) {
        let timer = cx.resources.timer2;

        timer.ack_compare_event(1);
        Write::write(&mut *cx.resources.spi, &[0, 0, 0, 0]).unwrap();
        let color = wheel(*cx.resources.counter);
        for _ in 0..NUM_DOTS {
            Write::write(
                &mut *cx.resources.spi,
                &[
                    0xE0 + *cx.resources.intensity,
                    color[0],
                    color[1],
                    color[2],
                ],
            )
            .unwrap();
        }
        *cx.resources.counter += 1;
        *cx.resources.counter %= 255;

        let _ = timer.fire_at(1, 100_000);
    }

    #[task(binds = TIMER3, resources = [timer3, led, led_on])]
    fn timer3(cx: timer3::Context) {
        let timer = cx.resources.timer3;

        timer.ack_compare_event(1);

        if *cx.resources.led_on {
            *cx.resources.led_on = false;
            cx.resources.led.set_low().unwrap();
        } else {
            *cx.resources.led_on = true;
            cx.resources.led.set_high().unwrap();
        }

        let _ = timer.fire_at(1, 100_000);
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
