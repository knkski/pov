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
use hal::clocks::{Clocks, LfOscConfiguration};
use hal::delay::Delay;
use hal::gpio::{p0, p1, Input, Level, Pin, PullUp};
use hal::gpiote::{Gpiote, TaskOutPolarity};
use hal::{ppi, prelude::*, spim, twim};
use nrf52840_hal as hal;
use nrf52840_pac as pac;
use rtic::app;

fn wheel(pos: u8) -> [u8; 3] {
    match pos {
        0..=84 => [255 - 3 * pos, 3 * pos, 0],
        85..=169 => [0, 255 - 3 * (pos - 85), 3 * (pos - 85)],
        170..=255 => [3 * (pos - 170), 0, 255 - 3 * (pos - 170)],
    }
}

const NUM_DOTS: u32 = 144;
const TIMER1_FREQ: u32 = 16_000;
const TIMER2_FREQ: u32 = 16_000;
const TIMER3_FREQ: u32 = 16_000;

fn mode_rgb_all(spi: &mut hal::Spim<pac::SPIM0>, counter: &mut u8) {
    Write::write(&mut *spi, &[0, 0, 0, 0]).unwrap();
    let color = wheel(*counter);
    for _ in 0..NUM_DOTS {
        Write::write(&mut *spi, &[0xFF, color[0], color[1], color[2]]).unwrap();
    }
    *counter = counter.wrapping_add(1);
}

fn mode_rgb_per(spi: &mut hal::Spim<pac::SPIM0>, counter: &mut u8) {
    Write::write(&mut *spi, &[0, 0, 0, 0]).unwrap();
    for i in 0..NUM_DOTS {
        let color = wheel(i as u8 + *counter);
        Write::write(&mut *spi, &[0xFF, color[0], color[1], color[2]]).unwrap();
    }
    *counter = counter.wrapping_add(1);
}

fn mode_orientation(spi: &mut hal::Spim<pac::SPIM0>, orientation: &mut [u8; 3]) {
    Write::write(&mut *spi, &[0, 0, 0, 0]).unwrap();
    for _ in 0..NUM_DOTS {
        Write::write(
            &mut *spi,
            &[0xFF, orientation[0], orientation[1], orientation[2]],
        )
        .unwrap();
    }
}

fn mode_acceleration(spi: &mut hal::Spim<pac::SPIM0>, accel_samples: &[f32], accel_head: &usize) {
    Write::write(&mut *spi, &[0, 0, 0, 0]).unwrap();
    for _ in 0..NUM_DOTS {
        Write::write(&mut *spi, &[0xFF, 0xFF, 0xFF, 0xFF]).unwrap();
    }
}

fn mode_switch(spi: &mut hal::Spim<pac::SPIM0>) {
    Write::write(&mut *spi, &[0, 0, 0, 0]).unwrap();
    for _ in 0..NUM_DOTS {
        Write::write(&mut *spi, &[0xFF, 0, 0, 0xFF]).unwrap();
    }
}

pub enum Mode {
    Orientation,
    Acceleration,
    RGBAll,
    RGBPer,
}

impl Mode {
    pub fn next(&mut self) {
        *self = match self {
            Mode::Orientation => Mode::Acceleration,
            Mode::Acceleration => Mode::RGBAll,
            Mode::RGBAll => Mode::RGBPer,
            Mode::RGBPer => Mode::Orientation,
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

        gpiote: Gpiote,
        btn1: Pin<Input<PullUp>>,

        /// The number of cycles currently elapsed since last mode switch,
        /// where a cycle is a single tick of TIMER2
        mode_switch_cycles: Option<u8>,
        spi: spim::Spim<pac::SPIM0>,
        bno055: Bno055<twim::Twim<pac::TWIM1>>,
        counter: u8,
        accel_samples: [f32; 64],
        accel_head: usize,
        temp: i8,
        orientation: [u8; 3],
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut timer0 = cx.device.TIMER0;
        timer0.init();
        let log_consumer = logger::init(timer0);

        // Configure to use external clocks, and start them
        Clocks::new(cx.device.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_external(LfOscConfiguration::NoExternalNoBypass)
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
        let port0 = p0::Parts::new(cx.device.P0);
        let spiclk = port0.p0_14.into_push_pull_output(Level::Low).degrade();
        let spimosi = port0.p0_13.into_push_pull_output(Level::Low).degrade();
        let pins = spim::Pins {
            sck: spiclk,
            miso: None,
            mosi: Some(spimosi),
        };
        let spi = spim::Spim::new(cx.device.SPIM0, pins, spim::Frequency::M1, spim::MODE_0, 0);

        // Set up on-board button to toggle on-board LED
        let port1 = p1::Parts::new(cx.device.P1);
        let btn1 = port1.p1_02.into_pullup_input().degrade();
        let led1 = port1.p1_15.into_push_pull_output(Level::High).degrade();
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        gpiote
            .channel0()
            .input_pin(&btn1)
            .hi_to_lo()
            .enable_interrupt();
        gpiote
            .channel1()
            .input_pin(&btn1)
            .lo_to_hi()
            .enable_interrupt();
        gpiote
            .channel2()
            .output_pin(led1)
            .task_out_polarity(TaskOutPolarity::Toggle)
            .init_low();
        let ppi_channels = ppi::Parts::new(cx.device.PPI);
        let mut ppi0 = ppi_channels.ppi0;
        ppi0.set_task_endpoint(gpiote.channel2().task_out());
        ppi0.set_event_endpoint(gpiote.channel0().event());
        ppi0.enable();
        let mut ppi1 = ppi_channels.ppi1;
        ppi1.set_task_endpoint(gpiote.channel2().task_out());
        ppi1.set_event_endpoint(gpiote.channel1().event());
        ppi1.enable();

        // Set up bno055
        let scl = port0.p0_11.into_floating_input().degrade();
        let sda = port0.p0_12.into_floating_input().degrade();
        let pins = twim::Pins { scl, sda };
        let i2c = twim::Twim::new(cx.device.TWIM1, pins, twim::Frequency::K100);
        let mut bno055 = bno055::Bno055::new(i2c).with_alternative_address();
        let mut delay = Delay::new(cx.core.SYST);
        bno055.init(&mut delay).unwrap();
        bno055
            .set_mode(bno055::BNO055OperationMode::ACC_GYRO, &mut delay)
            .unwrap();

        // We're all set up, hand off control back to RTIC
        init::LateResources {
            timer1,
            timer2,
            timer3,
            itm: cx.core.ITM,
            log_consumer,
            mode: Mode::Acceleration,
            gpiote,
            btn1,
            mode_switch_cycles: None,
            spi,
            bno055,
            counter: 0,
            accel_samples: [0.; 64],
            accel_head: 9,
            temp: 0,
            orientation: [0, 0, 0],
        }
    }

    #[task(binds = TIMER1, resources = [
        timer1,
        bno055,
        accel_samples,
        accel_head,
        mode,
        mode_switch_cycles,
        temp,
        orientation,
    ])]
    fn timer1(cx: timer1::Context) {
        let timer = cx.resources.timer1;
        let bno055 = cx.resources.bno055;
        let accel_samples = cx.resources.accel_samples;
        let accel_head = cx.resources.accel_head;

        timer.ack_compare_event(1);

        *cx.resources.temp = bno055.temperature().unwrap();

        match bno055.accel_data() {
            Ok(a) => {
                accel_samples[*accel_head] = a.x.max(a.y);
                *accel_head += 1;
                *accel_head %= accel_samples.len();
                if *accel_head == 0 {
                    log::info!("{}, {:?}", accel_head, accel_samples);
                }
            }
            Err(err) => {
                log::error!("{:?}", err);
            }
        }

        if let Ok(q) = bno055.quaternion() {
            *cx.resources.orientation = [
                (255. * q.v.x) as u8,
                (255. * q.v.y) as u8,
                (255. * q.v.z) as u8,
            ];
        }

        let _ = timer.fire_at(1, TIMER1_FREQ);
    }

    #[task(binds = TIMER2, resources = [
        timer2,
        spi,
        counter,
        orientation,
        mode,
        mode_switch_cycles,
        accel_samples,
        accel_head
    ])]
    fn timer2(cx: timer2::Context) {
        let timer = cx.resources.timer2;
        timer.ack_compare_event(1);

        let mode = cx.resources.mode;
        let cycles = cx.resources.mode_switch_cycles;

        match (cycles, mode) {
            (None, Mode::RGBAll) => mode_rgb_all(cx.resources.spi, cx.resources.counter),
            (None, Mode::RGBPer) => mode_rgb_per(cx.resources.spi, cx.resources.counter),
            (None, Mode::Orientation) => {
                mode_orientation(cx.resources.spi, cx.resources.orientation)
            }
            (None, Mode::Acceleration) => mode_acceleration(
                cx.resources.spi,
                cx.resources.accel_samples,
                cx.resources.accel_head,
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

    #[task(binds = TIMER3, resources = [timer3])]
    fn timer3(cx: timer3::Context) {
        let timer = cx.resources.timer3;

        timer.ack_compare_event(1);

        let _ = timer.fire_at(1, TIMER3_FREQ);
    }

    #[task(binds = GPIOTE, resources = [gpiote])]
    fn on_gpiote(ctx: on_gpiote::Context) {
        if ctx.resources.gpiote.channel0().is_event_triggered() {
            log::info!("Interrupt from channel 0 event");
            ctx.resources.gpiote.channel0().reset_events();
        }
        if ctx.resources.gpiote.channel1().is_event_triggered() {
            log::info!("Interrupt from channel 1 event");
            ctx.resources.gpiote.channel1().reset_events();
        }
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
