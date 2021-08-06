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
use hal::gpio::{p0, p1, Floating, Input, Level, Output, Pin, PullDown, PushPull};
use hal::gpiote::Gpiote;
use hal::{prelude::*, pwm, spim, twim};
use nrf52840_hal as hal;
use nrf52840_pac as pac;
use num_traits::float::Float;
use rtic::app;

fn wheel(pos: u8) -> [u8; 3] {
    match pos {
        0..=84 => [255 - 3 * pos, 3 * pos, 0],
        85..=169 => [0, 255 - 3 * (pos - 85), 3 * (pos - 85)],
        170..=255 => [3 * (pos - 170), 0, 255 - 3 * (pos - 170)],
    }
}

const NUM_DOTS: u32 = 144;
const TIMER1_FREQ: u32 = 500_000;
const TIMER2_FREQ: u32 = 16_000;

fn mode_rgb_all(t: u8, i: u32) -> [u8; 4] {
    let color = wheel(t);
    [0xFF, color[0], color[1], color[2]]
}

fn mode_rgb_per(t: u8, i: u32) -> [u8; 4] {
    let color = wheel(i as u8 + t);
    [0xFF, color[0], color[1], color[2]]
}

fn mode_orientation(t: u8, i: u32) -> [u8; 4] {
    [0xFF, 0xFF, 0xFF, 0xFF]
}

fn mode_acceleration(t: u8, i: u32) -> [u8; 4] {
    [0xFF, 0xFF, 0xFF, 0xFF]
}

fn mode_math(t: u8, i: u32) -> [u8; 4] {
    [
        0xE0 + ((((128. * t as f32 / 255.).sin() / 2. + 0.5) * 30.) + 1.) as u8,
        (((16. * (i as f32 + t as f32) / (NUM_DOTS as f32)).sin() / 2. + 0.5) * 255.) as u8,
        0,
        (((16. * (i as f32 + t as f32) / (NUM_DOTS as f32)).cos() / 2. + 0.5) * 255.) as u8,
    ]
}

fn mode_switch(t: u8, i: u32) -> [u8; 4] {
    [0xFF, 0, 0, 0xFF]
}

pub enum Mode {
    Orientation,
    Acceleration,
    RGBAll,
    RGBPer,
    Math,
}

impl Mode {
    pub fn next(&mut self) {
        *self = match self {
            Mode::Math => Mode::Orientation,
            Mode::Orientation => Mode::Acceleration,
            Mode::Acceleration => Mode::RGBAll,
            Mode::RGBAll => Mode::RGBPer,
            Mode::RGBPer => Mode::Math,
        }
    }
}

#[app(device = nrf52840_pac, peripherals = true)]
const APP: () = {
    struct Resources<'a> {
        timer1: pac::TIMER1,
        timer2: pac::TIMER2,
        itm: ITM,
        log_consumer: bbqueue::Consumer<'static, logger::LogBufferSize>,
        mode: Mode,

        gpiote: Gpiote,
        btn1: Pin<Input<PullDown>>,
        btn2: Pin<Input<Floating>>,
        led1: Pin<Output<PushPull>>,
        /// The number of cycles currently elapsed since last mode switch,
        /// where a cycle is a single tick of TIMER2
        mode_switch_cycles: Option<u8>,
        spi: spim::Spim<pac::SPIM0>,
        bno055: Option<Bno055<twim::Twim<pac::TWIM1>>>,
        counter: u8,
        accel_samples: [f32; 64],
        accel_head: usize,
        temp: i8,
        orientation: [f32; 3],
        pin13: Pin<Input<PullDown>>,
        num_dots: u32,
        pwm: pwm::PwmSeq<pac::PWM0, &'static mut [u16; 96], &'static mut [u16; 96]>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut BUF0: [u16; 96] = [0; 96];

        for (i, v) in BUF0.iter_mut().enumerate() {
            match (i % 4, i / 4) {
                (0, 0...8) => *v = 0x7FFF / 3,
                (0, _) => *v = 0x0000,
                (1...2, _) => *v = 0,
                (3, _) => *v = 0x7FFF,
                _ => unreachable!(),
            }
        }

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

        let neopixel = port0.p0_26.into_push_pull_output(Level::High).degrade();
        let mut pwm = pwm::Pwm::new(cx.device.PWM0);
        pwm.set_output_pin(pwm::Channel::C0, &neopixel);
        pwm.set_period(800u32.hz());
        // pwm.set_load_mode(pwm::LoadMode::Waveform);
        // pwm.set_step_mode(pwm::StepMode::Auto);
        // pwm.set_seq_end_delay(pwm::Seq::Seq0, 30);
        pwm.loop_inf();
        // pwm.set_seq_refresh(pwm::Seq::Seq0, 0);
        pwm.set_duty(pwm::Channel::C0, pwm.max_duty() / 3);
        pwm.enable();

        // Set up on-board button to toggle on-board LED
        let port1 = p1::Parts::new(cx.device.P1);
        let btn1 = port0.p0_06.into_pulldown_input().degrade();
        let pin12 = port0.p0_08.into_pulldown_input().degrade();
        let pin13 = port1.p1_09.into_pulldown_input().degrade();
        let btn2 = port1.p1_08.into_floating_input().degrade();
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
        // gpiote
        //     .channel2()
        //     .output_pin(led1)
        //     .task_out_polarity(TaskOutPolarity::Toggle)
        //     .init_low();
        gpiote
            .channel3()
            .input_pin(&pin12)
            .lo_to_hi()
            .enable_interrupt();
        gpiote
            .channel4()
            .input_pin(&btn2)
            .toggle()
            .enable_interrupt();

        // Allows setting up hardware-controlled interrupt handlers
        // let ppi_channels = ppi::Parts::new(cx.device.PPI);
        // let mut ppi0 = ppi_channels.ppi0;
        // ppi0.set_task_endpoint(gpiote.channel4().task_out());
        // ppi0.set_event_endpoint(gpiote.channel2().event());
        // ppi0.enable();

        // Set up bno055
        let bno055 = {
            let scl = port0.p0_11.into_floating_input().degrade();
            let sda = port0.p0_12.into_floating_input().degrade();
            let pins = twim::Pins { scl, sda };
            let i2c = twim::Twim::new(cx.device.TWIM1, pins, twim::Frequency::K100);
            let mut delay = Delay::new(cx.core.SYST);
            let mut bno055 = bno055::Bno055::new(i2c).with_alternative_address();
            if let Ok(()) = bno055.init(&mut delay) {
                Some(bno055)
            } else {
                None
            }
            // bno055
            //     .set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
            //     .unwrap();
        };

        // We're all set up, hand off control back to RTIC
        init::LateResources {
            timer1,
            timer2,
            itm: cx.core.ITM,
            log_consumer,
            mode: Mode::Math,
            gpiote,
            btn1,
            btn2,
            led1,
            mode_switch_cycles: None,
            spi,
            bno055,
            counter: 0,
            accel_samples: [0.; 64],
            accel_head: 9,
            temp: 0,
            orientation: [0., 0., 0.],
            pin13,
            num_dots: 144,
            pwm: pwm.load(Some(BUF0), None, true).ok().unwrap(),
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
        // btn2,
        // led1,
        // neopixel,
        // delay,
    ])]
    fn timer1(cx: timer1::Context) {
        let timer = cx.resources.timer1;
        let bno055 = cx.resources.bno055;

        timer.ack_compare_event(1);

        if let Some(t) = bno055.as_mut().and_then(|b| b.temperature().ok()) {
            *cx.resources.temp = t;
        }

        // Not compatible with quaternion mode
        if let Some(a) = bno055.as_mut().and_then(|b| b.accel_data().ok()) {
            let accel_samples = cx.resources.accel_samples;
            let accel_head = cx.resources.accel_head;
            accel_samples[*accel_head] = a.x.max(a.y);
            *accel_head += 1;
            *accel_head %= accel_samples.len();
        }

        if let Some(q) = bno055.as_mut().and_then(|b| b.quaternion().ok()) {
            let o = glam::Quat::from(q).mul_vec3(glam::Vec3::from_slice(&[1., 0., 0.]));
            *cx.resources.orientation = [o.x, o.y, o.z];
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
        accel_head,
        num_dots
    ])]
    fn timer2(cx: timer2::Context) {
        let timer = cx.resources.timer2;
        timer.ack_compare_event(1);

        let t = cx.resources.counter;
        let mode = cx.resources.mode;
        let cycles = cx.resources.mode_switch_cycles;

        Write::write(cx.resources.spi, &[0, 0, 0, 0]).unwrap();
        for i in 0..NUM_DOTS {
            let pixel = match mode {
                Mode::Math => mode_math(*t, i),
                Mode::Acceleration => mode_acceleration(*t, i),
                Mode::RGBAll => mode_rgb_all(*t, i),
                Mode::RGBPer => mode_rgb_per(*t, i),
                Mode::Orientation => mode_orientation(*t, i),
            };
            Write::write(cx.resources.spi, &pixel).unwrap();
        }

        *t = t.wrapping_add(1);
        let _ = timer.fire_at(1, TIMER2_FREQ);
    }

    #[task(binds = GPIOTE, resources = [gpiote, pin13, spi, num_dots, led1, btn2])]
    fn on_gpiote(ctx: on_gpiote::Context) {
        let gpiote = ctx.resources.gpiote;
        if gpiote.channel4().is_event_triggered() {
            gpiote.channel4().reset_events();
            if ctx.resources.btn2.is_high().unwrap() {
                ctx.resources.led1.set_high().unwrap();
            } else {
                ctx.resources.led1.set_low().unwrap();
            }
        }
        if gpiote.channel3().is_event_triggered() {
            gpiote.channel3().reset_events();
            if ctx.resources.pin13.is_high().unwrap() {
                *ctx.resources.num_dots = ctx.resources.num_dots.saturating_add(1);
            } else {
                *ctx.resources.num_dots = ctx.resources.num_dots.saturating_sub(1);
            }
        }
        if gpiote.channel1().is_event_triggered() {
            gpiote.channel1().reset_events();
            *ctx.resources.num_dots = 0;
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
