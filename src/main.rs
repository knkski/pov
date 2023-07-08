#![no_main]
#![no_std]

mod encoder;

use core::panic::PanicInfo;
use num_traits::float::Float;
use rtic::app;
use rtt_target::rprintln;

const NUM_DOTS: u8 = 72;

fn rand(i: f32, t: f32) -> f32 {
    ((((i * 12.9898).sin() * 43758.5453).fract() + 0.07) * t) / 255.
}

fn wheel(pos: u8) -> [u8; 3] {
    match pos {
        0..=84 => [255 - 3 * pos, 3 * pos, 0],
        85..=169 => [0, 255 - 3 * (pos - 85), 3 * (pos - 85)],
        170..=255 => [3 * (pos - 170), 0, 255 - 3 * (pos - 170)],
    }
}

fn mode_liquid(t: u8, i: u8) -> [u8; 4] {
    let t = t as f32;

    let noise = |x: f32| -> f32 { x.sin() };

    let fbm = |mut x: f32| -> f32 {
        let mut f = 0.5000 * noise(x);
        x *= 2.02;
        f += 0.2500 * noise(x);
        x *= 2.02;
        f += 0.1250 * noise(x);
        x *= 2.02;
        f += 0.0625 * noise(x);
        f / 0.9375
    };

    let pattern = |x: f32| -> f32 {
        let q = fbm(x + 5.2 * (t + 100.) * 0.01);
        let r = fbm(x + 4. * q + 1.7);
        fbm(x + 4. * r)
    };

    let p = (i as f32) / (NUM_DOTS as f32);
    let color = 0.5 - pattern(p);
    let color = (255. * color) as u8;
    let x = (((color as f32) / 255.) * 4.) as u8;
    [0xE7, color, 0x4 - x, 0]
}

fn mode_rgb_per(t: u8, i: u8) -> [u8; 4] {
    let color = wheel(i.wrapping_add(t));
    [0xE7, color[0], color[1], color[2]]
}

fn mode_rgb_per_slow(t: u8) -> [u8; 4] {
    let color = wheel(t);
    [0xE7, color[0], color[1], color[2]]
}

fn mode_math(t: u8, i: u8) -> [u8; 4] {
    [
        0xE0 + ((((16. * t as f32 / 255.).sin() / 2. + 0.5) * 15.) + 1.) as u8,
        (((16. * (i as f32 + t as f32) / (NUM_DOTS as f32)).sin() / 2. + 0.5) * 255.) as u8,
        0xF,
        (((16. * (i as f32 + t as f32) / (NUM_DOTS as f32)).cos() / 2. + 0.5) * 255.) as u8,
    ]
}

fn mode_crackle(t: u8, i: u8) -> [u8; 4] {
    // let crackle = rng.gen_range(-64, 64);
    let crackle = ((rand(i as f32, t as f32) - 0.5) * 128.) as i16;
    let c = (0xAF as i16 + crackle).clamp(0, 255) as u8;
    [0xE7, c, 0, 0]
}

#[derive(Clone, Copy)]
pub enum Mode {
    Crackle,
    RGBPerSlow,
    RGBPer,
    Math,
    Liquid,
}

impl Mode {
    pub fn next(&mut self) {
        *self = match self {
            Mode::Crackle => Mode::RGBPerSlow,
            Mode::RGBPerSlow => Mode::Math,
            Mode::Math => Mode::RGBPer,
            Mode::RGBPer => Mode::Liquid,
            Mode::Liquid => Mode::Crackle,
        }
    }

    pub fn prev(&mut self) {
        *self = match self {
            Mode::Crackle => Mode::Liquid,
            Mode::RGBPerSlow => Mode::Crackle,
            Mode::Math => Mode::RGBPerSlow,
            Mode::RGBPer => Mode::Math,
            Mode::Liquid => Mode::RGBPer,
        }
    }
}

#[app(device = pac, peripherals = true, dispatchers = [PDM, QDEC])]
mod app {
    use super::{
        mode_crackle, mode_liquid, mode_math, mode_rgb_per, mode_rgb_per_slow, Mode, NUM_DOTS,
    };
    use crate::encoder::Encoder;
    use embedded_hal::blocking::spi::Write;
    use hal::clocks::{Clocks, LfOscConfiguration};
    use hal::delay::Delay;
    use hal::gpio::{p0, p1, Level};
    use hal::gpiote::Gpiote;
    use hal::spim;
    use hal::twim::{Frequency, Pins, Twim};
    use nrf52840_hal as hal;
    use nrf52840_pac as pac;
    use rtt_target::{rprintln, rtt_init_print};

    #[shared]
    struct Shared {
        mode: Mode,
    }

    #[local]
    struct Local {
        timer1: pac::TIMER1,

        // Inputs
        interval: u32,
        gpiote: Gpiote,
        encoder: Encoder<Twim<pac::TWIM0>>,
        delay: hal::Delay,
        spim: spim::Spim<pac::SPIM1>,
        t: u8,
        pos: u8,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Configure to use external clocks, and start them
        Clocks::new(ctx.device.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_external(LfOscConfiguration::NoExternalNoBypass)
            .start_lfclk();

        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        rtt_init_print!();
        rprintln!("RTT initialized");

        let interval = 16_000;

        let timer1 = ctx.device.TIMER1;
        timer1.mode.write(|w| w.mode().timer());
        timer1.bitmode.write(|w| w.bitmode()._32bit());
        timer1.prescaler.write(|w| unsafe { w.prescaler().bits(4) });
        timer1.tasks_clear.write(|w| w.tasks_clear().set_bit());
        timer1.tasks_start.write(|w| w.tasks_start().set_bit());

        timer1.tasks_capture[0].write(|w| w.tasks_capture().set_bit());
        let now = timer1.cc[0].read().bits();

        let later = now.wrapping_add(interval);
        timer1.cc[1].write(|w| unsafe { w.bits(later) });
        timer1.events_compare[1].reset();
        timer1.intenset.write(|w| w.compare1().set());

        // Set up GPIO ports
        let port1 = p1::Parts::new(ctx.device.P1);

        let encoder_btn = port1.p1_08.into_floating_input().degrade();
        let gpiote = Gpiote::new(ctx.device.GPIOTE);

        gpiote
            .channel0()
            .input_pin(&encoder_btn)
            .hi_to_lo()
            .enable_interrupt();

        let mut delay = Delay::new(ctx.core.SYST);
        let p0 = p0::Parts::new(ctx.device.P0);
        let scl = p0.p0_11.into_floating_input().degrade();
        let sda = p0.p0_12.into_floating_input().degrade();

        let twim = Twim::new(ctx.device.TWIM0, Pins { scl, sda }, Frequency::K100);
        // let mut seesaw = Encoder::new(twim);
        // let mut board = seesaw.device(0x36);
        // let mut encoder = board.encoder(0);
        let mut encoder = Encoder::new(twim, 0x36);
        encoder.init(&mut delay).unwrap();
        encoder.enable_interrupt().unwrap();
        let version = encoder.get_version(&mut delay).unwrap();
        rprintln!("Found product {}", version);
        assert_eq!(version, 4991, "Wrong firmware loaded? Expected 4991");

        // Set up SPI for dot star light strip
        let spiclk = p0.p0_14.into_push_pull_output(Level::Low).degrade();
        let spimosi = p0.p0_13.into_push_pull_output(Level::Low).degrade();
        let pins = spim::Pins {
            sck: spiclk,
            miso: None,
            mosi: Some(spimosi),
        };
        let spim = spim::Spim::new(ctx.device.SPIM1, pins, spim::Frequency::M1, spim::MODE_0, 0);

        // We're all set up, hand off control back to RTIC
        let shared = Shared {
            mode: Mode::Crackle,
        };

        let local = Local {
            timer1,
            interval,
            gpiote,
            encoder,
            delay,
            spim,
            t: 0,
            pos: 0,
        };

        (shared, local, init::Monotonics())
    }

    #[task(
        binds = TIMER1,
        shared = [mode],
        local = [timer1, interval, spim, t]
    )]
    fn timer1(mut ctx: timer1::Context) {
        let timer = ctx.local.timer1;
        let interval = ctx.local.interval;
        let t = ctx.local.t;
        let mode = ctx.shared.mode.lock(|m| *m);

        timer.events_compare[1].reset();

        Write::write(ctx.local.spim, &[0, 0, 0, 0]).unwrap();
        for i in 0..NUM_DOTS {
            let pixel = match mode {
                Mode::Math => mode_math(*t, i),
                Mode::RGBPer => mode_rgb_per(*t, i),
                Mode::RGBPerSlow => mode_rgb_per_slow(*t),
                Mode::Crackle => mode_crackle(*t, i),
                Mode::Liquid => mode_liquid(*t, i),
            };
            Write::write(ctx.local.spim, &pixel).unwrap();
        }
        for i in (0..NUM_DOTS).rev() {
            let pixel = match mode {
                Mode::Math => mode_math(*t, i),
                Mode::RGBPer => mode_rgb_per(*t, i),
                Mode::RGBPerSlow => mode_rgb_per_slow(*t),
                Mode::Crackle => mode_crackle(*t, i),
                Mode::Liquid => mode_liquid(*t, i),
            };
            Write::write(ctx.local.spim, &pixel).unwrap();
        }

        *t = t.wrapping_add(1);

        timer.tasks_capture[0].write(|w| w.tasks_capture().set_bit());
        let now = timer.cc[0].read().bits();
        let later = now.wrapping_add(*interval);
        timer.cc[1].write(|w| unsafe { w.bits(later) });
        timer.events_compare[1].reset();
        timer.intenset.write(|w| w.compare1().set());
    }

    #[task(
        binds = GPIOTE,
        shared = [mode],
        local = [gpiote, encoder, delay, pos]
    )]
    fn on_gpiote(mut ctx: on_gpiote::Context) {
        let gpiote = ctx.local.gpiote;
        let encoder = ctx.local.encoder;
        let delay = ctx.local.delay;
        let current_pos = *ctx.local.pos;

        if gpiote.channel0().is_event_triggered() {
            gpiote.channel0().reset_events();
            let pos = encoder.get_position(delay).unwrap().max(0) as u8;

            ctx.shared.mode.lock(|m| {
                if pos > current_pos {
                    m.next()
                } else {
                    m.prev()
                }
            });
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    rprintln!("{}", info);
    loop {}
}
