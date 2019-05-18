//! Blinks an LED

#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

use core::f32::consts::E;

use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;
//use cortex_m_semihosting::hprintln;
use f3::{
    hal::{delay::Delay, prelude::*, stm32f30x, time::Hertz},
    led::Led,
};
use libm::F32Ext;

/// Calculates log curve basic on cycle count
fn get_delay(t: f32) -> f32 {
    let bottom = 20_000.;
    let top = 100_000. - bottom;

    bottom + top / (1. + E.powf(t - 5.))
}

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f30x::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let freq = 32.mhz();
    let freqh: Hertz = freq.into();
    let clocks = rcc.cfgr.sysclk(freq).freeze(&mut flash.acr);

    let mut leds: [Led; 8] = [
        gpioe.pe8.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
        gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
        gpioe.pe10.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
        gpioe.pe11.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
        gpioe.pe12.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
        gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
        gpioe.pe14.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
        gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper).into(),
    ];
    let mut delay = Delay::new(cp.SYST, clocks);
    let start = DWT::get_cycle_count();

    loop {
        leds[0].on();
        leds[0].off();
//        let elapsed = (DWT::get_cycle_count() - start) as f32 / (freqh.0 as f32);
//        let f = get_delay(elapsed) as u32;
//
//        leds[0].on();
//        delay.delay_us(f);
//        leds[0].off();
//        delay.delay_us(f);
    }
}
