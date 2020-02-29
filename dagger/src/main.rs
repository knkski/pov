#![no_std]
#![no_main]

extern crate panic_semihosting;
extern crate cortex_m_rt as rt;

mod board;

use nb::block;
use nrf52840_hal as hal;
use rt::entry;
use cortex_m::asm::nop;

use board::{
    hal::{
        prelude::*,
        timer::{self, Timer},
    },
    Board,
};

#[entry]
fn main() -> ! {
    let mut nrf52 = Board::take().unwrap();

    let mut timer = Timer::new(nrf52.TIMER0);

    // Alternately flash the red and blue leds
    loop {
        nrf52.leds.led_2.enable();
        delay(&mut timer, 250_000); // 250ms
        nrf52.leds.led_2.disable();
        delay(&mut timer, 1_000_000); // 1s
    }
}

fn delay<T>(timer: &mut Timer<T>, cycles: u32)
where
    T: timer::Instance,
{
    timer.start(cycles);
    let _ = block!(timer.wait());
}
