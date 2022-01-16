#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use panic_halt as _;
use arduino_hal::prelude::*;
use avr_device;

#[avr_device::interrupt(atmega328p)]
fn PCINT2() {
}


#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    
    // Enable interrupts globally
    unsafe { avr_device::interrupt::enable() };
    
    let mut led = pins.d13.into_output();
    
    // Inputs
    let mut encoder_a = pins.d2.into_floating_input();
    let mut encoder_b = pins.d3.into_floating_input();
    let mut light_push_button_input = pins.a0.into_floating_input();

    // Outputs
    let mut light_relay_output = pins.a1.into_output();
    let mut tlc5947_latch_output = pins.d10.into_output();

    // Initial values
    light_relay_output.set_low();
    tlc5947_latch_output.set_low();

    loop {
        continue;
    }
}
