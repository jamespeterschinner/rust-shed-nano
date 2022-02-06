#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(llvm_asm)]

mod fan;
mod lighting;
mod cie_correction;

use crate::fan::{FanAction, FanFSM};
use crate::lighting::{LightingAction, LightingFSM};
use ruduino::cores::atmega328p::port::*;
use ruduino::cores::atmega328p::*;
use ruduino::interrupt::without_interrupts;
use ruduino::modules::ClockSource16;
use ruduino::modules::ClockSource8;
use ruduino::modules::Timer16Setup;
use ruduino::modules::Timer8Setup;
use ruduino::modules::WaveformGenerationMode16;
use ruduino::modules::WaveformGenerationMode8;

use ruduino::Pin;
use ruduino::Register;
// use ruduino::RegisterBits;

const CPU_FREQUENCY_HZ: u64 = 16_000_000;

// The nameing convention used here is <name><Input|Output><Port & Pin>
// the port and pin is included as is helps know which bits to use for
// associated interrupt masks and the like

// Edge triggered interrupts
type TLC5947PWMFeedBackD2 = D2;

// Controls the timing of external interrupt 0
// which latches in new PWM values, at the end of the
// PWM window based upon this value 0 - 4096
// initial testing 4010 seemed good
static PWM_FEEDBACK_TIMING: u16 = 4010;

/*
Remember is that last value in the shift register, hence is written first.
Each byte is written MSB first.
*/
static PWM_FEEDBACK_BYTE_ONE: u8 = (PWM_FEEDBACK_TIMING >> 4) as u8 & 0xFF as u8;
static PWM_FEEDBACK_BYTE_TWO: u8 = (PWM_FEEDBACK_TIMING << 4) as u8 & 0xF0 as u8;

type EncoderInputD3 = D3;

//PCINT8..14 -> PCI2 (Pin Change Interrupt 1)
type LightToggleInputC0 = C0; //PCINT8
type FanStartInputC3 = C3; //PCINT11
type FanStopInputC4 = C4; //PCINT12

static ENABLED_PINC_INTERRUPTS: u8 = 0b00011001;

// Fan Stop is a normally closed contact
static INVERTED_INPUTS: u8 = 0b00010000;

// Outputs

type LightRelayOutputC1 = C1;
type FanPowerRelayOutputC2 = C2;
type FanStartRelayOutputD4 = D4;
type FanHighSpeedRelayOutputD5 = D5;

// SPI Interface
type TLC5947LatchOutputB2 = B2; // output
type MOSIB3 = B3; // output
type SCKB5 = B5; // output

// Input debounce
const INPUT_BUFFER_SIZE: usize = 12;
static mut INPUT_BUFFER_INDEX: usize = 0;

// Add one to the INPUT_BUFFER_SIZE here to remove off by one errors in the
// Timer interrupt routine
static mut INPUT_BUFFER: [u8; INPUT_BUFFER_SIZE + 1] = [0; INPUT_BUFFER_SIZE + 1];

static mut LIGHTING_FSM: LightingFSM = LightingFSM::new();

// The last 3 bits of the TCCR1B register control the clock source
// We'll allow this to be mutated so it can be set from the timers configuration
// in main (save it for later so we can enable and disable the timer)
static TCCR1B_CLOCK_ENABLE: u8 = 0b101;

// 7 second VSD power up delay, 9000 == 15min power off delay
static mut FAN_FSM: FanFSM = FanFSM::new(70, 9000);

#[no_mangle]
fn main() {
    without_interrupts(|| {
        // External interrupt 0 (INT0)
        TLC5947PWMFeedBackD2::set_input();
        // Enables the pullup resistor
        TLC5947PWMFeedBackD2::set_high();

        EncoderInputD3::set_input();

        // Last two bits set configure INT0 to
        // trigger on the rising edge
        EICRA::write(0b0011);

        //PCINT8..14 -> PCI2 (Pin Change Interrupt 1)
        LightToggleInputC0::set_input(); //PCINT8
        FanStartInputC3::set_input(); //PCINT11
        FanStopInputC4::set_input(); //PCINT12

        // Pin Change Mask Register 1 (enable for inputs)
        PCMSK1::write(ENABLED_PINC_INTERRUPTS);

        //Pin Change Interrupt Control Register (enable for inputs)
        PCICR::set(PCICR::PCIE1);

        // Disables the analogue to digital function on port c
        ADCSRA::write(0);

        // ACSR::unset(ACSR::ACD);
        // ACSR::unset(ACSR::ACIE);
        // DIDR1::write(0b11);
        DIDR0::write(0b11); // This must be se inorder to use port C as output pins!

        // Outputs
        TLC5947LatchOutputB2::set_output();
        TLC5947LatchOutputB2::set_low();

        LightRelayOutputC1::set_output();
        LightRelayOutputC1::set_low();

        FanPowerRelayOutputC2::set_output();
        FanPowerRelayOutputC2::set_low();

        FanStartRelayOutputD4::set_output();
        FanStartRelayOutputD4::set_low();

        FanHighSpeedRelayOutputD5::set_output();
        FanHighSpeedRelayOutputD5::set_low();

        // SPI interface
        MOSIB3::set_output();
        SCKB5::set_output();
        SCKB5::set_output();

        /*
        SPI settings bits 7 -> 0
        7 - interrupt enable
        6 - spi enable
        5 - bit transmission order (0 == MSB bit first)
        4 - master/slave (master == 1)
        3 - clock polarity (mode 0)
        2 - clock phase (mode 0)
        1 - SCK frequency (0)
        0 - SCK frequency (0) (fastest)
        */

        // Note: the onboard LED (B5) is the same pin as the SCK
        // meaning that driving it high while SPI is enabled will
        // have strange effects (maybe break it?)
        SPCR::write(0b01010011);

        // // sets the x2 SPI SCK frequency
        // SPSR::write(0x1);

        // Turns all PWN outputs off (0xFFF is off)
        spi_write_value(0xFFF);
        latch_spi_value();

        // Slow running low resolution timer (physical IO and sequential tasks)
        Timer8Setup::<Timer8>::new()
            .waveform_generation_mode(WaveformGenerationMode8::ClearOnTimerMatchOutputCompare)
            .clock_source(ClockSource8::Prescale1024)
            .output_compare_1(Some(0xFF))
            .configure();

        // Faster high resolution timer for accurate PWM timing updates
        // Initially configure it with no clock source as to not start the timer
        Timer16Setup::<Timer16>::new()
            .waveform_generation_mode(WaveformGenerationMode16::ClearOnTimerMatchOutputCompare)
            .clock_source(ClockSource16::Prescale64)
            .output_compare_1(Some(0xFFF))
            .configure();
    });

    loop {
        // This is some double redundancy in terms of duplicating logic
        // but we really wan't the fan to stop if the normally
        // closed stop signal goes open circuit and potenitally there
        // is an unknown bug in the other place where this is implemented.
        // (we don't really want to debounce this input)
        if FanStopInputC4::is_low() {
            // Best practice would be to have a safety circuit which implemented this
            // but the fan is in the roof and very inaccessible without tools
            // in which case the circuit breaker immediatly next to the fan is sufficient
            FanStartRelayOutputD4::set_low();
        }
    }
}

/*
TLC5947PWMFeedBackD2 (INT0)

This interrupt should only be enabled when there are new
values in the TLC5947's SPI input buffer.

This interrupt routine is triggered on the rising edge, which
occurs when the feedback PWM signal turns off, which stops
pulling the input low.
*/
#[no_mangle]
pub unsafe extern "avr-interrupt" fn __vector_1() {
    without_interrupts(|| {
        if TLC5947PWMFeedBackD2::is_high() {
            latch_spi_value();
            // Disables the interrupt
            EIMSK::write(EIMSK::read() & !0x01)
        }
    })
}

// Input routine
#[no_mangle]
pub unsafe extern "avr-interrupt" fn __vector_4() {
    without_interrupts(|| {
        let inputs = PINC::read() ^ INVERTED_INPUTS;
        let buffered_values = INPUT_BUFFER[INPUT_BUFFER_INDEX];
        // Logic to ensure that we are triggering on the rising edge
        if inputs & !buffered_values > 0 {
            // Disables this interrupt routine for high inputs
            PCMSK1::write(PCMSK1::read() & !inputs);

            // Store the current inputs in the input buffer to be
            // re enabled after the debounce interval
            INPUT_BUFFER[INPUT_BUFFER_INDEX] = buffered_values | inputs;

            // Do something with the input

            if light_toggle_input(inputs) {
                perform_lighting_action(LIGHTING_FSM.toggle());
            }

            if fan_start_input(inputs) {
                perform_fan_action(FAN_FSM.start());
            }

            if fan_stop_input(inputs) {
                perform_fan_action(FAN_FSM.stop());
            }
        }
    })
}

// 16 Bit timer routine for LED update timing
#[no_mangle]
pub unsafe extern "avr-interrupt" fn __vector_11() {
    without_interrupts(|| {
        perform_lighting_action(LIGHTING_FSM.clock());
    })
}

// 8 Bit Timer routine used for slow tasks (debouncing user input)
#[no_mangle]
pub unsafe extern "avr-interrupt" fn __vector_14() {
    without_interrupts(|| {
        INPUT_BUFFER_INDEX += 1;
        if INPUT_BUFFER_INDEX > INPUT_BUFFER_SIZE {
            INPUT_BUFFER_INDEX = 0;
        }

        // Enable the interrupts which were disabled previously
        PCMSK1::write(PCMSK1::read() | INPUT_BUFFER[INPUT_BUFFER_INDEX]);

        // Clear the prior captured inputs
        INPUT_BUFFER[INPUT_BUFFER_INDEX] = 0;

        perform_fan_action(FAN_FSM.clock());
    })
}

fn light_toggle_input(inputs: u8) -> bool {
    inputs & 1 > 0
}

fn fan_start_input(inputs: u8) -> bool {
    // Start button is pressed
    (inputs & 0x8 > 0) &&
    // Stop button is not pressed 
    (inputs & 0x10 == 0)
}

fn fan_stop_input(inputs: u8) -> bool {
    inputs & 0x10 > 0
}

// value 0 == fully on, value 0xFFF = fully off
fn spi_write_value(value: u16) {
    without_interrupts(|| {
        // Precompute values, compacting 12 bits per u16 into
        // three u8's
        let byte_one = (value >> 4 & 0xFF) as u8;
        let byte_two = (value << 4 & 0xF0 | value >> 8 & 0x0F) as u8;
        let byte_three = (value & 0xFF) as u8;
        // We really want this to go fast
        unsafe {
            // Unroll one of the iteratations to allow writing
            // the PWM feedback value first (shifted into the 24th channel)
            SPDR::write(PWM_FEEDBACK_BYTE_ONE);
            llvm_asm!("nop");
            while SPSR::is_clear(SPSR::SPIF) {}
            SPDR::read();

            SPDR::write(PWM_FEEDBACK_BYTE_TWO | (byte_two & 0x0F));
            llvm_asm!("nop");
            while SPSR::is_clear(SPSR::SPIF) {}
            SPDR::read();

            SPDR::write(byte_three);
            llvm_asm!("nop");
            while SPSR::is_clear(SPSR::SPIF) {}
            SPDR::read();

            for _ in 0..11 {
                SPDR::write(byte_one);
                // See: https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/SPI/src/SPI.h#L216
                llvm_asm!("nop");
                while SPSR::is_clear(SPSR::SPIF) {}
                SPDR::read();

                SPDR::write(byte_two);
                llvm_asm!("nop");
                while SPSR::is_clear(SPSR::SPIF) {}
                SPDR::read();

                SPDR::write(byte_three);
                llvm_asm!("nop");
                while SPSR::is_clear(SPSR::SPIF) {}
                SPDR::read();
            }
        }
    })
}

fn perform_lighting_action(action: LightingAction) {
    use LightingAction::*;
    match action {
        None => {}
        TurnRelayOn { switch_latency } => {
            LightRelayOutputC1::set_high();
            trigger_timer_1_in(switch_latency);
        }
        TurnRelayOff => {
            LightRelayOutputC1::set_low();
        }
        SetBrightness {
            brightness,
            duration,
        } => {
            spi_write_value(brightness);
            trigger_timer_1_in(duration);
            enable_latch_interrupt();
        }
        SetClock(interval) => {
            trigger_timer_1_in(interval);
        }
    }
}

#[inline]
fn trigger_timer_1_in(time_units: u16) {
    TCCR1B::write(0x0); // Disable timer clock
    TCNT1::write(0x0 as u16); // reset the timer
    OCR1A::write(time_units); // update compare match interval
    TCCR1B::write(TCCR1B_CLOCK_ENABLE); // Enable timer
}

#[inline]
fn enable_latch_interrupt() {
    EIMSK::write(EIMSK::read() | 0x01);
}

fn perform_fan_action(action: FanAction) {
    match action {
        FanAction::None => {}
        FanAction::PowerUpVSD => {
            FanPowerRelayOutputC2::set_high();
        }
        FanAction::PowerDownVSD => {
            FanPowerRelayOutputC2::set_low();
        }
        FanAction::RunHighSpeed => {
            FanStartRelayOutputD4::set_high();
            FanHighSpeedRelayOutputD5::set_high();
        }
        FanAction::RunLowSpeed => {
            FanStartRelayOutputD4::set_high();
            FanHighSpeedRelayOutputD5::set_low();
        }
        FanAction::DisableVSD => {
            FanStartRelayOutputD4::set_low();
            FanHighSpeedRelayOutputD5::set_low();
        }
    }
}
#[inline]
fn latch_spi_value() {
    TLC5947LatchOutputB2::set_high();
    TLC5947LatchOutputB2::set_low();
}
