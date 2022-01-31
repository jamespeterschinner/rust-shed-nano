#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(llvm_asm)]

mod lighting;
mod fan;

use crate::lighting::{LightingAction, LightingFSM};
use crate::fan::{FanAction, FanFSM};
use ruduino::cores::atmega328p::port::*;
use ruduino::cores::atmega328p::*;
use ruduino::interrupt::without_interrupts;
use ruduino::modules::ClockSource16::*;
use ruduino::modules::Timer16Setup;
use ruduino::modules::WaveformGenerationMode16::*;
use ruduino::Pin;
use ruduino::Register;
// use ruduino::RegisterBits;

const CPU_FREQUENCY_HZ: u64 = 16_000_000;
const DESIRED_HZ_TIM1: f64 = 10.0; // Period of 100 milli seconds
const TIM1_PRESCALER: u64 = 1024;
const INTERRUPT_EVERY_1_HZ_1024_PRESCALER: u16 =
    ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM1 * TIM1_PRESCALER as f64)) as u64 - 1) as u16;

// The nameing convention used here is <name><Input|Output><Port & Pin>
// the port and pin is included as is helps know which bits to use for
// associated interrupt masks and the like

// Edge triggered interrupts
type EncoderInputD2 = D2;
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
type FanHighSpeedRelayOutputD6 = D6;

// SPI Interface
type TLC5947LatchOutputB2 = B2; // output
type MOSIB3 = B3; // output
type SCKB5 = B5; // output

// Input debounce
const INPUT_BUFFER_SIZE: usize = 3;
static mut INPUT_BUFFER_INDEX: usize = 0;

// Add one to the INPUT_BUFFER_SIZE here to remove off by one errors in the
// Timer interrupt routine
static mut INPUT_BUFFER: [u8; INPUT_BUFFER_SIZE + 1] = [0; INPUT_BUFFER_SIZE + 1];

static mut LIGHTING_FSM: LightingFSM = LightingFSM::new(5); // 500ms switching delay

// 7 second VSD power up delay, 9000 == 15min power off delay
static mut FAN_FSM: FanFSM = FanFSM::new(70, 9000);

#[no_mangle]
fn main() {
    without_interrupts(|| {
        EncoderInputD2::set_input();
        EncoderInputD3::set_input();

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

        FanHighSpeedRelayOutputD6::set_output();
        FanHighSpeedRelayOutputD6::set_low();


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
        
        Timer16Setup::<Timer16>::new()
            .waveform_generation_mode(ClearOnTimerMatchOutputCompare)
            .clock_source(Prescale1024)
            .output_compare_1(Some(INTERRUPT_EVERY_1_HZ_1024_PRESCALER))
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

// Timer routine
#[no_mangle]
pub unsafe extern "avr-interrupt" fn __vector_11() {
    // 16bit (timer1) compare match interrupt
    without_interrupts(|| {
        INPUT_BUFFER_INDEX += 1;
        if INPUT_BUFFER_INDEX > INPUT_BUFFER_SIZE {
            INPUT_BUFFER_INDEX = 0;
        }

        // Enable the interrupts which were disabled previously
        PCMSK1::write(PCMSK1::read() | INPUT_BUFFER[INPUT_BUFFER_INDEX]);

        // Clear the prior captured inputs
        INPUT_BUFFER[INPUT_BUFFER_INDEX] = 0;

        perform_lighting_action(LIGHTING_FSM.clock());

        perform_fan_action(FAN_FSM.clock());
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
        let byte_one = (value & 0xFF) as u8;
        let byte_two = (value << 4 & 0xF0 | value >> 8 & 0x0F) as u8;
        let byte_three = (value >> 4 & 0xFF) as u8;
        // We really want this to go fast
        unsafe {
            for _ in 0..12 {
                SPDR::write(byte_one);
                llvm_asm!("nop"); // See: https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/SPI/src/SPI.h#L216
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
    match action {
        LightingAction::None => {}
        LightingAction::PowerUpLDD => {
            LightRelayOutputC1::set_high();
        }
        LightingAction::PowerDownLDD => {
            LightRelayOutputC1::set_low();
        }
        LightingAction::EnableLDDControl => {
            spi_write_value(0x000);
            latch_spi_value() // Keeping this seperate because we'll be doing this asynchronously (eventually)
        }
        LightingAction::DisableLDDControl => {
            spi_write_value(0xFFF);
            latch_spi_value()
        }
    }
  
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
            FanHighSpeedRelayOutputD6::set_high();
        }
        FanAction::RunLowSpeed => {
            FanStartRelayOutputD4::set_high();
            FanHighSpeedRelayOutputD6::set_low();
        }
        FanAction::DisableVSD => {
            FanStartRelayOutputD4::set_low();
            FanHighSpeedRelayOutputD6::set_low();
        }
    }
}

fn latch_spi_value() {
    TLC5947LatchOutputB2::set_high();
    TLC5947LatchOutputB2::set_low();
}
