#[derive(PartialEq, Copy, Clone)]
enum LightingState {
    Disabled,
    RelayOnWait(u8),
    ChasingSetpoint(u16, u16, u8),
    AtSetpoint(u16),
    LDDShuttingDown(u16, u8),
}

pub enum LightingAction {
    None,
    TurnRelayOn,
    TurnRelayOff,
    SetBrightness(u16),
}

use LightingAction::*;
use LightingState::*;

pub struct LightingFSM {
    state: LightingState,
}

const LDD_OFF_PWM_VALUE: u16 = 4095;

const CIE_CORRECTION_SCALING_FACTOR: u16 = u16::MAX / LDD_OFF_PWM_VALUE;

const MAX_PWM_VALUE: u16 = 4070;

const INITIAL_ON_SETPOINT: u16 = 2900;

const SOFT_SWITCHING_DELAY: u8 = 5;

const RAMP_RATE: u16 = 15;

impl LightingFSM {
    pub const fn new() -> Self {
        LightingFSM {
            state: LightingState::Disabled,
        }
    }
    pub fn toggle(&mut self) -> LightingAction {
        let (new_state, output) = match self.state {
            Disabled => (RelayOnWait(SOFT_SWITCHING_DELAY), TurnRelayOn),
            RelayOnWait(_) => (Disabled, TurnRelayOff),
            ChasingSetpoint(current_brightness, _, count_down) => {
                (LDDShuttingDown(current_brightness, count_down), None)
            }
            AtSetpoint(current_brightness) => (LDDShuttingDown(current_brightness, 10), None),
            LDDShuttingDown(current_brightness, count_down) => (
                ChasingSetpoint(current_brightness, INITIAL_ON_SETPOINT, count_down),
                None,
            ),
        };
        self.state = new_state;
        output
    }

    pub fn slow_clock(&mut self) {
        self.state = match self.state {
            RelayOnWait(0) => ChasingSetpoint(MAX_PWM_VALUE, INITIAL_ON_SETPOINT, 0),
            RelayOnWait(count) if count > 0 => RelayOnWait(count - 1),
            _ => self.state,
        };
    }

    pub fn fast_clock(&mut self) -> LightingAction {
        let (new_state, output) = match self.state {
            LDDShuttingDown(LDD_OFF_PWM_VALUE, _) => (Disabled, TurnRelayOff),
            LDDShuttingDown(current_brightness, _) if current_brightness >= MAX_PWM_VALUE => (
                LDDShuttingDown(LDD_OFF_PWM_VALUE, 10),
                SetBrightness(LDD_OFF_PWM_VALUE),
            ),
            LDDShuttingDown(current_brightness, count_down) if count_down > 0 => {
                (LDDShuttingDown(current_brightness, count_down - 1), None)
            }
            LDDShuttingDown(current_brightness, _count_down) => {
                let (step, delay) = cie_correction(current_brightness);

                let new_brightness = {
                    let temp = current_brightness + step;
                    if temp >= MAX_PWM_VALUE {
                        MAX_PWM_VALUE
                    } else {
                        temp
                    }
                };

                (
                    LDDShuttingDown(new_brightness, delay),
                    SetBrightness(new_brightness),
                )
            }
            ChasingSetpoint(current_brightness, setpoint, _) if current_brightness == setpoint => {
                (AtSetpoint(setpoint), None)
            }
            ChasingSetpoint(current_brightness, setpoint, 0) => {
                let (step, delay) = cie_correction(current_brightness);

                let new_brightness = if current_brightness < setpoint {
                    let temp = current_brightness + step;
                    if temp >= setpoint {
                        setpoint
                    } else {
                        temp
                    }
                } else {
                    if step >= current_brightness {
                        setpoint
                    } else {
                        current_brightness - step
                    }
                };
                (
                    ChasingSetpoint(new_brightness, setpoint, delay),
                    SetBrightness(new_brightness),
                )
            }
            ChasingSetpoint(current_brightness, setpoint, count_down) if count_down > 0 => (
                ChasingSetpoint(current_brightness, setpoint, count_down - 1),
                None,
            ),
            _ => (self.state, None),
        };
        self.state = new_state;
        output
    }
}

// Crude approximation to calculate the rate of change
// for a given brightness
#[inline]
fn cie_correction(brightness: u16) -> (u16, u8) {
    let value: u16 = ((brightness + CIE_CORRECTION_SCALING_FACTOR) * 7) >> 11;
    if value >= RAMP_RATE {
        (1, (value -  RAMP_RATE) as u8)
    } else {
        (RAMP_RATE - value, 0)
    }
}
