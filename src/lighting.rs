#[derive(PartialEq, Copy, Clone)]
enum LightingState {
    Disabled,
    RelayOnWait(u8),
    ChasingSetpoint(u16, u16, u8),
    LDDShuttingDown(u16, u8),
}

pub enum LightingAction {
    None,
    TurnRelayOn,
    TurnRelayOff,
    SetBrightness(u16),
}

use crate::encoder::EncoderChange;
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

const MIN_CHANGE_RATE: u16 = 15;

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

    pub fn fast_clock(&mut self, setpoint_change: EncoderChange) -> LightingAction {
        let (new_state, output) = match self.state {
            LDDShuttingDown(current_brightness, _) if current_brightness >= MAX_PWM_VALUE => {
                (Disabled, TurnRelayOff)
            }
            LDDShuttingDown(current_brightness, count_down) if count_down > 0 => {
                (LDDShuttingDown(current_brightness, count_down - 1), None)
            }
            LDDShuttingDown(current_brightness, _count_down) => {
                let (step, delay) = cie_correction(current_brightness, 0);

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
            ChasingSetpoint(current_brightness, setpoint, 0) => {
                let (updated_setpoint, rate) = match setpoint_change {
                    EncoderChange::None => (setpoint, 0),
                    EncoderChange::Up(value) => {
                        let (mut step, _delay) = cie_correction(setpoint, value);
                        if setpoint >= 3900 { step = 1};
                        if MAX_PWM_VALUE - setpoint <= step {
                            (MAX_PWM_VALUE, 0)
                        } else {
                            (setpoint + step, value >> 2)
                        }
                    }
                    EncoderChange::Down(value) => {
                        let (step, _delay) = cie_correction(setpoint, value);
                        if step >= setpoint {
                            (0, value >> 2)
                        } else {
                            (setpoint - step, value >> 2)
                        }
                    }
                };

                if updated_setpoint == current_brightness {
                    (self.state, None)
                } else {
                    let (step, delay) = cie_correction(current_brightness, rate);

                    let new_brightness = if current_brightness < updated_setpoint {
                        let temp = current_brightness + step;
                        if temp >= updated_setpoint {
                            updated_setpoint
                        } else {
                            temp
                        }
                    } else {
                        if step >= current_brightness {
                            updated_setpoint
                        } else {
                            current_brightness - step
                        }
                    };
                    (
                        ChasingSetpoint(new_brightness, updated_setpoint, delay),
                        SetBrightness(new_brightness),
                    )
                }
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
fn cie_correction(brightness: u16, rate_of_change: u16) -> (u16, u8) {
    let value: u16 = ((brightness + CIE_CORRECTION_SCALING_FACTOR) * 7) >> 11;
    let change_rate: u16 = MIN_CHANGE_RATE + rate_of_change;
    if value >= change_rate {
        // step. delay
        (1, (change_rate - MIN_CHANGE_RATE) as u8)
    } else {
        (change_rate - value, 0)
    }
}
