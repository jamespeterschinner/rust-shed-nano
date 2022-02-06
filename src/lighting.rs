use crate::cie_correction::{CIE_DELAY, CIE_THRESHOLD};

#[derive(PartialEq, Copy, Clone)]
enum LightingState {
    Disabled,
    RelayOnWait,
    ChasingSetpoint { current_value: u16, setpoint: u16 },
    AtSetpoint(u16),
    LDDShutdownWait,
}

pub enum LightingAction {
    None,
    TurnRelayOn { switch_latency: u16 },
    TurnRelayOff,
    SetBrightness { brightness: u16, duration: u16 },
    SetClock(u16),
}

use LightingAction::*;
use LightingState::*;

pub struct LightingFSM {
    state: LightingState,
}

// The maximum value which the LDD chips can be given with out
// the latch timing getting out of sync
const MAX_PWM_VALUE: u16 = 0xFD2;

const INITIAL_ON_SETPOINT: u16 = 0xABC;

const SOFT_SWITCHING_DELAY: u16 = 0xFFFF;

impl LightingFSM {
    pub const fn new() -> Self {
        LightingFSM {
            state: LightingState::Disabled,
        }
    }
    pub fn toggle(&mut self) -> LightingAction {
        let (new_state, output) = match self.state {
            Disabled => (
                RelayOnWait,
                TurnRelayOn {
                    switch_latency: SOFT_SWITCHING_DELAY,
                },
            ),
            LDDShutdownWait => initial_on(MAX_PWM_VALUE),
            RelayOnWait => (Disabled, TurnRelayOff),
            ChasingSetpoint {
                current_value,
                setpoint: MAX_PWM_VALUE,
            } => initial_on(current_value),
            ChasingSetpoint { current_value, .. } => (
                ChasingSetpoint {
                    current_value,
                    setpoint: MAX_PWM_VALUE,
                },
                None,
            ),
            AtSetpoint(MAX_PWM_VALUE) => initial_on(MAX_PWM_VALUE),
            AtSetpoint(current_value) => (
                ChasingSetpoint {
                    current_value,
                    setpoint: MAX_PWM_VALUE,
                },
                None,
            ),
        };
        self.state = new_state;
        output
    }

    pub fn clock(&mut self) -> LightingAction {
        let (new_state, output) = match self.state {
            RelayOnWait => initial_on(MAX_PWM_VALUE),
            ChasingSetpoint {
                current_value,
                setpoint,
            } => {
                if current_value == setpoint {
                    (AtSetpoint(current_value), SetClock(SOFT_SWITCHING_DELAY))
                } else {
                    let new_value = if current_value > setpoint {
                        current_value - 1
                    } else {
                        current_value + 1
                    };
                    (
                        ChasingSetpoint {
                            current_value: new_value,
                            setpoint,
                        },
                        SetBrightness {
                            brightness: new_value,
                            duration: look_up_duration(new_value),
                        },
                    )
                }
            }
            AtSetpoint(MAX_PWM_VALUE) => (
                LDDShutdownWait,
                SetBrightness {
                    brightness: 0xFFF,
                    duration: SOFT_SWITCHING_DELAY,
                },
            ),
            LDDShutdownWait => (Disabled, TurnRelayOff),
            _ => (self.state, None),
        };

        self.state = new_state;
        return output;
    }
}

#[inline]
fn initial_on(current_value: u16) -> (LightingState, LightingAction) {
    (
        ChasingSetpoint {
            current_value,
            setpoint: INITIAL_ON_SETPOINT,
        },
        SetBrightness {
            brightness: MAX_PWM_VALUE,
            duration: look_up_duration(MAX_PWM_VALUE),
        },
    )
}

fn look_up_duration(brightness: u16) -> u16 {
    // if brightness <= CIE_THRESHOLD {
    //     brightness + 10
    // } else {
    //     CIE_DELAY.load_at((brightness - CIE_THRESHOLD).into())
    // }
    30000
}
