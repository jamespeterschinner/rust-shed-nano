#[derive(PartialEq, Copy, Clone)]
enum LightingState {
    Disabled,
    RelayEnableWait,
    LDDEnabled,
    LDDShutdownWait,
}

pub enum LightingAction {
    None,
    EnableRelay,
    DisableRelay,
    EnableLDD,
    DisableLDD,
}

use LightingAction::*;
use LightingState::*;

pub struct LightingFSM {
    state: LightingState,
    soft_switching_delay: u8,
    delay_counter: u8,
}

impl LightingFSM {
    pub const fn new(soft_switching_delay: u8) -> Self {
        LightingFSM {
            state: LightingState::Disabled,
            soft_switching_delay,
            delay_counter: 0,
        }
    }

    pub fn toggle(&mut self) -> LightingAction {
        let (new_state, output) = match self.state {
            // In disabled/shutting down state
            Disabled => (RelayEnableWait, EnableRelay),
            LDDShutdownWait => (LDDEnabled, EnableLDD),
            // In started/starting state
            RelayEnableWait => (Disabled, DisableRelay),
            LDDEnabled => (LDDShutdownWait, DisableLDD),
        };
        self.state = new_state;
        output
    }

    pub fn clock(&mut self) -> LightingAction {
        if self.state == RelayEnableWait || self.state == LDDShutdownWait {
            self.delay_counter += 1
        }

        if self.delay_counter == self.soft_switching_delay {
            let (new_state, output) = match self.state {
                RelayEnableWait => (LDDEnabled, EnableLDD),
                LDDShutdownWait => (Disabled, DisableRelay),
                _ => (self.state, None),
            };
            self.state = new_state;
            self.delay_counter = 0;
            return output;
        }
        return None;
    }
}
