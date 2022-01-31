#[derive(PartialEq, Copy, Clone)]
enum FanState {
    Disabled,
    VSDPowerUpWait,
    FanHighSpeed,
    FanLowSpeed,
    VSDPowerDownWait,
}

pub enum FanAction {
    None,
    PowerUpVSD,
    PowerDownVSD,
    RunLowSpeed,
    RunHighSpeed,
    DisableVSD,
}

use FanAction::*;
use FanState::*;

pub struct FanFSM {
    state: FanState,
    vsd_power_up_delay: u8,
    vsd_power_down_delay: u16,
    delay_counter: u16,
}

impl FanFSM {
    pub const fn new(vsd_power_up_delay: u8, vsd_power_down_delay: u16) -> Self {
        FanFSM {
            state: FanState::Disabled,
            vsd_power_up_delay,
            vsd_power_down_delay,
            delay_counter: 0,
        }
    }

    pub fn start(&mut self) -> FanAction {
        let (new_state, output) = match self.state {
            // In disabled/shutting down state
            Disabled => (VSDPowerUpWait, PowerUpVSD),
            VSDPowerDownWait => (FanHighSpeed, RunHighSpeed),
            FanLowSpeed => (FanHighSpeed, RunHighSpeed),
            FanHighSpeed => (FanLowSpeed, RunLowSpeed),
            _ => (self.state, None),
        };
        self.state = new_state;
        output
    }

    pub fn stop(&mut self) -> FanAction {
        let (new_state, output) = match self.state {
            VSDPowerUpWait => (Disabled, PowerDownVSD),
            FanLowSpeed => (VSDPowerDownWait, DisableVSD),
            FanHighSpeed => (VSDPowerDownWait, DisableVSD),
            _ => (self.state, None),
        };
        self.state = new_state;
        output
    }

    pub fn clock(&mut self) -> FanAction {
        if self.state == VSDPowerUpWait || self.state == VSDPowerDownWait {
            self.delay_counter += 1;
        } else {
            self.delay_counter = 0;
        }

        if self.state == VSDPowerUpWait && self.delay_counter as u8 >= self.vsd_power_up_delay {
            self.state = FanHighSpeed;
            return RunHighSpeed
        }

        if self.state == VSDPowerDownWait && self.delay_counter >= self.vsd_power_down_delay{
            self.state = Disabled;
            return PowerDownVSD
        }

        return None
    }
}
