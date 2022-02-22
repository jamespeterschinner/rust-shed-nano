const CLOCK_SCALER: u8 = 2;

static VALID_ROTATION: u16 = 0b0110100110010110;

pub enum EncoderChange {
    None,
    Up(u16),
    Down(u16),
}

pub struct Encoder {
    previous_and_current_code: u8,
    store: u16,
    clock_count: u8,
    rate_of_change: u16,
}

impl Encoder {
    pub const fn new() -> Self {
        Encoder {
            previous_and_current_code: 0,
            store: 0,
            clock_count: CLOCK_SCALER,
            rate_of_change: 1,
        }
    }

    pub fn clock(&mut self) {
        self.clock_count -= 1;

        if self.clock_count == 0 && (self.rate_of_change > 0) {
            self.rate_of_change -= 1;
            self.clock_count = CLOCK_SCALER;
        }
    }

    pub fn read(&mut self, a: bool, b: bool) -> EncoderChange {
        self.previous_and_current_code <<= 2;
        if a {
            self.previous_and_current_code |= 0b10
        };
        if b {
            self.previous_and_current_code |= 0b01
        };
        self.previous_and_current_code &= 0x0F;

        if VALID_ROTATION >> self.previous_and_current_code & 0x01 > 0 {
            self.store <<= 4;
            self.store |= self.previous_and_current_code as u16;

            // I believe the detents on the encoder leave both encoder outputs
            // in the same state either open or closed. Therefore for each direction
            // there are two state transitions of interest from open to closed and closed to
            // open
            match self.store {
                0xbd42 | 0x17e8 => {
                    // This are 'bad' values I think the encoder generates these
                    // often due to switch bouncing
                    EncoderChange::None
                }
                0x42bd | 0x2bd4 |
                0xd42b | 0x42b | 0x2bd | 0xbd4 | 0xd42 | 0x42 | 0x2b
                | 0xbd | 0xd4 
                => {
                    self.rate_of_change += 1;
                    EncoderChange::Down(self.rate_of_change)
                }
                0xe817 | 0x817e |
                0x7e81 | 0xe81 | 0x817 | 0x17e | 0x7e8 | 0xe8 | 0x81 | 0x17 | 0x7e
                 => {
                    self.rate_of_change += 1;
                    EncoderChange::Up(self.rate_of_change)
                }
                _ => {
                    EncoderChange::None
                }
            }
        } else {
            EncoderChange::None
        }
    }
}
