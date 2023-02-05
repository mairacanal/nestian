const RAM_SIZE : u32 = 0x10000;

pub struct Memory {
    ram: Vec<u8>,
}

impl Memory {
    pub fn new() -> Self {
        Memory {
           ram: vec![0; RAM_SIZE as usize],
        }
    }

    pub fn get_byte(&self, addr: u16) -> u8 {
        self.ram[addr as usize]
    }

    /// NES 6502 CPU is little endian
    pub fn get_word(&self, addr: u16) -> u16 {
        self.ram[addr as usize] as u16 + (self.ram[addr as usize + 1] as u16) << 8
    }

    pub fn set_byte(&mut self, addr: u16, value: u8) {
        self.ram[addr as usize] = value;
    }

    // TODO
    pub fn set_bytes(&mut self, addr: u16, data: Vec<u8>) {
    }

    /// NES 6502 CPU is little endian
    pub fn set_word(&mut self, addr: u16, value: u16) {
        self.ram[addr as usize] = (value & 0xFF) as u8;
        self.ram[addr as usize + 1] = (value >> 8) as u8;
    }
}
