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
        let addr = addr as usize;

        self.ram[addr] as u16 + ((self.ram[addr + 1] as u16) << 8)
    }

    pub fn set_byte(&mut self, addr: u16, value: u8) {
        self.ram[addr as usize] = value;
    }

    pub fn set_bytes(&mut self, addr: u16, data: Vec<u8>) {
        assert!((data.len() + addr as usize) < RAM_SIZE as usize);

        let addr = addr as usize;
        self.ram[addr..addr + data.len()].copy_from_slice(&data[0..]);
    }

    /// NES 6502 CPU is little endian
    pub fn set_word(&mut self, addr: u16, value: u16) {
        self.ram[addr as usize] = (value & 0xff) as u8;
        self.ram[addr as usize + 1] = (value >> 8) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init() {
        let memory = Memory::new();

        for i in 0..RAM_SIZE {
            assert_eq!(memory.ram[i as usize], 0);
        }
    }

    #[test]
    fn bytes() {
        let mut memory = Memory::new();

        for i in 0..RAM_SIZE {
            assert_eq!(memory.get_byte(i as u16), 0);
        }

        memory.set_byte(0x01ab, 0x11);
        assert_eq!(memory.get_byte(0x01ab), 0x11);

        memory.set_byte(0xffff, 0x1b);
        assert_eq!(memory.get_byte(0xffff), 0x1b);

        memory.set_bytes(0xff, vec![0xab, 0x01, 0x00, 0xfd]);

        assert_eq!(memory.get_byte(0xff), 0xab);
        assert_eq!(memory.get_byte(0x0100), 0x01);
        assert_eq!(memory.get_byte(0x0101), 0x00);
        assert_eq!(memory.get_byte(0x0102), 0xfd);
    }

    #[test]
    fn word() {
        let mut memory = Memory::new();

        for i in 0..RAM_SIZE/2 {
            assert_eq!(memory.get_word(i as u16), 0);
        }

        memory.set_word(0x01ab, 0x110f);

        assert_eq!(memory.get_byte(0x01ab), 0x0f);
        assert_eq!(memory.get_byte(0x01ac), 0x11);

        assert_eq!(memory.get_word(0x01ab), 0x110f);

        memory.set_word(0x0fd2, 0xff01);

        assert_eq!(memory.get_byte(0x0fd2), 0x01);
        assert_eq!(memory.get_byte(0x0fd3), 0xff);

        assert_eq!(memory.get_word(0x0fd2), 0xff01);
    }
}
