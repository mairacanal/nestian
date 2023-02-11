use bit_vec::BitVec;

// https://www.nesdev.org/wiki/Status_flags
struct Flags {
    carry: bool,
    zero: bool,
    interrupt: bool,
    decimal: bool,
    B: bool,
    I: bool,
    overflow: bool,
    negative: bool,
}

impl Flags {
    fn new() -> Self {
        Self {
            carry: false,
            zero: false,
            interrupt: false,
            decimal: false,
            B: false,
            I: false,
            overflow: false,
            negative: false,
        }
    }

    fn get_status(&self) -> u8 {
        let mut status = BitVec::from_elem(8, false);

        status.set(7, self.carry);
        status.set(6, self.zero);
        status.set(5, self.interrupt);
        status.set(4, self.decimal);
        status.set(3, self.B);
        status.set(2, self.I);
        status.set(1, self.overflow);
        status.set(0, self.negative);

        status.to_bytes()[0]
    }

    fn set_status(&mut self, status: u8) {
        let status = BitVec::from_bytes(&[status]);

        self.carry = status.get(7).unwrap();
        self.zero = status.get(6).unwrap();
        self.interrupt = status.get(5).unwrap();
        self.decimal = status.get(4).unwrap();
        self.B = status.get(3).unwrap();
        self.I = status.get(2).unwrap();
        self.overflow = status.get(1).unwrap();
        self.negative = status.get(0).unwrap();
    }
}

pub struct Context {
    // Accumulator
    pub A: u8,

    // Index Register X
    pub X: u8,

    // Index Register Y
    pub Y: u8,

    // Program Counter
    pub PC: u16,

    // Stack Pointer
    pub S: u8,

    // Status Register
    pub P: u8,

    flags: Flags,
}

impl Context {
    pub fn new() -> Self {
        Self { A: 0, X: 0, Y: 0, PC: 0, S: 0, P: 0, flags: Flags::new(), }
    }

    pub fn set_carry(&mut self, arg: bool) {
        self.flags.carry = arg;
        self.P = self.flags.get_status();
    }

    pub fn get_carry(&self) -> bool {
        self.flags.carry
    }

    pub fn set_zero(&mut self, arg: bool) {
        self.flags.zero = arg;
        self.P = self.flags.get_status();
    }

    pub fn is_zero(&self) -> bool {
        self.flags.zero
    }

    pub fn set_interrupt(&mut self, arg: bool) {
        self.flags.interrupt = arg;
        self.P = self.flags.get_status();
    }

    pub fn is_interrupt(&self) -> bool {
        self.flags.interrupt
    }

    pub fn set_decimal(&mut self, arg: bool) {
        self.flags.decimal = arg;
        self.P = self.flags.get_status();
    }

    pub fn set_B(&mut self, arg: bool) {
        self.flags.B = arg;
        self.P = self.flags.get_status();
    }

    pub fn set_I(&mut self, arg: bool) {
        self.flags.I = arg;
        self.P = self.flags.get_status();
    }

    pub fn set_overflow(&mut self, arg: bool) {
        self.flags.overflow = arg;
        self.P = self.flags.get_status();
    }

    pub fn is_overflow(&self) -> bool {
        self.flags.overflow
    }

    pub fn set_negative(&mut self, arg: bool) {
        self.flags.negative = arg;
        self.P = self.flags.get_status();
    }

    pub fn is_negative(&self) -> bool {
        self.flags.negative
    }

    pub fn set_flags(&mut self, status: u8) {
        self.P = status;
        self.flags.set_status(self.P);
    }
}

