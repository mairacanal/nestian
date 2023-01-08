use bit_vec::BitVec;

pub struct Cpu {
    context : Context,
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            context: Context::new(),
        }
    }
}

// https://www.nesdev.org/wiki/Status_flags
struct Flags {
    carry : bool,
    zero : bool,
    interrupt : bool,
    decimal : bool,
    B : bool,
    I : bool,
    overflow : bool,
    negative : bool,
}

impl Flags {
    fn new() -> Flags {
        Flags {
            carry: false,
            zero: false,
            interrupt: false,
            decimal: false,
            B: false,
            I: false,
            overflow: false,
            negative: false
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
}

// https://www.nesdev.org/wiki/CPU_addressing_modes
enum AddrMode {
    imp,            // implicit
    acc,            // val = A
    imm,            // val = arg_8
    ind_jmp,        // val = *arg_16 for jmp operations
    rel,            // val = arg_8 as offset for branch operations
    abs,            // val = *arg_16
    abs_jmp,        // val = arg_16 for jmp operations
    zp,             // val = arg_8 for fetching the value from a 8-bit address on the zero
                    // page 
    zp_ind_x,       // val = *((arg_8 + X) % 256), takes 4 cycles
    zp_ind_y,       // val = *((arg_8 + Y) % 256), takes 4 cycles
    abs_x,          // val = *(arg_8 + X), takes 4 cycles or more
    abs_y,          // val = *(arg_8 + Y), takes 4 cycles or more
    ind_x,          // val = *(*((arg + X) % 256) + *((arg + X + 1) % 256) * 256), takes 6
                    // cycles
    ind_y           // val = *(*(arg) + *((arg + 1) % 256) * 256 + Y), takes +5 cycles
}

enum OperandKind {
    acc,
    imm,
    addr,
}

struct Operand {
    value: u16,
    kind: OperandKind,
    is_page_crossing: bool
}

struct Context {
    // Accumulator
    A : u8,

    // Index Register X
    X : u8,

    // Index Register Y
    Y : u8,

    // Program Counter
    PC : u16,

    // Stack Pointer
    S : u8,

    // Status Register
    P : u8,

    flags: Flags,
}

impl Context {
    fn new() -> Context {
        Context { A: 0, X: 0, Y: 0, PC: 0, S: 0, P: 0 , flags: Flags::new() }
    }

    pub fn set_carry(&mut self, arg : bool) {
        self.flags.carry = arg;
        self.S = self.flags.get_status();
    }

    pub fn set_zero(&mut self, arg : bool) {
        self.flags.zero = arg;
        self.S = self.flags.get_status();
    }

    pub fn set_interrupt(&mut self, arg : bool) {
        self.flags.interrupt = arg;
        self.S = self.flags.get_status();
    }

    pub fn set_decimal(&mut self, arg : bool) {
        self.flags.decimal = arg;
        self.S = self.flags.get_status();
    }

    pub fn set_B(&mut self, arg : bool) {
        self.flags.B = arg;
        self.S = self.flags.get_status();
    }

    pub fn set_I(&mut self, arg : bool) {
        self.flags.I = arg;
        self.S = self.flags.get_status();
    }

    pub fn set_overflow(&mut self, arg : bool) {
        self.flags.overflow = arg;
        self.S = self.flags.get_status();
    }

    pub fn set_negative(&mut self, arg : bool) {
        self.flags.negative = arg;
        self.S = self.flags.get_status();
    }
}
