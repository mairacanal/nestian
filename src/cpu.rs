use std::panic;

use crate::context::Context;
use crate::memory::Memory;

const STACK_OFFSET: u16 = 0x0100;

// https://www.nesdev.org/wiki/CPU_addressing_modes
enum AddrMode {
    Imp,    // implicit
    Acc,    // val = A
    Imm,    // val = arg_8
    IndJmp, // val = *arg_16 for jmp operations
    Rel,    // val = arg_8 as offset for branch operations
    Abs,    // val = *arg_16
    AbsJmp, // val = arg_16 for jmp operations
    Zp,     // val = arg_8 for fetching the value from a 8-bit address on the zero page
    ZpIndX, // val = *((arg_8 + X) % 256), takes 4 cycles
    ZpIndY, // val = *((arg_8 + Y) % 256), takes 4 cycles
    AbsX,   // val = *(arg_8 + X), takes 4 cycles or more
    AbsY,   // val = *(arg_8 + Y), takes 4 cycles or more
    IndX,   // val = *(*((arg + X) % 256) + *((arg + X + 1) % 256) * 256), takes 6 cycles
    IndY,   // val = *(*(arg) + *((arg + 1) % 256) * 256 + Y), takes +5 cycles
}

enum OperandKind {
    Acc,
    Imm,
    Addr,
}

struct Operand {
    value: u16,
    kind: OperandKind,
}

pub struct Cpu {
    memory: Box<Memory>,
    context: Context,
    cycle: u32,
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            memory: Box::new(Memory::new()),
            context: Context::new(),
            cycle: 0,
        }
    }

    pub fn load_program(&mut self, program: Vec<u8>, addr: u16) {
        self.memory.set_bytes(addr, program);
    }

    pub fn power_on(&mut self) {
        self.context.S = 0x34;
        self.context.set_flags(0x34);
    }

    pub fn run(&mut self, addr: u16) {
        self.power_on();
        self.context.PC = addr;
        self.execute();
    }

    // Memory Management

    pub fn peek(&self, addr: &u16) -> u8 {
        self.memory.get_byte(*addr)
    }

    pub fn poke(&mut self, addr: &u16, value: u8) {
        self.memory.set_byte(*addr, value);
    }

    // Stack Management

    /// Stack grows from the top-down
    /// No underflow/overflow detection
    pub fn push_byte(&mut self, value: u8) {
        self.memory
            .set_byte(self.context.S as u16 + STACK_OFFSET, value);
        self.context.S -= 1;
    }

    /// High-order bytes push first since the stack grows
    /// top-down and the CPU is little-endian
    pub fn push_word(&mut self, value: u16) {
        self.push_byte((value >> 8) as u8);
        self.push_byte((value & 0xFF) as u8);
    }

    pub fn pop_byte(&mut self) -> u8 {
        self.context.S += 1;
        self.memory.get_byte(self.context.S as u16 + STACK_OFFSET)
    }

    /// Low-order bytes pop first since the stack grows
    /// top-down and the CPU is little-endian
    pub fn pop_word(&mut self) -> u16 {
        let lsb: u16 = self.pop_byte() as u16;
        let msb: u16 = self.pop_byte() as u16;

        (msb << 8) + lsb
    }

    // Execution

    fn execute(&mut self) {
        loop {
            let op_code = self.decode_byte();

            match op_code {
                0x69 => self.adc(AddrMode::Imm),
                _ => panic!("Not an instruction!"),
            }
        }
    }

    fn decode_byte(&mut self) -> u8 {
        let byte = self.memory.get_byte(self.context.PC);
        self.context.PC += 1;
        byte
    }

    fn decode_word(&mut self) -> u16 {
        let word = self.memory.get_word(self.context.PC);
        self.context.PC += 2;
        word
    }

    fn cycle(&mut self, count: u8) {
        self.cycle += count as u32;
    }

    fn decode_operand(&mut self, mode: AddrMode) -> Operand {
        match mode {
            AddrMode::Acc => Operand {
                value: 0,
                kind: OperandKind::Acc,
            },
            AddrMode::Imm => Operand {
                value: self.decode_byte() as u16,
                kind: OperandKind::Imm,
            },
            _ => Operand {
                value: self.decode_operand_addr(mode),
                kind: OperandKind::Addr,
            },
        }
    }

    fn read_operand(&self, op: &Operand) -> u8 {
        match op.kind {
            OperandKind::Acc => self.context.A,
            OperandKind::Imm => op.value as u8,
            OperandKind::Addr => self.peek(&op.value),
        }
    }

    fn write_operand(&mut self, op: &Operand, value: u8) {
        match op.kind {
            OperandKind::Acc => self.context.A = value,
            OperandKind::Addr => self.poke(&op.value, value),
            _ => (),
        }
    }

    fn decode_operand_addr(&mut self, mode: AddrMode) -> u16 {
        match mode {
            AddrMode::Zp => self.decode_byte() as u16,
            AddrMode::ZpIndX => (self.decode_byte() + self.context.X) as u16,
            AddrMode::ZpIndY => (self.decode_byte() + self.context.Y) as u16,
            AddrMode::IndJmp => {
                let word = self.decode_word();
                self.memory.get_word(word)
            }
            AddrMode::Abs => self.decode_word(),
            AddrMode::AbsX => self.decode_word() + self.context.X as u16,
            AddrMode::AbsY => self.decode_word() + self.context.Y as u16,
            AddrMode::IndX => {
                let word = self.decode_byte() as u16;
                let addr = self.peek(&word);
                self.peek(&((addr + self.context.X) as u16)) as u16
                    + (self.peek(&((addr + self.context.X + 1) as u16)) as u16)
                    << 8
            }
            AddrMode::IndY => {
                let addr = self.decode_byte() as u16;
                self.peek(&addr) as u16 + (self.peek(&(addr + 1)) as u16)
                    << 8 + self.context.Y as u16
            }
            _ => {
                panic!("Not implemented yet");
            }
        }
    }

    // CPU instructions

    fn branch(&mut self, cond: bool, mode: AddrMode) {
        assert!(matches!(mode, AddrMode::Rel));

        if cond {
            let PC = self.context.PC as i8 + self.decode_byte() as i8;
            self.context.PC = PC as u16;
        }
    }

    fn calculate_alu_flag(&mut self, value: u8) {
        self.context.set_zero(value == 0);
        self.context.set_negative((value & 0x80) != 0);
    }

    fn determine_overflow_flag(&mut self, old_value: u8, new_value: u8) {
        self.context
            .set_overflow((old_value & 0x80) != (new_value & 0x80));
    }

    // ADC: Add with carry
    fn adc(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        // A + M + C -> A
        self.context.A += value + self.context.get_carry() as u8;

        // Update Flags
        // TODO: Update Carry flag
        self.determine_overflow_flag(value, self.context.A);
        self.calculate_alu_flag(self.context.A);
    }

    // AND: Logical AND
    fn and(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        self.context.A &= value;

        // Update Flags
        self.calculate_alu_flag(self.context.A);
    }

    // ASL: Arithmetic Shift Left
    fn asl(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let new_value = value << 1;

        self.write_operand(&operand, new_value);

        // Update Flags
        self.context.set_carry((value & 0x80) != 0);
        self.calculate_alu_flag(new_value);
    }

    // BCC: Branch if Carry Clean
    fn bcc(&mut self, mode: AddrMode) {
        self.branch(!self.context.get_carry(), mode);
    }

    // BCS: Branch if Carry Set
    fn bcs(&mut self, mode: AddrMode) {
        self.branch(self.context.get_carry(), mode);
    }

    // BEQ: Branch if Equal
    fn beq(&mut self, mode: AddrMode) {
        self.branch(self.context.is_zero(), mode);
    }

    // BIT: Test bit
    fn bit(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let mask = self.context.A & value;

        // Update Flags
        self.context.set_zero(mask == 0);
        self.context.set_overflow((value & 0x40) != 0);
        self.context.set_negative((value & 0x80) != 0);
    }

    // BMI: Branch if Minus
    fn bmi(&mut self, mode: AddrMode) {
        self.branch(self.context.is_negative(), mode);
    }

    // BNE: Branch if Not Equal
    fn bne(&mut self, mode: AddrMode) {
        self.branch(!self.context.is_zero(), mode);
    }

    // BPL: Branch if Positive
    fn bpl(&mut self, mode: AddrMode) {
        self.branch(!self.context.is_negative(), mode);
    }

    // BRK: Break
    fn brk(&mut self, mode: AddrMode) {}

    // BVC: Branch if Overflow Clear
    fn bvc(&mut self, mode: AddrMode) {
        self.branch(!self.context.is_overflow(), mode);
    }

    // BVS: Branch if Overflow Set
    fn bvs(&mut self, mode: AddrMode) {
        self.branch(self.context.is_overflow(), mode);
    }

    // CLC: Clear Carry Flag
    fn clc(&mut self, mode: AddrMode) {
        self.context.set_carry(false);
    }

    // CLD: Clear Decimal Flag
    fn cld(&mut self, mode: AddrMode) {
        self.context.set_decimal(false);
    }

    // CLI: Clear Interrupt Flag
    fn cli(&mut self, mode: AddrMode) {
        self.context.set_interrupt(false);
    }

    // CLV: Clear Overflow Flag
    fn clv(&mut self, mode: AddrMode) {
        self.context.set_overflow(false);
    }

    // CMP: Compare
    fn cmp(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        // Update Flags
        if self.context.A < value {
            self.context.set_negative(true);
        } else if self.context.A > value {
            self.context.set_carry(true);
        } else {
            self.context.set_carry(true);
            self.context.set_zero(true);
        }
    }

    // CPX: Compare X Register
    fn cpx(&mut self, mode: AddrMode) {}

    // CPY: Compare Y Register
    fn cpy(&mut self, mode: AddrMode) {}

    // DEC: Decrement Memory
    fn dec(&mut self, mode: AddrMode) {
        let addr = self.decode_operand_addr(mode);
        let new_value = self.peek(&addr) - 1;

        self.poke(&addr, new_value);

        self.calculate_alu_flag(new_value);
    }

    // DEX: Decrement X Register
    fn dex(&mut self, mode: AddrMode) {
        self.context.X -= 1;
        self.calculate_alu_flag(self.context.X);
    }

    // DEY: Decrement Y Register
    fn dey(&mut self, mode: AddrMode) {
        self.context.Y -= 1;
        self.calculate_alu_flag(self.context.Y);
    }

    // EOR: Exclusive OR
    fn eor(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        self.context.A ^= value;

        // Update Flags
        self.calculate_alu_flag(self.context.A);
    }

    // INC: Increment Memory
    fn inc(&mut self, mode: AddrMode) {
        let addr = self.decode_operand_addr(mode);
        let new_value = self.peek(&addr) + 1;

        self.poke(&addr, new_value);

        self.calculate_alu_flag(new_value);
    }

    // INX: Increment X Register
    fn inx(&mut self, mode: AddrMode) {
        self.context.X += 1;
        self.calculate_alu_flag(self.context.X);
    }

    // INY: Increment Y Register
    fn iny(&mut self, mode: AddrMode) {
        self.context.Y += 1;
        self.calculate_alu_flag(self.context.Y);
    }

    // JMP: Jump
    fn jmp(&mut self, mode: AddrMode) {
        self.context.PC = self.decode_operand_addr(mode);
    }

    // JSR: Jump to Subroutine
    fn jsr(&mut self, mode: AddrMode) {
        // We push the actual return address - 1, which is the current
        // place + 1
        self.push_word(self.context.PC + 1);
        self.context.PC = self.decode_operand_addr(mode);
    }

    // LDA: Load Accumulator
    fn lda(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        self.context.A = value;

        self.calculate_alu_flag(self.context.A);
    }

    // LDX: Load X Register
    fn ldx(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        self.context.X = value;

        self.calculate_alu_flag(self.context.X);
    }

    // LDY: Load Y Register
    fn ldy(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        self.context.Y = value;

        self.calculate_alu_flag(self.context.Y);
    }

    // LSR: Logical Shift Right
    fn lsr(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let new_value = value >> 1;

        self.write_operand(&operand, new_value);

        // Update Flags
        self.context.set_carry((value & 0x01) != 0);
        self.calculate_alu_flag(new_value);
    }

    // NOP: No Operation
    fn nop(&mut self, mode: AddrMode) {}

    // ORA: Logical Inclusive OR
    fn ora(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        self.context.A |= value;

        // Update Flags
        self.calculate_alu_flag(self.context.A);
    }

    /// PHA: Push Accumulator
    ///
    /// Pushes a copy of the accumulator on to the stack.
    fn pha(&mut self, mode: AddrMode) {
        self.push_byte(self.context.A);
    }

    /// PHP: Push Processor Status
    ///
    /// Pushes a copy of the status flags on to the stack.
    fn php(&mut self, mode: AddrMode) {
        self.push_byte(self.context.P);
    }

    /// PLA: Push Accumulator
    ///
    /// Pulls an 8 bit value from the stack and into the accumulator. The zero
    /// and negative flags are set as appropriate.
    fn pla(&mut self, mode: AddrMode) {
        self.context.A = self.pop_byte();
        self.calculate_alu_flag(self.context.A);
    }

    /// PLP: Pull Processor Status
    ///
    /// Pulls an 8 bit value from the stack and into the processor flags. The
    /// flags will take on new states as determined by the value pulled.
    fn plp(&mut self, mode: AddrMode) {
        let status = self.pop_byte();
        self.context.set_flags(status);
    }

    /// ROL: Rotate Left
    ///
    /// Move each of the bits in either A or M one place to the left. Bit 0 is
    /// filled with the current value of the carry flag whilst the old bit 7
    /// becomes the new carry flag value.
    fn rol(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let new_value = (value << 1) & self.context.get_carry() as u8;

        self.write_operand(&operand, new_value);

        // Update Flags
        self.context.set_carry((value & 0x80) != 0);
        self.context.set_zero(self.context.A == 0);
        self.context.set_negative((new_value & 0x80) != 0);
    }

    /// ROR: Rotate Right
    ///
    /// Move each of the bits in either A or M one place to the right. Bit 7
    /// is filled with the current value of the carry flag whilst the old bit
    /// 0 becomes the new carry flag value.
    fn ror(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let new_value = (value >> 1) & ((self.context.get_carry() as u8) << 7);

        self.write_operand(&operand, new_value);

        // Update Flags
        self.context.set_carry((value & 0x01) != 0);
        self.context.set_zero(self.context.A == 0);
        self.context.set_negative((new_value & 0x80) != 0);
    }

    /// RTI: Return from Interrupt
    ///
    /// The RTI instruction is used at the end of an interrupt processing routine.
    /// It pulls the processor flags from the stack followed by the program counter.
    fn rti(&mut self, mode: AddrMode) {}

    /// RTS: Return from Subroutine
    ///
    /// The RTS instruction is used at the end of a subroutine to return to the
    /// calling routine. It pulls the program counter (minus one) from the stack.
    fn rts(&mut self, mode: AddrMode) {
        // Set JSR: we pushed the actual return address - 1.
        let ret = self.pop_word() + 1;
        self.context.PC = ret;
    }

    /// SBC: Subtract with Carry
    ///
    /// This instruction subtracts the contents of a memory location to the
    /// accumulator together with the not of the carry bit. If overflow occurs
    /// the carry bit is clear, this enables multiple byte subtraction to be
    /// performed.
    fn sbc(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        // A,Z,C,N = A-M-(1-C)
        self.context.A = self.context.A - value - (1 - self.context.get_carry() as u8);

        self.context.set_carry(self.context.A > value);
        self.determine_overflow_flag(value, self.context.A);
        self.calculate_alu_flag(self.context.A);
    }

    /// SEC: Set Carry Flag
    ///
    /// Set the carry flag to one.
    fn sec(&mut self, mode: AddrMode) {
        self.context.set_carry(true);
    }

    /// SED: Set Decimal Flag
    ///
    /// Set the decimal flag to one.
    fn sed(&mut self, mode: AddrMode) {
        self.context.set_decimal(true);
    }

    /// SEI: Set Interrupt Flag
    ///
    /// Set the interrupt disable flag to one.
    fn sei(&mut self, mode: AddrMode) {
        self.context.set_interrupt(true);
    }

    /// STA: Store Accumulator
    ///
    /// Stores the contents of the accumulator into memory.
    fn sta(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = operand.value;

        self.poke(&value, self.context.A);
    }

    /// STX: Store X Register
    ///
    /// Stores the contents of the X Register into memory.
    fn stx(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = operand.value;

        self.poke(&value, self.context.X);
    }

    /// STY: Store Y Register
    ///
    /// Stores the contents of the Y Register into memory.
    fn sty(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = operand.value;

        self.poke(&value, self.context.Y);
    }

    /// TAX: Transfer Accumulator to X
    ///
    /// Copies the current contents of the accumulator into the X register and
    /// sets the zero and negative flags as appropriate.
    fn tax(&mut self, mode: AddrMode) {
        self.context.X = self.context.A;
        self.calculate_alu_flag(self.context.X);
    }

    /// TAY: Transfer Accumulator to Y
    ///
    /// Copies the current contents of the accumulator into the Y register and
    /// sets the zero and negative flags as appropriate.
    fn tay(&mut self, mode: AddrMode) {
        self.context.Y = self.context.A;
        self.calculate_alu_flag(self.context.Y);
    }

    /// TSX: Transfer Stack Pointer to X
    ///
    /// Copies the current contents of the stack register into the X register
    /// and sets the zero and negative flags as appropriate.
    fn tsx(&mut self, mode: AddrMode) {
        self.context.X = self.context.S;
        self.calculate_alu_flag(self.context.X);
    }

    /// TXA: Transfer X to Accumulator
    ///
    /// Copies the current contents of the X register into the accumulator
    /// and sets the zero and negative flags as appropriate.
    fn txa(&mut self, mode: AddrMode) {
        self.context.A = self.context.X;
        self.calculate_alu_flag(self.context.A);
    }

    /// TXS: Transfer X to Stack Pointer
    ///
    /// Copies the current contents of the X register into the stack register.
    fn txs(&mut self, mode: AddrMode) {
        self.context.S = self.context.X;
    }

    /// TYA: Transfer Y to Accumulator
    ///
    /// Copies the current contents of the Y register into the accumulator
    /// and sets the zero and negative flags as appropriate.
    fn tya(&mut self, mode: AddrMode) {
        self.context.A = self.context.Y;
        self.calculate_alu_flag(self.context.A);
    }
}
