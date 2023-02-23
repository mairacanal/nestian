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

    pub fn power_on(&mut self) {
        self.context.S = 0x34;
        self.context.set_flags(0x34);
    }

    pub fn run_program(&mut self, program: Vec<u8>, addr: u16) {
        self.power_on();

        self.memory.set_bytes(addr, program);

        self.context.PC = addr;
        self.execute();
    }

    // Memory Management

    pub fn peek(&self, addr: &u16) -> u8 {
        self.memory.get_byte(*addr)
    }

    pub fn peek_word(&self, addr: &u16) -> u16 {
        self.memory.get_word(*addr)
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
                0x65 => self.adc(AddrMode::Zp),
                0x75 => self.adc(AddrMode::ZpIndX),
                0x6d => self.adc(AddrMode::Abs),
                0x7d => self.adc(AddrMode::AbsX),
                0x79 => self.adc(AddrMode::AbsY),
                0x61 => self.adc(AddrMode::IndX),
                0x71 => self.adc(AddrMode::IndY),

                0x29 => self.and(AddrMode::Imm),
                0x25 => self.and(AddrMode::Zp),
                0x35 => self.and(AddrMode::ZpIndX),
                0x2d => self.and(AddrMode::Abs),
                0x3d => self.and(AddrMode::AbsX),
                0x39 => self.and(AddrMode::AbsY),
                0x21 => self.and(AddrMode::IndX),
                0x31 => self.and(AddrMode::IndY),

                0x0a => self.asl(AddrMode::Acc),
                0x06 => self.asl(AddrMode::Zp),
                0x16 => self.asl(AddrMode::ZpIndX),
                0x0e => self.asl(AddrMode::Abs),
                0x1e => self.asl(AddrMode::AbsX),

                0x90 => self.bcc(AddrMode::Rel),

                0xb0 => self.bcs(AddrMode::Rel),

                0xf0 => self.beq(AddrMode::Rel),

                0x24 => self.bit(AddrMode::Zp),
                0x2c => self.bit(AddrMode::Abs),

                0x30 => self.bmi(AddrMode::Rel),

                0xd0 => self.bne(AddrMode::Rel),

                0x10 => self.bpl(AddrMode::Rel),

                // 0x00 => self.brk(AddrMode::Imp),
                0x00 => break,

                0x50 => self.bvc(AddrMode::Rel),

                0x70 => self.bvs(AddrMode::Rel),

                0x18 => self.clc(AddrMode::Imp),

                0xd8 => self.cld(AddrMode::Imp),

                0x58 => self.cli(AddrMode::Imp),

                0xb8 => self.clv(AddrMode::Imp),

                0xc9 => self.cmp(AddrMode::Imm),
                0xc5 => self.cmp(AddrMode::Zp),
                0xd5 => self.cmp(AddrMode::ZpIndX),
                0xcd => self.cmp(AddrMode::Abs),
                0xdd => self.cmp(AddrMode::AbsX),
                0xd9 => self.cmp(AddrMode::AbsY),
                0xc1 => self.cmp(AddrMode::IndX),
                0xd1 => self.cmp(AddrMode::IndY),

                0xe0 => self.cpx(AddrMode::Imm),
                0xe4 => self.cpx(AddrMode::Zp),
                0xec => self.cpx(AddrMode::Abs),

                0xc0 => self.cpy(AddrMode::Imm),
                0xc4 => self.cpy(AddrMode::Zp),
                0xcc => self.cpy(AddrMode::Abs),

                0xc6 => self.dec(AddrMode::Zp),
                0xd6 => self.dec(AddrMode::ZpIndX),
                0xce => self.dec(AddrMode::Abs),
                0xde => self.dec(AddrMode::AbsX),

                0xca => self.dex(AddrMode::Imp),

                0x88 => self.dey(AddrMode::Imp),

                0x49 => self.eor(AddrMode::Imm),
                0x45 => self.eor(AddrMode::Zp),
                0x55 => self.eor(AddrMode::ZpIndX),
                0x4d => self.eor(AddrMode::Abs),
                0x5d => self.eor(AddrMode::AbsX),
                0x59 => self.eor(AddrMode::AbsY),
                0x41 => self.eor(AddrMode::IndX),
                0x51 => self.eor(AddrMode::IndY),

                0xe6 => self.inc(AddrMode::Zp),
                0xf6 => self.inc(AddrMode::ZpIndX),
                0xee => self.inc(AddrMode::Abs),
                0xfe => self.inc(AddrMode::AbsX),

                0xe8 => self.inx(AddrMode::Imp),

                0xc8 => self.iny(AddrMode::Imp),

                0x4c => self.jmp(AddrMode::Abs),
                0x6c => self.jmp(AddrMode::IndJmp),

                0x20 => self.jsr(AddrMode::Abs),

                0xa9 => self.lda(AddrMode::Imm),
                0xa5 => self.lda(AddrMode::Zp),
                0xb5 => self.lda(AddrMode::ZpIndX),
                0xad => self.lda(AddrMode::Abs),
                0xbd => self.lda(AddrMode::AbsX),
                0xb9 => self.lda(AddrMode::AbsY),
                0xa1 => self.lda(AddrMode::IndX),
                0xb1 => self.lda(AddrMode::IndY),

                0xa2 => self.ldx(AddrMode::Imm),
                0xa6 => self.ldx(AddrMode::Zp),
                0xb6 => self.ldx(AddrMode::ZpIndY),
                0xae => self.ldx(AddrMode::Abs),
                0xbe => self.ldx(AddrMode::AbsY),

                0xa0 => self.ldy(AddrMode::Imm),
                0xa4 => self.ldy(AddrMode::Zp),
                0xb4 => self.ldy(AddrMode::ZpIndX),
                0xac => self.ldy(AddrMode::Abs),
                0xbc => self.ldy(AddrMode::AbsX),

                0x4a => self.lsr(AddrMode::Imm),
                0x46 => self.lsr(AddrMode::Zp),
                0x56 => self.lsr(AddrMode::ZpIndX),
                0x4e => self.lsr(AddrMode::Abs),
                0x5e => self.lsr(AddrMode::AbsX),

                0xea => self.nop(AddrMode::Imp),

                0x09 => self.ora(AddrMode::Imm),
                0x05 => self.ora(AddrMode::Zp),
                0x15 => self.ora(AddrMode::ZpIndX),
                0x0d => self.ora(AddrMode::Abs),
                0x1d => self.ora(AddrMode::AbsX),
                0x19 => self.ora(AddrMode::AbsY),
                0x01 => self.ora(AddrMode::IndX),
                0x11 => self.ora(AddrMode::IndY),

                0x48 => self.pha(AddrMode::Imp),

                0x08 => self.php(AddrMode::Imp),

                0x68 => self.pla(AddrMode::Imp),

                0x28 => self.plp(AddrMode::Imp),

                0x2a => self.rol(AddrMode::Imm),
                0x26 => self.rol(AddrMode::Zp),
                0x36 => self.rol(AddrMode::ZpIndX),
                0x2e => self.rol(AddrMode::Abs),
                0x3e => self.rol(AddrMode::AbsX),

                0x6a => self.ror(AddrMode::Imm),
                0x66 => self.ror(AddrMode::Zp),
                0x76 => self.ror(AddrMode::ZpIndX),
                0x6e => self.ror(AddrMode::Abs),
                0x7e => self.ror(AddrMode::AbsX),

                0x40 => self.rti(AddrMode::Imp),

                0x60 => self.rts(AddrMode::Imp),

                0xe9 => self.sbc(AddrMode::Imm),
                0xe5 => self.sbc(AddrMode::Zp),
                0xf5 => self.sbc(AddrMode::ZpIndX),
                0xed => self.sbc(AddrMode::Abs),
                0xfd => self.sbc(AddrMode::AbsX),
                0xf9 => self.sbc(AddrMode::AbsY),
                0xe1 => self.sbc(AddrMode::IndX),
                0xf1 => self.sbc(AddrMode::IndY),

                0x38 => self.sec(AddrMode::Imp),

                0xf8 => self.sed(AddrMode::Imp),

                0x78 => self.sei(AddrMode::Imp),

                0x85 => self.sta(AddrMode::Zp),
                0x95 => self.sta(AddrMode::ZpIndX),
                0x8d => self.sta(AddrMode::Abs),
                0x9d => self.sta(AddrMode::AbsX),
                0x99 => self.sta(AddrMode::AbsY),
                0x81 => self.sta(AddrMode::IndX),
                0x91 => self.sta(AddrMode::IndY),

                0x86 => self.stx(AddrMode::Zp),
                0x96 => self.stx(AddrMode::ZpIndY),
                0x8e => self.stx(AddrMode::Abs),

                0x84 => self.sty(AddrMode::Zp),
                0x94 => self.sty(AddrMode::ZpIndX),
                0x8c => self.sty(AddrMode::Abs),

                0xaa => self.tax(AddrMode::Imp),

                0xa8 => self.tay(AddrMode::Imp),

                0xba => self.tsx(AddrMode::Imp),

                0x8a => self.txa(AddrMode::Imp),

                0x9a => self.txs(AddrMode::Imp),

                0x98 => self.tya(AddrMode::Imp),

                _ => panic!("[CPU] Unrecognized instruction or illegal instruction!"),
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
                let addr = self.decode_word();

                if addr & 0xff == 0xff {
                    // JMP hardware bug
                    // http://wiki.nesdev.com/w/index.php/Errata
                    self.peek(&addr) as u16 + (self.peek(&(addr & 0xff00)) as u16) << 8
                } else {
                    self.peek_word(&addr)
                }
            }
            AddrMode::Abs | AddrMode::AbsJmp => self.decode_word(),
            AddrMode::AbsX => self.decode_word() + self.context.X as u16,
            AddrMode::AbsY => self.decode_word() + self.context.Y as u16,
            AddrMode::IndX => {
                let addr = self.decode_byte();
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

        let rel = self.decode_byte() as i8;
        if cond {
            let pc = self.context.PC as i8 + rel;
            self.context.PC = pc as u16;
        }
    }

    fn calculate_alu_flag(&mut self, value: u8) {
        self.context.set_zero(value == 0);
        self.context.set_negative((value & 0x80) != 0);
    }

    // TODO: is val2 u8 or i8?
    fn is_sign_overflow(&mut self, val1: u8, val2: u8, new_value: u8) -> bool {
        (val1 & 0x80) == (val2 & 0x80) && (val1 & 0x80) != (new_value & 0x80)
    }

    /// ADC: Add with carry
    ///
    /// This instruction adds the contents of a memory location to the
    /// accumulator together with the carry bit. If overflow occurs the carry
    /// bit is set, this enables multiple byte addition to be performed.
    fn adc(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let old_value = self.context.A;

        // A + M + C -> A
        self.context.A += value + self.context.get_carry() as u8;

        let overflow = self.is_sign_overflow(old_value, value, self.context.A);

        self.context.set_overflow(overflow);
        self.context.set_carry(old_value > self.context.A);
        self.calculate_alu_flag(self.context.A);
    }

    /// AND: Logical AND
    ///
    /// A logical AND is performed, bit by bit, on the accumulator contents
    /// using the contents of a byte of memory.
    fn and(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);

        self.context.A &= value;

        self.calculate_alu_flag(self.context.A);
    }

    /// ASL: Arithmetic Shift Left
    ///
    ///
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

    /// CMP: Compare
    ///
    /// This instruction compares the contents of the accumulator with another
    /// memory held value and sets the zero and carry flags as appropriate.
    fn cmp(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let diff = self.context.A - value;

        self.context.set_carry(self.context.A >= value);
        self.context.set_zero(diff == 0);
        self.context.set_negative(diff & 0x80 != 0);
    }

    // CPX: Compare X Register
    fn cpx(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let diff = self.context.X - value;

        self.context.set_carry(self.context.X >= value);
        self.context.set_zero(diff == 0);
        self.context.set_negative(diff & 0x80 != 0);
    }

    // CPY: Compare Y Register
    fn cpy(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let diff = self.context.Y - value;

        self.context.set_carry(self.context.Y >= value);
        self.context.set_zero(diff == 0);
        self.context.set_negative(diff & 0x80 != 0);
    }

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

    /// EOR: Exclusive OR
    ///
    /// An exclusive OR is performed, bit by bit, on the accumulator contents
    /// using the contents of a byte of memory.
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

    /// LSR: Logical Shift Right
    ///
    /// Each of the bits in A or M is shift one place to the right. The bit that
    /// was in bit 0 is shifted into the carry flag. Bit 7 is set to zero.
    fn lsr(&mut self, mode: AddrMode) {
        let operand = self.decode_operand(mode);
        let value = self.read_operand(&operand);
        let new_value = value >> 1;

        self.write_operand(&operand, new_value);

        // Update Flags
        self.context.set_carry((value & 0x01) != 0);
        self.calculate_alu_flag(new_value);
    }

    /// NOP: No Operation
    ///
    /// The NOP instruction causes no changes to the processor other than the
    /// normal incrementing of the program counter to the next instruction.
    fn nop(&mut self, mode: AddrMode) {}

    /// ORA: Logical Inclusive OR
    ///
    /// An inclusive OR is performed, bit by bit, on the accumulator contents
    /// using the contents of a byte of memory.
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
        // http://wiki.nesdev.com/w/index.php/CPU_status_flag_behavior
        self.push_byte(self.context.P | 0x30);
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

        // http://wiki.nesdev.com/w/index.php/CPU_status_flag_behavior
        // Bit 5 and 4 are ignored when pulled from stack - which means they are preserved
        let status = (status & 0xef) | (self.context.P & 0x10) | 0x20;
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
        let new_value = (value << 1) | self.context.get_carry() as u8;

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
        let new_value = (value >> 1) | ((self.context.get_carry() as u8) << 7);

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
    fn rti(&mut self, mode: AddrMode) {
        self.plp(AddrMode::Imp);

        let addr = self.pop_word();
        self.context.PC = addr;
    }

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
        let value = !value + 1;
        let value = value - (1 - self.context.get_carry() as u8);
        let old_value = self.context.A;
        self.context.A += value;

        let overflow = self.is_sign_overflow(old_value, value, self.context.A);

        self.context.set_overflow(overflow);
        self.context.set_carry(self.context.A <= old_value);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lda_sta_add() {
        let mut cpu = Cpu::new();

        cpu.run_program(
            vec![
                0xa9, 0x10, // lda 0x10
                0x85, 0x20, // sta (0x20)
                0xa9, 0x01, // lda 0x01
                0x65, 0x20, // add (0x20)
                0x85, 0x21, // sta (0x21)
                0xe6, 0x21, // inc (0x21)
                0xa4, 0x21, // ldy (0x21)
                0xc8, // iny
                0x00, // brk
            ],
            0x1000,
        );

        assert_eq!(cpu.peek(&0x20), 0x10);
        assert_eq!(cpu.peek(&0x21), 0x12);
        assert_eq!(cpu.context.A, 0x11);
        assert_eq!(cpu.context.Y, 0x13);
    }
}
