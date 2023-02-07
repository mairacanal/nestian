use std::panic;

use crate::memory::Memory;
use crate::context::Context;

const STACK_OFFSET : u16 = 0x0100;

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

enum OpCode {
    ORA = 0x00,
    AND = 0x20,
    EOR = 0x40,
    ADC = 0x60,
    STA = 0x80,
    LDA = 0xA0,
    CMP = 0xC0,
    SBC = 0xE0,
    // BRK = 0x00,
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
        self.context.S = 0xFD;

        // P = 0x34
        self.context.set_interrupt(true);
        self.context.set_B(true);
        self.context.set_I(true);
    }

    pub fn run(&mut self, addr: u16) {
        self.power_on();
        self.context.PC = addr;
        self.execute();
    }

    // Memory Management
    pub fn peek(&self, addr : u16) -> u8 { self.memory.get_byte(addr) }

    pub fn poke(&mut self, addr : u16, value : u8) { self.memory.set_byte(addr, value); }

    // Stack Management

    /// Stack grows from the top-down
    /// No underflow/overflow detection
    pub fn push_byte(&mut self, value : u8) {
        self.memory.set_byte(self.context.S as u16 + STACK_OFFSET, value);
        self.context.S -= 1;
    }

    /// High-order bytes push first since the stack grows
    /// top-down and the CPU is little-endian
    pub fn push_word(&mut self, value : u16) {
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
        let lsb : u16 = self.pop_byte() as u16;
        let msb : u16 = self.pop_byte() as u16;

        (msb << 8) + lsb
    }

    // Execution
    fn execute(&self) {

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

    fn cycle(&mut self, count: u8) { self.cycle += count as u32; }

    fn decode_operand(&mut self, mode : AddrMode) -> Operand {
        match mode {
            AddrMode::acc => Operand { value: 0, kind: OperandKind::acc },
            AddrMode::imm => Operand { value: self.decode_byte() as u16, kind: OperandKind::imm },
            _ => Operand { value : self.decode_operand_addr(mode), kind : OperandKind::addr }
        }
    }

    fn read_operand(&self, op : Operand) -> u8 {
        match op.kind {
            OperandKind::acc => self.context.A,
            OperandKind::imm => op.value as u8,
            OperandKind::addr => self.peek(op.value),
        }
    }

    fn write_operand(&mut self, op : Operand, value : u8) {
        match op.kind {
            OperandKind::acc => self.context.A = value,
            OperandKind::addr => self.poke(op.value, value),
            _ => ()
        }
    }

    fn decode_operand_addr(&mut self, mode : AddrMode) -> u16 {
        match mode {
            AddrMode::zp => self.decode_byte() as u16,
            AddrMode::zp_ind_x => (self.decode_byte() + self.context.X) as u16,
            AddrMode::zp_ind_y => (self.decode_byte() + self.context.Y) as u16,
            AddrMode::ind_jmp => {
                let word = self.decode_word();
                self.memory.get_word(word)
            },
            AddrMode::abs => self.decode_word(),
            AddrMode::abs_x => self.decode_word() + self.context.X as u16,
            AddrMode::abs_y => self.decode_word() + self.context.Y as u16,
            AddrMode::ind_x => {
                let word = self.decode_byte() as u16;
                let addr = self.peek(word);
                self.peek((addr + self.context.X) as u16) as u16 +
                    (self.peek((addr + self.context.X + 1) as u16) as u16) << 8
            }
            AddrMode::ind_y => {
                let addr = self.decode_byte() as u16;
                self.peek(addr) as u16 + (self.peek(addr + 1) as u16) << 8 + self.context.Y as u16
            }
            _ => {
                panic!("Not implemented yet");
            }
        }
    }

    // CPU instructions
    fn calculate_alu_flag(&mut self, value : u8) {
        self.context.set_zero(value == 0);
        self.context.set_negative((value & 0x80) != 0);
    }

    fn determine_overflow_flag(&mut self, old_value: u8, new_value: u8) {
        self.context.set_overflow((old_value & 0x80) != (new_value & 0x80));
    }

    // ADC: Add with carry
    fn ADC(mode : AddrMode) {

    }

    // AND: Logical AND
    fn AND(mode : AddrMode) {

    }

    // ASL: Arithmetic Shift Left
    fn ASL(mode : AddrMode) {

    }

    // BCC: Branch if Carry Clean
    fn BCC(mode : AddrMode) {

    }

    // BCS: Branch if Carry Set
    fn BCS(mode : AddrMode) {

    }

    // BEQ: Branch if Equal
    fn BEQ(mode : AddrMode) {

    }

    // BIT: Test bit
    fn BIT(mode : AddrMode) {

    }

    // BMI: Branch if Minus
    fn BMI(mode : AddrMode) {

    }

    // BNE: Branch if Not Equal
    fn BNE(mode : AddrMode) {

    }

    // BPL: Branch if Positive
    fn BPL(mode : AddrMode) {

    }

    // BRK: Break
    fn BRK(mode : AddrMode) {

    }

    // BVC: Branch if Overflow Clear
    fn BVC(mode : AddrMode) {

    }

    // BVS: Branch if Overflow Set
    fn BVS(mode : AddrMode) {

    }

    // CLC: Clear Carry Flag
    fn CLC(mode : AddrMode) {

    }

    // CLD: Clear Decimal Flag
    fn CLD(mode : AddrMode) {

    }

    // CLI: Clear Interrupt Flag
    fn CLI(mode : AddrMode) {

    }
}

enum OperandKind {
    acc,
    imm,
    addr,
}

struct Operand {
    value: u16,
    kind: OperandKind,
}
