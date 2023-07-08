use std::{cell::RefCell, rc::Rc};

use crate::{Bus, Instruction};

/// The non-maskable interrupt vector.
///
/// Used by the system to read the 16-bit address of the NMI subroutine.
const NMI_VECTOR: u16 = 0xFFFA;

/// The reset vector.
///
/// Used by the system to read the 16-bit starting address after a physical
/// reset has been triggered (e.g., `RES` pin brought low for two cycles).
const RES_VECTOR: u16 = 0xFFFC;

/// The request interrupt vector.
///
/// Used by the system to read the 16-bit address of the RQI/BRK handler.
const IRQ_VECTOR: u16 = 0xFFFE;

/// A type that allows shared access with interior mutability.
type Shared<T> = Rc<RefCell<T>>;

/// Represents a specific bit of the processor status register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StatusFlag {
    /// Carry flag
    C = (1 << 0),
    /// Zero flag
    Z = (1 << 1),
    /// Interrupt disable flag
    I = (1 << 2),
    /// Decimal flag
    D = (1 << 3),
    /// Break flag
    B = (1 << 4),
    /// Unused
    _U = (1 << 5),
    /// Overflow flag
    V = (1 << 6),
    /// Negative flag
    N = (1 << 7),
}

/// A cycle-accurate 6502 processor emulator, connected to a [`Bus`].
#[derive(Debug)]
pub struct Cpu<B: Bus> {
    /// The program counter (`PC`).
    pc: u16,
    /// The stack pointer (`S`).
    s: u8,
    /// The processor status register (`P`).
    p: u8,
    /// The accumulator (`A` register).
    a: u8,
    /// The `X` index register.
    x: u8,
    /// The `Y` index register.
    y: u8,
    /// Internal reference to the connected bus.
    bus: Shared<B>,
    /// Instruction register.
    ir: Instruction,
    /// Current operand for the instruction.
    operand: u16,
    /// The cycle of the current instruction.
    ///
    /// Used as an abstract for timing states, which are quite complex.
    icycle: u8,
    /// Total cycles elapsed during the current program execution.
    cycles: u64,
}

impl<B: Bus> Cpu<B> {
    /// Creates a new [`Cpu`] connected to the given [`Bus`].
    pub fn new(bus: Shared<B>) -> Self {
        let cpu = Self {
            pc: Default::default(),
            s: Default::default(),
            a: Default::default(),
            x: Default::default(),
            y: Default::default(),
            p: Default::default(),
            bus,
            ir: Default::default(),
            operand: Default::default(),
            icycle: Default::default(),
            cycles: Default::default(),
        };

        cpu
    }

    /// Returns whether or not the given processor status flag is on.
    pub fn get_flag(&self, flag: StatusFlag) -> bool {
        (self.p & (flag as u8)) > 0
    }

    /// Sets the specified processor status flag to given state
    pub fn set_flag(&mut self, flag: StatusFlag, on: bool) {
        if on {
            self.p |= flag as u8;
        } else {
            self.p &= !(flag as u8);
        }
    }

    /// Sets the zero flag based on the given value.
    ///
    /// This method checks if the value is `0`.
    #[inline(always)]
    pub fn set_flag_z(&mut self, value: u8) {
        self.set_flag(StatusFlag::Z, value == 0);
    }

    /// Sets the zero flag based on the given value.
    ///
    /// This method checks if the most significant bit is `1` (e.g.,
    /// `value & 0x80 != 0`).
    #[inline(always)]
    pub fn set_flag_n(&mut self, value: u8) {
        self.set_flag(StatusFlag::N, value & 0x80 != 0);
    }

    /// Convenience method to set the [`Z`][Z] and [`N`][N] flags based on the
    /// same input value.
    ///
    /// These flags are commonly set together.
    ///
    /// [Z]: StatusFlag::Z
    /// [N]: StatusFlag::N
    #[inline(always)]
    pub fn set_flag_zn(&mut self, value: u8) {
        self.set_flag_z(value);
        self.set_flag_n(value);
    }

    /// Returns the stack pointer as a properly offset 16-bit address.
    #[inline(always)]
    pub fn sp_u16(&self) -> u16 {
        0x0100 | (self.s as u16)
    }

    /// Executes exactly one clock cycle.
    pub fn clock(&mut self) {
        todo!()
    }

    /// Executes the specified number of clock cycles.
    pub fn clock_for(&mut self, mut count: u64) {
        while count > 0 {
            count -= 1;
            self.clock();
        }
    }

    /// Steps through all cycles required to complete a single instruction.
    pub fn step(&mut self) {
        todo!()
    }

    /// Steps through all cycles required to complete the given number of
    /// instructions.
    pub fn step_for(&mut self, mut count: u64) {
        while count > 0 {
            count -= 1;
            self.step();
        }
    }

    /// Reads a single byte from the bus at the specified address.
    ///
    /// Unlike `read` methods, this does not mutate internal state.
    #[inline(always)]
    fn get_u8(&self, address: u16) -> u8 {
        self.bus.borrow().read(address)
    }

    /// Reads a byte from the bus at the current program counter address and
    /// increments it.
    #[inline(always)]
    fn read_u8(&mut self) -> u8 {
        let byte = self.get_u8(self.pc);
        self.pc = self.pc.wrapping_add(1);

        byte
    }

    /// Writes a byte to the bus at the given address.
    #[inline(always)]
    fn write_u8(&mut self, address: u16, data: u8) {
        self.bus.borrow_mut().write(address, data)
    }

    /// Writes a byte to the stack and decrements the stack pointer.
    #[inline(always)]
    fn push_u8(&mut self, data: u8) {
        self.write_u8(self.sp_u16(), data);
        self.s = self.s.wrapping_sub(1);
    }

    /// Increments the stack pointer and reads the byte at that address.
    #[inline(always)]
    fn pop_u8(&mut self) -> u8 {
        self.s = self.s.wrapping_add(1);
        self.get_u8(self.sp_u16())
    }
}
