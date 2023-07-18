#![allow(clippy::upper_case_acronyms)]

use std::{cell::RefCell, rc::Rc};

use crate::{AddressingMode, Bus, Instruction};

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

/// Increments the given expression with `wrapping_add` and stores the result.
///
/// Optional second argument specifies the amount to add.
macro_rules! inc {
    ($var:expr) => {{
        $var = $var.wrapping_add(1);
    }};
    ($var:expr, $x:expr) => {{
        $var = $var.wrapping_add($x);
    }};
}

/// Decrements the given expression with `wrapping_sub` and stores the result.
///
/// Optional second argument specifies the amount to subtract.
macro_rules! dec {
    ($var:expr) => {{
        $var = $var.wrapping_sub(1);
    }};
    ($var:expr, $x:expr) => {{
        $var = $var.wrapping_sub($x);
    }};
}

/// Implements common bodies of instructions.
macro_rules! impl_inst {
    ($self:ident, start) => {
        if !($self.resolve()) {
            return;
        }
    };
    ($self:ident, end) => {
        $self.finish();
    };
}

/// Implements a branch instruction method (`BCC`, `BCS`, `BEQ`, etc.).
macro_rules! impl_branch {
    (@ $self:ident, $flag:ident) => {
        $self.get_flag(StatusFlag::$flag)
    };
    (@ $self:ident, ! $flag:ident) => {
        !$self.get_flag(StatusFlag::$flag)
    };
    ($name:ident, $($tt:tt)*) => {
        fn $name(&mut self) {
            impl_inst!(self, start);
            match self.state {
                State::T2 => {
                    if impl_branch!(@ self, $($tt)*) {
                        self.state = State::T3;
                    } else {
                        self.finish();
                    }
                }
                State::T3 | State::T0 => {
                    self.finish();
                }
                _ => unreachable!(),
            }
        }
    };
}

/// Implements a clear or set flag instruction method (`CLC`, `SEC`, `CLI`, etc.).
macro_rules! impl_flag {
    ($name:ident, $flag:ident, $on:literal) => {
        fn $name(&mut self) {
            impl_inst!(self, start);
            self.set_flag(StatusFlag::$flag, $on);
            impl_inst!(self, end);
        }
    };
}

/// Implements a load instruction method (`LDA`, `LDX`, and `LDY`).
macro_rules! impl_ld {
    ($name:ident, $reg:ident) => {
        fn $name(&mut self) {
            impl_inst!(self, start);
            self.$reg = self.data;
            self.set_flag_zn(self.$reg);
            impl_inst!(self, end);
        }
    };
}

/// Implements a store instruction body (`STA`, `STX`, and `STY`).
macro_rules! impl_st {
    ($name:ident, $reg:ident) => {
        fn $name(&mut self) {
            impl_inst!(self, start);
            self.write_u8(self.addr, self.$reg);
            impl_inst!(self, end);
        }
    };
}

/// Implements a transfer instruction body (`TAX`, `TAY`, `TSX`, etc.).
macro_rules! impl_xfer {
    ($name:ident, $to:ident, $from:ident) => {
        fn $name(&mut self) {
            impl_inst!(self, start);
            self.$to = self.$from;
            self.set_flag_zn(self.$to);
            impl_inst!(self, end);
        }
    };
    ($name:ident, $to:ident, $from:ident, no_flag) => {
        fn $name(&mut self) {
            impl_inst!(self, start);
            self.$to = self.$from;
            impl_inst!(self, end);
        }
    };
}

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

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// No interrupt
    #[default]
    None,
    /// Interrupt request
    IRQ,
    /// Non-maskable interrupt
    NMI,
    /// Physical reset pin
    RES,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
enum State {
    T0,
    T0_2,
    #[default]
    T1,
    T2,
    T3,
    T4,
    T5,
    T6,
}

impl State {
    /// Advances to the next [`State`].
    ///
    /// Does not work for `T1` -> `T2_0`.
    fn next(&mut self) {
        *self = match *self {
            Self::T0 => Self::T1,
            Self::T0_2 => Self::T1,
            Self::T1 => Self::T2,
            Self::T2 => Self::T3,
            Self::T3 => Self::T4,
            Self::T4 => Self::T5,
            Self::T5 => Self::T6,
            Self::T6 => Self::T0,
        };
    }

    /// Sets the state to `T0`.
    fn t0(&mut self) {
        *self = Self::T0;
    }

    /// Sets the state to `T0_2`.
    fn t0_2(&mut self) {
        *self = Self::T0_2;
    }

    /// Sets the state to `T1`.
    fn t1(&mut self) {
        *self = Self::T1;
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
enum Phase {
    #[default]
    Read,
    Modify,
    Write,
}

impl Phase {
    fn next(&mut self) {
        *self = match *self {
            Phase::Read => Phase::Modify,
            Phase::Modify => Phase::Write,
            Phase::Write => Phase::Read,
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Interrupt {
    /// Where the interrupt was sourced from
    src: Source,
    /// Whether or not the interrupt has been acknowledged.
    ack: bool,
}

impl Interrupt {
    fn new(src: Source) -> Self {
        Self { src, ack: false }
    }

    fn clear(&mut self) {
        self.src = Source::None;
        self.ack = false;
    }
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
    /// The instruction register (`IR`).
    ///
    /// This value is updated *after* the instruction is decoded on the first
    /// cycle (e.g., when SYNC is high).
    ir: Instruction,
    /// Represents the CPU execution state.
    ///
    /// Used to coordinate between cycles what actions need to be done.
    state: State,
    /// Represents the instruction phase.
    ///
    /// Used by read-modify-write instructions to coordinate between cycles.
    phase: Phase,
    /// Combination of the ADL/ADH lines.
    addr: u16,
    /// Used between cycles to check if address resolution resulted in a carry.
    addr_carry: bool,
    /// Emulates the data bus (`db`), which would hold data to read or write.
    ///
    /// This is generally abstracted away since the emulation does not use a
    /// RW signal to communicate with the bus. Instead this field is used to
    /// hold data between cycles.
    data: u8,
    /// Current CPU interrupt status.
    interrupt: Interrupt,
    /// Total cycles elapsed during the current program execution.
    cycles: u64,
}

impl<B: Bus> Cpu<B> {
    /// Creates a new [`Cpu`] connected to the given [`Bus`].
    pub fn new(bus: Shared<B>) -> Self {
        Self {
            pc: Default::default(),
            s: Default::default(),
            a: Default::default(),
            x: Default::default(),
            y: Default::default(),
            p: StatusFlag::_U as u8,
            bus,
            ir: Default::default(),
            state: State::default(),
            phase: Default::default(),
            addr: Default::default(),
            addr_carry: Default::default(),
            data: Default::default(),
            interrupt: Interrupt::new(Source::RES),
            cycles: Default::default(),
        }
    }

    /// Returns the `PC` address.
    pub fn pc(&self) -> u16 {
        self.pc
    }

    /// Returns the contents of the processor status (`P`) register.
    pub fn p(&self) -> u8 {
        self.p
    }

    /// Returns the contents of the accumulator (`A`) register.
    pub fn a(&self) -> u8 {
        self.a
    }

    /// Returns the contents of the X-index (`X`) register.
    pub fn x(&self) -> u8 {
        self.x
    }

    /// Returns the contents of the Y-index (`Y`) register.
    pub fn y(&self) -> u8 {
        self.y
    }

    /// Returns the total number of CPU cycles executed since the last reset.
    pub fn cycles(&self) -> u64 {
        self.cycles
    }

    /// Emulates the SYNC pin by checking the execution state of the CPU.
    ///
    /// Returns `true` if an instruction will start on the current cycle,
    /// `false` otherwise.
    pub fn is_sync(&self) -> bool {
        self.state == State::T1
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

    /// Sets the negative flag based on the given value.
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
        // check for interrupts that can be handled
        if self.interrupt.src != Source::None {
            self.handle_interrupt();
        }

        // SYNC state indicates instruction is ready to start
        if self.state == State::T1 {
            // decode instruction, don't increment PC yet
            self.ir = self.read_u8(self.pc).into();

            // if the handler acknowledged the interrupt, overwrite the IR
            if self.interrupt.ack {
                self.ir = Instruction::from(0);
            }
        }

        inc!(self.cycles);

        self.execute();
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
        loop {
            self.clock();

            if self.state == State::T1 {
                break;
            }
        }
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
    #[inline(always)]
    fn read_u8(&self, address: u16) -> u8 {
        self.bus.borrow().read(address)
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
        dec!(self.s);
    }

    fn handle_interrupt(&mut self) {
        if self.interrupt.src == Source::None {
            self.interrupt.clear();
            return;
        }

        // interrupts can only be handled on SYNC
        if self.state == State::T1
            && ((self.interrupt.src == Source::IRQ && !self.get_flag(StatusFlag::I))
                || self.interrupt.src == Source::NMI
                || self.interrupt.src == Source::RES)
        {
            self.interrupt.ack = true;

            // clear cycles on a reset
            if self.interrupt.src == Source::RES {
                self.cycles = 0;
            }
        }
    }

    /// Resolves the current operand based on the [`AddressingMode`].
    fn resolve(&mut self) -> bool {
        use AddressingMode::*;

        match self.ir.mode {
            Accumulator | Implied => self.imp(),
            Absolute => self.abs(),
            AbsoluteX | AbsoluteY => self.abs_xy(),
            Immediate => self.imm(),
            IndirectX => self.izx(),
            IndirectY => self.izy(),
            Relative => self.rel(),
            ZeroPage => self.zpg(),
            ZeroPageX | ZeroPageY => self.zpg_xy(),
            _ => unreachable!(),
        }
    }

    /// Resolves the operand for `absolute` addressing mode.
    ///
    /// Returns `true` when resolution is complete, `false` otherwise.
    fn abs(&mut self) -> bool {
        debug_assert!(matches!(
            self.ir.mode,
            AddressingMode::Absolute | AddressingMode::AbsoluteX | AddressingMode::AbsoluteY
        ));

        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // fetch low byte of address, increment PC
            State::T2 => {
                self.addr = self.read_u8(self.pc) as u16;
                inc!(self.pc);
                self.state.next();
            }
            // fetch high byte of address, increment PC
            State::T3 => {
                self.addr |= (self.read_u8(self.pc) as u16) << 8;
                self.data = self.read_u8(self.addr);
                inc!(self.pc);

                if self.ir.is_rmw() {
                    self.state.next();
                } else {
                    self.state.t0();
                }
            }
            // fix high byte of address, handle T4 of RMW
            State::T4 => {
                if self.addr_carry {
                    inc!(self.addr, 0x100);
                    self.addr_carry = false;
                    self.data = self.read_u8(self.addr);

                    if !self.ir.is_rmw() {
                        self.state.t0();
                        return false;
                    }
                }

                // RMW abs can read in T4
                self.data = self.read_u8(self.addr);
                self.state.next();
                self.phase.next();
            }
            // RMW T5, modify data
            State::T5 => {
                debug_assert!(self.phase == Phase::Modify);
                return true;
            }
            State::T0 => return true,
            _ => unreachable!(),
        }

        false
    }

    fn abs_xy(&mut self) -> bool {
        debug_assert!(
            self.ir.mode == AddressingMode::AbsoluteX || self.ir.mode == AddressingMode::AbsoluteY
        );

        match self.state {
            State::T3 => {
                self.addr |= (self.read_u8(self.pc) as u16) << 8;
                inc!(self.pc);

                // add index register to low address byte
                let hi = self.addr & 0xFF00;
                let lo = if self.ir.mode == AddressingMode::AbsoluteX {
                    (self.addr & 0x00FF) + self.x as u16
                } else {
                    (self.addr & 0x00FF) + self.y as u16
                };
                self.addr_carry = (lo & 0xFF00) != 0;
                self.addr = hi | (lo & 0x00FF);

                // check if additional cycle required for offset carry
                // (RMW always requires an additional cycle)
                if self.addr_carry || self.ir.is_rmw() {
                    self.state.next();
                } else {
                    self.state.t0();
                }
            }
            State::T4 => {
                // R/W instructions can finish here with carry
                if self.addr_carry {
                    inc!(self.addr, 0x100);
                    self.addr_carry = false;
                    self.data = self.read_u8(self.addr);

                    if !self.ir.is_rmw() {
                        self.state.t0();
                        return false;
                    }
                }

                // RMW instructions require T5 to read
                self.state.next();
            }
            // RMW T5, read data
            State::T5 => {
                debug_assert!(self.phase == Phase::Read);
                self.data = self.read_u8(self.addr);
                self.phase.next();
                self.state.next();
            }
            // RMW T6, modify data
            State::T6 => {
                debug_assert!(self.phase == Phase::Modify);
                return true;
            }
            _ => return self.abs(),
        }

        false
    }

    /// Resolves the operand for `immediate` addressing mode.
    ///
    /// Returns `true` when resolution is complete, `false` otherwise.
    fn imm(&mut self) -> bool {
        match self.state {
            State::T1 => {
                inc!(self.pc);
                self.state.t0_2();
            }
            State::T0_2 => {
                self.data = self.read_u8(self.pc);
                inc!(self.pc);
                return true;
            }
            _ => unreachable!(),
        }

        false
    }

    /// Resolves the operand for `implied` and `accumulator` addressing modes.
    ///
    /// Returns `true` when resolution is complete, `false` otherwise.
    fn imp(&mut self) -> bool {
        debug_assert!(
            self.ir.mode == AddressingMode::Accumulator || self.ir.mode == AddressingMode::Implied
        );

        match self.state {
            State::T1 => {
                inc!(self.pc);
                self.state.t0_2();
            }
            State::T0_2 => {
                if self.ir.mode == AddressingMode::Accumulator {
                    self.data = self.a;
                } else {
                    self.data = self.read_u8(self.pc);
                }
                return true;
            }
            _ => unreachable!(),
        }

        false
    }

    /// Resolves the operand for `(indirect,X)` addressing mode.
    ///
    /// Returns `true` when resolution is complete, `false` otherwise.
    fn izx(&mut self) -> bool {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // fetch pointer address, increment PC
            State::T2 => {
                self.data = self.read_u8(self.pc);
                inc!(self.pc);
                self.state.next();
            }
            // add X to get effective address
            State::T3 => {
                self.data = self.data.wrapping_add(self.x);
                self.state.next();
            }
            // fetch effective address low
            State::T4 => {
                let ptr = self.data as u16;
                self.addr = self.read_u8(ptr) as u16;
                self.state.next();
            }
            // fetch effective address high
            State::T5 => {
                let ptr = self.data.wrapping_add(1) as u16;
                let hi = self.read_u8(ptr) as u16;

                self.addr |= hi << 8;
                self.data = self.read_u8(self.addr);

                self.state.t0();
            }
            State::T0 => return true,
            _ => unreachable!(),
        }

        false
    }

    /// Resolves the operand for `(indirect),Y` addressing mode.
    ///
    /// Returns `true` when resolution is complete, `false` otherwise.
    fn izy(&mut self) -> bool {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // fetch pointer address, increment PC
            State::T2 => {
                self.data = self.read_u8(self.pc);
                inc!(self.pc);
                self.state.next();
            }
            // fetch effective address low
            State::T3 => {
                let ptr = self.data as u16;
                self.addr = self.read_u8(ptr) as u16;

                if ptr == 0xFF {
                    self.addr_carry = true;
                }

                self.state.next();
            }
            // fetch effective address high
            State::T4 => {
                let ptr = self.data.wrapping_add(1) as u16;
                let hi = self.read_u8(ptr) as u16;
                self.addr |= hi << 8;

                if self.addr_carry {
                    self.state.next();
                    return false;
                }

                self.state.t0();
            }
            // fix high byte of effective address
            State::T5 => {
                let ptr = (self.data as u16) + 1;
                let hi = self.read_u8(ptr) as u16;

                self.addr = (hi << 8) | (self.addr & 0x00FF);
                self.data = self.read_u8(self.addr);

                self.addr_carry = false;
                self.state.t0();
            }
            State::T0 => return true,
            _ => unreachable!(),
        }

        false
    }

    /// Resolves the operand for `relative` addressing mode.
    ///
    /// Note that only branch instructions use this mode.
    ///
    /// Returns `true` when resolution is complete, `false` otherwise.
    fn rel(&mut self) -> bool {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // fetch operand, increment PC
            State::T2 => {
                self.data = self.read_u8(self.pc);
                inc!(self.pc);

                // instruction should check here if branch is taken
                // and advance the state accordingly
                return true;
            }
            // if branch is taken, add operand to PCL
            State::T3 => {
                let offset = self.data as u16;
                let lo = (self.pc & 0x00FF) + offset;

                self.addr_carry = lo & 0xFF00 != 0 || lo & 0x80 != 0;
                self.pc = (self.pc & 0xFF00) | (lo & 0x00FF);

                if self.addr_carry {
                    self.state.t0();
                    return false;
                }

                return true;
            }
            // fix PCH
            State::T0 => {
                debug_assert!(self.addr_carry);
                self.addr_carry = false;

                // if operand was negative, decrement to fix PCH
                // otherwise, increment to fix PCH
                if self.data & 0x80 != 0 {
                    dec!(self.pc, 0x100);
                } else {
                    inc!(self.pc, 0x100);
                }

                return true;
            }
            _ => unreachable!(),
        }

        false
    }

    /// Resolves the operand for all `zeropage` addressing modes.
    ///
    /// Returns `true` when resolution is complete, `false` otherwise.
    fn zpg(&mut self) -> bool {
        debug_assert!(matches!(
            self.ir.mode,
            AddressingMode::ZeroPage | AddressingMode::ZeroPageX | AddressingMode::ZeroPageY
        ));

        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // fetch address, increment PC
            State::T2 => {
                self.addr = self.read_u8(self.pc) as u16;
                self.data = self.read_u8(self.addr);
                inc!(self.pc);

                if !self.ir.is_rmw() {
                    self.state.t0();
                    return false;
                }

                self.state.next();
            }
            State::T3 => {
                debug_assert!(self.phase == Phase::Read);
                self.data = self.read_u8(self.addr);
                self.phase.next();
                self.state.next();
            }
            State::T4 => {
                debug_assert!(self.phase == Phase::Modify);
                return true;
            }
            State::T0 => return true,
            _ => unreachable!(),
        }

        false
    }

    fn zpg_xy(&mut self) -> bool {
        debug_assert!(
            self.ir.mode == AddressingMode::ZeroPageX || self.ir.mode == AddressingMode::ZeroPageY
        );

        match self.state {
            // fetch address, increment PC
            State::T2 => {
                self.zpg();
                self.state = State::T3;
            }
            // add index register to effective address
            State::T3 => {
                if self.ir.mode == AddressingMode::ZeroPageX {
                    inc!(self.addr, self.x as u16);
                } else {
                    inc!(self.addr, self.y as u16);
                };
                self.addr &= 0x00FF;
                self.data = self.read_u8(self.addr);

                if !self.ir.is_rmw() {
                    self.state.t0();
                    return false;
                }

                self.state.next();
            }
            State::T4 => {
                debug_assert!(self.phase == Phase::Read);
                self.data = self.read_u8(self.addr);
                self.phase.next();
                self.state.next();
            }
            State::T5 => {
                debug_assert!(self.phase == Phase::Modify);
                return true;
            }
            _ => return self.zpg(),
        }

        false
    }

    /// Executes the current instruction in the `IR`.
    fn execute(&mut self) {
        use crate::Mnemonic::*;

        match self.ir.mnemonic {
            ADC => self.adc(),
            AND => self.and(),
            ASL => self.asl(),
            BCC => self.bcc(),
            BCS => self.bcs(),
            BEQ => self.beq(),
            BIT => self.bit(),
            BMI => self.bmi(),
            BNE => self.bne(),
            BPL => self.bpl(),
            BRK => self.brk(),
            BVC => self.bvc(),
            BVS => self.bvs(),
            CLC => self.clc(),
            CLD => self.cld(),
            CLI => self.cli(),
            CLV => self.clv(),
            CMP => self.cmp(),
            CPX => self.cpx(),
            CPY => self.cpy(),
            DEC => self.dec(),
            DEX => self.dex(),
            DEY => self.dey(),
            EOR => self.eor(),
            INC => self.inc(),
            INX => self.inx(),
            INY => self.iny(),
            JMP => self.jmp(),
            JSR => self.jsr(),
            LDA => self.lda(),
            LDX => self.ldx(),
            LDY => self.ldy(),
            LSR => self.lsr(),
            NOP => self.nop(),
            ORA => self.ora(),
            PHA => self.pha(),
            PHP => self.php(),
            PLA => self.pla(),
            PLP => self.plp(),
            ROL => self.rol(),
            ROR => self.ror(),
            RTI => self.rti(),
            RTS => self.rts(),
            SBC => self.sbc(),
            SEC => self.sec(),
            SED => self.sed(),
            SEI => self.sei(),
            STA => self.sta(),
            STX => self.stx(),
            STY => self.sty(),
            TAX => self.tax(),
            TAY => self.tay(),
            TSX => self.tsx(),
            TXA => self.txa(),
            TXS => self.txs(),
            TYA => self.tya(),
            ILLEGAL => self.illegal(),
        }
    }

    /// Executes the ADC instruction.
    fn adc(&mut self) {
        todo!()
    }

    /// Executes the AND instruction.
    fn and(&mut self) {
        if !self.resolve() {
            return;
        }

        self.a &= self.data;
        self.set_flag_zn(self.a);

        self.finish();
    }

    /// Executes the ASL instruction.
    fn asl(&mut self) {
        todo!()
    }

    impl_branch!(bcc, !C);
    impl_branch!(bcs, C);
    impl_branch!(beq, Z);
    impl_branch!(bmi, N);
    impl_branch!(bne, !Z);
    impl_branch!(bpl, !N);

    /// Executes the BIT instruction.
    fn bit(&mut self) {
        if !self.resolve() {
            return;
        }

        let res = self.a & self.data;
        self.set_flag(StatusFlag::N, res & (StatusFlag::N as u8) != 0);
        self.set_flag(StatusFlag::V, res & (StatusFlag::V as u8) != 0);
        self.set_flag_z(res);

        self.finish();
    }

    /// Executes the BRK instruction.
    fn brk(&mut self) {
        debug_assert!(self.ir.mode == AddressingMode::Implied);

        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // read next instruction byte (and throw it away), increment PC
            State::T2 => {
                inc!(self.pc);
                self.state.next();
            }
            // push PCH on stack
            State::T3 => {
                if self.interrupt.src == Source::RES {
                    dec!(self.s);
                } else {
                    self.push_u8(((self.pc & 0xFF00) >> 8) as u8);
                }

                self.state.next();
            }
            // push PCL on stack
            State::T4 => {
                if self.interrupt.src == Source::RES {
                    dec!(self.s);
                } else {
                    self.push_u8((self.pc & 0x00FF) as u8);
                }

                self.state.next();
            }
            // push P on stack
            State::T5 => {
                if self.interrupt.src == Source::RES {
                    dec!(self.s);
                } else {
                    self.push_u8(self.p | StatusFlag::_U as u8 | StatusFlag::B as u8);
                }

                self.set_flag(StatusFlag::I, true);
                self.state.next();
            }
            // fetch PCL
            State::T6 => {
                self.pc = match self.interrupt.src {
                    Source::IRQ => self.read_u8(IRQ_VECTOR) as u16,
                    Source::NMI => self.read_u8(NMI_VECTOR) as u16,
                    Source::RES => self.read_u8(RES_VECTOR) as u16,
                    _ => unreachable!(),
                };
                self.state.t0();
            }
            // fetch PCH
            State::T0 => {
                let hi = match self.interrupt.src {
                    Source::IRQ => self.read_u8(IRQ_VECTOR.wrapping_add(1)) as u16,
                    Source::NMI => self.read_u8(NMI_VECTOR.wrapping_add(1)) as u16,
                    Source::RES => self.read_u8(RES_VECTOR.wrapping_add(1)) as u16,
                    _ => unreachable!(),
                };
                self.pc |= hi << 8;
                self.state.t1();
                // TODO: fix when interrupt is cleared
                self.interrupt.clear();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the BVC instruction.
    fn bvc(&mut self) {
        todo!()
    }

    /// Executes the BVS instruction.
    fn bvs(&mut self) {
        todo!()
    }

    impl_flag!(clc, C, false);
    impl_flag!(cld, D, false);
    impl_flag!(cli, I, false);
    impl_flag!(clv, V, false);

    /// Executes the CMP instruction.
    fn cmp(&mut self) {
        todo!()
    }

    /// Executes the CPX instruction.
    fn cpx(&mut self) {
        todo!()
    }

    /// Executes the CPY instruction.
    fn cpy(&mut self) {
        todo!()
    }

    /// Executes the DEC instruction.
    fn dec(&mut self) {
        if !self.resolve() {
            return;
        }

        match self.phase {
            Phase::Modify => {
                dec!(self.data);
                self.phase.next();
                self.state.t0();
            }
            Phase::Write => {
                self.write_u8(self.addr, self.data);
                self.set_flag_zn(self.data);

                self.phase = Phase::Read;
                self.state.t1();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the DEX instruction.
    fn dex(&mut self) {
        if !self.resolve() {
            return;
        }

        dec!(self.x);
        self.set_flag_zn(self.x);

        self.phase = Phase::Read;
        self.state.t1();
    }

    /// Executes the DEY instruction.
    fn dey(&mut self) {
        if !self.resolve() {
            return;
        }

        dec!(self.y);
        self.set_flag_zn(self.y);

        self.phase = Phase::Read;
        self.state.t1();
    }

    /// Executes the EOR instruction.
    fn eor(&mut self) {
        if !self.resolve() {
            return;
        }

        self.a ^= self.data;
        self.set_flag_zn(self.a);
        self.finish();
    }

    /// Executes the INC instruction.
    fn inc(&mut self) {
        if !self.resolve() {
            return;
        }

        match self.phase {
            Phase::Modify => {
                inc!(self.data);
                self.phase.next();
                self.state.t0();
            }
            Phase::Write => {
                self.write_u8(self.addr, self.data);
                self.set_flag_zn(self.data);

                self.phase = Phase::Read;
                self.state.t1();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the INX instruction.
    fn inx(&mut self) {
        if !self.resolve() {
            return;
        }

        inc!(self.x);
        self.set_flag_zn(self.x);

        self.phase = Phase::Read;
        self.state.t1();
    }

    /// Executes the INY instruction.
    fn iny(&mut self) {
        if !self.resolve() {
            return;
        }

        inc!(self.y);
        self.set_flag_zn(self.y);

        self.phase = Phase::Read;
        self.state.t1();
    }

    /// Executes the JMP instruction.
    fn jmp(&mut self) {
        debug_assert!(
            self.ir.mode == AddressingMode::Absolute || self.ir.mode == AddressingMode::Indirect
        );

        match (self.ir.mode, self.state) {
            // fetch opcode, increment PC
            (AddressingMode::Absolute | AddressingMode::Indirect, State::T1) => {
                inc!(self.pc);
                self.state.next();
            }

            // fetch low address byte, increment PC
            (AddressingMode::Absolute, State::T2) => {
                self.addr = self.read_u8(self.pc) as u16;
                inc!(self.pc);
                self.state.t0();
            }
            // copy low address byte to PCL, fetch high address byte to PCH
            (AddressingMode::Absolute, State::T0) => {
                self.addr |= (self.read_u8(self.pc) as u16) << 8;
                self.pc = self.addr;
                self.state.t1();
            }

            // fetch pointer address low, increment PC
            (AddressingMode::Indirect, State::T2) => {
                self.addr = self.read_u8(self.pc) as u16;
                inc!(self.pc);
                self.state.next();
            }
            // fetch pointer address high, increment PC
            (AddressingMode::Indirect, State::T3) => {
                self.addr |= (self.read_u8(self.pc) as u16) << 8;
                inc!(self.pc);
                self.state.next();
            }
            // fetch low address to latch
            (AddressingMode::Indirect, State::T4) => {
                self.data = self.read_u8(self.addr);
                self.state.t0();
            }
            // fetch PCH, copy latch to PCL
            (AddressingMode::Indirect, State::T0) => {
                // emulate hardware bug on page cross
                let addr = (self.addr & 0xFF00) | (self.addr.wrapping_add(1) & 0x00FF);
                let hi = (self.read_u8(addr) as u16) << 8;
                let lo = self.data as u16;

                self.addr = hi | lo;
                self.pc = self.addr;
                self.state.t1();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the JSR instruction.
    fn jsr(&mut self) {
        todo!()
    }

    impl_ld!(lda, a);
    impl_ld!(ldx, x);
    impl_ld!(ldy, y);

    /// Executes the LSR instruction.
    fn lsr(&mut self) {
        todo!()
    }

    /// Executes the NOP instruction.
    #[inline(always)]
    fn nop(&mut self) {
        match self.state {
            State::T1 => {
                inc!(self.pc);
                self.state.t0_2();
            }
            State::T0_2 => self.state.t1(),
            _ => unreachable!(),
        }
    }

    /// Executes the ORA instruction.
    fn ora(&mut self) {
        if !self.resolve() {
            return;
        }

        self.a |= self.data;
        self.set_flag_zn(self.a);
        self.finish();
    }

    /// Executes the PHA instruction.
    fn pha(&mut self) {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // read next instruction byte (and throw it away)
            State::T2 => {
                self.state.t0();
            }
            // push register on stack, decrement S
            State::T0 => {
                self.push_u8(self.a);
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the PHP instruction.
    fn php(&mut self) {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // read next instruction byte (and throw it away)
            State::T2 => {
                self.state.t0();
            }
            // push register on stack, decrement S
            State::T0 => {
                self.push_u8(self.p | StatusFlag::_U as u8 | StatusFlag::B as u8);
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the PLA instruction.
    fn pla(&mut self) {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // read next instruction byte (and throw it away)
            State::T2 => {
                self.state.next();
            }
            // increment S
            State::T3 => {
                inc!(self.s);
                self.state.t0();
            }
            // pull register from stack
            State::T0 => {
                self.a = self.read_u8(self.sp_u16());
                self.set_flag_zn(self.a);
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the PLP instruction.
    fn plp(&mut self) {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => {
                inc!(self.pc);
                self.state.next();
            }
            // read next instruction byte (and throw it away)
            State::T2 => {
                self.state.next();
            }
            // increment S
            State::T3 => {
                inc!(self.s);
                self.state.t0();
            }
            // pull register from stack
            State::T0 => {
                // ignore bit 5 and B from stack
                let p = self.read_u8(self.sp_u16()) & 0b1100_1111;
                self.p = (self.p & 0b0011_0000) | p;
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the ROL instruction.
    fn rol(&mut self) {
        if !self.resolve() {
            return;
        }

        if self.state == State::T0_2 {
            self.a = rol_impl(self, self.a);
            self.finish();
            return;
        }

        match self.phase {
            Phase::Modify => {
                self.data = rol_impl(self, self.data);
                self.phase.next();
                self.state.t0();
            }
            Phase::Write => {
                self.write_u8(self.addr, self.data);
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the ROR instruction.
    fn ror(&mut self) {
        if !self.resolve() {
            return;
        }

        if self.state == State::T0_2 {
            self.a = ror_impl(self, self.a);
            self.finish();
            return;
        }

        match self.phase {
            Phase::Modify => {
                self.data = ror_impl(self, self.data);
                self.phase.next();
                self.state.t0();
            }
            Phase::Write => {
                self.write_u8(self.addr, self.data);
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    /// Executes the RTI instruction.
    fn rti(&mut self) {
        todo!()
    }

    /// Executes the RTS instruction.
    fn rts(&mut self) {
        todo!()
    }

    /// Executes the SBC instruction.
    fn sbc(&mut self) {
        todo!()
    }

    impl_flag!(sec, C, true);
    impl_flag!(sed, D, true);
    impl_flag!(sei, I, true);
    impl_st!(sta, a);
    impl_st!(stx, x);
    impl_st!(sty, y);
    impl_xfer!(tax, x, a);
    impl_xfer!(tay, y, a);
    impl_xfer!(tsx, x, s);
    impl_xfer!(txa, a, x);
    impl_xfer!(txs, s, x, no_flag);
    impl_xfer!(tya, a, y);

    fn illegal(&self) {
        panic!(
            "attempted to execute illegal instruction `0x{:02X}`",
            self.ir.opcode
        );
    }

    /// Finalizes instruction execution (reset states, phases, etc.).
    fn finish(&mut self) {
        self.phase = Phase::Read;
        self.state.t1();
    }
}

fn rol_impl<B: Bus>(cpu: &mut Cpu<B>, mut value: u8) -> u8 {
    let carry = if cpu.get_flag(StatusFlag::C) { 1 } else { 0 };
    cpu.set_flag(StatusFlag::C, value & 0x80 != 0);

    value = ((value & 0x7F) << 1) | carry;
    cpu.set_flag_zn(value);

    value
}

fn ror_impl<B: Bus>(cpu: &mut Cpu<B>, mut value: u8) -> u8 {
    let carry = if cpu.get_flag(StatusFlag::C) { 0x80 } else { 0 };
    cpu.set_flag(StatusFlag::C, value & 1 != 0);

    value = carry | ((value & 0x7F) >> 1);
    cpu.set_flag_zn(value);

    value
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug)]
    struct StaticBus(u8);

    impl Bus for StaticBus {
        fn read(&self, _: u16) -> u8 {
            self.0
        }

        fn write(&mut self, _: u16, _: u8) {}
    }

    fn cpu<B: Bus>(bus: B) -> Cpu<B> {
        Cpu::new(Rc::new(RefCell::new(bus)))
    }

    macro_rules! assert_word_eq {
        ($lhs:expr, $rhs:expr) => {{
            if $lhs != $rhs {
                panic!(
                    "failed byte assertion:\n    \
                    expr: `{}`\n  \
                    expect: `0x{:04X}`\n     \
                    got: `0x{:04X}`\n",
                    stringify!($lhs),
                    $rhs,
                    $lhs,
                );
            }
        }};
    }

    macro_rules! assert_byte_eq {
        ($lhs:expr, $rhs:expr) => {{
            if $lhs != $rhs {
                panic!(
                    "failed byte assertion:\n    \
                    expr: `{}`\n  \
                    expect: `0x{:02X}`\n     \
                    got: `0x{:02X}`\n",
                    stringify!($lhs),
                    $rhs,
                    $lhs,
                );
            }
        }};
    }

    #[test]
    fn init_reset() {
        let mut cpu = cpu(StaticBus(0xEA));
        cpu.step();

        assert_word_eq!(cpu.pc, 0xEAEA);
        assert_byte_eq!(cpu.s, 0xFD);
        assert_byte_eq!(cpu.p, 0x24);
        assert_byte_eq!(cpu.a, 0x00);
        assert_byte_eq!(cpu.x, 0x00);
        assert_byte_eq!(cpu.y, 0x00);
        assert_eq!(cpu.cycles, 7);
        assert_eq!(cpu.state, State::T1);
    }

    #[test]
    fn sp_addr() {
        let mut cpu = cpu(StaticBus(0x00));

        cpu.s = 0x00;
        for sp in 0..=512 {
            assert_word_eq!(cpu.sp_u16(), (sp & 0x00FF) | 0x0100);
            inc!(cpu.s);
        }
    }

    #[test]
    fn cycles_nop() {
        let mut cpu = cpu(StaticBus(0xEA));

        cpu.step_for(3); // reset + 2 NOP

        // 7 for reset + 4 for 2x NOP
        assert_eq!(cpu.cycles, 11);
    }

    #[test]
    fn cycles_lda_imm() {
        let mut cpu = cpu(StaticBus(0xA9));

        cpu.step_for(2); // reset + LDA #
        assert_byte_eq!(cpu.a, 0xA9);

        // 7 for reset + 2 for LDA #
        assert_eq!(cpu.cycles, 9);
    }

    #[test]
    fn cycles_lda_abs() {
        let mut cpu = cpu(StaticBus(0xAD));

        cpu.step_for(2); // reset + LDA abs
        assert_byte_eq!(cpu.a, 0xAD);

        // 7 for reset + 4 for LDA abs
        assert_eq!(cpu.cycles, 11);
    }

    #[test]
    #[should_panic]
    fn illegal_opcode_panics() {
        let mut cpu = cpu(StaticBus(0xFF));
        cpu.step_for(2); // reset + illegal
    }

    #[test]
    fn set_flags() {
        use StatusFlag::*;

        macro_rules! test_flag {
            ($cpu:ident, $flag:ident, $on:literal, reset = $reset:literal) => {{
                if $reset {
                    $cpu.p = 0;
                }
                $cpu.set_flag($flag, $on);
                assert_eq!(
                    $cpu.get_flag($flag),
                    $on,
                    "Failed to set flag `{:?}` ({:08b}) to `{}` (status flags: {:08b})",
                    $flag,
                    $flag as u8,
                    $on,
                    $cpu.p
                );
            }};
        }

        let mut cpu = cpu(StaticBus(0xEA));

        cpu.p = 0;
        test_flag!(cpu, N, true, reset = true);
        test_flag!(cpu, V, false, reset = true);
        test_flag!(cpu, _U, true, reset = true);
        test_flag!(cpu, B, true, reset = true);
        test_flag!(cpu, D, true, reset = true);
        test_flag!(cpu, I, false, reset = true);
        test_flag!(cpu, Z, true, reset = true);
        test_flag!(cpu, C, false, reset = true);

        cpu.p = 0;
        test_flag!(cpu, N, true, reset = false);
        test_flag!(cpu, V, true, reset = false);
        test_flag!(cpu, _U, true, reset = false);
        test_flag!(cpu, B, true, reset = false);
        test_flag!(cpu, D, true, reset = false);
        test_flag!(cpu, I, true, reset = false);
        test_flag!(cpu, Z, true, reset = false);
        test_flag!(cpu, C, true, reset = false);

        assert_byte_eq!(cpu.p, 0xFF);
    }
}
