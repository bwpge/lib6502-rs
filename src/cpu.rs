#![allow(clippy::upper_case_acronyms)]

use std::{cell::RefCell, rc::Rc};

use crate::{AddressingMode, Bus, Instruction};

const NMI_VECTOR: u16 = 0xFFFA;
const RES_VECTOR: u16 = 0xFFFC;
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

/// Shorthand to increment the PC and advance the CPU state.
///
/// Optional second argument specifies the state to advance to.
macro_rules! advance {
    ($self:ident) => {{
        inc!($self.pc);
        $self.state.next();
    }};
    ($self:ident, $state:ident) => {{
        inc!($self.pc);
        $self.state = State::$state;
    }};
}

/// Implements the CPU execution method body.
macro_rules! exec_inst {
    ($self:ident, $($variant:ident, $func:ident),* $(,)?) => {{
        match $self.ir.mnemonic {
            $(
                $crate::Mnemonic::$variant => $self.$func(),
            )*
            _ => panic!(
                "attempted to execute illegal instruction '{}' `0x{:02X}`",
                $self.ir.mnemonic, $self.ir.opcode
            ),
        }
    }};
}

/// Implements instruction methods.
///
/// Syntax for this macro body is any repetition of `TYPE => (name, args...),`.
macro_rules! impl_inst {
    ($($tt:tt => ( $name:ident, $($args:tt),* )),* $(,)?) => {
        $(
            fn $name(&mut self) {
                if !(self.resolve()) {
                    return;
                }
                impl_inst_inner!($tt, self, $($args),*);
                self.finish();
            }
        )*
    };
}

/// Implements the instruction method body with the given arguments from `impl_inst`.
macro_rules! impl_inst_inner {
    (BRANCH, $self:ident, $flag:ident, $set:literal) => {
        match $self.state {
            State::T2 => {
                if $self.get_flag(StatusFlag::$flag) == $set {
                    $self.state = State::T3;
                    return;
                }
            }
            State::T3 | State::T0 => (),
            _ => unreachable!(),
        }
    };
    (COMPARE, $self:ident, $reg:ident) => {
        $self.set_flag(StatusFlag::C, $self.$reg >= $self.data);
        $self.set_flag(StatusFlag::Z, $self.$reg == $self.data);
        $self.set_flag(StatusFlag::N, $self.$reg >= 0x80);
    };
    (DECREG, $self:ident, $reg:ident) => {
        dec!($self.$reg);
        $self.set_flag_zn($self.$reg);
    };
    (FLAG, $self:ident, $flag:ident, $on:literal) => {
        $self.set_flag(StatusFlag::$flag, $on);
    };
    (INCDEC, $self:ident, $op:tt) => {
        match $self.phase {
            Phase::Modify => {
                $op!($self.data);
                $self.phase.next();
                $self.state.t0();
                return;
            }
            Phase::Write => {
                $self.write_u8($self.addr, $self.data);
                $self.set_flag_zn($self.data);
            }
            _ => unreachable!(),
        }
    };
    (INCREG, $self:ident, $reg:ident) => {
        inc!($self.$reg);
        $self.set_flag_zn($self.$reg);
    };
    (LOAD, $self:ident, $reg:ident) => {
        $self.$reg = $self.data;
        $self.set_flag_zn($self.$reg);
    };
    (LOGICAL, $self:ident, $op:tt) => {
        $self.a $op $self.data;
        $self.set_flag_zn($self.a);
    };
    (SHIFT_ROTATE, $self:ident, $func:ident) => {
        if $self.state == State::T0_2 {
            $self.a = $func($self, $self.a);
            $self.finish();
            return;
        }
        match $self.phase {
            Phase::Modify => {
                $self.data = $func($self, $self.data);
                $self.phase.next();
                $self.state.t0();
                return;
            }
            Phase::Write => $self.write_u8($self.addr, $self.data),
            _ => unreachable!(),
        }
    };
    (STORE, $self:ident, $reg:ident) => {
        $self.write_u8($self.addr, $self.$reg);
    };
    (TRANSFER, $self:ident, $to:ident, $from:ident) => {
        $self.$to = $self.$from;
        $self.set_flag_zn($self.$to);
    };
    (TRANSFER, $self:ident, $to:ident, $from:ident, NO_FLAGS) => {
        $self.$to = $self.$from;
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
    /// Software interrupt, similar to IRQ
    BRK,
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
    /// Note that no state advances to `T2_0` with this method.
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
            state: Default::default(),
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
    #[inline(always)]
    pub fn set_flag_z(&mut self, value: u8) {
        self.set_flag(StatusFlag::Z, value == 0);
    }

    /// Sets the negative flag based on the given value by checking if the most
    /// significant bit is `1`.
    #[inline(always)]
    pub fn set_flag_n(&mut self, value: u8) {
        self.set_flag(StatusFlag::N, value & 0x80 != 0);
    }

    /// Shorthand to set the [`Z`][Z] and [`N`][N] flags based on the same input
    /// value, as these flags are commonly set together.
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
            self.check_interrupt();
        }

        // SYNC state indicates instruction is ready to start
        if self.state == State::T1 {
            // decode instruction, don't increment PC yet
            self.ir = self.read_u8(self.pc).into();

            // annotate BRK opcodes -- these don't need to be acknowledged
            // since they are the active instruction
            if self.ir.opcode == 0x00 && self.interrupt.src == Source::None {
                self.interrupt.src = Source::BRK;
            }
            // if the handler acknowledged the interrupt, overwrite the IR
            else if self.interrupt.ack {
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

    /// Checks if the current [`Interrupt`] can be handled in this clock cycle.
    fn check_interrupt(&mut self) {
        debug_assert!(self.interrupt.src != Source::None);

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
            State::T1 => advance!(self),
            // fetch low byte of address, increment PC
            State::T2 => {
                self.addr = self.read_u8(self.pc) as u16;
                advance!(self);
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
                self.data = self.read_u8(self.addr);

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
            State::T1 => advance!(self, T0_2),
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
            State::T1 => advance!(self, T0_2),
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
            State::T1 => advance!(self),
            // fetch pointer address, increment PC
            State::T2 => {
                self.data = self.read_u8(self.pc);
                advance!(self);
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
            State::T1 => advance!(self),
            // fetch pointer address, increment PC
            State::T2 => {
                self.data = self.read_u8(self.pc);
                advance!(self);
            }
            // fetch effective address low
            State::T3 => {
                let ptr = self.data as u16;
                self.addr = self.read_u8(ptr) as u16;

                self.state.next();
            }
            // fetch effective address high, add Y to effective address
            State::T4 => {
                let ptr = self.data.wrapping_add(1) as u16;
                let hi = self.read_u8(ptr) as u16;
                self.addr |= hi << 8;

                // check if y index caused a page cross
                let lo = (self.addr & 0x00FF).wrapping_add(self.y as u16);
                self.addr_carry = lo & 0xFF00 != 0;

                self.addr = (self.addr & 0xFF00) | (lo & 0x00FF);
                self.data = self.read_u8(self.addr);
                if self.addr_carry {
                    self.state.next();
                    return false;
                }

                self.state.t0();
            }
            // fix high byte of effective address
            State::T5 => {
                debug_assert!(self.addr_carry);
                inc!(self.addr, 0x100);
                self.addr_carry = false;

                self.data = self.read_u8(self.addr);
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
            State::T1 => advance!(self),
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
                // adjust offset for 2's compliment
                let mut offset = self.data as u16;
                if self.data & 0x80 != 0 {
                    offset |= 0xFF00;
                }

                let addr = self.pc.wrapping_add(offset);
                self.addr_carry = (addr & 0xFF00) != (self.pc & 0xFF00);
                self.pc = (self.pc & 0xFF00) | (addr & 0x00FF);

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
            State::T1 => advance!(self),
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
        exec_inst!(
            self, ADC, adc, AND, and, ASL, asl, BCC, bcc, BCS, bcs, BEQ, beq, BIT, bit, BMI, bmi,
            BNE, bne, BPL, bpl, BRK, brk, BVC, bvc, BVS, bvs, CLC, clc, CLD, cld, CLI, cli, CLV,
            clv, CMP, cmp, CPX, cpx, CPY, cpy, DEC, dec, DEX, dex, DEY, dey, EOR, eor, INC, inc,
            INX, inx, INY, iny, JMP, jmp, JSR, jsr, LDA, lda, LDX, ldx, LDY, ldy, LSR, lsr, NOP,
            nop, ORA, ora, PHA, pha, PHP, php, PLA, pla, PLP, plp, ROL, rol, ROR, ror, RTI, rti,
            RTS, rts, SBC, sbc, SEC, sec, SED, sed, SEI, sei, STA, sta, STX, stx, STY, sty, TAX,
            tax, TAY, tay, TSX, tsx, TXA, txa, TXS, txs, TYA, tya
        );
    }

    impl_inst! {
        BRANCH => (bcs, C, true),
        BRANCH => (bcc, C, false),
        BRANCH => (beq, Z, true),
        BRANCH => (bne, Z, false),
        BRANCH => (bmi, N, true),
        BRANCH => (bpl, N, false),
        BRANCH => (bvs, V, true),
        BRANCH => (bvc, V, false),
        COMPARE => (cmp, a),
        COMPARE => (cpx, x),
        COMPARE => (cpy, y),
        FLAG => (clc, C, false),
        FLAG => (cld, D, false),
        FLAG => (cli, I, false),
        FLAG => (clv, V, false),
        FLAG => (sec, C, true),
        FLAG => (sed, D, true),
        FLAG => (sei, I, true),
        DECREG => (dex, x),
        DECREG => (dey, y),
        INCDEC => (dec, dec),
        INCDEC => (inc, inc),
        INCREG => (inx, x),
        INCREG => (iny, y),
        LOAD => (lda, a),
        LOAD => (ldx, x),
        LOAD => (ldy, y),
        LOGICAL => (and, &=),
        LOGICAL => (ora, |=),
        LOGICAL => (eor, ^=),
        SHIFT_ROTATE => (asl, asl_impl),
        SHIFT_ROTATE => (lsr, lsr_impl),
        SHIFT_ROTATE => (rol, rol_impl),
        SHIFT_ROTATE => (ror, ror_impl),
        STORE => (sta, a),
        STORE => (stx, x),
        STORE => (sty, y),
        TRANSFER => (tax, x, a),
        TRANSFER => (tay, y, a),
        TRANSFER => (tsx, x, s),
        TRANSFER => (txa, a, x),
        TRANSFER => (txs, s, x, NO_FLAGS),
        TRANSFER => (tya, a, y),
    }

    fn adc(&mut self) {
        if !self.resolve() {
            return;
        }

        if self.get_flag(StatusFlag::D) {
            todo!();
        } else {
            self.a = binary_add(self, self.data);
        }

        self.finish();
    }

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

    fn brk(&mut self) {
        debug_assert!(self.ir.mode == AddressingMode::Implied);
        debug_assert!(self.interrupt.src != Source::None);

        match self.state {
            // fetch opcode, increment PC
            State::T1 => advance!(self),
            // read next instruction byte (and throw it away)
            // increment PC for BRK instruction
            State::T2 => {
                if self.interrupt.src == Source::BRK {
                    inc!(self.pc);
                }
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
                // this point determines which interrupt vector is used
                self.addr = match self.interrupt.src {
                    Source::IRQ | Source::BRK => IRQ_VECTOR,
                    Source::NMI => NMI_VECTOR,
                    Source::RES => RES_VECTOR,
                    _ => unreachable!(),
                };

                if self.interrupt.src == Source::RES {
                    dec!(self.s);
                } else {
                    // NMI/IRQ push with B clear, BRK push with B set
                    let b =
                        if self.interrupt.src == Source::NMI || self.interrupt.src == Source::IRQ {
                            0
                        } else {
                            StatusFlag::B as u8
                        };
                    let p = self.p & 0b1100_1111 | StatusFlag::_U as u8 | b;
                    self.push_u8(p);
                }

                self.state.next();
            }
            // fetch PCL, set I flag
            State::T6 => {
                self.pc = self.read_u8(self.addr) as u16;
                inc!(self.addr);
                self.set_flag(StatusFlag::I, true);
                self.state.t0();
            }
            // fetch PCH
            State::T0 => {
                self.pc |= (self.read_u8(self.addr) as u16) << 8;
                self.state.t1();
                self.interrupt.clear();
            }
            _ => unreachable!(),
        }
    }

    fn jmp(&mut self) {
        debug_assert!(
            self.ir.mode == AddressingMode::Absolute || self.ir.mode == AddressingMode::Indirect
        );

        match (self.ir.mode, self.state) {
            // fetch opcode, increment PC
            (AddressingMode::Absolute | AddressingMode::Indirect, State::T1) => advance!(self),

            // fetch low address byte, increment PC
            (AddressingMode::Absolute, State::T2) => {
                self.addr = self.read_u8(self.pc) as u16;
                advance!(self, T0);
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
                advance!(self);
            }
            // fetch pointer address high, increment PC
            (AddressingMode::Indirect, State::T3) => {
                self.addr |= (self.read_u8(self.pc) as u16) << 8;
                advance!(self);
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

    fn jsr(&mut self) {
        debug_assert!(self.ir.mode == AddressingMode::Absolute);

        match self.state {
            // fetch opcode, increment PC
            State::T1 => advance!(self),
            // fetch low address byte, increment PC
            State::T2 => {
                self.data = self.read_u8(self.pc);
                advance!(self);
            }
            // internal cycle
            State::T3 => self.state.next(),
            // push PCH on stack
            State::T4 => {
                let pch = (self.pc & 0xFF00) >> 8;
                self.push_u8(pch as u8);
                self.state.next();
            }
            // push PCL on stack
            State::T5 => {
                let pcl = self.pc as u8;
                self.push_u8(pcl);
                self.state.t0();
            }
            // copy low address byte to PCL, fetch high address byte to PCH
            State::T0 => {
                let pcl = self.data as u16;
                self.data = self.read_u8(self.pc);
                let pch = self.data as u16;
                self.pc = (pch << 8) | pcl;
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    fn nop(&mut self) {
        match self.state {
            State::T1 => advance!(self, T0_2),
            State::T0_2 => self.state.t1(),
            _ => unreachable!(),
        }
    }

    fn pha(&mut self) {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => advance!(self),
            // read next instruction byte (and throw it away)
            State::T2 => self.state.t0(),
            // push register on stack, decrement S
            State::T0 => {
                self.push_u8(self.a);
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    fn php(&mut self) {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => advance!(self),
            // read next instruction byte (and throw it away)
            State::T2 => self.state.t0(),
            // push register on stack, decrement S
            State::T0 => {
                self.push_u8(self.p | StatusFlag::_U as u8 | StatusFlag::B as u8);
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    fn pla(&mut self) {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => advance!(self),
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

    fn plp(&mut self) {
        match self.state {
            // fetch opcode, increment PC
            State::T1 => advance!(self),
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

    fn rti(&mut self) {
        debug_assert!(self.ir.mode == AddressingMode::Implied);

        match self.state {
            // fetch opcode, increment PC
            State::T1 => advance!(self),
            // read next instruction byte (and throw it away)
            State::T2 => advance!(self),
            // increment S
            State::T3 => {
                inc!(self.s);
                self.state.next();
            }
            // pull P from stack, increment S
            State::T4 => {
                // ignore bit 5 and B from stack
                let p = self.read_u8(self.sp_u16()) & 0b1100_1111;
                self.p = (self.p & 0b0011_0000) | p;
                inc!(self.s);
                self.state.next();
            }
            // pull PCL from stack, increment S
            State::T5 => {
                self.pc = self.read_u8(self.sp_u16()) as u16;
                inc!(self.s);
                self.state.t0();
            }
            // pull PCH from stack
            State::T0 => {
                let pch = self.read_u8(self.sp_u16()) as u16;
                self.pc |= pch << 8;
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    fn rts(&mut self) {
        debug_assert!(self.ir.mode == AddressingMode::Implied);

        match self.state {
            // fetch opcode, increment PC
            State::T1 => advance!(self),
            // read next instruction byte (and throw it away)
            State::T2 => {
                self.state.next();
            }
            // increment S
            State::T3 => {
                inc!(self.s);
                self.state.next();
            }
            // pull PCL from stack, increment S
            State::T4 => {
                self.pc = self.read_u8(self.sp_u16()) as u16;
                inc!(self.s);
                self.state.next();
            }
            // pull PCH from stack
            State::T5 => {
                let pch = self.read_u8(self.sp_u16()) as u16;
                self.pc |= pch << 8;
                self.state.t0();
            }
            // increment PC
            State::T0 => {
                inc!(self.pc);
                self.finish();
            }
            _ => unreachable!(),
        }
    }

    fn sbc(&mut self) {
        if !self.resolve() {
            return;
        }

        if self.get_flag(StatusFlag::D) {
            todo!();
        } else {
            self.a = binary_add(self, !self.data);
        }

        self.finish();
    }

    /// Finalize instruction execution (reset state, phase, etc.).
    fn finish(&mut self) {
        self.phase = Phase::Read;
        self.state.t1();
    }
}

fn binary_add<B: Bus>(cpu: &mut Cpu<B>, value: u8) -> u8 {
    let mut res = cpu.a.wrapping_add(value);
    if cpu.get_flag(StatusFlag::C) {
        inc!(res);
    }

    cpu.set_flag(StatusFlag::V, (cpu.a ^ res) & (value ^ res) & 0x80 != 0);
    cpu.set_flag(StatusFlag::C, res < cpu.a);
    cpu.set_flag_zn(res);

    res
}

fn asl_impl<B: Bus>(cpu: &mut Cpu<B>, mut value: u8) -> u8 {
    cpu.set_flag(StatusFlag::C, value & 0x80 != 0);
    value = (value & 0x7F) << 1;
    cpu.set_flag_zn(value);

    value
}

fn lsr_impl<B: Bus>(cpu: &mut Cpu<B>, mut value: u8) -> u8 {
    cpu.set_flag(StatusFlag::C, value & 0x1 != 0);
    value = (value & 0xFE) >> 1;
    cpu.set_flag_zn(value);

    value
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
        ($lhs:expr, $rhs:expr) => {
            if $lhs != $rhs {
                panic!(
                    "failed byte assertion:\n    \
                    expr: `{}`\n  \
                    expect: `0x{:04X}`\n  \
                    actual: `0x{:04X}`\n",
                    stringify!($lhs),
                    $rhs,
                    $lhs,
                );
            }
        };
    }

    macro_rules! assert_byte_eq {
        ($lhs:expr, $rhs:expr) => {{
            if $lhs != $rhs {
                panic!(
                    "failed byte assertion:\n    \
                    expr: `{}`\n  \
                    expect: `0x{:02X}`\n  \
                    actual: `0x{:02X}`\n",
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
        test_flag!(cpu, V, true, reset = true);
        test_flag!(cpu, _U, true, reset = true);
        test_flag!(cpu, B, true, reset = true);
        test_flag!(cpu, D, true, reset = true);
        test_flag!(cpu, I, true, reset = true);
        test_flag!(cpu, Z, true, reset = true);
        test_flag!(cpu, C, true, reset = true);

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
