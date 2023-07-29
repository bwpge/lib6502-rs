//! Utilities for disassembling instructions.

use std::fmt::{self, Display, Write};

use crate::{Bus, Instruction};

/// The value operated on by an [`Instruction`][crate::Instruction].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Operand {
    #[default]
    /// No operand
    None,
    /// A single byte operand.
    Byte(u8),
    /// A 16-bit operand in little endian order (low byte, high byte).
    Word(u8, u8),
}

impl Operand {
    /// Returns a byte representation of the operand.
    ///
    /// - `None`: returns `0`
    /// - `Byte`: returns the byte
    /// - `Word`: returns the low byte
    ///
    /// It is the caller's responsibility to check if the output of this method
    /// makes sense for a particular use case.
    pub fn as_u8(&self) -> u8 {
        match *self {
            Operand::None => 0,
            Operand::Byte(lo) | Operand::Word(lo, _) => lo,
        }
    }

    /// Returns a 16-bit word representation of the operand.
    ///
    /// - `None`: returns `0`
    /// - `Byte`: returns the byte as a 16-bit word, with leading zeroes
    /// - `Word`: returns the bytes as a 16-bit word
    ///
    /// It is the caller's responsibility to check if the output of this method
    /// makes sense for a particular use case.
    pub fn as_u16(&self) -> u16 {
        match *self {
            Operand::None => 0,
            Operand::Byte(lo) => lo as u16,
            Operand::Word(lo, hi) => u16::from_le_bytes([lo, hi]),
        }
    }

    /// Returns the size of the operand in bytes.
    ///
    /// Useful for calculating address offsets after disassembling an instruction.
    pub fn size(&self) -> usize {
        match self {
            Operand::None => 0,
            Operand::Byte(_) => 1,
            Operand::Word(_, _) => 2,
        }
    }
}

/// A disassembled instruction.
///
/// # Formatting
///
/// By default, human readable representation is used (e.g., `JMP $ABCD`). The
/// alternate specifier (`#`) can be used to display the instruction as byte
/// codes (e.g., `4C CD AB`).
#[derive(Debug)]
pub struct Disassembled {
    addr: u16,
    instruction: Instruction,
    operand: Operand,
}

impl Disassembled {
    /// Returns the address at which the instruction was read.
    pub fn address(&self) -> u16 {
        self.addr
    }

    /// Returns the [`Instruction`].
    pub fn instruction(&self) -> Instruction {
        self.instruction
    }

    /// Returns the instruction [`Operand`].
    pub fn operand(&self) -> Operand {
        self.operand
    }

    fn display_buf(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut buf = String::with_capacity(2);
        self.display_impl(&mut buf, f.alternate())?;
        f.pad(&buf)
    }

    fn display_impl<W: Write>(&self, mut wtr: W, is_alternate: bool) -> fmt::Result {
        use crate::AddressingMode::*;

        if is_alternate {
            write!(wtr, "{:02X}", self.instruction.opcode())?;
            match self.operand {
                Operand::None => (),
                Operand::Byte(lo) => match self.instruction.mode() {
                    Immediate | IndirectX | IndirectY | Relative | ZeroPage | ZeroPageX
                    | ZeroPageY => write!(wtr, " {lo:02X}")?,
                    _ => unreachable!(),
                },
                Operand::Word(lo, hi) => match self.instruction.mode() {
                    Absolute | AbsoluteX | AbsoluteY | Indirect => {
                        write!(wtr, " {lo:02X} {hi:02X}")?
                    }
                    _ => unreachable!(),
                },
            }
        } else {
            write!(wtr, "{}", self.instruction.mnemonic())?;
            match self.instruction.mode() {
                Accumulator | Implied => (),
                Absolute => write!(wtr, " ${:04X}", self.operand.as_u16())?,
                AbsoluteX => write!(wtr, " ${:04X},X", self.operand.as_u16())?,
                AbsoluteY => write!(wtr, " ${:04X},Y", self.operand.as_u16())?,
                Immediate => write!(wtr, " #${:02X}", self.operand.as_u8())?,
                Indirect => write!(wtr, " $({:04X})", self.operand.as_u16())?,
                IndirectX => write!(wtr, " (${:02X},X)", self.operand.as_u8())?,
                IndirectY => write!(wtr, " (${:02X}),Y", self.operand.as_u8())?,
                Relative | ZeroPage => write!(wtr, " ${:02X}", self.operand.as_u8())?,
                ZeroPageX => write!(wtr, " ${:02X},X", self.operand.as_u8())?,
                ZeroPageY => write!(wtr, " ${:02X},Y", self.operand.as_u8())?,
            }
        }

        Ok(())
    }
}

impl Display for Disassembled {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.width().is_some() {
            self.display_buf(f)
        } else {
            let alt = f.alternate();
            self.display_impl(f, alt)
        }
    }
}

/// Disassembles instructions by reading bytes from a [`Bus`].
///
/// # Iterating
///
/// The [`iter`](#method.iter) or [`iter_at`](#method.iter_at) methods can be
/// used to create a borrowed [`Iterator`], similar to how [`Vec`] works.
///
/// See [`Iter`] for more information.
pub struct Disassembler<B: Bus> {
    bus: B,
}

impl<B: Bus> Disassembler<B> {
    /// Creates a new [`Disassembler`].
    pub fn new(bus: B) -> Self {
        Self { bus }
    }

    /// Disassembles the instruction at the given address.
    pub fn disassmble(&self, mut address: u16) -> Disassembled {
        use crate::AddressingMode::*;

        let addr = address;
        let instruction = Instruction::from(self.bus.read(addr));
        address = address.wrapping_add(1);

        let operand = match instruction.mode() {
            Accumulator | Implied => Operand::None,
            Immediate | IndirectX | IndirectY | Relative | ZeroPage | ZeroPageX | ZeroPageY => {
                let lo = self.bus.read(address);
                Operand::Byte(lo)
            }
            Absolute | AbsoluteX | AbsoluteY | Indirect => {
                let lo = self.bus.read(address);
                address = address.wrapping_add(1);
                let hi = self.bus.read(address);
                Operand::Word(lo, hi)
            }
        };

        Disassembled {
            addr,
            instruction,
            operand,
        }
    }

    /// Creates a borrowed [`Iterator`] that starts disassembly at address `0`.
    pub fn iter(&self) -> Iter<'_, B> {
        Iter::new(self)
    }

    /// Creates a borrowed [`Iterator`] that starts disassembly at the given
    /// `start` address.
    pub fn iter_at(&self, start: u16) -> Iter<'_, B> {
        let mut result = Iter::new(self);
        result.addr = start;
        result
    }
}

/// Immutable disassembly iterator.
///
/// This iterator is created with the [`iter`] or [`iter_at`] methods on a
/// [`Disassembler`].
///
/// The iterator will disassemble instructions until reaching the maximum 16-bit
/// address (`0xFFFF`). If any instruction [`Operand`] requires more bytes past
/// `0xFFFF`, they will be read by wrapping around to `0`. The iterator will
/// always terminate (return `None`) after reading an instruction or operand at
/// `0xFFFF`.
///
/// [`iter`]: Disassembler#method.iter
/// [`iter_at`]: Disassembler#method.iter_at
pub struct Iter<'d, B: Bus> {
    addr: u16,
    dasm: &'d Disassembler<B>,
    finished: bool,
}

impl<'d, B: Bus> Iter<'d, B> {
    fn new(d: &'d Disassembler<B>) -> Self {
        Self {
            addr: 0,
            dasm: d,
            finished: false,
        }
    }
}

impl<'d, B: Bus> Iterator for Iter<'d, B> {
    type Item = Disassembled;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        let result = self.dasm.disassmble(self.addr);
        let offset = result.operand.size() as u16 + 1;

        if let Some(addr) = self.addr.checked_add(offset) {
            self.addr = addr;
        } else {
            self.finished = true;
        }

        Some(result)
    }
}

#[cfg(test)]
mod tests {
    use crate::{AddressingMode, Mnemonic};

    use super::*;

    struct Ram {
        mem: [u8; 0x10000],
    }

    impl Bus for Ram {
        fn read(&self, address: u16) -> u8 {
            self.mem[address as usize]
        }

        fn write(&mut self, _: u16, _: u8) {}
    }

    fn setup(bytes: &[u8], fill: u8) -> Disassembler<Ram> {
        let mut mem = [fill; 0x10000];
        mem[..bytes.len()].copy_from_slice(bytes);
        Disassembler::new(Ram { mem })
    }

    #[test]
    fn disassemble() {
        let dasm = setup(&[0x6C, 0xCD, 0x1F], 0xFF);

        let d = dasm.disassmble(0);
        assert_eq!(d.addr, 0);
        assert_eq!(d.instruction.opcode, 0x6C);
        assert_eq!(d.instruction.mnemonic, Mnemonic::JMP);
        assert_eq!(d.instruction.mode, AddressingMode::Indirect);
        assert_eq!(d.operand, Operand::Word(0xCD, 0x1F));

        let d = dasm.disassmble(3);
        assert_eq!(d.addr, 3);
        assert_eq!(d.instruction.opcode, 0xFF);
        assert_eq!(d.instruction.mnemonic, Mnemonic::ILLEGAL);
        assert_eq!(d.operand, Operand::None);

        let d = dasm.disassmble(0xFFFF);
        assert_eq!(d.addr, 0xFFFF);
        assert_eq!(d.instruction.opcode, 0xFF);
        assert_eq!(d.instruction.mnemonic, Mnemonic::ILLEGAL);
        assert_eq!(d.operand, Operand::None);
    }

    #[test]
    fn iter() {
        let dasm = setup(&[0xEA, 0x00, 0xE8], 0);
        let d = dasm.iter().take(3).collect::<Vec<_>>();

        assert_eq!(d.len(), 3);
        assert_eq!(d[0].addr, 0);
        assert_eq!(d[0].instruction.mnemonic, Mnemonic::NOP);
        assert_eq!(d[1].addr, 1);
        assert_eq!(d[1].instruction.mnemonic, Mnemonic::BRK);
        assert_eq!(d[2].addr, 2);
        assert_eq!(d[2].instruction.mnemonic, Mnemonic::INX);

        assert_eq!(dasm.iter().count(), 0x10000);
    }

    #[test]
    fn iter_at() {
        let dasm = setup(&[0xEA, 0x00, 0xE8], 0xFF);
        let d = dasm.iter_at(1).take(3).collect::<Vec<_>>();

        assert_eq!(d.len(), 3);
        assert_eq!(d[0].addr, 1);
        assert_eq!(d[0].instruction.mnemonic, Mnemonic::BRK);
        assert_eq!(d[1].addr, 2);
        assert_eq!(d[1].instruction.mnemonic, Mnemonic::INX);
        assert_eq!(d[2].addr, 3);
        assert_eq!(d[2].instruction.mnemonic, Mnemonic::ILLEGAL);

        assert_eq!(dasm.iter_at(0x8000).count(), 0x8000);
    }
}
