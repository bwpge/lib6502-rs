use std::fmt::{Debug, Display};

const INSTRUCTION_SET: [Instruction; 256] = [
    Instruction::new(Mnemonic::BRK, AddressingMode::Implied, 0x00),
    Instruction::new(Mnemonic::ORA, AddressingMode::IndirectX, 0x01),
    Instruction::illegal(0x02),
    Instruction::illegal(0x03),
    Instruction::illegal(0x04),
    Instruction::new(Mnemonic::ORA, AddressingMode::ZeroPage, 0x05),
    Instruction::new(Mnemonic::ASL, AddressingMode::ZeroPage, 0x06),
    Instruction::illegal(0x07),
    Instruction::new(Mnemonic::PHP, AddressingMode::Implied, 0x08),
    Instruction::new(Mnemonic::ORA, AddressingMode::Immediate, 0x09),
    Instruction::new(Mnemonic::ASL, AddressingMode::Accumulator, 0xA),
    Instruction::illegal(0x0B),
    Instruction::illegal(0x0C),
    Instruction::new(Mnemonic::ORA, AddressingMode::Absolute, 0x0D),
    Instruction::new(Mnemonic::ASL, AddressingMode::Absolute, 0x0E),
    Instruction::illegal(0x0F),
    Instruction::new(Mnemonic::BPL, AddressingMode::Relative, 0x10),
    Instruction::new(Mnemonic::ORA, AddressingMode::IndirectY, 0x11),
    Instruction::illegal(0x12),
    Instruction::illegal(0x13),
    Instruction::illegal(0x14),
    Instruction::new(Mnemonic::ORA, AddressingMode::ZeroPageX, 0x15),
    Instruction::new(Mnemonic::ASL, AddressingMode::ZeroPageX, 0x16),
    Instruction::illegal(0x17),
    Instruction::new(Mnemonic::CLC, AddressingMode::Implied, 0x18),
    Instruction::new(Mnemonic::ORA, AddressingMode::AbsoluteY, 0x19),
    Instruction::illegal(0x1A),
    Instruction::illegal(0x1B),
    Instruction::illegal(0x1C),
    Instruction::new(Mnemonic::ORA, AddressingMode::AbsoluteX, 0x1D),
    Instruction::new(Mnemonic::ASL, AddressingMode::AbsoluteX, 0x1E),
    Instruction::illegal(0x1F),
    Instruction::new(Mnemonic::JSR, AddressingMode::Absolute, 0x20),
    Instruction::new(Mnemonic::AND, AddressingMode::IndirectX, 0x21),
    Instruction::illegal(0x22),
    Instruction::illegal(0x23),
    Instruction::new(Mnemonic::BIT, AddressingMode::ZeroPage, 0x24),
    Instruction::new(Mnemonic::AND, AddressingMode::ZeroPage, 0x25),
    Instruction::new(Mnemonic::ROL, AddressingMode::ZeroPage, 0x26),
    Instruction::illegal(0x27),
    Instruction::new(Mnemonic::PLP, AddressingMode::Implied, 0x28),
    Instruction::new(Mnemonic::AND, AddressingMode::Immediate, 0x29),
    Instruction::new(Mnemonic::ROL, AddressingMode::Accumulator, 0x2A),
    Instruction::illegal(0x2B),
    Instruction::new(Mnemonic::BIT, AddressingMode::Absolute, 0x2C),
    Instruction::new(Mnemonic::AND, AddressingMode::Absolute, 0x2D),
    Instruction::new(Mnemonic::ROL, AddressingMode::Absolute, 0x2E),
    Instruction::illegal(0x2F),
    Instruction::new(Mnemonic::BMI, AddressingMode::Relative, 0x30),
    Instruction::new(Mnemonic::AND, AddressingMode::IndirectY, 0x31),
    Instruction::illegal(0x32),
    Instruction::illegal(0x33),
    Instruction::illegal(0x34),
    Instruction::new(Mnemonic::AND, AddressingMode::ZeroPageX, 0x35),
    Instruction::new(Mnemonic::ROL, AddressingMode::ZeroPageX, 0x36),
    Instruction::illegal(0x37),
    Instruction::new(Mnemonic::SEC, AddressingMode::Implied, 0x38),
    Instruction::new(Mnemonic::AND, AddressingMode::AbsoluteY, 0x39),
    Instruction::illegal(0x3A),
    Instruction::illegal(0x3B),
    Instruction::illegal(0x3C),
    Instruction::new(Mnemonic::AND, AddressingMode::AbsoluteX, 0x3D),
    Instruction::new(Mnemonic::ROL, AddressingMode::AbsoluteX, 0x3E),
    Instruction::illegal(0x3F),
    Instruction::new(Mnemonic::RTI, AddressingMode::Implied, 0x40),
    Instruction::new(Mnemonic::EOR, AddressingMode::IndirectX, 0x41),
    Instruction::illegal(0x42),
    Instruction::illegal(0x43),
    Instruction::illegal(0x44),
    Instruction::new(Mnemonic::EOR, AddressingMode::ZeroPage, 0x45),
    Instruction::new(Mnemonic::LSR, AddressingMode::ZeroPage, 0x46),
    Instruction::illegal(0x47),
    Instruction::new(Mnemonic::PHA, AddressingMode::Implied, 0x48),
    Instruction::new(Mnemonic::EOR, AddressingMode::Immediate, 0x49),
    Instruction::new(Mnemonic::LSR, AddressingMode::Accumulator, 0x4A),
    Instruction::illegal(0x4B),
    Instruction::new(Mnemonic::JMP, AddressingMode::Absolute, 0x4C),
    Instruction::new(Mnemonic::EOR, AddressingMode::Absolute, 0x4D),
    Instruction::new(Mnemonic::LSR, AddressingMode::Absolute, 0x4E),
    Instruction::illegal(0x4F),
    Instruction::new(Mnemonic::BVC, AddressingMode::Relative, 0x50),
    Instruction::new(Mnemonic::EOR, AddressingMode::IndirectY, 0x51),
    Instruction::illegal(0x52),
    Instruction::illegal(0x53),
    Instruction::illegal(0x54),
    Instruction::new(Mnemonic::EOR, AddressingMode::ZeroPageX, 0x55),
    Instruction::new(Mnemonic::LSR, AddressingMode::ZeroPageX, 0x56),
    Instruction::illegal(0x57),
    Instruction::new(Mnemonic::CLI, AddressingMode::Implied, 0x58),
    Instruction::new(Mnemonic::EOR, AddressingMode::AbsoluteY, 0x59),
    Instruction::illegal(0x5A),
    Instruction::illegal(0x5B),
    Instruction::illegal(0x5C),
    Instruction::new(Mnemonic::EOR, AddressingMode::AbsoluteX, 0x5D),
    Instruction::new(Mnemonic::LSR, AddressingMode::AbsoluteX, 0x5E),
    Instruction::illegal(0x5F),
    Instruction::new(Mnemonic::RTS, AddressingMode::Implied, 0x60),
    Instruction::new(Mnemonic::ADC, AddressingMode::IndirectX, 0x61),
    Instruction::illegal(0x62),
    Instruction::illegal(0x63),
    Instruction::illegal(0x64),
    Instruction::new(Mnemonic::ADC, AddressingMode::ZeroPage, 0x65),
    Instruction::new(Mnemonic::ROR, AddressingMode::ZeroPage, 0x66),
    Instruction::illegal(0x67),
    Instruction::new(Mnemonic::PLA, AddressingMode::Implied, 0x68),
    Instruction::new(Mnemonic::ADC, AddressingMode::Immediate, 0x69),
    Instruction::new(Mnemonic::ROR, AddressingMode::Accumulator, 0x6A),
    Instruction::illegal(0x6B),
    Instruction::new(Mnemonic::JMP, AddressingMode::Indirect, 0x6C),
    Instruction::new(Mnemonic::ADC, AddressingMode::Absolute, 0x6D),
    Instruction::new(Mnemonic::ROR, AddressingMode::Absolute, 0x6E),
    Instruction::illegal(0x6F),
    Instruction::new(Mnemonic::BVS, AddressingMode::Relative, 0x70),
    Instruction::new(Mnemonic::ADC, AddressingMode::IndirectY, 0x71),
    Instruction::illegal(0x72),
    Instruction::illegal(0x73),
    Instruction::illegal(0x74),
    Instruction::new(Mnemonic::ADC, AddressingMode::ZeroPageX, 0x75),
    Instruction::new(Mnemonic::ROR, AddressingMode::ZeroPageX, 0x76),
    Instruction::illegal(0x77),
    Instruction::new(Mnemonic::SEI, AddressingMode::Implied, 0x78),
    Instruction::new(Mnemonic::ADC, AddressingMode::AbsoluteY, 0x79),
    Instruction::illegal(0x7A),
    Instruction::illegal(0x7B),
    Instruction::illegal(0x7C),
    Instruction::new(Mnemonic::ADC, AddressingMode::AbsoluteX, 0x7D),
    Instruction::new(Mnemonic::ROR, AddressingMode::AbsoluteX, 0x7E),
    Instruction::illegal(0x7F),
    Instruction::illegal(0x80),
    Instruction::new(Mnemonic::STA, AddressingMode::IndirectX, 0x81),
    Instruction::illegal(0x82),
    Instruction::illegal(0x83),
    Instruction::new(Mnemonic::STY, AddressingMode::ZeroPage, 0x84),
    Instruction::new(Mnemonic::STA, AddressingMode::ZeroPage, 0x85),
    Instruction::new(Mnemonic::STX, AddressingMode::ZeroPage, 0x86),
    Instruction::illegal(0x87),
    Instruction::new(Mnemonic::DEY, AddressingMode::Implied, 0x88),
    Instruction::illegal(0x89),
    Instruction::new(Mnemonic::TXA, AddressingMode::Implied, 0x8A),
    Instruction::illegal(0x8B),
    Instruction::new(Mnemonic::STY, AddressingMode::Absolute, 0x8C),
    Instruction::new(Mnemonic::STA, AddressingMode::Absolute, 0x8D),
    Instruction::new(Mnemonic::STX, AddressingMode::Absolute, 0x8E),
    Instruction::illegal(0x8F),
    Instruction::new(Mnemonic::BCC, AddressingMode::Relative, 0x90),
    Instruction::new(Mnemonic::STA, AddressingMode::IndirectY, 0x91),
    Instruction::illegal(0x92),
    Instruction::illegal(0x93),
    Instruction::new(Mnemonic::STY, AddressingMode::ZeroPageX, 0x94),
    Instruction::new(Mnemonic::STA, AddressingMode::ZeroPageX, 0x95),
    Instruction::new(Mnemonic::STX, AddressingMode::ZeroPageY, 0x96),
    Instruction::illegal(0x97),
    Instruction::new(Mnemonic::TYA, AddressingMode::Implied, 0x98),
    Instruction::new(Mnemonic::STA, AddressingMode::AbsoluteY, 0x99),
    Instruction::new(Mnemonic::TXS, AddressingMode::Implied, 0x9A),
    Instruction::illegal(0x9B),
    Instruction::illegal(0x9C),
    Instruction::new(Mnemonic::STA, AddressingMode::AbsoluteX, 0x9D),
    Instruction::illegal(0x9E),
    Instruction::illegal(0x9F),
    Instruction::new(Mnemonic::LDY, AddressingMode::Immediate, 0xA0),
    Instruction::new(Mnemonic::LDA, AddressingMode::IndirectX, 0xA1),
    Instruction::new(Mnemonic::LDX, AddressingMode::Immediate, 0xA2),
    Instruction::illegal(0xA3),
    Instruction::new(Mnemonic::LDY, AddressingMode::ZeroPage, 0xA4),
    Instruction::new(Mnemonic::LDA, AddressingMode::ZeroPage, 0xA5),
    Instruction::new(Mnemonic::LDX, AddressingMode::ZeroPage, 0xA6),
    Instruction::illegal(0xA7),
    Instruction::new(Mnemonic::TAY, AddressingMode::Implied, 0xA8),
    Instruction::new(Mnemonic::LDA, AddressingMode::Immediate, 0xA9),
    Instruction::new(Mnemonic::TAX, AddressingMode::Implied, 0xAA),
    Instruction::illegal(0xAB),
    Instruction::new(Mnemonic::LDY, AddressingMode::Absolute, 0xAC),
    Instruction::new(Mnemonic::LDA, AddressingMode::Absolute, 0xAD),
    Instruction::new(Mnemonic::LDX, AddressingMode::Absolute, 0xAE),
    Instruction::illegal(0xAF),
    Instruction::new(Mnemonic::BCS, AddressingMode::Relative, 0xB0),
    Instruction::new(Mnemonic::LDA, AddressingMode::IndirectY, 0xB1),
    Instruction::illegal(0xB2),
    Instruction::illegal(0xB3),
    Instruction::new(Mnemonic::LDY, AddressingMode::ZeroPageX, 0xB4),
    Instruction::new(Mnemonic::LDA, AddressingMode::ZeroPageX, 0xB5),
    Instruction::new(Mnemonic::LDX, AddressingMode::ZeroPageY, 0xB6),
    Instruction::illegal(0xB7),
    Instruction::new(Mnemonic::CLV, AddressingMode::Implied, 0xB8),
    Instruction::new(Mnemonic::LDA, AddressingMode::AbsoluteY, 0xB9),
    Instruction::new(Mnemonic::TSX, AddressingMode::Implied, 0xBA),
    Instruction::illegal(0xBB),
    Instruction::new(Mnemonic::LDY, AddressingMode::AbsoluteX, 0xBC),
    Instruction::new(Mnemonic::LDA, AddressingMode::AbsoluteX, 0xBD),
    Instruction::new(Mnemonic::LDX, AddressingMode::AbsoluteY, 0xBE),
    Instruction::illegal(0xBF),
    Instruction::new(Mnemonic::CPY, AddressingMode::Immediate, 0xC0),
    Instruction::new(Mnemonic::CMP, AddressingMode::IndirectX, 0xC1),
    Instruction::illegal(0xC2),
    Instruction::illegal(0xC3),
    Instruction::new(Mnemonic::CPY, AddressingMode::ZeroPage, 0xC4),
    Instruction::new(Mnemonic::CMP, AddressingMode::ZeroPage, 0xC5),
    Instruction::new(Mnemonic::DEC, AddressingMode::ZeroPage, 0xC6),
    Instruction::illegal(0xC7),
    Instruction::new(Mnemonic::INY, AddressingMode::Implied, 0xC8),
    Instruction::new(Mnemonic::CMP, AddressingMode::Immediate, 0xC9),
    Instruction::new(Mnemonic::DEX, AddressingMode::Implied, 0xCA),
    Instruction::illegal(0xCB),
    Instruction::new(Mnemonic::CPY, AddressingMode::Absolute, 0xCC),
    Instruction::new(Mnemonic::CMP, AddressingMode::Absolute, 0xCD),
    Instruction::new(Mnemonic::DEC, AddressingMode::Absolute, 0xCE),
    Instruction::illegal(0xCF),
    Instruction::new(Mnemonic::BNE, AddressingMode::Relative, 0xD0),
    Instruction::new(Mnemonic::CMP, AddressingMode::IndirectY, 0xD1),
    Instruction::illegal(0xD2),
    Instruction::illegal(0xD3),
    Instruction::illegal(0xD4),
    Instruction::new(Mnemonic::CMP, AddressingMode::ZeroPageX, 0xD5),
    Instruction::new(Mnemonic::DEC, AddressingMode::ZeroPageX, 0xD6),
    Instruction::illegal(0xD7),
    Instruction::new(Mnemonic::CLD, AddressingMode::Implied, 0xD8),
    Instruction::new(Mnemonic::CMP, AddressingMode::AbsoluteY, 0xD9),
    Instruction::illegal(0xDA),
    Instruction::illegal(0xDB),
    Instruction::illegal(0xDC),
    Instruction::new(Mnemonic::CMP, AddressingMode::AbsoluteX, 0xDD),
    Instruction::new(Mnemonic::DEC, AddressingMode::AbsoluteX, 0xDE),
    Instruction::illegal(0xDF),
    Instruction::new(Mnemonic::CPX, AddressingMode::Immediate, 0xE0),
    Instruction::new(Mnemonic::SBC, AddressingMode::IndirectX, 0xE1),
    Instruction::illegal(0xE2),
    Instruction::illegal(0xE3),
    Instruction::new(Mnemonic::CPX, AddressingMode::ZeroPage, 0xE4),
    Instruction::new(Mnemonic::SBC, AddressingMode::ZeroPage, 0xE5),
    Instruction::new(Mnemonic::INC, AddressingMode::ZeroPage, 0xE6),
    Instruction::illegal(0xE7),
    Instruction::new(Mnemonic::INX, AddressingMode::Implied, 0xE8),
    Instruction::new(Mnemonic::SBC, AddressingMode::Immediate, 0xE9),
    Instruction::new(Mnemonic::NOP, AddressingMode::Implied, 0xEA),
    Instruction::illegal(0xEB),
    Instruction::new(Mnemonic::CPX, AddressingMode::Absolute, 0xEC),
    Instruction::new(Mnemonic::SBC, AddressingMode::Absolute, 0xED),
    Instruction::new(Mnemonic::INC, AddressingMode::Absolute, 0xEE),
    Instruction::illegal(0xEF),
    Instruction::new(Mnemonic::BEQ, AddressingMode::Relative, 0xF0),
    Instruction::new(Mnemonic::SBC, AddressingMode::IndirectY, 0xF1),
    Instruction::illegal(0xF2),
    Instruction::illegal(0xF3),
    Instruction::illegal(0xF4),
    Instruction::new(Mnemonic::SBC, AddressingMode::ZeroPageX, 0xF5),
    Instruction::new(Mnemonic::INC, AddressingMode::ZeroPageX, 0xF6),
    Instruction::illegal(0xF7),
    Instruction::new(Mnemonic::SED, AddressingMode::Implied, 0xF8),
    Instruction::new(Mnemonic::SBC, AddressingMode::AbsoluteY, 0xF9),
    Instruction::illegal(0xFA),
    Instruction::illegal(0xFB),
    Instruction::illegal(0xFC),
    Instruction::new(Mnemonic::SBC, AddressingMode::AbsoluteX, 0xFD),
    Instruction::new(Mnemonic::INC, AddressingMode::AbsoluteX, 0xFE),
    Instruction::illegal(0xFF),
];

/// Represents the type of [`Instruction`] by its three-letter code.
///
/// Typically the mnemonic is useless without the [`AddressingMode`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Mnemonic {
    /// Add memory to accumulator with carry
    ADC,
    /// AND memory with accumulator
    AND,
    /// Shift left one bit (memory or accumulator)
    ASL,
    /// Branch on carry clear
    BCC,
    /// Branch on carry set
    BCS,
    /// Branch on result zero
    BEQ,
    /// Bit test
    BIT,
    /// Branch on minus (negative set)
    BMI,
    /// Branch on not equal (zero clear)
    BNE,
    /// Branch on plus (negative clear)
    BPL,
    /// Break / interrupt
    BRK,
    /// Branch on overflow clear
    BVC,
    /// Branch on overflow set
    BVS,
    /// Clear carry
    CLC,
    /// Clear decimal
    CLD,
    /// Clear interrupt disable
    CLI,
    /// Clear overflow
    CLV,
    /// Compare (with accumulator)
    CMP,
    /// Compare with X
    CPX,
    /// Compare with Y
    CPY,
    /// Decrement
    DEC,
    /// Decrement X
    DEX,
    /// Decrement Y
    DEY,
    /// Exclusive or (with accumulator)
    EOR,
    /// Increment
    INC,
    /// Increment X
    INX,
    /// Increment Y
    INY,
    /// Jump
    JMP,
    /// Jump subroutine
    JSR,
    /// Load accumulator
    LDA,
    /// Load X
    LDX,
    /// Load Y
    LDY,
    /// Logical shift right
    LSR,
    /// No operation
    NOP,
    /// Or with accumulator
    ORA,
    /// Push accumulator
    PHA,
    /// Push processor status (SR)
    PHP,
    /// Pull accumulator
    PLA,
    /// Pull processor status (SR)
    PLP,
    /// Rotate left
    ROL,
    /// Rotate right
    ROR,
    /// Return from interrupt
    RTI,
    /// Return from subroutine
    RTS,
    /// Subtract with carry
    SBC,
    /// Set carry
    SEC,
    /// Set decimal
    SED,
    /// Set interrupt disable
    SEI,
    /// Store accumulator
    STA,
    /// Store X
    STX,
    /// Store Y
    STY,
    /// Transfer accumulator to X
    TAX,
    /// Transfer accumulator to Y
    TAY,
    /// Transfer stack pointer to X
    TSX,
    /// Transfer X to accumulator
    TXA,
    /// Transfer X to stack pointer
    TXS,
    /// Transfer Y to accumulator
    TYA,
    /// Instruction which is not officially supported
    #[default]
    ILLEGAL,
}

impl Display for Mnemonic {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match *self {
            Mnemonic::ILLEGAL => f.write_str("???"),
            _ => Debug::fmt(self, f),
        }
    }
}

/// Represents different [`Instruction`] memory addressing modes.
///
/// Typically the addressing mode is useless without the [`Mnemonic`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum AddressingMode {
    /// Operand is accumulator (implied single byte instruction)
    Accumulator,
    /// Operand is address
    Absolute,
    /// Operand is address; effective address is address incremented by X with
    /// carry
    AbsoluteX,
    /// Operand is address; effective address is address incremented by Y with
    /// carry
    AbsoluteY,
    /// Operand is byte
    Immediate,
    /// Operand is implied
    #[default]
    Implied,
    /// Operand is address; effective address is contents of word at address
    Indirect,
    /// Operand is zeropage address; effective address is word in
    /// `(LL + X, LL + X + 1)`, incremented without carry
    IndirectX,
    /// Operand is zeropage address; effective address is word in `(LL, LL + 1)`
    /// incremented by `Y` with carry
    IndirectY,
    /// Branch target is `PC` + signed offset
    Relative,
    /// Operand is zeropage address (hi-byte is zero)
    ZeroPage,
    /// Operand is zeropage address; effective address is address incremented by
    /// `X` without carry
    ZeroPageX,
    /// Operand is zeropage address; effective address is address incremented by
    /// `Y` without carry
    ZeroPageY,
}

/// A decoded instruction encapsulating [`Mnemonic`], [`AddressingMode`], and
/// opcode.
///
/// # Constructor
///
/// A constructor is not provided for this type.
///
/// Instead, use [`Instruction::from`] to create a decoded instruction from a
/// byte ([`u8`]). This is done to ensure correctness and make invalid
/// instructions unrepresentable.
///
/// # Execution
///
/// This type does not provide any information on how it should behave on the
/// CPU (e.g., how to manipulate registers, writing, etc.). This is a deliberate
/// design choice, contrary to many other implementations.
///
/// From an architecture perspective, an instruction does not *execute*
/// itself. Execution only exists in the context of the *executor* (e.g., a
/// processor), and so these details are part of the CPU implementation. This
/// also allows more accurate emulation of the overall system, where operand
/// resolution can be emulated over the course cycles, instead of through a
/// single function call.
///
/// # Example
///
/// ```
/// use lib6502::{AddressingMode, Instruction, Mnemonic};
///
/// let instruction = Instruction::from(0xEA);
///
/// assert_eq!(instruction.mnemonic(), Mnemonic::NOP);
/// assert_eq!(instruction.mode(), AddressingMode::Implied);
/// assert_eq!(instruction.opcode(), 0xEA);
///
/// // can also be converted back to a `u8`
/// assert_eq!(u8::from(instruction), 0xEA);
/// ```
///
/// # References
///
/// See <https://www.masswerk.at/6502/6502_instruction_set.html> for detailed
/// instruction information.
///
/// [`Instruction::from`]: #impl-From<u8>-for-Instruction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Instruction {
    /// The name of this instruction, without any addressing information.
    pub(crate) mnemonic: Mnemonic,
    /// How the operand is resolved by this instruction.
    pub(crate) mode: AddressingMode,
    /// Opcode for this instruction.
    pub(crate) opcode: u8,
}

impl Default for Instruction {
    fn default() -> Self {
        INSTRUCTION_SET[0]
    }
}

impl Instruction {
    /// Returns the human-readable code for this instruction.
    pub fn mnemonic(self) -> Mnemonic {
        self.mnemonic
    }

    /// Returns the addressing mode for this instruction.
    pub fn mode(self) -> AddressingMode {
        self.mode
    }

    /// Returns the one-byte opcode for this instruction.
    pub fn opcode(self) -> u8 {
        self.opcode
    }

    /// Creates a new [`Instruction`] in a `const` context with the given parameters.
    ///
    /// Used in macros and populating the [`INSTRUCTION_SET`] array.
    const fn new(kind: Mnemonic, mode: AddressingMode, opcode: u8) -> Self {
        Self {
            mnemonic: kind,
            mode,
            opcode,
        }
    }

    /// Creates an illegal [`Instruction`] in a `const` context with the given code.
    ///
    /// Used in macros and populating the [`INSTRUCTION_SET`] array.
    const fn illegal(opcode: u8) -> Self {
        Self {
            mnemonic: Mnemonic::ILLEGAL,
            mode: AddressingMode::Implied,
            opcode,
        }
    }
}

impl From<u8> for Instruction {
    fn from(value: u8) -> Self {
        INSTRUCTION_SET[value as usize]
    }
}

impl From<Instruction> for u8 {
    fn from(value: Instruction) -> Self {
        value.opcode
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    macro_rules! expand_mode {
        (acc) => {
            AddressingMode::Accumulator
        };
        (abs) => {
            AddressingMode::Absolute
        };
        (abs_x) => {
            AddressingMode::AbsoluteX
        };
        (abs_y) => {
            AddressingMode::AbsoluteY
        };
        (imm) => {
            AddressingMode::Immediate
        };
        (imp) => {
            AddressingMode::Implied
        };
        (rel) => {
            AddressingMode::Relative
        };
        (ind) => {
            AddressingMode::Indirect
        };
        (x_ind) => {
            AddressingMode::IndirectX
        };
        (ind_y) => {
            AddressingMode::IndirectY
        };
        (zpg) => {
            AddressingMode::ZeroPage
        };
        (zpg_x) => {
            AddressingMode::ZeroPageX
        };
        (zpg_y) => {
            AddressingMode::ZeroPageY
        };
    }

    macro_rules! test_op {
        ($opcode:ident, -, -) => {{
            assert_eq!(
                Instruction::illegal($opcode),
                Instruction::from($opcode),
                "with opcode `{:#04X}`",
                $opcode
            );
        }};
        ($opcode:ident, $kind:tt, $mode:tt) => {
            assert_eq!(
                Instruction::new($kind, expand_mode!($mode), $opcode),
                Instruction::from($opcode),
                "with opcode `{:#04X}`",
                $opcode
            );
        };
    }

    macro_rules! test_chart {
        ($($op:tt $mode:tt),+ $(,)?) => {{
            use Mnemonic::*;
            let mut __op = 0u8;
            $(
                test_op!(__op, $op, $mode);
                __op = __op.wrapping_add(1);
            )+
            // ensure we have 256 input instructions
            assert_eq!(__op, 0u8);
        }};
    }

    #[test]
    fn decode_instruction() {
        // building this chart below allows testing that all instructions are
        // properly accounted for, without copy and pasting implementation code.
        // instead we can just visually enter values from the data sheet and
        // verify the implementation follows the correct logic.
        //
        // see:
        //   https://www.masswerk.at/6502/6502_instruction_set.html
        test_chart! {
            // HI  / LO
            //     / -0        -1          -2        -3     -4          -5          -6          -7    -8        -9          -A        -B    -C          -D          -E          -F
            /* 0- */ BRK imp,  ORA x_ind,  - -,      - -,   - -,        ORA zpg,    ASL zpg,    - -,  PHP imp,  ORA imm,    ASL acc,  - -,  - -,        ORA abs,    ASL abs,    - -,
            /* 1- */ BPL rel,  ORA ind_y,  - -,      - -,   - -,        ORA zpg_x,  ASL zpg_x,  - -,  CLC imp,  ORA abs_y,  - -,      - -,  - -,        ORA abs_x,  ASL abs_x,  - -,
            /* 2- */ JSR abs,  AND x_ind,  - -,      - -,   BIT zpg,    AND zpg,    ROL zpg,    - -,  PLP imp,  AND imm,    ROL acc,  - -,  BIT abs,    AND abs,    ROL abs,    - -,
            /* 3- */ BMI rel,  AND ind_y,  - -,      - -,   - -,        AND zpg_x,  ROL zpg_x,  - -,  SEC imp,  AND abs_y,  - -,      - -,  - -,        AND abs_x,  ROL abs_x,  - -,
            /* 4- */ RTI imp,  EOR x_ind,  - -,      - -,   - -,        EOR zpg,    LSR zpg,    - -,  PHA imp,  EOR imm,    LSR acc,  - -,  JMP abs,    EOR abs,    LSR abs,    - -,
            /* 5- */ BVC rel,  EOR ind_y,  - -,      - -,   - -,        EOR zpg_x,  LSR zpg_x,  - -,  CLI imp,  EOR abs_y,  - -,      - -,  - -,        EOR abs_x,  LSR abs_x,  - -,
            /* 6- */ RTS imp,  ADC x_ind,  - -,      - -,   - -,        ADC zpg,    ROR zpg,    - -,  PLA imp,  ADC imm,    ROR acc,  - -,  JMP ind,    ADC abs,    ROR abs,    - -,
            /* 7- */ BVS rel,  ADC ind_y,  - -,      - -,   - -,        ADC zpg_x,  ROR zpg_x,  - -,  SEI imp,  ADC abs_y,  - -,      - -,  - -,        ADC abs_x,  ROR abs_x,  - -,
            /* 8- */ - -,      STA x_ind,  - -,      - -,   STY zpg,    STA zpg,    STX zpg,    - -,  DEY imp,  - -,        TXA imp,  - -,  STY abs,    STA abs,    STX abs,    - -,
            /* 9- */ BCC rel,  STA ind_y,  - -,      - -,   STY zpg_x,  STA zpg_x,  STX zpg_y,  - -,  TYA imp,  STA abs_y,  TXS imp,  - -,  - -,        STA abs_x,  - -,        - -,
            /* A- */ LDY imm,  LDA x_ind,  LDX imm,  - -,   LDY zpg,    LDA zpg,    LDX zpg,    - -,  TAY imp,  LDA imm,    TAX imp,  - -,  LDY abs,    LDA abs,    LDX abs,    - -,
            /* B- */ BCS rel,  LDA ind_y,  - -,      - -,   LDY zpg_x,  LDA zpg_x,  LDX zpg_y,  - -,  CLV imp,  LDA abs_y,  TSX imp,  - -,  LDY abs_x,  LDA abs_x,  LDX abs_y,  - -,
            /* C- */ CPY imm,  CMP x_ind,  - -,      - -,   CPY zpg,    CMP zpg,    DEC zpg,    - -,  INY imp,  CMP imm,    DEX imp,  - -,  CPY abs,    CMP abs,    DEC abs,    - -,
            /* D- */ BNE rel,  CMP ind_y,  - -,      - -,   - -,        CMP zpg_x,  DEC zpg_x,  - -,  CLD imp,  CMP abs_y,  - -,      - -,  - -,        CMP abs_x,  DEC abs_x,  - -,
            /* E- */ CPX imm,  SBC x_ind,  - -,      - -,   CPX zpg,    SBC zpg,    INC zpg,    - -,  INX imp,  SBC imm,    NOP imp,  - -,  CPX abs,    SBC abs,    INC abs,    - -,
            /* F- */ BEQ rel,  SBC ind_y,  - -,      - -,   - -,        SBC zpg_x,  INC zpg_x,  - -,  SED imp,  SBC abs_y,  - -,      - -,  - -,        SBC abs_x,  INC abs_x,  - -,
        };
    }

    #[test]
    fn to_from_u8() {
        let byte = 0xEA;
        let inst = Instruction::from(byte);

        assert_eq!(inst.opcode(), byte);
        assert_eq!(u8::from(inst), byte);
    }

    #[test]
    fn default_instruction() {
        let inst = Instruction::default();

        assert_eq!(inst.opcode(), 0x00);
        assert_eq!(u8::from(inst), 0x00);
    }
}
