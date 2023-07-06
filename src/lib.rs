//! A library for 6502 emulation.

mod instruction;
mod traits;

pub use instruction::{AddressingMode, Instruction, Mnemonic};
pub use traits::Bus;
