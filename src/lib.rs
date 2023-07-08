//! A library for 6502 emulation.

mod cpu;
mod instruction;
mod traits;

pub use cpu::{Cpu, StatusFlag};
pub use instruction::{AddressingMode, Instruction, Mnemonic};
pub use traits::Bus;
