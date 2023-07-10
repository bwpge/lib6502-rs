pub mod macros;

use lib6502::Bus;

pub struct Ram {
    mem: Box<[u8; 0x10000]>,
}

impl Ram {
    /// Creates a new [`RAM`] with zeroed memory.
    pub fn new() -> Self {
        Self {
            mem: Box::new([0u8; 0x10000]),
        }
    }

    /// Load the `program` at the given starting address.
    #[must_use]
    pub fn load_at(mut self, program: &[u8], start: u16) -> Self {
        let start = start as usize;
        let end = start + program.len();
        self.mem[start..end].copy_from_slice(program);

        self
    }
}

impl Bus for Ram {
    fn read(&self, address: u16) -> u8 {
        self.mem[address as usize]
    }

    fn write(&mut self, address: u16, data: u8) {
        self.mem[address as usize] = data;
    }
}
