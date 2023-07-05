/// A type that emulates a [bus] for data exchange.
///
/// From an electronics perspective, this trait abstracts the concept of devices
/// being connected to the bus and addressable as one "unit", rather than modeling
/// the physical wires of the bus itself.
///
/// Buses typically have some differentiator (e.g., *memory* bus or *data* bus)
/// to indicate the usage. From a Rust and traits perspective, there is no need
/// to qualify the usage. A bus can handle none, one, or both methods of this
/// trait. For example, it is perfectly fine to send a write signal on a bus to
/// a ROM chip, where the incoming writes have no effect. This should not cause
/// a crash or panic in the emulation, as opposed to how many other projects do.
///
/// As such, implementors of this trait may freely decide what to do with any
/// `read` or `write` calls.
///
/// # Example
///
/// Simple implementation for read-only memory (ROM):
///
/// ```
/// use lib6502::Bus;
///
/// // 16K read-only memory
/// pub struct Rom {
///     mem: Box<[u8; 0x4000]>
/// }
///
/// impl Rom {
///     pub fn new() -> Self {
///         Self { mem: Box::new([0u8; 0x4000]) }
///     }
/// }
///
/// impl Bus for Rom {
///     fn read(&self, address: u16) -> u8 {
///         self.mem[address as usize]
///     }
///
///     fn write(&mut self, address: u16, data: u8) {
///         // write does nothing to a ROM
///     }
/// }
///
/// let mut rom = Rom::new();
/// assert_eq!(rom.read(0x30A1), 0);
///
/// // write into the void
/// rom.write(0xA1, 0xFF);
/// assert_eq!(rom.read(0x30A1), 0);
/// ```
///
/// A [`Bus`] can also be nested within structures, which enables complex device
/// modeling:
///
/// ```
/// use lib6502::Bus;
///
/// pub struct Rom {
///     mem: Box<[u8; 0x1FF0]>
/// }
///
/// impl Rom {
///     pub fn new() -> Self {
///         Self { mem: Box::new([0u8; 0x1FF0]) }
///     }
/// }
///
/// impl Bus for Rom {
///     fn read(&self, address: u16) -> u8 {
///         self.mem[address as usize]
///     }
///
///     fn write(&mut self, address: u16, data: u8) {
///         // write does nothing to a ROM
///     }
/// }
///
/// pub struct Mapper {
///     registers: [u8; 0x10],
///     rom: Rom,
/// }
///
/// impl Mapper {
///     pub fn new() -> Self {
///         let registers = [0; 0x10];
///         let rom = Rom::new();
///
///         Self { registers, rom }
///     }
/// }
///
/// impl Bus for Mapper {
///     fn read(&self, address: u16) -> u8 {
///         let addr = (address % 0x2000) as usize;
///         match (addr) {
///             0x0..=0xF => self.registers[addr],
///             // let ROM bus handle the read
///             _ => self.rom.read(address),
///         }
///     }
///
///     fn write(&mut self, address: u16, data: u8) {
///         let addr = (address % 0x2000) as usize;
///         match addr {
///             0x0..=0xF => self.registers[addr] = data,
///             // let ROM bus handle the write
///             _ => self.rom.write(address, data),
///         };
///     }
/// }
///
/// let mut mapper = Mapper::new();
/// assert_eq!(mapper.read(0x09), 0);
///
/// // write to Mapper register
/// mapper.write(0x2009, 0xEA);
/// assert_eq!(mapper.read(0x09), 0xEA);
/// assert_eq!(mapper.read(0x2009), 0xEA);
/// assert_eq!(mapper.read(0x1F00), 0);
///
/// // address maps outside of register, write ignored by ROM
/// mapper.write(0x1F00, 0xEA);
/// assert_eq!(mapper.read(0x1F00), 0);
///
/// ```
///
/// [bus]: https://en.wikipedia.org/wiki/Bus_(computing)
pub trait Bus {
    /// Reads a byte from the bus at the specified address.
    fn read(&self, address: u16) -> u8;

    /// Writes a byte to the bus at the specified address.
    fn write(&mut self, address: u16, data: u8);
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestStruct {
        data: u8,
    }

    impl Bus for TestStruct {
        fn read(&self, _: u16) -> u8 {
            self.data
        }

        fn write(&mut self, _: u16, data: u8) {
            self.data = data
        }
    }

    #[test]
    fn test_bus() {
        let mut s = TestStruct { data: 0 };

        assert_eq!(s.read(0), 0);
        assert_eq!(s.read(0xFFFF), 0);

        s.write(0xFFFF, 0xEA);
        assert_eq!(s.read(0), 0xEA);
        assert_eq!(s.read(0xFFFF), 0xEA);

        s.write(0x8000, 0xA1);
        assert_eq!(s.read(0), 0xA1);
        assert_eq!(s.read(0xFFFF), 0xA1);
    }
}
