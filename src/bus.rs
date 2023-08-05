//! Emulation of a data exchange.
//!
//! From an electronics perspective, the [`Bus`] trait abstracts the concept of
//! devices being connected to the bus and addressable as one "unit", rather than
//! modeling the physical wires of the bus itself.
//!
//! # Example
//!
//! A basic implementation for read-only memory (ROM):
//!
//! ```
//! use lib6502::Bus;
//!
//! // 16K read-only memory
//! struct Rom {
//!     mem: Box<[u8; 0x4000]>
//! }
//!
//! impl Rom {
//!     fn new() -> Self {
//!         Self { mem: Box::new([0u8; 0x4000]) }
//!     }
//! }
//!
//! impl Bus for Rom {
//!     fn read(&self, address: u16) -> u8 {
//!         let addr = (address as usize) % self.mem.len();
//!         self.mem[address as usize]
//!     }
//!
//!     fn write(&mut self, address: u16, data: u8) {
//!         // write does nothing to a ROM
//!     }
//! }
//!
//! let mut rom = Rom::new();
//! assert_eq!(rom.read(0x30A1), 0);
//!
//! // write into the void
//! rom.write(0xA1, 0xFF);
//! assert_eq!(rom.read(0x30A1), 0);
//! ```
//! # Shared Buses
//!
//! Often times, a bus will need to be shared between system components, such
//! as a CPU and input device. Exclusive (mutable) references cannot be shared
//! between components (e.g., for `write`), thus this requires some interior
//! mutability pattern.
//!
//! The [`Bus`] trait provides implementations for common wrapper types to deal
//! with this issue, such as [`RefCell`] and [`Mutex`], as well as smart-pointers
//! used in conjunction with these types, such as [`Rc<RefCell>`] and
//! [`Arc<Mutex>`]).
//!
//! ```
//! use lib6502::Bus;
//!
//! use std::{cell::RefCell, rc::Rc};
//!
//! // sample implementation of 1-byte of memory
//! struct Mem(u8);
//!
//! impl Bus for Mem {
//!     fn read(&self, _: u16) -> u8 {
//!         self.0
//!     }
//!
//!     fn write(&mut self, _: u16, data: u8) {
//!         self.0 = data
//!     }
//! }
//!
//! // sample component that need exclusive access to the bus
//! struct Component1<T: Bus> {
//!     bus: T,
//! }
//!
//! impl<T: Bus> Component1<T> {
//!     fn write_bus(&mut self, addr: u16, data: u8) {
//!         self.bus.write(addr, data);
//!     }
//! }
//!
//! // second component that also needs exclusive access
//! struct Component2<T: Bus> {
//!     bus: T,
//! }
//!
//! impl<T: Bus> Component2<T> {
//!     fn write_bus(&mut self, addr: u16, data: u8) {
//!         self.bus.write(addr, data);
//!     }
//! }
//!
//! let mem = Mem(42);
//! assert_eq!(mem.read(0), 42);
//!
//! // create a shared bus for components
//! let shared = Rc::new(RefCell::new(mem));
//! let mut c1 = Component1 { bus: shared.clone() };
//! let mut c2 = Component2 { bus: shared.clone() };
//!
//! // ensure all buses are reading the same memory
//! assert_eq!(shared.read(0), 42);
//! assert_eq!(c1.bus.read(0), 42);
//! assert_eq!(c2.bus.read(0), 42);
//!
//! // write with the first component
//! c1.write_bus(0, 0xCD);
//! assert_eq!(shared.read(0), 0xCD);
//! assert_eq!(c1.bus.read(0), 0xCD);
//! assert_eq!(c2.bus.read(0), 0xCD);
//!
//! // write with the second component
//! c2.write_bus(0, 0xEA);
//! assert_eq!(shared.read(0), 0xEA);
//! assert_eq!(c1.bus.read(0), 0xEA);
//! assert_eq!(c2.bus.read(0), 0xEA);
//! ```
//!

use std::{
    cell::RefCell,
    rc::Rc,
    sync::{Arc, Mutex, RwLock},
};

/// A type that emulates a bus for data exchange.
///
/// See the [module-level documentation][crate::bus] for more details.
///
/// Implementors of this trait may freely decide what to do with any `read` or
/// `write` calls (e.g., read-only buses may do nothing in `write` calls).
/// Implementors must also guarantee that all 16-bit addresses do not cause
/// a panic in `read` or `write` calls.
pub trait Bus {
    /// Reads a byte from the bus at the specified address.
    fn read(&self, address: u16) -> u8;

    /// Writes a byte to the bus at the specified address.
    fn write(&mut self, address: u16, data: u8);
}

impl<B: Bus> Bus for RefCell<B> {
    fn read(&self, address: u16) -> u8 {
        self.borrow().read(address)
    }

    fn write(&mut self, address: u16, data: u8) {
        self.borrow_mut().write(address, data)
    }
}

impl<B: Bus> Bus for RwLock<B> {
    fn read(&self, address: u16) -> u8 {
        self.read().expect("poisoned lock").read(address)
    }

    fn write(&mut self, address: u16, data: u8) {
        RwLock::write(self)
            .expect("poisoned lock")
            .write(address, data);
    }
}

impl<B: Bus> Bus for Mutex<B> {
    fn read(&self, address: u16) -> u8 {
        self.lock().expect("already locked").read(address)
    }

    fn write(&mut self, address: u16, data: u8) {
        self.get_mut().expect("already locked").write(address, data);
    }
}

impl<B: Bus> Bus for Rc<RefCell<B>> {
    fn read(&self, address: u16) -> u8 {
        Bus::read(&**self, address)
    }

    fn write(&mut self, address: u16, data: u8) {
        RefCell::borrow_mut(&*self).write(address, data)
    }
}

impl<B: Bus> Bus for Arc<RwLock<B>> {
    fn read(&self, address: u16) -> u8 {
        Bus::read(&**self, address)
    }

    fn write(&mut self, address: u16, data: u8) {
        RwLock::write(&*self)
            .expect("poisoned lock")
            .write(address, data)
    }
}

impl<B: Bus> Bus for Arc<Mutex<B>> {
    fn read(&self, address: u16) -> u8 {
        Bus::read(&**self, address)
    }

    fn write(&mut self, address: u16, data: u8) {
        Mutex::lock(&*self)
            .expect("already locked")
            .write(address, data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Ram {
        mem: [u8; 0x10000],
    }

    impl Bus for Ram {
        fn read(&self, address: u16) -> u8 {
            self.mem[address as usize]
        }

        fn write(&mut self, address: u16, data: u8) {
            self.mem[address as usize] = data;
        }
    }

    #[allow(unused)]
    fn ram() -> Ram {
        Ram { mem: [0; 0x10000] }
    }

    macro_rules! make_test {
        ($name:ident, $bus:expr) => {
            #[test]
            fn $name() {
                let mut ram = Ram { mem: [0; 0x10000] };
                assert_eq!(ram.read(0xFFFF), 0);

                ram.write(0xFFFF, 0xCD);
                assert_eq!(ram.read(0xFFFF), 0xCD);
            }
        };
    }

    make_test!(struct_bus, ram());
    make_test!(refcell_bus, RefCell::new(ram()));
    make_test!(rwlock_bus, RwLock::new(ram()));
    make_test!(mutex_bus, Mutex::new(ram()));
    make_test!(rc_refcell_bus, Rc::new(RefCell::new(ram())));
    make_test!(arc_rwlock_bus, Arc::new(RwLock::new(ram())));
    make_test!(arc_mutex_bus, Arc::new(Mutex::new(ram())));
}
