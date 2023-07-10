mod common;

use std::{cell::RefCell, rc::Rc};

use lib6502::{Bus, Cpu, StatusFlag};

use crate::common::Ram;

#[test]
fn load_and_store() {
    let prog = asm! {
        size = 0x10000,
        //                          RESET       7 cyc
        0xC000: 0x4C 0xF5 0xC5; //  JMP $C5F5   3 cyc
        0xC5F5: 0xA2 0x01; //       LDX #$01    2 cyc
        0xC5F7: 0x86 0x00; //       STX $00     3 cyc
        0xC5F9: 0x86 0x10; //       STX $10     3 cyc
        0xC5FB: 0x86 0x11; //       STX $11     3 cyc
        0xC5FD: 0xEA; //            NOP         2 cyc
        0xC5FE: 0xEA; //            NOP         2 cyc
        0xC5FF: 0xA9 0x42; //       LDA #$42    2 cyc
        0xC601: 0x85 0x42; //       STA $42     3 cyc
        0xC603: 0xEA; //            NOP         2 cyc
        0xC604: 0xA0 0x69; //       LDY #$69    2 cyc
        0xC606: 0x8C 0x00 0xF0; //  STY $F000   4 cyc
        0xC609: 0xEA; //            NOP         2 cyc
        0xC60A: 0xA2 0x00; //       LDX #$00    2 cyc
        0xFFFC: 0x00 0xC0; //       program start $C000
    };
    let ram = Rc::new(RefCell::new(Ram::new().load_at(&prog, 0)));
    let mut cpu = Cpu::new(ram.clone());

    // execute reset
    cpu.step();

    // validate start state
    assert_word_eq!(cpu.pc(), 0xC000);
    assert_byte_eq!(cpu.a(), 0x00);
    assert_byte_eq!(cpu.x(), 0x00);
    assert_byte_eq!(cpu.y(), 0x00);
    assert_byte_eq!(cpu.p(), 0x24);
    for i in 0..=0xBFFF {
        assert_byte_eq!(ram.borrow().read(i), 0x00);
    }

    // execute program (14 instructions)
    cpu.step_for(14);

    // validate final state
    assert_word_eq!(cpu.pc(), 0xC60C);
    assert_eq!(cpu.get_flag(StatusFlag::Z), true);
    assert_byte_eq!(cpu.a(), 0x42);
    assert_byte_eq!(cpu.x(), 0x00);
    assert_byte_eq!(cpu.y(), 0x69);
    assert_eq!(cpu.cycles(), 42);
    assert_byte_eq!(ram.borrow().read(0x0000), 0x01);
    assert_byte_eq!(ram.borrow().read(0x0010), 0x01);
    assert_byte_eq!(ram.borrow().read(0x0011), 0x01);
    assert_byte_eq!(ram.borrow().read(0x0042), 0x42);
    assert_byte_eq!(ram.borrow().read(0xF000), 0x69);
}
