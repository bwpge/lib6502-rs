mod common;

use std::{cell::RefCell, rc::Rc};

use lib6502::{Bus, Cpu};

use crate::common::Ram;

fn setup(prog: [u8; 0x10000]) -> (Rc<RefCell<Ram>>, Cpu<Ram>) {
    let ram = Rc::new(RefCell::new(Ram::new().load_at(&prog, 0)));
    let mut cpu = Cpu::new(ram.clone());
    cpu.step();

    (ram, cpu)
}

#[test]
fn load_and_store() {
    let (ram, mut cpu) = setup(asm! {
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
    });
    assert_state!(
        cpu,
        pc = 0xC000,
        p = 0x24,
        s = 0x01FD,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        cyc = 7,
    );

    cpu.step_for(14);
    assert_state!(
        cpu,
        pc = 0xC60C,
        p = 0x26,
        s = 0x01FD,
        a = 0x42,
        x = 0x00,
        y = 0x69,
        cyc = 42,
    );

    assert_mem_eq!(ram, 0x0000, 0x01);
    assert_mem_eq!(ram, 0x0010, 0x01);
    assert_mem_eq!(ram, 0x0011, 0x01);
    assert_mem_eq!(ram, 0x0042, 0x42);
    assert_mem_eq!(ram, 0xF000, 0x69);
}

#[test]
fn zpg_store() {
    let (ram, mut cpu) = setup(asm! {
        size = 0x10000,
        //                     RESET        7 cyc
        0xC000: 0xA9 0xFF; //  LDA #$FF     2 cyc
        0xC002: 0xA2 0x10; //  LDX #$10     2 cyc
        0xC004: 0xA0 0x01; //  LDY #$01     2 cyc
        0xC006: 0x95 0xA0; //  STA $A0,X    4 cyc
        0xC008: 0x85 0xCC; //  STA $CC      3 cyc
        0xC00A: 0x96 0xCC; //  STX $CC,Y    4 cyc
        0xFFFC: 0x00 0xC0; //  program start $C000
    });
    assert_state!(
        cpu,
        pc = 0xC000,
        p = 0x24,
        s = 0x01FD,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        cyc = 7,
    );

    cpu.step_for(6);
    assert_state!(
        cpu,
        pc = 0xC00C,
        p = 0x24,
        s = 0x01FD,
        a = 0xFF,
        x = 0x10,
        y = 0x01,
        cyc = 24,
    );

    assert_mem_eq!(ram, 0x00B0, 0xFF);
    assert_mem_eq!(ram, 0x00CC, 0xFF);
    assert_mem_eq!(ram, 0x00CD, 0x10);
}
