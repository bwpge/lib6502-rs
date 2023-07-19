mod common;

use crate::common::setup;

#[test]
fn nops() {
    let (_, mut cpu) = setup(asm! {
        //                          RESET        7 cyc
        0xC000: 0xEA 0xEA 0xEA; //  NOP NOP NOP  6 cyc
        0xFFFC: 0x00 0xC0; //       program start $C000
    });

    assert_cpu!(cpu, pc = 0xC000, cyc = 7);

    cpu.step_for(3);
    assert_cpu!(cpu, pc = 0xC003, cyc = 13);
}

#[test]
fn load_and_store() {
    let (ram, mut cpu) = setup(asm! {
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
    assert_cpu!(
        cpu,
        pc = 0xC000,
        s = 0x01FD,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x24,
        cyc = 7,
    );

    cpu.step_for(14);
    assert_cpu!(
        cpu,
        pc = 0xC60C,
        s = 0x01FD,
        a = 0x42,
        x = 0x00,
        y = 0x69,
        p = 0x26,
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
        //                     RESET        7 cyc
        0xC000: 0xA9 0xFF; //  LDA #$FF     2 cyc
        0xC002: 0xA2 0x10; //  LDX #$10     2 cyc
        0xC004: 0xA0 0x01; //  LDY #$01     2 cyc
        0xC006: 0x95 0xA0; //  STA $A0,X    4 cyc
        0xC008: 0x85 0xCC; //  STA $CC      3 cyc
        0xC00A: 0x96 0xCC; //  STX $CC,Y    4 cyc
        0xFFFC: 0x00 0xC0; //  program start $C000
    });
    assert_cpu!(
        cpu,
        pc = 0xC000,
        s = 0x01FD,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x24,
        cyc = 7,
    );

    cpu.step_for(6);
    assert_cpu!(
        cpu,
        pc = 0xC00C,
        s = 0x01FD,
        a = 0xFF,
        x = 0x10,
        y = 0x01,
        p = 0x24,
        cyc = 24,
    );

    assert_mem_eq!(ram, 0x00B0, 0xFF);
    assert_mem_eq!(ram, 0x00CC, 0xFF);
    assert_mem_eq!(ram, 0x00CD, 0x10);
}

#[test]
fn fib_13() {
    let (ram, mut cpu) = setup(asm! {
        0x0000: 0xA0 0x0D; //        LDY #$0D
        0x0002: 0xA9 0x00; //        LDA #$00
        0x0004: 0x85 0xCC; //        STA $CC
        0x0006: 0xA9 0x01; //        LDA #$01
        0x0008: 0xAA; //      L0008  TAX
        0x0009: 0x18; //             CLC
        0x000A: 0x65 0xCC; //        ADC $CC
        0x000C: 0x86 0xCC; //        STX $CC
        0x000E: 0x88; //             DEY
        0x000F: 0xD0 0xF7; //        BNE L0008
        0x0011: 0xEA; //             NOP
    });
    assert_mem_eq!(ram, 0x00CC, 0x00);
    assert_cpu!(cpu, pc = 0x0000, a = 0x00, x = 0x00, cyc = 7);

    cpu.step_for(83);
    // cycles tested in visual6502:
    // -> 204 cycles (+1 for 0 start, +7 for reset) = 212
    assert_cpu!(cpu, pc = 0x0012, a = 0x79, x = 0xE9, cyc = 212);
    assert_mem_eq!(ram, 0x00CC, 0xE9);
}
