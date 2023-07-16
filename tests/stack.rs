mod common;

use crate::common::setup;

#[test]
fn push_pop_a() {
    let (ram, mut cpu) = setup(asm! {
        0x0000: 0xA9 0x7C; //  LDA #$7C  2 cyc
        0x0002: 0x48; //       PHA       3 cyc
        0x0003: 0xA9 0x00; //  LDA #$00  2 cyc
        0x0005: 0x68; //       PLA       4 cyc
    });
    assert_cpu!(cpu, pc = 0x0000, p = 0x24, s = 0x01FD, a = 0x00, cyc = 7);
    assert_mem_eq!(ram, 0x01FD, 0x00);

    cpu.step(); // LDA $7C
    assert_cpu!(cpu, pc = 0x0002, p = 0x24, s = 0x01FD, a = 0x7C, cyc = 9);
    assert_mem_eq!(ram, 0x01FD, 0x00);

    cpu.step(); // PHA
    assert_cpu!(cpu, pc = 0x0003, p = 0x24, s = 0x01FC, a = 0x7C, cyc = 12);
    assert_mem_eq!(ram, 0x01FD, 0x7C);

    cpu.step(); // LDA $00
    assert_cpu!(cpu, pc = 0x0005, p = 0x26, s = 0x01FC, a = 0x00, cyc = 14);
    assert_mem_eq!(ram, 0x01FD, 0x7C);

    cpu.step(); // PLA
    assert_cpu!(cpu, pc = 0x0006, p = 0x24, s = 0x01FD, a = 0x7C, cyc = 18);
    assert_mem_eq!(ram, 0x01FD, 0x7C);
}

#[test]
fn push_pop_s() {
    let (ram, mut cpu) = setup(asm! {
        0x0000: 0x58 0x08; //  CLI PHP   5 cyc
        0x0002: 0xA9 0xFF; //  LDA #$FF  2 cyc
        0x0004: 0x48; //       PHA       3 cyc
        0x0005: 0x28; //       PLP       4 cyc
    });
    assert_cpu!(cpu, pc = 0x0000, p = 0x24, s = 0x01FD, a = 0x00, cyc = 7);
    assert_mem_eq!(ram, 0x01FD, 0x00);

    cpu.step_for(2); // CLI PHP
    assert_cpu!(cpu, pc = 0x0002, p = 0x20, s = 0x01FC, a = 0x00, cyc = 12);
    assert_mem_eq!(ram, 0x01FD, 0x30); // bit 5 and B should be set

    cpu.step(); // LDA #$FF
    assert_cpu!(cpu, pc = 0x0004, p = 0xA0, s = 0x01FC, a = 0xFF, cyc = 14);
    assert_mem_eq!(ram, 0x01FC, 0x00);

    cpu.step(); // PHA
    assert_cpu!(cpu, pc = 0x0005, p = 0xA0, s = 0x01FB, a = 0xFF, cyc = 17);
    assert_mem_eq!(ram, 0x01FC, 0xFF);

    cpu.step(); // PLP
    assert_cpu!(cpu, pc = 0x0006, p = 0xEF, s = 0x01FC, a = 0xFF, cyc = 21);
}
