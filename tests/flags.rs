mod common;

use crate::common::setup;

#[test]
fn clear() {
    let (_, mut cpu) = setup(asm! {
        0x0000: 0x18 0xD8 0x58 0xB8; // CLC CLD CLI CLV  8 cyc
    });
    assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

    cpu.step_for(4);
    assert_cpu!(cpu, pc = 0x0004, p = 0x20, cyc = 15);
}

#[test]
fn set() {
    let (_, mut cpu) = setup(asm! {
        0x0000: 0x38 0xF8 0x78; // SEC SED SEI  6 cyc
    });
    assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

    cpu.step_for(3);
    assert_cpu!(cpu, pc = 0x0003, p = 0x2D, cyc = 13);
}
