mod common;

use crate::common::setup;

#[test]
fn tax() {
    let (_, mut cpu) = setup(asm! {
        0x0000: 0xAA; //       TAX       2 cyc
        0x0001: 0xA9 0x0F; //  LDA #$0F  2 cyc
        0x0003: 0xAA; //       TAX       2 cyc
    });
    assert_cpu!(
        cpu,
        pc = 0x0000,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x24,
        cyc = 7
    );

    cpu.step();
    assert_cpu!(
        cpu,
        pc = 0x0001,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x26,
        cyc = 9
    );

    cpu.step_for(2);
    assert_cpu!(
        cpu,
        pc = 0x0004,
        a = 0x0F,
        x = 0x0F,
        y = 0x00,
        p = 0x24,
        cyc = 13
    );
}

#[test]
fn tay() {
    let (_, mut cpu) = setup(asm! {
        0x0000: 0xA8; //       TAY       2 cyc
        0x0001: 0xA9 0x0F; //  LDA #$0F  2 cyc
        0x0003: 0xA8; //       TAY       2 cyc
    });
    assert_cpu!(
        cpu,
        pc = 0x0000,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x24,
        cyc = 7
    );

    cpu.step();
    assert_cpu!(
        cpu,
        pc = 0x0001,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x26,
        cyc = 9
    );

    cpu.step_for(2);
    assert_cpu!(
        cpu,
        pc = 0x0004,
        a = 0x0F,
        x = 0x00,
        y = 0x0F,
        p = 0x24,
        cyc = 13
    );
}

#[test]
fn tsx() {
    let (_, mut cpu) = setup(asm! {
        0x0000: 0xBA; //  TSX  2 cyc
    });
    assert_cpu!(
        cpu,
        pc = 0x0000,
        s = 0x01FD,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x24,
        cyc = 7
    );

    cpu.step();
    assert_cpu!(
        cpu,
        pc = 0x0001,
        s = 0x01FD,
        a = 0x00,
        x = 0xFD,
        y = 0x00,
        p = 0xA4,
        cyc = 9
    );
}

#[test]
fn txa() {
    let (_, mut cpu) = setup(asm! {
        0x0000: 0x8A; //       TXA       2 cyc
        0x0001: 0xA2 0x0F; //  LDX #$0F  2 cyc
        0x0003: 0x8A; //       TXA       2 cyc
    });
    assert_cpu!(
        cpu,
        pc = 0x0000,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x24,
        cyc = 7
    );

    cpu.step();
    assert_cpu!(
        cpu,
        pc = 0x0001,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x26,
        cyc = 9
    );

    cpu.step();
    assert_cpu!(
        cpu,
        pc = 0x0003,
        a = 0x00,
        x = 0x0F,
        y = 0x00,
        p = 0x24,
        cyc = 11
    );

    cpu.step();
    assert_cpu!(
        cpu,
        pc = 0x0004,
        a = 0x0F,
        x = 0x0F,
        y = 0x00,
        p = 0x24,
        cyc = 13
    );
}

#[test]
fn txs() {
    let (_, mut cpu) = setup(asm! {
        0x0000: 0x9A; //  TXS  2 cyc
    });
    assert_cpu!(cpu, pc = 0x0000, s = 0x01FD, x = 0x00, cyc = 7);

    cpu.step();
    assert_cpu!(cpu, pc = 0x0001, s = 0x0100, x = 0x00, cyc = 9);
}

#[test]
fn tya() {
    let (_, mut cpu) = setup(asm! {
        0x0000: 0x98; //       TYA       2 cyc
        0x0001: 0xA0 0x0F; //  LDY #$0F  2 cyc
        0x0003: 0x98; //       TYA       2 cyc
    });
    assert_cpu!(
        cpu,
        pc = 0x0000,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x24,
        cyc = 7
    );

    cpu.step();
    assert_cpu!(
        cpu,
        pc = 0x0001,
        a = 0x00,
        x = 0x00,
        y = 0x00,
        p = 0x26,
        cyc = 9
    );

    cpu.step_for(2);
    assert_cpu!(
        cpu,
        pc = 0x0004,
        a = 0x0F,
        x = 0x00,
        y = 0x0F,
        p = 0x24,
        cyc = 13
    );
}
