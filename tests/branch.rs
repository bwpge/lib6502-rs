mod common;

use crate::common::setup;

macro_rules! test_branch {
    ($name:ident, $prog:expr, step = $step:literal, pc = $pc:literal, p = $p:literal, cyc = $cyc:literal) => {
        #[test]
        fn $name() {
            let (_, mut cpu) = setup($prog);
            assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

            cpu.step_for($step);
            assert_cpu!(cpu, pc = $pc, p = $p, cyc = $cyc);
        }
    };
}

#[cfg(test)]
mod bcc {
    use super::*;

    test_branch!(
        not_taken_no_cross,
        asm! { 0x0000: 0x38 0x90 0x0E; },
        step = 2,
        pc = 0x0003,
        p = 0x25,
        cyc = 11
    );

    test_branch!(
        taken_no_cross,
        asm! { 0x0000: 0x18 0x90 0x0E; },
        step = 2,
        pc = 0x0011,
        p = 0x24,
        cyc = 12
    );

    test_branch!(
        taken_cross_neg,
        asm! { 0x0000: 0x18 0x90 0xF2; },
        step = 2,
        pc = 0xFFF5,
        p = 0x24,
        cyc = 13
    );

    test_branch!(
        taken_cross_pos,
        asm! {
            0x0000: 0x4C 0xF0 0x00;
            0x00F0: 0x18 0x90 0x0F;
        },
        step = 3,
        pc = 0x0102,
        p = 0x24,
        cyc = 16
    );
}

#[cfg(test)]
mod bcs {
    use super::*;

    test_branch!(
        not_taken_no_cross,
        asm! { 0x0000: 0x18 0xB0 0x0E; },
        step = 2,
        pc = 0x0003,
        p = 0x24,
        cyc = 11
    );

    test_branch!(
        taken_no_cross,
        asm! { 0x0000: 0x38 0xB0 0x0E; },
        step = 2,
        pc = 0x0011,
        p = 0x25,
        cyc = 12
    );

    test_branch!(
        taken_cross_neg,
        asm! { 0x0000: 0x38 0xB0 0xF2; },
        step = 2,
        pc = 0xFFF5,
        p = 0x25,
        cyc = 13
    );

    test_branch!(
        taken_cross_pos,
        asm! {
            0x0000: 0x4C 0xF0 0x00;
            0x00F0: 0x38 0xB0 0x0F;
        },
        step = 3,
        pc = 0x0102,
        p = 0x25,
        cyc = 16
    );
}

#[cfg(test)]
mod beq {
    use super::*;

    test_branch!(
        not_taken_no_cross,
        asm! { 0x0000: 0xA9 0x01 0xF0 0x0E; },
        step = 2,
        pc = 0x0004,
        p = 0x24,
        cyc = 11
    );

    test_branch!(
        taken_no_cross,
        asm! { 0x0000: 0xA9 0x00 0xF0 0x0E; },
        step = 2,
        pc = 0x0012,
        p = 0x26,
        cyc = 12
    );

    test_branch!(
        taken_cross_neg,
        asm! { 0x0000: 0xA9 0x00 0xF0 0xF2; },
        step = 2,
        pc = 0xFFF6,
        p = 0x26,
        cyc = 13
    );

    test_branch!(
        taken_cross_pos,
        asm! {
            0x0000: 0x4C 0xF0 0x00;
            0x00F0: 0xA9 0x00 0xF0 0x0F;
        },
        step = 3,
        pc = 0x0103,
        p = 0x26,
        cyc = 16
    );
}

#[cfg(test)]
mod bne {
    use super::*;

    test_branch!(
        not_taken_no_cross,
        asm! { 0x0000: 0xA9 0x00 0xD0 0x0E; },
        step = 2,
        pc = 0x0004,
        p = 0x26,
        cyc = 11
    );

    test_branch!(
        taken_no_cross,
        asm! { 0x0000: 0xA9 0x01 0xD0 0x0E; },
        step = 2,
        pc = 0x0012,
        p = 0x24,
        cyc = 12
    );

    test_branch!(
        taken_cross_neg,
        asm! { 0x0000: 0xA9 0x01 0xD0 0xF2; },
        step = 2,
        pc = 0xFFF6,
        p = 0x24,
        cyc = 13
    );

    test_branch!(
        taken_cross_pos,
        asm! {
            0x0000: 0x4C 0xF0 0x00;
            0x00F0: 0xA9 0x01 0xD0 0x0F;
        },
        step = 3,
        pc = 0x0103,
        p = 0x24,
        cyc = 16
    );
}

#[cfg(test)]
mod bpl {
    use super::*;

    test_branch!(
        not_taken_no_cross,
        asm! { 0x0000: 0xA9 0xFF 0x10 0x0E; },
        step = 2,
        pc = 0x0004,
        p = 0xA4,
        cyc = 11
    );

    test_branch!(
        taken_no_cross,
        asm! { 0x0000: 0xA9 0x0F 0x10 0x0E; },
        step = 2,
        pc = 0x0012,
        p = 0x24,
        cyc = 12
    );

    test_branch!(
        taken_cross_neg,
        asm! { 0x0000: 0xA9 0x0F 0x10 0xF2; },
        step = 2,
        pc = 0xFFF6,
        p = 0x24,
        cyc = 13
    );

    test_branch!(
        taken_cross_pos,
        asm! {
            0x0000: 0x4C 0xF0 0x00;
            0x00F0: 0xA9 0x0F 0x10 0x0F;
        },
        step = 3,
        pc = 0x0103,
        p = 0x24,
        cyc = 16
    );
}

#[cfg(test)]
mod bmi {
    use super::*;

    test_branch!(
        not_taken_no_cross,
        asm! { 0x0000: 0xA9 0x0F 0x30 0x0E; },
        step = 2,
        pc = 0x0004,
        p = 0x24,
        cyc = 11
    );

    test_branch!(
        taken_no_cross,
        asm! { 0x0000: 0xA9 0xFF 0x30 0x0E; },
        step = 2,
        pc = 0x0012,
        p = 0xA4,
        cyc = 12
    );

    test_branch!(
        taken_cross_neg,
        asm! { 0x0000: 0xA9 0xFF 0x30 0xF2; },
        step = 2,
        pc = 0xFFF6,
        p = 0xA4,
        cyc = 13
    );

    test_branch!(
        taken_cross_pos,
        asm! {
            0x0000: 0x4C 0xF0 0x00;
            0x00F0: 0xA9 0xFF 0x30 0x0F;
        },
        step = 3,
        pc = 0x0103,
        p = 0xA4,
        cyc = 16
    );
}

#[cfg(test)]
mod bvc {
    use super::*;

    test_branch!(
        not_taken_no_cross,
        asm! {
            0x0000: 0xA9 0x70; //  LDA #$70  2 cyc
            0x0002: 0x69 0x11; //  ADC #$11  2 cyc
            0x0004: 0x50 0x0E; //  BVC #$0E  2 cyc (not taken)
        },
        step = 3,
        pc = 0x0006,
        p = 0xE4,
        cyc = 13
    );

    test_branch!(
        taken_no_cross,
        asm! { 0x0000: 0x50 0x0E; },
        step = 1,
        pc = 0x0010,
        p = 0x24,
        cyc = 10
    );

    test_branch!(
        taken_cross_neg,
        asm! {
            0x0000: 0x50 0xF2;
        },
        step = 1,
        pc = 0xFFF4,
        p = 0x24,
        cyc = 11
    );

    test_branch!(
        taken_cross_pos,
        asm! {
            0x0000: 0x4C 0xF0 0x00;
            0x00F0: 0x50 0x0F;
        },
        step = 2,
        pc = 0x0101,
        p = 0x24,
        cyc = 14
    );
}

#[cfg(test)]
mod bvs {
    use super::*;

    test_branch!(
        not_taken_no_cross,
        asm! { 0x0000: 0x70 0x0E; },
        step = 1,
        pc = 0x0002,
        p = 0x24,
        cyc = 9
    );

    test_branch!(
        taken_no_cross,
        asm! {
            0x0000: 0xA9 0x70; //  LDA #$70  2 cyc
            0x0002: 0x69 0x11; //  ADC #$11  2 cyc
            0x0004: 0x70 0x0E; //  BVS #$0E  3 cyc (taken)
        },
        step = 3,
        pc = 0x0014,
        p = 0xE4,
        cyc = 14
    );

    test_branch!(
        taken_cross_neg,
        asm! {
            0x0000: 0xA9 0x70; //  LDA #$70  2 cyc
            0x0002: 0x69 0x11; //  ADC #$11  2 cyc
            0x0004: 0x70 0xF2; //  BVS #$F2  4 cyc (taken with cross)
        },
        step = 3,
        pc = 0xFFF8,
        p = 0xE4,
        cyc = 15
    );

    test_branch!(
        taken_cross_pos,
        asm! {
            0x0000: 0xA9 0x70; //       LDA #$70   2 cyc
            0x0002: 0x69 0x11; //       ADC #$11   2 cyc
            0x0004: 0x4C 0xF0 0x00; //  JMP $00F0  3 cyc
            0x00F0: 0x70 0x0F; //       BVS #$F0   4 cyc (taken with cross)
        },
        step = 4,
        pc = 0x0101,
        p = 0xE4,
        cyc = 18
    );
}
