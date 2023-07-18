mod common;

use crate::common::setup;

#[cfg(test)]
mod bcc {
    use super::*;

    #[test]
    fn branch_not_taken_no_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x38; //       SEC       2 cyc
            0x0001: 0x90 0x0E; //  BCC #$0E  2 cyc (branch not taken)
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0003, p = 0x25, cyc = 11);
    }

    #[test]
    fn branch_taken_no_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x18; //       CLC       2 cyc
            0x0001: 0x90 0x0E; //  BCC #$0E  3 cyc (branch taken)
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0011, p = 0x24, cyc = 12);
    }

    #[test]
    fn branch_taken_negtive_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x18; //       CLC       2 cyc
            0x0001: 0x90 0xF2; //  BCC #$F2  4 cyc (branch taken with page cross)
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0xFFF5, p = 0x24, cyc = 13);
    }

    #[test]
    fn branch_taken_positive_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x4C 0xF0 0x00; //  JMP $00F0  3 cyc
            0x00F0: 0x18; //            CLC        2 cyc
            0x00F1: 0x90 0x0F; //       BCC #$0F   4 cyc (branch taken with page cross)
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0102, p = 0x24, cyc = 16);
    }
}

#[cfg(test)]
mod bcs {
    use super::*;

    #[test]
    fn branch_not_taken_no_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x18; //       CLC       2 cyc
            0x0001: 0xB0 0x0E; //  BCC #$0E  2 cyc (branch not taken)
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0003, p = 0x24, cyc = 11);
    }

    #[test]
    fn branch_taken_no_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x38; //       SEC       2 cyc
            0x0001: 0xB0 0x0E; //  BCC #$0E  3 cyc (branch taken)
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0011, p = 0x25, cyc = 12);
    }

    #[test]
    fn branch_taken_negtive_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x38; //       SEC       2 cyc
            0x0001: 0xB0 0xF2; //  BCC #$F2  4 cyc (branch taken with page cross)
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0xFFF5, p = 0x25, cyc = 13);
    }

    #[test]
    fn branch_taken_positive_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x4C 0xF0 0x00; //  JMP $00F0  3 cyc
            0x00F0: 0x38; //            SEC        2 cyc
            0x00F1: 0xB0 0x0F; //       BCC #$0F   4 cyc (branch taken with page cross)
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0102, p = 0x25, cyc = 16);
    }
}
