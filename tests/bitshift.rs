mod common;

use crate::common::setup;

#[cfg(test)]
mod shift {
    use super::*;

    #[test]
    fn asl_a() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0xAA; //  LDA #$AA  2 cyc
            0x0002: 0x0A; //       ASL A     2 cyc
            0x0003: 0x0A; //       ASL A     2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, a = 0x00, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0003, p = 0x25, a = 0x54, cyc = 11);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0004, p = 0xA4, a = 0xA8, cyc = 13);
    }

    #[test]
    fn lsr_a() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0xAA; //  LDA #$AA  2 cyc
            0x0002: 0x4A; //       LSR A     2 cyc
            0x0003: 0x4A; //       LSR A     2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, a = 0x00, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0003, p = 0x24, a = 0x55, cyc = 11);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0004, p = 0x25, a = 0x2A, cyc = 13);
    }
}

#[cfg(test)]
mod rotate {
    use super::*;

    #[test]
    fn rol_a() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x08; //  LDA #$08  2 cyc
            0x0002: 0x38; //       SEC       2 cyc
            0x0003: 0x2A; //       ROL A     2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, a = 0x00, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0003, p = 0x25, a = 0x08, cyc = 11);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0004, p = 0x24, a = 0x11, cyc = 13);
    }

    #[test]
    fn ror_a() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x08; //  LDA #$08  2 cyc
            0x0002: 0x38; //       SEC       2 cyc
            0x0003: 0x6A; //       ROR A     2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, a = 0x00, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0003, p = 0x25, a = 0x08, cyc = 11);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0004, p = 0xA4, a = 0x84, cyc = 13);
    }
}
