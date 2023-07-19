mod common;

use crate::common::setup;

#[cfg(test)]
mod adc {
    use super::*;

    #[test]
    fn immediate() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42  2 cyc
            0x0002: 0x69 0x42; //  ADC #$42  2 cyc
            0x0004: 0x69 0x7F; //  ADC #$7F  2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, a = 0x84, p = 0xE4, cyc = 11);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0006, a = 0x03, p = 0x25, cyc = 13);
    }
}

#[cfg(test)]
mod sbc {
    use super::*;

    #[test]
    fn immediate() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42  2 cyc
            0x0002: 0xE9 0x42; //  SBC #$42  2 cyc
            0x0004: 0xE9 0x7F; //  SBC #$7F  2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, a = 0xFF, p = 0xA4, cyc = 11);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0006, a = 0x7F, p = 0x65, cyc = 13);
    }
}

#[cfg(test)]
mod cmp {
    use super::*;

    #[test]
    fn immediate() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42  2 cyc
            0x0002: 0xC9 0x42; //  CMP #$42  2 cyc
            0x0004: 0xC9 0x00; //  CMP #$00  2 cyc
            0x0006: 0xC9 0xFF; //  CMP #$FF  2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, a = 0x42, p = 0x27, cyc = 11);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0006, a = 0x42, p = 0x25, cyc = 13);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0008, a = 0x42, p = 0x24, cyc = 15);
    }
}

#[cfg(test)]
mod cpx {
    use super::*;

    #[test]
    fn immediate() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA2 0x42; //  LDA #$42  2 cyc
            0x0002: 0xE0 0x42; //  CMP #$42  2 cyc
            0x0004: 0xE0 0x00; //  CMP #$00  2 cyc
            0x0006: 0xE0 0xFF; //  CMP #$FF  2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, x = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, x = 0x42, p = 0x27, cyc = 11);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0006, x = 0x42, p = 0x25, cyc = 13);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0008, x = 0x42, p = 0x24, cyc = 15);
    }
}

#[cfg(test)]
mod cpy {
    use super::*;

    #[test]
    fn immediate() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA0 0x42; //  LDA #$42  2 cyc
            0x0002: 0xC0 0x42; //  CMP #$42  2 cyc
            0x0004: 0xC0 0x00; //  CMP #$00  2 cyc
            0x0006: 0xC0 0xFF; //  CMP #$FF  2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, y = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, y = 0x42, p = 0x27, cyc = 11);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0006, y = 0x42, p = 0x25, cyc = 13);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0008, y = 0x42, p = 0x24, cyc = 15);
    }
}
