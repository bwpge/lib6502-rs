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

    #[test]
    fn zeropage() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42  2 cyc
            0x0002: 0x65 0x42; //  ADC $42   3 cyc
            0x0042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, a = 0x0E, p = 0x25, cyc = 12);
    }

    #[test]
    fn zeropage_x() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42   2 cyc
            0x0002: 0xA2 0x02; //  LDX #$42   2 cyc
            0x0004: 0x75 0x40; //  ADC $40,X  4 cyc
            0x0042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0006, a = 0x0E, p = 0x25, cyc = 15);
    }

    #[test]
    fn absolute() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42   2 cyc
            0x0002: 0x6D 0x42 0xC0; //  ADC $C042  4 cyc
            0xC042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0005, a = 0x0E, p = 0x25, cyc = 13);
    }

    #[test]
    fn absolute_x() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42     2 cyc
            0x0002: 0xA2 0x42; //       LDX #$42     2 cyc
            0x0004: 0x7D 0x00 0xC0; //  ADC $C000,X  4 cyc
            0xC042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0007, a = 0x0E, p = 0x25, cyc = 15);
    }

    #[test]
    fn absolute_x_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42     2 cyc
            0x0002: 0xA2 0x01; //       LDX #$01     2 cyc
            0x0004: 0x7D 0xFF 0xC0; //  ADC $C0FF,X  5 cyc (from crossing page)
            0xC100: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0007, a = 0x0E, p = 0x25, cyc = 16);
    }

    #[test]
    fn absolute_y() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42     2 cyc
            0x0002: 0xA0 0x42; //       LDY #$42     2 cyc
            0x0004: 0x79 0x00 0xC0; //  ADC $C000,Y  4 cyc
            0xC042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0007, a = 0x0E, p = 0x25, cyc = 15);
    }

    #[test]
    fn absolute_y_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42     2 cyc
            0x0002: 0xA0 0x01; //       LDY #$01     2 cyc
            0x0004: 0x79 0xFF 0xC0; //  ADC $C0FF,Y  5 cyc (from crossing page)
            0xC100: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0007, a = 0x0E, p = 0x25, cyc = 16);
    }

    #[test]
    fn indirect_x() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42     2 cyc
            0x0002: 0xA2 0x42; //  LDX #$42     2 cyc
            0x0004: 0x61 0x05; //  ADC ($05,X)  6 cyc
            0x0047: 0x11 0x55;
            0x5511: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0006, a = 0x0E, p = 0x25, cyc = 17);
    }

    #[test]
    fn indirect_y() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42     2 cyc
            0x0002: 0xA0 0x42; //  LDY #$42     2 cyc
            0x0004: 0x71 0x42; //  ADC ($05),Y  5 cyc
            0x0042: 0x11 0x55;
            0x5553: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0006, a = 0x0E, p = 0x25, cyc = 16);
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

    #[test]
    fn zeropage() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42  2 cyc
            0x0002: 0xE5 0x42; //  SBC $42   3 cyc
            0x0042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, a = 0x75, p = 0x24, cyc = 12);
    }

    #[test]
    fn zeropage_x() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42   2 cyc
            0x0002: 0xA2 0x02; //  LDX #$42   2 cyc
            0x0004: 0xF5 0x40; //  SBC $40,X  4 cyc
            0x0042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0006, a = 0x75, p = 0x24, cyc = 15);
    }

    #[test]
    fn absolute() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42   2 cyc
            0x0002: 0xED 0x42 0xC0; //  ADC $C042  4 cyc
            0xC042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0005, a = 0x75, p = 0x24, cyc = 13);
    }

    #[test]
    fn absolute_x() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42     2 cyc
            0x0002: 0xA2 0x42; //       LDX #$42     2 cyc
            0x0004: 0xFD 0x00 0xC0; //  ADC $C000,X  4 cyc
            0xC042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0007, a = 0x75, p = 0x24, cyc = 15);
    }

    #[test]
    fn absolute_x_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42     2 cyc
            0x0002: 0xA2 0x01; //       LDX #$01     2 cyc
            0x0004: 0xFD 0xFF 0xC0; //  ADC $C0FF,X  5 cyc (from crossing page)
            0xC100: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0007, a = 0x75, p = 0x24, cyc = 16);
    }

    #[test]
    fn absolute_y() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42     2 cyc
            0x0002: 0xA0 0x42; //       LDY #$42     2 cyc
            0x0004: 0xF9 0x00 0xC0; //  ADC $C000,Y  4 cyc
            0xC042: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0007, a = 0x75, p = 0x24, cyc = 15);
    }

    #[test]
    fn absolute_y_cross() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //       LDA #$42     2 cyc
            0x0002: 0xA0 0x01; //       LDY #$01     2 cyc
            0x0004: 0xF9 0xFF 0xC0; //  ADC $C0FF,Y  5 cyc (from crossing page)
            0xC100: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0007, a = 0x75, p = 0x24, cyc = 16);
    }

    #[test]
    fn indirect_x() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42     2 cyc
            0x0002: 0xA2 0x42; //  LDX #$42     2 cyc
            0x0004: 0xE1 0x05; //  SBC ($05,X)  6 cyc
            0x0047: 0x11 0x55;
            0x5511: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0006, a = 0x75, p = 0x24, cyc = 17);
    }

    #[test]
    fn indirect_y() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x42; //  LDA #$42     2 cyc
            0x0002: 0xA0 0x42; //  LDY #$42     2 cyc
            0x0004: 0xF1 0x42; //  SBC ($05),Y  5 cyc
            0x0042: 0x11 0x55;
            0x5553: 0xCC;
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, p = 0x24, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0x0006, a = 0x75, p = 0x24, cyc = 16);
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
