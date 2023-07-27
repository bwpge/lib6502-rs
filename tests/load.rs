mod common;

use crate::common::setup;

#[cfg(test)]
mod lda {
    use super::*;

    #[test]
    fn immediate() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0xFE; //  LDA #$FE  2 cyc
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
            pc = 0x0002,
            a = 0xFE,
            x = 0x00,
            y = 0x00,
            p = 0xA4,
            cyc = 9
        );
    }

    #[test]
    fn zeropage() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA5 0xFE; //  LDA $FE  3 cyc
            0x00FE: 0xCD;
        });
        assert_mem_eq!(ram, 0x00FE, 0xCD);
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
            pc = 0x0002,
            a = 0xCD,
            x = 0x00,
            y = 0x00,
            p = 0xA4,
            cyc = 10
        );
    }

    #[test]
    fn zeropage_x() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA2 0x05; //  LDX #$05   2 cyc
            0x0002: 0xB5 0xF0; //  LDA $F0,X  4 cyc
            0x00F5: 0x0C;
        });
        assert_mem_eq!(ram, 0x00F5, 0x0C);
        assert_cpu!(
            cpu,
            pc = 0x0000,
            a = 0x00,
            x = 0x00,
            y = 0x00,
            p = 0x24,
            cyc = 7
        );

        cpu.step_for(2);
        assert_cpu!(
            cpu,
            pc = 0x0004,
            a = 0x0C,
            x = 0x05,
            y = 0x00,
            p = 0x24,
            cyc = 13
        );
    }

    #[test]
    fn absolute() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xAD 0xCC 0x08; //  LDA $08CC  4 cyc
            0x08CC: 0x55;
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
            pc = 0x0003,
            a = 0x55,
            x = 0x00,
            y = 0x00,
            p = 0x24,
            cyc = 11
        );
    }

    #[test]
    fn absolute_x() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA2 0xA1; //       LDX #$A1     2 cyc
            0x0002: 0xBD 0xF0 0x55; //  LDA $55F0,X  5 cyc (from crossing page)
            0x5691: 0x0C;
        });
        assert_mem_eq!(ram, 0x5691, 0x0C);
        assert_cpu!(
            cpu,
            pc = 0x0000,
            a = 0x00,
            x = 0x00,
            y = 0x00,
            p = 0x24,
            cyc = 7
        );

        cpu.step_for(2);
        assert_cpu!(
            cpu,
            pc = 0x0005,
            a = 0x0C,
            x = 0xA1,
            y = 0x00,
            p = 0x24,
            cyc = 14
        );
    }

    #[test]
    fn absolute_y() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA0 0xA1; //       LDY #$A1     2 cyc
            0x0002: 0xB9 0xF0 0x55; //  LDA $55F0,Y  5 cyc (from crossing page)
            0x5691: 0x0C;
        });
        assert_mem_eq!(ram, 0x5691, 0x0C);
        assert_cpu!(
            cpu,
            pc = 0x0000,
            a = 0x00,
            x = 0x00,
            y = 0x00,
            p = 0x24,
            cyc = 7
        );

        cpu.step_for(2);
        assert_cpu!(
            cpu,
            pc = 0x0005,
            a = 0x0C,
            x = 0x00,
            y = 0xA1,
            p = 0x24,
            cyc = 14
        );
    }

    #[test]
    fn indirect_x() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA2 0x0F; //  LDX #$0F     2 cyc
            0x0002: 0xA1 0x10; //  LDA ($10,X)  6 cyc
            0x0020: 0x99;
            0x001F: 0x99;
            0x9999: 0x05;
        });
        assert_mem_eq!(ram, 0x001F, 0x99);
        assert_mem_eq!(ram, 0x0020, 0x99);
        assert_mem_eq!(ram, 0x9999, 0x05);
        assert_cpu!(
            cpu,
            pc = 0x0000,
            a = 0x00,
            x = 0x00,
            y = 0x00,
            p = 0x24,
            cyc = 7
        );

        cpu.step_for(2);
        assert_cpu!(
            cpu,
            pc = 0x0004,
            a = 0x05,
            x = 0x0F,
            y = 0x00,
            p = 0x24,
            cyc = 15
        );
    }

    #[test]
    fn indirect_y_cross() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA0 0x01; //  LDY #$01     2 cyc
            0x0002: 0xB1 0xFF; //  LDA ($FF),Y  6 cyc (from crossing page)
            0x00FF: 0xFF;
            0xA100: 0x05;
        });
        assert_mem_eq!(ram, 0x00FF, 0xFF);
        assert_mem_eq!(ram, 0xA100, 0x05);
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, y = 0x00, p = 0x24, cyc = 7);

        // LDA ($FF),Y should read FF as the low byte on the zero page, then
        // it should wrap around to 00 and read A0 as the high byte. with Y=1,
        // the offset will cross pages to A100, costing an extra cycle.
        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, a = 0x05, y = 0x01, p = 0x24, cyc = 15);
    }
}
