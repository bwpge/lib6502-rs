mod common;

use crate::common::setup;

#[cfg(test)]
mod decrement {
    use super::*;

    #[test]
    fn dec_abs() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xCE 0x01 0x0F; //  DEC $0F01  6 cyc
            0x0F01: 0xCC;
        });
        assert_mem_eq!(ram, 0x0F01, 0xCC);
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step();
        assert_mem_eq!(ram, 0x0F01, 0xCB);
        assert_cpu!(cpu, pc = 0x0003, p = 0xA4, cyc = 13);
    }

    #[test]
    fn dec_abx() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA2 0x02; //       LDX #$02     2 cyc
            0x0002: 0xDE 0xFC 0x01; //  DEC $01FF,X  7 cyc
            0x01FE: 0xCC;
        });
        assert_mem_eq!(ram, 0x01FE, 0xCC);
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_mem_eq!(ram, 0x01FE, 0xCB);
        assert_cpu!(cpu, pc = 0x0005, p = 0xA4, cyc = 16);
    }

    #[test]
    fn dec_zpg() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xC6 0x11; //  DEC $11  5 cyc
            0x0011: 0xCC;
        });
        assert_mem_eq!(ram, 0x0011, 0xCC);
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step();
        assert_mem_eq!(ram, 0x0011, 0xCB);
        assert_cpu!(cpu, pc = 0x0002, p = 0xA4, cyc = 12);
    }

    #[test]
    fn dec_zpx_wrap() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA2 0xF2; //  LDX #$02   2 cyc
            0x0002: 0xD6 0xFF; //  DEC $FF,X  6 cyc
            0x00F1: 0xCC;
        });
        assert_mem_eq!(ram, 0x00F1, 0xCC);
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_mem_eq!(ram, 0x00F1, 0xCB);
        assert_cpu!(cpu, pc = 0x0004, p = 0xA4, cyc = 15);
    }

    #[test]
    fn dec_abx_page_cross() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA2 0x02; //       LDX #$02     2 cyc
            0x0002: 0xDE 0xFF 0x01; //  DEC $01FF,X  7 cyc
            0x0201: 0xCC;
        });
        assert_mem_eq!(ram, 0x0201, 0xCC);
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_mem_eq!(ram, 0x0201, 0xCB);
        assert_cpu!(cpu, pc = 0x0005, p = 0xA4, cyc = 16);
    }

    #[test]
    fn dex() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA2 0x00; //  LDX #$00  2 cyc
            0x0002: 0xCA; //       DEX       2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, x = 0x00, cyc = 7);
        cpu.step();
        assert_cpu!(cpu, pc = 0x0002, p = 0x26, x = 0x00, cyc = 9);
        cpu.step();
        assert_cpu!(cpu, pc = 0x0003, p = 0xA4, x = 0xFF, cyc = 11);
    }

    #[test]
    fn dey() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA0 0x00; //  LDY #$00  2 cyc
            0x0002: 0x88; //       DEY       2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, y = 0x00, cyc = 7);
        cpu.step();
        assert_cpu!(cpu, pc = 0x0002, p = 0x26, y = 0x00, cyc = 9);
        cpu.step();
        assert_cpu!(cpu, pc = 0x0003, p = 0xA4, y = 0xFF, cyc = 11);
    }
}

#[cfg(test)]
mod increment {
    use super::*;

    #[test]
    fn inc_abs() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xEE 0x01 0x0F; //  INC $0F01  6 cyc
            0x0F01: 0xCC;
        });
        assert_mem_eq!(ram, 0x0F01, 0xCC);
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step();
        assert_mem_eq!(ram, 0x0F01, 0xCD);
        assert_cpu!(cpu, pc = 0x0003, p = 0xA4, cyc = 13);
    }

    #[test]
    fn inc_abx() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA2 0x02; //       LDX #$02     2 cyc
            0x0002: 0xFE 0xFC 0x01; //  DEC $01FF,X  7 cyc
            0x01FE: 0xCC;
        });
        assert_mem_eq!(ram, 0x01FE, 0xCC);
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_mem_eq!(ram, 0x01FE, 0xCD);
        assert_cpu!(cpu, pc = 0x0005, p = 0xA4, cyc = 16);
    }

    #[test]
    fn inc_abx_page_cross() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA2 0x02; //       LDX #$02     2 cyc
            0x0002: 0xFE 0xFF 0x01; //  DEC $01FF,X  7 cyc
            0x0201: 0xCC;
        });
        assert_mem_eq!(ram, 0x0201, 0xCC);
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, cyc = 7);

        cpu.step_for(2);
        assert_mem_eq!(ram, 0x0201, 0xCD);
        assert_cpu!(cpu, pc = 0x0005, p = 0xA4, cyc = 16);
    }

    #[test]
    fn inx() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA2 0xFF; //  LDX #$FF  2 cyc
            0x0002: 0xE8; //       INX       2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, x = 0x00, cyc = 7);
        cpu.step();
        assert_cpu!(cpu, pc = 0x0002, p = 0xA4, x = 0xFF, cyc = 9);
        cpu.step();
        assert_cpu!(cpu, pc = 0x0003, p = 0x26, x = 0x00, cyc = 11);
    }

    #[test]
    fn iny() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA0 0xFF; //  LDY #$FF  2 cyc
            0x0002: 0xC8; //       INY       2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, y = 0x00, cyc = 7);
        cpu.step();
        assert_cpu!(cpu, pc = 0x0002, p = 0xA4, y = 0xFF, cyc = 9);
        cpu.step();
        assert_cpu!(cpu, pc = 0x0003, p = 0x26, y = 0x00, cyc = 11);
    }
}
