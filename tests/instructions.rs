mod common;

use lib6502::Bus;

use crate::common::setup;

#[cfg(test)]
mod flags {
    use super::*;

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
}

#[cfg(test)]
mod rmw {
    use super::*;

    #[test]
    fn dec_abs() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xCE 0x01 0x0F; //  DEC $0F01  6 cyc
        });
        ram.borrow_mut().write(0x0F01, 0xCC);
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
        });
        ram.borrow_mut().write(0x01FE, 0xCC);
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
        });
        ram.borrow_mut().write(0x0011, 0xCC);
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
        });
        ram.borrow_mut().write(0x00F1, 0xCC);
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
        });
        ram.borrow_mut().write(0x0201, 0xCC);
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

    #[test]
    fn inc_abs() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xEE 0x01 0x0F; //  INC $0F01  6 cyc
        });
        ram.borrow_mut().write(0x0F01, 0xCC);
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
        });
        ram.borrow_mut().write(0x01FE, 0xCC);
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
        });
        ram.borrow_mut().write(0x0201, 0xCC);
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

#[cfg(test)]
mod jmp {
    use super::*;

    #[test]
    fn absolute() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x4C 0x1A 0xDF; //  JMP $DF1A  3 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, x = 0x00, y = 0x00, cyc = 7);

        cpu.step();
        assert_cpu!(cpu, pc = 0xDF1A, a = 0x00, x = 0x00, y = 0x00, cyc = 10);
    }

    #[test]
    fn indirect() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA9 0xC0; //       LDA #$C0     2 cyc
            0x0002: 0x85 0x11; //       STA $11      3 cyc
            0x0004: 0x6C 0x10 0x00; //  JMP ($0010)  5 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, a = 0x00, x = 0x00, y = 0x00, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(
            cpu,
            pc = 0xC000,
            s = 0x01FD,
            a = 0xC0,
            x = 0x00,
            y = 0x00,
            p = 0xA4,
            cyc = 17,
        );

        assert_mem_eq!(ram, 0x0010, 0x00);
        assert_mem_eq!(ram, 0x0011, 0xC0);
    }
}

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
        });
        ram.borrow_mut().write(0x00FE, 0xCD);
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
        });
        ram.borrow_mut().write(0x00F5, 0x0C);
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
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xAD 0xCC 0x08; //  LDA $08CC  4 cyc
        });
        ram.borrow_mut().write(0x08CC, 0x55);
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
        });
        ram.borrow_mut().write(0x5691, 0x0C);
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
        });
        ram.borrow_mut().write(0x5691, 0x0C);
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
        });
        ram.borrow_mut().write(0x001F, 0x99);
        ram.borrow_mut().write(0x0020, 0x99);
        ram.borrow_mut().write(0x9999, 0x05);
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
    fn indirect_y() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA0 0x01; //  LDY #$01     2 cyc
            0x0002: 0xB1 0xFF; //  LDA ($FF),Y  6 cyc (from crossing page)
        });
        ram.borrow_mut().write(0x00FF, 0x99);
        ram.borrow_mut().write(0x0100, 0x99);
        ram.borrow_mut().write(0x9999, 0x05);
        assert_mem_eq!(ram, 0x00FF, 0x99);
        assert_mem_eq!(ram, 0x0100, 0x99);
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
            x = 0x00,
            y = 0x01,
            p = 0x24,
            cyc = 15
        );
    }
}

#[cfg(test)]
mod transfer {
    use super::*;

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
            0x0000: 0x9A; //  TAY  2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, s = 0x01FD, x = 0x00, cyc = 7);

        cpu.step();
        assert_cpu!(cpu, pc = 0x0001, s = 0x0100, x = 0x00, cyc = 9);
    }

    #[test]
    fn tya() {
        // TODO
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
}
