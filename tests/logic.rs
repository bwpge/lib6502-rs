mod common;

use crate::common::setup;

#[cfg(test)]
mod and {
    use super::*;

    #[test]
    fn imm() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0xFF; //  LDA #$FF  2 cyc
            0x0002: 0x29 0x01; //  AND #$01  2 cyc
            0x0004: 0x29 0x80; //  AND #$80  2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, a = 0x00, cyc = 7);
        cpu.step();
        // ensure N flag turned on from loading $FF
        assert_cpu!(cpu, pc = 0x0002, p = 0xA4, a = 0xFF, cyc = 9);
        // $FF & $01 = $01, ensure both Z and N flag are off
        cpu.step();
        assert_cpu!(cpu, pc = 0x0004, p = 0x24, a = 0x01, cyc = 11);
        // $01 & $80 = $00, ensure Z flag turns on
        cpu.step();
        assert_cpu!(cpu, pc = 0x0006, p = 0x26, a = 0x00, cyc = 13);
    }
}

#[cfg(test)]
mod bit {
    use super::*;

    #[test]
    fn zpg_abs() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA9 0xFF; //       LDA #$FF   2 cyc
            0x0002: 0xA2 0x55; //       LDX #$55   2 cyc
            0x0004: 0x86 0xC0; //       STX $C0    3 cyc
            0x0006: 0x24 0xC0; //       BIT $C0    3 cyc
            0x0008: 0x2C 0xCC 0xCC; //  BIT $CCCC  4 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, a = 0x00, cyc = 7);

        cpu.step_for(3); // LDA, LDX, STX
        assert_cpu!(cpu, pc = 0x0006, p = 0x24, a = 0xFF, x = 0x55, cyc = 14);
        assert_mem_eq!(ram, 0x00C0, 0x55);

        cpu.step(); // BIT $C0 => $55
        assert_cpu!(cpu, pc = 0x0008, p = 0x64, a = 0xFF, x = 0x55, cyc = 17);

        cpu.step(); // BIT $CCCC => $00
        assert_cpu!(cpu, pc = 0x000B, p = 0x26, a = 0xFF, x = 0x55, cyc = 21);
    }
}

#[cfg(test)]
mod ora {
    use super::*;

    #[test]
    fn imm() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x0F; //  LDA #$0F  2 cyc
            0x0002: 0x09 0xF0; //  ORA #$F0  2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, a = 0x00, cyc = 7);
        // $0F | $F0 = $FF, ensure N flag turns on
        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, p = 0xA4, a = 0xFF, cyc = 11);
    }
}

#[cfg(test)]
mod eor {
    use super::*;

    #[test]
    fn imm() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0xA9 0xFF; //  LDA #$FF  2 cyc
            0x0002: 0x49 0xFF; //  EOR #$FF  2 cyc
            0x0004: 0x49 0x0F; //  EOR #$0F  2 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, a = 0x00, cyc = 7);
        // $FF ^ $FF = $00, ensure Z flag turns on
        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0004, p = 0x26, a = 0x00, cyc = 11);
        // $00 ^ $0F = $0F, ensure Z flag turns off
        cpu.step();
        assert_cpu!(cpu, pc = 0x0006, p = 0x24, a = 0x0F, cyc = 13);
    }
}
