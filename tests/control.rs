mod common;

use crate::common::setup;

#[cfg(test)]
mod jmp {
    use super::*;

    #[test]
    fn absolute() {
        let (_, mut cpu) = setup(asm! {
            0x0000: 0x4C 0x1A 0xDF; //  JMP $DF1A  3 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, cyc = 7);

        cpu.step();
        assert_cpu!(cpu, pc = 0xDF1A, cyc = 10);
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
mod jsr {
    use super::*;

    #[test]
    fn absolute() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0x4C 0x56 0xC0; //  JMP $C056  3 cyc
            0xC056: 0x20 0x1A 0xCE; //  JSR $CE1A  6 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0xCE1A, cyc = 16);
        assert_mem_eq!(ram, 0x01FD, 0xC0);
        assert_mem_eq!(ram, 0x01FC, 0x58);
    }
}

#[cfg(test)]
mod rts {
    use super::*;

    #[test]
    fn absolute() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0x4C 0x56 0xC0; //  JMP $C056  3 cyc
            0xC056: 0x20 0x1A 0xCE; //  JSR $CE1A  6 cyc
            0xCE1A: 0x60; //            RTS        6 cyc
        });
        assert_cpu!(cpu, pc = 0x0000, cyc = 7);

        cpu.step_for(3);
        assert_cpu!(cpu, pc = 0xC059, cyc = 22);
        assert_mem_eq!(ram, 0x01FD, 0xC0);
        assert_mem_eq!(ram, 0x01FC, 0x58);
    }
}