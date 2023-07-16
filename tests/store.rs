mod common;

use crate::common::setup;

mod store {
    use super::*;

    #[test]
    fn sta_abs() {
        let (ram, mut cpu) = setup(asm! {
            0x0000: 0xA9 0x79; //       LDA #$79   2 cyc
            0x0002: 0x8D 0x55 0xC5; //  STA $C555  4 cyc
        });
        assert_mem_eq!(ram, 0xC555, 0x00);
        assert_cpu!(cpu, pc = 0x0000, p = 0x24, a = 0x00, cyc = 7);

        cpu.step_for(2);
        assert_cpu!(cpu, pc = 0x0005, p = 0x24, a = 0x79, cyc = 13);
        assert_mem_eq!(ram, 0xC555, 0x79);
    }
}
