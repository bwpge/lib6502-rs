#[macro_export]
macro_rules! assert_word_eq {
    ($lhs:expr, $rhs:expr) => {
        if $lhs != $rhs {
            panic!(
                "failed byte assertion:\n    \
                expr: `{}`\n  \
                expect: `0x{:04X}`\n  \
                actual: `0x{:04X}`\n",
                stringify!($lhs),
                $rhs,
                $lhs,
            );
        }
    };
}

#[macro_export]
macro_rules! assert_byte_eq {
    ($lhs:expr, $rhs:expr) => {{
        if $lhs != $rhs {
            panic!(
                "failed byte assertion:\n    \
                expr: `{}`\n  \
                expect: `0x{:02X}`\n  \
                actual: `0x{:02X}`\n",
                stringify!($lhs),
                $rhs,
                $lhs,
            );
        }
    }};
}

#[macro_export]
macro_rules! asm {
    ($($addr:literal : $($byte:literal)+ ;)+) => {{
        asm!(size = 0x10000, $($addr : $($byte)+ ;)+)
    }};
    (size = $size:literal, $($addr:literal : $($byte:literal)+ ;)+) => {{
        let mut __program = [0u8; $size];
        $(
            asm!(@expand __program, $addr, $($byte)+);
        )+

        __program
    }};
    (@expand $prog:ident, $addr:literal, $($byte:literal)+) => {
        let mut __i = $addr;
        $(
            $prog[__i] = $byte;
            __i += 1;
        )+
    }
}

#[macro_export]
macro_rules! assert_cpu {
    ($cpu:ident, $($tt:tt = $val:literal),* $(,)?) => {
        $(
            assert_cpu!($cpu, $tt, $val);
        )*
    };
    ($cpu:ident, pc, $pc:literal) => {{
        assert_word_eq!($cpu.pc(), $pc);
    }};
    ($cpu:ident, p, $p:literal) => {{
        assert_byte_eq!($cpu.p(), $p);
    }};
    ($cpu:ident, s, $s:literal) => {{
        assert_word_eq!($cpu.sp_u16(), $s);
    }};
    ($cpu:ident, a, $a:literal) => {{
        assert_byte_eq!($cpu.a(), $a);
    }};
    ($cpu:ident, x, $x:literal) => {{
        assert_byte_eq!($cpu.x(), $x);
    }};
    ($cpu:ident, y, $y:literal) => {{
        assert_byte_eq!($cpu.y(), $y);
    }};
    ($cpu:ident, cyc, $cycles:literal) => {{
        assert_eq!($cpu.cycles(), $cycles);
    }};
}

#[macro_export]
macro_rules! assert_mem_eq {
    ($shared:ident, $addr:literal, $val:literal) => {{
        use ::lib6502::Bus;
        assert_byte_eq!($shared.borrow().read($addr), $val);
    }};
}
