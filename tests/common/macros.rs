#[macro_export]
macro_rules! assert_word_eq {
    ($lhs:expr, $rhs:expr) => {
        if $lhs != $rhs {
            panic!(
                "failed byte assertion:\n    \
                        expr: `{}`\n  \
                        expect: `0x{:04X}`\n     \
                        got: `0x{:04X}`\n",
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
                expect: `0x{:02X}`\n     \
                got: `0x{:02X}`\n",
                stringify!($lhs),
                $rhs,
                $lhs,
            );
        }
    }};
}

#[macro_export]
macro_rules! asm {
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
