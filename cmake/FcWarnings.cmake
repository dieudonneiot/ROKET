# Strict warnings for our own flight code; third-party sources keep their defaults.
# -ffp-contract=off keeps host results bit-identical to the Cortex-M4F build
# (no fused multiply-add), so simulations test exactly what flies.
function(fc_enable_strict_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /fp:precise)
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Werror
            -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion -Wstrict-prototypes
            -ffp-contract=off)
    endif()
endfunction()
