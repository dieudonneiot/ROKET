set(FC_CORE_SOURCES
    src/att7.c
    src/fc_info.c
    src/flight.c
    src/kf_vertical.c
)

add_library(fc_core STATIC ${FC_CORE_SOURCES})
target_include_directories(fc_core PUBLIC include)
fc_enable_strict_warnings(fc_core)

# Some C libraries keep roundf/fabsf in a separate libm.
find_library(FC_MATH_LIBRARY m)
if(FC_MATH_LIBRARY)
    target_link_libraries(fc_core PUBLIC ${FC_MATH_LIBRARY})
endif()

if(FC_BUILD_SHARED)
    # Loaded by the Python tools through ctypes (tools/ukb_tester).
    add_library(fc_core_shared SHARED ${FC_CORE_SOURCES})
    target_include_directories(fc_core_shared PUBLIC include)
    target_compile_definitions(fc_core_shared PRIVATE FC_BUILD_SHARED)
    set_target_properties(fc_core_shared PROPERTIES C_VISIBILITY_PRESET hidden)
    fc_enable_strict_warnings(fc_core_shared)
    if(FC_MATH_LIBRARY)
        target_link_libraries(fc_core_shared PRIVATE ${FC_MATH_LIBRARY})
    endif()
endif()
