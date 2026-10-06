cmake_minimum_required(VERSION 3.20)

project(teknofest_a4_avionics
    VERSION 0.1.0
    DESCRIPTION "TEKNOFEST A4 International: flight core, unit tests and tools"
    LANGUAGES C)

# Host builds get the tests and the Python-facing shared library; cross builds
# (cmake/arm-none-eabi-cortex-m4.cmake) only need the static flight core.
if(CMAKE_CROSSCOMPILING)
    set(FC_HOST_DEFAULT OFF)
else()
    set(FC_HOST_DEFAULT ON)
endif()
option(FC_BUILD_TESTS "Build the host unit tests of the flight core" ${FC_HOST_DEFAULT})
option(FC_BUILD_SHARED "Build the flight core as a shared library for the Python tools" ${FC_HOST_DEFAULT})

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FcWarnings)

add_subdirectory(firmware/core)

if(FC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(firmware/core/tests)
endif()
