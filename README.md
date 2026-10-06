# TEKNOFEST 2027 · A4 International: avionics and ground software

Software for our rocket in the TEKNOFEST Rocket Competition, A4 International
category: the team-built flight computer (I-FCC), the tools that test it, and
later the payload firmware, ground station and simulations.

The rules, constraints and design decisions behind this code are in
[TEKNOFEST_A4_Findings_Log.docx](TEKNOFEST_A4_Findings_Log.docx). Code and tests cite
its finding IDs (for example `F-29`) and spec clauses (for example `3.3.6.32`), so
every rule-driven behaviour can be traced back to its source.

## Status

| Part | State |
|---|---|
| Referee test protocol (Attachment-7): frame codec, parser, test-mode session | Done, with unit tests |
| `ukb-tester`: emulator of the referee test device, 4 synthetic flights, scoring | Done, with tests |
| CI: Linux and Windows builds, Cortex-M4F cross-build, formatting | Done |
| Flight algorithm (Kalman filter, state machine, fault handling) | Next |
| STM32 firmware for the I-FCC | After the processor decision (D-02) |
| Ground station, payload firmware, RocketPy simulations | Planned |

## Repository layout

```
CMakeLists.txt            host build: flight core, shared library for Python, unit tests
cmake/                    compiler warnings; Cortex-M4F cross-compilation toolchain
firmware/core/            portable C11 flight core, no hardware access
  include/fc/att7.h       referee test protocol: frames, parser, test-mode session
  tests/                  Unity unit tests
firmware/third_party/     vendored code (Unity test framework)
tools/ukb_tester/         Python emulator of the referee test device
docs/THIRD_PARTY.md       third-party components, licences and rules
.github/workflows/ci.yml  continuous integration
```

## Build and test

Requirements: CMake 3.20 or newer, a C compiler (GCC, Clang or MSVC) and Python 3.10 or newer.

```sh
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure

python -m pip install -e "tools/ukb_tester[dev]"
FC_CORE_LIB_DIR=build python -m pytest tools/ukb_tester   # PowerShell: $env:FC_CORE_LIB_DIR="build"
```

The Python tests include a bit-for-bit comparison with the C core. They load the
shared library found under `FC_CORE_LIB_DIR` and are skipped if it has not been built.

Cross-build the core for the STM32 (Cortex-M4F) with the Arm GNU toolchain:

```sh
cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-cortex-m4.cmake
cmake --build build-arm
```

## Testing the flight computer with `ukb-tester`

Connect a USB-to-RS232 adapter to the I-FCC's D-SUB9 port (RX, TX and GND only; F-20), then:

```sh
ukb-tester list-ports
ukb-tester smt --port COM5 --seconds 10    # Sensor Monitoring Test: checks the 10 Hz sensor stream
ukb-tester sft --port COM5 --dataset 1     # Synthetic Flight Test: plays a flight and scores the result
ukb-tester export --dataset 4 --csv ds4.csv
```

Data sets 1–4 stand in for the committee's own, which arrive only after CDR (F-27):
1 nominal, 2 barometer noise, 3 barometer and accelerometer noise, 4 abnormal inputs
(spikes, frozen pressure, a NaN, a bump on the pad). `sft` checks that launch is
detected, the drogue is commanded within 2 s after apogee, the main between 400 and
600 m above ground, and that status bits stay set.

The defaults follow the conventions past teams used successfully (F-29): big-endian
floats and checksums that include the header. To probe other conventions, use
`--float-order le`, `--checksum no-header` or `--command-checksum table1`.

## Engineering rules

- **Flight core:** C11, no dynamic memory, no recursion, single precision only (the
  Cortex-M4F has a single-precision FPU), fixed-width types, warnings treated as errors.
- **Traceability:** every rule-driven behaviour cites its finding ID or spec clause
  in code and tests.
- **Third-party code:** no GPL or AGPL; register every component in
  [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md). The I-FCC flight algorithm must be our
  own (spec 3.3.6.13).
- **Changes** go through pull requests with green CI.
