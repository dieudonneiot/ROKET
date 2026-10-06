# Third-party component register

Every component we did not write ourselves is listed here, with its licence and
what it is used for. The CDR asks teams to declare third-party parts, and the
rules forbid any third-party flight algorithm on our flight computer (I-FCC).

## Rules

- **No third-party flight algorithm on the I-FCC.** Spec 3.3.6.13 disqualifies a
  team whose I-FCC uses a COTS flight algorithm. Third-party code is limited to
  drivers, hardware abstraction, test frameworks and tools (findings log F-15, F-91).
- **No GPL or AGPL code** in firmware or tools. Read such projects (for example
  CATS or Altus Metrum AltOS) for ideas only. Do not copy or translate them line by line.
- **Repositories without a licence are all rights reserved:** read only.
- **MIT, BSD and Apache code** may be used, keeping its copyright and licence text
  (and NOTICE for Apache).
- Add a row here in the same pull request that introduces a component.

## Components in use

| Component | Version | Licence | Used for | Where | In flight firmware? |
|---|---|---|---|---|---|
| Unity (ThrowTheSwitch) | 2.7.0 | MIT | C unit tests | Vendored in `firmware/third_party/unity` (see below) | No |
| pyserial | ≥ 3.5 | BSD-3-Clause | Serial port access in `ukb-tester` | PyPI dependency | No |
| pytest | ≥ 8 | MIT | Python tests | Development dependency | No |
| ruff | ≥ 0.6 | MIT | Python linting and formatting | Development dependency | No |
| clang-format | 23.1.2 | Apache-2.0 with LLVM exception | C formatting check in CI | CI only | No |
| TEKNOFEST M1850 thrust curve (`Teknofest26_M1850_Rev1.eng`) | 2026 | Organizer data | Synthetic flights in `ukb-tester` | `tools/ukb_tester/src/ukb_tester/scenarios.py` | No |

### Unity provenance

- Files: `unity.c`, `unity.h`, `unity_internals.h` and `LICENSE.txt`, copied
  unmodified from the v2.7.0 release.
- Source: https://github.com/ThrowTheSwitch/Unity/archive/refs/tags/v2.7.0.tar.gz
  (SHA-256 `e84eb301ca7967831e68b1728f911e87fa2d345d8ddb64f897bc2f2ee24a321c`).
- Vendored rather than downloaded at build time so the tests also run offline,
  for example in the integration area at the range. To upgrade, replace all four
  files from a new release and update this section.

## Planned, not yet used

| Component | Licence | Intended use |
|---|---|---|
| STM32CubeF4 HAL/LL | BSD-3-Clause | Hardware access in the I-FCC firmware |
| CMSIS | Apache-2.0 | Cortex-M core support |
| RocketPy | MIT | Monte Carlo flight simulation |
| OpenRocket | GPL-3.0 | Mandatory design and simulation tool; used as an external program only, never linked or copied |
