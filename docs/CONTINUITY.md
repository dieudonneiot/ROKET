# Continuity: where the work stands and what comes next

Read this first when picking the work up (on any machine, by anyone).
Update it at every checkpoint commit.

## Last checkpoint

Branch `feature/flight-algorithm` (not merged yet).

Done and tested:

- `firmware/core/src/kf_vertical.c`: Kalman filter on altitude, vertical speed and
  accelerometer bias, with sequential barometer updates and 3-sigma gating.
- `firmware/core/src/flight.c`: flight state machine mapped to the 8 Attachment-7
  status bits (findings log section 14.3).
  - Two criteria per event; no GPS or timer triggers.
  - Barometers are de-weighted between Mach 0.6 and 0.75, and after our own pyro.
  - Each barometer is checked for stuck readings, long rejection, and NaN.
  - The accelerometer is checked for plausibility; after burnout, positive axial force counts as a spike.
- `fc_flight_step_att7()` runs the algorithm directly on referee test frames: the
  altitude field and the altitude derived from pressure are the two barometers.
- `tools/ukb_tester/tests/test_sil.py`: the C core flies the 4 synthetic flights
  and passes every check. Observed results:

  | Data set | Drogue command after apogee | Main command altitude |
  |---|---|---|
  | 1, 4 | 0.91 s | 526 m |
  | 2, 3 | 0.51 s | 526–527 m |

Not done yet on this branch:

1. C unit tests for `kf_vertical.c` and `flight.c` (`firmware/core/tests/`): KF
   convergence and gating, pressure-to-altitude, vertical acceleration on the pad
   (must be about 0), pad bump does not launch, stuck and NaN barometers, apogee
   below the main altitude fires both chutes in sequence.
2. Add the new tests to `firmware/core/tests/CMakeLists.txt`.
3. Check that `docs/THIRD_PARTY.md` and `README.md` are still accurate (status table).
4. Push, open the pull request, and wait for green CI (Linux, Windows, Cortex-M4F, formatting).

## Next steps after this branch

- `ukb-tester sil --dataset N`: a command that runs the C core in the loop without hardware.
- Monte Carlo in RocketPy with the C core in the loop (findings log F-69):
  one campaign per motor curve.
- Attitude estimation (gyro quaternion from the pad) once the IMU is chosen.
- STM32 firmware skeleton once the processor decision D-02 is made.

## Open decisions that block later work

D-01 to D-06 in the findings log (section 14.9), and committee questions Q-01 to Q-16.
The burnout minimum time is off (`burnout_min_time_s = 0`) until Q-03 is answered.

## Working environment

- Linux: `sudo apt install build-essential cmake ninja-build python3-venv gcc-arm-none-eabi`,
  then follow the README (build, ctest, `pip install -e "tools/ukb_tester[dev]"`, pytest
  with `FC_CORE_LIB_DIR=build`).
- Windows: same steps with Visual Studio Build Tools, or zig's clang from pip.
- Commits are authored by `dieudonneiot`, with no co-author trailers.
