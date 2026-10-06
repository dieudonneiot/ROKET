import math

import pytest

from ukb_tester.scenarios import DATASETS, SITE_ALTITUDE_M, build_dataset, isa_pressure_mbar


@pytest.mark.parametrize("number", sorted(DATASETS))
def test_datasets_have_matching_lengths(number):
    scenario = build_dataset(number)
    assert scenario.period_s == pytest.approx(0.1)
    assert len(scenario.samples) == len(scenario.agl_m) > 1000


def test_nominal_flight_is_plausible_for_a4():
    truth = build_dataset(1).truth
    assert (
        truth.ignition_s
        < truth.rail_exit_s
        < truth.burnout_s
        < truth.apogee_s
        < truth.drogue_open_s
        < truth.main_open_s
        < truth.landing_s
    )
    assert 2438.0 < truth.apogee_agl_m < 6000.0  # above the 8,000 ft minimum (spec 3.2.11)
    assert truth.rail_exit_speed_mps >= 25.0  # spec 3.3.4.8


def test_nominal_pad_values():
    scenario = build_dataset(1)
    first = scenario.samples[0]
    assert first.altitude_m == pytest.approx(SITE_ALTITUDE_M)
    assert first.pressure_mbar == pytest.approx(isa_pressure_mbar(SITE_ALTITUDE_M))
    assert first.accel_x_mps2 == pytest.approx(9.80665 * math.sin(math.radians(85.0)))
    assert first.angle_y_deg == pytest.approx(85.0)


def test_main_opens_at_the_main_altitude():
    scenario = build_dataset(1)
    # Within one 10 Hz sample (1.5 m at 15 m/s) of the 500 m trigger.
    assert scenario.agl_at(scenario.truth.main_open_s) == pytest.approx(500.0, abs=2.0)


def test_abnormal_set_contains_the_documented_faults():
    scenario = build_dataset(4)
    truth = scenario.truth
    assert any(math.isnan(s.pressure_mbar) for s in scenario.samples)
    pad = [s for i, s in enumerate(scenario.samples) if scenario.time_of(i) < truth.ignition_s]
    assert max(s.accel_x_mps2 for s in pad) == pytest.approx(60.0)


def test_datasets_are_reproducible():
    assert build_dataset(3).samples == build_dataset(3).samples


def test_unknown_dataset_is_rejected():
    with pytest.raises(ValueError):
        build_dataset(9)
