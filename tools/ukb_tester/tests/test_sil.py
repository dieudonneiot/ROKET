"""Software in the loop: the C flight core flies the four synthetic flights.

Each frame goes through fc_flight_step_att7(), the same path the firmware uses
in the referee Synthetic Flight Test, and the result is scored with the same
rules as a hardware run (launch, drogue within 2 s of apogee, main 400-600 m).
"""

import ctypes

import pytest
from test_parity import CSample, core  # noqa: F401  (fixture re-export)

from ukb_tester.device import SftResult, evaluate_sft
from ukb_tester.scenarios import DATASETS, build_dataset


@pytest.fixture(scope="module")
def flight_core(core):  # noqa: F811
    core.fc_flight_sizeof.restype = ctypes.c_size_t
    core.fc_flight_init.restype = None
    core.fc_flight_init.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    core.fc_flight_step_att7.restype = ctypes.c_uint16
    core.fc_flight_step_att7.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(CSample),
        ctypes.c_float,
    ]
    return core


def fly(lib, number: int) -> SftResult:
    scenario = build_dataset(number)
    state = ctypes.create_string_buffer(lib.fc_flight_sizeof())
    lib.fc_flight_init(state, None)
    result = SftResult(scenario)
    for index, sample in enumerate(scenario.samples):
        bits = lib.fc_flight_step_att7(
            state, ctypes.byref(CSample(*sample.fields())), scenario.period_s
        )
        result.statuses.append((scenario.time_of(index), bits))
    return result


@pytest.mark.parametrize("number", sorted(DATASETS))
def test_flight_core_passes_every_dataset(flight_core, number):
    checks = evaluate_sft(fly(flight_core, number))
    failed = [f"{c.name}: {c.detail}" for c in checks if not c.passed]
    assert not failed, failed
