"""Bit-for-bit parity between this Python codec and the C flight core.

Needs the shared library built by CMake (target fc_core_shared). Set FC_CORE_LIB
to its path, or FC_CORE_LIB_DIR to a build directory to search. The tests are
skipped when it cannot be found, unless FC_REQUIRE_CORE_LIB=1 (as in CI).
"""

import ctypes
import glob
import os
import random
import struct

import pytest

from ukb_tester import protocol as p

PATTERNS = ("**/*fc_core_shared.dll", "**/libfc_core_shared.so", "**/libfc_core_shared.dylib")


def _find_library() -> str | None:
    explicit = os.environ.get("FC_CORE_LIB")
    if explicit:
        return explicit
    root = os.environ.get("FC_CORE_LIB_DIR")
    if not root:
        return None
    for pattern in PATTERNS:
        matches = sorted(glob.glob(os.path.join(root, pattern), recursive=True))
        if matches:
            return matches[0]
    return None


class CConfig(ctypes.Structure):
    _fields_ = [
        ("tx_checksum", ctypes.c_uint8),
        ("float_order", ctypes.c_uint8),
        ("round_2dp", ctypes.c_uint8),
        ("strict_rx_checksum", ctypes.c_uint8),
        ("status_low_first", ctypes.c_uint8),
    ]


FIELDS = [
    "altitude_m", "pressure_mbar", "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
    "angle_x_deg", "angle_y_deg", "angle_z_deg",
]  # fmt: skip


class CSample(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in FIELDS]


class CEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint8),
        ("command", ctypes.c_uint8),
        ("checksum_ok", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("sample", CSample),
    ]


@pytest.fixture(scope="module")
def core():
    path = _find_library()
    if path is None:
        if os.environ.get("FC_REQUIRE_CORE_LIB") == "1":
            pytest.fail("flight-core shared library not found (FC_CORE_LIB / FC_CORE_LIB_DIR)")
        pytest.skip("flight-core shared library not built")
    lib = ctypes.CDLL(os.path.abspath(path))
    u8, f32, size = ctypes.c_uint8, ctypes.c_float, ctypes.c_size_t
    buffer = ctypes.POINTER(ctypes.c_uint8)
    signatures = {
        "fc_abi_version": (ctypes.c_uint32, []),
        "att7_round_2dp": (f32, [f32]),
        "att7_checksum": (u8, [buffer, size, u8]),
        "att7_encode_command": (size, [u8, u8, buffer]),
        "att7_encode_data": (size, [ctypes.POINTER(CConfig), ctypes.POINTER(CSample), buffer]),
        "att7_encode_status": (size, [ctypes.POINTER(CConfig), ctypes.c_uint16, buffer]),
        "att7_parser_init": (None, [ctypes.c_void_p]),
        "att7_parser_push": (
            u8,
            [ctypes.c_void_p, ctypes.POINTER(CConfig), u8, ctypes.c_uint32, ctypes.POINTER(CEvent)],
        ),
    }
    for name in ("config", "sample", "event", "parser", "session"):
        signatures[f"att7_sizeof_{name}"] = (size, [])
    for name, (restype, argtypes) in signatures.items():
        function = getattr(lib, name)
        function.restype, function.argtypes = restype, argtypes
    return lib


def _c_config(config: p.Config) -> CConfig:
    return CConfig(
        int(config.tx_checksum),
        int(config.float_order),
        int(config.round_2dp),
        int(config.strict_rx_checksum),
        int(config.status_low_first),
    )


def _call_encoder(function, *args, length: int) -> bytes:
    out = (ctypes.c_uint8 * length)()
    assert function(*args, out) == length
    return bytes(out)


def _random_sample(rng: random.Random) -> p.Sample:
    return p.Sample(*(rng.uniform(-2000.0, 6000.0) for _ in FIELDS))


CONFIGS = [
    p.Config(),
    p.Config(float_order=p.FloatOrder.LITTLE_ENDIAN),
    p.Config(tx_checksum=p.Checksum.WITHOUT_HEADER, round_2dp=False),
    p.Config(status_low_first=False, round_2dp=False, float_order=p.FloatOrder.LITTLE_ENDIAN),
]


def test_binary_interface(core):
    assert core.fc_abi_version() == 1
    assert core.att7_sizeof_config() == ctypes.sizeof(CConfig)
    assert core.att7_sizeof_sample() == ctypes.sizeof(CSample)
    assert core.att7_sizeof_event() == ctypes.sizeof(CEvent)


def test_rounding_parity(core):
    rng = random.Random(7)
    values = [1.23456, -1.235, 2.675, 0.005, -0.004999, 1013.256, 3.0e8, 0.0, -0.0]
    values += [rng.uniform(-10000.0, 10000.0) for _ in range(5000)]
    for value in values:
        expected = struct.pack("<f", p.round_2dp(value))
        assert struct.pack("<f", core.att7_round_2dp(value)) == expected, value


@pytest.mark.parametrize("config", CONFIGS)
def test_encoder_parity(core, config):
    rng = random.Random(42)
    c_config = _c_config(config)
    for _ in range(500):
        sample = _random_sample(rng)
        c_sample = CSample(*sample.fields())
        c_frame = _call_encoder(
            core.att7_encode_data,
            ctypes.byref(c_config),
            ctypes.byref(c_sample),
            length=p.DATA_LEN,
        )
        assert c_frame == p.encode_data(sample, config)
    for status in (0, 0x14, 0xFF, 0x0123, 0xFFFF):
        c_frame = _call_encoder(
            core.att7_encode_status, ctypes.byref(c_config), status, length=p.STATUS_LEN
        )
        assert c_frame == p.encode_status(status, config)


@pytest.mark.parametrize("style", list(p.CommandChecksum))
def test_command_parity(core, style):
    for command in (p.CMD_START_SMT, p.CMD_START_SFT, p.CMD_STOP):
        c_frame = _call_encoder(core.att7_encode_command, command, int(style), length=p.COMMAND_LEN)
        assert c_frame == p.encode_command(command, style)


@pytest.mark.parametrize("config", CONFIGS[:2])
def test_c_parser_reads_python_frames(core, config):
    rng = random.Random(3)
    c_config = _c_config(config)
    parser = ctypes.create_string_buffer(core.att7_sizeof_parser())
    core.att7_parser_init(parser)
    event = CEvent()
    for i in range(200):
        sample = _random_sample(rng)
        frame = p.encode_data(sample, config)
        got = [
            core.att7_parser_push(parser, ctypes.byref(c_config), b, i * 100, ctypes.byref(event))
            for b in frame
        ]
        assert got[-1] == 1 and event.type == 2 and event.checksum_ok == 1
        decoded = p.decode_data(frame, config)
        assert [getattr(event.sample, name) for name in FIELDS] == list(decoded.fields())
