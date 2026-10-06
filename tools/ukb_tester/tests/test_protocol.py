import math

import pytest

from ukb_tester import protocol as p

# Same golden vectors as firmware/core/tests/test_att7.c, computed with Python's struct module.
GOLDEN_SAMPLE = p.Sample(1234.5, 1013.25, 98.0, -0.5, 9.75, 1.25, 85.5, -180.0)
GOLDEN_BE = bytes.fromhex(
    "ab449a5000447d500042c40000bf000000411c00003fa0000042ab0000c3340000cf0d0a"
)
GOLDEN_LE = bytes.fromhex(
    "ab00509a4400507d440000c442000000bf00001c410000a03f0000ab42000034c3cf0d0a"
)


def test_encode_data_golden_both_byte_orders():
    assert p.encode_data(GOLDEN_SAMPLE) == GOLDEN_BE
    le = p.Config(float_order=p.FloatOrder.LITTLE_ENDIAN)
    assert p.encode_data(GOLDEN_SAMPLE, le) == GOLDEN_LE
    assert p.decode_data(GOLDEN_LE, le) == GOLDEN_SAMPLE


def test_encode_data_checksum_without_header():
    frame = p.encode_data(GOLDEN_SAMPLE, p.Config(tx_checksum=p.Checksum.WITHOUT_HEADER))
    assert frame[p.DATA_CHECKSUM_INDEX] == 0x24
    assert frame[: p.DATA_CHECKSUM_INDEX] == GOLDEN_BE[: p.DATA_CHECKSUM_INDEX]


@pytest.mark.parametrize(
    ("command", "header_sum", "table1"),
    [(p.CMD_START_SMT, 0xCA, 0x8C), (p.CMD_START_SFT, 0xCC, 0x8E), (p.CMD_STOP, 0xCE, 0x90)],
)
def test_commands_both_checksum_styles(command, header_sum, table1):
    assert p.encode_command(command) == bytes((0xAA, command, header_sum, 0x0D, 0x0A))
    assert p.encode_command(command, p.CommandChecksum.TABLE1) == bytes(
        (0xAA, command, table1, 0x0D, 0x0A)
    )


def test_status_frames():
    frame = p.encode_status(p.Status.MIN_ALTITUDE | p.Status.DESCENDING)
    assert frame == bytes.fromhex("aa1400be0d0a")
    assert p.decode_status(frame) == 0x14
    high_first = p.Config(status_low_first=False)
    assert p.encode_status(0x14, high_first) == bytes.fromhex("aa0014be0d0a")
    assert p.decode_status(p.encode_status(0x0123, high_first), high_first) == 0x0123


@pytest.mark.parametrize(
    ("value", "expected"),
    [(1.23456, 1.23), (-1.235, -1.24), (2.675, 2.68), (0.005, 0.01), (1013.256, 1013.26)],
)
def test_round_2dp_matches_firmware(value, expected):
    assert p.round_2dp(value) == p.to_float32(expected)


def test_round_2dp_edge_cases():
    zero = p.round_2dp(-0.004999)
    assert zero == 0.0 and math.copysign(1.0, zero) == 1.0
    assert math.isnan(p.round_2dp(math.nan))
    assert p.round_2dp(math.inf) == math.inf
    assert p.round_2dp(3.0e8) == 3.0e8


def test_parser_reads_status_and_data_through_noise_and_splits():
    stream = b"\x00\x13" + p.encode_status(0x21) + b"\xff" + GOLDEN_BE
    parser = p.FcFrameParser()
    frames = parser.feed(stream[:10], now=0.0) + parser.feed(stream[10:], now=0.01)
    assert [f.kind for f in frames] == ["status", "data"]
    assert frames[0].status == 0x21
    assert frames[1].sample == GOLDEN_SAMPLE
    assert all(f.checksum_rule == p.Checksum.WITH_HEADER for f in frames)


def test_parser_identifies_checksum_rule_and_strict_mode():
    without = p.encode_status(0x14, p.Config(tx_checksum=p.Checksum.WITHOUT_HEADER))
    assert p.FcFrameParser().feed(without, 0.0)[0].checksum_rule == p.Checksum.WITHOUT_HEADER

    bad = bytes.fromhex("aa1400000d0a")
    lenient = p.FcFrameParser().feed(bad, 0.0)
    assert len(lenient) == 1 and lenient[0].checksum_rule is None
    strict = p.FcFrameParser(p.Config(strict_rx_checksum=True))
    assert strict.feed(bad, 0.0) == [] and strict.dropped == 1


def test_parser_drops_partial_frame_after_timeout():
    parser = p.FcFrameParser()
    assert parser.feed(GOLDEN_BE[:10], now=0.0) == []
    frames = parser.feed(p.encode_status(0x01), now=p.FRAME_TIMEOUT_S + 0.01)
    assert [f.status for f in frames] == [0x01]
    assert parser.timeouts == 1
