"""Referee test protocol of TEKNOFEST A4 (Attachment-7), Python side.

Mirrors firmware/core/include/fc/att7.h so the emulator speaks exactly what the
firmware expects; tests/test_parity.py compares both through the compiled core.
Conventions follow findings log F-29: send header-included checksums and
big-endian floats rounded to 2 decimals; accept both checksum rules on receive.
"""

from __future__ import annotations

import math
import struct
from dataclasses import astuple, dataclass
from enum import IntEnum, IntFlag

HEADER_COMMAND = 0xAA
HEADER_DATA = 0xAB
FOOTER = b"\x0d\x0a"

CMD_START_SMT = 0x20
CMD_START_SFT = 0x22
CMD_STOP = 0x24

COMMAND_LEN = 5
DATA_LEN = 36
STATUS_LEN = 6
DATA_CHECKSUM_INDEX = 33
STATUS_CHECKSUM_INDEX = 3
TABLE1_CHECKSUM_OFFSET = 0x6C

ACTIVATION_DELAY_S = 1.0
TX_PERIOD_S = 0.1
FRAME_TIMEOUT_S = 0.05
ROUND_LIMIT = 1.0e7


class Checksum(IntEnum):
    """Byte range summed for a frame checksum."""

    WITH_HEADER = 0
    WITHOUT_HEADER = 1


class FloatOrder(IntEnum):
    BIG_ENDIAN = 0
    LITTLE_ENDIAN = 1


class CommandChecksum(IntEnum):
    """Checksum style of a command frame."""

    HEADER_SUM = 0  # 0xAA + command, e.g. AA 20 CA 0D 0A
    TABLE1 = 1  # command + 0x6C as printed in Table 1, e.g. AA 20 8C 0D 0A


class Status(IntFlag):
    """SFT status bits (Attachment-7 Table 5)."""

    LAUNCH = 1 << 0
    BURNOUT_LOCKOUT = 1 << 1
    MIN_ALTITUDE = 1 << 2
    BODY_ANGLE = 1 << 3
    DESCENDING = 1 << 4
    DROGUE_COMMAND = 1 << 5
    BELOW_MAIN = 1 << 6
    MAIN_COMMAND = 1 << 7


@dataclass(frozen=True)
class Config:
    tx_checksum: Checksum = Checksum.WITH_HEADER
    float_order: FloatOrder = FloatOrder.BIG_ENDIAN
    round_2dp: bool = True
    strict_rx_checksum: bool = False
    status_low_first: bool = True


DEFAULT_CONFIG = Config()


@dataclass(frozen=True)
class Sample:
    """The 8 fields of a data frame. Body frame: x toward the nose, y right, z down."""

    altitude_m: float  # above sea level
    pressure_mbar: float
    accel_x_mps2: float
    accel_y_mps2: float
    accel_z_mps2: float
    angle_x_deg: float  # roll
    angle_y_deg: float  # pitch
    angle_z_deg: float  # yaw

    def fields(self) -> tuple[float, ...]:
        return astuple(self)


def to_float32(value: float) -> float:
    """Returns the binary32 value a C float would hold."""
    try:
        return struct.unpack("<f", struct.pack("<f", value))[0]
    except OverflowError:
        return math.copysign(math.inf, value)


def round_2dp(value: float) -> float:
    """Same result as att7_round_2dp(): float32 maths, half away from zero, never -0.0."""
    v = to_float32(value)
    if not math.isfinite(v) or abs(v) >= ROUND_LIMIT:
        return v
    scaled = to_float32(v * 100.0)
    rounded = math.copysign(math.floor(abs(scaled) + 0.5), scaled)
    result = to_float32(rounded / 100.0)
    return 0.0 if result == 0.0 else result


def checksum(frame: bytes, checksum_index: int, mode: Checksum = Checksum.WITH_HEADER) -> int:
    start = 1 if mode == Checksum.WITHOUT_HEADER else 0
    return sum(frame[start:checksum_index]) & 0xFF


def encode_command(command: int, style: CommandChecksum = CommandChecksum.HEADER_SUM) -> bytes:
    if style == CommandChecksum.TABLE1:
        check = (command + TABLE1_CHECKSUM_OFFSET) & 0xFF
    else:
        check = (HEADER_COMMAND + command) & 0xFF
    return bytes((HEADER_COMMAND, command & 0xFF, check)) + FOOTER


def _float_format(order: FloatOrder, count: int = 1) -> str:
    return ("<" if order == FloatOrder.LITTLE_ENDIAN else ">") + f"{count}f"


def encode_data(sample: Sample, config: Config = DEFAULT_CONFIG) -> bytes:
    values = [round_2dp(v) if config.round_2dp else to_float32(v) for v in sample.fields()]
    body = bytearray([HEADER_DATA])
    body += struct.pack(_float_format(config.float_order, len(values)), *values)
    body.append(checksum(body, DATA_CHECKSUM_INDEX, config.tx_checksum))
    return bytes(body) + FOOTER


def decode_data(frame: bytes, config: Config = DEFAULT_CONFIG) -> Sample:
    values = struct.unpack(_float_format(config.float_order, 8), frame[1:DATA_CHECKSUM_INDEX])
    return Sample(*values)


def encode_status(status: int, config: Config = DEFAULT_CONFIG) -> bytes:
    low, high = status & 0xFF, (status >> 8) & 0xFF
    data1, data2 = (low, high) if config.status_low_first else (high, low)
    body = bytearray((HEADER_COMMAND, data1, data2))
    body.append(checksum(body, STATUS_CHECKSUM_INDEX, config.tx_checksum))
    return bytes(body) + FOOTER


def decode_status(frame: bytes, config: Config = DEFAULT_CONFIG) -> int:
    data1, data2 = frame[1], frame[2]
    if config.status_low_first:
        return data1 | (data2 << 8)
    return data2 | (data1 << 8)


@dataclass(frozen=True)
class Frame:
    """A frame received from the flight computer."""

    kind: str  # "status" (SFT) or "data" (SMT)
    raw: bytes
    checksum_rule: Checksum | None  # rule that matched, None if neither did
    status: int = 0
    sample: Sample | None = None


class FcFrameParser:
    """Streaming parser for frames the flight computer sends to the test device.

    Header 0xAA starts a 6-byte status frame, 0xAB a 36-byte data frame.
    """

    def __init__(self, config: Config = DEFAULT_CONFIG) -> None:
        self.config = config
        self._buf = bytearray()
        self._last_byte_time: float | None = None
        self.frames_ok = 0
        self.dropped = 0
        self.timeouts = 0

    def feed(self, data: bytes, now: float) -> list[Frame]:
        if (
            self._buf
            and self._last_byte_time is not None
            and now - self._last_byte_time > FRAME_TIMEOUT_S
        ):
            self.timeouts += 1
            self._buf.clear()
        if data:
            self._last_byte_time = now
            self._buf += data

        frames: list[Frame] = []
        while True:
            while self._buf and self._buf[0] not in (HEADER_COMMAND, HEADER_DATA):
                del self._buf[0]
            if not self._buf:
                break
            length = STATUS_LEN if self._buf[0] == HEADER_COMMAND else DATA_LEN
            if len(self._buf) < length:
                break
            frame = self._decode(bytes(self._buf[:length]))
            if frame is None:
                self.dropped += 1
                del self._buf[0]
                continue
            self.frames_ok += 1
            frames.append(frame)
            del self._buf[:length]
        return frames

    def _decode(self, raw: bytes) -> Frame | None:
        if raw[-2:] != FOOTER:
            return None
        index = len(raw) - 3
        rule = next((mode for mode in Checksum if raw[index] == checksum(raw, index, mode)), None)
        if rule is None and self.config.strict_rx_checksum:
            return None
        if raw[0] == HEADER_COMMAND:
            return Frame("status", raw, rule, status=decode_status(raw, self.config))
        return Frame("data", raw, rule, sample=decode_data(raw, self.config))
