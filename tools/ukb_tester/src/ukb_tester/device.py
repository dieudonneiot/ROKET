"""Emulates the referee test device: drives an I-FCC over a serial port and scores it.

Works with any port object that has write(), read() and in_waiting, so it runs
the same against a real USB-RS232 adapter (pyserial) or a fake in the tests.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from itertools import pairwise
from typing import Protocol

from .protocol import (
    ACTIVATION_DELAY_S,
    CMD_START_SFT,
    CMD_START_SMT,
    CMD_STOP,
    DEFAULT_CONFIG,
    Checksum,
    CommandChecksum,
    Config,
    FcFrameParser,
    Frame,
    Sample,
    Status,
    encode_command,
    encode_data,
)
from .scenarios import Scenario

DROGUE_WINDOW_S = 2.0  # spec 3.3.1.7
MAIN_WINDOW_M = (400.0, 600.0)  # spec 3.3.1.8
LAUNCH_WINDOW_S = 1.0


class Port(Protocol):
    @property
    def in_waiting(self) -> int: ...

    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> int | None: ...


@dataclass
class SmtResult:
    duration_s: float
    frames: list[tuple[float, Sample]] = field(default_factory=list)
    checksum_rules: set[Checksum | None] = field(default_factory=set)
    dropped: int = 0

    @property
    def rate_hz(self) -> float:
        if len(self.frames) < 2:
            return 0.0
        span = self.frames[-1][0] - self.frames[0][0]
        return (len(self.frames) - 1) / span if span > 0.0 else 0.0


@dataclass
class SftResult:
    scenario: Scenario
    statuses: list[tuple[float, int]] = field(default_factory=list)  # (scenario time, bits)
    checksum_rules: set[Checksum | None] = field(default_factory=set)
    frames_sent: int = 0
    dropped: int = 0

    def first_set(self, bit: Status) -> float | None:
        return next((t for t, bits in self.statuses if bits & bit), None)


@dataclass(frozen=True)
class Check:
    name: str
    passed: bool
    detail: str


class RefereeDevice:
    def __init__(
        self,
        port: Port,
        config: Config = DEFAULT_CONFIG,
        command_checksum: CommandChecksum = CommandChecksum.HEADER_SUM,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.port = port
        self.config = config
        self.command_checksum = command_checksum
        self.clock = clock
        self.sleep = sleep

    def send_command(self, command: int) -> None:
        self.port.write(encode_command(command, self.command_checksum))

    def _read(self, parser: FcFrameParser) -> list[Frame]:
        waiting = self.port.in_waiting
        data = self.port.read(waiting) if waiting else b""
        return parser.feed(data, self.clock())

    def _wait(self, parser: FcFrameParser, seconds: float, sink: list[Frame]) -> None:
        end = self.clock() + seconds
        while self.clock() < end:
            sink.extend(self._read(parser))
            self.sleep(min(0.01, max(end - self.clock(), 0.0)))

    def run_smt(self, duration_s: float = 10.0) -> SmtResult:
        """Starts the Sensor Monitoring Test, records the sensor stream, then stops it."""
        parser = FcFrameParser(self.config)
        result = SmtResult(duration_s)
        self.send_command(CMD_START_SMT)
        start = self.clock()
        received: list[Frame] = []
        end = start + duration_s
        while self.clock() < end:
            for frame in self._read(parser):
                if frame.kind == "data" and frame.sample is not None:
                    result.frames.append((self.clock() - start, frame.sample))
                    result.checksum_rules.add(frame.checksum_rule)
                received.append(frame)
            self.sleep(0.01)
        self.send_command(CMD_STOP)
        self._wait(parser, 0.3, received)
        result.dropped = parser.dropped
        return result

    def run_sft(self, scenario: Scenario, settle_s: float = ACTIVATION_DELAY_S + 0.2) -> SftResult:
        """Plays a scenario at its own rate and records the status bits the FCC reports."""
        parser = FcFrameParser(self.config)
        result = SftResult(scenario)
        self.send_command(CMD_START_SFT)
        ignored: list[Frame] = []
        self._wait(parser, settle_s, ignored)

        start = self.clock()
        for index, sample in enumerate(scenario.samples):
            due = start + index * scenario.period_s
            while self.clock() < due:
                self._collect(parser, result, index - 1)
                self.sleep(min(0.005, max(due - self.clock(), 0.0)))
            self.port.write(encode_data(sample, self.config))
            result.frames_sent += 1
            self._collect(parser, result, index)

        self.sleep(scenario.period_s)
        self._collect(parser, result, len(scenario.samples) - 1)
        self.send_command(CMD_STOP)
        result.dropped = parser.dropped
        return result

    def _collect(self, parser: FcFrameParser, result: SftResult, index: int) -> None:
        for frame in self._read(parser):
            if frame.kind == "status":
                time_s = result.scenario.time_of(max(index, 0))
                result.statuses.append((time_s, frame.status))
                result.checksum_rules.add(frame.checksum_rule)


def evaluate_sft(result: SftResult) -> list[Check]:
    """Scores a synthetic flight against the A4 recovery rules.

    Status times are those of the last data frame sent before the status
    arrived, so they lag the FCC's decision by up to one frame (0.1 s): the
    checks are slightly conservative.
    """
    truth = result.scenario.truth
    checks: list[Check] = []

    early = [bits for t, bits in result.statuses if t < truth.ignition_s and bits]
    checks.append(
        Check("no event before ignition", not early, f"{len(early)} status frames with bits set")
    )

    launch = result.first_set(Status.LAUNCH)
    checks.append(
        Check(
            "launch detected within 1 s of ignition",
            launch is not None and truth.ignition_s <= launch <= truth.ignition_s + LAUNCH_WINDOW_S,
            _describe(launch, truth.ignition_s, "ignition"),
        )
    )

    drogue = result.first_set(Status.DROGUE_COMMAND)
    checks.append(
        Check(
            "drogue commanded within 2 s after apogee",
            drogue is not None and truth.apogee_s <= drogue <= truth.apogee_s + DROGUE_WINDOW_S,
            _describe(drogue, truth.apogee_s, "apogee"),
        )
    )

    main = result.first_set(Status.MAIN_COMMAND)
    main_agl = result.scenario.agl_at(main) if main is not None else math.nan
    checks.append(
        Check(
            "main commanded between 400 and 600 m above ground",
            main is not None and MAIN_WINDOW_M[0] <= main_agl <= MAIN_WINDOW_M[1],
            "never commanded" if main is None else f"at {main_agl:.0f} m above ground",
        )
    )

    cleared = [
        (t, before & ~after)
        for (_, before), (t, after) in pairwise(result.statuses)
        if before & ~after
    ]
    checks.append(
        Check(
            "status bits stay set once set",
            not cleared,
            "ok" if not cleared else f"bits {cleared[0][1]:#06x} cleared at {cleared[0][0]:.1f} s",
        )
    )
    return checks


def _describe(time_s: float | None, reference_s: float, label: str) -> str:
    if time_s is None:
        return "never set"
    return f"at {time_s:.1f} s ({time_s - reference_s:+.2f} s after {label})"
