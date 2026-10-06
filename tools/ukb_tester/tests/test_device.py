"""End-to-end runs of the emulator against a fake flight computer on a virtual clock."""

from collections.abc import Callable

from ukb_tester import protocol as p
from ukb_tester.device import RefereeDevice, evaluate_sft
from ukb_tester.scenarios import Scenario, build_dataset


class VirtualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def time(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(seconds, 1e-6)


class FakeFcc:
    """Speaks the flight computer's side of Attachment-7 with scripted behaviour."""

    def __init__(self, clock: VirtualClock, decide: Callable[[int, p.Sample], int]) -> None:
        self.clock = clock
        self.decide = decide
        self.inbox = bytearray()
        self.outbox = bytearray()
        self.mode = "normal"
        self.pending: tuple[str, float] | None = None
        self.next_tx = 0.0
        self.frames = 0
        self.bits = 0

    @property
    def in_waiting(self) -> int:
        self._tick()
        return len(self.outbox)

    def read(self, size: int = 1) -> bytes:
        chunk = bytes(self.outbox[:size])
        del self.outbox[:size]
        return chunk

    def write(self, data: bytes) -> int:
        self.inbox += data
        while self.inbox:
            length = p.COMMAND_LEN if self.inbox[0] == p.HEADER_COMMAND else p.DATA_LEN
            if len(self.inbox) < length:
                break
            frame = bytes(self.inbox[:length])
            del self.inbox[:length]
            self._handle(frame)
        return len(data)

    def _tick(self) -> None:
        if self.pending and self.clock.now - self.pending[1] >= p.ACTIVATION_DELAY_S:
            self.mode, self.pending = self.pending[0], None
            self.next_tx = self.clock.now
        if self.mode == "smt" and self.clock.now >= self.next_tx:
            self.next_tx += p.TX_PERIOD_S
            self.outbox += p.encode_data(p.Sample(1000.0, 900.0, 0.0, 0.0, -9.81, 0, 90, 0))

    def _handle(self, frame: bytes) -> None:
        self._tick()
        if frame[0] == p.HEADER_COMMAND:
            command = frame[1]
            if command == p.CMD_STOP:
                self.mode, self.pending = "normal", None
            elif command in (p.CMD_START_SMT, p.CMD_START_SFT):
                mode = "smt" if command == p.CMD_START_SMT else "sft"
                self.mode, self.pending = "normal", (mode, self.clock.now)
        elif self.mode == "sft":
            self.bits |= self.decide(self.frames, p.decode_data(frame))
            self.frames += 1
            self.outbox += p.encode_status(self.bits)


def ideal_fcc(scenario: Scenario, drogue_offset_s: float = 0.9, main_agl_m: float = 520.0):
    truth = scenario.truth

    def decide(index: int, _: p.Sample) -> int:
        t = scenario.time_of(index)
        agl = scenario.agl_m[index]
        bits = 0
        if t >= truth.ignition_s + 0.2:
            bits |= p.Status.LAUNCH
        if t >= truth.burnout_s + 0.2:
            bits |= p.Status.BURNOUT_LOCKOUT
        if agl > 1200.0:
            bits |= p.Status.MIN_ALTITUDE
        if t >= truth.apogee_s - 3.0:
            bits |= p.Status.BODY_ANGLE
        if t >= truth.apogee_s + 0.3:
            bits |= p.Status.DESCENDING
        if t >= truth.apogee_s + drogue_offset_s:
            bits |= p.Status.DROGUE_COMMAND
        if t > truth.apogee_s and agl < main_agl_m:
            bits |= p.Status.BELOW_MAIN | p.Status.MAIN_COMMAND
        return bits

    return decide


def run(scenario: Scenario, decide) -> list:
    clock = VirtualClock()
    device = RefereeDevice(FakeFcc(clock, decide), clock=clock.time, sleep=clock.sleep)
    result = device.run_sft(scenario)
    assert result.frames_sent == len(scenario.samples)
    assert result.checksum_rules == {p.Checksum.WITH_HEADER}
    return evaluate_sft(result)


def test_a_correct_fcc_passes_every_check():
    scenario = build_dataset(1)
    checks = run(scenario, ideal_fcc(scenario))
    assert all(check.passed for check in checks), checks


def test_an_early_drogue_fails():
    scenario = build_dataset(1)
    checks = {c.name: c for c in run(scenario, ideal_fcc(scenario, drogue_offset_s=-1.0))}
    assert not checks["drogue commanded within 2 s after apogee"].passed


def test_a_late_drogue_fails():
    scenario = build_dataset(1)
    checks = {c.name: c for c in run(scenario, ideal_fcc(scenario, drogue_offset_s=2.5))}
    assert not checks["drogue commanded within 2 s after apogee"].passed


def test_a_main_fired_too_high_fails():
    scenario = build_dataset(1)
    checks = {c.name: c for c in run(scenario, ideal_fcc(scenario, main_agl_m=800.0))}
    assert not checks["main commanded between 400 and 600 m above ground"].passed


def test_smt_measures_the_stream_rate():
    clock = VirtualClock()
    device = RefereeDevice(FakeFcc(clock, lambda i, s: 0), clock=clock.time, sleep=clock.sleep)
    result = device.run_smt(duration_s=5.0)
    assert 9.0 <= result.rate_hz <= 11.0
    assert result.checksum_rules == {p.Checksum.WITH_HEADER}
    assert result.frames[-1][1].angle_y_deg == 90.0
