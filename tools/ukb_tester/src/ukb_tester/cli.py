"""Command-line entry point: ukb-tester."""

from __future__ import annotations

import argparse
import csv
import sys

from . import __version__
from .device import RefereeDevice, evaluate_sft
from .protocol import Checksum, CommandChecksum, Config, FloatOrder
from .scenarios import DATASETS, build_dataset

BAUD_RATE = 115200  # Attachment-7 Table 7: 115200 baud, 8 data bits, no parity, 1 stop bit


def _config(args: argparse.Namespace) -> Config:
    return Config(
        tx_checksum=Checksum.WITHOUT_HEADER
        if args.checksum == "no-header"
        else Checksum.WITH_HEADER,
        float_order=FloatOrder.LITTLE_ENDIAN if args.float_order == "le" else FloatOrder.BIG_ENDIAN,
    )


def _device(args: argparse.Namespace) -> RefereeDevice:
    import serial  # pyserial

    port = serial.Serial(args.port, BAUD_RATE, bytesize=8, parity="N", stopbits=1, timeout=0)
    style = (
        CommandChecksum.TABLE1 if args.command_checksum == "table1" else CommandChecksum.HEADER_SUM
    )
    return RefereeDevice(port, _config(args), style)


def _rules(rules: set[Checksum | None]) -> str:
    names = sorted("no match" if r is None else r.name.lower() for r in rules)
    return ", ".join(names) or "none received"


def cmd_list_ports(_: argparse.Namespace) -> int:
    from serial.tools import list_ports

    for port in list_ports.comports():
        print(f"{port.device}\t{port.description}")
    return 0


def cmd_smt(args: argparse.Namespace) -> int:
    result = _device(args).run_smt(args.seconds)
    print(f"Sensor frames received: {len(result.frames)} in {args.seconds:.0f} s")
    print(f"Rate: {result.rate_hz:.1f} Hz (expected 10 Hz)")
    print(f"Checksum rule used by the FCC: {_rules(result.checksum_rules)}")
    print(f"Dropped bytes or frames: {result.dropped}")
    if result.frames:
        t, s = result.frames[-1]
        print(
            f"Last frame at {t:.1f} s: altitude {s.altitude_m:.2f} m, pressure "
            f"{s.pressure_mbar:.2f} mbar, accel ({s.accel_x_mps2:.2f}, {s.accel_y_mps2:.2f}, "
            f"{s.accel_z_mps2:.2f}) m/s2, angles ({s.angle_x_deg:.2f}, {s.angle_y_deg:.2f}, "
            f"{s.angle_z_deg:.2f}) deg"
        )
    return 0 if 9.0 <= result.rate_hz <= 11.0 else 1


def cmd_sft(args: argparse.Namespace) -> int:
    scenario = build_dataset(args.dataset)
    duration = len(scenario.samples) * scenario.period_s
    print(f"Data set {scenario.number} ({scenario.name}): {scenario.description}")
    print(f"Playing {len(scenario.samples)} frames ({duration:.0f} s) ...")
    result = _device(args).run_sft(scenario)
    checks = evaluate_sft(result)
    print(
        f"Status frames received: {len(result.statuses)}; checksum rule used by the FCC: "
        f"{_rules(result.checksum_rules)}"
    )
    for check in checks:
        print(f"[{'PASS' if check.passed else 'FAIL'}] {check.name}: {check.detail}")
    return 0 if all(check.passed for check in checks) else 1


def cmd_export(args: argparse.Namespace) -> int:
    scenario = build_dataset(args.dataset)
    with open(args.csv, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "time_s", "agl_m", "altitude_m", "pressure_mbar", "accel_x_mps2", "accel_y_mps2",
            "accel_z_mps2", "angle_x_deg", "angle_y_deg", "angle_z_deg",
        ])  # fmt: skip
        for index, (sample, agl) in enumerate(zip(scenario.samples, scenario.agl_m, strict=True)):
            writer.writerow([f"{scenario.time_of(index):.1f}", f"{agl:.2f}", *sample.fields()])
    truth = scenario.truth
    print(f"Wrote {len(scenario.samples)} rows to {args.csv}")
    print(
        f"Apogee {truth.apogee_agl_m:.0f} m above ground at {truth.apogee_s:.1f} s; "
        f"rail exit {truth.rail_exit_speed_mps:.1f} m/s; main opens at {truth.main_open_s:.1f} s"
    )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="ukb-tester",
        description="Emulator of the TEKNOFEST referee test device (Attachment-7 SMT/SFT).",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list-ports", help="list serial ports").set_defaults(func=cmd_list_ports)

    link = argparse.ArgumentParser(add_help=False)
    link.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    link.add_argument(
        "--command-checksum",
        choices=("header-sum", "table1"),
        default="header-sum",
        help="checksum style of our commands (default: header-sum, i.e. 0xAA + cmd)",
    )
    link.add_argument(
        "--checksum",
        choices=("header", "no-header"),
        default="header",
        help="checksum rule of the data frames we send (default: header)",
    )
    link.add_argument(
        "--float-order",
        choices=("be", "le"),
        default="be",
        help="float byte order (default: be, big-endian)",
    )

    smt = sub.add_parser("smt", parents=[link], help="run the Sensor Monitoring Test")
    smt.add_argument("--seconds", type=float, default=10.0)
    smt.set_defaults(func=cmd_smt)

    sft = sub.add_parser("sft", parents=[link], help="run the Synthetic Flight Test")
    sft.add_argument("--dataset", type=int, choices=sorted(DATASETS), default=1)
    sft.set_defaults(func=cmd_sft)

    export = sub.add_parser("export", help="write a data set to CSV for inspection")
    export.add_argument("--dataset", type=int, choices=sorted(DATASETS), default=1)
    export.add_argument("--csv", required=True)
    export.set_defaults(func=cmd_export)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
