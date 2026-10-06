"""Synthetic flights for the Synthetic Flight Test (SFT).

Stand-ins for the four official data sets, which the committee releases only
after CDR (findings log F-27). Fields follow Attachment-7: altitude above sea
level, pressure in mbar, body-frame specific force in m/s^2 (x toward the nose,
y right, z down) and Z-Y-X Euler angles in degrees.

The rocket is a 2-D point mass on a gravity turn with the organizer's motor
curve. It is deliberately simple: the profiles exercise the flight logic, they
do not predict our real flight (that is RocketPy's job).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, replace
from functools import lru_cache
from itertools import pairwise

from .protocol import Sample

G = 9.80665
SITE_ALTITUDE_M = 970.0  # mandated OpenRocket launch site (F-60)
RAIL_LENGTH_M = 6.0
RAIL_ELEVATION_DEG = 85.0

# Official organizer curve Teknofest26_M1850_Rev1.eng: (time s, thrust N), F-61.
TF26_M1850_THRUST = (
    (0.0, 0.0), (0.104, 2411.0), (0.208, 2135.0), (0.312, 2015.0), (0.415, 2000.0),
    (0.519, 2055.0), (1.038, 2098.0), (1.558, 1860.0), (2.077, 1788.0), (2.596, 1659.0),
    (3.115, 1468.0), (3.375, 1423.0), (3.479, 1334.0), (3.635, 1201.0), (3.894, 934.0),
    (3.946, 930.0), (4.154, 881.0), (4.414, 600.0), (4.673, 468.0), (4.933, 400.0),
    (5.192, 290.0), (5.712, 85.0), (6.231, 23.0), (6.750, 0.0),
)  # fmt: skip
TF26_PROPELLANT_KG = 4.103


@dataclass(frozen=True)
class RocketModel:
    """Placeholder airframe; replace with the real design once it exists."""

    liftoff_mass_kg: float = 25.0
    diameter_m: float = 0.115
    drag_coefficient: float = 0.5
    drogue_delay_s: float = 1.5  # apogee to a fully open drogue (assumed)
    drogue_descent_mps: float = 15.0
    main_altitude_agl_m: float = 500.0
    main_descent_mps: float = 6.5


DEFAULT_MODEL = RocketModel()


@dataclass(frozen=True)
class Truth:
    """What really happened in the synthetic flight, in scenario seconds."""

    ignition_s: float
    rail_exit_s: float
    rail_exit_speed_mps: float
    burnout_s: float
    apogee_s: float
    apogee_agl_m: float
    drogue_open_s: float
    main_open_s: float
    landing_s: float


@dataclass(frozen=True)
class Scenario:
    number: int
    name: str
    description: str
    period_s: float
    samples: tuple[Sample, ...]
    agl_m: tuple[float, ...]  # true height above ground at each sample
    truth: Truth

    def time_of(self, index: int) -> float:
        return index * self.period_s

    def agl_at(self, time_s: float) -> float:
        index = min(max(round(time_s / self.period_s), 0), len(self.agl_m) - 1)
        return self.agl_m[index]


def isa_pressure_mbar(altitude_asl_m: float) -> float:
    return 1013.25 * (1.0 - 2.25577e-5 * altitude_asl_m) ** 5.25588


def isa_density(altitude_asl_m: float) -> float:
    return 1.225 * (1.0 - 2.25577e-5 * altitude_asl_m) ** 4.2559


def _thrust(t: float) -> float:
    curve = TF26_M1850_THRUST
    if t <= 0.0 or t >= curve[-1][0]:
        return 0.0
    for (t0, f0), (t1, f1) in pairwise(curve):
        if t <= t1:
            return f0 + (f1 - f0) * (t - t0) / (t1 - t0)
    return 0.0


TF26_TOTAL_IMPULSE_NS = sum(
    (t1 - t0) * (f0 + f1) / 2.0 for (t0, f0), (t1, f1) in pairwise(TF26_M1850_THRUST)
)


def _body_frame(fx: float, fz: float, pitch_rad: float) -> tuple[float, float]:
    """Projects a specific force (horizontal, up) onto body x (nose) and z (down)."""
    x_axis = (math.cos(pitch_rad), math.sin(pitch_rad))
    z_axis = (math.sin(pitch_rad), -math.cos(pitch_rad))
    return fx * x_axis[0] + fz * x_axis[1], fx * z_axis[0] + fz * z_axis[1]


@lru_cache(maxsize=8)
def simulate_flight(
    model: RocketModel = DEFAULT_MODEL,
    pad_s: float = 5.0,
    after_landing_s: float = 5.0,
    period_s: float = 0.1,
    dt: float = 0.005,
    max_time_s: float = 1800.0,
) -> tuple[tuple[Sample, ...], tuple[float, ...], Truth]:
    """Integrates the flight and samples it every `period_s` seconds."""
    area = math.pi * (model.diameter_m / 2.0) ** 2
    rail_dir = math.radians(RAIL_ELEVATION_DEG)
    steps_per_sample = round(period_s / dt)

    x = z = vx = vz = 0.0
    rail_travel = 0.0
    impulse = 0.0
    phase = "pad"  # pad, rail, free, drogue, main, landed
    events: dict[str, float] = {"ignition": pad_s}
    rail_exit_speed = 0.0
    samples: list[Sample] = []
    agl: list[float] = []
    step = 0

    while True:
        t = step * dt
        if t > max_time_s:
            raise RuntimeError(f"no landing within {max_time_s} s; check the rocket model")
        burn_t = t - pad_s
        thrust = _thrust(burn_t) if phase in ("rail", "free") else 0.0
        mass = model.liftoff_mass_kg - TF26_PROPELLANT_KG * impulse / TF26_TOTAL_IMPULSE_NS
        speed = math.hypot(vx, vz)
        drag = 0.5 * isa_density(SITE_ALTITUDE_M + z) * speed**2 * model.drag_coefficient * area

        if phase == "pad":
            ax = az = 0.0
            pitch = rail_dir
            if t >= pad_s:
                phase = "rail"
        elif phase == "rail":
            along = max((thrust - drag) / mass - G * math.sin(rail_dir), 0.0)
            ax, az = along * math.cos(rail_dir), along * math.sin(rail_dir)
            pitch = rail_dir
        elif phase == "free":
            ux, uz = (vx / speed, vz / speed) if speed > 0.0 else (0.0, 1.0)
            ax = (thrust - drag) / mass * ux
            az = (thrust - drag) / mass * uz - G
            pitch = math.atan2(vz, vx)
        elif phase in ("drogue", "main"):
            target = model.drogue_descent_mps if phase == "drogue" else model.main_descent_mps
            tau = 0.7 if phase == "drogue" else 1.0
            ax = -vx / tau
            az = (-target - vz) / tau
            pitch = math.radians(-80.0 + 5.0 * math.sin(2.0 * math.pi * 0.5 * t))
        else:  # landed: lying on the ground
            ax = az = 0.0
            pitch = 0.0

        if step % steps_per_sample == 0:
            force_x, force_z = _body_frame(ax, az + G, pitch)
            samples.append(
                Sample(
                    altitude_m=SITE_ALTITUDE_M + z,
                    pressure_mbar=isa_pressure_mbar(SITE_ALTITUDE_M + z),
                    accel_x_mps2=force_x,
                    accel_y_mps2=0.0,
                    accel_z_mps2=force_z,
                    angle_x_deg=0.0,
                    angle_y_deg=math.degrees(pitch),
                    angle_z_deg=0.0,
                )
            )
            agl.append(z)
            if phase == "landed" and t >= events["landing"] + after_landing_s:
                break

        if phase in ("rail", "free"):
            impulse += thrust * dt
        prev_vz = vz
        vx += ax * dt
        vz += az * dt
        x += vx * dt
        z += vz * dt

        if phase == "rail":
            rail_travel = math.hypot(x, z)
            if rail_travel >= RAIL_LENGTH_M:
                phase = "free"
                events["rail_exit"] = t + dt
                rail_exit_speed = math.hypot(vx, vz)
        elif phase == "free":
            if "burnout" not in events and burn_t >= TF26_M1850_THRUST[-1][0]:
                events["burnout"] = t
            if "apogee" not in events and prev_vz > 0.0 >= vz:
                events["apogee"] = t
                events["apogee_agl"] = z
            if "apogee" in events and t >= events["apogee"] + model.drogue_delay_s:
                phase = "drogue"
                events["drogue_open"] = t
        elif phase == "drogue" and z <= model.main_altitude_agl_m:
            phase = "main"
            events["main_open"] = t
        if phase in ("free", "drogue", "main") and "apogee" in events and z <= 0.0:
            z = vx = vz = 0.0
            phase = "landed"
            events["landing"] = t
        step += 1

    truth = Truth(
        ignition_s=events["ignition"],
        rail_exit_s=events["rail_exit"],
        rail_exit_speed_mps=rail_exit_speed,
        burnout_s=events["burnout"],
        apogee_s=events["apogee"],
        apogee_agl_m=events["apogee_agl"],
        drogue_open_s=events["drogue_open"],
        main_open_s=events["main_open"],
        landing_s=events["landing"],
    )
    return tuple(samples), tuple(agl), truth


def _add_noise(
    samples: list[Sample] | tuple[Sample, ...],
    rng: random.Random,
    altitude_m: float = 0.0,
    pressure_mbar: float = 0.0,
    accel_mps2: float = 0.0,
    angle_deg: float = 0.0,
) -> list[Sample]:
    def jitter(sigma: float) -> float:
        return rng.gauss(0.0, sigma) if sigma > 0.0 else 0.0

    return [
        Sample(
            s.altitude_m + jitter(altitude_m),
            s.pressure_mbar + jitter(pressure_mbar),
            s.accel_x_mps2 + jitter(accel_mps2),
            s.accel_y_mps2 + jitter(accel_mps2),
            s.accel_z_mps2 + jitter(accel_mps2),
            s.angle_x_deg + jitter(angle_deg),
            s.angle_y_deg + jitter(angle_deg),
            s.angle_z_deg + jitter(angle_deg),
        )
        for s in samples
    ]


def _inject_faults(
    samples: list[Sample], agl: tuple[float, ...], truth: Truth, period_s: float
) -> None:
    """Abnormal inputs of the kind Attachment-7 names: spikes, frozen and meaningless data."""

    def index(time_s: float) -> int:
        return min(max(round(time_s / period_s), 0), len(samples) - 1)

    def patch(i: int, **changes: float) -> None:
        samples[i] = replace(samples[i], **changes)

    # A bump on the pad: big acceleration, no climb. Must not count as a launch.
    patch(index(truth.ignition_s - 3.0), accel_x_mps2=60.0)
    # A one-frame altitude drop in the coast phase: a fake descent.
    i = index(truth.apogee_s - 6.0)
    patch(i, altitude_m=samples[i].altitude_m - 400.0)
    # Pressure frozen for 2 s during the coast.
    start = index(truth.apogee_s - 4.0)
    for j in range(start, start + 20):
        patch(j, pressure_mbar=samples[start].pressure_mbar)
    # A single acceleration spike shortly before apogee.
    patch(index(truth.apogee_s - 2.0), accel_x_mps2=150.0)
    # Meaningless data just after apogee.
    i = index(truth.apogee_s + 0.5)
    patch(i, pressure_mbar=math.nan, altitude_m=samples[i].altitude_m + 500.0)
    # A one-frame altitude drop to 300 m below the truth at 700 m above ground:
    # it must not fire the main parachute early.
    for j in range(index(truth.drogue_open_s), len(agl)):
        if agl[j] <= 700.0:
            patch(j, altitude_m=samples[j].altitude_m - 300.0)
            break


DATASETS = {
    1: ("nominal", "Clean nominal flight."),
    2: ("baro-noise", "Altitude noise 2 m and pressure noise 0.24 mbar (1 standard deviation)."),
    3: (
        "baro-accel-noise",
        "Barometer noise as in set 2, plus 0.5 m/s^2 acceleration and 0.5 deg angle noise.",
    ),
    4: (
        "abnormal",
        "Low noise plus faults: a pad bump, altitude spikes, frozen pressure, an "
        "acceleration spike and a NaN.",
    ),
}


def build_dataset(number: int, model: RocketModel = DEFAULT_MODEL, seed: int = 2027) -> Scenario:
    """Builds data set 1-4 (unofficial stand-ins for the committee's sets)."""
    if number not in DATASETS:
        raise ValueError(f"unknown data set {number}; choose one of {sorted(DATASETS)}")
    period_s = 0.1
    clean, agl, truth = simulate_flight(model, period_s=period_s)
    samples = list(clean)
    rng = random.Random(seed + number)
    if number == 2:
        samples = _add_noise(samples, rng, altitude_m=2.0, pressure_mbar=0.24)
    elif number == 3:
        samples = _add_noise(
            samples, rng, altitude_m=2.0, pressure_mbar=0.24, accel_mps2=0.5, angle_deg=0.5
        )
    elif number == 4:
        samples = _add_noise(samples, rng, altitude_m=0.5, pressure_mbar=0.06)
        _inject_faults(samples, agl, truth, period_s)
    name, description = DATASETS[number]
    return Scenario(number, name, description, period_s, tuple(samples), agl, truth)
