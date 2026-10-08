"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: test_sensors.py

Title: Sensor and actuator checks

Purpose: Exercise the RACECAR Neo V2 sensor and actuator API (encoder, battery,
magnetometer, dot matrix, LED strip) and report each check as PASS or FAIL. Runs
in RacecarSim and on the physical car.

Usage:
    python test_sensors.py -s                      interactive (RacecarSim)
    python test_sensors.py                         interactive (physical car)
    python test_sensors.py -s --section encoder    run one section, print the report, exit

Interactive controls, in user program mode:
    A        api      every sensor read and actuator command once
    B        encoder  drive at speed 0.25, 0.5, 1.0 (max speed 1.0) and compare;
                      needs about 25 m of straight road (RacecarSim: Long
                      Hallway Sandbox)
    Y        print the menu again
    Triggers and left joystick drive the car between sections.

Expected Outcome: every check in the chosen section prints PASS.
"""

import argparse
import os
import signal
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

import racecar_core

# add_help=False: racecar_core uses -h for headless mode
parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("--section", help="run one section and exit")
parser.add_argument("--angle", type=float, default=0.0, help="steering for driving sections")
args, _ = parser.parse_known_args()

rc = racecar_core.create_racecar()


class Report:
    """PASS/FAIL rows for one section."""

    def __init__(self, title: str) -> None:
        self.title = title
        self.rows: List[Tuple[str, bool, str]] = []

    def check(self, name: str, passed: bool, detail: str = "") -> None:
        self.rows.append((name, bool(passed), detail))

    def print(self) -> bool:
        width = max([len(name) for name, _, _ in self.rows] + [5])
        print(f"\n== {self.title}")
        for name, passed, detail in self.rows:
            print(f"  {'PASS' if passed else 'FAIL'}  {name.ljust(width)}  {detail}")
        failed = sum(1 for _, passed, _ in self.rows if not passed)
        print(f"  {len(self.rows) - failed}/{len(self.rows)} passed")
        return failed == 0


class Section:
    """A check that runs over one or more frames. step() returns True when finished."""

    title = ""

    def __init__(self) -> None:
        self.report = Report(self.title)
        self.start_time = time.monotonic()

    def elapsed(self) -> float:
        return time.monotonic() - self.start_time

    def step(self) -> bool:
        raise NotImplementedError


def finite(values: Any) -> bool:
    return bool(np.all(np.isfinite(np.asarray(values, dtype=float))))


class ApiSection(Section):
    """Calls every sensor read and actuator command once and checks the result types."""

    title = "api: sensor reads and actuator commands"

    def step(self) -> bool:
        r = self.report
        for name, read in (("linear acceleration", rc.physics.get_linear_acceleration),
                           ("angular velocity", rc.physics.get_angular_velocity),
                           ("magnetic field", rc.physics.get_magnetic_field)):
            value = read()
            r.check(f"{name} is 3 floats", value is not None and np.shape(value) == (3,) and finite(value),
                    np.array2string(np.asarray(value), precision=6) if value is not None else "None")
        for name, read in (("encoder speed", rc.physics.get_encoder_speed),
                           ("battery voltage", rc.physics.get_battery_voltage),
                           ("battery current", rc.physics.get_battery_current)):
            value = read()
            r.check(f"{name} is a float", isinstance(value, float) and finite(value), f"{value:.4f}")
        r.check("battery current not negative", rc.physics.get_battery_current() >= 0)

        samples = rc.lidar.get_samples()
        r.check("lidar samples", len(samples) == rc.lidar.get_num_samples() == 1080, f"{len(samples)}")

        matrix = rc.display.new_matrix()
        r.check("new_matrix shape", matrix.shape == (8, 24), f"{matrix.shape}")
        matrix[::2, ::2] = 1
        matrix[1::2, 1::2] = 1
        rc.display.set_matrix(matrix)
        r.check("set_matrix / get_matrix round trip", np.array_equal(rc.display.get_matrix(), matrix))
        rc.display.show_text("API OK")

        r.check("LED count", rc.led.get_num_pixels() == 84, f"{rc.led.get_num_pixels()}")
        colors = [(i * 3 % 256, 255 - i * 3 % 256, 40) for i in range(rc.led.get_num_pixels())]
        rc.led.set_pixels(colors)
        r.check("set_pixels / get_pixels round trip", [tuple(c) for c in rc.led.get_pixels()] == colors)
        rc.led.set_pixel(0, (255, 0, 0))
        r.check("set_pixel", tuple(rc.led.get_pixels()[0]) == (255, 0, 0))
        return True


class EncoderSection(Section):
    """Holds three speed commands at max speed 1.0 and compares the encoder with the target."""

    title = "encoder: steady speed against the command (max speed 1.0)"
    # (speed command, seconds); the last second of each step is averaged
    STEPS = [(0.25, 3.0), (0.5, 3.0), (1.0, 3.0), (0.0, 3.0)]
    FULL_SPEED = 4.0  # m/s at speed 1.0 and max speed 1.0
    # The speed loop overshoots a step by about 6% and settles over several
    # seconds, and the encoder reads wheel speed, which includes about 1% slip
    TOLERANCE = 0.05

    def __init__(self) -> None:
        super().__init__()
        self.index = 0
        self.step_start = time.monotonic()
        self.samples: List[float] = []
        rc.drive.set_max_speed(1.0)

    def step(self) -> bool:
        command, seconds = self.STEPS[self.index]
        rc.drive.set_speed_angle(command, args.angle if command else 0.0)
        t = time.monotonic() - self.step_start
        if t > seconds - 1.0:
            self.samples.append(rc.physics.get_encoder_speed())
        if t < seconds:
            return False

        target = command * self.FULL_SPEED
        mean = float(np.mean(self.samples)) if self.samples else float("nan")
        spread = float(np.std(self.samples)) if self.samples else float("nan")
        if command == 0:
            self.report.check("stopped", abs(mean) < 0.05, f"mean {mean:.3f} m/s")
        else:
            error = (mean - target) / target
            self.report.check(f"speed {command:.2f} -> {target:.1f} m/s", abs(error) <= self.TOLERANCE,
                              f"mean {mean:.3f} m/s, std {spread:.3f}, error {error:+.1%}")
        self.index += 1
        self.samples = []
        self.step_start = time.monotonic()
        if self.index < len(self.STEPS):
            return False
        rc.drive.set_speed_angle(0, 0)
        return True


SECTIONS: Dict[str, Tuple[Optional[Any], Callable[[], Section]]] = {
    "api": (rc.controller.Button.A, ApiSection),
    "encoder": (rc.controller.Button.B, EncoderSection),
}

active: Optional[Section] = None
results: List[bool] = []


def print_menu() -> None:
    print("\nSensor and actuator checks. Press:")
    for name, (button, factory) in SECTIONS.items():
        print(f"  {button.name:<3} {name:<8} {factory.title}")
    print("  Y   menu")


def finish() -> None:
    """Leave through the library's SIGINT path, which tells RacecarSim the program exited."""
    os.kill(os.getpid(), signal.SIGINT)


def start() -> None:
    global active
    rc.drive.stop()
    if args.section:
        active = SECTIONS[args.section][1]()
    else:
        print_menu()


def update() -> None:
    global active
    if active is not None:
        if active.step():
            results.append(active.report.print())
            active = None
            if args.section:
                finish()
        return

    for name, (button, factory) in SECTIONS.items():
        if rc.controller.was_pressed(button):
            print(f"\nRunning {name}...")
            active = factory()
            return
    if rc.controller.was_pressed(rc.controller.Button.Y):
        print_menu()

    # Manual driving between sections
    speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT) - rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    rc.drive.set_speed_angle(speed, angle)


if __name__ == "__main__":
    if args.section and args.section not in SECTIONS:
        print(f"Unknown section {args.section}; choose from {', '.join(SECTIONS)}")
        sys.exit(2)
    rc.set_start_update(start, update)
    rc.go()
