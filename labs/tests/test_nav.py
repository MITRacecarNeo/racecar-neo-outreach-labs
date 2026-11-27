"""
MIT BWSI Autonomous RACECAR
GNU General Public License v3.0
racecar-neo-outreach-labs

File Name: test_nav.py

Title: Navigation checks

Purpose: Exercise rc.nav (go_to, move, missions, cancel, status) with the
car's navigator and report each check as PASS or FAIL. Physical car only;
RacecarSim has no navigator.

Before running: load a saved map in the Planner (http://<car>:8090) or with
`racecar map load NAME`. Set the pose if the car is not localized. Clear
about 3 m around the car. Hold RB while the car drives.

Usage:
    python test_nav.py                        interactive
    python test_nav.py --section status       run one section, print the report, exit
    python test_nav.py --section back         press BACK during a goal, then START
    python test_nav.py --section busy         with a Planner mission running
    python test_nav.py --mission NAME         the mission for LJOY (default: the first saved)

Interactive controls, in user program mode:
    A     status   status word, missions, the car held by this program
    B     go       go_to 1.5 m ahead and back. rc.drive and repeated go_to
                   calls do nothing meanwhile.
    X     move     move(1.0): the car drives about 1 m
    LJOY  mission  run a saved mission to its end
    RJOY  cancel   a 3 m goal, canceled after 1.5 s. The car stops.
    Y     print the menu again
    Triggers and left joystick drive the car between sections.

Expected Outcome: every check in the chosen section prints PASS.
"""

import argparse
import math
import os
import signal
import sys
import time
from typing import Any, Dict, List, Optional, Tuple, Type

import racecar_core

# add_help=False: racecar_core uses -h for headless mode
parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("--section", help="run one section and exit")
parser.add_argument("--mission", help="the mission to run")
args, _ = parser.parse_known_args()

rc = racecar_core.create_racecar()

ARRIVE_M = 0.4  # the waypoint radius (0.25 m) plus the stop


class Report:
    """PASS/FAIL rows for one section."""

    def __init__(self, title: str) -> None:
        self.title = title
        self.rows: List[Tuple[str, bool, str]] = []

    def check(self, name: str, passed: bool, detail: str = "") -> None:
        self.rows.append((name, bool(passed), detail))

    def print(self) -> None:
        width = max([len(name) for name, _, _ in self.rows] + [5])
        print(f"\n== {self.title}")
        for name, passed, detail in self.rows:
            print(f"  {'PASS' if passed else 'FAIL'}  {name.ljust(width)}  {detail}")
        failed = sum(1 for _, passed, _ in self.rows if not passed)
        print(f"  {len(self.rows) - failed}/{len(self.rows)} passed")


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


def manual_drive() -> None:
    speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT) - rc.controller.get_trigger(
        rc.controller.Trigger.LEFT
    )
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    rc.drive.set_speed_angle(speed, angle)


def ahead(distance: float) -> Optional[Tuple[float, float, float]]:
    """Return the point `distance` m ahead of the car, keeping its heading."""
    pose = rc.slam.get_pose()
    if pose is None:
        return None
    x, y, heading = pose
    rad = math.radians(heading)
    return x + distance * math.cos(rad), y + distance * math.sin(rad), heading


def distance_to(point: Tuple[float, float, float]) -> float:
    pose = rc.slam.get_pose()
    return math.inf if pose is None else math.hypot(pose[0] - point[0], pose[1] - point[1])


def ready(report: Report) -> bool:
    status = rc.nav.get_status()
    if status == rc.nav.OFF:
        report.check("navigation on", False, "Load a map in the Planner (port 8090). Run the test again.")
        return False
    if not rc.slam.is_localized():
        report.check("localized", False, "Use Set pose in the Planner. Run the test again.")
        return False
    return True


class StatusSection(Section):
    """The status word, the saved missions, and the car held by this program."""

    title = "status: status word, missions, the car held"

    def step(self) -> bool:
        r = self.report
        status = rc.nav.get_status()
        r.check("navigation on", status != rc.nav.OFF, f"status {status!r}")
        r.check("no goal of this program", not rc.nav.is_active(), status)
        r.check("this program holds the car", rc.drive.holds_car(), rc.drive.refused_by())
        missions = rc.nav.get_missions()
        r.check("missions listed", isinstance(missions, list), ", ".join(missions) or "none saved")
        r.check("no distance without a goal", rc.nav.get_distance_left() is None, "")
        return True


class GoSection(Section):
    """go_to 1.5 m ahead and back, with rc.drive and go_to called every frame meanwhile."""

    title = "go: go_to ahead and back, drive and repeats ignored"
    TIMEOUT_S = 40.0

    def __init__(self) -> None:
        super().__init__()
        self.legs: List[Tuple[float, float, float]] = []
        self.leg = 0
        self.repeats_accepted = 0
        self.leg_start = time.monotonic()

    def step(self) -> bool:
        r = self.report
        if not self.legs:
            if not ready(r):
                return True
            there, home = ahead(1.5), ahead(0.0)
            self.legs = [there, home]
            accepted = rc.nav.go_to(*there)
            r.check("go_to accepted", accepted, "" if accepted else rc.drive.refused_by())
            self.leg_start = time.monotonic()
            return not accepted
        # Students' drive code keeps running in the same loop.
        rc.drive.set_speed_angle(1.0, 0.0)
        self.repeats_accepted += rc.nav.go_to(*self.legs[self.leg])
        status = rc.nav.get_status()
        if status in (rc.nav.NAVIGATING, rc.nav.WAITING, rc.nav.LOST, rc.nav.PAUSED):
            if time.monotonic() - self.leg_start < self.TIMEOUT_S:
                return False
        target = self.legs[self.leg]
        r.check(f"leg {self.leg + 1} arrived", status == rc.nav.ARRIVED,
                f"{status!r}, {distance_to(target):.2f} m from the goal")
        r.check(f"leg {self.leg + 1} within {ARRIVE_M} m", distance_to(target) <= ARRIVE_M, "")
        if self.leg == 0:
            self.leg = 1
            r.check("go_to back accepted", rc.nav.go_to(*self.legs[1]), "")
            self.leg_start = time.monotonic()
            return False
        rc.drive.stop()
        r.check("repeated go_to refused", self.repeats_accepted == 0,
                f"{self.repeats_accepted} accepted")
        return True


class MoveSection(Section):
    """move(1.0): the car drives about 1 m along its heading."""

    title = "move: move(1.0)"
    TIMEOUT_S = 30.0

    def __init__(self) -> None:
        super().__init__()
        self.start_pose = rc.slam.get_pose()
        self.encoder = 0.0
        self.sent = False

    def step(self) -> bool:
        r = self.report
        if not self.sent:
            if not ready(r):
                return True
            r.check("move accepted", rc.nav.move(1.0), "")
            self.sent = True
            return False
        self.encoder += abs(rc.physics.get_encoder_speed()) * rc.get_delta_time()
        if rc.nav.is_active() and self.elapsed() < self.TIMEOUT_S:
            return False
        r.check("arrived", rc.nav.get_status() == rc.nav.ARRIVED, rc.nav.get_status())
        pose = rc.slam.get_pose()
        moved = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1])
        r.check("moved about 1 m", abs(moved - 1.0) <= ARRIVE_M,
                f"pose {moved:.2f} m, encoder {self.encoder:.2f} m")
        return True


class MissionSection(Section):
    """A saved mission run to its end."""

    title = "mission: run_mission to the end"
    TIMEOUT_S = 180.0

    def __init__(self) -> None:
        super().__init__()
        self.sent = False

    def step(self) -> bool:
        r = self.report
        if not self.sent:
            if not ready(r):
                return True
            names = rc.nav.get_missions()
            name = args.mission or (names[0] if names else "")
            if not name:
                r.check("a saved mission", False, "Save one in the Planner. Run the test again.")
                return True
            r.check(f"run_mission {name!r} accepted", rc.nav.run_mission(name), "")
            self.sent = True
            return False
        if rc.nav.is_active() and self.elapsed() < self.TIMEOUT_S:
            return False
        r.check("arrived", rc.nav.get_status() == rc.nav.ARRIVED, rc.nav.get_status())
        return True


class CancelSection(Section):
    """A 3 m goal canceled after 1.5 s: the car stops and the status reads canceled."""

    title = "cancel: cancel() during a goal"

    def __init__(self) -> None:
        super().__init__()
        self.sent = False
        self.canceled_at: Optional[float] = None

    def step(self) -> bool:
        r = self.report
        if not self.sent:
            if not ready(r):
                return True
            r.check("go_to accepted", rc.nav.go_to(*ahead(3.0)), "")
            self.sent = True
            return False
        if self.canceled_at is None:
            if self.elapsed() < 1.5:
                return False
            rc.nav.cancel()
            self.canceled_at = time.monotonic()
            r.check("status canceled", rc.nav.get_status() == rc.nav.CANCELED, rc.nav.get_status())
            r.check("no longer active", not rc.nav.is_active(), "")
            return False
        speed = abs(rc.physics.get_encoder_speed())
        if speed > 0.05 and time.monotonic() - self.canceled_at < 3.0:
            return False
        r.check("car stopped", speed <= 0.05, f"{speed:.2f} m/s after "
                f"{time.monotonic() - self.canceled_at:.1f} s")
        return True


class BusySection(Section):
    """With a Planner mission running, go_to is refused and names the Planner."""

    title = "busy: go_to while the Planner drives"

    def step(self) -> bool:
        r = self.report
        r.check("go_to refused", not rc.nav.go_to(*(ahead(1.0) or (0.0, 0.0, 0.0))), "")
        r.check("rc.drive refused", not rc.drive.holds_car(), rc.drive.refused_by())
        r.check("named the Planner", "Planner" in rc.drive.refused_by(), "")
        return True


class BackSection(Section):
    """Starts a 3 m goal; the tester presses BACK, then START, and start() checks it."""

    title = "back: BACK cancels the goal"

    def step(self) -> bool:
        global back_pending
        r = self.report
        if not ready(r):
            return True
        r.check("go_to accepted", rc.nav.go_to(*ahead(3.0)), "")
        back_pending = True
        print("  Press BACK now. Then press START.")
        return True


SECTIONS: Dict[str, Tuple[Any, Type[Section]]] = {
    "status": (rc.controller.Button.A, StatusSection),
    "go": (rc.controller.Button.B, GoSection),
    "move": (rc.controller.Button.X, MoveSection),
    "mission": (rc.controller.Button.LJOY, MissionSection),
    "cancel": (rc.controller.Button.RJOY, CancelSection),
}
ONLY_BY_NAME: Dict[str, Type[Section]] = {"busy": BusySection, "back": BackSection}

active: Optional[Section] = None
back_pending = False


def print_menu() -> None:
    print("\nNavigation checks. Press:")
    for name, (button, factory) in SECTIONS.items():
        print(f"  {button.name:<5} {name:<8} {factory.title}")
    print("  Y     menu")


def finish() -> None:
    """Leave through the library's SIGINT path."""
    os.kill(os.getpid(), signal.SIGINT)


def start() -> None:
    global active, back_pending
    rc.drive.stop()
    if back_pending:
        back_pending = False
        r = Report("back: BACK cancels the goal")
        r.check("status canceled after BACK", rc.nav.get_status() == rc.nav.CANCELED,
                rc.nav.get_status())
        r.check("no longer active", not rc.nav.is_active(), "")
        r.print()
        if args.section:
            finish()
            return
    if args.section:
        factory = SECTIONS[args.section][1] if args.section in SECTIONS else ONLY_BY_NAME[args.section]
        active = factory()
    else:
        print_menu()


def update() -> None:
    global active
    if active is not None:
        if active.step():
            active.report.print()
            active = None
            if args.section and not back_pending:
                finish()
        return

    for name, (button, factory) in SECTIONS.items():
        if rc.controller.was_pressed(button):
            print(f"\nRunning {name}...")
            active = factory()
            return
    if rc.controller.was_pressed(rc.controller.Button.Y):
        print_menu()
    manual_drive()


if __name__ == "__main__":
    if args.section and args.section not in SECTIONS and args.section not in ONLY_BY_NAME:
        choices = ", ".join([*SECTIONS, *ONLY_BY_NAME])
        print(f"Unknown section {args.section}. Choose from: {choices}")
        sys.exit(2)
    rc.set_start_update(start, update)
    rc.go()
