"""The head: nozzle rotation, vacuum, lights, E-stop. Every output goes through the controller as a G-code line
from the profile (D3, D18): the controller runs it after the moves before it."""
from __future__ import annotations

import time

from ..motion.controller import Controller
from ..profile.schema import Light as LightCfg
from ..profile.schema import Profile, ServoPwm, StepperAxis
from .base import Availability, Capability, CapabilityError, NoneCap, NotBuilt


# ---------------------------------------------------------------- rotation
class ServoRotation(Capability):
    """A hobby servo on the spindle PWM pin: M3 S<us>. No feedback: a settle wait after every turn, and every
    angle is reached from the same side (approach_us shorter first) so the gear play sits the same way."""
    role, kind = "rotation", "servo_pwm"

    def __init__(self, cfg: ServoPwm, ctl: Controller | None, firmware_ok: bool = True) -> None:
        self.cfg, self.ctl, self.firmware_ok = cfg, ctl, firmware_ok
        self.angle: float | None = None
        self.pulse: int | None = None
        self.table = None                    # the measured pulse -> angle table (Setup, step 6), when valid

    @property
    def accuracy_deg(self) -> float | None:
        if self.table is not None and self.table.accuracy_deg is not None:
            return float(self.table.accuracy_deg)
        return self.cfg.accuracy_deg

    @property
    def us_per_deg(self) -> float | None:
        c = self.cfg
        return None if not c.nominal_range_deg else (c.max_us - c.min_us) / c.nominal_range_deg

    @property
    def range_deg(self) -> tuple[float, float] | None:
        if self.table is not None:
            lo = self.table.angle_for(self.cfg.min_us + self.cfg.approach_us)
            hi = self.table.angle_for(self.cfg.max_us)
            return (min(lo, hi), max(lo, hi))
        k = self.us_per_deg
        if k is None:
            return None
        return ((self.cfg.min_us + self.cfg.approach_us - self.cfg.centre_us) / k,
                (self.cfg.max_us - self.cfg.centre_us) / k)

    def angle_nominal(self, us: int) -> float | None:
        k = self.us_per_deg
        return None if k is None else (us - self.cfg.centre_us) / k

    def angle_of_pulse(self, us: int) -> float | None:
        return self.table.angle_for(us) if self.table is not None else self.angle_nominal(us)

    def availability(self) -> Availability:
        if self.ctl is None:
            return Availability.no("The controller is not connected", "Press Connect")
        if not self.firmware_ok:
            return Availability.no("The controller firmware is not the servo build of the profile",
                                   "Flash it (firmware/README.md) or fix controller.firmware")
        if self.cfg.settle_s is None:
            return Availability.no("The servo settle time is TBD", "Measure it (Setup, step 6)")
        if self.us_per_deg is None and self.table is None:
            return Availability.no("The servo's angle per pulse is not known", "Write nominal_range_deg, or measure "
                                   "the angle table (Setup, step 6)")
        return Availability.yes()

    def pulse_for(self, deg: float) -> int:
        if self.table is not None:
            try:
                us = self.table.pulse_for(deg)
            except ValueError as err:
                raise CapabilityError(str(err)) from None
        else:
            us = round(self.cfg.centre_us + deg * self.us_per_deg)
        if not self.cfg.min_us <= us <= self.cfg.max_us:
            lo, hi = self.range_deg
            raise CapabilityError(f"{deg:+.1f}° is outside the servo range {lo:+.0f}° … {hi:+.0f}°")
        return us

    def turn_to(self, deg: float) -> None:
        self.require()
        self.set_pulse(self.pulse_for(deg))
        self.angle = float(deg)

    def set_pulse(self, us: int) -> None:
        c = self.cfg
        if not c.min_us <= us <= c.max_us:
            raise CapabilityError(f"pulse {us} µs is outside {c.min_us}-{c.max_us} µs")
        if c.approach_us > 0:
            self.ctl.gcode(c.command.format(us=max(c.min_us, us - c.approach_us)))
        self.ctl.gcode(c.command.format(us=us))
        self.ctl.gcode(f"G4 P{c.settle_s:g}", timeout=c.settle_s + 10)
        self.pulse = us
        self.angle = self.angle_of_pulse(us)

    def limp(self) -> None:
        if self.ctl is not None:
            self.ctl.gcode(self.cfg.limp)
        self.pulse = None


class StepperRotation(Capability):
    """A stepper on a controller axis (A / B / C) whose steps/mm setting is steps per degree."""
    role, kind = "rotation", "stepper_axis"

    def __init__(self, cfg: StepperAxis, ctl: Controller | None) -> None:
        self.cfg, self.ctl = cfg, ctl
        self.angle: float | None = None
        self.table: dict | None = None       # the Setup step's measurement: scale, backlash, repeat

    @property
    def accuracy_deg(self) -> float | None:
        if self.table and self.table.get("accuracy_deg") is not None:
            return float(self.table["accuracy_deg"])
        return self.cfg.accuracy_deg

    @property
    def range_deg(self) -> tuple[float, float]:
        return tuple(self.cfg.range_deg) if self.cfg.range_deg else (-180.0, 180.0)   # a stepper turns all round

    def command_for(self, deg: float) -> float:
        if not self.table:
            return deg
        return (deg - float(self.table.get("zero_deg", 0.0))) / float(self.table["scale"])

    def approach(self) -> float:
        if not self.table:
            return self.cfg.approach_deg
        # the marks read the backlash only to ~1° (a 1.5° play moves the tip 0.04 mm): come from well below it
        return max(self.cfg.approach_deg, float(self.table.get("backlash_deg", 0.0)) + 2.0)

    def availability(self) -> Availability:
        if self.ctl is None:
            return Availability.no("The controller is not connected", "Press Connect")
        return Availability.yes()

    def turn_to(self, deg: float) -> None:
        self.require()
        if not self.range_deg[0] <= deg <= self.range_deg[1]:
            raise CapabilityError(f"{deg:+.1f}° is outside the rotation range {self.range_deg}")
        self.command(self.command_for(deg))
        self.angle = float(deg)

    def command(self, cmd: float, approach: float | None = None) -> None:
        """Every angle from the same side (below), so the backlash always sits the same way."""
        a = self.approach() if approach is None else approach
        if a != 0:                                        # a < 0: from above (only the backlash measurement)
            self.ctl.gcode(f"G90 G1 {self.cfg.letter}{cmd - a:.3f} F{self.cfg.feed_deg_min:.0f}")
        self.ctl.gcode(f"G90 G1 {self.cfg.letter}{cmd:.3f} F{self.cfg.feed_deg_min:.0f}")
        self.ctl.sync()

    def limp(self) -> None:
        pass


# ---------------------------------------------------------------- vacuum
class ControllerVacuum(Capability):
    """Pump (and valve) on controller outputs by role. Without a sensor it can't tell if a part is held."""
    role = "vacuum"

    def __init__(self, p: Profile, ctl: Controller | None) -> None:
        self.cfg, self.ctl = p.vacuum, ctl
        self.kind = p.vacuum.type
        self.pump_on = False
        self.reader = None                   # () -> the sensor's value. The simulator wires its sensor here; HOW a real
                                             # controller reports the sensor input is TBD until the hardware exists
        self.threshold = p.vacuum.sensor.held_threshold if p.vacuum.sensor else None

    @property
    def can_sense(self) -> bool:
        return self.cfg.sensor is not None and self.reader is not None and self.threshold is not None

    def read(self) -> float | None:
        return None if self.reader is None else float(self.reader())

    def availability(self) -> Availability:
        if self.ctl is None:
            return Availability.no("The controller is not connected", "Press Connect")
        return Availability.yes()

    def pump(self, on: bool) -> None:
        self.require()
        self.ctl.gcode(self.cfg.pump.start if on else self.cfg.pump.stop)
        self.pump_on = on

    def pick(self) -> None:
        self.pump(True)
        if self.cfg.valve is not None:
            self.ctl.gcode(self.cfg.valve.start)

    def release(self) -> None:
        if self.cfg.valve is not None:
            self.ctl.gcode(self.cfg.valve.stop)           # often M9: it stops the pump too
            self.pump_on = self.cfg.valve.stop != self.cfg.pump.stop and self.pump_on
        else:
            self.pump(False)
            if self.cfg.release_s:                       # waited in the controller: after the moves, like the pump
                self.ctl.gcode(f"G4 P{self.cfg.release_s:g}", timeout=self.cfg.release_s + 10)

    def dwell(self) -> None:
        """Wait with the nozzle down and the pump on, so the vacuum builds up."""
        if self.cfg.pick_dwell_s:
            self.ctl.gcode(f"G4 P{self.cfg.pick_dwell_s:g}", timeout=self.cfg.pick_dwell_s + 10)

    def held(self) -> bool | None:
        """True / False from the vacuum sensor; None when there is none (or it is not set up): can't tell."""
        if not self.can_sense:
            return None
        return self.read() > self.threshold


# ---------------------------------------------------------------- lights
class ControllerLight(Capability):
    def __init__(self, role: str, cfg: LightCfg, ctl: Controller | None) -> None:
        self.role, self.kind, self.cfg, self.ctl = role, "controller", cfg, ctl
        self.on = False

    def availability(self) -> Availability:
        return Availability.yes() if self.ctl is not None else Availability.no("Not connected", "Press Connect")

    def set(self, on: bool) -> None:
        self.require()
        self.ctl.gcode(self.cfg.start if on else self.cfg.stop)
        self.on = on


def build_light(role: str, cfg: LightCfg | None, ctl: Controller | None) -> Capability:
    if cfg is None:
        return NoneCap(role, "No light in this profile")
    if cfg.via == "controller":
        return ControllerLight(role, cfg, ctl)
    return NotBuilt(role, cfg.via, "Phase G (lights)")


# ---------------------------------------------------------------- e-stop
class SoftwareEStop(Capability):
    """STOP on the page / space bar: cancel jogs, hold other motion."""
    role, kind = "estop", "software"

    def triggered(self, ctl: Controller) -> bool:
        return False

    def stop(self, ctl: Controller) -> None:
        ctl.stop()


class ControllerInputEStop(SoftwareEStop):
    """The E-stop button's second pole on a controller input: grblHAL shows 'E' in Pn and raises ALARM:10."""
    kind = "controller_input"

    def triggered(self, ctl: Controller) -> bool:
        st = ctl.link.status
        return st is not None and "E" in st.pins


def build_rotation(p: Profile, ctl: Controller | None, firmware_ok: bool) -> Capability:
    r = p.rotation
    if r.type == "servo_pwm":
        return ServoRotation(r.servo_pwm, ctl, firmware_ok)
    if r.type == "stepper_axis":
        return StepperRotation(r.stepper_axis, ctl)
    return NoneCap("rotation", "This nozzle does not turn")


def build_estop(p: Profile) -> Capability:
    return ControllerInputEStop() if p.estop.type == "controller_input" else SoftwareEStop()
