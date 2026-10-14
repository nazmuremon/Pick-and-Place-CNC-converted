"""Homing (XY) and the Z reference."""
from __future__ import annotations

import re
import time

from ..motion.controller import Controller
from ..motion.grbl import ControllerError
from ..profile.schema import Profile
from .base import Availability, Capability, CapabilityError, NoneCap, NotBuilt


class ManualHoming(Capability):
    """The operator jogs the camera cross onto the HOME label's centre and presses Home."""
    role, kind, restores_after_reset = "homing", "manual", False

    def home(self, ctl: Controller) -> dict[str, float]:
        raw = ctl.raw()
        return {"X": raw["X"], "Y": raw["Y"]}


class SwitchHoming(Capability):
    """$H with the controller's limit switches. The controller's machine frame is then the travel frame."""
    role, kind, restores_after_reset = "homing", "switches", True

    def __init__(self, timeout_s: float = 120.0) -> None:
        self.timeout_s = timeout_s

    def home(self, ctl: Controller) -> dict[str, float]:
        ctl.gcode("$H", timeout=self.timeout_s)
        ctl.wait_idle(time.monotonic(), 5.0)
        return {"X": 0.0, "Y": 0.0}


class TouchOff(Capability):
    """The operator jogs the nozzle down until the tip just touches a known surface, then presses 'Set Z here'."""
    role, kind = "z_reference", "touch_off"

    def establish(self, ctl: Controller, surface_z: float) -> float:
        return ctl.raw()["Z"] - surface_z


PRB = re.compile(r"\[PRB:([-\d.,]+):(\d)\]")


class ProbeZ(Capability):
    """G38.2 down until the probe input triggers, then lift by retract_mm."""
    role, kind = "z_reference", "probe"

    def __init__(self, feed: float, max_depth_mm: float, retract_mm: float) -> None:
        self.feed, self.max_depth, self.retract = feed, max_depth_mm, retract_mm

    def establish(self, ctl: Controller, surface_z: float) -> float:
        timeout = self.max_depth / (self.feed / 60.0) + 10.0
        try:
            replies = ctl.gcode(f"G38.2 G91 {ctl.axes.word('Z', -self.max_depth)} F{self.feed:.0f}", timeout=timeout)
        except ControllerError as err:
            raise CapabilityError(f"The probe did not trigger within {self.max_depth:g} mm ({err})") from None
        finally:
            try:
                ctl.gcode("G90")
            except ControllerError:
                pass
        hit = next((PRB.search(r) for r in replies + ctl.link.messages[-3:] if PRB.search(r)), None)
        if hit is None or hit.group(2) != "1":
            raise CapabilityError("The probe did not report a touch")
        nums = [float(v) for v in hit.group(1).split(",")]
        z_raw = ctl.axes.to_machine({"XYZABC"[i]: v for i, v in enumerate(nums)})["Z"]
        if self.retract > 0:
            t = ctl.jog({"Z": self.retract}, self.feed)
            ctl.wait_idle(t, 30.0)
        return z_raw - surface_z


def build_homing(p: Profile) -> Capability:
    if p.homing.method == "manual":
        return ManualHoming()
    if p.homing.method == "switches":
        return SwitchHoming()
    return NotBuilt("homing", "camera_marker", "Phase C (vision)")


def build_z_reference(p: Profile) -> Capability:
    z = p.z_reference
    if z.method == "touch_off":
        return TouchOff()
    if z.method == "probe":
        return ProbeZ(z.probe.feed, z.probe.max_depth_mm, z.probe.retract_mm)
    if z.method == "switch":
        return NotBuilt("z_reference", "switch", "Phase G (with a Z switch)")
    return NoneCap("z_reference", "No Z reference in this profile")


__all__ = ["Availability", "ManualHoming", "ProbeZ", "SwitchHoming", "TouchOff", "build_homing", "build_z_reference"]
