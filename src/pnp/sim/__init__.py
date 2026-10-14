"""The simulator (docs/13 §4.12): a virtual controller and a virtual camera over a world drawn from bed.yaml.
Used by the tests and by `pnp sim` (the whole page without hardware)."""
from __future__ import annotations

from ..profile.schema import Profile
from ..vision.camera_model import CameraModel
from .camera import SimCamera
from .controller import SimGrbl, SimPort
from .nozzle import NozzleTruth, Placed, SimNozzle
from .world import BoardPose, Pocket, Strip, World, board_layer, board_layers, charuco_layer, extent_for, strip_layer


def true_camera(profile: Profile) -> CameraModel:
    """The simulator's real camera for this profile (the program does not know it)."""
    head = profile.cameras.head
    size = tuple(head.resolution)
    c = profile.sim.camera
    if c is None:
        return CameraModel.looking_down(size, profile.sim.camera_px_per_mm * 80.0, 80.0,
                                        turn_deg=profile.sim.camera_turn_deg)
    m = CameraModel.looking_down(size, c.f_px, c.height_mm, c.tilt_deg, c.tilt_dir_deg, tuple(c.dist), c.turn_deg)
    m.moves_with_z = bool(head.moves_with_z)
    return m


def build(profile: Profile, bed: dict, seed: int = 0, speed: float | None = None,
          grbl: SimGrbl | None = None, run_cameras: bool = True, trays: list | None = None,
          dictionary: str | None = None, world: World | None = None) -> tuple[SimGrbl, dict]:
    """The virtual controller (a new one, or `grbl` reused) and the cameras for a profile."""
    grbl = grbl or SimGrbl(profile, seed=seed)
    if speed is not None:
        grbl.speed = speed
    cams = {}
    head = profile.cameras.head
    if head is not None:
        world = world or World(bed, extent_for(profile.sim.hard_ends), trays=trays, dictionary=dictionary)
        light = profile.lights.head
        port = None
        if light is not None and light.via == "controller":
            f = _flag_of(light.start or "")
            port = int(f[4:]) if f.startswith("dout") else None
        cams["camera_head"] = SimCamera("camera_head", world, grbl, true_camera(profile), head.exposure.value,
                                        seed=seed, run=run_cameras, light_port=port)
        s = profile.sim
        if s.nozzle_offset_mm is not None:
            grbl.hooks[:] = [h for h in grbl.hooks if not isinstance(getattr(h, "__self__", None), SimNozzle)]
            flag = _flag_of(profile.vacuum.pump.start)
            valve = _flag_of(profile.vacuum.valve.start) if profile.vacuum.valve is not None else None
            SimNozzle(grbl, world, NozzleTruth(tuple(s.nozzle_offset_mm), s.nozzle_runout_mm,
                                               s.nozzle_runout_phase_deg, s.nozzle_spring_mm), seed, flag, valve)
    if profile.sim.bottom_camera is not None and profile.cameras.bottom is not None:
        from .bottom import SimBottomCamera
        noz = next((n for n in profile.nozzles.items if n.id == profile.nozzles.current), None)
        cams["camera_bottom"] = SimBottomCamera("camera_bottom", grbl, profile.sim.bottom_camera,
                                                tuple(profile.cameras.bottom.resolution), seed=seed, run=run_cameras,
                                                od_mm=(noz.od_mm if noz and noz.od_mm else 0.72))
    return grbl, cams


def _flag_of(code: str) -> str:
    """Which simulated output a G-code line switches: M8 flood, M7 mist, M62/M64 P<n> a digital output."""
    import re
    c = code.strip().upper()
    m = re.match(r"M6[24]\s*P(\d+)", c)
    if m:
        return f"dout{m.group(1)}"
    return "mist" if c.startswith("M7") else "flood"


def nozzle_of(grbl: SimGrbl) -> SimNozzle | None:
    """The simulated nozzle on this virtual controller, if the profile has one."""
    return next((h.__self__ for h in grbl.hooks if isinstance(getattr(h, "__self__", None), SimNozzle)), None)


__all__ = ["BoardPose", "NozzleTruth", "Placed", "SimNozzle", "nozzle_of", "Pocket", "SimCamera", "SimGrbl", "SimPort", "Strip", "World", "board_layer", "board_layers", "build",
           "charuco_layer", "extent_for", "strip_layer", "true_camera"]
