"""`pnp init`: the first run on a new machine (docs/13 §4.13). It asks a few questions and writes a profile, then
copies the example bed and the package library into the config folder. Every answer that is not known stays null
(TBD): the program switches that capability off and the Setup tab says what to measure.
"""
from __future__ import annotations

import shutil
from pathlib import Path
from typing import Callable

import yaml

from . import profile as prof

DATA = Path(__file__).resolve().parent / "data"


def _ask(ask: Callable[[str, str], str], q: str, default: str, choices: tuple[str, ...] | None = None) -> str:
    while True:
        a = (ask(f"{q}" + (f" [{'/'.join(choices)}]" if choices else "") + f" ({default}): ", default) or default).strip()
        if not choices or a in choices:
            return a


def run(cfg_dir: Path, ask: Callable[[str, str], str] | None = None) -> Path:
    ask = ask or (lambda q, d: input(q))
    base = yaml.safe_load((DATA / "profiles" / "pico_grblhal.yaml").read_text())
    name = _ask(ask, "A short name for this machine", "my_pnp")
    ctype = _ask(ask, "Controller", "grblhal", ("grbl", "grblhal"))
    port = _ask(ask, "Controller port (empty = the first USB serial port)", "")
    homing = _ask(ask, "Homing", "camera_marker", ("camera_marker", "switches", "manual"))
    zref = _ask(ask, "Z reference", "touch_off", ("touch_off", "probe", "none"))
    rot = _ask(ask, "Nozzle rotation", "servo_pwm", ("none", "servo_pwm", "stepper_axis"))
    cam = _ask(ask, "Head camera device (e.g. /dev/video0)", "/dev/video0")
    res = _ask(ask, "Camera resolution WxH", "1280x720")
    p = dict(base)
    p["name"], p["title"] = name, f"{name} (made by pnp init)"
    p["controller"] = dict(base["controller"], type=ctype, port=port or None, firmware=None, expected_settings={})
    p["homing"] = {"method": homing}
    if homing == "switches":
        p["motion"] = dict(base["motion"], travel={"frame": "controller", "x": None, "y": None, "z": None})
    p["z_reference"] = {"method": zref} if zref != "probe" else \
        {"method": "probe", "probe": {"feed": 100, "max_depth_mm": 10, "retract_mm": 2}}
    if rot == "none":
        p["rotation"] = {"type": "none"}
    elif rot == "stepper_axis":
        p["rotation"] = {"type": "stepper_axis", "stepper_axis": {"letter": "A", "feed_deg_min": 3600}}
    w, _, h = res.partition("x")
    p["cameras"] = {"head": {"driver": "v4l2", "device": cam, "resolution": [int(w), int(h)], "fourcc": "MJPG",
                             "hardware_id": f"{name}-camera-1", "moves_with_z": None, "tilted": None}}
    p.pop("sim", None)
    pdir = cfg_dir / "profiles"
    pdir.mkdir(parents=True, exist_ok=True)
    out = pdir / f"{name}.yaml"
    head = ("# Made by `pnp init`. null = TBD: that capability is off until you measure it (Setup tab).\n"
            "# Profile reference: docs/builder/profiles.md\n")
    out.write_text(head + yaml.safe_dump(p, sort_keys=False))
    prof.load(out)                                              # it must be valid
    for src, dst in (("bed.example.yaml", "bed.yaml"), ("packages.yaml", "packages.yaml")):
        if not (cfg_dir / dst).exists():
            shutil.copy(DATA / src, cfg_dir / dst)
    return out
