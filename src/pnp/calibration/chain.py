"""The validity chain (docs/13 §4.10, §4.14): what every calibration result depends on.

A result is saved with its inputs: hardware ids from the profile (camera, nozzle, rotation axis) and the dates of
the results it was built on. When an input changes (a new camera id, the camera calibrated again, another nozzle),
the result is STALE and the wizard says why. Nothing here measures: it only names the inputs.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..machine import Machine


@dataclass(frozen=True)
class Spec:
    title: str
    hardware: tuple[str, ...]        # profile ids it was measured with: camera, nozzle, rotation, homing
    upstream: tuple[str, ...]        # other results it is built on (their dates are inputs)


CHAIN: dict[str, Spec] = {
    "exposure_head": Spec("Camera exposure", ("camera",), ()),
    "exposure_head_off": Spec("Camera exposure, light off", ("camera",), ()),
    "camera_head": Spec("Camera calibration", ("camera",), ()),
    "travel": Spec("Travel (soft limits)", ("homing",), ()),
    "bed_survey": Spec("Bed survey (labels)", ("camera", "homing"), ("camera_head",)),
    "nozzle_offset": Spec("Nozzle offset", ("camera", "nozzle", "rotation"), ("camera_head",)),
    "rotation_table": Spec("Rotation table", ("rotation",), ("camera_head",)),   # the servo, not the needle
    "first_part": Spec("First part correction", ("rotation",), ()),   # the operator's correction, all nozzles
    "bottom_camera": Spec("Bottom camera", ("bottom_camera", "nozzle"), ("nozzle_offset",)),
    "vacuum_sensor": Spec("Vacuum sensor", ("vacuum", "nozzle"), ()),
}

PER_NOZZLE = {"nozzle_offset", "bottom_camera"}     # kept for each nozzle: <key>@<nozzle id>


def skey(m: Machine, key: str) -> str:
    """Where a result is stored: per-nozzle results under the CURRENT nozzle (Phase G: nozzle changes)."""
    return f"{key}@{m.nozzle_current()}" if key in PER_NOZZLE else key


def entry(m: Machine, key: str) -> dict | None:
    return m.calib.get(skey(m, key))


FRIENDLY = {"camera": "the camera (its hardware id)", "nozzle": "the nozzle", "rotation": "the rotation axis",
            "homing": "the homing method", "board": "the board", "token": "the controller session"}


def _hash(obj) -> str:
    return hashlib.sha1(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:10]


def hardware(m: Machine) -> dict[str, str]:
    p = m.profile
    rot = p.rotation
    rot_cfg = rot.model_dump(exclude={"servo_pwm": {"accuracy_deg", "settle_s"}}) if rot.type == "servo_pwm" else \
        rot.model_dump()
    return {"camera": p.cameras.head.hardware_id if p.cameras.head else "none",
            "bottom_camera": p.cameras.bottom.hardware_id if p.cameras.bottom else "none",
            "nozzle": m.nozzle_current(),
            "rotation": f"{rot.type}:{_hash(rot_cfg)}",
            "homing": f"{p.homing.method}:{p.motion.travel.frame}",
            "vacuum": f"{p.vacuum.type}:{_hash(p.vacuum.model_dump())}"}


def inputs(m: Machine, key: str) -> dict:
    """The inputs a result of `key` is saved with, and checked against, now."""
    spec = CHAIN[key]
    hw = hardware(m)
    out = {k: hw[k] for k in spec.hardware}
    for up in spec.upstream:
        out[up] = m.calib.date(skey(m, up))
    return out


def status(m: Machine, key: str) -> dict:
    """{state: valid | stale | missing, why, date, method, quality}."""
    e = m.calib.get(skey(m, key))
    if e is None:
        return {"state": "missing", "why": "not measured yet" + (f" (nozzle {m.nozzle_current()})"
                                                                  if key in PER_NOZZLE else ""), "date": None}
    have = e.get("meta", {}).get("inputs") or {}
    now = inputs(m, key)
    for k, v in now.items():
        if have.get(k) != v:
            if k in CHAIN:
                why = f"{CHAIN[k].title} was " + ("measured again since" if v else "removed") + ": measure this again"
            else:
                why = f"{FRIENDLY.get(k, k)} changed since: measure this again"
            return {"state": "stale", "why": why, "date": e["meta"].get("date"), "method": e["meta"].get("method")}
    return {"state": "valid", "why": "", "date": e["meta"].get("date"), "method": e["meta"].get("method"),
            "quality": e["meta"].get("quality", {})}


def value(m: Machine, key: str) -> dict | None:
    """The value if it is valid now, else None."""
    return m.calib.get(skey(m, key))["value"] if status(m, key)["state"] == "valid" else None


def put(m: Machine, key: str, val: dict, method: str, quality: dict | None = None) -> None:
    m.calib.put(skey(m, key), val, inputs(m, key), method, quality)


def dependants(key: str) -> list[str]:
    """Results that become stale when `key` is measured again (the chain shown in the wizard)."""
    out, todo = [], [key]
    while todo:
        k = todo.pop()
        for name, spec in CHAIN.items():
            if k in spec.upstream and name not in out:
                out.append(name)
                todo.append(name)
    return out
