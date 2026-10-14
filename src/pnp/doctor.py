"""Doctor (docs/13 §4.11): a list of checks, each green / red / note, with a one-line fix: the profile and its TBD
fields, files, disk, the port keeper, the service, the controller and its settings, the cameras and the exposure,
HOME in view, the bed survey, every calibration of the validity chain, an unfinished job, the machine state."""
from __future__ import annotations

import os
import shutil
import socket
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import yaml

from .profile import Profile, missing

if TYPE_CHECKING:
    from .machine import Machine

MIN_FREE_GB = 1.0


@dataclass
class Check:
    name: str
    ok: bool | None          # None = a note (not wrong, but worth knowing)
    detail: str
    fix: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


def _port_keeper(url: str) -> Check:
    hostport = url.split("://", 1)[1]
    host, _, port = hostport.partition(":")
    try:
        with socket.create_connection((host, int(port.split("/")[0])), timeout=1.0):
            pass
        return Check("Port keeper", True, f"answers on {hostport}")
    except (OSError, ValueError) as err:
        return Check("Port keeper", False, f"{hostport}: {err}", "Start it: `pnp portd` (or its systemd service)")


def run(p: Profile, profile_path: Path, m: Machine | None = None, cfg_dir: Path | None = None,
        data_dir: Path | None = None) -> list[Check]:
    out = [Check("Profile", True, f"{p.name} ({profile_path.name}) is valid")]
    if m is not None and m.profile_error is not None:
        out.append(Check("Profile file", False, "; ".join(m.profile_error.problems),
                         "Fix the file: the running profile stays until it is valid"))
    for mi in missing(p):
        out.append(Check(f"TBD: {mi.field}", None, mi.effect, mi.fix))
    cfg_dir = cfg_dir or profile_path.parent.parent
    try:
        bed = yaml.safe_load((cfg_dir / p.bed_file).read_text())
        out.append(Check("Bed file", isinstance(bed, dict) and "markers" in bed, f"{p.bed_file}"))
    except (OSError, yaml.YAMLError) as err:
        out.append(Check("Bed file", False, f"{p.bed_file}: {err}", "Create it (docs/05-feed-tray.md)"))
    if data_dir is not None:
        free = shutil.disk_usage(data_dir).free / 1e9
        out.append(Check("Free disk", free >= MIN_FREE_GB, f"{free:.1f} GB free in {data_dir}",
                         "Delete old job photos or logs"))
        out.append(Check("Data folder", os.access(data_dir, os.W_OK), str(data_dir), "Make it writable"))
    port = p.controller.port or ""
    if "://" in port and port.startswith("socket://"):
        out.append(_port_keeper(port))
    elif m is None or not m.sim_mode:
        out.append(Check("Port keeper", None, "not used: a restart of the program may reset an Arduino Uno",
                         "Run `pnp portd` and set controller.port: socket://127.0.0.1:7450 (D22)"))
    service = os.environ.get("PNP_SERVICE") == "1"            # set by deploy/systemd/pnp.service
    out.append(Check("Service", True if service else None, "running as the systemd service" if service else
                     "started by hand", "" if service else "Install the systemd units (deploy/systemd/README.md)"))
    if m is None:
        return out
    if m.sim_mode:
        out.append(Check("Simulator", None, "this is the simulator: nothing real moves"))
    ctl = m.ctl
    if ctl is None:
        out.append(Check("Controller", False, "not connected", "Press Connect (Machine tab)"))
    else:
        st = ctl.link.status
        fresh = st is not None and time.monotonic() - st.t < 2 * p.controller.watchdog_s
        out.append(Check("Controller", fresh and not ctl.link.lost,
                         f"{ctl.info.version or '?'}, {st.state if st else 'no status'}",
                         "Check the USB cable and the controller power, then Connect"))
        if p.controller.firmware:
            out.append(Check("Firmware", ctl.info.firmware_ok, f"{ctl.info.version} (profile: {p.controller.firmware})",
                             "Flash the right build (firmware/README.md) or fix controller.firmware"))
        bad = ctl.info.mismatched
        out.append(Check("Controller settings", not bad,
                         "as the profile expects" if not bad else
                         ", ".join(f"{k}={v[1]} (want {v[0]})" for k, v in bad.items()),
                         "Set them in the controller (G-code console: $n=value)"))
        if p.controller.session_token:
            out.append(Check("Session token", m.session.token is not None,
                             f"T{m.session.token}" if m.session.token else "not written",
                             "Connect again; if it stays, a reset can't be seen after a restart"))
    for role, cam in m.cameras.items():
        a = cam.availability()
        if cam.kind != "none":
            out.append(Check(f"Camera ({role.removeprefix('camera_')})", a.ok, "frames arrive" if a.ok else a.why,
                             a.todo))
    out += _vision_checks(m)
    b = m.safety.banner()
    out.append(Check("Machine state", True if b["level"] == "ok" else (None if b["level"] == "warn" else False),
                     b["title"], b.get("todo", "")))
    return out


def _vision_checks(m: Machine) -> list[Check]:
    """The camera's light, HOME in the picture, and every calibration result: valid / stale / missing (§4.10)."""
    from .calibration import chain
    out = []
    try:
        problem = m.vision.light_problem()
    except Exception as err:                                     # noqa: BLE001 - a check never breaks the page
        problem = f"can't check: {err}"
    if problem is not None:
        out.append(Check("Exposure", None if "never set" in problem else False, problem,
                         "Machine tab: Find and lock the exposure (lights on)"))
    elif m.cameras.get("camera_head") is not None:
        out.append(Check("Exposure", True, "the picture is as bright as when the exposure was locked"))
    cam = m.cameras.get("camera_head")
    model = m.vision.model()
    if cam is not None and model is not None and m.markers is not None and getattr(cam, "latest", None):
        last = cam.latest()
        if last is not None:
            seen = m.markers.find(last[1])
            out.append(Check("Labels in view", None, ", ".join(sorted(seen)) or "none (that is fine away from them)"))
    for key, spec in chain.CHAIN.items():
        st = chain.status(m, key)
        if st["state"] == "valid":
            out.append(Check(spec.title, True, f"valid, {st.get('date')}"))
        elif st["state"] == "stale":
            out.append(Check(spec.title, False, f"stale: {st['why']}", "Measure it again (Setup tab)"))
        else:
            out.append(Check(spec.title, None, "not measured yet", "Setup tab"))
    v = chain.value(m, "bed_survey")
    if v is not None:
        lab = v.get("labels", {})
        missing = [t.name for t in m.feeders.trays.values() if t.name not in lab]
        out.append(Check("Bed survey", not missing, f"{len(lab)} labels measured" +
                         (f"; not found: {', '.join(missing)}" if missing else ""),
                         "Survey again (Setup, step 8) after moving or adding trays"))
    j = m.jobs.job if hasattr(m, "jobs") else None
    if j is not None and j.status in ("paused", "running"):
        out.append(Check("Job", None, f"{j.id} is {j.status}: {j.pause.get('why', '')}".strip(": "),
                         "Job tab: resume it or abort it"))
    return out
