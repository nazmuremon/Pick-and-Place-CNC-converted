"""Export / import bundle (docs/13 §4.10): one zip with everything that makes this machine THIS machine, for support
and for cloning a machine. config/ (profiles, bed.yaml, feeders.yaml, packages.user.yaml, calibration.yaml) and data/
(boards, the parts store, what the trays hold, the placed state; job reports, photos only on request). The session
(the controller's position) is never exported: a new machine homes itself.

Import never overwrites silently: what is there is zipped into data/config_history/ first.
"""
from __future__ import annotations

import io
import json
import time
import zipfile
from pathlib import Path

from . import __version__

CONFIG = ("profiles/*.yaml", "bed.yaml", "feeders.yaml", "packages.user.yaml", "calibration.yaml")
DATA = ("boards/**/*", "store.json", "feeder_state.json", "templates/*", "jobs/*/*/state.json", "jobs/*/*/report.html")
NEVER = ("session", "events", "active_profile", "_imports")


def _files(root: Path, patterns, extra=()):
    seen = set()
    for pat in tuple(patterns) + tuple(extra):
        for p in root.glob(pat):
            if p.is_file() and not any(n in str(p.relative_to(root)) for n in NEVER) and p not in seen:
                seen.add(p)
                yield p


def export(cfg_dir: Path, data_dir: Path, photos: bool = False) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        n = 0
        for p in _files(cfg_dir, CONFIG):
            z.write(p, "config/" + str(p.relative_to(cfg_dir)))
            n += 1
        for p in _files(data_dir, DATA, ("jobs/*/*/photos/**/*",) if photos else ()):
            z.write(p, "data/" + str(p.relative_to(data_dir)))
            n += 1
        z.writestr("manifest.json", json.dumps({"program": "pnp", "version": __version__, "files": n,
                                                "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                                                "photos": photos}, indent=1))
    return buf.getvalue()


def restore(blob: bytes, cfg_dir: Path, data_dir: Path) -> dict:
    """Unpack a bundle over this program's config and data. The files it replaces are kept in a zip first."""
    with zipfile.ZipFile(io.BytesIO(blob)) as z:
        names = z.namelist()
        if "manifest.json" not in names:
            raise ValueError("Not a pnp bundle (no manifest.json)")
        man = json.loads(z.read("manifest.json"))
        for n in names:
            parts = Path(n).parts
            if n == "manifest.json":
                continue
            if not parts or parts[0] not in ("config", "data") or ".." in parts or Path(n).is_absolute():
                raise ValueError(f"Refused: {n} is outside config/ and data/")
            if any(k in n for k in NEVER):
                raise ValueError(f"Refused: {n} must not be imported")
        hist = data_dir / "config_history"
        hist.mkdir(parents=True, exist_ok=True)
        before = hist / f"before_import_{time.strftime('%Y%m%d-%H%M%S')}.zip"
        before.write_bytes(export(cfg_dir, data_dir))
        count = 0
        for n in names:
            if n == "manifest.json" or n.endswith("/"):
                continue
            root = cfg_dir if n.startswith("config/") else data_dir
            target = root / n.split("/", 1)[1]
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(z.read(n))
            count += 1
    return {"files": count, "from_version": man.get("version"), "created": man.get("created"), "backup": str(before)}
