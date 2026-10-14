"""What survives a program restart (D22): the session token, HOME and the Z reference in the controller frame,
and an open lock. Valid only while the controller keeps the same token (not reset since)."""
from __future__ import annotations

import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml


@dataclass
class Session:
    profile: str = ""
    token: int | None = None
    home_raw: dict[str, float] | None = None       # controller-frame XY of the travel origin
    z_ref_raw: float | None = None                 # controller-frame Z of the Z reference surface
    z_surface_note: str = ""
    home_view: list[float] | None = None           # where the head was when HOME was measured (the drift check)
    lock: dict | None = None                       # {code, detail, since}
    saved: str = ""
    extra: dict = field(default_factory=dict)


class SessionStore:
    def __init__(self, path: Path) -> None:
        self.path = path

    def load(self) -> Session | None:
        try:
            raw = yaml.safe_load(self.path.read_text()) or {}
            return Session(**{k: v for k, v in raw.items() if k in Session.__dataclass_fields__})
        except FileNotFoundError:
            return None
        except (OSError, yaml.YAMLError, TypeError):
            return None

    def save(self, s: Session) -> None:
        s.saved = time.strftime("%Y-%m-%dT%H:%M:%S")
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(yaml.safe_dump(asdict(s), sort_keys=False))
        os.replace(tmp, self.path)                 # never a half-written file
