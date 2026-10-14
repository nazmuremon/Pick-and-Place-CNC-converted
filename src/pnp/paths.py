"""Where the program keeps its files.

- config dir: the profiles (`profiles/*.yaml`), `bed.yaml` and the other hand-kept files.
  `PNP_CONFIG_DIR`, else `config/` of this checkout, else `~/.config/pnp`.
- data dir: what the program writes by itself while it runs (state, events, config backups, jobs).
  `PNP_DATA_DIR`, else `var/` of this checkout, else `~/.local/share/pnp`.
"""
from __future__ import annotations

import os
from pathlib import Path

CHECKOUT = Path(__file__).resolve().parents[2]       # src/pnp/paths.py -> the repo, when run from a checkout


def _in_checkout() -> bool:
    return (CHECKOUT / "pyproject.toml").is_file() and (CHECKOUT / "config").is_dir()


def config_dir() -> Path:
    env = os.environ.get("PNP_CONFIG_DIR")
    if env:
        return Path(env)
    if _in_checkout():
        return CHECKOUT / "config"
    return Path.home() / ".config" / "pnp"


def data_dir() -> Path:
    env = os.environ.get("PNP_DATA_DIR")
    if env:
        p = Path(env)
    elif _in_checkout():
        p = CHECKOUT / "var"
    else:
        p = Path.home() / ".local" / "share" / "pnp"
    p.mkdir(parents=True, exist_ok=True)
    return p


def profiles_dir(cfg: Path | None = None) -> Path:
    return (cfg or config_dir()) / "profiles"
