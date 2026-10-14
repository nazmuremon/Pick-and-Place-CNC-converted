"""Fixtures for the v2 tests (src/pnp/). Frozen inputs only: tests/data/pnp/bench_uno.yaml (profile) and
tests/data/config/bed.yaml (bed). Every test gets its own config and data folders in tmp_path."""
from __future__ import annotations

import shutil
from pathlib import Path
from typing import Callable

import pytest
import yaml

from pnp.machine import Machine

DATA = Path(__file__).resolve().parents[1] / "data"
PROFILE = DATA / "pnp" / "bench_uno.yaml"
BED = DATA / "config" / "bed.yaml"
PACKAGES = DATA / "pnp"                      # packages.yaml + packages.user.yaml (frozen)
BOARDS = DATA / "boards" / "ht5023"
SPEED = 50.0                       # the simulator runs 50 times faster than real


def raw_profile() -> dict:
    return yaml.safe_load(PROFILE.read_text())


@pytest.fixture(autouse=True)
def data_dir(tmp_path, monkeypatch) -> Path:
    d = tmp_path / "data"
    d.mkdir()
    monkeypatch.setenv("PNP_DATA_DIR", str(d))
    return d


@pytest.fixture
def make_cfg(tmp_path) -> Callable[..., Path]:
    """A config folder with the frozen bed and a (changed) bench profile. Returns the profile's path."""
    def make(change: Callable[[dict], None] | None = None, name: str = "bench_uno") -> Path:
        cfg = tmp_path / "config"
        (cfg / "profiles").mkdir(parents=True, exist_ok=True)
        shutil.copy(BED, cfg / "bed.yaml")
        for f in ("packages.yaml", "packages.user.yaml"):
            shutil.copy(PACKAGES / f, cfg / f)
        raw = raw_profile()
        raw["sim"]["speed"] = SPEED
        if change is not None:
            change(raw)
        p = cfg / "profiles" / f"{name}.yaml"
        p.write_text(yaml.safe_dump(raw, sort_keys=False))
        return p
    return make


@pytest.fixture
def machine(make_cfg, data_dir):
    """start(change=None, connect=True, **kw) -> a Machine on the simulator. Closed after the test."""
    made: list[Machine] = []

    def start(change=None, connect: bool = True, **kw) -> Machine:
        path = kw.pop("path", None) or make_cfg(change)
        m = Machine(path, sim=True, data_dir=data_dir, watch=False, **kw)
        made.append(m)
        if connect:
            assert m.connect()["ok"]
            assert m.worker.wait(20)
            assert m.safety.connected, [e["text"] for e in m.events.tail(10)]
        return m

    yield start
    for m in made:
        m.close()


def manual_homing(raw: dict) -> None:
    raw["homing"]["method"] = "manual"


def wait_until(fn, timeout: float = 5.0) -> bool:
    import time
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if fn():
            return True
        time.sleep(0.02)
    return False
