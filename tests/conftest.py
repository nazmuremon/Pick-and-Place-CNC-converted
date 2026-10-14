"""Test set-up: the tests never read or write the live settings in config/ or the files in logs/.

Rule (docs/13-build-prompt-v2.md, bug 12): the live config/*.yaml change every time the owner calibrates the
machine, so the tests use a FROZEN copy in tests/data/config/. This file runs before the test modules import the
tools: it copies the frozen settings to a fresh temporary folder and points the tools at it with PNP_CONFIG_DIR
(and at an empty temporary logs folder with PNP_LOGS_DIR, tools/pnp_paths.py). So even a tool that saves its
settings during a test writes only the temporary copy.

At the end of the session it checks that no live config/ file changed (hashes taken at the start).
"""
from __future__ import annotations

import atexit
import hashlib
import os
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FROZEN_CONFIG = Path(__file__).resolve().parent / "data" / "config"
LIVE_CONFIG = ROOT / "config"

_TMP = Path(tempfile.mkdtemp(prefix="pnp-tests-"))
atexit.register(shutil.rmtree, _TMP, ignore_errors=True)
TEST_CONFIG = _TMP / "config"
TEST_LOGS = _TMP / "logs"
shutil.copytree(FROZEN_CONFIG, TEST_CONFIG)
TEST_LOGS.mkdir()
os.environ["PNP_CONFIG_DIR"] = str(TEST_CONFIG)
os.environ["PNP_LOGS_DIR"] = str(TEST_LOGS)
sys.path.insert(0, str(ROOT / "tools"))

if "pnp_paths" in sys.modules or "bench" in sys.modules:   # imported before this file ran: too late to redirect
    raise RuntimeError("tests/conftest.py must run before the tools are imported (they would use the live config/)")


def _live_hashes() -> dict[str, str]:
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(LIVE_CONFIG.glob("*.yaml"))}


_LIVE_AT_START = _live_hashes()


def pytest_sessionfinish(session, exitstatus):
    """Fail the run if a test changed, added or removed a live config/ file."""
    now = _live_hashes()
    changed = sorted(n for n in set(now) | set(_LIVE_AT_START) if now.get(n) != _LIVE_AT_START.get(n))
    if changed:
        print(f"\nERROR: the tests changed the live settings in config/: {', '.join(changed)}", file=sys.stderr)
        session.exitstatus = 1
