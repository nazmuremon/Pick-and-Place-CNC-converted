"""Where the bench tools and the web page find their settings (config/) and write their logs (logs/).

One place for both, so the tests can point them somewhere else:
  PNP_CONFIG_DIR = the settings folder (default: config/ in the project)
  PNP_LOGS_DIR   = the logs folder     (default: logs/ in the project)
The tests (tests/conftest.py) set both to temporary folders with a frozen copy of the settings, so a test
never reads or writes the live files that the calibration tools change.
No other imports here: every tool can use it (smd_identify and make_labels do not need the serial port).
"""
from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def config_dir() -> Path:
    """The settings folder: $PNP_CONFIG_DIR, else config/ in the project."""
    return Path(os.environ.get("PNP_CONFIG_DIR") or ROOT / "config")


def logs_dir() -> Path:
    """The logs folder: $PNP_LOGS_DIR, else logs/ in the project."""
    return Path(os.environ.get("PNP_LOGS_DIR") or ROOT / "logs")


# Read once, when the tools are imported (the tests set the variables before that).
CONFIG_DIR = config_dir()
LOGS_DIR = logs_dir()
