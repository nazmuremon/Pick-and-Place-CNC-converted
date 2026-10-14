"""Guard (docs/13-build-prompt-v2.md, bug 12): no test reads the live config/ or logs/. They change every time the
owner calibrates the machine; the tests use the frozen copy in tests/data/config/ (tests/conftest.py)."""
import os
import re
import sys
from pathlib import Path

import bench
import camera_rectify as cr
import make_labels as ml
import nozzle_calibrate as nc
import smd_identify as si
from pnp_paths import CONFIG_DIR, LOGS_DIR

TESTS = Path(__file__).resolve().parent
ROOT = TESTS.parent
sys.path.insert(0, str(ROOT))
from ui import engine as e  # noqa: E402

LIVE = re.compile(r"""ROOT\s*/\s*["'](config|logs)["']|["'](config|logs)/[\w.]+\.yaml["']\s*\)""")


def test_no_test_module_builds_a_path_to_the_live_config():
    bad = []
    for f in sorted(TESTS.glob("test_*.py")):
        if f.name == Path(__file__).name:
            continue
        for n, line in enumerate(f.read_text().splitlines(), 1):
            if LIVE.search(line):
                bad.append(f"{f.name}:{n}: {line.strip()}")
    assert not bad, "use pnp_paths.CONFIG_DIR (the frozen copy), not the live config/ or logs/:\n" + "\n".join(bad)


def test_the_tools_use_the_temporary_copy():
    assert Path(os.environ["PNP_CONFIG_DIR"]) == CONFIG_DIR and Path(os.environ["PNP_LOGS_DIR"]) == LOGS_DIR
    for p in (CONFIG_DIR, LOGS_DIR):
        assert not p.resolve().is_relative_to(ROOT), p
    for p in (bench.BENCH_CONFIG, ml.BED_CONFIG, nc.CAL_FILE, cr.RECTIFY_FILE, si.DEFAULT_CONFIG,
              e.FEEDER_FILE, e.FEEDER_EXAMPLE, e.REG_FILE, e.BED_FILE, e.BENCH_FILE):
        assert p.parent == CONFIG_DIR, p
    for p in (bench.STATE_FILE, nc.LOG_DIR, cr.LOG_DIR, e.JOB_FILE, e.t.POSITION_FILE, e.t.CAMERA_FILE):
        assert p.is_relative_to(LOGS_DIR), p


def test_the_frozen_copies_say_so():
    for f in sorted((TESTS / "data" / "config").glob("*.yaml")):
        assert f.read_text().startswith("# FROZEN TEST COPY"), f.name
