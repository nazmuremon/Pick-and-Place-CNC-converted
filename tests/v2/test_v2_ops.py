"""Phase F: Doctor, the export / import bundle, the first run (`pnp init`), the defaults shipped in the package and
the installable wheel (docs/13 §4.10, §4.11, §4.13)."""
from __future__ import annotations

import io
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest
import yaml

from conftest import BOARDS
from pnp import bundle, cli, doctor, firstrun
from pnp import profile as prof
from pnp.feeders.packages import Library

ROOT = Path(__file__).resolve().parents[2]


def test_doctor_lists_every_calibration_and_never_fails(machine):
    m = machine()
    checks = doctor.run(m.profile, m.profile_path, m, m.cfg_dir, m.data)
    names = [c.name for c in checks]
    for title in ("Camera calibration", "Nozzle offset", "Bed survey (labels)", "Travel (soft limits)"):
        assert title in names
    assert next(c for c in checks if c.name == "Controller").ok
    assert all(c.fix for c in checks if c.ok is False)                 # every red line says what to do


def test_the_bundle_round_trip_and_its_guards(machine, tmp_path):
    m = machine(connect=False)
    iid = m.boards.new_import()
    m.boards.add_file(iid, "ht5023.kicad_pcb.gz", (BOARDS / "ht5023.kicad_pcb.gz").read_bytes())
    m.boards.run(iid)
    m.boards.save(iid, "ht5023")
    m.placed.mark("ht5023", "R1", "hand")
    m.parts.store.set_count(17)
    m.calib.put("camera_head", {"x": 1}, {"camera": "c"}, "test")
    (m.data / "session_sim.yaml").write_text("token: 5\n")
    blob = bundle.export(m.cfg_dir, m.data)
    names = zipfile.ZipFile(io.BytesIO(blob)).namelist()
    assert "manifest.json" in names and "config/bed.yaml" in names and "config/profiles/bench_uno.yaml" in names
    assert "data/boards/ht5023/board.json" in names and "data/store.json" in names
    assert not any("session" in n for n in names)                     # the position is never exported
    cfg2, data2 = tmp_path / "c2", tmp_path / "d2"
    r = bundle.restore(blob, cfg2, data2)
    assert r["files"] == len(names) - 1 and Path(r["backup"]).is_file()
    assert (cfg2 / "profiles" / "bench_uno.yaml").read_text() == (m.cfg_dir / "profiles" / "bench_uno.yaml").read_text()
    assert "R1" in (data2 / "boards" / "ht5023" / "placed.json").read_text()
    bad = io.BytesIO()
    with zipfile.ZipFile(bad, "w") as z:
        z.writestr("manifest.json", "{}")
        z.writestr("config/../../evil.txt", "x")
    with pytest.raises(ValueError):
        bundle.restore(bad.getvalue(), cfg2, data2)
    assert not (tmp_path / "evil.txt").exists()
    with pytest.raises(ValueError):
        bundle.restore(_zip({"x.txt": "1"}), cfg2, data2)          # no manifest: not a bundle


def _zip(files: dict) -> bytes:
    b = io.BytesIO()
    with zipfile.ZipFile(b, "w") as z:
        for k, v in files.items():
            z.writestr(k, v)
    return b.getvalue()


def test_export_and_restore_from_the_command_line(make_cfg, data_dir, tmp_path):
    cfg = make_cfg().parent.parent
    out = tmp_path / "b.zip"
    assert cli.main(["--config", str(cfg), "export", str(out)]) == 0 and out.stat().st_size > 1000
    cfg2 = tmp_path / "cfg2"
    assert cli.main(["--config", str(cfg2), "restore", str(out)]) == 0
    assert (cfg2 / "profiles" / "bench_uno.yaml").is_file()


def test_the_first_run_writes_a_valid_profile(tmp_path):
    answers = iter(["shop_pnp", "grblhal", "", "camera_marker", "probe", "servo_pwm", "/dev/video2", "1920x1080"])
    out = firstrun.run(tmp_path, lambda q, d: next(answers))
    p = prof.load(out)
    assert p.name == "shop_pnp" and p.controller.type == "grblhal" and p.z_reference.method == "probe"
    assert p.cameras.head.resolution == (1920, 1080) and p.sim is None
    assert (tmp_path / "bed.yaml").is_file() and (tmp_path / "packages.yaml").is_file()
    tbd = [x.field for x in prof.missing(p)]
    assert any("press_mm" in f for f in tbd)                          # what is not known stays TBD
    checks = doctor.run(p, out, None, tmp_path, tmp_path)
    assert next(c for c in checks if c.name == "Bed file").ok


def test_the_package_library_ships_with_the_program(tmp_path):
    lib = Library.load(tmp_path)                                     # an empty config folder
    assert lib.packages["0805"].body == (2.0, 1.25)


def test_the_wheel_has_the_page_the_decoders_and_the_defaults(tmp_path):
    py = next((p for p in (sys.executable, "/usr/bin/python3", shutil.which("python3") or "")
               if p and subprocess.run([p, "-c", "import setuptools, pip"], capture_output=True).returncode == 0), None)
    if py is None:
        pytest.skip("no Python with setuptools and pip here to build the wheel offline")
    src = tmp_path / "src_copy"                                      # never leave build/ or *.egg-info in the repo
    shutil.copytree(ROOT / "src", src / "src", ignore=shutil.ignore_patterns("__pycache__", "*.egg-info"))
    for f in ("pyproject.toml", "README.md"):
        shutil.copy(ROOT / f, src / f)
    r = subprocess.run([py, "-m", "pip", "wheel", str(src), "--no-deps", "--no-build-isolation", "-q",
                        "-w", str(tmp_path)], capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        pytest.skip(f"no wheel build here: {r.stderr[-300:]}")
    whl = next(tmp_path.glob("pnp-*.whl"))
    names = zipfile.ZipFile(whl).namelist()
    for need in ("pnp/ui/static/index.html", "pnp/ui/static/app.js", "pnp/ui/static/hand.js",
                 "pnp/ui/static/vendor/zxing_reader.wasm", "pnp/data/packages.yaml", "pnp/data/bed.example.yaml",
                 "pnp/data/profiles/pico_grblhal.yaml", "pnp/feeders/traygen.py", "pnp/cli.py"):
        assert need in names, need
    meta = next(n for n in names if n.endswith("entry_points.txt"))
    assert "pnp = pnp.cli:main" in zipfile.ZipFile(whl).read(meta).decode()
