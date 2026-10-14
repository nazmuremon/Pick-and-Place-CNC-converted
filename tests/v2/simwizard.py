"""A simulated machine taken through the set-up wizard (Phase D), for the job tests: calibrated, homed, Z set,
nozzle offset and angle table, bed survey, the HT5023 imported and registered B side up."""
from __future__ import annotations

import random

from pathlib import Path

from conftest import BOARDS
from pnp.machine import Machine
from test_v2_machine_vision import _cfg


def run(m: Machine, r: dict, t: float = 1200) -> None:
    assert r["ok"], r
    assert m.worker.wait(t), "timeout"
    assert m.safety.lock is None, [e["text"] for e in m.events.tail(8)]


def ready_machine(root: Path, change=None, speed: float = 80.0, **kw) -> Machine:
    m = Machine(_cfg(root, change), sim=True, data_dir=root / "data", watch=False, sim_speed=speed, **kw)
    run(m, m.connect(), 30)
    run(m, m.jog(dy=-40))
    run(m, m.jog(dx=-20))
    m.sim_event("charuco")
    run(m, m.calibrate("bed"))
    m.sim_event("charuco_block")
    run(m, m.calibrate("block", 5.0))
    m.sim_event("charuco_off")
    run(m, m.calibrate("solve"))
    run(m, m.jog(dx=20))
    run(m, m.jog(dy=40))
    run(m, m.home())
    run(m, m.jog(dz=-20.0))
    run(m, m.set_z(0.0))
    run(m, m.jog(dz=8.0))
    run(m, m.goto(-77.0, -60.0))
    m.sim_event("mark_sheet")
    run(m, m.setup_action("nozzle_marks", {"sheet_mm": 0.1}))
    run(m, m.goto(-77.0, -75.0))
    run(m, m.setup_action("rotation_marks", {"sheet_mm": 0.1}))
    run(m, m.setup_action("survey", {}))
    iid = m.boards.new_import()
    m.boards.add_file(iid, "ht5023.kicad_pcb.gz", (BOARDS / "ht5023.kicad_pcb.gz").read_bytes())
    m.boards.run(iid)
    m.boards.save(iid, "ht5023")
    random.seed(7)                                                   # a repeatable board pose
    m.sim_event("board")
    run(m, m.register_board("B", "back"))
    return m


def load(m: Machine, trays: dict) -> None:
    """Put a strip of each line into its tray and confirm the first part (Feeders tab)."""
    for t, line in trays.items():
        assert m.jobs.load_tray(int(t), line)["ok"]
        m.sim_event("strip_in", int(t))
        run(m, m.setup_action("look_at_slot", {"slot": int(t), "package": line.split("|")[1]}))
        assert m.setup_action("confirm", {"yes": True})["ok"], f"tray {t}: no candidate"
