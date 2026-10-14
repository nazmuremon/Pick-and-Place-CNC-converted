"""Phase G: feeders beyond strips (docs/13 §4.14 "Feeder types"): a matrix tray (a grid of pockets, e.g. a JEDEC
tray or a printed grid) and a stick (a tube that feeds one pick point). Both are a list entry ("Add tray") with their
geometry; the camera checks each fixed pocket against the confirmed first part."""
from __future__ import annotations

import numpy as np
import pytest

from pnp import sim
from pnp.feeders.bank import FeederBank, Tray
from simwizard import load, ready_machine, run


def test_a_matrix_and_a_stick_need_their_heights(tmp_path):
    b = FeederBank(tmp_path, {})
    with pytest.raises(ValueError):
        b.add(9, type_="matrix", rows=3, cols=5)                   # where the parts sit: measure it, never a guess
    t = b.add(9, type_="matrix", rows=3, cols=5, pitch_u=4.0, pitch_v=4.0, first_u=22.0, tape_floor_mm=2.0,
              top_mm=3.0)
    assert t.capacity(4.0) == 15 and t.marker_z == 3.0
    assert t.cells()[2] == (2, [22.0, 26.0, 30.0, 34.0, 38.0], 8.0)
    s = b.add(10, type_="stick", first_u=22.0, tape_floor_mm=1.0, top_mm=4.0)
    assert s.cells() == [(0, [22.0], 0.0)] and s.capacity(4.0) > 1000


@pytest.fixture(scope="module")
def fm(tmp_path_factory):
    m = ready_machine(tmp_path_factory.mktemp("pnp_kinds"))
    try:
        yield m
    finally:
        m.close()


def test_picks_from_a_matrix_tray_and_a_stick(fm):
    m = fm
    m.feeders.add(9, type_="matrix", rows=3, cols=5, pitch_u=4.0, pitch_v=4.0, first_u=22.0, plan_xy=(0.0, -22.0),
                  tape_floor_mm=2.0, top_mm=3.0)
    m.feeders.add(10, type_="stick", first_u=22.0, plan_xy=(0.0, -112.0), tape_floor_mm=1.0, top_mm=4.0)
    m.markers = m._markers()
    assert m.sim_event("tray", 9)["ok"] and m.sim_event("tray", 10)["ok"]
    run(m, m.setup_action("survey", {}))
    view = {t["number"]: t for t in m.jobs.trays_view()}
    assert view[9]["measured"] and view[10]["measured"], view
    assert view[9]["reach"] == "reachable" and view[10]["reach"] == "reachable"
    for t in list(m.feeder_state.d):
        m.feeder_state.unload(int(t))
    load(m, {9: "100nF|0805", 10: "100nF|0805"})
    run(m, m.jobs.start("pick_test", n_picks=24), 1800)
    st = m.jobs.status()
    assert st["status"] == "done", st["pause"]
    assert st["picks"]["picked"] == 24, st["picks"]
    used = m.feeder_state.get(9)["used"]
    assert len(used) == 12 and {u // 1000 for u in used} <= {0, 1, 2}     # rows of the matrix: row * 1000 + k
    assert not m.feeder_state.get(10)["used"]                             # a stick is never 'used up' by a pick
    assert sim.nozzle_of(m.sim_grbl).crashes == 0
