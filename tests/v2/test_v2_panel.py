"""Phase G: a panel — several boards of the same kind on the bed in one job (docs/13 §6.11). Each copy is registered
on its own; the plan takes the parts of every copy; each copy has its own placed state. The bench is too small for two
HT5023s, so this simulated machine has a wider frame."""
from __future__ import annotations

import numpy as np
import pytest

from pnp import sim
from simwizard import load, ready_machine, run


def wide(raw):
    raw["motion"]["travel"]["x"] = [-330.0, 10.0]
    raw["sim"]["hard_ends"]["X"] = [-340.0, 15.0]


@pytest.fixture(scope="module")
def pm(tmp_path_factory):
    m = ready_machine(tmp_path_factory.mktemp("pnp_panel"), wide)
    try:
        yield m
    finally:
        m.close()


def test_two_boards_in_one_job(pm):
    m = pm
    one = len(m.jobs.plan().parts)
    m.vision.sim_prop("board", side="B", xy=(-300.0, -242.0), copy="2")
    board = m.boards.current()
    run(m, m.register_board("B", "back", "2", -300.0 + board.width / 2, -242.0 + board.height / 2))
    assert [c for c, _ in m.vision.registrations()] == ["1", "2"]
    plan = m.jobs.plan()
    assert len(plan.parts) == 2 * one                                # the same parts, twice
    ld = plan.loads[0]
    load(m, ld.trays)
    first = next(p for p in plan.parts if p.load == 1)
    tray = next(t for t, ln in ld.trays.items() if ln == first.line)
    run(m, m.setup_action("first_part", {"slot": tray, "ref": first.ref}))
    m.setup_action("first_part_answer", {"ok": True})
    refs = [p.ref for p in plan.parts if p.load == 1 and p.copy == "1" and p.ref != first.ref][:3]
    run(m, m.jobs.start("real", refs=refs), 1800)
    st = m.jobs.status()
    assert st["status"] == "done", st["pause"]
    parts = m.jobs.job.parts
    assert sorted((p.ref, p.copy) for p in parts) == sorted([(r, c) for r in refs for c in ("1", "2")])
    noz = sim.nozzle_of(m.sim_grbl)
    for p in parts:
        pose = m.vision.sim_board_poses[p.copy]
        c = board.component(p.ref)
        t = pose.to_machine(board, (c.x, c.y))
        q = min(noz.placed, key=lambda q: np.hypot(q.x - t[0], q.y - t[1]))
        assert np.hypot(q.x - t[0], q.y - t[1]) < 0.15, (p.ref, p.copy)
    assert set(refs) <= set(m.placed.get("ht5023")) and set(refs) <= set(m.placed.get("ht5023#2"))
    again = m.jobs.plan()
    assert not any(p.ref in refs for p in again.parts)              # placed on both copies: not planned again
