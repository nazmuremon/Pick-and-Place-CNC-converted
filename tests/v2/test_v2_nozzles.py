"""Phase G: more than one nozzle (docs/13 §4.14 "Nozzle set", manual change): every package names its nozzle; the
job puts the parts of the nozzle on the head first and pauses for a change; each nozzle has its own offset."""
from __future__ import annotations

import pytest

from pnp.calibration import chain
from simwizard import load, ready_machine, run


def two_nozzles(raw):
    n = raw["nozzles"]["items"][0]
    n["packages"] = ["0603"]
    raw["nozzles"]["items"].append(dict(n, id="N18G", od_mm=0.84, packages=["0805"]))


@pytest.fixture(scope="module")
def nm(tmp_path_factory):
    m = ready_machine(tmp_path_factory.mktemp("pnp_nozzles"), two_nozzles)
    try:
        yield m
    finally:
        m.close()


def test_the_plan_names_each_parts_nozzle_and_starts_with_the_one_on(nm):
    plan = nm.jobs.plan()
    assert nm.nozzle_current() == "N22G"
    by = {p.package: p.nozzle for p in plan.parts}
    assert by.get("0603") == "N22G" and by.get("0805") == "N18G"
    for ld in plan.loads:
        noz = [p.nozzle for p in plan.parts if p.load == ld.index]
        assert noz == sorted(noz, key=lambda n: (n != "N22G", n))     # the one on the head first, one change
    st = {s["id"]: s for s in nm.wizard()}
    assert "N22G" in st["nozzle_offset"]["title"]


def test_a_job_pauses_for_the_nozzle_and_each_nozzle_has_its_offset(nm):
    m = nm
    plan = m.jobs.plan()
    p0603 = next(p for p in plan.parts if p.package == "0603")
    p0805 = next(p for p in plan.parts if p.package == "0805")
    load(m, {3: p0603.line, 4: p0805.line})
    run(m, m.setup_action("first_part", {"slot": 3, "ref": p0603.ref}))
    m.setup_action("first_part_answer", {"ok": True})
    run(m, m.jobs.start("real", refs=[p0805.ref]), 600)
    st = m.jobs.status()
    assert st["status"] == "paused" and st["pause"]["kind"] == "nozzle" and st["pause"]["nozzle"] == "N18G"
    assert m.jobs.nozzle_changed("N18G")["ok"]
    assert m.nozzle_current() == "N18G"
    assert chain.status(m, "nozzle_offset")["state"] == "missing"            # its own offset: not measured yet
    r = m.jobs.resume()
    assert not r["ok"] and "offset" in r["why"]
    run(m, m.goto(-77.0, -60.0))                                  # the sheet between two trays
    m.sim_event("mark_sheet")
    run(m, m.setup_action("nozzle_marks", {"sheet_mm": 0.1}))
    assert chain.status(m, "nozzle_offset")["state"] == "valid", [e["text"][:90] for e in m.events.tail(4)]
    assert m.calib.get("nozzle_offset@N22G") is not None and m.calib.get("nozzle_offset@N18G") is not None
    run(m, m.jobs.resume(), 900)
    assert m.jobs.status()["status"] == "done" and m.jobs.job.parts[0].state == "verified", m.jobs.status()["pause"]
    assert m.set_nozzle("N22G")["ok"] and chain.status(m, "nozzle_offset")["state"] == "valid"   # kept for each
