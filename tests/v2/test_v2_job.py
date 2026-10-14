"""Phase E: feeders as a list (Add tray, markers, the tray generator), the load split, the job planner and runner
(pick / place checks, reports, STOP with a part on the nozzle, resume after a crash, refills) and the pick test
(docs/13 §4.4.1, §4.7, §8). The machine part runs in the simulator after the Phase D wizard (simwizard.py)."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from conftest import BED
from pnp import sim
from pnp.feeders.bank import FeederBank, Tray
from pnp.feeders.loads import Line, split
from pnp.machine import Machine
from pnp.vision.markers import Markers
from simwizard import load, ready_machine, run


# ================================================================ the feeder bank
@pytest.fixture
def bed():
    return yaml.safe_load(BED.read_text())


def test_the_trays_come_from_bed_yaml_the_first_time(tmp_path, bed):
    b = FeederBank(tmp_path, bed)
    assert sorted(b.trays) == [3, 4, 5, 6, 7] and b.migrated
    assert [b.trays[n].marker_id for n in (3, 4, 5, 6, 7)] == [42, 43, 44, 45, 46]
    assert b.trays[3].plan_xy == [-29.2, -1.5] and b.trays[3].type == "paper_label"
    b.save()
    b2 = FeederBank(tmp_path, bed)                                 # from feeders.yaml now
    assert not b2.migrated and sorted(b2.trays) == [3, 4, 5, 6, 7]


def test_new_trays_take_free_markers_and_switch_the_dictionary_past_50(tmp_path, bed):
    b = FeederBank(tmp_path, bed)
    assert b.add(8, 8, "paper").marker_id == 47                    # like the paper labels: 40 + n - 1 when free
    assert b.add(1, 12, "embossed").marker_id == 40
    t = b.add(12, 8, "paper")
    assert t.marker_id not in (49, 30, 31, 32, 33, 20, 21, 22, 23) and t.marker_id not in (42, 43, 44, 45, 46)
    n = 13
    while b.dictionary == "DICT_4X4_50":
        b.add(n)
        n += 1
    assert b.dictionary == "DICT_4X4_250" and max(t.marker_id for t in b.trays.values()) >= 50
    ids = [t.marker_id for t in b.trays.values()]
    assert len(set(ids)) == len(ids)
    with pytest.raises(ValueError):
        b.add(12)                                                  # tray 12 exists
    with pytest.raises(ValueError):
        b.add(100)
    # every label printed so far still reads with the bigger dictionary
    img = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                                           42, 120), 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=255)
    seen = Markers(bed, b.names(), b.dictionary).find(img)
    assert list(seen) == ["slot_3"]                                # marker 42 = tray 3


def test_a_marker_used_twice_is_an_error(bed):
    with pytest.raises(ValueError):
        Markers(bed, {49: "slot_9"})


def test_the_load_split_follows_the_tray_count():
    lines = [Line(f"v{i}|0603", f"v{i}", "0603", [f"R{i}_{k}" for k in range(3 + i % 5)]) for i in range(43)]
    lines[0].refs += [f"X{k}" for k in range(20)]                 # a long line: a spare tray when one is free
    for n in (6, 12, 24):
        trays = [Tray(k, 40 + k) for k in range(1, n + 1)]
        loads = split(lines, trays, lambda t, ln: t.capacity(ln.pitch))
        assert len(loads) == math.ceil(43 / n), (n, len(loads))
        assert all(len(ld.trays) <= n for ld in loads)
        assert sorted(k for ld in loads for k in ld.lines()) == sorted(ln.key for ln in lines)
    trays = [Tray(k, 40 + k) for k in range(1, 25)]
    few = lines[:3]
    ld = split(few, trays, lambda t, ln: t.capacity(ln.pitch))[0]
    assert list(ld.trays.values()).count(few[0].key) == 2          # 23 parts, 15 per strip: two trays


# ================================================================ the tray generator (docs/13 §4.4.1)
tg = pytest.importorskip("pnp.feeders.traygen")


def _top_marker(m, p):
    """Render the holder's top face (the recessed cells are black) and read its marker."""
    z = p.top - p.recess / 2
    res = 40.0
    w, h = p.quiet * 2 + p.marker, p.marker + 2 * p.quiet
    sol = np.zeros((int(h * res), int(w * res)), np.uint8)
    cnts = [np.array([[x * res, (h / 2 - y) * res] for x, y in poly], np.int32) for poly in m.slice(z).to_polygons()]
    cv2.fillPoly(sol, cnts, 255)                                   # the outline and the cells: cells come out black
    big = cv2.copyMakeBorder(sol, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    d = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250))
    _, ids, _ = d.detectMarkers(big)
    return None if ids is None else int(ids.ravel()[0])


@pytest.mark.parametrize("number,width,kind,marker", [(1, 8, "paper", 40), (9, 8, "embossed", 0), (10, 12, "embossed", 5),
                                                     (99, 16, "embossed", 48), (23, 24, "embossed", 120)])
def test_holders_for_any_number_and_tape_width(number, width, kind, marker, tmp_path):
    from dataclasses import replace
    import manifold3d as mf
    p = replace(tg.P(), tape_w=float(width), kind=kind)
    dic = "DICT_4X4_250" if marker >= 50 else "DICT_4X4_50"
    m = tg.holder(p, number, marker, dic)
    assert m.status() == mf.Error.NoError and len(m.decompose()) == 1         # one closed (watertight) body
    assert _top_marker(m, p) == marker
    out = tg.make_holder_stl(number, marker, width, kind, tmp_path / "t.stl", dic)
    assert out.stat().st_size > 10000
    with pytest.raises(ValueError):
        tg.holder(p, 100, marker, dic)


# ================================================================ the job in the simulator
def _true_err(m, ref):
    board, _ = m.vision.registration()
    pose = m.vision.sim_board_pose
    c = board.component(ref)
    t = pose.to_machine(board, (c.x, c.y))
    noz = sim.nozzle_of(m.sim_grbl)
    pl = [q for q in noz.placed if np.hypot(q.x - t[0], q.y - t[1]) < 0.6]    # 0603 neighbours are ~1 mm away
    if len(pl) != 1:
        return None, len(pl)
    q = pl[0]
    return float(np.hypot(q.x - t[0], q.y - t[1])), (q.angle - (pose.theta + c.rotation) + 90) % 180 - 90


@pytest.fixture(scope="module")
def jm(tmp_path_factory):
    root = tmp_path_factory.mktemp("pnp_job")
    m = ready_machine(root)
    try:
        yield m
    finally:
        m.close()


def test_a_7th_tray_is_added_from_the_page(jm):
    """docs/13 §11 Phase E: "a 7th tray added from the page" — here tray 8 next to HOME (5 labels + 1 = the bench's
    6th, plus the first part's: the count does not matter, nothing assumes it)."""
    m = jm
    before = {t["number"]: t["reach"] for t in m.jobs.trays_view()}
    r = m.jobs.add_tray(8, 8, "paper", 60.0, "holder", 0.0, -22.0)
    assert r["ok"] and r["marker"] == 47 and Path(r["files"]["stl"]).is_file()
    assert m.sim_event("tray", 8)["ok"]
    run(m, m.setup_action("survey", {}))
    view = {t["number"]: t for t in m.jobs.trays_view()}
    assert view[8]["measured"] and view[8]["reach"] == "reachable"
    reach = [n for n, t in view.items() if t["reach"] == "reachable"]
    assert sorted(reach) == sorted([n for n, r in before.items() if r == "reachable"] + [8])
    pv = m.jobs.plan_view()
    assert len(pv["loads"][0]["trays"]) == len(reach)               # the load split uses it
    assert yaml.safe_load((m.cfg_dir / "feeders.yaml").read_text())["trays"][-1]["number"] == 8


def test_the_plan_says_why_parts_are_not_placed(jm):
    pv = jm.jobs.plan_view()
    s = pv["summary"]
    assert s["parts"] > 50 and s["loads"] >= 5
    reasons = " ".join(s["skip_reasons"])
    assert "bottom camera" in reasons and "by hand" in reasons and "polar" in reasons


def test_a_20_part_job_over_two_loads(jm):
    m = jm
    plan = m.jobs.plan()
    loads = {ld.index: ld for ld in plan.loads}
    refs = [p.ref for p in plan.parts if p.load == 1][:12] + [p.ref for p in plan.parts if p.load == 2][:8]
    assert len(refs) == 20
    load(m, loads[1].trays)
    first = next(p for p in plan.parts if p.load == 1 and p.ref not in refs)
    tray = next(t for t, ln in loads[1].trays.items() if ln == first.line)
    run(m, m.setup_action("first_part", {"slot": tray, "ref": first.ref}))
    m.setup_action("first_part_answer", {"ok": True})
    t0 = time.monotonic()
    run(m, m.jobs.start("real", refs=refs), 1800)
    st = m.jobs.status()
    assert st["status"] == "paused" and st["pause"]["kind"] == "load", st["pause"]
    assert "Load 2" in st["pause"]["why"]
    for t in loads[2].trays:
        m.feeder_state.unload(t)
    load(m, loads[2].trays)
    run(m, m.jobs.resume(), 1800)
    st = m.jobs.status()
    assert st["status"] == "done", (st["status"], st["pause"])
    noz = sim.nozzle_of(m.sim_grbl)
    assert noz.crashes == 0, noz.t.log
    good = 0
    for p in m.jobs.job.parts:
        e, a = _true_err(m, p.ref)
        assert e is not None, (p.ref, a)                            # exactly one part on its place
        good += e < 0.15 and abs(a) < 3.0
    assert good >= 19, good                                         # docs/13 §8: 95 % within 0.15 mm and 3°
    assert st["counts"].get("verified", 0) >= 18, st["counts"]      # the camera's own place check agrees
    rep = Path(st["report"])
    assert rep.is_file() and "photos/" in rep.read_text()
    assert set(refs) <= set(m.placed.get("ht5023"))                 # the shared placed state (§4.15)
    assert m.jobs.job.home_checks and all(h.get("mm", 0) < 0.1 for h in m.jobs.job.home_checks)
    assert time.monotonic() - t0 < 600


def test_stop_with_a_part_on_the_nozzle_then_place_it(jm):
    m = jm
    plan = m.jobs.plan()
    ld = plan.loads[0]
    for t in list(m.feeder_state.d):
        m.feeder_state.unload(int(t))
    load(m, ld.trays)
    refs = [p.ref for p in plan.parts if p.load == ld.index][:3]
    assert len(refs) == 3
    assert m.jobs.start("real", refs=refs)["ok"]
    noz = sim.nozzle_of(m.sim_grbl)
    deadline = time.monotonic() + 300
    while time.monotonic() < deadline and not (noz.held is not None and m.position()["Z"] > 4.0):
        time.sleep(0.005)                                           # a part on the nozzle, on its way to the board
    assert noz.held is not None
    m.stop()
    assert m.worker.wait(120)
    st = m.jobs.status()
    assert st["status"] == "paused" and st["pause"]["kind"] == "held", st["pause"]
    assert m.caps.vacuum.pump_on                                    # the part stays on the nozzle
    assert m.position()["Z"] >= m.jobs.runner.pp.moving - 0.01      # the nozzle lifted
    assert not m.jobs.resume()["ok"]                                # first the choice for the held part
    assert not m.jobs.abort()["ok"]
    run(m, m.jobs.answer("place"))
    held = st["pause"]["ref"]
    assert next(p for p in m.jobs.job.parts if p.ref == held).state in ("verified", "failed")
    run(m, m.jobs.resume(), 900)
    assert m.jobs.status()["status"] == "done"
    for r in refs:
        e, _ = _true_err(m, r)
        assert e is not None and e < 0.2, (r, e)


def test_a_job_resumes_after_the_program_crashed(jm, tmp_path):
    """The state file says 'running' with a part on the way (what a crash leaves): a new program picks the job up,
    paused, and asks about the nozzle; 'pick again' then finishes it."""
    m = jm
    plan = m.jobs.plan()
    ld = plan.loads[0]
    for t in list(m.feeder_state.d):
        m.feeder_state.unload(int(t))
    load(m, ld.trays)
    refs = [p.ref for p in plan.parts if p.load == ld.index][:3]
    assert len(refs) == 3
    assert m.jobs.start("real", refs=refs)["ok"]
    while not any(p.state in ("verified", "failed") for p in m.jobs.job.parts):
        time.sleep(0.05)
    m.stop()
    m.worker.wait(120)
    m.jobs.answer("place") if m.jobs.job.pause.get("kind") == "held" else None
    m.worker.wait(120)
    j = m.jobs.job
    d = json.loads(j.path.read_text())
    nxt = next(p for p in d["parts"] if p["state"] == "planned")
    nxt["state"] = "picking"
    d["status"] = "running"
    j.path.write_text(json.dumps(d))
    from pnp.machine_job import JobOps
    ops = JobOps(m)                                                 # what a restarted program does first
    assert ops.job.id == j.id and ops.job.status == "paused" and ops.job.pause["kind"] == "held"
    assert ops.job.pause["ref"] == nxt["ref"]
    m.jobs = ops
    run(m, m.jobs.answer("repick"))
    run(m, m.jobs.resume(), 900)
    assert m.jobs.status()["status"] == "done"
    assert all(p.state in ("verified", "failed") for p in m.jobs.job.parts)


def test_the_pick_test_100_picks_with_a_refill(jm):
    """docs/13 §8: pick success ≥ 95 % over 100 picks, every miss with its photo. The trays run empty on the way:
    the job stops for a refill and goes on."""
    m = jm
    for t in list(m.feeder_state.d):
        m.feeder_state.unload(int(t))
    trays = {t.number: "100nF|0805" for t in m.jobs.reachable_trays()}
    assert len(trays) >= 3
    load(m, trays)
    run(m, m.jobs.start("pick_test", n_picks=100), 1800)
    refills = 0
    while m.jobs.status()["status"] == "paused":
        st = m.jobs.status()
        assert st["pause"]["kind"] == "refill", st["pause"]
        for t in trays:
            m.sim_event("strip_in", t)
            m.jobs.refilled(t)
        refills += 1
        run(m, m.jobs.resume(), 1800)
        assert refills < 4
    st = m.jobs.status()
    assert st["status"] == "done" and refills >= 1
    assert st["picks"]["done"] == 100 and st["picks"]["rate"] >= 0.95, st["picks"]
    rep = Path(st["report"]).read_text()
    assert "Pick test" in rep
    for p in m.jobs.job.picks:
        if p["verdict"] != "picked":
            assert all(Path(v).is_file() for v in p["photos"].values())
