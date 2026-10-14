"""Phase C on the whole machine (simulator): camera calibration from the ChArUco sheet, camera homing, board
registration, the overlay, the 20-part dry run (docs/13 §11 Phase C "sim 20-part dry run passes") and the stall
lock. One machine is set up once for the module (about 20 s); the tests run in this file's order."""
from __future__ import annotations

import random

import shutil
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from conftest import BED, BOARDS, PACKAGES, SPEED, raw_profile
from pnp import sim
from pnp.job import dryrun
from pnp.machine import Machine
from pnp.machine_vision import MachineRig
from pnp.vision import parts as vp


def _cfg(root: Path, change=None) -> Path:
    cfg = root / "config"
    (cfg / "profiles").mkdir(parents=True)
    shutil.copy(BED, cfg / "bed.yaml")
    for f in ("packages.yaml", "packages.user.yaml"):
        shutil.copy(PACKAGES / f, cfg / f)
    raw = raw_profile()
    raw["sim"]["speed"] = SPEED
    if change:
        change(raw)
    p = cfg / "profiles" / "bench_uno.yaml"
    p.write_text(yaml.safe_dump(raw, sort_keys=False))
    return p


def _run(m: Machine, r: dict, t: float = 300) -> None:
    assert r["ok"], r
    assert m.worker.wait(t), "timeout"
    assert m.safety.lock is None, [e["text"] for e in m.events.tail(8)]


def _texts(m: Machine, n: int = 40) -> list[str]:
    return [e["text"] for e in m.events.tail(n)]


@pytest.fixture(scope="module")
def ready(tmp_path_factory):
    """Connected, calibrated, homed, the HT5023 imported and registered B side up."""
    root = tmp_path_factory.mktemp("pnp_vision")
    m = Machine(_cfg(root), sim=True, data_dir=root / "data", watch=False)
    try:
        _run(m, m.connect(), 20)
        _run(m, m.jog(dy=-40))
        _run(m, m.jog(dx=-20))
        r = m.home()
        assert not r["ok"] and "calibrat" in (r.get("why", "") + r.get("todo", "")).lower(), r
        m.sim_event("charuco")
        _run(m, m.calibrate("bed"))
        m.sim_event("charuco_block")
        _run(m, m.calibrate("block", 5.0))
        m.sim_event("charuco_off")
        _run(m, m.calibrate("solve"))
        _run(m, m.jog(dx=20))
        _run(m, m.jog(dy=40))
        _run(m, m.home())
        iid = m.boards.new_import()
        m.boards.add_file(iid, "ht5023.kicad_pcb.gz", (BOARDS / "ht5023.kicad_pcb.gz").read_bytes())
        m.boards.run(iid)
        m.boards.save(iid, "ht5023")
        random.seed(7)                                                   # a repeatable board pose
        m.sim_event("board")
        _run(m, m.register_board("B", "back"))
        yield m
    finally:
        m.close()


def test_the_calibration_matches_the_simulated_camera(ready):
    m = ready
    model, true = m.vision.model(), m.cameras["camera_head"].model
    assert abs(model.tilt_deg - true.tilt_deg) < 0.1 and abs(model.height_mm - true.height_mm) < 0.5
    st = m.status()["vision"]
    assert st["model"]["rms_px"] < 0.3
    assert max(abs(v - 1.0) for v in model.quality["move_scale"]) < 0.001       # the head moves as counted
    # remembered with its inputs (camera hardware id), with how it was made and how good it is
    v, why = m.calib.value("camera_head", {"camera": m.profile.cameras.head.hardware_id})
    assert v is not None, why


def test_camera_homing_finds_home_to_a_few_hundredths(ready):
    m = ready
    pos, phys = m.position(), m.sim_grbl.phys                # the simulator's true head, in mm from HOME
    assert abs(pos["X"] - phys["X"]) < 0.05 and abs(pos["Y"] - phys["Y"]) < 0.05


def test_registration_puts_every_part_within_a_tenth(ready):
    m = ready
    board, reg = m.vision.registration()
    pose = m.vision.sim_board_pose
    assert reg.kind == "rigid" and reg.rms < 0.1 and reg.max < 0.2
    tr = m.limits.travel
    box = (tr["X"][0], tr["X"][1], tr["Y"][0], tr["Y"][1])
    err = {c.ref: np.hypot(*(reg.to_machine(board, (c.x, c.y)) - pose.to_machine(board, (c.x, c.y))))
           for c in board.components}
    near = [e for c in board.components if dryrun._inside(box, reg.to_machine(board, (c.x, c.y))) for e in [err[c.ref]]]
    # the feature finders are good to ~0.1 mm at 5 px/mm (a hole over a groove, a corner's edge): over random board
    # poses the reachable parts land within 0.13 mm, the far side (4 features on one side only) within 0.2 mm
    assert len(near) > 50 and max(near) < 0.15, max(near)
    assert max(err.values()) < 0.2, max(err.values())
    # 4 nearby features: the report must not show the (meaningless) scale and skew
    assert not any("skew" in t for t in _texts(m, 30) if "fit on 4" in t)


def test_the_overlay_draws_the_pads_in_the_picture(ready):
    """The drawn pads (0..1 of the picture) must sit on the bright pads of the live picture."""
    import cv2
    m = ready
    ov = m.overlay()
    w, h = ov["w"], ov["h"]
    inside = [p for p in ov["pads"] if all(0 <= x <= 1 and 0 <= y <= 1 for x, y in p)]
    assert len(inside) > 10
    mask = np.zeros((h, w), np.uint8)
    for p in inside:
        cv2.fillPoly(mask, [np.round(np.array(p) * [w, h]).astype(np.int32)], 1)
    mask = cv2.erode(mask, np.ones((3, 3), np.uint8))
    ring = cv2.dilate(mask, np.ones((9, 9), np.uint8)) - cv2.dilate(mask, np.ones((5, 5), np.uint8))
    img = np.mean(np.asarray(m.cameras["camera_head"].frames_after(time.monotonic(), 2), np.float32), axis=0)
    assert np.median(img[mask > 0]) - np.median(img[ring > 0]) > 25


def test_the_20_part_dry_run(ready):
    """docs/13 §11 Phase C: 20 parts from 2 strips to the registered board, camera only."""
    m = ready
    board, reg = m.vision.registration()
    pose = m.vision.sim_board_pose
    w = m.cameras["camera_head"].world
    strips = {s: sim.Strip(slot=s) for s in (3, 4)}
    for s in strips.values():
        w.add(sim.strip_layer(w, s))
    slots = {}
    for s in strips:
        c, u, v = w.slot_frame(s)
        slots[s] = dryrun.SlotSetup(s, c, u, v, vp.StripSpec(body=(2.0, 1.25)), strips[s].z_top)
    tr = m.limits.travel
    box = (tr["X"][0], tr["X"][1], tr["Y"][0], tr["Y"][1])
    inside = [c for c in board.components if c.side == "B" and c.package == "0805"
              and dryrun._inside(box, reg.to_machine(board, (c.x, c.y)))]
    assert len(inside) >= 20
    plan = [(c.ref, 3 if i < 10 else 4) for i, c in enumerate(inside[:20])]
    asked = []
    out: list[dryrun.DryPart] = []

    def job():
        out.extend(dryrun.run(MachineRig(m, "home"), m.vision.model(), board, reg, plan, slots,
                              m.vision.board_top_z(board), confirm=lambda img, f: asked.append(f) or True,
                              say=lambda s: None, reach=box))
    t0 = time.monotonic()
    assert m.worker.submit("dry run", job)
    assert m.worker.wait(600)
    assert m.safety.lock is None, _texts(m, 10)
    assert len(asked) == 2                                    # the operator confirms the first part of each slot
    assert all(p.pick_xy is not None and p.place_xy is not None for p in out), [(p.ref, p.why) for p in out]
    pick_err, place_err = [], []
    for p in out:
        s = strips[p.slot]
        c, u, _v = w.slot_frame(p.slot)
        us = np.array([s.first_u + k * s.pitch for k in range(len(s.pockets))])
        k = int(np.argmin(abs(us - np.dot(np.array(p.pick_xy) - c, u))))
        pick_err.append(np.hypot(*(np.array(p.pick_xy) - (c + u * us[k]))))
        comp = board.component(p.ref)
        place_err.append(np.hypot(*(np.array(p.place_xy) - pose.to_machine(board, (comp.x, comp.y)))))
    assert len({(p.slot, round(np.dot(np.array(p.pick_xy) - w.slot_frame(p.slot)[0], w.slot_frame(p.slot)[1]) / 4))
                for p in out}) == 20                          # never the same pocket twice
    assert max(pick_err) < 0.1, max(pick_err)
    assert max(place_err) < 0.15, max(place_err)                # the registration: ~0.1 mm (see above)
    assert sum(bool(p.pads_seen) for p in out) == 20
    assert time.monotonic() - t0 < 300


def test_a_click_on_the_picture_brings_that_spot_to_the_centre(ready):
    m = ready
    model = m.vision.model()
    w, h = model.size
    _run(m, m.goto(-50.0, -60.0))                             # room around it
    before = m.position()
    want = model.px_to_xy(np.array([[0.7 * w, 0.4 * h]]), 0.0)[0]
    _run(m, m.click(0.7, 0.4))
    after = m.position()
    assert abs(after["X"] - before["X"] - want[0]) < 0.05 and abs(after["Y"] - before["Y"] - want[1]) < 0.05


def test_a_click_on_the_board_uses_the_board_height(ready):
    """The board top is 1.6 mm above the bed: the 25° camera sees it ~0.75 mm shifted. A click on a part's place
    must bring that place (not a bed point under it) to the picture centre."""
    m = ready
    board, reg = m.vision.registration()
    model = m.vision.model()
    w, h = model.size
    c = next(c for c in board.components if c.side == "B" and c.package == "0805"
             and dryrun._inside((-110, -15, -200, -100), reg.to_machine(board, (c.x, c.y))))
    target = reg.to_machine(board, (c.x, c.y))
    _run(m, m.goto(float(target[0]) - 8.0, float(target[1]) + 5.0))
    pos = m.position()
    u, v = model.xy_to_px([target - np.array([pos["X"], pos["Y"]])], m.vision.board_top_z(board))[0]
    _run(m, m.click(u / w, v / h))
    pos = m.position()
    assert np.hypot(pos["X"] - target[0], pos["Y"] - target[1]) < 0.05
    assert np.allclose(reg.to_board(board, target), (c.x, c.y), atol=1e-9)


def test_a_blocked_move_locks_the_machine(ready):
    """Last in the file: it locks the module's machine. The frame's end is moved next to the head (the head hits
    it at once); Grbl still counts the steps, only the picture can tell."""
    m = ready
    g = m.sim_grbl
    pos = m.position()
    g.ends["X"] = (g.ends["X"][0], g.phys["X"] + 0.2)
    r = m.goto(pos["X"] + 12.0, pos["Y"])
    assert r["ok"], r
    assert m.worker.wait(60)
    assert m.safety.mode == "locked" and m.safety.lock.title.lower().startswith("stall") or \
        "stall" in str(m.safety.lock).lower(), (m.safety.mode, m.safety.lock, _texts(m, 8))


def test_calibration_is_refused_when_the_head_loses_steps(tmp_path):
    """The view grid runs into the frame's end: the head-move scale is wrong (the bench grid run, 2026-10-07),
    so the model is not saved."""
    def tight(raw):
        # the grid's back row (Y −45.5) is beyond it. (An end at the grid's first column is not enough: every later
        # view then lands the same 9 mm off, a plain offset that does no harm before homing.)
        raw["sim"]["hard_ends"]["Y"] = [-330.0, -50.0]
        raw["sim"]["start"].update(X=-44.0, Y=-60.5)         # over the bars, 10.5 mm in front of that end
    m = Machine(_cfg(tmp_path, tight), sim=True, data_dir=tmp_path / "data", watch=False)
    try:
        _run(m, m.connect(), 20)
        m.sim_event("charuco")
        r = m.calibrate("bed")
        assert r["ok"]
        assert m.worker.wait(300)
        if m.safety.lock is None:                            # not caught by a move: then by the solve
            m.sim_event("charuco_block")
            _run(m, m.calibrate("block", 5.0))
            m.sim_event("charuco_off")
            assert m.calibrate("solve")["ok"]
            assert m.worker.wait(120)
        assert m.vision.model() is None, _texts(m, 10)
        assert any("lost steps" in t or "scale" in t.lower() or "stall" in t.lower() for t in _texts(m, 40))
    finally:
        m.close()
