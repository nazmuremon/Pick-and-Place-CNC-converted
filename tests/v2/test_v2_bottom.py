"""Phase G: the bottom camera (docs/13 §4.14 PlaceCorrector). The simulator's camera sits in the bed at (0, -150),
40 px/mm, turned 7°, mirrored, sharp at tip Z 3.0: the Setup step must measure all of it; then a job looks at every
part from below and puts its real centre on the target."""
from __future__ import annotations

import math

import numpy as np
import pytest
import yaml

from pnp import sim
from pnp.calibration import chain
from pnp.vision.bottom import BottomCal, find_blob
from simwizard import load, ready_machine, run

TRUE = {"xy": [0.0, -150.0], "px_per_mm": 40.0, "focus_z": 3.0, "turn_deg": 7.0, "mirror": True}


def with_bottom(raw):
    raw["cameras"]["bottom"] = {"driver": "v4l2", "device": "/dev/null", "resolution": [640, 480], "fourcc": None,
                                "hardware_id": "sim-bottom-1", "moves_with_z": False, "tilted": False}
    raw["sim"]["bottom_camera"] = TRUE


def test_a_blob_and_the_bottom_map():
    img = np.full((200, 300), 18, np.uint8)
    import cv2
    cv2.rectangle(img, (100, 90), (160, 110), 200, -1)
    b = find_blob(img)
    assert abs(b.u - 130) < 1 and abs(b.v - 100) < 1 and (b.angle_px < 5 or b.angle_px > 175)
    cal = BottomCal([[-40.0, 0.0], [0.0, -40.0]], [150.0, 100.0], [5.0, 6.0], 3.0)
    assert np.allclose(cal.to_machine([190.0, 100.0]), [4.0, 6.0])        # mirrored: right in the picture = -X
    assert abs(cal.px_per_mm() - 40) < 1e-9


@pytest.fixture(scope="module")
def bm(tmp_path_factory):
    root = tmp_path_factory.mktemp("pnp_bottom")
    m = ready_machine(root, with_bottom)
    try:
        yield m
    finally:
        m.close()


def test_the_bottom_camera_step_measures_the_camera(bm):
    m = bm
    noz = sim.nozzle_of(m.sim_grbl)
    st = {s["id"]: s for s in m.wizard()}
    assert st["bottom"]["n"] == "5b" and st["bottom"]["state"] == "missing"
    tip = noz.tip_xy() - np.array([m.sim_grbl.phys["X"], m.sim_grbl.phys["Y"]])
    head = np.array(TRUE["xy"]) - tip + [0.4, -0.3]                 # roughly over it, as the operator would jog
    run(m, m.goto(*head))
    run(m, m.jog(dz=TRUE["focus_z"] - m.position()["Z"]))
    run(m, m.setup_action("bottom_calibrate", {}))
    cal = BottomCal.from_dict(chain.value(m, "bottom_camera"))
    # picture v points down: a camera looking down has det(A) < 0; a mirrored one looking up has det(A) > 0
    assert abs(cal.px_per_mm() - 40.0) < 0.5 and np.linalg.det(np.asarray(cal.A)) > 0
    assert np.hypot(*(np.asarray(cal.cam_xy) - TRUE["xy"])) < 0.05, cal.cam_xy
    a = math.degrees(math.atan2(cal.A[1][0], -cal.A[0][0]))               # +X head -> the picture's direction
    assert abs(((abs(a) - TRUE["turn_deg"]) + 180) % 360 - 180) < 0.5, a
    assert noz.crashes == 0
    assert {s["id"]: s["state"] for s in m.wizard()}["bottom"] == "valid"


def test_a_job_looks_at_every_part_from_below(bm):
    m = bm
    plan = m.jobs.plan()
    assert all(p.bottom for p in plan.parts)                       # job.use_bottom_camera: always
    ld = plan.loads[0]
    load(m, ld.trays)
    first = plan.parts[0]
    tray = next(t for t, ln in ld.trays.items() if ln == first.line)
    run(m, m.setup_action("first_part", {"slot": tray, "ref": first.ref}))
    m.setup_action("first_part_answer", {"ok": True})
    refs = [p.ref for p in plan.parts if p.load == 1][1:7]
    run(m, m.jobs.start("real", refs=refs), 1800)
    assert m.jobs.status()["status"] == "done", m.jobs.status()["pause"]
    board, _ = m.vision.registration()
    pose = m.vision.sim_board_pose
    noz = sim.nozzle_of(m.sim_grbl)
    for p in m.jobs.job.parts:
        c = board.component(p.ref)
        t = pose.to_machine(board, (c.x, c.y))
        q = min(noz.placed, key=lambda q: np.hypot(q.x - t[0], q.y - t[1]))
        assert np.hypot(q.x - t[0], q.y - t[1]) < 0.12, p.ref
        assert abs((q.angle - (pose.theta + c.rotation) + 90) % 180 - 90) < 3.0, p.ref
        assert any("bottom" in k for k in p.result["photos"]), p.result["photos"].keys()
    assert noz.crashes == 0


def test_a_bottom_camera_turns_soic_parts_into_machine_parts(bm):
    m = bm
    user = m.cfg_dir / "packages.user.yaml"
    d = yaml.safe_load(user.read_text()) or {}
    d.setdefault("packages", {})["SOIC-8"] = {"tape_angle": 0}             # TEST: as if measured on a first part
    user.write_text(yaml.safe_dump(d))
    pv = m.jobs.plan_view()
    soic = [s for s in pv["skipped"] if s["package"] == "SOIC-8"]
    assert soic and all("12 mm tape" in s["reason"] for s in soic)        # unlocked; it needs a 12 mm tray now
