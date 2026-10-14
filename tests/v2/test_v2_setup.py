"""Phase D: the calibration wizard (docs/13 §4.6), the validity chain and Undo (§4.10), derived heights, the nozzle
offset and rotation table from ink marks, the bed survey, pick and place with the place check, and the first part.

The simulator's nozzle has a TRUE offset (4.97, 29.44), a needle 1.56 mm off the servo axis and (here) a servo that
turns only 95 % of its nominal angle: the wizard must measure all of it. One machine goes through the whole wizard
once for the module (about 2 minutes); the tests run in this file's order."""
from __future__ import annotations

import random

import math
import time

import cv2
import numpy as np
import pytest

from conftest import BOARDS
from pnp import sim
from pnp.calibration import chain
from pnp.calibration.heights import HeightRules
from pnp.calibration.nozzle import Mark, NozzleOffset, choose_angles, fit_offset, fit_table, mark_angles, repeat_spread
from pnp.calibration.store import CalibrationStore
from pnp.machine import Machine
from pnp.profile import ProfileError
from pnp.vision.marks import find_change
from test_v2_machine_vision import _cfg


# ================================================================ pure parts
def test_the_store_keeps_previous_values_for_undo(tmp_path):
    st = CalibrationStore(tmp_path / "c.yaml", tmp_path / "hist")
    st.put("k", {"v": 1}, {"camera": "a"}, "m1")
    d1 = st.date("k")
    st.put("k", {"v": 2}, {"camera": "a"}, "m2")
    assert st.date("k") != d1                                   # two saves in one second still differ
    assert st.value("k", {"camera": "a"})[0] == {"v": 2}
    assert st.status("k", {"camera": "b"})[0] == "stale"
    assert st.undo("k") and st.value("k")[0] == {"v": 1} and st.date("k") == d1
    assert st.undo("k") and st.get("k") is None
    assert not st.undo("k")
    assert any((tmp_path / "hist").iterdir())                    # a copy before every write


def test_the_chain_names_upstream_results():
    assert set(chain.dependants("camera_head")) >= {"bed_survey", "nozzle_offset", "rotation_table", "bottom_camera"}
    assert chain.dependants("first_part") == []


def _marks(axis, r, phase, angles, noise=0.0, rng=np.random.default_rng(1), head=(0.0, 0.0), real=None):
    out = []
    for i, a in enumerate(angles):
        ra = a if real is None else real[i]
        t = math.radians(phase + ra)
        tip = np.asarray(axis) + r * np.array([math.cos(t), math.sin(t)]) + rng.normal(0, noise, 2)
        h = np.asarray(head) + [i * 2.5, 0]
        out.append(Mark(a, tuple(h), tuple(h + tip)))
    return out


def test_the_offset_circle_gives_axis_runout_and_phase():
    m = _marks((4.97, 29.44), 1.56, 30.0, [-90, -45, 0, 45, 90], noise=0.01)
    off = fit_offset(m)
    assert off.runout_known and abs(off.runout - 1.56) < 0.03 and off.rms < 0.03
    assert np.hypot(*(np.array(off.axis) - (4.97, 29.44))) < 0.03 and abs(off.phase_deg - 30.0) < 2.0
    assert np.hypot(*(off.tip(45) - m[3].offset)) < 0.05
    one = fit_offset(m[:1])                                     # one mark: the tip there, runout unknown
    assert not one.runout_known and np.allclose(one.tip(80), m[0].offset)
    assert NozzleOffset.from_dict(off.to_dict()).tip(10) == pytest.approx(off.tip(10))


def test_the_angle_table_and_the_repeat_from_marks():
    pulses = [700, 1000, 1300, 1500, 1800, 2100, 2500]
    real = [(p - 1500) * 0.1282 for p in pulses]               # a servo that turns 95 % of its nominal 0.135°/µs
    off = fit_offset(_marks((5, 29), 1.56, 0.0, [-90, -45, 0, 45, 90]))
    m = _marks((5, 29), 1.56, 0.0, [0] * len(pulses), noise=0.005, real=real)
    for mk, p in zip(m, pulses):
        mk.pulse = p
    t = fit_table(m, off)
    assert abs(t.deg_per_us - 0.1282) < 0.002 and t.rms_deg < 1.0
    assert abs(t.angle_for(1500)) < 1.0 and t.pulse_for(t.angle_for(1900)) == pytest.approx(1900, abs=3)
    rep = _marks((5, 29), 1.56, 0.0, [0] * 5, real=[0, 2, -1, 3, -2])
    assert repeat_spread(rep, off) == pytest.approx(2.6, abs=0.3)
    with pytest.raises(ValueError):
        fit_table(m, NozzleOffset((5, 29), 0.2, 0, 0, True))     # too close to the axis to read angles
    assert mark_angles(m[:1], off)[0] == pytest.approx(real[0], abs=1.0)


def test_the_servo_angles_leave_room_for_the_turn():
    assert choose_angles(90.0, (-135, 135), True) == (-45.0, 45.0, 90.0)
    assert choose_angles(170.0, (-135, 135), True)[2] == pytest.approx(-10.0)      # a chip: 180° is the same part
    a, b, t = choose_angles(170.0, (-135, 135), False)                              # polar: really 170°
    assert t == 170.0 and -135 <= a <= 135 and -135 <= b <= 135 and b - a == 170.0
    with pytest.raises(ValueError):
        choose_angles(170.0, (-60, 60), False)


def test_heights_are_derived():
    r = HeightRules(press=0.3, clearance=3.0, tape_floor=0.0, board_top=1.6, tallest_part=1.35)
    assert r.pick(1.35) == pytest.approx(1.05) and r.place(1.35) == pytest.approx(2.65)
    assert r.mark(0.1) == pytest.approx(-0.2)
    assert r.moving() == pytest.approx(1.6 + 1.35 + 1.35 + 3.0)
    with pytest.raises(ValueError):
        HeightRules(0.3, 3.0, 0.0, None, 1.0).place(1.0)


def test_a_new_ink_mark_is_found_and_old_ones_are_not():
    rng = np.random.default_rng(0)
    before = np.full((200, 300), 230, np.float32) + rng.normal(0, 2, (200, 300)).astype(np.float32)
    cv2.circle(before, (100, 100), 2, 30, -1)                   # an old mark
    after = before.copy()
    cv2.circle(after, (160, 110), 2, 30, -1)
    ch = find_change(before, after, (150, 100), 40, darker=True)
    assert ch is not None and abs(ch.u - 160) < 0.5 and abs(ch.v - 110) < 0.5
    assert find_change(before, before + 40, (150, 100), 40, darker=True) is None   # the light changed: no answer


def test_the_press_must_be_less_than_the_spring(make_cfg):
    def bad(raw):
        raw["nozzles"]["items"][0]["spring_mm"] = 0.2
    with pytest.raises(ProfileError) as e:
        Machine(make_cfg(bad), sim=True, watch=False)
    assert "press_mm" in str(e.value)


# ================================================================ the whole wizard in the simulator
def _run(m: Machine, r: dict, t: float = 600) -> None:
    assert r["ok"], r
    assert m.worker.wait(t), "timeout"
    assert m.safety.lock is None, [e["text"] for e in m.events.tail(8)]


def _texts(m, n=30):
    return [e["text"] for e in m.events.tail(n)]


def _state(m, sid):
    return next(s for s in m.wizard() if s["id"] == sid)


@pytest.fixture(scope="module")
def wiz(tmp_path_factory):
    root = tmp_path_factory.mktemp("pnp_setup")

    def change(raw):
        raw["sim"]["servo_scale"] = 0.95                        # the real servo turns less than its datasheet
    m = Machine(_cfg(root, change), sim=True, data_dir=root / "data", watch=False)
    try:
        _run(m, m.connect(), 20)
        yield m
    finally:
        m.close()


def test_a_fresh_machine_starts_with_everything_missing(wiz):
    st = {s["id"]: s for s in wiz.wizard()}
    assert st["connect"]["state"] == "valid"
    assert st["camera"]["state"] == "missing" and st["home_z"]["blocked_by"] == ["camera"]
    assert st["first_part"]["blocked_by"]
    assert st["travel"]["state"] == "stale"                     # from the profile, not measured
    r = wiz.setup_action("nozzle_marks", {"sheet_mm": 0.1})
    assert not r["ok"] and "Home" in r["why"] + r.get("todo", "")


def test_step3_and_4_camera_home_and_z(wiz):
    m = wiz
    _run(m, m.jog(dy=-40))
    _run(m, m.jog(dx=-20))
    m.sim_event("charuco")
    _run(m, m.calibrate("bed"))
    m.sim_event("charuco_block")
    _run(m, m.calibrate("block", 5.0))
    m.sim_event("charuco_off")
    _run(m, m.calibrate("solve"))
    _run(m, m.jog(dx=20))
    _run(m, m.jog(dy=40))
    _run(m, m.home())
    _run(m, m.jog(dz=-20.0))                                    # the tip just touches the bed (sim: Z 0)
    _run(m, m.set_z(0.0))
    _run(m, m.jog(dz=8.0))
    assert _state(m, "camera")["state"] == "valid" and _state(m, "home_z")["state"] == "valid"
    assert _state(m, "heights")["state"] == "valid"


def test_step2_travel_finds_an_end_with_the_camera(wiz):
    m = wiz
    _run(m, m.goto(0.0, -60.0))
    _run(m, m.setup_action("creep", {"dir": "X+"}))
    end = m.setup.ends["X+"]
    assert 10.0 <= end <= 13.01, end                            # the frame's end is at X 15: one step + 2 mm inside
    assert not m.safety.homed and "Home again" in " ".join(_texts(m, 3))
    _run(m, m.jog(dx=-20.0))
    _run(m, m.jog(dx=-14.0))
    _run(m, m.jog(dy=60.0))
    _run(m, m.home())
    _run(m, m.setup_action("save_travel", {}))
    assert m.limits.source == "measured" and m.limits.travel["X"][1] == pytest.approx(end)
    assert _state(m, "travel")["state"] == "valid"


def test_step5_the_nozzle_offset_from_ink_marks(wiz):
    m = wiz
    noz = sim.nozzle_of(m.sim_grbl)
    _run(m, m.goto(-77.0, -60.0))
    m.sim_event("mark_sheet")
    _run(m, m.setup_action("nozzle_marks", {"sheet_mm": 0.1}))
    v = chain.entry(m, "nozzle_offset")["value"]
    assert np.hypot(*(np.array(v["axis"]) - noz.t.offset)) < 0.05, v["axis"]
    assert abs(v["runout"] - noz.t.runout) < 0.05 and v["rms"] < 0.03
    assert noz.crashes == 0 and abs(noz.max_press - 0.3) < 0.02     # pressed exactly the profile's press_mm
    assert len(noz.marks) == 5
    assert _state(m, "nozzle_offset")["state"] == "valid"


def test_a_big_offset_change_waits_for_a_yes(wiz):
    m = wiz
    good = chain.entry(m, "nozzle_offset")["value"]
    far = dict(good, axis=[good["axis"][0] + 2.0, good["axis"][1]])
    m.setup._offer("nozzle_offset", far, "test", {}, "Test offset")
    assert "nozzle_offset" in m.setup.pending
    assert chain.entry(m, "nozzle_offset")["value"]["axis"] == good["axis"]          # not saved yet
    assert any(a["id"] == "accept" for a in _state(m, "nozzle_offset")["actions"])
    m.setup.pending.clear()


def test_step6_the_rotation_table_measures_the_real_servo(wiz):
    m = wiz
    _run(m, m.goto(-77.0, -75.0))
    _run(m, m.setup_action("rotation_marks", {"sheet_mm": 0.1}))
    t = m.calib.get("rotation_table")["value"]
    assert abs(t["deg_per_us"] - 0.135 * 0.95) < 0.003, t["deg_per_us"]
    assert t["accuracy_deg"] is not None and t["accuracy_deg"] < 3.0
    assert m.caps.rotation.table is not None
    assert _state(m, "rotation")["state"] == "valid"


def test_step8_the_bed_survey(wiz):
    m = wiz
    _run(m, m.setup_action("survey", {}))
    lab = m.calib.get("bed_survey")["value"]["labels"]
    plan = m.bed["layout"]["slot_marker_xy"]
    for n in (3, 4, 5, 6):
        assert np.hypot(*(np.array(lab[f"slot_{n}"]["xy"]) - plan[n])) < 0.1, n
        assert np.dot(lab[f"slot_{n}"]["u"], (0, -1)) > 0.999        # the labels run to the front
    assert "slot_7" not in lab                                     # too far outside the travel: said, not guessed


def test_step9_and_10_the_first_part(wiz):
    m = wiz
    iid = m.boards.new_import()
    m.boards.add_file(iid, "ht5023.kicad_pcb.gz", (BOARDS / "ht5023.kicad_pcb.gz").read_bytes())
    m.boards.run(iid)
    m.boards.save(iid, "ht5023")
    random.seed(7)                                                   # a repeatable board pose
    m.sim_event("board")
    _run(m, m.register_board("B", "back"))
    m.sim_event("strip")
    r = m.setup_action("first_part", {"slot": 3, "ref": "R41"})
    assert r["ok"]
    m.worker.wait(60)
    assert "not confirmed" in " ".join(_texts(m, 2))               # the operator must say yes first
    _run(m, m.setup_action("look_at_slot", {"slot": 3, "package": "0805"}))
    assert m.setup.candidate is not None
    assert m.setup_action("confirm", {"yes": True})["ok"]
    noz = sim.nozzle_of(m.sim_grbl)
    n0 = len(noz.placed)
    t0 = time.monotonic()
    _run(m, m.setup_action("first_part", {"slot": 3, "ref": "R41"}))
    assert len(noz.placed) == n0 + 1 and noz.crashes == 0
    board, reg = m.vision.registration()
    c = board.component("R41")
    true_target = m.vision.sim_board_pose.to_machine(board, (c.x, c.y))
    pl = noz.placed[-1]
    true_err = np.hypot(pl.x - true_target[0], pl.y - true_target[1])
    assert true_err < 0.15, true_err                              # docs/13 §8: within 0.15 mm
    true_ang = (pl.angle - (m.vision.sim_board_pose.theta + c.rotation) + 90) % 180 - 90
    assert abs(true_ang) < 3.0
    last = m.setup.last_first_part
    assert last["error"] is not None
    measured = np.array(last["error"])                             # the camera's place check vs the truth
    assert np.hypot(*(measured - (np.array([pl.x, pl.y]) - reg.to_machine(board, (c.x, c.y))))) < 0.12   # the place check: ~0.06 mean, 0.12 worst (bench of 36 places)
    assert abs(noz.max_press - 0.3) < 0.02                         # the pick and the place pressed press_mm
    m.setup_action("first_part_answer", {"ok": True})
    assert _state(m, "first_part")["state"] == "valid"
    assert time.monotonic() - t0 < 120


def test_every_step_is_valid_after_the_wizard(wiz):
    st = {s["id"]: s["state"] for s in wiz.wizard()}
    assert all(v == "valid" for v in st.values()), [(k, v, s["why"]) for s in wiz.wizard() for k, v in [(s["id"], s["state"])] if v != "valid"]


def test_undo_makes_the_results_built_on_it_stale(wiz):
    m = wiz
    assert m.undo("nozzle_offset")["ok"]                           # back to "nothing": the first measurement
    st = {s["id"]: s for s in m.wizard()}
    assert st["nozzle_offset"]["state"] == "missing"
    assert "nozzle_offset" in st["first_part"]["blocked_by"]         # grey until step 5 is measured again
    r = m.setup_action("first_part", {"slot": 3, "ref": "R42"})
    assert r["ok"]
    m.worker.wait(60)
    assert "nozzle offset" in " ".join(_texts(m, 2)).lower()        # refused: no valid offset


def test_z_goes_down_only_over_a_known_surface(wiz):
    from pnp.job.pickplace import PickPlace
    from pnp.motion.state import Refused
    m = wiz
    m.setup._offer("nozzle_offset", {"axis": [4.97, 29.44], "runout": 0.0, "phase_deg": 0.0, "rms": 0.0,
                                     "runout_known": False, "marks": []}, "test", {}, "test")
    board, reg = m.vision.registration()
    pp = PickPlace(m, board, reg)
    pp.lift()
    _run(m, m.goto(0.0, -40.0))                                    # the tip over the bed, no tray, no board
    with pytest.raises(Refused):
        pp.down(1.0, "tape")
    with pytest.raises(Refused):
        pp.down(1.0, "board")
    pos = m.position()
    pp.camera_over((pos["X"], pos["Y"]))
    pp.lift()
    m.safety.z_ref = False
    try:
        with pytest.raises(Refused):
            m._move_z(0.0)
    finally:
        m.safety.z_ref = True
