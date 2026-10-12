"""The pick and place page (ui/engine.py, ui/app.py) against the simulated head of test_nozzle_calibrate: lines,
feeders, board corners, the dry and the real job, the reach, and the web API."""
import sys
import threading
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
from ui import app as ua  # noqa: E402
from ui import engine as e  # noqa: E402
import test_dry_run_bench as tdr  # noqa: E402
import test_nozzle_calibrate as tnc  # noqa: E402

pytest.importorskip("fastapi")


@pytest.fixture(autouse=True)
def private_files(tmp_path, monkeypatch):
    monkeypatch.setattr(e, "FEEDER_FILE", tmp_path / "feeder.yaml")
    monkeypatch.setattr(e, "REG_FILE", tmp_path / "board_registration.yaml")
    monkeypatch.setattr(e, "JOB_FILE", tmp_path / "job_state.yaml")
    monkeypatch.setattr(tnc.nc, "CAL_FILE", tmp_path / "nozzle_calibration.yaml")
    monkeypatch.setattr(tnc.nc.cr, "RECTIFY_FILE", tmp_path / "camera_rectify.yaml")
    monkeypatch.setattr(tnc.nc.time, "sleep", lambda s: None)
    monkeypatch.setattr(tdr.d.time, "sleep", lambda s: None)
    monkeypatch.setattr(e.time, "sleep", lambda s: None)


@pytest.fixture(scope="module")
def board():
    if not tdr.d.BOARD_JS.exists():
        pytest.skip("no board data")
    return tdr.d.load_board()


class SimPnP(e.PnPMachine, tnc.SimCal):
    """The page's machine with the simulated head and camera (the nozzle really picks, turns and places)."""

    def __init__(self, board, world, tmp_path):
        tnc.SimCal.__init__(self, board, world, tmp_path)
        self.setup_pnp(tdr.BED)


def make(board, tmp_path, parts=3, real=False, aa=1):
    m = tnc.real_sim(board, tmp_path, parts=parts, aa=aa) if real else tnc.sim(board, tmp_path, parts=parts, aa=aa)
    p = SimPnP.__new__(SimPnP)
    p.__dict__.update(m.__dict__)
    p.setup_pnp(tdr.BED)
    p.cam = p                                                # the simulated camera looks from THIS head
    p.stop_flag = threading.Event()
    p.M, p.c = (m.M, m.c) if m.M is not None else (np.eye(2), p.nominal_T())     # the board lies on the plan
    return p


def line_key(m, value, package):
    return next(g["key"] for g in m.lines() if g["value"] == value and g["package"] == package)


# ---------------------------------------------------------------- lines and feeders

def test_values_are_read_like_the_parts_list():
    assert e.value_key("100nF/50V") == ("C", pytest.approx(1e-7)) and e.value_key("0.1uF")[1] == pytest.approx(1e-7)
    assert e.value_key("1k5") == ("R", 1500.0) and e.value_key("2.2M") == ("R", 2.2e6) and e.value_key("100R")[1] == 100
    assert e.value_key("1N4148W")[0] == "?"
    assert e.pretty("C", 1e-7) == "100 nF" and e.pretty("R", 4700) == "4.7 kohm"


def test_lines_group_one_value_and_package_and_say_what_the_machine_can_place(board):
    lines = {g["label"]: g for g in e.board_lines(board, "B", e.load_feeder())}
    c100 = lines["100 nF  C_0805  x17"]                      # 0.1uF + 100nF + 100nF/50V, C_ and D_0805
    assert c100["supported"] and set(c100["values"]) == {"0.1uF", "100nF", "100nF/50V"}
    by_pkg = {g["package"]: g for g in lines.values()}
    assert not by_pkg["SOD-123"]["supported"] and "polarity" in by_pkg["SOD-123"]["why"]
    assert not by_pkg["SOT-23"]["supported"]
    lq = next(g for g in lines.values() if "LQFP" in g["package"])
    assert not lq["supported"]


def test_feeders_are_kept_and_unsupported_lines_refused(board, tmp_path):
    m = make(board, tmp_path)
    key = line_key(m, "100 nF", "C_0805")
    m.assign(3, key)
    saved = yaml.safe_load(e.FEEDER_FILE.read_text())
    assert saved["slots"][3]["line"] == key and saved["slots"][3]["qty"] == 17
    sod = next(g for g in m.lines() if g["package"] == "SOD-123")
    with pytest.raises(ValueError, match="polarity"):
        m.assign(4, sod["key"])
    with pytest.raises(ValueError):
        m.assign(9, key)                                     # no slot 9 on this bed
    m.assign(3, None)
    assert m.assignments() == []


# ---------------------------------------------------------------- board corners

def test_four_corners_set_by_hand_register_the_board_and_are_used_again(board, tmp_path):
    m = make(board, tmp_path)
    a = np.radians(0.4)
    M_true = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    c_true = m.nominal_T() + (3.0, -2.0)
    m.M = m.c = None
    for k, hole in enumerate(tdr.d.HOLES):
        m.pos = M_true @ (m.A @ m.hole_xy[k]) + c_true       # the owner centred the camera on this hole
        m.job_set_corner(hole)
    m.job_fit_board()
    assert m.M is not None and max(m.reg_info["residuals_mm"]) < 1e-6
    part = m.todo[0]
    want = M_true @ (m.A @ np.array([part["x"], part["y"]])) + c_true
    assert np.allclose(m.board_to_machine([part["x"], part["y"]]), want, atol=1e-6)
    m2 = make(board, tmp_path)                               # next session, same side and camera height
    m2.M = m2.c = None
    assert m2.apply_saved_registration() and np.allclose(m2.board_to_machine([part["x"], part["y"]]), want)


def test_corners_that_do_not_fit_the_board_are_refused(board, tmp_path):
    m = make(board, tmp_path)
    m.M = m.c = None
    for k, hole in enumerate(tdr.d.HOLES[:3]):
        m.pos = m.A @ m.hole_xy[k] * 1.1 + m.nominal_T()      # 10 % too big: a wrong hole
        m.job_set_corner(hole)
    m.pos = m.A @ m.hole_xy[3] + m.nominal_T()
    m.job_set_corner("H4")
    m.job_fit_board()
    assert m.M is None and "don't fit" in m.messages[-1]


def test_corners_need_the_camera_height(board, tmp_path):
    m = make(board, tmp_path)
    m.z_ok = False
    m.job_set_corner("H1")
    assert not m.corners and "camera height" in m.messages[-1]


# ---------------------------------------------------------------- the job

def test_dry_job_visits_every_part_of_the_assigned_slot(board, tmp_path):
    m = make(board, tmp_path, parts=3)
    m.assign(3, line_key(m, "1 uF", "C_0805"))               # 2 parts on the B side
    m.job_run("dry")
    assert m.job_info["done"] == 2 and m.job_info["total"] == 2
    assert not m.world.placed                                # camera only
    assert (tmp_path / "job_report.yaml").exists()


def test_real_job_places_the_parts_on_their_pads(board, tmp_path):
    m = make(board, tmp_path, parts=3, real=True, aa=3)
    m.assign(3, line_key(m, "1 uF", "C_0805"))
    m.job_run("real")
    assert len(m.world.placed) == 2, m.messages
    refs = [r for g in m.lines() if g["value"] == "1 uF" for r in g["refs"]]
    for (centre, *_), ref in zip(m.world.placed, sorted(refs, key=lambda r: next(
            (p["y"], p["x"]) for p in board["parts"] if p["ref"] == r))):
        part = next(p for p in board["parts"] if p["ref"] == ref)
        assert np.hypot(*(centre - m.board_to_machine([part["x"], part["y"]]))) < 0.1
    assert m.placed_refs == set(refs) and set(yaml.safe_load(e.JOB_FILE.read_text())["placed"]) == set(refs)
    m.job_run("real")                                        # again: nothing more to place on this board
    assert len(m.world.placed) == 2 and m.job_info["total"] == 0


def test_an_empty_slot_waits_for_a_refill_then_goes_on(board, tmp_path):
    m = make(board, tmp_path, parts=2)
    m.assign(3, line_key(m, "10 uF", "C_0805"))               # 8 parts, but only 2 in the strip
    states = []
    inner = m.wait_if_paused

    def owner():                                             # the owner refills the strip and presses Resume
        if m.pause_flag.is_set():
            states.append(m.run_state)
            m.world.spots = [(m.world.slot_xy(19.0 + 4.0 * k, -0.5), 2.2, 1.4, tdr.BLACK) for k in range(3)]
            m.pause_flag.clear()
        inner()
    m.wait_if_paused = owner
    m.job_run("dry")
    assert states == ["paused: refill slot 3"] * 2 and m.job_info["done"] == 8     # 2 + 3 + 3


def test_the_job_needs_feeders_home_and_a_registered_board(board, tmp_path):
    m = make(board, tmp_path)
    m.job_run("dry")
    assert "Assign components" in m.messages[-1]
    m.assign(3, line_key(m, "1 uF", "C_0805"))
    m.M = None
    m.job_run("dry")
    assert "Register the board" in m.messages[-1]


# ---------------------------------------------------------------- reach

def test_the_reach_is_measured_by_driving_to_the_four_ends(board, tmp_path, monkeypatch):
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "bench.yaml").write_text((ROOT / "config" / "bench.yaml").read_text())
    monkeypatch.setattr(e.t, "ROOT", tmp_path)
    monkeypatch.setattr(e, "load_bench", lambda: yaml.safe_load((tmp_path / "config" / "bench.yaml").read_text()))
    m = make(board, tmp_path)
    m.job_reach_start()
    assert m.reach is None and m.reach_mode
    m.extremes = np.array([-150.0, -6.0, -225.0, 20.0])
    m.job_reach_save()
    assert m.reach == {"x": [-150, -6], "y": [-225, 20]} and not m.reach_mode and not m.reach_stale


# ---------------------------------------------------------------- the web API

class FakeCam:
    def __init__(self, img):
        self.img, self.t = img, 0.0

    def latest(self):
        self.t += 1.0
        return self.t, self.img


@pytest.fixture
def client(board, tmp_path):
    from fastapi.testclient import TestClient
    m = make(board, tmp_path, parts=3)
    m.submit = lambda fn, *a: (fn(*a), True)[1]             # run every job at once, in the test
    cam = FakeCam(m.world.picture(m.pos))
    return TestClient(ua.create_app(m, cam)), m, cam


def test_page_status_and_buttons(client):
    c, m, _ = client
    assert c.get("/").status_code == 200 and "Pick and place" in c.get("/").text
    s = c.get("/api/status").json()
    assert {"pos", "z", "heights", "offsets", "reach", "board", "run", "events"} <= set(s)
    before = m.pos.copy()
    assert c.post("/api/action/jog", json={"dx": -5, "dy": -2}).json()["ok"]
    assert np.allclose(m.pos - before, [-5, -2])
    assert c.post("/api/action/height", json={"name": "nonsense"}).status_code == 400
    assert c.post("/api/action/nothing").status_code == 404
    assert c.post("/api/stop").json()["ok"] and m.stop_flag.is_set()


def test_feeders_over_the_web(client):
    c, m, _ = client
    L = c.get("/api/lines").json()
    key = next(g["key"] for g in L["lines"] if g["supported"])
    assert c.post("/api/feeder", json={"slot": 3, "key": key}).json()["ok"]
    assert c.get("/api/lines").json()["assigned"]["3"]["line"] == key
    bad = next(g["key"] for g in L["lines"] if not g["supported"])
    assert not c.post("/api/feeder", json={"slot": 4, "key": bad}).json()["ok"]


def test_the_live_picture_is_a_jpeg_stream(client):
    _, m, cam = client
    chunk = next(ua.mjpeg_frames(m, cam, limit=1))
    assert chunk.startswith(b"--frame") and b"\xff\xd8" in chunk


# ---------------------------------------------------------------- map, turn, landing

def test_landing_then_a_click_gives_the_offset(board, tmp_path):
    m = make(board, tmp_path, real=True)
    true = tnc.tip_offset(1500)
    m.cal["offset_pick"] = {"xy": (true + (1.0, -0.5)).tolist(), "ref": tnc.REF}       # an old, rough estimate
    m.pos = m.pos + (-30.0, -40.0)
    head = m.pos.copy()
    m.job_land("part")
    assert m.click_mode == "land" and m.grbl.z == 0                # down and back up at Z0
    u, v = m.to_px((head + true).reshape(1, 2))[0]                  # the owner clicks the mark
    m.job_goto_px(u, v)
    assert np.allclose(m.cal["offset_pick"]["xy"], true, atol=0.01) and m.click_mode == "move"
    assert m.cal["offset_pick"]["ref"] == m.ref_id()


def test_landing_needs_the_height(board, tmp_path):
    m = make(board, tmp_path, real=True)
    del m.cal["touch_place"]
    m.job_land("place")
    assert m.click_mode == "move" and "place height" in m.messages[-1]


def test_the_nozzle_turns_by_the_servo_table(board, tmp_path):
    m = make(board, tmp_path, real=True)
    m.job_turn(10)
    assert m.turn_deg == 10 and "M3 S1578" in m.grbl.sent
    m.job_turn_to(0)
    assert "M3 S1500" in m.grbl.sent[-3:]


def test_a_click_on_the_map_moves_the_camera(board, tmp_path):
    m = make(board, tmp_path)
    m.job_goto_xy(-20.0, -10.0)
    assert np.allclose(m.pos - m.home, [-20, -10])
    data = m.map_data()
    assert set(data["holes"]) == {"H1", "H2", "H3", "H4"} and data["board"] and data["parts"]
    assert {"home", "slot_3"} <= set(data["labels"])


def test_map_and_new_buttons_over_the_web(client):
    c, m, _ = client
    assert c.get("/api/map").status_code == 200
    assert c.post("/api/action/goto", json={"x": -15, "y": -5}).json()["ok"]
    assert np.allclose(m.pos - m.home, [-15, -5])
    assert c.post("/api/action/land", json={"surface": "board"}).status_code == 400


def test_the_pick_spot_gives_the_pick_height_and_with_one_click_the_offset(board, tmp_path):
    m = make(board, tmp_path, real=True)
    true = tnc.tip_offset(1500)
    part = m.world.spots[0][0]
    del m.cal["touch_part"], m.cal["offset_pick"]
    m.pos = part - true                                       # the owner put the nozzle on the part ...
    m.grbl.z = tnc.TOUCH_PART - 0.3                           # ... at the pick height
    m.pump_on = True
    m.job_pick_here()
    assert not m.pump_on and m.pick_mark is not None
    m.grbl.z = 0.0                                            # Home lifted it and found the camera height
    m.apply_pick_mark()
    assert m.cal["touch_part"]["z"] == pytest.approx(tnc.TOUCH_PART) and m.click_mode == "land"
    m.pos = part + (2.0, 1.0)                                 # the camera brought near the part (map / arrows)
    u, v = m.to_px(part.reshape(1, 2))[0]
    m.job_goto_px(u, v)                                       # the owner clicks the part
    assert np.allclose(m.cal["offset_pick"]["xy"], true, atol=0.01)


def test_place_one_part_then_stop(board, tmp_path):
    m = make(board, tmp_path, parts=3, real=True)
    m.assign(3, line_key(m, "1 uF", "C_0805"))
    m.job_run_one()
    assert len(m.world.placed) == 1 and m.job_info["done"] == 1 and "Stopped after 1" in m.messages[-2]
