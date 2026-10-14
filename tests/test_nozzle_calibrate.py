"""Nozzle calibration (tools/nozzle_calibrate.py): the maths, and both jobs against a simulated head whose
nozzle really leaves marks, picks parts with the pump and turns them with the servo."""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import nozzle_calibrate as nc  # noqa: E402
import test_dry_run_bench as tdr  # noqa: E402
from pnp_paths import CONFIG_DIR  # noqa: E402  (the frozen test copy, tests/conftest.py)

BENCH = yaml.safe_load((CONFIG_DIR / "bench.yaml").read_text())
SHAFT = np.array([12.0, -3.0])          # true servo shaft: camera centre + this (mm)
RUNOUT = np.array([0.3, 0.0])           # the needle tip sits 0.3 mm off the shaft (at angle 0)
OD = 0.91                               # 20G needle
TOUCH_PAPER, TOUCH_PART = -10.0, -9.4   # Z where the tip touches the paper / a part in its slot
PART_H = 0.55
PAPER = np.array([-60.0, -120.0])       # spots, mm from HOME (HOME is at 0, 0 in the simulation)
TAPE = np.array([-60.0, -140.0])
MARKER = 7.4                            # camera scale when the touch heights were set
F_CAM, H0 = 540.0, 73.0                 # camera: scale = F_CAM / height; 73 mm above the labels at Z0
Z_PLAY = 0.05                           # Z lags this much after it turns from going down to going up
REF = "ref-1"                           # the remembered camera height the heights below were set at


def angle(pulse):
    """True nozzle angle (deg) for a pulse: 0.128 deg/us, not the nominal 0.135, and 2 deg off at the centre."""
    return (pulse - 1500.0) * 0.128 + 2.0


def rot(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


@pytest.fixture(autouse=True)
def private_cal_file(tmp_path, monkeypatch):
    monkeypatch.setattr(nc, "CAL_FILE", tmp_path / "nozzle_calibration.yaml")
    monkeypatch.setattr(nc.cr, "RECTIFY_FILE", tmp_path / "camera_rectify.yaml")      # never the real one
    monkeypatch.setattr(nc.time, "sleep", lambda s: None)
    monkeypatch.setattr(tdr.d.time, "sleep", lambda s: None)


class FakeGrbl:
    axes = type("Axes", (), {"steps_setting": staticmethod(lambda axis: "$100")})()

    def __init__(self) -> None:
        self.z, self.sent = 0.0, []

    def settings(self):
        return {"$100": "402.000"}

    def status(self):
        return "Idle", {"X": 0.0, "Y": 0.0, "Z": self.z}

    def move_axes(self, dists, feed):
        self.z += dists.get("Z", 0.0)

    def send(self, line, timeout=5.0):
        self.sent.append(line)
        return [f"[VER:{BENCH['nozzle_servo']['firmware']}:]"] if line == "$I" else []


class SimCal(nc.NozzleCal, tdr.SimDryRun):
    """The calibration jobs with a simulated head: the tip marks the paper when it goes down with the pump off,
    picks the part under it with the pump on, and turns what it holds with the servo."""

    def __init__(self, board, world, tmp_path) -> None:
        tdr.SimDryRun.__init__(self, board, world, tmp_path)
        self.grbl = FakeGrbl()
        cfg = dict(BENCH)
        cfg["nozzle"] = {**BENCH["nozzle"], "offset_guess_mm": [11.0, -2.4], "od_mm": OD, "part_height_mm": PART_H}
        self.setup(cfg)
        self.marker_px_mm = self.home_px_mm = MARKER
        self.pulse = 1500.0
        self.held = None
        self.done = False
        self.z_real, self.alpha, self.rng = 0.0, 1.0, np.random.default_rng(3)
        self.slide = 0.0                     # the head is this much higher (+) than at the remembered Z0 when Grbl says 0
        self.z_ok = True                     # the camera height was checked (tests of the restore set it False)

    def marker_scale(self, name):
        """What the camera sees of the HOME label: F_CAM / height, with 0.02 % noise."""
        return F_CAM / (H0 + self.alpha * self.z_real) * (1 + self.rng.normal(0, 2e-4))

    def tip(self):
        return self.pos + SHAFT + rot(angle(self.pulse)) @ RUNOUT

    def z_to(self, z, by_hand=False):
        before = self.grbl.z
        super().z_to(z, by_hand)
        phys = self.grbl.z + self.slide            # the head's real height, from the remembered Z0
        if self.grbl.z < before:
            self.z_real = phys                     # going down: the head follows the command
        elif self.grbl.z > before:
            self.z_real = phys - Z_PLAY            # going up: it lags by the play
        if self.pump_on and self.held is None and phys <= TOUCH_PART - 0.1:
            near = [s for s in self.world.spots if s[3] == tdr.BLACK and np.hypot(*(s[0] - self.tip())) < 0.8]
            if near:
                s = near[0]
                self.world.spots = [x for x in self.world.spots if x is not s]     # by identity (numpy inside)
                axis, phi = self.pos + SHAFT, angle(self.pulse)
                # strip parts lie with their long side along the slot's v axis (machine -X): 180 deg
                self.held = (rot(-phi) @ (s[0] - axis), 180.0 - phi, s[1], s[2])
        elif not self.pump_on and self.held is None and phys <= TOUCH_PAPER - 0.1:
            self.world.marks.append((self.tip(), OD / 2))

    def pump(self, on):
        super().pump(on)
        if not on and self.held is not None and self.grbl.z + self.slide <= TOUCH_PAPER + PART_H - 0.1:
            rel, rel_ang, length, width = self.held
            phi = angle(self.pulse)
            self.world.placed.append((self.pos + SHAFT + rot(phi) @ rel, length, width, phi + rel_ang))
            self.held = None


def sim(board, tmp_path, parts=0, aa=1):
    world = tdr.World(aa=aa).strip([(19.0 + 4.0 * k, -0.5, 2.2, 1.4, tdr.BLACK) for k in range(parts)])
    world.paper = [(PAPER + (-6, 6), PAPER + (24, -12)), (TAPE + (-6, 6), TAPE + (24, -12))]
    m = SimCal(board, world, tmp_path)
    m.cal = {"paper_xy_from_home": PAPER.tolist(), "tape_xy_from_home": TAPE.tolist(),
             "z_reference": {"label": "slot_3", "px_mm": F_CAM / H0, "f_mm": None, "id": REF,
                             "z_steps_per_mm": "402.000", "needle_od_mm": OD, "camera_rectify": None},
             "touch_paper": {"z": TOUCH_PAPER, "ref": REF},
             "touch_part": {"z": TOUCH_PART, "ref": REF}}
    return m


# ---------------------------------------------------------------- maths

def test_find_mark_takes_the_nearest_mark_of_the_needle_size():
    w = tdr.World()
    w.paper = [(PAPER + (-20, 20), PAPER + (20, -20))]
    w.marks = [(PAPER + (0.3, -0.2), OD / 2), (PAPER + (4.0, 0.0), OD / 2), (PAPER + (-1.0, 3.0), 0.05)]  # + dirt
    img = w.picture(PAPER)
    m = nc.find_mark(img, tdr.J_TRUE, PAPER, tdr.CENTRE, PAPER, 2.0, OD, 0.25)
    assert np.hypot(*(m - (PAPER + (0.3, -0.2)))) < 0.03
    assert nc.find_mark(img, tdr.J_TRUE, PAPER, tdr.CENTRE, PAPER + (2, -2), 1.0, OD, 0.25) is None


def test_find_mark_finds_the_centre_of_a_putty_ring():
    img = np.full(tdr.FRAME, 180, np.uint8)
    cv2.circle(img, (300, 210), 4, 70, 2)                 # a ring, not a filled spot
    m = nc.find_mark(img, tdr.J_TRUE, np.zeros(2), tdr.CENTRE, np.zeros(2), 10.0, OD, 0.25)
    want = np.linalg.solve(tdr.J_TRUE, tdr.CENTRE - (300, 210))
    assert np.hypot(*(m - want)) < 0.03


def test_fit_circle_and_offset_for():
    pts = [SHAFT + rot(a) @ RUNOUT for a in (-70, -30, 0, 40, 75)]
    c, r = nc.fit_circle(pts)
    assert np.allclose(c, SHAFT, atol=1e-9) and r == pytest.approx(0.3)
    table = {900: [0.0, 0.0], 1500: [1.0, 2.0], 2100: [2.0, 0.0]}
    assert np.allclose(nc.offset_for(1200, table), [0.5, 1.0]) and np.allclose(nc.offset_for(3000, table), [2, 0])


def test_unwrap_and_fit_the_servo_table():
    pulses = [1500, 700, 1100, 1900, 2300]
    true = [(p - 1500) * 0.128 for p in pulses]
    raw = [((a + 90) % 180) - 90 for a in true]            # what the camera sees: the outline, mod 180 deg
    turns = nc.unwrap_turns(pulses, raw, 1500)
    assert turns == pytest.approx(true)
    fit = nc.fit_table(pulses, turns, 1500)
    assert fit["deg_per_us"] == pytest.approx(0.128) and fit["worst_deg"] < 1e-6


def test_height_check():
    assert nc.height_ok(7.40, 7.40, 0.006) and nc.height_ok(7.43, 7.40, 0.006)
    assert not nc.height_ok(7.50, 7.40, 0.006) and not nc.height_ok(None, 7.4, 0.006)


# ---------------------------------------------------------------- jobs on the simulated head

def test_offset_job_finds_the_tip_at_every_servo_position_the_shaft_and_the_runout(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.job_offset()
    assert m.done, m.messages
    off = m.cal["offset"]
    for p, o in off["by_pulse_us"].items():
        assert np.hypot(*(np.array(o) - (SHAFT + rot(angle(p)) @ RUNOUT))) < 0.03, p
    assert np.hypot(*(np.array(off["shaft_xy"]) - SHAFT)) < 0.05
    assert off["runout_mm"] == pytest.approx(0.3, abs=0.05) and off["repeat_mm"] < 0.02
    assert m.grbl.z == 0 and len(m.world.marks) == len(BENCH["nozzle"]["mark_pulses_us"])
    assert yaml.safe_load(nc.CAL_FILE.read_text())["offset"] == off


def test_servo_job_measures_the_pulse_to_angle_table(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path, parts=6, aa=2)      # grey edges like a real lens: angles to ~0.5 deg
    m.cal["offset"] = {"by_pulse_us": {p: (SHAFT + rot(angle(p)) @ RUNOUT).tolist() for p in range(500, 2501, 100)}}
    m.job_servo()
    assert m.done, m.messages
    servo = m.cal["servo"]
    assert len(servo["pulse_to_turn_deg"]) == len(BENCH["nozzle"]["table_pulses_us"])
    for p, turn in servo["pulse_to_turn_deg"]:
        assert turn == pytest.approx(angle(p) - angle(1500), abs=1.0), p
    assert servo["fit"]["deg_per_us"] == pytest.approx(0.128, rel=0.02)
    assert max(servo["landing_error_mm"]) < 0.2               # the offset table puts parts where they should go
    assert m.grbl.z == 0 and not m.pump_on and m.held is None


def test_no_z_down_before_the_camera_height_is_checked(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.z_ok = False                                           # a new session: h has not run yet
    m.job_offset()
    assert not m.done and "press h" in m.messages[-1]
    assert m.grbl.z == 0 and not m.world.marks


def test_no_xy_jog_with_the_nozzle_down(tdr_board, tmp_path):
    """X / Y only with the tip above the HIGHEST surface it knows (the part tops are above the paper): by
    nozzle.jog_clearance_mm for the owner's jogs, nozzle.move_clearance_mm for every other move."""
    m = sim(tdr_board, tmp_path)
    jog = float(m.ncfg["jog_clearance_mm"])
    m.z = TOUCH_PAPER + 0.5
    assert not m.nozzle_up(jog)
    m.z = TOUCH_PAPER + jog + 0.5               # clear of the paper, but not of the part tops
    assert not m.nozzle_up(jog)
    m.z = TOUCH_PART + jog + 0.5
    assert m.nozzle_up(jog) and not m.nozzle_up()
    with pytest.raises(nc.GrblError, match="X / Y refused"):
        m.move([5.0, 0.0])                      # not a jog: needs move_clearance_mm


@pytest.fixture(scope="module")
def tdr_board():
    if not tdr.d.BOARD_JS.exists():
        pytest.skip("no board data")
    return tdr.d.load_board()


@pytest.mark.parametrize("deg", [0, 17, 33, 90, 147])
def test_part_angle_is_read_to_under_a_degree(deg):
    """The servo table needs the part angle to ~0.5 deg; a 1206 top is only ~16 x 10 px (the outline gave 4 deg)."""
    w = tdr.World(aa=2)
    w.paper = [(TAPE + (-8, 8), TAPE + (8, -8))]
    w.placed = [(TAPE + (0.05, 0.03), 2.2, 1.4, deg)]
    zone = np.array([[-4, -4], [4, -4], [4, 4], [-4, 4]], float)
    p = tdr.d.find_slot_parts(w.picture(TAPE), tdr.J_TRUE, TAPE, tdr.CENTRE, TAPE, np.array([1.0, 0.0]),
                              np.array([0.0, 1.0]), zone, BENCH["slot_parts"], 0.5)
    assert len(p) == 1 and abs(((p[0]["angle_deg"] - deg + 90) % 180) - 90) < 0.8
    assert np.hypot(*(p[0]["xy"] - (TAPE + (0.05, 0.03)))) < 0.03


# ---------------------------------------------------------------- Z with the camera

def test_z_maths():
    zs = [0, -1, -2, -3, -4]
    fit = nc.fit_z_sweep(zs, [F_CAM / (H0 + z) for z in zs])
    assert fit["f_mm"] == pytest.approx(F_CAM) and fit["h0_mm"] == pytest.approx(H0) and fit["worst_mm"] < 1e-9
    assert nc.z_drift(F_CAM, F_CAM / (H0 - 4), F_CAM / H0) == pytest.approx(-4.0)       # 4 mm lower now
    alpha, steps = nc.steps_from_block(F_CAM / (H0 - 5), F_CAM / H0, F_CAM / (H0 - 0.98 * 5), 402.0)
    assert alpha == pytest.approx(0.98) and steps == pytest.approx(402.0 / 0.98)


def test_z_sweep_finds_the_line_the_play_and_the_repeat(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.job_z()
    assert m.done, m.messages
    z = m.cal["z"]
    assert z["f_mm"] == pytest.approx(F_CAM, rel=0.01)
    assert z["camera_height_mm"] == pytest.approx(H0, abs=0.5)
    assert z["play_mm"] == pytest.approx(Z_PLAY, abs=0.03) and z["straight_within_mm"] < 0.08
    assert m.grbl.z == 0


def test_the_camera_height_is_brought_back_after_z_slid(tdr_board, tmp_path):
    """Power off overnight: Z slid 4 mm down. h drives it back up: the heights of yesterday are right again."""
    m = sim(tdr_board, tmp_path)
    m.slide, m.z_ok = -4.0, False                            # Grbl says 0, but the head is 4 mm low
    m.z_real = m.slide
    assert m.restore_camera_height(), m.messages
    assert m.grbl.z + m.slide == pytest.approx(0.0, abs=0.08)      # the head is back at the remembered Z0
    assert m.z_now() == pytest.approx(0.0, abs=1e-9) and m.z_ok
    assert m.cal["z_reference"]["f_mm"] == pytest.approx(F_CAM, rel=0.05)
    assert m.touch_z("paper") == TOUCH_PAPER                 # used as they were set


def test_the_camera_height_is_not_driven_down_far_by_itself(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.slide, m.z_ok = 2.0, False                             # 2 mm too high: Z does not slide up, something else
    m.z_real = m.slide
    assert not m.restore_camera_height()
    assert "HIGHER" in m.messages[-1] and not m.z_ok and m.grbl.z <= 1.0 + 1e-9


def test_the_camera_height_is_set_only_with_0_twice(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    del m.cal["z_reference"]
    m.z_ok = False
    assert not m.restore_camera_height() and "press 0 twice" in m.messages[-1]     # never by itself
    m.job_new_reference()
    assert "Press 0 again" in m.messages[-1] and "z_reference" not in m.cal
    m.job_new_reference()
    assert m.z_ok and m.cal["z_reference"]["label"] == "slot_3"
    assert m.cal["z_reference"]["z_steps_per_mm"] == "402.000" and m.cal["z_reference"]["needle_od_mm"] == OD
    assert m.touch_z("paper", quiet=True) is None             # the old heights belong to the old camera height


def test_a_changed_set_up_is_refused_not_converted(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.z_ok = False
    m.ncfg = {**m.ncfg, "od_mm": 0.72}                        # another needle
    assert not m.restore_camera_height() and "needle_od_mm" in m.messages[-1]


def test_teaching_rules(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.grbl.z = -9.0
    m.job_touch("place")                                      # no part held
    assert "touch_place" not in m.cal and "pump on" in m.messages[-1]
    m.grbl.z = -2.0
    m.job_touch("part")                                       # too close under Z0
    assert m.cal["touch_part"]["z"] == TOUCH_PART and "Z0 is too low" in m.messages[-1]
    m.grbl.z = 1.0
    m.job_touch("move")                                       # above Z0
    assert "touch_move" not in m.cal


def test_after_an_unclean_end_only_lifting_by_hand_works(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.z_ok = False                                           # a new session: nothing checked yet
    m.z_locked = "The last session did not end cleanly"
    with pytest.raises(nc.GrblError, match="did not end cleanly"):
        m.move([5.0, 0.0])
    with pytest.raises(nc.GrblError):
        m.z_to(-1.0, by_hand=True)                           # down: never
    m.job_jog_z(1.0)                                         # up by hand: yes
    assert m.grbl.z == pytest.approx(1.0)
    m.find_home = lambda: False                              # (the label search is not simulated here)
    m.job_start()                                            # h: the owner says it is clear
    lift = float(m.zcfg["max_drift_mm"])
    assert m.z_locked is None and m.z_now() == pytest.approx(lift)        # lifted before any X / Y move
    assert m.grbl.z == pytest.approx(1.0 + lift)


def test_a_grbl_reset_stops_every_z_move(tdr_board, tmp_path):
    import bench
    g = object.__new__(bench.Grbl)
    g.ser = type("S", (), {"readline": staticmethod(lambda: b"Grbl 1.1h ['$' for help]\r\n")})()
    g.armed, g.reset_seen = True, False
    with pytest.raises(bench.GrblError, match="reset"):
        g._readline(nc.time.monotonic() + 1)
    assert g.reset_seen
    m = sim(tdr_board, tmp_path)
    m.grbl.reset_seen = True
    with pytest.raises(nc.GrblError, match="reset"):
        m.z_to(-1.0)


def test_block_check_finds_a_z_steps_per_mm_error(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.alpha = 0.98                                           # 1 mm of command moves 0.98 mm
    m.zcfg = {**m.zcfg, "block_height_mm": 4.0}              # paper touch at Z -10: a 4 mm block stays clear
    m.markers = type("Labels", (), {"size_mm": 10.0, "names": {49: "home", 42: "slot_3"},
                                    "detector": type("D", (), {"detectMarkers": staticmethod(
                                        lambda img: ((np.zeros((1, 4, 2)), np.zeros((1, 4, 2))),
                                                     np.array([[30], [42]]), None))})()})()
    top = {30: 4.0, 42: 0.0}                                 # the block's label is 4 mm above the bed label
    m.scale_of_id = lambda i: (F_CAM / (H0 - top[i] + m.alpha * m.z_real) * (1 + m.rng.normal(0, 5e-5)),
                               tdr.CENTRE.copy())
    m.done = False
    m.job_block()
    assert m.done, m.messages
    check = m.cal["z"]["steps_check"]
    assert check["real_mm_per_command_mm"] == pytest.approx(0.98, abs=0.003)
    assert check["spread_between_repeats"] < 0.003
    assert check["steps_per_mm_exact"] == pytest.approx(402 / 0.98, rel=0.003)
    assert m.grbl.z == 0
    assert not any(line.startswith("$100=") for line in m.grbl.sent)   # never changed without the owner


def test_block_check_refuses_when_the_nozzle_hangs_too_low(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.zcfg = {**m.zcfg, "block_height_mm": 9.5}               # paper touch at Z -10: only 0.5 mm would be left
    m.job_block()
    assert not m.done and "thinner block" in m.messages[-1] and m.grbl.z == 0


def test_n_twice_gives_the_rough_offset_without_a_ruler(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path, parts=0)
    label = tdr.SLOT_XY.copy()
    m.markers = type("M", (), {"find": staticmethod(lambda f: {"slot_3": np.zeros((4, 2)) + tdr.CENTRE})})()
    m.locate = lambda name: label
    m.job_nozzle_point()                                    # 1st n: the label is kept
    assert m.nozzle_target is not None and "NOZZLE" in m.messages[-1]
    m.pos = label - (SHAFT + (0.4, -0.3))                    # the owner jogged the tip above it, 0.5 mm off by eye
    m.job_nozzle_point()                                    # 2nd n
    assert m.cal["offset_guess_mm"] == pytest.approx((SHAFT + (0.4, -0.3)).round(1).tolist())
    m.ncfg = {**m.ncfg, "offset_guess_mm": None}             # the job starts from it and still finds the tip
    m.job_offset()
    assert m.done, m.messages


def test_no_servo_makes_three_marks_at_one_angle(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.no_servo = True
    m.job_offset()
    assert m.done, m.messages
    off = m.cal["offset"]
    assert list(off["by_pulse_us"]) == [1500] and off["runout_mm"] == 0.0 and off["repeat_mm"] < 0.02
    assert np.hypot(*(np.array(off["by_pulse_us"][1500]) - (SHAFT + rot(angle(1500)) @ RUNOUT))) < 0.03
    assert not any(c.startswith("M3") for c in m.grbl.sent) and len(m.world.marks) == 3


# ---------------------------------------------------------------- the real pick and place (key r)

def tip_offset(pulse):
    """True nozzle tip = camera centre + this, at a servo pulse."""
    return SHAFT + rot(angle(pulse)) @ RUNOUT


def real_sim(board, tmp_path, parts=4, offsets=True, place_z=TOUCH_PAPER + PART_H - 0.2, move_z=-4.0, aa=1):
    m = sim(board, tmp_path, parts=parts, aa=aa)
    m.M, m.c = np.eye(2), m.nominal_T()                       # the board lies where the plan puts it
    m.cal["touch_place"] = {"z": place_z, "ref": REF}
    if move_z is not None:
        m.cal["touch_move"] = {"z": move_z, "ref": REF}
    if offsets:
        m.cal["offset_pick"] = {"xy": tip_offset(1500).tolist(), "ref": REF}
        m.cal["offset_place"] = {"xy": tip_offset(1500).tolist(), "ref": REF}
        m.cal["offset"] = {"by_pulse_us": {p: tip_offset(p).tolist() for p in (700, 1100, 1500, 1900, 2300)},
                           "ref": REF}
    m.cal["servo"] = {"fit": {"deg_per_us": 0.128, "offset_deg": 0.0, "worst_deg": 0.2}}
    return m


def test_real_places_one_part_per_press_on_its_pads(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path, aa=3)                    # grey edges like a real lens: angles to ~0.5 deg
    for k in range(3):
        m.job_real()
        assert len(m.world.placed) == k + 1, m.messages
        assert not m.pump_on and m.z == 0 and m.held is None
    for (centre, length, width, ang), part in zip(m.world.placed, m.todo):
        target = m.board_to_machine([part["x"], part["y"]])
        assert np.hypot(*(centre - target)) < 0.1, (part["ref"], centre - target)
        c = [m.board_to_machine(np.array(p["p"][0]).reshape(-1, 2)).mean(axis=0) for p in part["pads"]]
        pads = np.degrees(np.arctan2(*(c[1] - c[0])[::-1]))
        assert abs((ang - pads + 90) % 180 - 90) < 1.5, (part["ref"], ang, pads)   # long side on the pads
    assert [p["ref"] for p in m.report["placements"]] == [p["ref"] for p in m.todo[:3]]
    assert (tmp_path / f"after_{m.todo[0]['ref']}.png").exists()


def test_real_refuses_without_the_heights_or_with_a_low_moving_height(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path)
    del m.cal["touch_place"]
    m.job_real()
    assert "place height first (d)" in m.messages[-1] and not m.world.placed and m.grbl.z == 0
    m = real_sim(tdr_board, tmp_path, move_z=TOUCH_PART + 0.5)
    m.job_real()
    assert "moving height" in m.messages[-1] and not m.world.placed and m.grbl.z == 0
    m = real_sim(tdr_board, tmp_path, offsets=False)
    m.cal.pop("offset_guess_mm", None)
    m.ncfg = {**m.ncfg, "offset_guess_mm": None}
    m.job_real()
    assert "No pick offset" in m.messages[-1] and not m.world.placed


def test_real_notices_a_missed_pick_and_keeps_the_place(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path)
    m.cal["offset_pick"] = {"xy": (tip_offset(1500) + (1.5, 0.0)).tolist(), "ref": REF}   # 1.5 mm off
    m.job_real()
    assert any("still in its pocket" in x for x in m.messages), m.messages
    assert not m.pump_on and not m.world.placed and m.next_k == 0          # the place waits for the next part
    m.cal["offset_pick"] = {"xy": tip_offset(1500).tolist(), "ref": REF}        # offset measured again
    m.job_real()
    assert len(m.world.placed) == 1 and m.report["placements"][0]["ref"] == m.todo[0]["ref"]


def test_moving_height_must_clear_the_pick_and_place_heights(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path, move_z=None)
    m.grbl.z = TOUCH_PART + 0.5
    m.job_touch("move")
    assert "touch_move" not in m.cal and "not kept" in m.messages[-1]
    m.grbl.z = -4.0
    m.job_touch("move")
    assert m.cal["touch_move"]["z"] == -4.0 and m.cal["touch_move"]["ref"] == REF


def test_n_on_a_part_gives_the_pick_offset(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path, parts=3)
    part = m.world.spots[1][0]
    m.move_to(part + (1.0, -0.8))                              # the owner put the part near the red cross
    m.job_nozzle_point()
    assert m.nozzle_target is not None and m.nozzle_target[0] == "part", m.messages
    assert np.hypot(*(m.nozzle_target[2] - part)) < 0.05
    m.pos = part - tip_offset(1500) + (0.2, -0.1)              # tip jogged above it, 0.2 mm off by eye
    m.job_nozzle_point()
    assert np.allclose(m.cal["offset_pick"]["xy"], tip_offset(1500) - (0.2, -0.1), atol=0.06)


def test_part_axis_uses_both_slot_axes():
    frame = (np.zeros(2), np.array([0.0, -1.0]), np.array([-1.0, 0.0]))     # v is u turned -90 deg (a real label)
    assert nc.d.part_axis_deg(90.0, frame) % 360 == pytest.approx(180.0)     # long side along v: machine -X
    assert nc.d.part_axis_deg(80.0, frame) % 360 == pytest.approx(190.0)     # 10 deg back towards u (-Y)


def record_xy_moves(m):
    """Remember the Z and the pump at every X / Y move of the simulated head."""
    m.xy_moves = []
    inner = m.move

    def move(dist):
        if np.hypot(*np.asarray(dist, float)) > 1e-9:
            m.xy_moves.append((m.z, m.pump_on))
        inner(dist)
    m.move = move


def test_every_r_press_lifts_first_and_moves_only_at_z0_or_the_moving_height(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path)
    record_xy_moves(m)
    m.job_real()
    m.grbl.z = -8.5                                   # the owner jogged the nozzle down between two presses
    m.job_real()
    assert len(m.world.placed) == 2
    assert {round(z, 6) for z, _ in m.xy_moves} <= {0.0, -4.0}, sorted({z for z, _ in m.xy_moves})
    assert all(z == -4.0 for z, pump in m.xy_moves if pump and z != 0.0)


def test_a_stop_while_carrying_keeps_the_part_and_the_place(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path)
    m.job_real()                                      # one part placed
    inner = m.at_place

    def stop_in_the_carry(*a):
        raise nc.t.Stopped()
    m.at_place = stop_in_the_carry
    with pytest.raises(nc.t.Stopped):
        m.job_real()
    assert m.pump_on and m.part_on_nozzle and m.z >= 0.0           # held, lifted, not dropped
    assert m.next_k == 1 and m.report["interrupted"] == [m.todo[1]["ref"]]
    m.at_place = inner
    m.job_real()
    assert "part may still be on the nozzle" in m.messages[-1] and len(m.world.placed) == 1
    m.job_pump()                                      # a: the owner lets it go over the tape
    m.held = None
    m.job_real()
    assert len(m.world.placed) == 2 and m.report["placements"][1]["ref"] == m.todo[1]["ref"]


def test_z0_must_clear_the_pick_and_place_heights(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path, place_z=-0.85, move_z=5.0)
    m.cal["touch_part"] = {"z": -0.6, "ref": REF}
    m.job_real()
    assert "Z0, the camera height" in m.messages[-1] and m.grbl.z == 0 and not m.pump_on


def test_r_goes_on_after_the_slot_is_refilled(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path, parts=2)
    m.job_real()
    m.job_real()
    m.job_real()
    assert any("slot_3 is empty" in x for x in m.messages) and len(m.world.placed) == 2
    m.world.spots = [(m.world.slot_xy(19.0 + 4.0 * k, -0.5), 2.2, 1.4, tdr.BLACK) for k in range(2)]   # refilled
    m.job_real()
    assert len(m.world.placed) == 3
    assert [p["ref"] for p in m.report["placements"]] == [p["ref"] for p in m.todo[:3]]
    assert not any(x["ref"] == m.todo[2]["ref"] for x in m.report.get("not_placed", []))


def test_offsets_of_another_camera_height_are_refused(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path)
    m.cal["offset_place"]["ref"] = "an older camera height"
    m.job_real()
    assert "another camera height" in m.messages[-1] and not m.world.placed


def test_the_status_line_survives_a_block_check_without_a_sweep(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.cal["z"] = {"steps_check": {"real_mm_per_command_mm": 0.98}}       # b ran, z never did
    assert "pick" in m.status_extra()
    assert m.touch_z("paper", quiet=True) == pytest.approx(TOUCH_PAPER)


def test_the_servo_table_finds_a_reversed_servo():
    pulses = [1500.0, 700.0, 1100.0, 1900.0, 2300.0]
    true = [-0.128 * (p - 1500.0) for p in pulses]                       # longer pulse = clockwise
    raw = [(a + 90.0) % 180.0 - 90.0 for a in true]                       # what the camera sees (two pads)
    fits = [nc.fit_table(pulses, nc.unwrap_turns(pulses, raw, 1500.0, s), 1500.0) for s in (1.0, -1.0)]
    best = min(fits, key=lambda f: f["worst_deg"])
    assert best["deg_per_us"] == pytest.approx(-0.128, abs=1e-4) and best["worst_deg"] < 1e-6
    assert fits[0]["worst_deg"] > 10                                      # the anticlockwise guess does not fit


# ---------------------------------------------------------------- round 2: keys, start, quit, resets

def test_every_key_starts_its_job():
    class M:
        def __init__(self):
            self.jobs, self.new_ref_armed, self.said = [], True, []
            self.stop_flag = type("F", (), {"set": lambda s: self.jobs.append("STOP")})()

        def submit(self, fn, *args):
            self.jobs.append((fn.__name__, args))

        def say(self, text):
            self.said.append(text)

        def __getattr__(self, name):                        # job_* stand-ins
            if name.startswith("job_"):
                f = lambda *a: None
                f.__name__ = name
                return f
            raise AttributeError(name)
    m, step = M(), 5.0
    want = {"h": "job_start", "a": "job_pump", "c": "job_centre", "r": "job_real", "n": "job_nozzle_point",
            "o": "job_offset", "v": "job_servo", "z": "job_z", "b": "job_block", "0": "job_new_reference",
            "k": "job_touch", "d": "job_touch", "m": "job_touch", "t": "job_touch", "p": "job_set_spot",
            "l": "job_set_spot", "]": "job_jog_z", "}": "job_jog_z", "[": "job_jog_z", "{": "job_jog_z"}
    for key, job in want.items():
        m.jobs.clear()
        stop, step = nc.handle_key(m, ord(key), step)
        assert not stop and m.jobs and m.jobs[0][0] == job, (key, m.jobs)
    assert nc.handle_key(m, ord("q"), step)[0] and nc.handle_key(m, 27, step)[0]
    assert nc.handle_key(m, -1, step) == (False, step)
    m.new_ref_armed = True
    nc.handle_key(m, ord("k"), step)
    assert not m.new_ref_armed                               # 0 must be pressed twice IN A ROW


def physical_xy_moves(m):
    m.phys_moves = []
    inner = m.move

    def move(dist):
        if np.hypot(*np.asarray(dist, float)) > 1e-9:
            m.phys_moves.append((m.grbl.z + m.slide, m.z_ok))
        inner(dist)
    m.move = move


def test_h_lifts_before_any_xy_move_after_z_slid_overnight(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.slide, m.z_ok = -5.0, False                            # 5 mm low after the night
    m.z_real = m.slide
    m.find_home = lambda: True                               # (HOME is known in this simulation)
    m.measure_home_scale = lambda: None
    physical_xy_moves(m)
    m.job_start()
    assert m.z_ok and abs(m.grbl.z + m.slide) < 0.08, m.messages
    assert all(z >= -0.1 for z, _ in m.phys_moves), m.phys_moves      # never below Z0 while moving


def test_the_owners_jog_is_kept_when_h_is_pressed_again(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.slide, m.z_ok = 2.0, False                             # the camera is 2 mm too HIGH: the restore refuses
    m.z_real = m.slide
    m.find_home = lambda: True
    m.measure_home_scale = lambda: None
    m.job_start()
    assert not m.z_ok and "HIGHER" in m.messages[-1]
    m.job_jog_z(-6.0 - 2.0)                                  # the owner jogs down as told (after h's 6 mm lift)
    m.job_start()
    assert m.z_ok and abs(m.grbl.z + m.slide) < 0.08, m.messages


def test_quit_never_drives_down_to_an_unchecked_height(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.z_ok = False
    m.grbl.z = 4.0                                           # a failed restore left the head 4 mm up
    assert m.park() is False and m.grbl.z == 4.0             # stays up, next start locked
    m.grbl.z = -3.0
    assert m.park() is False and m.grbl.z == pytest.approx(0.0)      # only up
    m.started_parked = True                                  # a session that started parked, back at its start height
    m.grbl.z = -4.1
    assert m.park() is True and m.grbl.z == pytest.approx(0.0)
    m.z_ok = True
    m.grbl.z = -2.0
    assert m.park() is True and m.grbl.z == pytest.approx(0.0)


def test_after_a_reset_nothing_is_kept_and_h_forgets_the_old_frame(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.home = np.array([10.0, 10.0])
    m.grbl.reset_seen = True
    m.grbl.z = -8.0
    m.job_touch("part")
    assert "nothing is kept" in m.messages[-1] and m.cal["touch_part"]["z"] == TOUCH_PART and not m.z_ok
    m.find_home = lambda: False
    m.job_start()                                            # 1st h: locked, the owner must lift by hand first
    assert m.z_locked and "Lift the nozzle by hand" in m.messages[-1] and m.home is not None
    m.job_jog_z(3.0)                                         # } } }: up by hand works while locked
    m.job_start()                                            # 2nd h: goes on, the old Grbl frame is forgotten
    assert m.home is None and m.z_locked is None and not m.grbl.reset_seen


def test_the_z_sweep_needs_the_checked_camera_height(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.z_ok = False
    m.job_z()
    assert "press h" in m.messages[-1] and m.grbl.z == 0


def test_a_stop_after_the_release_does_not_place_twice(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path)
    m.job_real()
    inner = m.frame
    calls = {"n": 0}

    def camera_drops_after_the_release():
        if m.report["placements"] and len(m.report["placements"]) == 2:
            raise TimeoutError("camera gives no frames")
        return inner()
    m.frame = camera_drops_after_the_release
    with pytest.raises(TimeoutError):
        m.job_real()
    m.frame = inner
    assert [p["ref"] for p in m.report["placements"]] == [p["ref"] for p in m.todo[:2]]
    m.job_real()
    assert [p["ref"] for p in m.report["placements"]] == [p["ref"] for p in m.todo[:3]]   # C3rd, not the 2nd again
    assert len(m.world.placed) == 3



def test_a_reset_while_working_locks_at_once_and_h_moves_nothing_before_a_hand_lift(tdr_board, tmp_path):
    m = real_sim(tdr_board, tmp_path)
    m.job_real()                                             # one part placed, z_ok True
    m.grbl.reset_seen = True                                 # a power dip during the last Z move
    import threading
    m.stop_flag, m.busy = threading.Event(), False           # the real submit wrapper, run in this thread
    m.jobs = type("Q", (), {"put": staticmethod(lambda job: job[0](*job[1]))})()
    nc.NozzleCal.submit(m, lambda: None)                     # any job that ends with the reset seen
    assert m.z_locked and not m.z_ok and "Lift the nozzle by hand" in m.messages[-1]
    with pytest.raises(nc.GrblError):
        m.move([5.0, 0.0])                                   # locked: no X / Y move by itself
    z_before = m.grbl.z
    zs = []
    inner = m.grbl.move_axes
    m.grbl.move_axes = lambda d, f: (zs.append(d.get("Z", 0.0)), inner(d, f))
    m.find_home = lambda: False
    m.job_start()                                            # h (the owner's "it is clear"): Z only goes UP first
    assert zs and zs[0] > 0 and m.grbl.z >= z_before and m.z_locked is None and m.home is None


def test_h_does_not_lift_again_and_again(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    m.z_ok = False
    m.find_home = lambda: False                              # h fails every time (for example: reach not measured)
    for _ in range(4):
        m.job_start()
    assert m.grbl.z == pytest.approx(float(m.zcfg["max_drift_mm"]))       # one lift, not four


def test_the_first_n_needs_the_checked_camera_height(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path, parts=3)
    m.z_ok = False
    m.grbl.z = 6.0                                           # h lifted, then its restore failed
    m.job_nozzle_point()
    assert "press h" in m.messages[-1] and m.grbl.z == 6.0 and m.nozzle_target is None


def test_0_twice_drives_to_the_reference_label_when_it_is_not_in_view(tdr_board, tmp_path):
    m = sim(tdr_board, tmp_path)
    del m.cal["z_reference"]
    m.z_ok = False
    m.nominal = {"home": np.zeros(2), "slot_3": tdr.SLOT_XY.copy()}
    m.visible = lambda: {}                                   # slot 3 is out of the picture
    went = []
    inner = m.move_to
    m.move_to = lambda xy: (went.append(np.asarray(xy, float)), inner(xy))
    m.job_new_reference()
    m.job_new_reference()
    assert went and np.allclose(went[0], m.clip_to_reach(tdr.SLOT_XY)) and m.z_ok, m.messages
