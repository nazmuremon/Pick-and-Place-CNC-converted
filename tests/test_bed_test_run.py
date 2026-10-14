"""The automatic bed test (tools/bed_test_run.py --auto) against a simulated machine and camera."""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import bed_test_run as t  # noqa: E402
import make_labels as ml  # noqa: E402
from pnp_paths import CONFIG_DIR  # noqa: E402  (the frozen test copy, tests/conftest.py)

J_TRUE = np.array([[-7.45, 0.0], [0.0, 7.48]])   # raw picture seen from the front: px per mm of head move
FRAME = (400, 640)
BENCH_SEARCH = yaml.safe_load((CONFIG_DIR / "bench.yaml").read_text())["label_search"]
# The bed's look (bars, board, dirt) for the picture-shift check: 2 px per mm, from X -100 / Y -300 mm.
TEXTURE = cv2.GaussianBlur(np.random.default_rng(7).uniform(0, 255, (1000, 1000)).astype(np.float32), (0, 0), 2)


class SimMachine(t.Machine):
    """The Machine jobs with a fake head (stops dead at the ends, like the real one) and a fake camera."""

    def __init__(self, world: dict, start, ends, nominal: dict, tmp_path) -> None:
        self.world, self.ends = world, ends
        self.nominal, self.out_dir = nominal, tmp_path
        self.max_step = 10.0
        self.feed = 300.0
        self.reach = yaml.safe_load((CONFIG_DIR / "bench.yaml").read_text())["limits"]["reach_from_home"]
        self.pos = np.array(start, float)
        self.true_pos = self.pos.copy()
        self.home = self.J = self.extremes = None
        self.results, self.frame_shape, self.msg, self.auto_done = [], FRAME, "", False
        self.stop_flag = type("F", (), {"is_set": staticmethod(lambda: False)})()
        self.lock = type("L", (), {"__enter__": lambda s: None, "__exit__": lambda s, *a: None})()
        self.messages = []
        self.markers = type("Markers", (), {"size_mm": 10.0})()

    def say(self, text):
        self.messages.append(text)

    def move(self, d):
        d = np.asarray(d, float)
        if self.home is not None and self.reach is not None:
            rel = self.pos + d - self.home
            (x0, x1), (y0, y1) = self.reach["x"], self.reach["y"]
            if not (x0 <= rel[0] <= x1 and y0 <= rel[1] <= y1):
                raise t.GrblError(f"refused {rel}")
        self.pos = self.pos + d                       # what the controller believes
        (x0, x1), (y0, y1) = self.ends
        self.true_pos = np.array([np.clip(self.true_pos[0] + d[0], x0, x1),
                                  np.clip(self.true_pos[1] + d[1], y0, y1)])   # the head stops at the end

    def _picture(self):
        centre = np.array([(FRAME[1] - 1) / 2, (FRAME[0] - 1) / 2])
        out = {}
        for name, xy in self.world.items():
            c = centre - J_TRUE @ (np.asarray(xy) - self.true_pos)
            if abs(c[0] - centre[0]) < 280 and abs(c[1] - centre[1]) < 165:
                out[name] = c + np.array([[-37, -37], [37, -37], [37, 37], [-37, 37]], float)
        return out

    def look(self, name, frames=3):
        return self._picture().get(name)

    def picture(self):
        h, w = FRAME
        v, u = np.mgrid[:h, :w]
        centre = np.array([(w - 1) / 2, (h - 1) / 2])
        mm = self.true_pos + np.linalg.solve(J_TRUE, (centre - np.c_[u.ravel(), v.ravel()]).T).T
        mx = ((mm[:, 0] + 100) * 2).reshape(h, w).astype(np.float32)
        my = ((mm[:, 1] + 300) * 2).reshape(h, w).astype(np.float32)
        return cv2.remap(TEXTURE, mx, my, cv2.INTER_LINEAR)

    def visible(self):
        return self._picture()

    def snapshot(self, name):
        pass

    def save(self, drift):
        self.saved_drift = drift


@pytest.fixture(autouse=True)
def private_state_files(tmp_path, monkeypatch):
    """The simulated machine must never write the real position / camera files in logs/."""
    monkeypatch.setattr(t, "POSITION_FILE", tmp_path / "head_position.yaml")
    monkeypatch.setattr(t, "CAMERA_FILE", tmp_path / "camera_calibration.yaml")


@pytest.fixture
def plan():
    bed = ml.load_bed()
    labels = ml.make_all(bed)
    return {n: np.array(c, float) for n, c, _ in ml.label_placements(bed, labels)}


def make_world(plan, home_true, rng):
    """Where the labels really are (machine mm): the plan + a few mm of hand-placement error."""
    return {n: home_true + (xy - plan["home"]) + (rng.normal(0, 1.5, 2) if n != "home" else 0) for n, xy in plan.items()}


def test_auto_finds_home_and_every_slot(plan, tmp_path):
    rng = np.random.default_rng(1)
    home_true = np.array([200.0, 50.0])                    # machine coordinates are arbitrary
    world = make_world(plan, home_true, rng)
    # physical ends: the measured reach + the owner's ~5 mm margin (HOME is ~4 mm outside the reach)
    ends = ((home_true[0] - 159, home_true[0] + 1), (home_true[1] - 232, home_true[1] + 8))
    start = world["slot_6"] + np.array([-9.0, 3.0])        # where the head was left: over slot 6
    m = SimMachine(world, start, ends, plan, tmp_path)
    m.job_auto()
    assert m.auto_done, m.messages
    assert np.allclose(m.home, home_true, atol=0.5)       # HOME measured without driving onto it
    found = {r["label"]: np.array(r["measured_mm"]) for r in m.results}
    for n in ("slot_3", "slot_4", "slot_5", "slot_6", "slot_7"):
        assert np.allclose(found[n], world[n] - home_true, atol=0.3), n
    assert m.saved_drift is not None and np.hypot(*m.saved_drift) < 0.3
    assert not any("STALL" in x or "refused" in x for x in m.messages)


def test_auto_stops_when_the_head_hits_an_end(plan, tmp_path):
    """If HOME is further right than the head can go, the camera notices the stall and stops."""
    rng = np.random.default_rng(2)
    home_true = np.array([200.0, 50.0])
    world = make_world(plan, home_true, rng)
    ends = ((home_true[0] - 159, home_true[0] - 40), (home_true[1] - 232, home_true[1] + 8))  # right end 40 mm short
    m = SimMachine(world, world["slot_6"] + np.array([-9.0, 3.0]), ends, plan, tmp_path)
    with pytest.raises(t.Stalled):
        m.job_auto()
    assert not m.auto_done


def test_marker_just_outside_the_reach_is_still_measured(plan, tmp_path):
    """A slot stuck 8 mm further back than planned: the camera can't centre on it, but measures it."""
    home_true = np.array([200.0, 50.0])
    world = make_world(plan, home_true, np.random.default_rng(3))
    world["slot_5"] = world["slot_5"] + np.array([0.0, 8.0])
    ends = ((home_true[0] - 159, home_true[0] + 1), (home_true[1] - 232, home_true[1] + 8))
    m = SimMachine(world, world["slot_6"] + np.array([-9.0, 3.0]), ends, plan, tmp_path)
    m.job_auto()
    assert m.auto_done, m.messages
    found = {r["label"]: np.array(r["measured_mm"]) for r in m.results}
    assert np.allclose(found["slot_5"], world["slot_5"] - home_true, atol=0.3)


def test_a_target_clipped_onto_the_reach_edge_is_accepted(tmp_path, monkeypatch):
    """Rounding: home + clip(rel) - pos + pos - home must still count as inside the reach."""
    monkeypatch.setattr(t, "POSITION_FILE", tmp_path / "head_position.yaml")   # never touch the real one
    class FakeGrbl:
        max_step = 30.0
        def __init__(self): self.at = np.zeros(2)
        def move_axes(self, d, feed): self.at = self.at + np.array([d.get("X", 0.0), d.get("Y", 0.0)])
        def status(self): return "Idle", {"X": float(self.at[0]), "Y": float(self.at[1]), "Z": 0.0}
    m = t.Machine.__new__(t.Machine)
    m.grbl, m.max_step, m.feed = FakeGrbl(), 30.0, 300.0
    m.reach = {"x": [-154, -4], "y": [-227, 3]}
    m.stop_flag = type("F", (), {"is_set": staticmethod(lambda: False)})()
    m.lock = type("L", (), {"__enter__": lambda s: None, "__exit__": lambda s, *a: None})()
    m.extremes = None
    m.home = np.array([0.1 + 0.2, -0.3])            # values that do not add up exactly in binary
    m.pos = np.array([-37.123456789, -91.7]) + m.home
    m.grbl.at = m.pos.copy()
    for rel in ([5.0, -217.0], [-160.0, 10.0], [-4.0, 3.0]):
        m.move_to(m.clip_to_reach(m.home + np.array(rel)))      # must not raise
        assert m.in_reach(m.pos - m.home)


def test_auto_searches_to_the_back_when_no_label_is_in_view(plan, tmp_path):
    """Head moved by hand to the front, far from every label (run 2026-10-07): it drives to the back
    in checked steps until a label is in view, then finds HOME and finishes, never pushing into an end."""
    (tmp_path / "camera_calibration.yaml").write_text(yaml.safe_dump({"J": J_TRUE.tolist()}))
    home_true = np.array([200.0, 50.0])
    world = make_world(plan, home_true, np.random.default_rng(4))
    ends = ((home_true[0] - 159, home_true[0] + 1), (home_true[1] - 232, home_true[1] + 8))
    m = SimMachine(world, home_true + np.array([-90.0, -182.0]), ends, plan, tmp_path)
    assert not m.visible()
    m.job_auto()
    assert m.auto_done, m.messages
    assert np.allclose(m.home, home_true, atol=0.5)
    assert np.all(m.true_pos <= np.array([ends[0][1], ends[1][1]]) - 1)    # never pushed into an end
    assert np.allclose(m.pos, m.true_pos)                                  # no lost steps


def test_search_stops_at_the_back_end_when_there_is_no_label(plan, tmp_path):
    """No label on the bed at all: the picture stops moving at the back end, so the search stops there."""
    (tmp_path / "camera_calibration.yaml").write_text(yaml.safe_dump({"J": J_TRUE.tolist()}))
    home_true = np.array([200.0, 50.0])
    ends = ((home_true[0] - 159, home_true[0] + 1), (home_true[1] - 232, home_true[1] + 8))
    m = SimMachine({}, home_true + np.array([-90.0, -182.0]), ends, plan, tmp_path)
    with pytest.raises(t.Stalled):
        m.job_auto()
    assert m.true_pos[1] == pytest.approx(ends[1][1])                # it got to the back end ...
    assert m.pos[1] - m.true_pos[1] <= float(BENCH_SEARCH["step_mm"])  # ... and pushed at most one step


def test_moved_as_commanded_tells_moved_blocked_and_cannot_tell():
    margin = float(BENCH_SEARCH["margin"])
    big = cv2.GaussianBlur(np.random.default_rng(3).uniform(0, 255, (600, 800)).astype(np.float32), (0, 0), 3)
    a = big[100:500, 80:720]
    b = big[28:428, 75:715]                                   # the same scene, moved 72 px down, 5 px right
    assert t.moved_as_commanded(a, b, (5, 72), margin) is True
    assert t.moved_as_commanded(a, a, (5, 72), margin) is False       # nothing moved: blocked
    stripes = np.tile(big[0, :640], (400, 1))                 # long bars along the move
    assert t.moved_as_commanded(stripes, stripes, (0, 72), margin) is None
    assert t.moved_as_commanded(np.full((400, 640), 150.0), np.full((400, 640), 150.0), (0, 72), margin) is None


def test_moved_as_commanded_is_no_false_stop_on_a_real_bars_only_picture():
    """Run 2026-10-07: only T-slot bars in view, the head moved to the back, the old check said "blocked"."""
    img = cv2.imread(str(ROOT / "tests" / "data" / "bars_only.png"), cv2.IMREAD_GRAYSCALE).astype(np.float32)
    a, b = img[76:400], img[0:324]                           # the scene moved 76 px down (10 mm to the back)
    assert t.moved_as_commanded(a, b, (0, 76), float(BENCH_SEARCH["margin"])) is not False


def test_search_allows_for_a_camera_turned_since_its_calibration(plan, tmp_path):
    """2026-10-07: the camera was turned 180 deg when the nozzle was mounted; the saved calibration made the
    search think the head was blocked. Now it notices the turn and still finds the labels."""
    (tmp_path / "camera_calibration.yaml").write_text(yaml.safe_dump({"J": (-J_TRUE).tolist()}))   # before the turn
    home_true = np.array([200.0, 50.0])
    world = make_world(plan, home_true, np.random.default_rng(4))
    ends = ((home_true[0] - 159, home_true[0] + 1), (home_true[1] - 232, home_true[1] + 8))
    m = SimMachine(world, home_true + np.array([-90.0, -182.0]), ends, plan, tmp_path)
    m.job_auto()
    assert m.auto_done, m.messages
    assert any("turned by" in x for x in m.messages)
    assert np.allclose(m.home, home_true, atol=0.5)


def test_the_camera_is_calibrated_on_a_slot_label_not_on_home(plan, tmp_path):
    """HOME cannot be brought to the picture centre: off-centre the calibration was rejected (2026-10-07)."""
    m = SimMachine({}, np.zeros(2), ((-500, 500), (-500, 500)), plan, tmp_path)
    corners = np.zeros((4, 2))
    seen = {"home": corners + (330, 200), "slot_3": corners + (100, 210)}
    assert m.calibration_label(seen) == "slot_3"
    assert m.calibration_label({"slot_4": corners, "slot_3": corners + (319.5, 199.5)}) == "slot_3"


def test_a_head_parked_on_the_reach_edge_can_still_jog_along_it(plan, tmp_path):
    """Grbl reports positions to 0.001 mm: a head clipped onto the edge reads a hair outside it."""
    m = SimMachine({}, (0.0, 0.0), ((-1e3, 1e3), (-1e3, 1e3)), plan, tmp_path)
    x1, y0 = m.reach["x"][1], m.reach["y"][0]
    assert t.Machine.in_reach(m, np.array([x1 + 0.0004, -5.0]))     # on the edge, read back a hair outside
    assert t.Machine.in_reach(m, np.array([-20.0, y0 - 0.003]))
    assert not t.Machine.in_reach(m, np.array([x1 + 1.0, -5.0]))    # really outside: refused


def test_a_stale_reach_stops_every_move_but_the_owners_jogs(plan, tmp_path):
    m = SimMachine({}, (0.0, 0.0), ((-1e3, 1e3), (-1e3, 1e3)), plan, tmp_path)
    m.reach_stale = True
    with pytest.raises(t.GrblError, match="measure-reach"):
        t.Machine.move(m, np.array([5.0, 0.0]))
    seen = []
    m.move = lambda d: seen.append(getattr(m, "by_hand", False))
    m.job_jog((5.0, 0.0))
    assert seen == [True] and not m.by_hand


def test_saving_the_reach_clears_the_stale_flag(tmp_path, monkeypatch):
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "bench.yaml").write_text(
        "limits:\n  reach_from_home:\n    x: [-154, -4]\n    y: [-227, 3]\n  reach_stale: true\n")
    monkeypatch.setattr(t, "BENCH_CONFIG", tmp_path / "config" / "bench.yaml")
    t.save_reach(np.array([-150.0, -6.0, -220.0, 25.0]))
    cfg = yaml.safe_load((tmp_path / "config" / "bench.yaml").read_text())["limits"]
    assert cfg["reach_from_home"] == {"x": [-150, -6], "y": [-220, 25]} and cfg["reach_stale"] is False
