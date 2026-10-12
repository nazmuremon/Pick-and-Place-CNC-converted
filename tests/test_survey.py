"""The bed survey of the page (ui/engine.py job_bed_survey): every label measured from wide pictures with the
camera high, moving only to the front and the left, and no further to the front than where PCB 1 / PCB 2 show."""
import sys
import threading
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ui import engine as e  # noqa: E402

FRAME = (400, 640)
CENTRE = np.array([(FRAME[1] - 1) / 2, (FRAME[0] - 1) / 2])
J_TRUE = np.array([[5.0, 0.0], [0.0, -5.0]])       # camera high (5 px/mm) and turned 180 deg, like 2026-10-08
BED = e.ml.load_bed()
# where the labels really are (mm from HOME): a few mm off the plan
TRUE = {"home": (0.0, 0.0), "slot_3": (-29.5, -1.0), "slot_4": (-59.0, 0.5), "slot_5": (-89.5, 2.0),
        "slot_6": (-120.0, 2.5), "slot_7": (-150.5, 1.5),
        "pcb_1": (-140.0, -238.0), "pcb_2": (30.0, -236.0), "pcb_3": (31.0, -82.0), "pcb_4": (-141.0, -84.0)}


class SurveySim(e.PnPMachine):
    """The page's machine with a fake head (it stops at the ends of travel) and a camera that sees markers."""

    def __init__(self, tmp_path, start=(14.0, -19.0), ends=((-150.0, 40.0), (-215.0, 0.0))):
        self.home_true = np.array([100.0, 50.0])                  # HOME in machine mm
        self.pos = self.home_true + np.array(start)
        self.ends = [np.array(ends[0]) + self.home_true[0], np.array(ends[1]) + self.home_true[1]]
        self.start = self.pos.copy()
        self.bed, self.out_dir = BED, tmp_path
        self.nominal = {n: np.array(c, float) for n, c, _ in e.ml.label_placements(BED, e.ml.make_all(BED))}
        self.markers = type("M", (), {"size_mm": 10.0})()
        self.z_locked, self.reach_mode, self.run_state = None, False, "idle"
        self.J = self.home = self.extremes = self.M = self.c = None
        self.corners, self.reach, self.reach_stale = {}, {"x": [-154, -4], "y": [-227, 3]}, False
        self.feed, self.frame_shape, self.stop_flag = 1000.0, FRAME, threading.Event()
        self.lock, self.msg, self.step_name, self.events = threading.Lock(), "", "", []
        self.messages, self.path = [], [self.pos.copy()]

    def say(self, text):
        self.messages.append(text)

    def nozzle_up(self, gap=None):
        return True

    def visible(self):
        out = {}
        for name, xy in TRUE.items():
            m = self.home_true + np.array(xy)
            corners = m + np.array([[-5, -5], [5, -5], [5, 5], [-5, 5]], float)
            px = CENTRE + (self.pos - corners) @ J_TRUE.T
            if px.min() >= 0 and px[:, 0].max() <= FRAME[1] - 1 and px[:, 1].max() <= FRAME[0] - 1:
                out[name] = px
        return out

    def look(self, name, frames=3):
        return self.visible().get(name)

    def move(self, d):
        target = self.pos + np.asarray(d, float)
        self.pos = np.array([np.clip(target[0], *self.ends[0]), np.clip(target[1], *self.ends[1])])  # blocked at an end
        self.path.append(self.pos.copy())
        if self.home is not None:
            x, y = self.pos - self.home
            ex = self.extremes
            self.extremes = np.array([x, x, y, y]) if ex is None else np.array(
                [min(ex[0], x), max(ex[1], x), min(ex[2], y), max(ex[3], y)])


@pytest.fixture
def files(tmp_path, monkeypatch):
    bed = tmp_path / "bed.yaml"
    bed.write_text((ROOT / "config" / "bed.yaml").read_text())
    monkeypatch.setattr(e, "BED_FILE", bed)
    monkeypatch.setattr(e.t, "CAMERA_FILE", tmp_path / "camera_calibration.yaml")
    saved = {}
    monkeypatch.setattr(e.t, "save_reach", lambda ext: saved.setdefault("reach", np.array(ext)))
    return bed, saved


def test_the_survey_measures_every_label_and_the_board_area(tmp_path, files):
    bed_file, saved = files
    m = SurveySim(tmp_path)
    m.job_bed_survey()
    data = yaml.safe_load((tmp_path / "survey.yaml").read_text())
    got = {r["label"]: r["measured_mm"] for r in data["labels"]}
    assert got["pcb_1"] is None and data["pcb_estimated_from_the_other_3"] == "pcb_1"   # 2 mm further front
    for name, xy in TRUE.items():
        if name != "pcb_1":
            assert got[name] == pytest.approx(list(xy), abs=0.05), name
    path = np.array(m.path) - m.start
    assert path[:, 0].max() <= 1e-9 and path[:, 1].max() <= 1e-9   # never right of / behind the start
    assert np.allclose(m.pos, m.start)                             # back where it started
    front = path[:, 1].min() + m.start[1] - m.home_true[1]
    assert front > -215 + 3                                        # stopped before the end of travel
    bed = yaml.safe_load(bed_file.read_text())
    lay = bed["layout"]
    assert lay["slot_marker_xy"][3] == pytest.approx([-29.5, -1.0], abs=0.06)
    assert lay["pcb_corner_marker_xy"][1] == pytest.approx([30.0, -236.0], abs=0.06)
    assert lay["pcb_corner_marker_xy"][0] == pytest.approx([30 - 141 - 31, -236 - 84 + 82], abs=0.06)  # 2 + 4 - 3
    p1 = np.array([-142.0, -238.0])
    cx, cy = (p1[0] - 141 + 30 + 31) / 4, (p1[1] - 236 - 82 - 84) / 4  # the board centred between the labels
    assert lay["pcb_area_xy"] == pytest.approx([cx - 65, cy - 70], abs=0.06)
    assert "# PCB 1-4 marker centres, bed survey" in bed_file.read_text()   # comments kept
    assert saved["reach"][2] == pytest.approx(front, abs=1e-6)      # the reach driven = the saved reach
    assert "Bed survey saved" in m.messages[-1]


def test_the_front_labels_are_approached_in_1_mm_steps(tmp_path, files):
    m = SurveySim(tmp_path)
    m.job_bed_survey()
    ys = [p[1] for p in m.path]
    steps = np.abs(np.diff(ys))
    i_front = int(np.argmin(ys))
    assert steps[i_front - 1] <= 1.0 + 1e-9 and steps[i_front - 5] <= 1.0 + 1e-9


def test_a_head_blocked_at_an_end_stops_the_survey(tmp_path, files):
    bed_file, saved = files
    before = bed_file.read_text()
    m = SurveySim(tmp_path, ends=((-150.0, 40.0), (-60.0, 0.0)))   # the front end comes long before the labels
    with pytest.raises(e.t.Stalled):
        m.job_bed_survey()
    assert bed_file.read_text() == before and "reach" not in saved


def test_survey_needs_home_in_the_picture(tmp_path, files):
    m = SurveySim(tmp_path, start=(-60.0, -150.0))
    m.job_bed_survey()
    assert "HOME must be in the picture" in m.messages[-1] and len(m.path) == 1


def test_pcb_area_and_layout_text():
    pcb = {"pcb_1": np.array([-140.0, -240.0]), "pcb_2": np.array([30.0, -240.0]),
           "pcb_3": np.array([30.0, -80.0]), "pcb_4": np.array([-140.0, -80.0])}
    area, info = e.pcb_area_from_labels(pcb, [130, 140], 10.0, 2.5)
    assert area == [-120.0, -230.0]
    assert info["marker_box_mm"] == [170.0, 160.0] and info["gap_x_mm"] == 12.5 and info["gap_y_mm"] == 2.5
    text = "layout:\n  slot_marker_xy:   # c\n    3: [-29.2, -1.5]\n    4: [1, 2]\n  pcb_area_xy: [-144, -224]  # x\n" \
           "  pcb_corner_marker_xy: []\n"
    out = e.layout_text(text, {3: np.array([-29.46, -1.04])}, pcb, area, "2026-10-08")
    assert "    3: [-29.5, -1.0]\n    4: [1, 2]" in out and "pcb_area_xy: [-120.0, -230.0]  # x" in out
    assert yaml.safe_load(out)["layout"]["pcb_corner_marker_xy"][2] == [30.0, -80.0]
