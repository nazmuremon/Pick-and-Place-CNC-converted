"""Tests for tools/bench.py: the machine-axis -> Grbl-axis map (no hardware needed)."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import bench  # noqa: E402


def test_bench_yaml_maps_to_right_back_up():
    """Checked on the machine 2026-10-06: Grbl +Y = head left, Grbl +Z = head back, Grbl +X = camera down."""
    axes = bench.AxisMap.from_config(bench.load_bench())
    assert axes.word("X", 1.0) == "Y-1.000"      # +X = right = Grbl -Y
    assert axes.word("Y", -2.5) == "Z-2.500"     # +Y = back = Grbl +Z
    assert axes.word("Z", 2.0) == "X-2.000"      # +Z = up = Grbl -X
    assert axes.steps_setting("Z") == "$100"
    assert axes.steps_setting("Y") == "$102"
    assert axes.steps_setting("X") == "$101"


def test_machine_pos_is_the_inverse_of_word():
    axes = bench.AxisMap.from_config(bench.load_bench())
    pos = axes.machine_pos(np.array([1.0, 2.0, 3.0]))   # Grbl X, Y, Z
    assert pos == {"X": -2.0, "Y": 3.0, "Z": -1.0}


@pytest.mark.parametrize("axis_map", [
    {"X": {"grbl": "X"}, "Y": {"grbl": "X"}, "Z": {"grbl": "Z"}},               # X used twice
    {"X": {"grbl": "X"}, "Y": {"grbl": "Y"}},                                   # Z missing
    {"X": {"grbl": "X", "sign": 2}, "Y": {"grbl": "Y"}, "Z": {"grbl": "Z"}},    # bad sign
])
def test_bad_axis_map_is_refused(axis_map):
    with pytest.raises(ValueError):
        bench.AxisMap.from_config({"axis_map": axis_map})


def test_bed_test_run_saves_the_measured_reach(tmp_path, monkeypatch):
    import shutil
    import yaml
    import bed_test_run
    (tmp_path / "config").mkdir()
    shutil.copy(ROOT / "config" / "bench.yaml", tmp_path / "config" / "bench.yaml")
    monkeypatch.setattr(bed_test_run, "ROOT", tmp_path)
    bed_test_run.save_reach(np.array([-150.04, -3.96, -225.0, 2.5]))
    text = (tmp_path / "config" / "bench.yaml").read_text()
    reach = yaml.safe_load(text)["limits"]["reach_from_home"]
    assert reach == {"x": [-150.0, -4.0], "y": [-225.0, 2.5]}
    assert "ESTIMATE" not in text and "# Measured with --measure-reach" in text
    assert yaml.safe_load(text)["axis_map"] == yaml.safe_load((ROOT / "config" / "bench.yaml").read_text())["axis_map"]


def test_grabber_reopens_the_camera_after_a_usb_drop(monkeypatch):
    """The head camera dropped off the USB while jogging (2026-10-07): frames must come back by themselves."""
    import time as _time
    import numpy as _np
    opened = []

    class FakeCap:
        def __init__(self, device, backend):
            opened.append(device)
            self.n = 0
            self.alive = len(opened) != 2                 # the 2nd open fails: the camera is still away

        def isOpened(self):
            return self.alive

        def set(self, *a):
            return True

        def read(self):
            self.n += 1
            if len(opened) == 1 and self.n > 3:          # the first connection drops after 3 frames
                return False, None
            _time.sleep(0.005)
            return True, _np.zeros((4, 4), _np.uint8)

        def release(self):
            pass

    monkeypatch.setattr(bench.cv2, "VideoCapture", FakeCap)
    g = bench.Grabber("/dev/fake")
    try:
        frames = g.frames_after(_time.monotonic() + 0.1, 3, timeout=5.0)    # after the drop
        assert len(frames) == 3 and len(opened) >= 3
    finally:
        g.close()
