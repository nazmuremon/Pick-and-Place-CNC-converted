"""tools/camera_square_check.py: a 10 mm marker seen by a straight and by a tilted pinhole camera."""
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import camera_square_check as csc  # noqa: E402
from pnp_paths import CONFIG_DIR  # noqa: E402  (the frozen test copy, tests/conftest.py)

CFG = yaml.safe_load((CONFIG_DIR / "bench.yaml").read_text())["camera_square"]
F_PX = 330.0          # focal length in px: about 8.5 px/mm at 39 mm above the bed
HEIGHT = 39.0         # mm, lens above the bed
CENTRE = np.array([319.5, 199.5])


def view(tilt_deg: float = 0.0, axis: str = "x", turn_deg: float = 0.0, size: float = 10.0, at=(0.0, 0.0)):
    """Picture corners (u, v) and their distance from the lens (mm) of a square marker on the bed,
    seen by a camera HEIGHT above it, tilted by tilt_deg about its x or y axis and turned by turn_deg.
    at = mm from the spot in the middle of the picture (where the tilted optical axis meets the bed)."""
    b = np.radians(tilt_deg)
    c, s_ = np.cos(b), np.sin(b)
    R = np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]]) if axis == "x" else np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
    d = R.T @ np.array([0.0, 0.0, 1.0])                      # the optical axis, camera frame before the tilt
    s = size / 2
    sq = np.array([[-s, -s], [s, -s], [s, s], [-s, s]])
    a = np.radians(turn_deg)
    sq = sq @ np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]).T + np.asarray(at) + d[:2] * HEIGHT / d[2]
    p = np.column_stack([sq, np.full(4, HEIGHT)])           # camera frame before the tilt: z = depth
    q = p @ R.T
    return q[:, :2] / q[:, 2:] * F_PX + CENTRE, q[:, 2]


def test_straight_camera_sees_a_square_at_any_turn():
    for turn in (0, 30, 90, 180, 223):
        corners, _ = view(turn_deg=turn)
        shape = csc.marker_shape(corners)
        assert shape["spread_pct"] < 1e-6
        assert shape["corner_err"] < 1e-6
        assert csc.straight(shape, CFG)


@pytest.mark.parametrize("axis", ["x", "y"])
@pytest.mark.parametrize("tilt", [-15.0, -8.0, 8.0, 15.0])
def test_tilted_camera_is_caught_and_the_red_edge_is_the_near_one(axis, tilt):
    corners, depth = view(tilt, axis, turn_deg=20)
    shape = csc.marker_shape(corners)
    assert not csc.straight(shape, CFG)
    near_side = int(np.argmin([depth[i] + depth[(i + 1) % 4] for i in range(4)]))
    assert shape["near_edge"] == near_side
    assert shape["sides"][near_side] == shape["sides"].max()
    mid = (corners[near_side] + corners[(near_side + 1) % 4]) / 2 - shape["centre"]
    assert shape["tilt"] @ mid > 0                              # the arrow points at the near edge


def test_angles_add_up_to_360():
    for tilt, axis in ((0, "x"), (12, "x"), (-9, "y")):
        assert csc.marker_shape(view(tilt, axis, turn_deg=37)[0])["angles"].sum() == pytest.approx(360)


def test_a_small_tilt_passes():
    shape = csc.marker_shape(view(1.0, "y")[0])
    assert csc.straight(shape, CFG)


def test_the_measured_bad_mount_fails():
    """2026-10-07 after the nozzle was mounted: slot 3 had sides 12 % apart. A tilt that gives this fails."""
    shape = csc.marker_shape(view(17.0, "x", turn_deg=5)[0])
    assert shape["spread_pct"] > 8
    assert not csc.straight(shape, CFG)


def test_nearest_to_centre():
    found = {"home": view(at=(8, 0))[0], "slot_3": view(at=(-3, 2))[0]}
    assert csc.nearest_to_centre(found, CENTRE) == "slot_3"
    assert csc.nearest_to_centre({}, CENTRE) is None


def test_report_lines():
    shape = csc.marker_shape(view(12.0, "x")[0])
    lines = csc.report("slot_3", shape, CFG, 10.0, 200.0, 50.0, 60.0)
    text = " ".join(s for s, _ in lines)
    assert "TILTED" in text and "RED edge" in text and "from the middle" in text
    ok = csc.report("slot_3", csc.marker_shape(view()[0]), CFG, 10.0, 5.0, 50.0, 60.0)
    assert "STRAIGHT" in ok[0][0]
    assert "No label" in csc.report(None, None, CFG, 10.0, 0.0, 0.0, 0.0)[0][0]


def test_draw_makes_a_picture():
    gray = np.full((400, 640), 200, np.uint8)
    corners = view(10.0, "y")[0]
    shape = csc.marker_shape(corners) | {"corners": corners}
    for rot in (False, True):
        img = csc.draw(gray, shape, csc.straight(shape, CFG), CFG, rot, [("x", (255, 255, 255))])
        assert img.shape[1] == int(640 * csc.SCALE) and img.ndim == 3
