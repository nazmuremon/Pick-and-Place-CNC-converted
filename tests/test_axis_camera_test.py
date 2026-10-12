"""Tests for the camera measurement in tools/axis_camera_test.py (no camera, no Grbl needed)."""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import axis_camera_test as act  # noqa: E402
import smd_identify as si  # noqa: E402

HI = 20.0      # px/mm of the drawn sheet
MARGIN = 15.0  # mm of paper around the marker rectangle
CAM_W, CAM_H = 640, 400


@pytest.fixture(scope="module")
def cfg():
    return si.load_config(si.DEFAULT_CONFIG)[0]


@pytest.fixture(scope="module")
def sheet_image(cfg):
    sheet = cfg["sheet"]
    w, h = sheet["area_mm"]
    img = np.full((int((h + 2 * MARGIN) * HI), int((w + 2 * MARGIN) * HI)), 220, np.uint8)
    d = si.aruco_dictionary(sheet["dictionary"])
    s = sheet["marker_size_mm"]
    for marker_id, corners in si.marker_positions_mm(sheet).items():
        bits = cv2.aruco.generateImageMarker(d, marker_id, 6, borderBits=1)
        big = cv2.resize(bits, (int(s * HI),) * 2, interpolation=cv2.INTER_NEAREST)
        x, y = (int((c + MARGIN) * HI) for c in corners[0])
        img[y:y + big.shape[0], x:x + big.shape[1]] = big
    return img


def camera_view(sheet_image, centre_mm, px_per_mm=7.0, angle_deg=3.0):
    """What a camera sees when its image centre is over centre_mm of the sheet."""
    a = np.radians(angle_deg)
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    lin = HI / px_per_mm * rot                     # camera px -> sheet image px
    t = (np.asarray(centre_mm) + MARGIN) * HI - lin @ np.array([CAM_W / 2, CAM_H / 2])
    m = np.hstack([lin, t.reshape(2, 1)])
    return cv2.warpAffine(sheet_image, m, (CAM_W, CAM_H), flags=cv2.INTER_AREA | cv2.WARP_INVERSE_MAP,
                          borderValue=220)


def test_centre_is_found(cfg, sheet_image):
    meter = act.SheetMeter(cfg)
    m = meter.measure([camera_view(sheet_image, (30.0, 20.0))] * 3)
    assert m is not None and m.markers == 4
    assert np.allclose(m.xy, (30.0, 20.0), atol=0.05)
    assert m.px_per_mm == pytest.approx(7.0, rel=0.02)


@pytest.mark.parametrize("move", [(1.0, 0.0), (0.0, 1.0), (0.5, 0.0), (-2.0, 1.5)])
def test_move_is_measured(cfg, sheet_image, move):
    meter = act.SheetMeter(cfg)
    a = meter.measure([camera_view(sheet_image, (30.0, 20.0))] * 3)
    b = meter.measure([camera_view(sheet_image, (30.0 + move[0], 20.0 + move[1]))] * 3)
    assert np.allclose(b.xy - a.xy, move, atol=0.05)


def test_no_markers_gives_none(cfg):
    meter = act.SheetMeter(cfg)
    blank = np.full((CAM_H, CAM_W), 220, np.uint8)
    assert meter.measure([blank] * 3) is None
