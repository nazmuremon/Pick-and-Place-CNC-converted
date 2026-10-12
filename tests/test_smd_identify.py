"""Tests for tools/smd_identify.py with a synthetic camera image (no camera needed)."""
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import smd_identify as si  # noqa: E402

HI = 20.0      # px/mm of the drawn scene
MARGIN = 8.0   # mm of paper around the marker rectangle
PAPER = 220


def _box(img, center, size, angle, value):
    pts = cv2.boxPoints(((center[0] * HI, center[1] * HI), (size[0] * HI, size[1] * HI), angle))
    cv2.fillPoly(img, [np.round(pts).astype(np.int32)], value)


def _offset(p):
    return (p[0] + MARGIN, p[1] + MARGIN)


def _scene(cfg, extra=()) -> np.ndarray:
    """The sheet with markers and 4 parts, drawn top-down at HI px/mm."""
    sheet = cfg["sheet"]
    w, h = sheet["area_mm"]
    img = np.full((int((h + 2 * MARGIN) * HI), int((w + 2 * MARGIN) * HI)), PAPER, np.uint8)
    d = si.aruco_dictionary(sheet["dictionary"])
    s = sheet["marker_size_mm"]
    for marker_id, corners in si.marker_positions_mm(sheet).items():
        bits = cv2.aruco.generateImageMarker(d, marker_id, 6, borderBits=1)
        big = cv2.resize(bits, (int(s * HI),) * 2, interpolation=cv2.INTER_NEAREST)
        x, y = (int((c + MARGIN) * HI) for c in corners[0])
        img[y:y + big.shape[0], x:x + big.shape[1]] = big

    # 0805 resistor at 30 deg: black top, lighter metal ends
    _box(img, _offset((20, 22)), (2.0, 1.25), 30, 160)
    _box(img, _offset((20, 22)), (1.2, 1.25), 30, 40)
    # 1206 capacitor: even, light brown body
    _box(img, _offset((34, 14)), (3.2, 1.6), 0, 120)
    # SOT-23: 2.9 x 1.3 body, 3 leads, 2.4 overall
    _box(img, _offset((50, 22)), (2.9, 1.3), 0, 40)
    for dx, dy in [(-0.95, -0.95), (0.95, -0.95), (0, 0.95)]:
        _box(img, _offset((50 + dx, 22 + dy)), (0.4, 0.5), 0, 150)
    # SOIC-8: 4.9 x 3.9 body, 4 leads each side, 6.0 overall
    _box(img, _offset((35, 29)), (4.9, 3.9), 0, 35)
    for i in range(4):
        x = 35 - 1.905 + i * 1.27
        for y in (29 - 2.5, 29 + 2.5):
            _box(img, _offset((x, y)), (0.42, 1.0), 0, 150)
    for center, size, angle, value in extra:
        _box(img, _offset(center), size, angle, value)
    return img


def _camera_image(cfg, seed=0, extra=()) -> np.ndarray:
    """Look at the scene with a tilted camera at ~6.4 px/mm, like the Xitech camera, with blur and noise."""
    scene = cv2.GaussianBlur(_scene(cfg, extra), (0, 0), 2.0)
    w, h = cfg["sheet"]["area_mm"]
    area_mm = np.float32([(0, 0), (w, 0), (w, h), (0, h)])
    area_cam = np.float32([(95, 60), (545, 45), (560, 345), (80, 335)])
    H = cv2.getPerspectiveTransform(area_mm, area_cam)  # mm -> camera
    mm_from_scene = np.array([[1 / HI, 0, -MARGIN], [0, 1 / HI, -MARGIN], [0, 0, 1]])
    cam = cv2.warpPerspective(scene, H @ mm_from_scene, (640, 400), flags=cv2.INTER_AREA, borderValue=90)
    noise = np.random.default_rng(seed).normal(0, 3, cam.shape)
    return np.clip(cam + noise, 0, 255).astype(np.uint8)


@pytest.fixture
def cfg_packages():
    return si.load_config(si.DEFAULT_CONFIG)


def test_classify_picks_nearest_package(cfg_packages):
    _, packages = cfg_packages
    best, err, _ = si.classify(2.05, 1.2, packages, 0.25)
    assert best.name == "0805" and err < 0.1
    best, _, _ = si.classify(20.0, 20.0, packages, 0.25)
    assert best is None


def test_finds_parts_in_synthetic_camera_image(cfg_packages):
    cfg, packages = cfg_packages
    ident = si.Identifier(cfg, packages)
    res = None
    for seed in range(4):  # a few frames, like the live view (smoothing)
        res = ident.process(_camera_image(cfg, seed))
    assert res.found
    assert 5.5 < res.camera_px_per_mm < 7.5
    assert len(res.parts) == 4
    # The right package must be the first or the second guess (1206 and Crystal 3215 are nearly the same size).
    found = {}
    for p in res.parts:
        assert p.match is not None, f"unknown part {p.length_mm:.2f} x {p.width_mm:.2f} mm"
        for guess in (p.match, p.runner_up):
            if guess:
                found[guess.name] = p
    for name in ("0805", "1206", "SOT-23", "SOIC-8 / SOP-8"):
        assert name in found, f"{name} not found; got {sorted(found)}"
    assert "resistor" in found["0805"].hint
    assert "capacitor" in found["1206"].hint


def test_sheet_pdf_has_exact_scale(tmp_path, cfg_packages):
    cfg, _ = cfg_packages
    pdf = tmp_path / "sheet.pdf"
    si.make_sheet(cfg, pdf)
    assert pdf.read_bytes().startswith(b"%PDF")
    if not shutil.which("pdftoppm"):
        pytest.skip("pdftoppm not installed")
    subprocess.run(["pdftoppm", "-r", "600", "-gray", "-png", "-singlefile", str(pdf), str(tmp_path / "p")],
                   check=True)
    page = cv2.imread(str(tmp_path / "p.png"), cv2.IMREAD_GRAYSCALE)
    detector = cv2.aruco.ArucoDetector(si.aruco_dictionary(cfg["sheet"]["dictionary"]))
    corners, ids, _ = detector.detectMarkers(page)
    assert sorted(ids.flatten().tolist()) == sorted(cfg["sheet"]["marker_ids"])
    px_per_mm = 600 / 25.4
    centers = {int(i): c.reshape(4, 2).mean(axis=0) / px_per_mm for c, i in zip(corners, ids.flatten())}
    s = cfg["sheet"]["marker_size_mm"]
    w, h = cfg["sheet"]["area_mm"]
    a, b, c = (centers[i] for i in cfg["sheet"]["marker_ids"][:3])
    assert abs(np.linalg.norm(b - a) - (w - s)) < 0.1
    assert abs(np.linalg.norm(c - b) - (h - s)) < 0.1


def test_outside_margin_finds_part_next_to_the_rectangle(cfg_packages):
    cfg, packages = cfg_packages
    outside_part = [((-3.0, 20.0), (3.2, 1.6), 90, 120)]   # a 1206, 3 mm left of the marker rectangle
    counts = {}
    for margin in (0.0, 6.0):
        ident = si.Identifier(cfg, packages, outside_mm=margin)
        res = None
        for seed in range(4):
            res = ident.process(_camera_image(cfg, seed, outside_part))
        counts[margin] = res.parts
    assert len(counts[0.0]) == 4
    assert len(counts[6.0]) == 5
    new = [p for p in counts[6.0] if p.corners_mm[:, 0].mean() < 0]
    assert len(new) == 1
    assert np.allclose(new[0].corners_mm.mean(axis=0), (-3.0, 20.0), atol=0.3)
    assert "1206" in (new[0].match.name, new[0].runner_up.name if new[0].runner_up else "")
