"""Tests for tools/make_labels.py: the bed labels on one A4 sticker sheet (no printer needed)."""
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import make_labels as ml  # noqa: E402

PPM = 12.0   # px/mm for the rendered test pictures


def first_slot(labels):
    return next(lab for name, lab in labels.items() if name.startswith("slot_"))


@pytest.fixture(scope="module")
def bed():
    return ml.load_bed()


@pytest.fixture(scope="module")
def labels(bed):
    return ml.make_all(bed)


def detector():
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)


def detect(img: np.ndarray, angle_deg: float = 0.0):
    """Put the picture on white paper, rotate it, blur a little; return (ids, corners, label mm -> image)."""
    side = int(np.hypot(*img.shape)) + 160             # square canvas: the label fits at any angle
    pad_y, pad_x = (side - img.shape[0]) // 2, (side - img.shape[1]) // 2
    big = cv2.copyMakeBorder(img, pad_y, side - img.shape[0] - pad_y, pad_x, side - img.shape[1] - pad_x,
                             cv2.BORDER_CONSTANT, value=255)
    rot = cv2.getRotationMatrix2D((side / 2, side / 2), angle_deg, 1.0)
    big = cv2.GaussianBlur(cv2.warpAffine(big, rot, (side, side), flags=cv2.INTER_LINEAR, borderValue=255), (0, 0), 1.0)
    corners, ids, _ = detector().detectMarkers(big)
    # label mm -> image px (pixel-centre convention)
    to_img = rot @ np.array([[PPM, 0, pad_x - 0.5], [0, PPM, pad_y - 0.5], [0, 0, 1]])
    return ([] if ids is None else ids.ravel().tolist()), corners, to_img


def test_slot_zone_geometry(bed):
    zone = ml.slot_zone_mm(bed)
    assert np.allclose(zone, [(17, -8), (77, -8), (77, 8), (17, 8)])
    lay = ml.slot_layout(bed)
    assert (lay.w, lay.h) == (87.0, 20.0)
    assert lay.zone_h >= 12        # 12 mm tape must fit


def test_slot_centre_marks_are_on_the_zone_centre_line(bed, labels):
    lay = ml.slot_layout(bed)
    cy = lay.zone_y + lay.zone_h / 2
    marks = [s for s in first_slot(labels).shapes
             if isinstance(s, ml.Rect) and s.w > s.h and abs(s.y + s.h / 2 - cy) < 1e-9]
    assert len(marks) == 2
    assert marks[0].x + marks[0].w < lay.zone_x and marks[1].x > lay.zone_x + lay.zone_w   # outside the zone


def test_layout_fits_the_measured_reach(bed, labels):
    """The board and every row of parts are inside the camera reach; every marker can be seen from it."""
    assert ml.slot_numbers(bed) == [3, 4, 5, 6, 7]
    xs = sorted(xy[0] for xy in bed["layout"]["slot_marker_xy"].values())
    assert np.allclose(np.diff(xs), 30, atol=2)            # T-slot bars: about 30 mm apart
    (rx0, rx1), (ry0, ry1) = ml.reach_box()
    inside = lambda x, y, mx=0.0, my=0.0: rx0 - mx <= x <= rx1 + mx and ry0 - my <= y <= ry1 + my
    (ax, ay), (aw, ah) = bed["layout"]["pcb_area_xy"], bed["pcb_area"]["size_mm"]
    # 20 mm slack (D21): the reach is the old camera's; the bed survey measures the new one with the camera high
    assert inside(ax, ay, 20, 20) and inside(ax + aw, ay + ah, 20, 20), "the board is outside the reach"
    zone = ml.slot_zone_mm(bed)
    for name, (cx, cy), (dx, dy) in ml.label_placements(bed, labels):
        if name.startswith("slot_"):          # the row of parts: the zone centre line, both ends
            for u in (zone[0, 0], zone[1, 0]):
                assert inside(cx + u * dx, cy + u * dy), f"{name}: parts out of reach"
        # a 10 mm marker is still fully in the 85 x 53 mm view this far from the camera centre. Not the PCB
        # labels (D21, 2026-10-08): they sit at the front / right ends, seen with the camera high (bed survey)
        if not name.startswith("pcb_"):
            assert inside(cx, cy, 35, 20), f"{name} cannot be seen from inside the reach"


def test_nothing_overlaps(bed, labels):
    feet = ml.label_footprints(bed, labels)
    names = sorted(feet)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            A, B = feet[a], feet[b]
            assert A[2] <= B[0] or B[2] <= A[0] or A[3] <= B[1] or B[3] <= A[1], f"{a} overlaps {b}"
    (ax, ay), (aw, ah) = bed["layout"]["pcb_area_xy"], bed["pcb_area"]["size_mm"]
    for name, (x0, y0, x1, y1) in feet.items():
        assert x1 <= ax or ax + aw <= x0 or y1 <= ay or ay + ah <= y0, f"{name} lies in the PCB area"


def test_every_label_has_the_right_marker(bed, labels):
    m = bed["markers"]
    expected = {"home": m["home_id"]}
    if bed["pcb_area"].get("corner_labels", True):
        expected.update({f"pcb_{i}": v for i, v in enumerate(m["pcb_corner_ids"], start=1)})
    expected.update({f"slot_{n}": m["slot_ids"][n - 1] for n in ml.slot_numbers(bed)})
    assert set(labels) == set(expected)
    for name, lab in labels.items():
        ids, _, _ = detect(ml.render_label(lab, PPM))
        assert ids == [expected[name]], name
        assert lab.marker_xy[0] >= m["quiet_mm"] and lab.marker_xy[1] >= m["quiet_mm"], name


@pytest.mark.parametrize("angle", [0.0, 37.0, 180.0])
def test_slot_zone_follows_the_marker(bed, labels, angle):
    """The zone computed from the detected marker lands on the printed zone, at any label angle."""
    lab = first_slot(labels)
    ids, corners, to_img = detect(ml.render_label(lab, PPM), angle)
    assert ids == [lab.marker_id]
    s = bed["markers"]["size_mm"] / 2
    marker_mm = np.float32([(-s, -s), (s, -s), (s, s), (-s, s)])     # ArUco corner order 0..3
    H = cv2.getPerspectiveTransform(marker_mm, corners[0].reshape(4, 2).astype(np.float32))
    found = cv2.perspectiveTransform(ml.slot_zone_mm(bed).reshape(-1, 1, 2).astype(np.float32), H).reshape(4, 2)
    lay = ml.slot_layout(bed)
    x0, y0, x1, y1 = lay.zone_x, lay.zone_y, lay.zone_x + lay.zone_w, lay.zone_y + lay.zone_h
    printed = np.array([(x0, y0, 1), (x1, y0, 1), (x1, y1, 1), (x0, y1, 1)], float) @ to_img.T
    assert np.abs(found - printed).max() < 0.2 * PPM      # < 0.2 mm


def test_page_fits_a4_and_has_two_check_bars(bed, labels):
    shapes, where = ml.page_layout(bed, labels)
    pw, ph = bed["print"]["page_mm"]
    for name, (x, y) in where.items():
        assert 5 <= x and x + labels[name].w <= pw - 5 and 5 <= y and y + labels[name].h <= ph - 5, name
    bar = bed["print"]["check_bar_mm"]
    rects = [s for s in shapes if isinstance(s, ml.Rect)]
    assert any(r.w == bar for r in rects) and any(r.h == bar for r in rects)


def test_pdf_markers_have_exact_size_and_place(tmp_path, bed, labels):
    """Render the real PDF like a printer would and measure every marker on the page."""
    if not shutil.which("pdftoppm"):
        pytest.skip("pdftoppm not installed")
    shapes, where = ml.page_layout(bed, labels)
    pdf = tmp_path / "labels.pdf"
    ml.write_pdf(shapes, bed["print"]["page_mm"], pdf)
    dpi = 600
    subprocess.run(["pdftoppm", "-r", str(dpi), "-gray", "-png", "-singlefile", str(pdf), str(tmp_path / "p")],
                   check=True)
    page = cv2.imread(str(tmp_path / "p.png"), cv2.IMREAD_GRAYSCALE)
    corners, ids, _ = detector().detectMarkers(page)
    found = {int(i): c.reshape(4, 2) / (dpi / 25.4) for c, i in zip(corners, ids.ravel())}
    assert set(found) == {lab.marker_id for lab in labels.values()}
    m = bed["markers"]["size_mm"]
    for lab in labels.values():
        c = found[lab.marker_id]
        sides = np.linalg.norm(np.roll(c, -1, axis=0) - c, axis=1)
        assert np.allclose(sides, m, atol=0.1), lab.name
        x, y = where[lab.name]
        expected_centre = (x + lab.marker_xy[0] + m / 2, y + lab.marker_xy[1] + m / 2)
        assert np.allclose(c.mean(axis=0), expected_centre, atol=0.1), lab.name


def test_bed_map_draws(bed, labels):
    img = ml.bed_map(bed, labels)
    assert img.ndim == 3 and img.shape[1] > 300 and img.shape[0] > 300
    names = [p[0] for p in ml.label_placements(bed, labels)]
    assert sorted(names) == sorted(labels)


def test_tape_labels_fit_18mm_tape_with_10mm_markers(bed, labels):
    tp = bed["tape_printer"]
    tape_px, dpi = tp["tape_px"][tp["tape_mm"]], tp["dpi"]
    items = list(labels.values()) + [ml.scale_label(bed["print"]["check_bar_mm"])]
    for lab in items:
        img = ml.tape_image(lab, dpi, tape_px)
        assert img.shape[0] == tape_px, lab.name
        if lab.marker_id is None:
            continue
        big = cv2.copyMakeBorder(cv2.resize(img, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST),
                                 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
        corners, ids, _ = detector().detectMarkers(big)
        assert ids is not None and ids.ravel().tolist() == [lab.marker_id], lab.name
        c = corners[0].reshape(4, 2) / 4 / (dpi / 25.4)
        sides = np.linalg.norm(np.roll(c, -1, axis=0) - c, axis=1)
        assert np.allclose(sides, bed["markers"]["size_mm"], atol=0.15), lab.name


def test_tape_scale_label_is_100_mm(bed):
    lab = ml.scale_label(bed["print"]["check_bar_mm"])
    bar = [s for s in lab.shapes if isinstance(s, ml.Rect) and s.w > 50]
    assert len(bar) == 1 and bar[0].w == 100


def test_ptouch_print_accepts_the_tape_labels(tmp_path, bed, labels):
    if not shutil.which("ptouch-print"):
        pytest.skip("ptouch-print not installed")
    slot = first_slot(labels)
    paths = ml.make_tape(bed, {"home": labels["home"], slot.name: slot}, tmp_path)
    out = tmp_path / "strip.png"
    tape_px = bed["tape_printer"]["tape_px"][bed["tape_printer"]["tape_mm"]]
    args = ["ptouch-print", f"--force-tape-width={tape_px}"]
    for i, p in enumerate(paths):
        args += (["--cutmark"] if i else []) + ["--image", str(p)]
    subprocess.run(args + [f"--writepng={out}"], check=True, capture_output=True)
    strip = cv2.imread(str(out), cv2.IMREAD_GRAYSCALE)
    assert strip.shape[0] == tape_px
    big = cv2.resize(strip, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    _, ids, _ = detector().detectMarkers(cv2.copyMakeBorder(big, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255))
    assert sorted(ids.ravel().tolist()) == sorted([bed["markers"]["home_id"], slot.marker_id])


def test_owner_layout_relations(bed, labels):
    """HOME on the next bar right of the slots, in line with the slot markers; the board just in front."""
    lay = bed["layout"]
    slots = [xy for xy in lay["slot_marker_xy"].values()]
    hx, hy = lay["home_marker_xy"]
    assert all(abs(y - hy) < 4 for _, y in slots)          # in line, within hand-placement error
    assert abs(hx - (max(x for x, _ in slots) + 30)) < 3
    (ax, ay), (aw, ah) = lay["pcb_area_xy"], bed["pcb_area"]["size_mm"]
    gap = ml.front_line_l(bed, labels) - (ay + ah)
    assert 3 <= gap <= 25
    # D21 (2026-10-08): PCB 1 front-left, 2 front-right, 3 back-right, 4 back-left; the board centred between them
    assert bed["pcb_area"]["corner_labels"]
    p = np.array(lay["pcb_corner_marker_xy"], float)
    assert p[0, 0] < p[1, 0] and p[3, 0] < p[2, 0] and p[0, 1] < p[3, 1] and p[1, 1] < p[2, 1]
    assert np.allclose([ax + aw / 2, ay + ah / 2], p.mean(axis=0), atol=1.5)