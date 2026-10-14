"""The 3D printed SMD tray (hardware/tray/smd_tray.py): one closed body, and a rail that is `fit` smaller than the
measured T-slot on every side. Geometry only, no files written."""
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

mf = pytest.importorskip("manifold3d")
pytest.importorskip("trimesh")

SPEC = importlib.util.spec_from_file_location(
    "smd_tray", Path(__file__).resolve().parents[1] / "hardware" / "tray" / "smd_tray.py")
st = importlib.util.module_from_spec(SPEC)
sys.modules["smd_tray"] = st            # dataclasses look the module up while it loads
SPEC.loader.exec_module(st)


from dataclasses import replace

SET = [(8, "paper"), (8, "embossed"), (12, "embossed"), (16, "embossed")]


@pytest.fixture(scope="module")
def holder3():
    return st.holder(st.P(), 3, 42)


def solid_at(m, x):
    import cv2
    loops = [np.asarray(q, np.float32) for q in m.warp(lambda v: (v[1], v[2], v[0])).slice(x).to_polygons()]
    return lambda y, z: any(cv2.pointPolygonTest(q, (float(y), float(z)), False) > 0 for q in loops)


def test_holder_is_one_closed_body(holder3):
    assert holder3.status() == mf.Error.NoError
    assert len(holder3.decompose()) == 1


def test_rail_is_fit_smaller_than_the_slot_on_every_side(holder3):
    p = st.P()
    loop = np.asarray(holder3.warp(lambda v: (v[1], v[2], v[0])).slice(50.0).to_polygons()[0])
    rail = loop[loop[:, 1] < -0.001]
    neck = loop[(loop[:, 1] < 0) & (loop[:, 1] > -p.slot_b)]
    assert np.ptp(neck[:, 0]) == pytest.approx(p.slot_a - 2 * p.fit, abs=1e-3)
    assert np.ptp(rail[:, 0]) == pytest.approx(p.slot_c - 2 * p.fit, abs=1e-3)
    assert rail[:, 1].min() == pytest.approx(-(p.slot_b + p.slot_d) + p.fit, abs=1e-3)   # above the slot floor
    shoulder = loop[np.isclose(np.abs(loop[:, 0]), (p.slot_a - 2 * p.fit) / 2, atol=1e-3)][:, 1].min()
    assert shoulder == pytest.approx(-(p.slot_b + p.fit), abs=1e-3)                       # under the lips
    assert p.fit == pytest.approx(-0.10)


def test_the_set_is_four_8mm_one_12mm_one_16mm():
    assert st.DEFAULT_SET == {1: 8, 2: 8, 3: 8, 4: 8, 5: 12, 6: 16}
    assert st.KINDS[8] == ("paper", "embossed")


@pytest.mark.parametrize("w,kind", SET)
def test_every_holder_holds_its_tape_at_the_same_height(w, kind):
    """The tape's top is at tape_top in every holder (one pick height), the parts' centre line on the bar centre,
    the lips over both edges, the window open, the pockets of embossed tape in a groove."""
    p = replace(st.P(), tape_w=w, kind=kind)
    m = st.holder(p, 3, 42)
    assert m.status() == mf.Error.NoError and len(m.decompose()) == 1
    solid = solid_at(m, 50.0)
    he, fe = st.tape_edges(w)
    under = st.under_of(p)
    assert p.tape_top == st.P().tape_top and under == pytest.approx(p.tape_top - (1.0 if kind == "paper" else 0.3))
    for y in (-fe + 0.3, he - 0.3):                                  # both tape edges: floor below, lip above
        assert solid(y, under - 0.05) and not solid(y, under + 0.05) and solid(y, p.tape_top + p.lip_gap + 0.05)
    assert not solid(-fe - 0.05, p.tape_top - 0.1) and not solid(he + 0.05, p.tape_top - 0.1)    # channel width
    assert solid(-fe - 0.4, p.tape_top - 0.1) and solid(he + 0.4, p.tape_top - 0.1)
    assert not solid(0.0, p.top - 0.1)                               # the window over the parts is open
    if kind == "embossed":                                           # any part height: the pockets hang in a groove
        assert not solid(0.0, p.base + 0.05) and solid(0.0, p.base - 0.05)
    else:
        assert solid(0.0, under - 0.05)
    bb = m.bounding_box()
    assert bb[4] - bb[1] <= 24 and bb[5] == pytest.approx(p.top)     # beside the others on bars ~30 apart


def test_pocket_depth_raises_the_groove_floor_and_too_deep_is_refused():
    p = replace(st.P(), tape_w=12, kind="embossed", pocket=2.5)
    solid = solid_at(st.holder(p, 5, 44), 50.0)
    floor = st.under_of(p) - 2.5
    assert solid(0.0, floor - 0.05) and not solid(0.0, floor + 0.05)
    with pytest.raises(ValueError):
        st.holder(replace(st.P(), tape_w=8, kind="embossed", pocket=7.0), 1, 40)


CSPEC = importlib.util.spec_from_file_location(
    "pcb_corner", Path(__file__).resolve().parents[1] / "hardware" / "tray" / "pcb_corner.py")
pc = importlib.util.module_from_spec(CSPEC)
sys.modules["pcb_corner"] = pc
CSPEC.loader.exec_module(pc)


@pytest.mark.parametrize("n", [1, 2, 3, 4])
def test_pcb_corner_pieces_are_one_body_and_their_marker_reads(n):
    import cv2
    c, p = pc.C(), st.P()
    m = pc.piece(c, p, n)
    assert m.status() == mf.Error.NoError and len(m.decompose()) == 1
    sc, img = 8, np.zeros((400, 720), np.uint8)
    for q in m.slice(c.top - c.recess / 2).to_polygons():          # top view: white top, black recesses
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, [((np.asarray(q) * [1, -1] + [45, 15]) * sc).astype(np.int32)], 255)
        img ^= mask
    img = cv2.copyMakeBorder(img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    _, ids, _ = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)).detectMarkers(img)
    assert ids is not None and ids.ravel().tolist() == [pc.MARKER_IDS[n]]


def test_back_corner_pieces_hold_the_corner():
    """PCB 4: inner corner of the L exactly at the board's back-left corner; PCB 3: the leg leaves side_play."""
    c, p = pc.C(), st.P()
    for n, x_edge in ((4, c.left_arm), (3, -(c.right_arm - c.side_play))):
        m = pc.piece(c, p, n)
        just_inside = (x_edge + (0.2 if n == 4 else -0.2), -0.2)       # on the board, next to the corner
        just_outside = (x_edge + (-0.2 if n == 4 else 0.2), -0.2)      # in the leg
        behind = (x_edge + (0.2 if n == 4 else -0.2), 0.2)             # in the back wall
        cut = m.slice(1.0)
        def solid(xy):
            return any(cv2_point_in(q, xy) for q in cut.to_polygons())
        assert not solid(just_inside) and solid(just_outside) and solid(behind)


def cv2_point_in(poly, xy) -> bool:
    import cv2
    return cv2.pointPolygonTest(np.asarray(poly, np.float32), tuple(map(float, xy)), False) > 0


def test_home_piece_is_one_body_with_the_home_marker_at_its_centre():
    import cv2
    h, p = pc.Home(), st.P()
    m = pc.home_piece(h, p)
    assert m.status() == mf.Error.NoError and len(m.decompose()) == 1
    sc, img = 10, np.zeros((400, 300), np.uint8)
    for q in m.slice(h.top - h.recess / 2).to_polygons():
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, [((np.asarray(q) * [1, -1] + [15, 12]) * sc).astype(np.int32)], 255)
        img ^= mask
    img = cv2.copyMakeBorder(img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    corners, ids, _ = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)).detectMarkers(img)
    assert pc.HOME_ID == 49 and ids is not None and ids.ravel().tolist() == [49]
    assert np.allclose(corners[0][0].mean(0), [40 + 15 * sc, 40 + 12 * sc], atol=2)   # marker centre = (0, 0)
