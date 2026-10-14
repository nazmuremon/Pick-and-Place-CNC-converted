"""Phase C: vision. Real bench pictures (tests/data/bench/, pixels only: no calibration exists for them) and the
simulator (a tilted camera with lens bending, ~5 px/mm, like the bench)."""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from conftest import BED, BOARDS, PROFILE

from pnp import sim
from pnp.board import kicad
from pnp.profile import load
from pnp.vision import calibrate as vcal
from pnp.vision import charuco, exposure, features, markers, parts, register
from pnp.vision.camera_model import CameraModel
from pnp.vision.motion_check import moved_as_predicted

BENCH = Path(__file__).resolve().parents[1] / "data" / "bench"


def grey(name: str) -> np.ndarray:
    img = cv2.imread(str(BENCH / name), cv2.IMREAD_GRAYSCALE)
    assert img is not None, name
    return img


# ================================================================ the camera model
def test_the_camera_model_maps_both_ways_on_every_plane():
    m = CameraModel.looking_down((640, 400), 600.0, 70.0, tilt_deg=25, tilt_dir_deg=90, dist=(-0.25, 0.08),
                                 turn_deg=3.8)
    assert abs(m.tilt_deg - 25) < 1e-9 and abs(m.height_mm - 70) < 1e-9
    assert np.allclose(m.px_to_xy([[320, 200]], 0.0), 0, atol=1e-9)              # D23: the picture centre
    px = np.array([[0, 0], [639, 399], [100, 300], [600, 40]], float)
    for z in (0.0, 1.6, 5.0):
        assert np.abs(m.xy_to_px(m.px_to_xy(px, z), z) - px).max() < 1e-6
    a, b = m.px_to_xy([[320, 200]], 0.0)[0], m.px_to_xy([[320, 200]], 1.0)[0]
    assert abs(np.hypot(*(b - a)) - np.tan(np.radians(25))) < 1e-9           # 0.47 mm per mm (docs/13 §3.2)
    m2 = CameraModel.from_dict(m.to_dict())
    assert np.allclose(m2.px_to_xy(px, 2.0), m.px_to_xy(px, 2.0))


# ================================================================ the simulator's bench
@pytest.fixture(scope="module")
def bench():
    p = load(PROFILE)
    bed = yaml.safe_load(BED.read_text())
    g, cams = sim.build(p, bed, run_cameras=False)
    cam = cams["camera_head"]
    yield p, bed, g, cam
    g.shutdown()


class Rig:
    """Moves the simulated head directly (no controller: fast). scale: the head moves this much per counted mm."""

    def __init__(self, g, cam, scale=(1.0, 1.0)):
        self.g, self.cam, self.scale = g, cam, scale
        self.counted = np.array([g.phys["X"], g.phys["Y"]], float)

    def head(self):
        return (self.counted[0], self.counted[1], 0.0)

    def move_by(self, dx, dy):
        self.counted += (dx, dy)
        self.g.phys["X"] += dx * self.scale[0]
        self.g.phys["Y"] += dy * self.scale[1]

    def move_to(self, x, y):
        self.move_by(x - self.counted[0], y - self.counted[1])

    def frames(self, n):
        return [self.cam.render() for _ in range(n)]

    def frame(self):
        return self.cam.render()


@pytest.fixture(scope="module")
def calibrated(bench):
    p, bed, g, cam = bench
    w = cam.world
    g.phys.update(X=-60.0, Y=-150.0)
    w.add(sim.charuco_layer(w, -88.0, -170.0, theta=7.0, z=0.0))
    v0 = vcal.collect(Rig(g, cam), 0, say=lambda s: None)
    w.add(sim.charuco_layer(w, -92.0, -168.0, theta=-4.0, z=5.0))
    v1 = vcal.collect(Rig(g, cam), 1, say=lambda s: None)
    w.remove("charuco")
    return vcal.solve(v0 + v1, cam.model.size, 5.0), v0


def test_calibration_finds_the_true_camera(bench, calibrated):
    _, _, _, cam = bench
    res, _ = calibrated
    m, true = res.model, cam.model
    assert abs(m.f - true.f) < 2 and abs(m.dist[0] - true.dist[0]) < 0.005
    assert abs(m.tilt_deg - true.tilt_deg) < 0.1 and abs(m.height_mm - true.height_mm) < 0.5
    assert res.rms_px < 0.3 and all(v < 0.05 for v in res.rms_mm.values())
    assert max(abs(s - 1) for s in res.move_scale) < 0.001 and res.warnings == []
    px = np.array([[20, 20], [620, 380], [320, 200], [100, 300], [600, 40]], float)
    for z in (0.0, 1.6, 5.0):                                                # every working plane (§4.5)
        assert np.hypot(*(m.px_to_xy(px, z) - true.px_to_xy(px, z)).T).max() < 0.05


def test_a_label_measures_its_size_anywhere_in_the_picture(bench, calibrated):
    """docs/13 §4.5: a 10 mm label must measure 10.0 ± 0.1 mm anywhere in the picture.
    In the sim at ~5 px/mm the ArUco corners themselves are good to about 0.5 px: the same pictures measured with
    the TRUE camera give 9.80–10.07 mm per side (mean down to 9.88 near the edge). So the test splits it: the
    calibrated camera must measure like the true camera (±0.05 mm per side: the calibration's share), and the
    mean side must be 10.0 ± 0.15 mm (the corner finder's share, honest for this pixel size)."""
    p, bed, g, cam = bench
    model = calibrated[0].model
    mk = markers.Markers(bed)
    seen = 0
    for dx, dy in ((0, 0), (-25, 10), (20, -8)):                             # HOME near the middle and the edges
        g.phys.update(X=-24.0 + dx, Y=-20.5 + dy)
        frames = [cam.render() for _ in range(6)]
        s = mk.measure(frames, model, "home")                                # corners averaged, like homing
        t = mk.measure(frames, cam.model, "home")
        if s is None or t is None:
            continue
        seen += 1
        assert np.abs(s.sides_mm - t.sides_mm).max() < 0.05, (dx, dy, s.sides_mm, t.sides_mm)
        assert abs(s.sides_mm.mean() - 10.0) < 0.15, (dx, dy, s.sides_mm)
    assert seen >= 2


def test_the_head_move_scale_shows_a_steps_per_mm_error(bench, calibrated):
    p, bed, g, cam = bench
    _, v0 = calibrated
    w = cam.world
    g.phys.update(X=-60.0, Y=-150.0)
    w.add(sim.charuco_layer(w, -88.0, -170.0, theta=7.0, z=0.0))
    views = vcal.collect(Rig(g, cam, scale=(1.02, 1.0)), 0, say=lambda s: None)
    w.remove("charuco")
    res = vcal.solve(views, cam.model.size)
    assert abs(res.move_scale[0] - 1.02) < 0.003 and abs(res.move_scale[1] - 1.0) < 0.003
    assert any("2.0 % more than counted in X" in x for x in res.warnings)


def test_calibration_needs_enough_pictures_and_head_moves():
    d = charuco.Detector()
    det = d.find(charuco.sheet_image(8.0))
    assert len(det.ids) == 88
    with pytest.raises(ValueError, match="at least 3"):
        vcal.solve([vcal.View((0, 0, 0), 0, det)], (640, 400))
    with pytest.raises(ValueError, match="did not move"):
        vcal.solve([vcal.View((0, 0, 0), 0, det)] * 3, (640, 400))


def test_the_charuco_pdf_is_written(tmp_path):
    p = charuco.write_pdf(tmp_path / "sheet.pdf", dpi=150)
    assert p.read_bytes()[:4] == b"%PDF"


# ================================================================ markers, homing, exposure, motion
def test_bench_pictures_show_home_and_slot_4():
    mk = markers.Markers(yaml.safe_load(BED.read_text()))
    for name in ("strip/20261011-164558_home.png", "pick/20261011-164558_C9_before.png"):
        seen = mk.find(grey(name))
        assert {"home", "slot_4"} <= set(seen), name


def test_home_is_measured_where_it_is_seen(bench, calibrated):
    p, bed, g, cam = bench
    model = calibrated[0].model
    mk = markers.Markers(bed)
    g.phys.update(X=-21.3, Y=-24.8)
    off, why = markers.home_offset(mk, [cam.render() for _ in range(3)], model)
    assert why == "" and np.hypot(*(off - (21.3, 24.8))) < 0.05             # HOME is at (0, 0)
    bad = CameraModel.from_dict({**model.to_dict(), "f": model.f * 1.1})
    off, why = markers.home_offset(mk, [cam.render()], bad)
    assert off is None and "does not fit" in why                            # a wrong calibration is refused


def test_exposure_is_found_and_a_light_change_is_seen(bench):
    p, bed, g, cam = bench
    g.phys.update(X=-24.0, Y=-20.5)
    value, light = exposure.auto(cam.set_exposure, cam.render, start=20.0)
    assert value is not None and abs(light["p95"] - 200) <= 12
    assert exposure.changed(light, light) is None
    cam.set_exposure(value * 1.5)
    assert "light changed" in exposure.changed(exposure.light(cam.render()), light)
    cam.set_exposure(None)


def test_the_stall_check_sees_a_blocked_move_and_does_not_guess(bench, calibrated):
    from pnp.machine_vision import move_homography
    p, bed, g, cam = bench
    model = calibrated[0].model
    g.phys.update(X=-40.0, Y=-30.0)
    a = cam.render()
    H = move_homography(model, (12.0, 0.0))                                  # across the bars and the labels
    g.phys["X"] += 12.0
    assert moved_as_predicted(a, cam.render(), H) is True
    g.phys["X"] -= 12.0
    assert moved_as_predicted(a, cam.render(), H) is False                   # blocked: the picture did not move
    g.phys.update(X=-40.0, Y=-60.0)                                          # bars only, a move along them:
    b = cam.render()
    assert moved_as_predicted(b, b, move_homography(model, (0.0, 12.0))) is None     # the picture can't tell
    w = cam.world
    w.add(sim.strip_layer(w, sim.Strip(slot=3)))
    c, u, v = w.slot_frame(3)
    g.phys.update(X=float((c + u * 47)[0]), Y=float((c + u * 47)[1]))
    b = cam.render()
    # one pocket along the strip: the strip looks the same; the bars too. Never a stall on a guess.
    assert moved_as_predicted(b, b, move_homography(model, u * 4.0)) in (None, True)
    w.remove("strip_3")


# ================================================================ parts in a strip
def _bench_strip(img):
    """Straighten slot 3 of a bench picture from the HOME and slot-4 labels (their surveyed places in bed.yaml):
    no calibration exists for these pictures."""
    bed = yaml.safe_load(BED.read_text())
    mk = markers.Markers(bed)
    lay = bed["layout"]
    u = np.array(lay["slot_direction"], float)
    v = np.array([u[1], -u[0]])
    local = np.array([[-5, -5], [5, -5], [5, 5], [-5, 5.0]])
    cm = lambda c, k=0: np.array([np.asarray(c, float) + u * a + v * b for a, b in np.roll(local, k, axis=0)])  # noqa
    seen = mk.find(img)
    src = np.r_[cm(lay["slot_marker_xy"][4]), cm(lay["home_marker_xy"], 3)]     # HOME's label is stuck turned 90°
    dst = np.r_[seen["slot_4"].corners_px, seen["home"].corners_px]
    Hm, _ = cv2.findHomography(src, dst)
    s3 = np.array(lay["slot_marker_xy"][3], float)
    spec = parts.StripSpec(body=(2.0, 1.25))
    f = lambda uv: cv2.perspectiveTransform((s3 + uv[:, :1] * u + uv[:, 1:] * v).reshape(-1, 1, 2), Hm).reshape(-1, 2)  # noqa
    strip, back = parts.straighten(img, f, spec)
    return strip, back, spec


def code(found, n=9):
    return "".join({"present": "P", "candidate": "c", "empty": ".", "unclear": "?"}[x.state] for x in found[:n])


def test_bench_strip_parts_need_a_confirmed_first_part():
    """docs/13 §4.4: without the operator's yes on the first part, nothing is 'present' (a 5 px/mm camera sees an
    empty pocket and a part about as dark). With it, the real strip reads as the bench saw it."""
    strip, back, spec = _bench_strip(grey("pick/20261011-164558_C9_before.png"))
    plain = parts.find(strip, back, spec, pitch_tol=0.06)
    assert "P" not in code(plain) and code(plain).count("c") >= 7
    tpl = parts.learn_template(strip, plain[2], spec)
    assert code(parts.find(strip, back, spec, template=tpl, pitch_tol=0.06)) == "..PPPPPPP"
    after, back2, _ = _bench_strip(grey("pick/20261011-164558_C9_after.png"))
    assert code(parts.find(after, back2, spec, template=tpl, pitch_tol=0.06)) == "...PPPPPP"   # the picked one
    start, back3, _ = _bench_strip(grey("strip/20261011-164558_home.png"))
    assert code(parts.find(start, back3, spec, template=tpl, pitch_tol=0.06)) == "PPPPPPPPP"


@pytest.mark.parametrize("tape", ["paper", "black", "clear"])
def test_simulated_strips_on_every_tape(bench, calibrated, tape):
    from pnp.job import dryrun
    p, bed, g, cam = bench
    model = calibrated[0].model
    w = cam.world
    pockets = [sim.Pocket(present=k not in (1, 5, 6)) for k in range(14)]
    st = sim.Strip(slot=4, tape=tape, pockets=pockets)
    w.add(sim.strip_layer(w, st))
    c, u, v = w.slot_frame(4)
    s = dryrun.SlotSetup(4, c, u, v, parts.StripSpec(body=(2.0, 1.25)), st.z_top)
    mid = c + u * 47
    g.phys.update(X=float(mid[0]), Y=float(mid[1]))
    strip, back = parts.straighten(cam.render().astype(np.float32), dryrun.mapper(model, mid, s), s.spec)
    plain = parts.find(strip, back, s.spec)
    first = next(f for f in plain if f.state == "candidate" and abs(f.u - 20) < 0.5)
    tpl = parts.learn_template(strip, first, s.spec)
    found = [f for f in parts.find(strip, back, s.spec, template=tpl) if 18 < f.u < 74]
    want = ["P" if q.present else "." for q in pockets][:len(found)]
    assert code(found, 99) == "".join(want), tape
    for f in found:
        k = round((f.u - st.first_u) / st.pitch)
        if f.state == "present":
            assert abs(f.u - (st.first_u + k * st.pitch)) < 0.12 and abs(f.v) < 0.12
    w.remove("strip_4")


@pytest.mark.parametrize("pair, verdict", [
    ("20261011-164558_C23", "picked"), ("20261011-164558_C5", "picked"), ("20261011-164558_C9", "picked"),
    ("20261011-164558_C50", "picked"), ("20261011-164558_C48", "missed"), ("20261011-160459_C32", "missed"),
    ("20261011-144955_C24", "unclear"), ("20261011-164235_C6", "unclear"),
])
def test_the_pick_check_on_real_picks(pair, verdict):
    r = parts.pick_check(grey(f"pick/{pair}_before.png"), grey(f"pick/{pair}_after.png"), (320, 200), (4, 3))
    assert r.verdict == verdict, (pair, r)


# ================================================================ holes, corners, registration
@pytest.mark.parametrize("name, near, want", [
    ("H2", (101, 207), (102, 207)), ("H3", (99, 204), (98.5, 204)), ("H4", (304, 213), (305, 215))])
def test_bench_holes_are_found(name, near, want):
    h = features.find_hole(grey(f"holes/20261011-162240_{name}.png"), near, 8.5, (0, 0), 10)
    assert h is not None and np.hypot(*(h.centre - want)) < 4.0


@pytest.mark.parametrize("name, near, d1, d2, want", [
    ("H1", (328, 185), (-1, 0), (0, 1), (330.6, 194.3)), ("H3", (78, 228), (1, 0), (0, -1), (74.5, 224.9)),
    ("H4", (328, 235), (-1, 0), (0, -1), (325.7, 237.3))])
def test_bench_board_corners_are_found(name, near, d1, d2, want):
    c = features.find_corner(grey(f"holes/20261011-162240_{name}.png"), near, d1, d2, 14, 70, 8)
    assert c is not None and np.hypot(*(c.point - want)) < 2.0 and abs(c.angle_deg - 90) < 2


@pytest.fixture(scope="module")
def board_on_bed(bench):
    p, bed, g, cam = bench
    b = kicad.load(BOARDS / "ht5023.kicad_pcb.gz")
    pose = sim.BoardPose(-125.0, -240.0, 1.5, "B", 1.6)
    layers = sim.board_layers(cam.world, b, pose)
    for L in layers:
        cam.world.add(L)
    yield b, pose
    for L in layers:
        cam.world.remove(L.name)


def test_holes_seen_at_an_angle_are_measured_on_the_board_top(bench, calibrated, board_on_bed):
    """The two-rim model: a plain circle fit is 0.35–0.74 mm off (the lens of top and bottom rims, a hole half over
    a groove); the two-rim fit is within 0.08 mm on all four."""
    p, bed, g, cam = bench
    m = calibrated[0].model
    b, pose = board_on_bed
    for h in b.holes:
        true = pose.to_machine(b, (h.x, h.y))
        g.phys.update(X=true[0] + 4.0, Y=true[1] - 3.0)
        img = cam.render()
        rel = true - np.array([g.phys["X"], g.phys["Y"]])
        near = m.xy_to_px([rel + (0.4, -0.3)], 1.6)[0]
        delta = m.xy_to_px([rel], 0.0)[0] - m.xy_to_px([rel], 1.6)[0]
        hole = features.find_hole(img, near, 1.6 * m.px_per_mm(1.6), delta, 12)
        assert hole is not None, h.ref
        assert np.hypot(*(m.px_to_xy([hole.centre], 1.6)[0] - rel)) < 0.08, h.ref


def test_registration_places_every_part_within_a_tenth(bench, calibrated, board_on_bed):
    p, bed, g, cam = bench
    m = calibrated[0].model
    b, pose = board_on_bed
    M0, t0 = register.guess(b, "B", "back", (pose.x + b.width / 2 + 3.0, pose.y + b.height / 2 - 2.0))
    reg = register.run(Rig(g, cam), b, "B", register.rotation(0.2) @ M0, t0, m, 1.6, say=lambda s: None)
    assert reg is not None and reg.kind == "rigid" and reg.rms < 0.12
    err = [np.hypot(*(reg.to_machine(b, (c.x, c.y)) - pose.to_machine(b, (c.x, c.y)))) for c in b.components]
    assert max(err) < 0.1
    # a feature far off is left out by name, the others stay
    feats = [register.Feature(**{**f.__dict__}) for f in reg.features]
    bad = next(f for f in feats if f.measured is not None)
    bad.measured = (bad.measured[0] + 1.5, bad.measured[1])
    r2 = register.fit(b, "B", feats)
    assert bad.name not in r2.residuals and "did not fit" in bad.why
    # features outside the reach are skipped, not fatal
    reach = (pose.x - 10, pose.x + 60, pose.y - 10, pose.y + 70)
    r3 = register.run(Rig(g, cam), b, "B", np.asarray(reg.M), np.asarray(reg.t), m, 1.6, reach, say=lambda s: None)
    assert any(f.why == "outside the camera's reach" for f in r3.features)
