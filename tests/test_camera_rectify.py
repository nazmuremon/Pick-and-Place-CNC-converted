"""tools/camera_rectify.py against a simulated head camera tilted 25 deg (like the bench camera on 2026-10-07)."""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import bed_test_run as t  # noqa: E402
import camera_rectify as cr  # noqa: E402
import camera_square_check as csc  # noqa: E402
import make_labels as ml  # noqa: E402
from pnp_paths import CONFIG_DIR  # noqa: E402  (the frozen test copy, tests/conftest.py)

CFG = yaml.safe_load((CONFIG_DIR / "bench.yaml").read_text())["camera_rectify"]
SIZE = (640, 400)
PX_MM = 20                        # the simulated bed picture
BED_X, BED_Y = (-110.0, 30.0), (-45.0, 25.0)


class TiltCam:
    """A pinhole camera on the head, tilted by tilt_deg about an axis in its picture plane (it looks to the
    front, like the bench camera). Bed plane z = 0; head position = the lens X/Y; the lens is lens mm up."""

    def __init__(self, tilt_deg=25.0, axis=(1.0, 0.3), lens=55.0, f=480.0, k1=0.0):
        r0 = np.array([[1.0, 0, 0], [0, -1, 0], [0, 0, -1]])          # straight down, a view from the front
        a = np.array([axis[0], axis[1], 0.0]) / np.hypot(*axis)
        rt = cv2.Rodrigues(a * np.radians(tilt_deg))[0]
        self.R, self.lens, self.f, self.k1, self.z = rt @ r0, lens, f, k1, 0.0

    def project(self, P, head):
        P = np.asarray(P, float).reshape(-1, 2)
        C = np.array([head[0], head[1], self.lens + self.z])
        Xc = (np.column_stack([P, np.zeros(len(P))]) - C) @ self.R.T
        x, y = Xc[:, 0] / Xc[:, 2], Xc[:, 1] / Xc[:, 2]
        d = 1 + self.k1 * (x * x + y * y)
        return np.column_stack([self.f * x * d + (SIZE[0] - 1) / 2, self.f * y * d + (SIZE[1] - 1) / 2])

    def bed_of_pixels(self, uv, head):
        """The bed point (z = 0) seen at each picture px (lens bending undone by iteration)."""
        uv = np.asarray(uv, float).reshape(-1, 2)
        xd = (uv[:, 0] - (SIZE[0] - 1) / 2) / self.f
        yd = (uv[:, 1] - (SIZE[1] - 1) / 2) / self.f
        x, y = xd.copy(), yd.copy()
        for _ in range(30):
            d = 1 + self.k1 * (x * x + y * y)
            x, y = xd / d, yd / d
        rays = np.column_stack([x, y, np.ones(len(x))]) @ self.R        # camera -> world
        C = np.array([head[0], head[1], self.lens + self.z])
        s = -C[2] / rays[:, 2]
        return C[:2] + rays[:, :2] * s[:, None]

    def axis_ray(self):
        return self.R.T @ np.array([0.0, 0.0, 1.0])                    # the picture centre's ray (world)

    def g_true(self):
        d = self.axis_ray()
        return d[:2] / d[2]


def bed_picture(world: dict) -> np.ndarray:
    """White bed with the 10 mm markers at their places (machine mm), PX_MM px per mm, top row = back."""
    w = int((BED_X[1] - BED_X[0]) * PX_MM)
    h = int((BED_Y[1] - BED_Y[0]) * PX_MM)
    img = np.full((h, w), 235, np.uint8)
    bed = ml.load_bed()
    ids = {"home": int(bed["markers"]["home_id"])}
    ids.update({f"slot_{n}": int(bed["markers"]["slot_ids"][n - 1]) for n in range(1, 9)})
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, bed["markers"]["dictionary"]))
    side = int(10 * PX_MM)
    for name, xy in world.items():
        m = cv2.aruco.generateImageMarker(dic, ids[name], side)
        u = int(round((xy[0] - BED_X[0]) * PX_MM - side / 2))
        v = int(round((BED_Y[1] - xy[1]) * PX_MM - side / 2))
        img[v:v + side, u:u + side] = np.where(m > 0, 235, 20).astype(np.uint8)
    return img


def bed_px_to_mm(uv):
    uv = np.asarray(uv, float)
    return np.column_stack([BED_X[0] + uv[:, 0] / PX_MM, BED_Y[1] - uv[:, 1] / PX_MM])


class SimCamera:
    def __init__(self, cam: TiltCam, bed: np.ndarray, machine) -> None:
        self.cam, self.bed, self.m = cam, bed, machine
        self.soft = cv2.GaussianBlur(bed, (0, 0), 1.2)          # no aliasing when sampled at ~8 px/mm
        self.after, self.cache = None, {}

    def raw(self) -> np.ndarray:
        key = (round(float(self.m.pos[0]), 6), round(float(self.m.pos[1]), 6), round(self.cam.z, 6))
        if key not in self.cache:
            self.cache[key] = self._render()
        return self.cache[key]

    def _render(self) -> np.ndarray:
        if self.cam.k1 != 0:                                   # lens bending: every pixel through the lens
            v, u = np.mgrid[:SIZE[1], :SIZE[0]]
            P = self.cam.bed_of_pixels(np.column_stack([u.ravel(), v.ravel()]), self.m.pos)
            mx = ((P[:, 0] - BED_X[0]) * PX_MM).reshape(SIZE[1], SIZE[0]).astype(np.float32)
            my = ((BED_Y[1] - P[:, 1]) * PX_MM).reshape(SIZE[1], SIZE[0]).astype(np.float32)
            f = cv2.remap(self.soft, mx, my, cv2.INTER_LINEAR, borderValue=128)
            return cv2.GaussianBlur(f, (0, 0), 0.6)
        h, w = self.bed.shape
        src = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], float)
        dst = self.cam.project(bed_px_to_mm(src), self.m.pos)
        H = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
        f = cv2.warpPerspective(self.bed, H, SIZE, flags=cv2.INTER_AREA, borderValue=128)
        return cv2.GaussianBlur(f, (0, 0), 0.6)

    def frames_after(self, t0, n, timeout=5.0):
        f = self.raw() if self.after is None else self.after(self.raw())
        return [f] * n


class SimGrbl:
    def __init__(self, cam: TiltCam):
        self.cam, self.z_moves = cam, []

    def move_axes(self, d, feed):
        self.cam.z += d.get("Z", 0.0)
        self.z_moves.append(d.get("Z", 0.0))


class SimMachine(t.Machine):
    """The Machine jobs with a fake head and the simulated tilted camera (no thread, no Grbl)."""

    def __init__(self, cam: TiltCam, world: dict, nominal: dict, start, tmp_path) -> None:
        self.nominal, self.out_dir = nominal, tmp_path
        self.max_step, self.feed, self.reach = 400.0, 2000.0, None
        self.pos = np.array(start, float)
        self.home = self.J = self.extremes = self.J_hint = self.marker_px_mm = None
        self.results, self.frame_shape, self.msg = [], (SIZE[1], SIZE[0]), ""
        self.stop_flag = type("F", (), {"is_set": staticmethod(lambda: False)})()
        self.lock = type("L", (), {"__enter__": lambda s: None, "__exit__": lambda s, *a: None})()
        self.messages, self.visited = [], []
        self.markers = t.Markers(ml.load_bed())
        self.tilt = cam
        self.cam = SimCamera(cam, bed_picture(world), self)
        self.grbl = SimGrbl(cam)

    def say(self, text):
        self.messages.append(text)

    def move(self, d):
        self.pos = self.pos + np.asarray(d, float)
        self.visited.append(self.pos.copy())


@pytest.fixture(autouse=True)
def private_state_files(tmp_path, monkeypatch):
    """The simulation must never read or write the real camera / position files in logs/ and config/."""
    monkeypatch.setattr(t, "POSITION_FILE", tmp_path / "head_position.yaml")
    monkeypatch.setattr(t, "CAMERA_FILE", tmp_path / "camera_calibration.yaml")
    monkeypatch.setattr(cr, "RECTIFY_FILE", tmp_path / "camera_rectify.yaml")


@pytest.fixture
def plan():
    bed = ml.load_bed()
    labels = ml.make_all(bed)
    return {n: np.array(c, float) for n, c, _ in ml.label_placements(bed, labels)}


def world_from(plan, rng):
    """Slot 3, 4, 5 and HOME as stuck by hand: the plan + a little error."""
    return {n: plan[n] + (rng.normal(0, 1.0, 2) if n != "home" else 0) for n in ("home", "slot_3", "slot_4", "slot_5")}


def head_over(cam: TiltCam, P, shift=(0.0, 0.0)) -> np.ndarray:
    """The head position that shows bed point P in the middle of the picture (+ shift mm)."""
    d = cam.axis_ray()
    return np.asarray(P, float) - d[:2] * (cam.lens + cam.z) / -d[2] + np.asarray(shift)


# ---------------------------------------------------------------- the pieces
def test_projective_centre_is_the_true_centre_under_perspective():
    cam = TiltCam()
    head = head_over(cam, (0, 0))
    sq = np.array([[-5, 5], [5, 5], [5, -5], [-5, -5]], float) + [3.0, 2.0]
    c = cam.project(sq, head)
    true = cam.project([[3.0, 2.0]], head)[0]
    assert np.allclose(cr.projective_centre(c), true, atol=1e-9)
    assert np.hypot(*(c.mean(axis=0) - true)) > 0.3            # the corner mean is off


ANGLES = {"home": 0.0, "slot_3": 0.02, "slot_4": -0.015, "slot_5": 0.03}     # labels stuck a little crooked (rad)


def corners_on_bed(P, angle, size=10.0):
    return np.asarray(P, float) + size * cr.SQUARE @ cr.turn(angle).T


def sightings_for(cam, world, heads, margin=20):
    out = []
    for h in heads:
        for name, P in world.items():
            bed = corners_on_bed(P, ANGLES[name])
            c = cam.project(bed, h)
            if not np.allclose(cam.bed_of_pixels(c, h), bed, atol=1e-6):
                continue                       # far outside the view: the lens formula folds it into the picture
            if c[:, 0].min() > margin and c[:, 1].min() > margin and c[:, 0].max() < SIZE[0] - margin \
                    and c[:, 1].max() < SIZE[1] - margin:
                out.append((np.asarray(h, float), name, c))
    return out


def grid_heads(cam, world):
    start = head_over(cam, world["slot_3"], (-4.0, -3.0))
    return [start + [-x, -y] for y in (0, 4, 8, 12) for x in (0, 10, 20, 30, 40, 50, 60)]


def test_fit_views_recovers_the_tilted_view(plan):
    cam = TiltCam()
    world = world_from(plan, np.random.default_rng(3))
    s = sightings_for(cam, world, grid_heads(cam, world))
    model, res, poses, report = cr.fit_views(s, "slot_3", SIZE)
    assert res.max() < 1e-3 and report["left_out"] == []
    for n, (M, a, ang) in poses.items():                 # each label's place (from slot 3), size and angle
        assert np.allclose(M, world[n] - world["slot_3"], atol=1e-3), n
        assert a == pytest.approx(10.0, abs=1e-3) and ang == pytest.approx(ANGLES[n], abs=1e-4), n
    # anywhere in the picture: the model is the view of bed point z + slot 3 from a head at 0
    z = np.random.default_rng(0).uniform(-40, 40, (50, 2)) - grid_heads(cam, world)[0]
    assert np.allclose(cr.projector(model)(z), cam.project(z + world["slot_3"], np.zeros(2)), atol=1e-3)


def test_lens_bending_is_fitted_with_the_camera_model(plan):
    cam = TiltCam(k1=-0.25)
    world = world_from(plan, np.random.default_rng(4))
    model, res, poses, report = cr.fit_views(sightings_for(cam, world, grid_heads(cam, world)), "slot_3", SIZE)
    assert model["kind"] == "camera" and report["left_out"] == []
    assert report["rms"]["homography"] > 1.0 and np.sqrt(np.mean(res ** 2)) < 0.02
    assert model["dist"][0] == pytest.approx(-0.25, abs=0.03)


def test_height_shift_and_lens_height():
    cam = TiltCam()
    head = head_over(cam, (0, 0), (1.0, -2.0))
    sq = np.array([[-5, 5], [5, 5], [5, -5], [-5, -5]], float)
    p0 = cam.project(sq, head)
    cam.z = 3.0
    p1 = cam.project(sq, head)
    cam.z = 0.0
    raw_view = cr.Rectifier({"kind": "homography", "H": homography_of(cam, head).tolist()}, SIZE)
    b0, b1 = raw_view.bed_of_raw(p0), raw_view.bed_of_raw(p1)

    def side(b):
        return float(np.mean(np.linalg.norm(np.roll(b, -1, axis=0) - b, axis=1)))
    g, lens = cr.height_shift(cr.projective_centre(b0), cr.projective_centre(b1), side(b0), side(b1), 3.0, raw_view.b_c)
    assert np.allclose(g, cam.g_true(), atol=1e-6)
    assert np.hypot(*cam.g_true()) == pytest.approx(np.tan(np.radians(25)), rel=1e-6)
    assert lens == pytest.approx(cam.lens, rel=1e-6)


def homography_of(cam: TiltCam, head) -> np.ndarray:
    """bed mm relative to the head -> px, exact (no lens bending)."""
    src = np.array([[-30, -30], [30, -30], [30, 30], [-30, 30]], float)
    return cv2.getPerspectiveTransform(src.astype(np.float32), cam.project(src + head, head).astype(np.float32))


def test_rectifier_makes_the_labels_square(plan):
    cam = TiltCam()
    world = world_from(plan, np.random.default_rng(5))
    head = head_over(cam, world["slot_3"], (2.0, -3.0))
    m = SimMachine(cam, world, plan, head, None)
    raw = m.cam.raw()
    rect = cr.Rectifier({"kind": "homography", "H": homography_of(cam, head).tolist()}, SIZE)
    found_raw = m.markers.find(raw)
    found = m.markers.find(rect(raw))
    assert "slot_3" in found_raw and "slot_3" in found
    assert csc.marker_shape(found_raw["slot_3"])["spread_pct"] > 5        # a trapezoid in the raw picture
    for name, c in found.items():
        s = csc.marker_shape(c)
        assert s["spread_pct"] < 1.0 and s["corner_err"] < 1.0, name
        assert s["sides"].mean() / rect.scale == pytest.approx(10.0, rel=0.01), name
    # the middle of the raw picture stays in the middle
    assert np.allclose(rect.project(rect.bed_of([rect.centre])), [rect.centre], atol=1e-6)


def test_grid_goes_to_the_front_and_towards_the_middle_slots(plan):
    right = cr.grid_moves(CFG, plan, "slot_3")
    left = cr.grid_moves(CFG, plan, "slot_7")
    assert all(d[1] <= 0 for d in right + left)                  # never to the back
    assert min(d[0] for d in right) < 0 and max(d[0] for d in right) == 0    # slot 3 is on the right: go left
    assert max(d[0] for d in left) > 0 and min(d[0] for d in left) == 0
    assert len(right) == len(CFG["grid_x_mm"]) * len(CFG["grid_y_mm"])


# ---------------------------------------------------------------- the whole job
@pytest.mark.parametrize("k1", [0.0, -0.2])
def test_job_measures_the_tilt_and_the_calibration_then_passes(plan, tmp_path, k1, monkeypatch):
    strict = {**t.load_bench(), "camera_check": {"max_axis_angle_error_deg": 2.0, "max_scale_error": 0.05}}
    monkeypatch.setattr(t, "load_bench", lambda: strict)       # the check for a straight camera
    cam = TiltCam(k1=k1)
    world = world_from(plan, np.random.default_rng(6))
    start = head_over(cam, world["slot_3"], (-3.0, -3.5))
    m = SimMachine(cam, world, plan, start, tmp_path)
    if k1 == 0:                       # before: the normal (strict) calibration refuses the tilted camera
        assert not m.calibrate("slot_3")
        assert any("REJECTED" in x for x in m.messages)
        m.J, m.pos = None, start.copy()

    out = tmp_path / "camera_rectify.yaml"
    data = cr.job_rectify(m, CFG, tmp_path, 3.0, path=out)
    assert data is not None, m.messages
    assert out.exists() and (tmp_path / "straightened.png").exists() and (tmp_path / "measurements.yaml").exists()
    assert data["fit_rms_px"] < 0.3
    assert (cr.yaml.safe_load(out.read_text())["model"]["kind"] == "camera") == (k1 != 0)
    assert np.allclose(data["height_shift_mm_per_mm"], cam.g_true(), atol=0.02)
    assert data["lens_height_mm"] == pytest.approx(cam.lens, rel=0.05)
    assert m.grbl.z_moves == [3.0, -3.0] and cam.z == 0.0          # Z back where it was
    start_rel = np.array(m.visited) - start
    assert start_rel[:, 1].max() <= 1e-9                          # never behind the start (the back row)
    assert start_rel[:, 0].max() <= 1e-9                          # slot 3 is on the right: only to the left
    assert np.allclose(m.pos, start)
    assert {"slot_3", "slot_4"} <= set(data["check_in_straightened_picture"])
    for name, c in data["check_in_straightened_picture"].items():
        assert c["sides_apart_pct"] < 1.5 and c["worst_corner_deg"] < 1.0, name

    # the saved measurements fit again without the machine, to the same correction
    again = cr.refit(tmp_path, CFG, path=tmp_path / "again.yaml")
    assert again is not None and again["fit_rms_px"] == pytest.approx(data["fit_rms_px"], abs=1e-3)

    # after: with straightened pictures the normal calibration passes, a clean view from the front
    rect = cr.load_rectifier(out, say=lambda s: None)
    m2 = SimMachine(cam, world, plan, start, tmp_path)
    m2.cam.after = rect
    assert m2.calibrate("slot_3"), m2.messages
    s = rect.scale
    assert np.allclose(m2.J, [[-s, 0], [0, s]], atol=0.03 * s)


def test_job_refuses_a_bad_fit(plan, tmp_path, monkeypatch):
    cam = TiltCam()
    world = world_from(plan, np.random.default_rng(7))
    m = SimMachine(cam, world, plan, head_over(cam, world["slot_3"], (-3.0, -3.5)), tmp_path)
    monkeypatch.setitem(CFG, "max_rms_px", -1.0)
    out = tmp_path / "camera_rectify.yaml"
    assert cr.job_rectify(m, CFG, tmp_path, 0.0, path=out) is None
    assert not out.exists()
    assert any("REFUSED" in x for x in m.messages)


def test_save_and_load(tmp_path):
    cam = TiltCam()
    head = np.zeros(2)
    rect = cr.Rectifier({"kind": "homography", "H": homography_of(cam, head).tolist()}, SIZE)
    p = tmp_path / "r.yaml"
    p.write_text(yaml.safe_dump({**rect.to_dict(), "time": "now"}))
    back = cr.load_rectifier(p, say=lambda s: None)
    img = np.random.default_rng(1).integers(0, 255, (SIZE[1], SIZE[0]), dtype=np.uint8)
    assert np.array_equal(back(img), rect(img))
    assert cr.load_rectifier(tmp_path / "none.yaml") is None


def test_a_label_that_moved_during_the_run_is_left_out(plan):
    cam = TiltCam(k1=-0.15)
    world = world_from(plan, np.random.default_rng(8))
    s = sightings_for(cam, world, grid_heads(cam, world))
    moved, k = [], 0
    for h, n, c in s:                                   # slot 5 slides 0.3 mm further at every sighting
        if n == "slot_5":
            k += 1
            c = cam.project(corners_on_bed(world[n] + [0.3 * k, 0.0], ANGLES[n]), h)
        moved.append((h, n, c))
    model, res, poses, report = cr.fit_views(moved, "slot_3", SIZE)
    assert [n for n, _ in report["left_out"]] == ["slot_5"] and "slot_5" not in poses
    assert np.sqrt(np.mean(res ** 2)) < 0.05


def test_points_the_camera_cannot_see_are_masked():
    cam = TiltCam(k1=-0.2)
    head = head_over(cam, (0, 0))
    x = np.concatenate([[cam.f, cam.k1, 0.0], cv2.Rodrigues(cam.R)[0].ravel(), -cam.R @ np.array([0, 0, cam.lens])])
    model = cr.camera_model(x, SIZE)
    near = np.array([[0.0, 0.0], [5.0, -5.0]]) - head
    far = np.array([[0.0, 400.0], [0.0, -400.0], [500.0, 0.0]]) - head        # behind the lens or far outside
    assert cr.seen_by(model, near).all() and not cr.seen_by(model, far).any()
