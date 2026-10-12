#!/usr/bin/env python3
"""Straighten the pictures of the tilted head camera (bench).

Since the nozzle was mounted (2026-10-07) the head camera looks about 25 deg off vertical, and it can't be
re-mounted. Square labels look like trapezoids, the scale changes across the picture, and the camera
calibration (px per mm) was refused. This tool measures the tilt with the machine and saves a correction:

    .venv/bin/python tools/camera_rectify.py           # start with a slot label in view
    .venv/bin/python tools/camera_rectify.py --no-z    # without the Z move (no height shift measured)
    .venv/bin/python tools/camera_rectify.py --refit logs/camera_rectify/<time>   # fit a run again, no machine

1. A rough camera calibration on the slot label nearest the picture centre (not checked: it is tilted).
2. The head moves over a grid (config/bench.yaml camera_rectify), ONLY to the front and towards the middle
   slots: limits.reach_from_home was measured with the camera before it was moved, so it is not used.
   Every label in view is measured at the crossing of its diagonals (perspective keeps that point).
3. A perspective map from bed mm to picture px is fitted to all of them (with lens bending, if that fits
   clearly better).
4. Z goes up z_mm and back down: from how the labels shrink and shift, the lens height and the height
   shift g: a thing h mm HIGHER than the labels, seen in the middle of the picture, is g * h mm away from
   where a label at that spot would be. The nozzle offset must allow for it (tape, parts, the board).
5. Saves config/camera_rectify.yaml and pictures in logs/camera_rectify/<time>/. From then on the bench
   tools straighten every picture: a view from the front, X to the right, Y up, the same px per mm all
   over the label plane. The rest of the code then works as with a straight camera.

Run it again whenever the camera is moved on the head. Delete config/camera_rectify.yaml for raw pictures.
Keys: space = stop after the current move, q = quit (the head stays where it is).
"""
from __future__ import annotations

import argparse
import datetime
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

import bed_test_run as t
import make_labels as ml
from bench import Grabber, Grbl, load_bench, mark_parked, was_parked
from smd_identify import DEFAULT_CAMERA

ROOT = Path(__file__).resolve().parents[1]
RECTIFY_FILE = ROOT / "config" / "camera_rectify.yaml"
LOG_DIR = ROOT / "logs" / "camera_rectify"
WINDOW = "Camera rectify"


# ---------------------------------------------------------------- geometry
def projective_centre(c) -> np.ndarray:
    """Where the diagonals of the 4 corners cross. Perspective keeps it on the marker centre (the mean of
    the corners moves towards the near side)."""
    p0, p1, p2, p3 = np.asarray(c, float).reshape(4, 2)
    d1, d2 = p2 - p0, p3 - p1
    s = np.linalg.solve(np.column_stack([d1, -d2]), p1 - p0)[0]
    return p0 + s * d1


def warp(H, pts) -> np.ndarray:
    pts = np.asarray(pts, float).reshape(-1, 2)
    q = np.column_stack([pts, np.ones(len(pts))]) @ np.asarray(H, float).T
    return q[:, :2] / q[:, 2:]


def projector(model: dict):
    """bed mm (relative to the head, +X right, +Y back) -> raw picture px, for a saved model."""
    if model["kind"] == "homography":
        H = np.asarray(model["H"], float)
        return lambda b: warp(H, b)
    K, dist = np.asarray(model["K"], float), np.asarray(model["dist"], float)
    rvec, tvec = np.asarray(model["rvec"], float), np.asarray(model["tvec"], float)

    def project(b):
        b = np.asarray(b, float).reshape(-1, 2)
        obj = np.column_stack([b, np.zeros(len(b))])
        return cv2.projectPoints(obj, rvec, tvec, K, dist)[0].reshape(-1, 2)
    return project


def seen_by(model: dict, b) -> np.ndarray:
    """True where bed point b (relative to the head) can really be in the picture: in front of the lens, and
    for the camera model inside the range where the lens bending formula holds (beyond it the formula folds
    far points back into the picture)."""
    b = np.asarray(b, float).reshape(-1, 2)
    if model["kind"] == "homography":
        H = np.asarray(model["H"], float)
        return np.column_stack([b, np.ones(len(b))]) @ H[2] > 0
    R = cv2.Rodrigues(np.asarray(model["rvec"], float))[0]
    Xc = np.column_stack([b, np.zeros(len(b))]) @ R.T + np.asarray(model["tvec"], float)
    with np.errstate(divide="ignore", invalid="ignore"):
        r2 = (Xc[:, 0] ** 2 + Xc[:, 1] ** 2) / Xc[:, 2] ** 2
    k1, k2 = model["dist"][:2]
    return (Xc[:, 2] > 0) & (1 + 3 * k1 * r2 + 5 * k2 * r2 * r2 > 0.2)


def invert(project, p, guess, iters: int = 20) -> np.ndarray:
    """The bed point(s) b with project(b) = p (Newton steps from guess)."""
    p = np.asarray(p, float).reshape(-1, 2)
    b = np.broadcast_to(np.asarray(guess, float).reshape(-1, 2), p.shape).copy()
    e = 1e-3
    for _ in range(iters):
        f0 = project(b) - p
        fx = (project(b + [e, 0]) - project(b - [e, 0])) / (2 * e)
        fy = (project(b + [0, e]) - project(b - [0, e])) / (2 * e)
        J = np.stack([fx, fy], axis=2)                       # n x 2 x 2
        b = b - np.linalg.solve(J, f0[:, :, None])[:, :, 0]
        if np.abs(f0).max() < 1e-6:
            break
    return b


def local_scale(project, b) -> float:
    """px per mm on the label plane at b (mean of the two directions)."""
    e = 0.5
    fx = (project(np.asarray(b) + [e, 0]) - project(np.asarray(b) - [e, 0]))[0] / (2 * e)
    fy = (project(np.asarray(b) + [0, e]) - project(np.asarray(b) - [0, e]))[0] / (2 * e)
    return float(np.sqrt(abs(np.linalg.det(np.column_stack([fx, fy])))))


# ---------------------------------------------------------------- fitting
# A sighting = (head position (machine mm), label name, its 4 corners (px, ArUco order)). The corners of a
# label lie at M + a * R(angle) * SQUARE on the bed: its place M, size a (~10 mm) and angle on the bed are
# fitted with the camera. The size is free, so the scale comes from the head moves only.
SQUARE = np.array([[-0.5, 0.5], [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5]])   # TL TR BR BL, +Y = back


def turn(a: float) -> np.ndarray:
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def similarity(q: np.ndarray, s: np.ndarray) -> tuple[np.ndarray, float, float]:
    """M, a, angle with q ~ M + a * R(angle) * s (least squares, no mirror)."""
    qm, sm = q.mean(axis=0), s.mean(axis=0)
    Q, S = q - qm, s - sm
    n2 = float((S * S).sum())
    c = float((S * Q).sum()) / n2
    d = float((S[:, 0] * Q[:, 1] - S[:, 1] * Q[:, 0]).sum()) / n2
    a, ang = float(np.hypot(c, d)), float(np.arctan2(d, c))
    return qm - a * turn(ang) @ sm, a, ang


def levenberg_marquardt(fun, x0, iters: int = 200) -> np.ndarray:
    """Minimise |fun(x)|^2 (numeric Jacobian, Marquardt scaling: parameters of any size)."""
    x = np.asarray(x0, float).copy()
    r = fun(x)
    cost, lam = float(r @ r), 1e-3
    for _ in range(iters):
        eps = 1e-6 * np.maximum(1.0, np.abs(x))
        J = np.empty((len(r), len(x)))
        for i in range(len(x)):
            xi = x.copy()
            xi[i] += eps[i]
            J[:, i] = (fun(xi) - r) / eps[i]
        A, g = J.T @ J, J.T @ r
        while True:
            step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            xn = x + step
            rn = fun(xn)
            cn = float(rn @ rn)
            if np.isfinite(cn) and cn < cost:
                done = cost - cn < 1e-12 * max(cost, 1e-12) or np.all(np.abs(step) < 1e-10 * np.maximum(1.0, np.abs(x)))
                x, r, cost, lam = xn, rn, cn, max(lam / 3, 1e-12)
                break
            lam *= 4
            if lam > 1e10:
                return x
        if done:
            break
    return x


class Views:
    """The sightings of the grid run, and the label poses (place M, size a, angle) in a flat vector."""

    def __init__(self, sightings: list, ref: str) -> None:
        self.ref = ref
        self.names = [ref] + sorted({n for _, n, _ in sightings} - {ref})
        self.heads = np.array([h for h, _, _ in sightings], float)
        self.which = np.array([self.names.index(n) for _, n, _ in sightings])
        self.px = np.array([np.asarray(c, float).reshape(4, 2) for _, _, c in sightings]).reshape(-1, 2)
        self.centres = np.array([projective_centre(np.asarray(c, float)) for _, _, c in sightings])

    def poses(self, v: np.ndarray) -> list[tuple[np.ndarray, float, float]]:
        return [(np.array(p[:2]), float(p[2]), float(p[3])) for p in np.asarray(v, float).reshape(-1, 4)]

    def bed_points(self, v: np.ndarray) -> np.ndarray:
        """Every corner on the bed, relative to the head at that sighting (mm)."""
        P = np.asarray(v, float).reshape(-1, 4)
        c, s = np.cos(P[:, 3]), np.sin(P[:, 3])
        R = np.stack([np.stack([c, -s], -1), np.stack([s, c], -1)], -2)                # labels x 2 x 2
        corners = P[:, None, :2] + P[:, 2, None, None] * np.einsum("kij,nj->kni", R, SQUARE)
        return (corners[self.which] - self.heads[:, None, :]).reshape(-1, 2)

    def pin(self, v: np.ndarray) -> np.ndarray:
        return 1e3 * np.asarray(v[:2])          # the ref label sits at 0: it fixes where 0 is

    def first_poses(self, unproject, near_px=None, keep: int = 6) -> np.ndarray:
        """Label poses through a camera guess (unproject: px -> bed mm relative to the head). With near_px,
        only each label's sightings nearest to those picture points are used (where the guess is good)."""
        out = []
        for k in range(len(self.names)):
            rows = np.where(self.which == k)[0]
            if near_px is not None and k > 0:
                d = np.min(np.linalg.norm(self.centres[rows][:, None, :] - near_px[None, :, :], axis=2), axis=1)
                rows = rows[np.argsort(d)[:keep]]
            q = np.vstack([unproject(self.px[4 * i: 4 * i + 4]) + self.heads[i] for i in rows])
            M, a, ang = similarity(q, np.tile(SQUARE, (len(rows), 1)))
            out += [M[0], M[1], a, ang]
        return np.array(out)

    def label_rms(self, residual_px: np.ndarray) -> dict[str, float]:
        r = residual_px.reshape(-1, 4)
        return {n: float(np.sqrt(np.mean(r[self.which == k] ** 2))) for k, n in enumerate(self.names)}


def homography_model(x: np.ndarray) -> dict:
    return {"kind": "homography", "H": np.append(x[:8], 1.0).reshape(3, 3).tolist()}


def camera_model(x: np.ndarray, size) -> dict:
    w, h = size
    f, k1, k2 = x[:3]
    return {"kind": "camera", "K": [[float(f), 0.0, (w - 1) / 2], [0.0, float(f), (h - 1) / 2], [0.0, 0.0, 1.0]],
            "dist": [float(k1), float(k2), 0.0, 0.0, 0.0], "rvec": [float(v) for v in x[3:6]],
            "tvec": [float(v) for v in x[6:9]]}


def focal_guesses(H: np.ndarray, size) -> list[float]:
    """Focal lengths (px) to start from: the two that make the homography a rotation (principal point at the
    picture centre), and a spread of usual ones."""
    w, h = size
    Hp = np.array([[1, 0, -(w - 1) / 2], [0, 1, -(h - 1) / 2], [0, 0, 1.0]]) @ H
    out = []
    with np.errstate(divide="ignore", invalid="ignore"):
        for f2 in (-(Hp[0, 0] * Hp[0, 1] + Hp[1, 0] * Hp[1, 1]) / (Hp[2, 0] * Hp[2, 1]),
                   (Hp[0, 1] ** 2 + Hp[1, 1] ** 2 - Hp[0, 0] ** 2 - Hp[1, 0] ** 2) / (Hp[2, 0] ** 2 - Hp[2, 1] ** 2)):
            if np.isfinite(f2) and 100 ** 2 < f2 < 5000 ** 2:
                out.append(float(np.sqrt(f2)))
    return out + [200.0, 300.0, 450.0, 650.0, 900.0, 1300.0, 2000.0]


def pose_from_homography(H: np.ndarray, f: float, size) -> tuple[np.ndarray, np.ndarray]:
    """rvec, tvec of a pinhole camera (focal length f, principal point at the centre) that sees the bed plane
    through homography H (bed mm -> px)."""
    w, h = size
    K = np.array([[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1.0]])
    M = np.linalg.inv(K) @ H
    lam = 2 / (np.linalg.norm(M[:, 0]) + np.linalg.norm(M[:, 1]))
    if M[2, 2] * lam < 0:                       # the bed in front of the camera
        lam = -lam
    r1, r2 = M[:, 0] * lam, M[:, 1] * lam
    U, _, Vt = np.linalg.svd(np.column_stack([r1, r2, np.cross(r1, r2)]))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = U @ np.diag([1.0, 1.0, -1.0]) @ Vt
    return cv2.Rodrigues(R)[0].ravel(), M[:, 2] * lam


def _residuals(views: Views, kind: str, size):
    def res(x):
        n = 8 if kind == "homography" else 9
        model = homography_model(x) if kind == "homography" else camera_model(x, size)
        r = (projector(model)(views.bed_points(x[n:])) - views.px).ravel()
        return np.concatenate([r, views.pin(x[n:])])
    return res


def _fit(views: Views, kind: str, x0: np.ndarray, size, iters: int = 200) -> tuple[np.ndarray, np.ndarray]:
    res = _residuals(views, kind, size)
    x = levenberg_marquardt(res, x0, iters)
    return x, np.linalg.norm(res(x)[:-2].reshape(-1, 2), axis=1)


def _camera_from_homography(views: Views, xh: np.ndarray, size) -> np.ndarray | None:
    """The camera (f, k1, k2, pose) for fixed label poses, from the homography at several focal lengths:
    some starts get stuck far off, the best one is kept."""
    H = np.append(xh[:8], 1.0).reshape(3, 3)
    b = views.bed_points(xh[8:])
    best = None
    for f0 in focal_guesses(H, size):
        rv, tv = pose_from_homography(H, f0, size)
        xc = levenberg_marquardt(lambda x: (projector(camera_model(x, size))(b) - views.px).ravel(),
                                 np.concatenate([[f0, 0.0, 0.0], rv, tv]), iters=100)
        r = float(np.sqrt(np.mean(np.sum((projector(camera_model(xc, size))(b) - views.px) ** 2, axis=1))))
        if xc[0] > 0 and np.isfinite(r) and (best is None or r < best[0]):
            best = (r, xc)
    return None if best is None else best[1]


def fit_views(sightings: list, ref: str, size) -> tuple[dict, np.ndarray, dict, dict]:
    """Fit the camera to all label corners of the grid run. Returns the model (a pinhole camera with lens
    bending k1, k2 if it fits clearly better than a plain perspective map), the corner residuals (px) of the
    labels kept, {label: (M, size, angle)} and a report (rms of both fits, labels left out)."""
    ref_rows = [s for s in sightings if s[1] == ref]
    if len(ref_rows) < 4:
        raise ValueError(f"{ref} was seen fewer than 4 times")
    # 1. the ref label alone: a homography from its centres, then with its corners, then the camera
    one = Views(ref_rows, ref)
    H0 = cv2.findHomography(-one.heads, one.centres, 0)[0]
    H0 /= H0[2, 2]
    xh = _fit(one, "homography", np.concatenate([H0.ravel()[:8], one.first_poses(lambda p: warp(np.linalg.inv(H0), p))]),
              size)[0]
    Href = np.append(xh[:8], 1.0).reshape(3, 3)
    xc = _camera_from_homography(one, xh, size)
    if xc is not None:
        xc = _fit(one, "camera", np.concatenate([xc, xh[8:]]), size)[0][:9]

    # 2. every label, placed through that model from its sightings nearest to the ref's track, then all
    # fitted together, twice (the second time the labels are placed with all their sightings)
    def unproject_with(kind, x):
        if kind == "homography":
            return lambda p: warp(np.linalg.inv(np.append(x[:8], 1.0).reshape(3, 3)), p)
        project = projector(camera_model(x, size))
        return lambda p: invert(project, p, warp(np.linalg.inv(Href), p))

    report = {"left_out": []}
    kinds = {"homography": xh[:8]} | ({} if xc is None else {"camera": xc})
    fits = {}
    for kind, xm in kinds.items():
        keep = list(sightings)
        while True:
            views = Views(keep, ref)
            n = len(xm)
            x = np.concatenate([xm, views.first_poses(unproject_with(kind, xm), near_px=one.centres)])
            x, r = _fit(views, kind, x, size)
            x, r = _fit(views, kind, np.concatenate([x[:n], views.first_poses(unproject_with(kind, x[:n]))]), size)
            per = views.label_rms(r)
            worst = max((nm for nm in per if nm != ref), key=per.get, default=None)
            if worst is None or per[worst] <= max(0.5, 3 * per[ref]):
                break
            keep = [s for s in keep if s[1] != worst]          # a label at the bent edge that does not fit
            report["left_out"].append((kind, worst, round(per[worst], 2)))
        fits[kind] = (x, r, views)
    rms = {k: float(np.sqrt(np.mean(v[1] ** 2))) for k, v in fits.items()}
    report["rms"] = rms | ({} if "camera" in rms else {"camera": None})
    kind = "camera" if "camera" in rms and rms["camera"] < 0.8 * rms["homography"] else "homography"
    x, r, views = fits[kind]
    n = 8 if kind == "homography" else 9
    model = homography_model(x) if kind == "homography" else camera_model(x[:n], size)
    report["left_out"] = [(nm, v) for k, nm, v in report["left_out"] if k == kind]
    return model, r, {nm: p for nm, p in zip(views.names, views.poses(x[n:]))}, report


def height_shift(b0, b1, side0: float, side1: float, dz: float, b_c) -> tuple[np.ndarray, float]:
    """The head went UP dz mm. A label at b0 (bed mm, measured through the Z0 map) is then seen at b1, and its
    side shrinks from side0 to side1 (k = side1/side0). Pinhole camera: the picture shrinks about the point
    below the lens n. Returns g (mm per mm: a thing h mm higher than the labels, seen at the picture centre,
    is at centre + g * h on the label plane) and the lens height above the labels (mm)."""
    k = side1 / side0
    n = (np.asarray(b1, float) - k * np.asarray(b0, float)) / (1 - k)
    lens = k * dz / (1 - k)
    return (n - np.asarray(b_c, float)) / lens, float(lens)


# ---------------------------------------------------------------- the straightened picture
class Rectifier:
    """Raw picture -> straightened picture: a view from the front (X right, Y up) with the same px per mm all
    over the label plane. The point seen in the middle of the raw picture stays in the middle."""

    def __init__(self, model: dict, size, scale: float | None = None) -> None:
        self.model, self.size = model, (int(size[0]), int(size[1]))
        self.project = projector(model)
        w, h = self.size
        self.centre = np.array([(w - 1) / 2, (h - 1) / 2])
        guess = np.zeros(2)
        if model["kind"] == "homography":
            guess = warp(np.linalg.inv(np.asarray(model["H"], float)), self.centre)[0]
        else:
            guess = invert(self.project, self.centre, guess, iters=50)[0]
        self.b_c = invert(self.project, self.centre, guess)[0]
        self.scale = float(scale) if scale else local_scale(self.project, self.b_c)
        qx, qy = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
        b = self.bed_of(np.column_stack([qx.ravel(), qy.ravel()]))
        raw = self.project(b)
        raw[~seen_by(model, b)] = -1e4             # not seen by the camera: filled from the picture edge
        mx = raw[:, 0].reshape(h, w).astype(np.float32)
        my = raw[:, 1].reshape(h, w).astype(np.float32)
        self.map1, self.map2 = cv2.convertMaps(mx, my, cv2.CV_16SC2)

    def bed_of(self, q) -> np.ndarray:
        """Straightened px -> bed mm relative to the head."""
        return self.b_c + (np.asarray(q, float).reshape(-1, 2) - self.centre) * np.array([1.0, -1.0]) / self.scale

    def bed_of_raw(self, p) -> np.ndarray:
        """Raw px -> bed mm relative to the head (label plane)."""
        p = np.asarray(p, float).reshape(-1, 2)
        return invert(self.project, p, np.tile(self.b_c, (len(p), 1)))

    def __call__(self, gray: np.ndarray) -> np.ndarray:
        return cv2.remap(gray, self.map1, self.map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

    def to_dict(self) -> dict:
        return {"model": self.model, "size": list(self.size), "scale_px_mm": float(self.scale)}


def load_rectifier(path: Path = RECTIFY_FILE, say=print) -> Rectifier | None:
    """The saved correction, or None (no file: raw pictures)."""
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    say(f"Camera pictures are straightened with {path.name} (tilted camera, measured {data.get('time', '?')}).")
    return Rectifier(data["model"], data["size"], data["scale_px_mm"])


def open_camera(device: str = DEFAULT_CAMERA, raw: bool = False) -> Grabber:
    """The bench camera, straightened with config/camera_rectify.yaml if it exists (and raw is False)."""
    return Grabber(device, rectify=None if raw else load_rectifier())


# ---------------------------------------------------------------- the job on the machine
def average_markers(m, frames: int, edge_px: float) -> dict[str, np.ndarray]:
    """Labels seen in most of `frames` fresh pictures, corners averaged. Labels at the edge are left out."""
    seen: dict[str, list] = {}
    for f in m.cam.frames_after(time.monotonic() + t.SETTLE_S, frames):
        m.frame_shape = f.shape
        h, w = f.shape
        for name, c in m.markers.find(f).items():
            if c[:, 0].min() > edge_px and c[:, 1].min() > edge_px and c[:, 0].max() < w - 1 - edge_px \
                    and c[:, 1].max() < h - 1 - edge_px:
                seen.setdefault(name, []).append(c)
    return {n: np.mean(cs, axis=0) for n, cs in seen.items() if len(cs) >= (frames + 1) // 2}


def grid_moves(cfg: dict, nominal: dict, ref: str) -> list[np.ndarray]:
    """Head moves from the start (mm): to the front, and along X towards the middle of the slot row. Serpentine."""
    slots = [xy[0] for n, xy in nominal.items() if n.startswith("slot_")]
    sx = 1.0 if np.mean(slots) > nominal[ref][0] else -1.0
    xs = [sx * float(v) for v in cfg["grid_x_mm"]]
    out = []
    for i, y in enumerate(float(v) for v in cfg["grid_y_mm"]):
        for x in (xs if i % 2 == 0 else xs[::-1]):
            out.append(np.array([x, -y]))
    return out


def job_rectify(m, cfg: dict, out_dir: Path, z_mm: float, path: Path = RECTIFY_FILE) -> dict | None:
    """The whole measurement on machine m (a bed_test_run.Machine with the RAW camera). The measurements are
    saved first (logs/camera_rectify/<time>/measurements.yaml, raw.png: --refit can fit them again without
    the machine), then fitted and checked; the correction is saved only if it passes."""
    m.say("1/5 rough camera calibration (the camera is tilted, so it is not checked)")
    m.home = None                             # the saved reach is not used: the camera was moved
    seen = m.visible()                        # no label search: it drives far to the back, and the reach is not known
    ref = m.calibration_label(seen) if seen else None
    if ref is None:
        m.say("No label in view: jog until a slot label is in the picture, then start again")
        return None
    if not m.calibrate(ref, strict=False):
        return None
    start = m.pos.copy()
    frames, edge = int(cfg["frames"]), float(cfg["edge_px"])
    sightings, moves = [], grid_moves(cfg, m.nominal, ref)
    approach = np.array([-1.0, -1.0])         # arrive at every point from the same side (play)
    for i, d in enumerate(moves):
        target = start + d
        m.move_checked(target + approach - m.pos)
        m.move(target - m.pos)
        got = average_markers(m, frames, edge)
        for name, c in got.items():
            sightings.append((m.pos.copy(), name, c))
        m.say(f"2/5 grid {i + 1}/{len(moves)}: head {d[0]:+.0f} / {d[1]:+.0f} mm, labels: "
              + (", ".join(n.upper().replace("_", " ") for n in sorted(got)) or "none"))
    m.move_checked(start + approach - m.pos)
    m.move(start - m.pos)

    zv = None
    if z_mm > 0:
        m.say(f"3/5 Z up {z_mm:g} mm and back down (lens height and height shift)")
        zv = {"z_mm": float(z_mm), "before": average_markers(m, frames, edge).get(ref)}
        try:
            m.grbl.move_axes({"Z": z_mm}, float(cfg["z_feed"]))
            zv["up"] = average_markers(m, frames, edge).get(ref)
        finally:
            m.grbl.move_axes({"Z": -z_mm}, float(cfg["z_feed"]))     # back down: arrives from above
        zv["after"] = average_markers(m, frames, edge).get(ref)
    raw = m.picture()
    meas = {"reference_label": ref, "size": [int(m.frame_shape[1]), int(m.frame_shape[0])],
            "sightings": [{"head": [float(v) for v in h], "label": n, "corners_px": np.round(c, 3).tolist()}
                          for h, n, c in sightings],
            "z_views": None if zv is None else {k: (v if k == "z_mm" or v is None else np.round(v, 3).tolist())
                                                for k, v in zv.items()}}
    (out_dir / "measurements.yaml").write_text(yaml.safe_dump(meas, sort_keys=False))
    cv2.imwrite(str(out_dir / "raw.png"), raw)
    return solve(meas, raw, cfg, m.markers, out_dir, path, m.say)


def solve(meas: dict, raw: np.ndarray, cfg: dict, markers, out_dir: Path, path: Path, say=print) -> dict | None:
    """Fit the camera to the measurements, check it, and save config/camera_rectify.yaml if it passes."""
    ref, size = meas["reference_label"], tuple(meas["size"])
    sightings = [(np.array(s["head"], float), s["label"], np.array(s["corners_px"], float)) for s in meas["sightings"]]
    say("4/5 fitting the camera (tilt and lens bending) to every label corner")
    try:
        model, res, poses, report = fit_views(sightings, ref, size)
        rms_both = report["rms"]
    except ValueError as e:
        say(f"Not enough sightings: {e}. Start with a slot label near the middle of the picture.")
        return None
    rms, worst = float(np.sqrt(np.mean(res ** 2))), float(res.max())
    rect = Rectifier(model, size)
    say(f"   {len(sightings)} sightings of {len(poses)} labels; corners fit {rms:.2f} px rms, worst {worst:.2f} px "
        f"({model['kind']}; plain perspective {rms_both['homography']:.2f} px"
        + (f", with lens bending {rms_both['camera']:.2f} px" if rms_both["camera"] is not None else "")
        + f"); straightened scale {rect.scale:.2f} px/mm")
    for name, v in report["left_out"]:
        say(f"   {name.upper().replace('_', ' ')} left out: its corners fit only {v:.2f} px (seen at the bent edge)")
    if model["kind"] == "camera":
        k1, k2 = model["dist"][:2]
        say(f"   focal length {model['K'][0][0]:.0f} px, lens bending k1 {k1:+.3f} k2 {k2:+.3f}")
    if rms > float(cfg["max_rms_px"]):
        say(f"REFUSED: the corners fit worse than {cfg['max_rms_px']} px rms (a label moved, lost steps, or the "
            "camera moved). Nothing saved.")
        return None

    g, lens, zv = None, None, meas.get("z_views")
    if zv and zv.get("before") is not None and zv.get("up") is not None:
        c0, c1 = np.array(zv["before"], float), np.array(zv["up"], float)
        back = None if zv.get("after") is None else float(np.abs(np.array(zv["after"], float) - c0).max())
        b0, b1 = rect.bed_of_raw(c0), rect.bed_of_raw(c1)

        def side(b):
            return float(np.mean(np.linalg.norm(np.roll(b, -1, axis=0) - b, axis=1)))
        g, lens = height_shift(projective_centre(b0), projective_centre(b1), side(b0), side(b1), float(zv["z_mm"]),
                               rect.b_c)
        tilt = float(np.degrees(np.arctan(np.hypot(*g))))
        say(f"   Z: lens {lens:.0f} mm above the labels, height shift {g[0]:+.3f} / {g[1]:+.3f} mm per mm "
            f"(tilt about {tilt:.0f} deg); back at Z0 the label is {back if back is not None else float('nan'):.2f} px "
            "from where it was")
        if back is None or back > float(cfg["z_return_px"]) or not 15 < lens < 300 or tilt > 50:
            say("   The Z result is not trusted (the picture did not come back, or the numbers are impossible): "
                "the height shift is not saved")
            g, lens = None, None
    elif zv:
        say(f"   {ref} not seen at both heights: no height shift measured")

    say("5/5 check: every label in the straightened picture must be square and the same size")
    straight = rect(raw)
    cv2.imwrite(str(out_dir / "straightened.png"), straight)
    checks, bad = {}, []
    for name, c in sorted(markers.find(straight).items()):
        shape = square_check(c)
        side_mm = float(shape["sides"].mean() / rect.scale)
        checks[name] = {"sides_apart_pct": round(shape["spread_pct"], 2),
                        "worst_corner_deg": round(shape["corner_err"], 2), "side_mm": round(side_mm, 3)}
        ok = shape["spread_pct"] <= float(cfg["check_spread_pct"]) and shape["corner_err"] <= float(cfg["check_corner_deg"])
        bad += [] if ok else [name]
        say(f"   {name.upper().replace('_', ' ')}: sides {shape['spread_pct']:.1f} % apart, corners within "
            f"{shape['corner_err']:.1f} deg of 90, side {side_mm:.2f} mm" + ("" if ok else "  <- NOT SQUARE"))
    sizes = [c["side_mm"] for c in checks.values()]
    if len(sizes) > 1 and (max(sizes) - min(sizes)) / np.mean(sizes) * 100 > float(cfg["check_spread_pct"]):
        bad.append("label sizes differ")
    if not checks or bad:
        say(f"REFUSED: the straightened picture is not right ({', '.join(bad) or 'no label in view'}). Nothing "
            f"saved; the measurements are in {out_dir.name}/ (--refit).")
        return None

    data = {**rect.to_dict(),
            "time": datetime.datetime.now().isoformat(timespec="seconds"),
            "reference_label": ref,
            "fit_rms_px": round(rms, 3), "fit_worst_px": round(worst, 3),
            "sightings": len(sightings), "labels": sorted(poses),
            "label_side_mm": {n: round(float(p[1]), 3) for n, p in poses.items()},
            "bed_at_picture_centre_mm": [round(float(v), 3) for v in rect.b_c],
            "height_shift_mm_per_mm": None if g is None else [round(float(v), 4) for v in g],
            "lens_height_mm": None if lens is None else round(lens, 1),
            "check_in_straightened_picture": checks,
            "measurements": str(out_dir.relative_to(ROOT) if out_dir.is_relative_to(ROOT) else out_dir)}
    path.write_text("# Written by tools/camera_rectify.py. The bench tools straighten every camera picture with it\n"
                    "# (tilted head camera, 2026-10-07). Run the tool again if the camera moves on the head;\n"
                    "# delete this file for raw pictures. height_shift_mm_per_mm = g: a thing h mm higher than\n"
                    "# the labels, seen in the middle of the picture, is g * h mm from where a label would be.\n"
                    + yaml.safe_dump(data, sort_keys=False))
    say(f"Saved {path.relative_to(ROOT) if path.is_relative_to(ROOT) else path}. The other bench tools now "
        "use straightened pictures.")
    return data


def refit(run_dir: Path, cfg: dict, path: Path = RECTIFY_FILE) -> dict | None:
    """Fit the saved measurements of an earlier run again (no machine, no camera)."""
    meas = yaml.safe_load((run_dir / "measurements.yaml").read_text())
    raw = cv2.imread(str(run_dir / "raw.png"), cv2.IMREAD_GRAYSCALE)
    return solve(meas, raw, cfg, t.Markers(ml.load_bed()), run_dir, path)


def square_check(corners) -> dict:
    import camera_square_check as csc
    return csc.marker_shape(corners)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-z", action="store_true", help="no Z move (the height shift is not measured)")
    ap.add_argument("--camera", default=DEFAULT_CAMERA)
    ap.add_argument("--refit", type=Path, metavar="RUN_DIR",
                    help="fit the measurements of an earlier run again (logs/camera_rectify/<time>): no machine")
    args = ap.parse_args()
    bench = load_bench()
    cfg = bench["camera_rectify"]
    if args.refit:
        refit(args.refit, cfg)
        return
    bed = ml.load_bed()
    labels = ml.make_all(bed)
    nominal = {name: np.array(c, float) for name, c, _ in ml.label_placements(bed, labels)}
    out_dir = LOG_DIR / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    cam = Grabber(args.camera)                     # RAW pictures: this tool measures the tilt
    parked = was_parked()                          # before the port opens (that marks it "not parked")
    grbl = Grbl(cfg=bench)
    grbl.check_ready()
    m = t.Machine(grbl, cam, t.Markers(bed), nominal, out_dir)
    m.feed = float(bench["limits"]["max_feed"])
    m.reach = None                                 # measured with the camera before it moved: not used here
    m.reach_stale = False                          # this tool only moves towards the middle of the bed (see above)
    if not parked and bench["nozzle"].get("mounted"):
        m.not_parked = ("the last program did not leave the nozzle up: nothing moves by itself. Start "
                        "tools/nozzle_calibrate.py, lift the nozzle, press h, quit with q")
    m.msg = "Measuring the camera tilt..."
    m.start()
    done = {}

    def job() -> None:
        try:
            done["result"] = job_rectify(m, cfg, out_dir, 0.0 if args.no_z else float(cfg["z_mm"]))
        finally:
            done.setdefault("result", None)          # also after an error: the window then closes by itself
    m.submit(job)
    print(f"Camera rectify. Pictures: {out_dir}", flush=True)
    cv2.namedWindow(WINDOW)
    quit_at = None
    try:
        while True:
            item = cam.latest()
            if item is not None:
                cv2.imshow(WINDOW, t.draw(item[1], m.markers.find(item[1]), m, 0))
            key = cv2.waitKeyEx(30)
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                m.stop_flag.set()
                m.say("Stopping after the current move...")
            if not m.busy and "result" in done and quit_at is None:
                quit_at = time.monotonic() + 5
            if quit_at is not None and time.monotonic() > quit_at:
                break
            try:
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    finally:
        m.stop_flag.set()
        t0 = time.monotonic()
        while m.busy and time.monotonic() - t0 < 30:
            time.sleep(0.05)
        grbl.close()
        if not m.busy:                                 # its Z move always comes back (finally)
            mark_parked(parked, "camera_rectify")
        cam.close()
        cv2.destroyAllWindows()
        print("Bye.", flush=True)


if __name__ == "__main__":
    main()
