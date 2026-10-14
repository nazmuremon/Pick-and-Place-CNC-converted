"""Camera calibration from the ChArUco sheet (docs/13 §4.5, D28).

What the operator does: lay the sheet flat on the bed under the camera ("plane 0", the label surface), then on a
block of known height (measured with calipers, "plane 1"). The head moves the camera over a grid so the sheet is
seen in every part of the picture.

What is fitted, all at once (least squares on the corner pixels):
- the lens: focal length, principal point (held near the picture centre), radial bending k1, k2;
- the camera's place and angle on the head (its tilt);
- where each sheet lies (rotation and position) — not needed afterwards;
- the head's real move per counted mm in X and Y: a steps/mm check against the printed sheet.
Plane 0 alone fixes the lens and the tilt when the camera is tilted; the block (known height) fixes the focal
length for any camera and gives the height shift (parallax) directly.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera_model import CameraModel, rodrigues
from .charuco import Detection, Detector

MIN_CORNERS = 20


@dataclass
class View:
    head: tuple[float, float, float]          # counted head X, Y (mm) and the camera's Z offset (0 unless the
                                              # camera rides on the Z slide)
    plane: int                                # 0 = sheet on the bed, 1 = sheet on the block
    det: Detection


@dataclass
class Result:
    model: CameraModel
    rms_px: float
    max_px: float
    rms_mm: dict[int, float]                  # per plane
    move_scale: tuple[float, float]           # head move per counted mm (1.0 = steps/mm right)
    views: int
    corners: int
    warnings: list[str] = field(default_factory=list)

    def report(self) -> list[str]:
        m = self.model
        out = [f"{self.views} pictures, {self.corners} corners: fit error {self.rms_px:.2f} px RMS "
               f"(worst {self.max_px:.2f} px), " + ", ".join(f"plane {p}: {v:.3f} mm" for p, v in self.rms_mm.items()),
               f"Lens: focal {m.f:.1f} px, centre ({m.c[0]:.1f}, {m.c[1]:.1f}), k1 {m.dist[0]:+.4f}, k2 {m.dist[1]:+.4f}",
               f"Camera: {m.tilt_deg:.2f}° off vertical, {m.height_mm:.1f} mm above the labels, "
               f"{m.px_per_mm(0):.3f} px/mm at the picture centre",
               f"Head moves: X {self.move_scale[0]:.4f}, Y {self.move_scale[1]:.4f} mm per counted mm"]
        return out + self.warnings


def _pack(f, c, k, rvec, tvec, planes, scale) -> np.ndarray:
    return np.r_[f, c, k, rvec, tvec, np.ravel(planes), scale]


def _unpack(p: np.ndarray, n_planes: int):
    f = p[0]
    c = p[1:3]
    k = p[3:5]
    rvec, tvec = p[5:8], p[8:11]
    planes = p[11:11 + 3 * n_planes].reshape(n_planes, 3)            # theta, sx, sy
    scale = p[11 + 3 * n_planes:13 + 3 * n_planes]
    return f, c, k, rvec, tvec, planes, scale


def _points(views: list[View], planes_z: dict[int, float]):
    """For every corner: its sheet position, plane, view index."""
    rows = []
    for i, v in enumerate(views):
        for xy in v.det.sheet_mm:
            rows.append((i, v.plane, xy[0], xy[1]))
    return np.array(rows, np.float64)


def _residuals(p, views, planes_z, plane_ids, size, prior_w) -> np.ndarray:
    f, c, k, rvec, tvec, planes, scale = _unpack(p, len(plane_ids))
    K = np.array([[f, 0, c[0]], [0, f, c[1]], [0, 0, 1]])
    D = np.array([k[0], k[1], 0, 0, 0])
    out = []
    for v in views:
        j = plane_ids.index(v.plane)
        th, sx, sy = planes[j]
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        m = v.det.sheet_mm @ R.T + (sx, sy)                          # machine position of the corners
        head = np.array([v.head[0] * scale[0], v.head[1] * scale[1]])
        X = np.c_[m - head, np.full(len(m), planes_z[v.plane] - v.head[2])]
        px, _ = cv2.projectPoints(X, rvec, tvec, K, D)
        out.append((px.reshape(-1, 2) - v.det.px).ravel())
    # weak pull of the principal point to the picture centre (it is poorly fixed by a flat target)
    out.append(prior_w * (c - np.array(size) / 2.0))
    return np.concatenate(out)


def _lm(fun, p0: np.ndarray, fixed: np.ndarray, iters: int = 60) -> np.ndarray:
    """Levenberg-Marquardt with a numerical Jacobian. fixed: True where a parameter must not move."""
    p = p0.copy()
    r = fun(p)
    cost = float(r @ r)
    lam = 1e-3
    free = np.where(~fixed)[0]
    for _ in range(iters):
        J = np.zeros((len(r), len(free)))
        for col, i in enumerate(free):
            h = 1e-6 * max(1.0, abs(p[i]))
            q = p.copy()
            q[i] += h
            J[:, col] = (fun(q) - r) / h
        A = J.T @ J
        g = J.T @ r
        improved = False
        for _ in range(10):
            try:
                step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            q = p.copy()
            q[free] += step
            rq = fun(q)
            cq = float(rq @ rq)
            if cq < cost:
                p, r, cost, lam = q, rq, cq, max(lam / 3, 1e-9)
                improved = True
                break
            lam *= 10
        if not improved or np.abs(step).max() < 1e-9:
            break
    return p


def solve(views: list[View], size: tuple[int, int], block_mm: float | None = None,
          fit_move_scale: bool = True) -> Result:
    views = [v for v in views if len(v.det.ids) >= MIN_CORNERS]
    if len([v for v in views if v.plane == 0]) < 3:
        raise ValueError("Too few pictures of the sheet on the bed (at least 3 with 20 corners each)")
    planes_z = {0: 0.0, 1: float(block_mm or 0.0)}
    plane_ids = sorted({v.plane for v in views})
    if 1 in plane_ids and not block_mm:
        raise ValueError("Pictures on the block, but no block height")
    w, h = size
    # ---- start: OpenCV's planar calibration of the bed pictures (principal point fixed at the centre)
    bed = [v for v in views if v.plane == 0]
    obj = [np.c_[v.det.sheet_mm, np.zeros(len(v.det.ids))].astype(np.float32) for v in bed]
    img = [v.det.px.astype(np.float32) for v in bed]
    K0 = cv2.initCameraMatrix2D(obj, img, size)
    if not (0.3 * w < K0[0, 0] < 5 * w):
        K0 = np.array([[0.9 * w, 0, w / 2], [0, 0.9 * w, h / 2], [0, 0, 1]])
    K0[0, 2], K0[1, 2] = w / 2, h / 2
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO |
             cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
    _, K, D, rvecs, tvecs = cv2.calibrateCamera(obj, img, size, K0, np.zeros(5), flags=flags)
    f0 = float(K[0, 0])
    # camera rotation and the sheet's rotation in the machine frame, from the head moves
    Rv = [rodrigues(r) for r in rvecs]
    tv = [np.asarray(t, float).ravel() for t in tvecs]
    a, b = [], []
    for i in range(len(bed)):
        for k in range(i + 1, len(bed)):
            d_head = np.array(bed[i].head[:2]) - np.array(bed[k].head[:2])
            if np.hypot(*d_head) < 1.0:
                continue
            a.append((Rv[i].T @ (tv[k] - tv[i]))[:2])
            b.append(d_head)
    if not a:
        raise ValueError("The head did not move between the pictures: no machine axes")
    a, b = np.array(a), np.array(b)
    # Rz(theta0)^T b = a  ->  a = Rz(-theta0) b
    ang = math.atan2(float(np.sum(b[:, 0] * a[:, 1] - b[:, 1] * a[:, 0])), float(np.sum(b[:, 0] * a[:, 0] + b[:, 1] * a[:, 1])))
    theta0 = -ang
    Rz = np.array([[math.cos(theta0), -math.sin(theta0), 0], [math.sin(theta0), math.cos(theta0), 0], [0, 0, 1]])
    R = Rv[0] @ Rz.T
    h0 = np.array(bed[0].head, float)                              # X_cam = R (X - head) + t
    t = tv[0] + R @ h0                                             # sheet origin at machine (0, 0)
    planes = [[theta0, 0.0, 0.0]]
    for pid in plane_ids[1:]:
        v = next(x for x in views if x.plane == pid)
        ok, rv, tvv = cv2.solvePnP(np.c_[v.det.sheet_mm, np.zeros(len(v.det.ids))], v.det.px, K, D)
        Rp = R.T @ rodrigues(rv)                                   # the sheet's rotation: ~ about z
        thp = math.atan2(Rp[1, 0], Rp[0, 0])
        origin = R.T @ (tvv.ravel() - t) + np.array([v.head[0], v.head[1], v.head[2]])
        planes.append([thp, origin[0], origin[1]])
    p0 = _pack(f0, [w / 2, h / 2], D.ravel()[:2], cv2.Rodrigues(R)[0].ravel(), t, planes, [1.0, 1.0])
    fixed = np.zeros(len(p0), bool)
    fixed[11 + 1] = fixed[11 + 2] = True                            # plane 0's sheet position: the gauge
    if not fit_move_scale:
        fixed[-2:] = True
    prior_w = 0.05
    fun = lambda p: _residuals(p, views, planes_z, plane_ids, size, prior_w)   # noqa: E731
    p = _lm(fun, p0, fixed)
    r = fun(p)[:-2].reshape(-1, 2)
    f, c, k, rvec, tvec, planes, scale = _unpack(p, len(plane_ids))
    model = CameraModel(size, float(f), (float(c[0]), float(c[1])), (float(k[0]), float(k[1])),
                        tuple(float(x) for x in rvec), tuple(float(x) for x in tvec))
    err = np.hypot(r[:, 0], r[:, 1])
    rms_mm = {}
    i = 0
    per_plane: dict[int, list] = {}
    for v in views:
        n = len(v.det.ids)
        per_plane.setdefault(v.plane, []).append(err[i:i + n])
        i += n
    for pid, chunks in per_plane.items():
        e = np.concatenate(chunks)
        rms_mm[pid] = float(np.sqrt(np.mean(e ** 2)) / model.px_per_mm(planes_z[pid]))
    res = Result(model.anchor(), float(np.sqrt(np.mean(err ** 2))), float(err.max()), rms_mm,
                 (float(scale[0]), float(scale[1])), len(views), int(len(err)))
    if 1 not in plane_ids and model.tilt_deg < 5:
        res.warnings.append("The camera looks almost straight down: without the block pictures its height (and so "
                            "the parallax of things above the bed) is not well known. Add the block pictures")
    for axis, s in zip("XY", scale):
        if abs(s - 1) > 0.005:
            res.warnings.append(f"The head moves {abs(s - 1) * 100:.1f} % {'more' if s > 1 else 'less'} than counted "
                                f"in {axis}: check the steps/mm of that axis (Grbl ${'100' if axis == 'X' else '101'} "
                                "or the axis map) before trusting long moves")
    if res.rms_px > 1.0:
        res.warnings.append("The fit is poor (more than 1 px): did the sheet move, is it flat, is the print 100 %?")
    res.model.quality = {"rms_px": round(res.rms_px, 4), "max_px": round(res.max_px, 4),
                         "rms_mm": {str(k_): round(v_, 4) for k_, v_ in rms_mm.items()},
                         "move_scale": [round(s, 5) for s in res.move_scale], "views": res.views,
                         "corners": res.corners, "block_mm": block_mm, "date": time.strftime("%Y-%m-%d %H:%M")}
    return res


def view_grid(dx: float = 25.0, dy: float = 15.0) -> list[tuple[float, float]]:
    """Head offsets from the start that put the sheet in every part of the picture."""
    return [(x, y) for y in (-dy, 0.0, dy) for x in (-dx, 0.0, dx)]


def collect(rig, plane: int, offsets=None, detector: Detector | None = None, say=print) -> list[View]:
    """Move the head over the grid around where it is now and take a picture at each place.
    rig: .head() -> (x, y, z) counted; .move_by(dx, dy); .frame() -> grey picture after the move."""
    det = detector or Detector()
    start = np.array(rig.head()[:2], float)
    views = []
    for ox, oy in offsets or view_grid():
        cur = np.array(rig.head()[:2], float)
        rig.move_by(*(start + (ox, oy) - cur))
        d = det.find(rig.frame())
        hx, hy, hz = rig.head()
        if d is None or len(d.ids) < MIN_CORNERS:
            say(f"   at {ox:+.0f}, {oy:+.0f}: sheet not seen well ({0 if d is None else len(d.ids)} corners)")
            continue
        views.append(View((hx, hy, hz), plane, d))
        say(f"   at {ox:+.0f}, {oy:+.0f}: {len(d.ids)} corners")
    cur = np.array(rig.head()[:2], float)
    rig.move_by(*(start - cur))
    return views


def label_sizes(model: CameraModel, corners_px: np.ndarray, z: float = 0.0) -> np.ndarray:
    """The four side lengths (mm) of a square marker seen at corners_px (4 x 2) on the plane z."""
    xy = model.px_to_xy(corners_px, z)
    return np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)
