"""Board features in a picture (docs/13 §4.5): holes and fiducials (edge-based circles, bright or dark inside)
and outline corners (two straight edges fitted and crossed). Pixel space only; the caller maps to millimetres with
the camera model at the board's top plane.

Bench facts: a mounting hole over the dark T-slot grooves shows no contrast, over the bars it is a faint lighter
disc; round pads are look-alikes (a hole is plain inside, a pad has a hole of its own or is brighter at its rim).
So: edge strength around the WHOLE circle at the expected radius, near the predicted place, and a sub-pixel fit.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


def fit_circle(points) -> tuple[np.ndarray, float]:
    """Least-squares circle through the points: centre and radius (from tools/nozzle_calibrate.py)."""
    P = np.asarray(points, float)
    sol, *_ = np.linalg.lstsq(np.c_[2 * P, np.ones(len(P))], (P ** 2).sum(axis=1), rcond=None)
    centre = sol[:2]
    return centre, float(np.sqrt(max(0.0, sol[2] + centre @ centre)))


def _gradients(gray: np.ndarray, sigma: float = 1.0):
    g = cv2.GaussianBlur(np.asarray(gray, np.float32), (0, 0), sigma)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3) / 8.0
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3) / 8.0
    return g, gx, gy


def _sample(img: np.ndarray, x, y) -> np.ndarray:
    """Bilinear samples of img at (x, y) of any shape (cv2.remap wants 2-D maps under 32k wide: rows of 1024)."""
    x, y = np.asarray(x, np.float32), np.asarray(y, np.float32)
    shape = x.shape
    n = x.size
    cols = 1024
    rows = max(1, -(-n // cols))
    fx = np.zeros(rows * cols, np.float32)
    fy = np.zeros(rows * cols, np.float32)
    fx[:n], fy[:n] = x.ravel(), y.ravel()
    out = cv2.remap(img, fx.reshape(rows, cols), fy.reshape(rows, cols), cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_REPLICATE)
    return out.ravel()[:n].reshape(shape)


@dataclass
class Circle:
    centre: np.ndarray            # px
    radius: float                 # px
    score: float                  # edge strength around the circle / typical edge strength (0..~3)
    polarity: str                 # bright | dark (inside)
    rms: float                    # px, edge points to the fitted circle
    points: int


def find_circle(gray: np.ndarray, near, r_px: float, search_px: float, polarity: str = "any",
                min_score: float = 0.6, step: float = 1.0, r_range: tuple[float, float] = (0.8, 1.25)) -> Circle | None:
    """The circle of about r_px radius whose centre is within search_px of `near`."""
    g, gx, gy = _gradients(gray)
    h, w = g.shape
    nx, ny = float(near[0]), float(near[1])
    xs = np.arange(nx - search_px, nx + search_px + 1e-9, step)
    ys = np.arange(ny - search_px, ny + search_px + 1e-9, step)
    cx, cy = np.meshgrid(xs, ys)
    cx, cy = cx.ravel(), cy.ravel()
    ang = np.linspace(0, 2 * np.pi, 64, endpoint=False)
    ca, sa = np.cos(ang), np.sin(ang)
    region = (slice(max(0, int(ny - search_px - 2 * r_px)), min(h, int(ny + search_px + 2 * r_px))),
              slice(max(0, int(nx - search_px - 2 * r_px)), min(w, int(nx + search_px + 2 * r_px))))
    typical = float(np.mean(np.hypot(gx[region], gy[region]))) + 1e-3
    best = None
    for r in np.arange(r_range[0] * r_px, r_range[1] * r_px + 1e-9, 0.5):
        px = cx[:, None] + r * ca[None, :]
        py = cy[:, None] + r * sa[None, :]
        rad = _sample(gx, px, py) * ca[None, :] + _sample(gy, px, py) * sa[None, :]
        # a disc brighter inside: the brightness falls going out, the outward gradient is negative
        m = rad.mean(axis=1)
        if polarity == "bright":
            s = -m
        elif polarity == "dark":
            s = m
        else:
            s = np.abs(m)
        # the edge must go all round (not a straight edge passing by): penalise the weakest quarter
        q = np.sort(np.abs(rad), axis=1)[:, : rad.shape[1] // 4].mean(axis=1)
        s = (s + 0.5 * q) / typical
        i = int(np.argmax(s))
        if best is None or s[i] > best[0]:
            best = (float(s[i]), cx[i], cy[i], r, "bright" if m[i] < 0 else "dark")
    if best is None or best[0] < min_score:
        return None
    score, bx, by, br, pol = best
    # sub-pixel: the strongest edge along 64 rays near the radius, then a circle fit (2 rounds, outliers out)
    ts = np.arange(0.6 * br, 1.4 * br, 0.25)
    rx = bx + ts[None, :] * ca[:, None]
    ry = by + ts[None, :] * sa[:, None]
    prof = np.abs(_sample(gx, rx, ry) * ca[:, None] + _sample(gy, rx, ry) * sa[:, None])
    k = np.argmax(prof, axis=1)
    pts = np.c_[bx + ts[k] * ca, by + ts[k] * sa]
    keep = np.ones(len(pts), bool)
    c, rr = np.array([bx, by]), br
    for _ in range(2):
        if keep.sum() < 12:
            return None
        c, rr = fit_circle(pts[keep])
        d = np.abs(np.hypot(*(pts - c).T) - rr)
        keep = d < max(1.0, 2.5 * float(np.median(d[keep])))
    rms = float(np.sqrt(np.mean((np.hypot(*(pts[keep] - c).T) - rr) ** 2)))
    if not (r_range[0] - 0.05) * r_px <= rr <= (r_range[1] + 0.05) * r_px or rms > 1.5:
        return None
    return Circle(c, float(rr), score, pol, rms, int(keep.sum()))


def find_hole(gray: np.ndarray, near, r_px: float, delta_px, search_px: float, min_score: float = 0.4) -> Circle | None:
    """A drilled hole seen at an angle; the centre of its TOP rim (radius r_px = the drill) is returned.

    Seen at an angle, every visible edge of a hole lies on one of two circles of the drill's size: the top rim
    (solder mask -> the hole's light wall or the bed) and the bottom rim (the wall -> the bed), shifted by delta_px
    (camera model x board thickness). Either can be faint in places (a groove as dark as the mask). So a centre is
    scored by how many clear edges lie on EITHER circle (consensus, not strength: an edge inside the hole, a bar
    over a groove, is strong but in the wrong place), then fitted on the matched points of both circles.
    Refused when the top rim's matched points do not wrap far enough round to fix the centre."""
    delta = np.asarray(delta_px, float)
    g, gx, gy = _gradients(gray)
    h, w = g.shape
    n_ang = 96
    ang = np.linspace(0, 2 * np.pi, n_ang, endpoint=False)
    ca, sa = np.cos(ang), np.sin(ang)
    two = np.hypot(*delta) >= 0.3
    nx, ny = float(near[0]), float(near[1])
    region = (slice(max(0, int(ny - search_px - 2 * r_px)), min(h, int(ny + search_px + 2 * r_px))),
              slice(max(0, int(nx - search_px - 2 * r_px)), min(w, int(nx + search_px + 2 * r_px))))
    typical = float(np.mean(np.hypot(gx[region], gy[region]))) + 1e-3

    def rims(r):
        base = np.c_[r * ca, r * sa]
        nrm = np.c_[ca, sa]
        if not two:
            return base, nrm, np.zeros(n_ang, bool)
        return np.r_[base, base + delta], np.r_[nrm, nrm], np.r_[np.zeros(n_ang, bool), np.ones(n_ang, bool)]

    def edges(cxs, cys, pts, nrm, span, step):
        """For every centre and boundary point: offset (px, along the normal) of the clear edge nearest the rim."""
        ts = np.arange(-span, span + 1e-9, step)
        px = cxs[:, None, None] + pts[None, :, None, 0] + ts[None, None, :] * nrm[None, :, None, 0]
        py = cys[:, None, None] + pts[None, :, None, 1] + ts[None, None, :] * nrm[None, :, None, 1]
        prof = np.abs(_sample(gx, px, py) * nrm[None, :, None, 0] + _sample(gy, px, py) * nrm[None, :, None, 1])
        is_max = np.zeros_like(prof, bool)
        is_max[..., 1:-1] = (prof[..., 1:-1] >= prof[..., :-2]) & (prof[..., 1:-1] >= prof[..., 2:])
        clear = is_max & (prof > 0.6 * typical)
        dist = np.where(clear, np.abs(ts)[None, None, :], np.inf)
        k = np.argmin(dist, axis=2)
        ok = np.isfinite(np.take_along_axis(dist, k[..., None], axis=2)[..., 0])
        kk = np.clip(k, 1, len(ts) - 2)
        a_ = np.take_along_axis(prof, (kk - 1)[..., None], axis=2)[..., 0]
        b_ = np.take_along_axis(prof, kk[..., None], axis=2)[..., 0]
        c_ = np.take_along_axis(prof, (kk + 1)[..., None], axis=2)[..., 0]
        den = a_ - 2 * b_ + c_
        sub = np.where(np.abs(den) > 1e-9, 0.5 * (a_ - c_) / np.where(np.abs(den) > 1e-9, den, 1.0), 0.0)
        return np.where(ok, ts[kk] + np.clip(sub, -0.5, 0.5) * step, np.nan)

    xs = np.arange(nx - search_px, nx + search_px + 1e-9, 1.0)
    ys = np.arange(ny - search_px, ny + search_px + 1e-9, 1.0)
    cx, cy = (v.ravel() for v in np.meshgrid(xs, ys))
    best = None
    for r in np.arange(0.95 * r_px, 1.05 * r_px + 1e-9, 0.5):    # the drill size is known
        pts, nrm, _ = rims(r)
        off = edges(cx, cy, pts, nrm, 0.35 * r, 0.5)
        sc = np.mean(np.abs(off) <= 1.0, axis=1)
        i = int(np.argmax(sc))
        if best is None or sc[i] > best[0]:
            best = (float(sc[i]), np.array([cx[i], cy[i]]), r)
    if best is None or best[0] < min_score:
        return None
    score, c, r = best
    pts, nrm, is_bot = rims(r)
    for span in (2.0, 1.25, 0.75):                               # px: narrower each round
        off = edges(np.array([c[0]]), np.array([c[1]]), pts, nrm, span + 1.0, 0.25)[0]
        keep = np.isfinite(off) & (np.abs(off) <= span)
        if keep.sum() < 12:
            return None
        sol, *_ = np.linalg.lstsq(nrm[keep], off[keep], rcond=None)   # only the centre moves
        c = c + sol
    res = off[keep] - nrm[keep] @ sol
    rms = float(np.sqrt(np.mean(res ** 2)))
    # the matched points must wrap round: a short arc can't fix the centre (and may be something else)
    top_ang = ang[keep[:n_ang]] if keep[:n_ang].any() else np.zeros(0)
    all_ang = np.r_[top_ang, ang[keep[n_ang:]]] if two else top_ang
    if len(all_ang) < 12 or _largest_gap(all_ang) > np.radians(150):
        return None
    if rms > 1.0:
        return None
    return Circle(c, float(r), score, "?", rms, int(keep.sum()))


def _largest_gap(angles: np.ndarray) -> float:
    a = np.sort(np.mod(angles, 2 * np.pi))
    gaps = np.diff(np.r_[a, a[0] + 2 * np.pi])
    return float(gaps.max())


@dataclass
class Line:
    point: np.ndarray
    direction: np.ndarray         # unit
    rms: float
    points: int


@dataclass
class Corner:
    point: np.ndarray             # px
    angle_deg: float              # between the two edges
    edges: tuple[Line, Line]


def edge_points(gray: np.ndarray, start, along, across_px: float, length_px: float, skip_px: float,
                stations: int = 16, inner=None, double_px: float = 0.0) -> np.ndarray:
    """Points of a straight edge that runs from near `start` in the direction `along`: at `stations` places
    from skip_px to length_px along it, the strongest change across it within ±across_px (one sign only).
    inner: a direction pointing into the board. Seen at an angle a board shows its top edge AND, on the sides that
    face the camera, its bottom edge double_px further out: when two edges of the same sign sit that far apart,
    the inner one (the top) is taken."""
    g, gx, gy = _gradients(gray, 1.0)
    a = np.asarray(along, float)
    a = a / np.linalg.norm(a)
    n = np.array([-a[1], a[0]])
    if inner is not None and float(np.dot(n, np.asarray(inner, float))) < 0:
        n = -n                                                   # n points into the board
    ts = np.linspace(skip_px, length_px, stations)
    ss = np.arange(-across_px, across_px + 1e-9, 0.25)
    px = start[0] + ts[:, None] * a[0] + ss[None, :] * n[0]
    py = start[1] + ts[:, None] * a[1] + ss[None, :] * n[1]
    d = _sample(gx, px, py) * n[0] + _sample(gy, px, py) * n[1]
    # either sign: along a board edge the bed changes (bright bar, dark groove), so the edge flips sign
    sd = np.abs(d)
    top = float(np.percentile(sd.max(axis=1), 75))
    pts = []
    for i in range(len(ts)):
        row = sd[i]
        peaks = [j for j in range(1, len(row) - 1) if row[j] >= row[j - 1] and row[j] >= row[j + 1] and row[j] > 0.3 * top]
        if not peaks:
            continue
        j = max(peaks, key=lambda q: row[q])
        if inner is not None and double_px > 1.0:
            # a second edge about double_px further OUT (smaller s) than another strong one: keep the inner one
            for q in peaks:
                if q != j and abs((ss[q] - ss[j]) - double_px) < 1.0 and row[q] > 0.5 * row[j]:
                    j = q
                    break
        sub = 0.0
        if 0 < j < len(row) - 1:
            den = row[j - 1] - 2 * row[j] + row[j + 1]
            sub = float(np.clip(0.5 * (row[j - 1] - row[j + 1]) / den, -0.5, 0.5)) if abs(den) > 1e-9 else 0.0
        s_ = ss[j] + sub * 0.25
        pts.append((start[0] + ts[i] * a[0] + s_ * n[0], start[1] + ts[i] * a[1] + s_ * n[1]))
    return np.array(pts).reshape(-1, 2)


def fit_line(pts: np.ndarray, rounds: int = 3) -> Line | None:
    """A straight line through the points, robust: start from the median, drop the far ones, refit."""
    if len(pts) < 5:
        return None
    keep = np.ones(len(pts), bool)
    for _ in range(rounds):
        P = pts[keep]
        c = np.median(P, axis=0) if _ == 0 else P.mean(axis=0)
        _, _, vt = np.linalg.svd(P - c)
        d = vt[0]
        nrm = np.array([-d[1], d[0]])
        res = np.abs((pts - c) @ nrm)
        keep = res < max(1.0, 3.0 * float(np.median(res[keep])))
        if keep.sum() < 5:
            return None
    P = pts[keep]
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c)
    d = vt[0]
    rms = float(np.sqrt(np.mean(((P - c) @ np.array([-d[1], d[0]])) ** 2)))
    return Line(c, d, rms, int(keep.sum()))


def intersect(a: Line, b: Line) -> np.ndarray | None:
    A = np.c_[a.direction, -b.direction]
    if abs(np.linalg.det(A)) < 1e-6:
        return None
    t = np.linalg.solve(A, b.point - a.point)
    return a.point + t[0] * a.direction


def find_corner(gray: np.ndarray, near, dir1, dir2, search_px: float = 14.0, length_px: float = 60.0,
                skip_px: float = 6.0, max_rms: float = 1.2, delta_px=None) -> Corner | None:
    """The outline corner near `near` where the board's two edges run off in the directions dir1 and dir2.
    delta_px: where the board's bottom surface shows relative to its top (camera model x thickness)."""
    lines = []
    for d, other in ((dir1, dir2), (dir2, dir1)):
        dbl = 0.0
        if delta_px is not None:
            a = np.asarray(d, float) / np.linalg.norm(d)
            nrm = np.array([-a[1], a[0]])
            if float(np.dot(nrm, other)) < 0:
                nrm = -nrm                                       # into the board
            dbl = float(-np.dot(np.asarray(delta_px, float), nrm))   # > 0: the bottom edge shows outside
        pts = edge_points(gray, near, d, search_px, length_px, skip_px, inner=other, double_px=dbl)
        ln = fit_line(pts)
        if ln is None or ln.rms > max_rms:
            return None
        lines.append(ln)
    p = intersect(*lines)
    if p is None or np.hypot(*(p - np.asarray(near, float))) > 2 * search_px:
        return None
    ang = math.degrees(math.acos(min(1.0, abs(float(lines[0].direction @ lines[1].direction)))))
    if not 60 <= ang <= 120:
        return None
    return Corner(p, ang, (lines[0], lines[1]))
