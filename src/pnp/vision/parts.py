"""Parts in a cut-tape strip (docs/13 §4.5, bug 5.7): model-based, not threshold-based.

1. The strip is straightened into a top view in the SLOT frame (u along the label from the marker, v across; the
   pocket row on v = 0, the sprocket holes on v = −3.5 for 8 mm tape), from the camera model or — for a picture
   without a calibration — from the slot marker's four corners alone.
2. Only the pocket row is searched, at the tape's pitch: sprocket holes (another row) can't be taken as parts.
3. A part and an empty pocket can look alike to a 5 px/mm camera (bench 2026-10-11: a part 57 % darker than the
   tape, an empty pocket 51 %): no fixed threshold is trusted. Without a template, a pocket that is not plain tape
   is only a CANDIDATE and the operator confirms the first part of the line (docs/13 §4.4). Its picture becomes
   the line's template; from then on every pocket is matched against it (normalized correlation: on the bench
   strip parts match 0.89–0.94, empty pockets 0.70–0.73).
4. Every result says present / candidate / empty / unclear, with a score and a reason. Only present is picked.

The pick check compares the pocket before and after a pick, and refuses to judge when the rest of the picture
changed too (a hand in view, the light, the camera moved): docs/13 §5.6 — never switch the pump off on a guess.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import cv2
import numpy as np

RES = 10.0                                    # px per mm of the straightened strip


@dataclass
class StripSpec:
    body: tuple[float, float]                 # package length, width (mm): the long side lies across the tape
    pitch: float = 4.0
    u_range: tuple[float, float] = (17.0, 77.0)   # the tape mark (slot frame)
    v_row: float = 0.0                        # the pocket row
    hole_row: float = -3.5                    # the sprocket holes (8 mm tape): 3.5 mm from the pocket row (EIA-481)
    hole_d: float = 1.5                       # sprocket hole diameter
    hole_offset: float = 2.0                  # pocket centre to the nearest hole along the tape (EIA-481 P2)
    across: bool = True                       # the part's long side across the tape (EIA-481 for chips: 90°)


@dataclass
class Found:
    u: float                                  # pocket centre along the tape (mm, slot frame)
    v: float
    angle: float                              # long side direction, degrees from +u (≈ 90 across the tape)
    state: str                                # present | candidate (confirm the first part) | empty | unclear
    score: float
    why: str = ""
    xy: np.ndarray | None = None              # machine / head-relative position, when the caller maps it


def homography_from_marker(corners_px: np.ndarray, size_mm: float) -> np.ndarray:
    """Slot frame (u, v) mm on the label plane -> pixels, from the slot marker's corners (docs/07 slot frame: corner
    0 -> 1 is +u, corner 0 -> 3 is +v; the sprocket holes are on −v). Perspective only (no lens bending)."""
    h = size_mm / 2
    uv = np.array([[-h, -h], [h, -h], [h, h], [-h, h]], np.float32)
    H, _ = cv2.findHomography(uv, np.asarray(corners_px, np.float32))
    return H


def straighten(gray: np.ndarray, to_px: Callable[[np.ndarray], np.ndarray], spec: StripSpec,
               v_half: float = 7.0, res: float = RES) -> tuple[np.ndarray, Callable]:
    """The strip as a top view: rows = v (from +v_half down to −v_half), columns = u. Returns the picture and the
    function (col, row) -> (u, v)."""
    u0, u1 = spec.u_range
    us = np.arange(u0, u1, 1.0 / res)
    vs = np.arange(v_half, -v_half, -1.0 / res)
    uu, vv = np.meshgrid(us, vs)
    px = to_px(np.c_[uu.ravel(), vv.ravel()]).reshape(len(vs), len(us), 2).astype(np.float32)
    img = cv2.remap(np.asarray(gray, np.float32), px[..., 0], px[..., 1], cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=-1)

    def back(col, row):
        return u0 + col / res, v_half - row / res
    return img, back


def homography_mapper(H: np.ndarray) -> Callable[[np.ndarray], np.ndarray]:
    def f(uv):
        return cv2.perspectiveTransform(np.asarray(uv, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)
    return f


def _box_mask(shape, cu, cv_, lu, lv, res) -> np.ndarray:
    m = np.zeros(shape, bool)
    r0, r1 = int(round(cv_ - lv * res / 2)), int(round(cv_ + lv * res / 2))
    c0, c1 = int(round(cu - lu * res / 2)), int(round(cu + lu * res / 2))
    m[max(0, r0):max(0, r1) + 1, max(0, c0):max(0, c1) + 1] = True
    return m


def _local_std(g: np.ndarray, valid: np.ndarray, kw: int, kh: int) -> np.ndarray:
    m = cv2.boxFilter(g, -1, (kw, kh), normalize=True, borderType=cv2.BORDER_REFLECT)
    m2 = cv2.boxFilter(g * g, -1, (kw, kh), normalize=True, borderType=cv2.BORDER_REFLECT)
    sd = np.sqrt(np.maximum(m2 - m * m, 0.0))
    ok = cv2.boxFilter(valid.astype(np.float32), -1, (kw, kh), normalize=True) > 0.999
    return np.where(ok, sd, 0.0)


def _hole_row(g: np.ndarray, valid: np.ndarray, spec: StripSpec, res: float, rows: range, pitch_tol: float = 0.0):
    """The sprocket hole row: round spots of the hole's size at the tape's pitch (dark over a dark bed, or light).
    Returns (row, phase, quality 0..1) or None. The holes fix where the pockets are (EIA-481)."""
    r = spec.hole_d / 2 * res
    n = int(round(4 * r)) | 1
    yy, xx = np.mgrid[:n, :n] - n // 2
    disk = (np.hypot(xx, yy) <= r).astype(np.float32)
    t = disk - disk.mean()
    resp = np.abs(cv2.matchTemplate(g.astype(np.float32), t, cv2.TM_CCOEFF_NORMED))
    resp = np.pad(resp, ((n // 2, n // 2), (n // 2, n // 2)))
    resp[~valid] = 0
    w = g.shape[1]
    best = None
    pitches = spec.pitch * res * (1 + np.linspace(-pitch_tol, pitch_tol, 9 if pitch_tol else 1))
    for row in rows:
        if row < 0 or row >= resp.shape[0]:
            continue
        prof = resp[row]
        for p, ph in ((p, ph) for p in pitches for ph in np.arange(0, p, 1.0)):
            idx = np.round(np.arange(ph, w, p)).astype(int)
            idx = idx[(idx >= n) & (idx < w - n)]
            if len(idx) < 4:
                continue
            q = float(np.percentile(prof[idx], 75))             # the strip may cover only part of the row
            if best is None or q > best[2]:
                best = (row, ph, q, p)
    return best if best is not None and best[2] > 0.5 else None


def find(img: np.ndarray, back: Callable, spec: StripSpec, template: np.ndarray | None = None,
         res: float = RES, match: float = 0.8, row_search_mm: float = 4.0, pitch_tol: float = 0.0) -> list[Found]:
    """The pockets of a straightened strip, at the tape's pitch. Missing picture (outside the view) = unclear.
    The pocket row is searched within ±row_search_mm of spec.v_row (the strip is never exactly on its mark).
    pitch_tol: allow this much scale error (a picture straightened without the camera calibration)."""
    L, W = spec.body
    lu, lv = (W, L) if spec.across else (L, W)                  # the part's size along u and along v
    h, w = img.shape
    valid = img >= 0
    g = cv2.GaussianBlur(np.where(valid, img, 0).astype(np.float32), (0, 0), 0.6)
    kw, kh = max(3, int(round(lu * res))), max(3, int(round(lv * res)))
    S = _local_std(g, valid, kw, kh)                            # texture of a part-sized window
    p = spec.pitch * res
    row_nom = (back(0, 0)[1] - spec.v_row) * res
    best = None
    hole_nom = row_nom + (spec.v_row - spec.hole_row) * res        # rows grow towards −v
    holes = _hole_row(g, valid, spec, res, range(int(hole_nom - row_search_mm * res),
                                                 int(hole_nom + row_search_mm * res) + 1), pitch_tol)
    if holes is not None:                                       # the holes fix the pocket row and phase
        hr, hph, _, p = holes
        k = p / (spec.pitch * res)                              # the picture's scale error, if any
        best = (1.0, int(round(hr - (spec.v_row - spec.hole_row) * res * k)), (hph + spec.hole_offset * res * k) % p)
    for row in ([] if best is not None else
                np.arange(row_nom - row_search_mm * res, row_nom + row_search_mm * res + 1, 1.0)):
        r = int(round(row))
        if r < kh // 2 or r >= h - kh // 2:
            continue
        prof = S[r]
        for ph in np.arange(0, p, 1.0):
            peaks = np.round(np.arange(ph, w, p)).astype(int)
            vall = np.round(np.arange(ph + p / 2, w, p)).astype(int)
            peaks, vall = peaks[peaks < w], vall[vall < w]
            score = float(prof[peaks].mean() - prof[vall].mean())
            if best is None or score > best[0]:
                best = (score, r, ph)
    if best is None:
        return []
    _, row0, phase = best
    tape_level = float(np.median(g[max(0, row0 - kh):row0 + kh][:, np.round(np.arange(phase + p / 2, w, p)).astype(int)
                                  .clip(0, w - 1)][valid[max(0, row0 - kh):row0 + kh][:, np.round(
                                      np.arange(phase + p / 2, w, p)).astype(int).clip(0, w - 1)]])) if valid.any() else 128.0
    out: list[Found] = []
    for c in np.arange(phase, w, p):
        if c < kw / 2 or c > w - kw / 2:
            continue
        cc, rr, state, score, why, ang = _pocket(g, valid, S, c, row0, lu, lv, tape_level, res, template, match)
        u, v = back(cc, rr)
        out.append(Found(round(u, 4), round(v, 4), ang, state, round(score, 3), why))
    return out


def _pocket(g, valid, S, c, row0, lu, lv, tape, res, template, match):
    """(col, row, state, score, why, angle) for the pocket near column c."""
    h, w = g.shape
    r = 0.8 * res                                               # refine ±0.8 mm around the grid position
    kw, kh = max(3, int(round(lu * res))), max(3, int(round(lv * res)))
    c0, c1 = int(max(0, c - r)), int(min(w, c + r + 1))
    r0, r1 = int(max(0, row0 - r)), int(min(h, row0 + r + 1))
    R0, R1 = max(0, r0 - kh), min(h, r1 + kh + 1)
    C0, C1 = max(0, c0 - kw), min(w, c1 + kw + 1)
    if valid[R0:R1, C0:C1].mean() < 0.98:
        return c, row0, "unclear", 0.0, "not all of the pocket is in the picture", 90.0
    if template is not None:
        win = g[R0:R1, C0:C1].astype(np.float32)
        if win.shape[0] < template.shape[0] or win.shape[1] < template.shape[1]:
            return c, row0, "unclear", 0.0, "the pocket is too small in the picture", 90.0
        m = cv2.matchTemplate(win, template.astype(np.float32), cv2.TM_CCOEFF_NORMED)
        _, best, _, (lx, ly) = cv2.minMaxLoc(m)
        dx = _parabola(m[ly, max(0, lx - 1):lx + 2]) if 0 < lx < m.shape[1] - 1 else 0.0
        dy = _parabola(m[max(0, ly - 1):ly + 2, lx]) if 0 < ly < m.shape[0] - 1 else 0.0
        cc = C0 + lx + dx + (template.shape[1] - 1) / 2
        rr = R0 + ly + dy + (template.shape[0] - 1) / 2
        if best < match:
            return cc, rr, "empty", float(best), f"does not look like the line's first part ({best:.2f} < {match})", 90.0
        return cc, rr, "present", float(best), "", _angle(g, cc, rr, kw, kh)
    # no template yet: is it plain tape, or something pocket- or part-like?
    dev = np.abs(g[R0:R1, C0:C1] - tape)
    boxsum = cv2.boxFilter(dev, -1, (kw, kh), normalize=True, borderType=cv2.BORDER_CONSTANT)
    lim = np.zeros_like(boxsum, bool)
    lim[r0 - R0:r1 - R0, c0 - C0:c1 - C0] = True
    pr, pc = np.unravel_index(int(np.argmax(np.where(lim, boxsum, -1))), boxsum.shape)
    cc, rr = C0 + pc, R0 + pr
    texture = float(boxsum[pr, pc]) / max(tape, 20.0)
    if texture < 0.08:
        return cc, rr, "empty", texture, "plain tape here", 90.0
    # a box sliding over a pocket a little bigger than the part has a flat top: its argmax sits at one end. The
    # centroid of the difference from the tape is symmetric: the pocket's middle (the template is cut there).
    y0_, x0_ = max(0, rr - kh), max(0, cc - kw)
    patch = np.abs(g[y0_:rr + kh + 1, x0_:cc + kw + 1] - tape)
    wts = np.where(patch > 0.3 * patch.max(), patch, 0.0)
    if wts.sum() > 0:
        ys, xs = np.mgrid[0:patch.shape[0], 0:patch.shape[1]]
        cc, rr = x0_ + (wts * xs).sum() / wts.sum(), y0_ + (wts * ys).sum() / wts.sum()
    return cc, rr, "candidate", texture, "confirm the first part of this line (yes / no)", _angle(g, cc, rr, kw, kh)


def _parabola(v: np.ndarray) -> float:
    """Sub-pixel offset of the peak of three values."""
    if len(v) != 3:
        return 0.0
    den = v[0] - 2 * v[1] + v[2]
    return float(0.5 * (v[0] - v[2]) / den) if abs(den) > 1e-9 else 0.0


def _angle(g, cc, rr, kw, kh) -> float:
    """The long side's direction (degrees from +u) from the second moments around the box's own median."""
    r0, c0 = int(round(rr - kh / 2)), int(round(cc - kw / 2))
    box = g[max(0, r0):r0 + kh + 1, max(0, c0):c0 + kw + 1]
    if box.size < 9:
        return 90.0
    d = np.abs(box - float(np.median(box)))
    ys, xs = np.mgrid[0:box.shape[0], 0:box.shape[1]]
    wts = d.ravel() ** 2
    if wts.sum() <= 0:
        return 90.0
    mx, my = (wts * xs.ravel()).sum() / wts.sum(), (wts * ys.ravel()).sum() / wts.sum()
    cov = np.cov(np.vstack([xs.ravel() - mx, ys.ravel() - my]), aweights=wts)
    long_dir = np.linalg.eigh(cov)[1][:, 1]
    return math.degrees(math.atan2(-long_dir[1], long_dir[0])) % 180.0      # rows grow downwards = −v


def learn_template(img: np.ndarray, f: Found, spec: StripSpec, v_half: float = 7.0, res: float = RES) -> np.ndarray:
    """The picture of a part the operator confirmed (straightened), to recognise the next parts of the line."""
    col, row = (f.u - spec.u_range[0]) * res, (v_half - f.v) * res
    L, W = spec.body
    lu, lv = (W, L) if spec.across else (L, W)
    hw, hh = int(round((lu / 2 + 0.3) * res)), int(round((lv / 2 + 0.3) * res))
    if row - hh - 1 < 0 or col - hw - 1 < 0 or row + hh + 1 > img.shape[0] - 1 or col + hw + 1 > img.shape[1] - 1:
        raise ValueError("the confirmed part is at the edge of the view: confirm one nearer the middle")
    # cut at the part's exact (sub-pixel) centre: a template off by half a pixel moves every later match with it
    t = cv2.getRectSubPix(img.astype(np.float32), (2 * hw + 1, 2 * hh + 1), (float(col), float(row)))
    if (t < 0).any():
        raise ValueError("the confirmed part is not all in the picture")
    return t


# ---------------------------------------------------------------- the pick check
@dataclass
class PickCheck:
    verdict: str                     # picked | missed | unclear
    change: float                    # grey levels inside the part's outline
    elsewhere: float                 # grey levels in the rest of the picture
    why: str = ""
    numbers: dict = field(default_factory=dict)


def pick_check(before: np.ndarray, after: np.ndarray, centre_px, half_px: tuple[float, float],
               limit: float = 20.0, elsewhere_limit: float = 8.0) -> PickCheck:
    """Did the part leave its pocket? Measured inside the part's own outline (80 % of its size) around centre_px
    on the same view before and after. The rest of the picture must not have changed (a hand in view, the light,
    the camera): then the answer is 'unclear' and the pump stays as it is."""
    a = cv2.GaussianBlur(np.asarray(before, np.float32), (0, 0), 0.8)
    b = cv2.GaussianBlur(np.asarray(after, np.float32), (0, 0), 0.8)
    cx, cy = (int(round(v)) for v in centre_px)
    hu, hv = max(2, int(round(half_px[0]))), max(2, int(round(half_px[1])))
    box = (slice(cy - hv, cy + hv + 1), slice(cx - hu, cx + hu + 1))
    inside = float(np.abs(a[box] - b[box]).mean())
    mask = np.ones(a.shape, bool)
    mask[cy - 4 * hv: cy + 4 * hv + 1, cx - 4 * hu: cx + 4 * hu + 1] = False        # the pocket and its neighbours
    h, w = a.shape
    mask[: h // 10] = mask[-h // 10:] = False                    # the nozzle's shadow / the picture edges
    elsewhere = float(np.median(np.abs(a - b)[mask])) if mask.any() else 0.0
    big = float(np.mean(np.abs(a - b)[mask] > 25)) if mask.any() else 0.0
    nums = {"before": round(float(a[box].mean()), 1), "after": round(float(b[box].mean()), 1),
            "changed_share": round(big, 3)}
    if elsewhere > elsewhere_limit or big > 0.05:
        return PickCheck("unclear", inside, elsewhere, "the rest of the picture changed too (a hand in view, the light, "
                         "the camera?): look at the pocket", nums)
    if inside > limit:
        return PickCheck("picked", inside, elsewhere, "", nums)
    return PickCheck("missed", inside, elsewhere, "the pocket did not change", nums)


def find_cells(img: np.ndarray, back: Callable, spec: StripSpec, us: list[float], template: np.ndarray | None = None,
               res: float = RES, match: float = 0.8) -> list[Found]:
    """Pockets at KNOWN places on the row v = 0 (a matrix tray's row, a stick's pick point, Phase G): no row or
    pitch search, each place checked like a strip's pocket (template when confirmed, else candidate)."""
    L, W = spec.body
    lu, lv = (W, L) if spec.across else (L, W)
    h, w = img.shape
    valid = img >= 0
    g = cv2.GaussianBlur(np.where(valid, img, 0).astype(np.float32), (0, 0), 0.6)
    kw, kh = max(3, int(round(lu * res))), max(3, int(round(lv * res)))
    S = _local_std(g, valid, kw, kh)
    row0 = (back(0, 0)[1] - spec.v_row) * res
    ring = g[max(0, int(row0) - kh):int(row0) + kh + 1]
    okr = valid[max(0, int(row0) - kh):int(row0) + kh + 1]
    level = float(np.median(ring[okr])) if okr.any() else 128.0       # the tray's own colour around the pockets
    out = []
    for u in us:
        c = (u - spec.u_range[0]) * res
        cc, rr, state, score, why, ang = _pocket(g, valid, S, c, row0, lu, lv, level, res, template, match)
        uu, vv = back(cc, rr)
        out.append(Found(round(uu, 4), round(vv, 4), ang, state, round(score, 3), why))
    return out
