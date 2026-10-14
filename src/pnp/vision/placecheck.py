"""Where did the part land? (docs/13 §4.7 "verified", §8 placement accuracy.)

A plain before / after difference is biased: where the part lies on its pads the picture hardly changes. So the
check is model-based: the part's own picture (the template the operator confirmed in the tray) is pasted into the
BEFORE picture at a candidate position and angle, and the candidate that best explains the AFTER picture wins.
Everything is done in a top view of the part's top plane (the camera model, any tilt).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .camera_model import CameraModel

RES = 10.0                           # px per mm of the top view (the strip template's resolution)


@dataclass
class Placed:
    xy: np.ndarray                   # the part's centre, mm (machine frame)
    dtheta: float                    # its turn from the expected angle, degrees
    score: float                     # 1 = the pasted part explains the after picture perfectly
    gain: float                      # how much the part explains (0: nothing changed)


def _top_view(img, model: CameraModel, head, z: float, centre, half: float, res: float = RES):
    n = int(round(2 * half * res))
    ax = (np.arange(n) + 0.5) / res - half
    xx, yy = np.meshgrid(ax, -ax)                                    # rows = −Y (picture-like)
    pts = np.c_[xx.ravel() + centre[0], yy.ravel() + centre[1]]
    px = model.xy_to_px(pts - np.asarray(head, float), z).reshape(n, n, 2).astype(np.float32)
    out = cv2.remap(np.asarray(img, np.float32), px[..., 0], px[..., 1], cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=-1)
    return out, xx, yy


def measure_placed(before, after, model: CameraModel, head, z_top: float, template: np.ndarray,
                   u_vec, v_vec, body: tuple[float, float], centre_guess, search_mm: float = 1.0,
                   search_deg: float = 8.0) -> Placed | None:
    """template: the part in the strip's top view (rows = −v, cols = +u, 10 px/mm, centred on the part).
    u_vec, v_vec: the slot frame axes at the pick, turned by the turn made (where the template's axes point now).
    body: (length, width); the template's long side runs along v when the part lay across the tape."""
    half = max(body) / 2 + search_mm + 0.8
    B, xx, yy = _top_view(before, model, head, z_top, centre_guess, half)
    A, _, _ = _top_view(after, model, head, z_top, centre_guess, half)
    if (B < 0).any() or (A < 0).any():
        return None
    th, tw = template.shape
    u_vec, v_vec = np.asarray(u_vec, float), np.asarray(v_vec, float)
    L, W = body
    lu, lv = W / 2, L / 2                                             # the body in the template (across the tape)
    base = float(np.sum((A - B) ** 2))
    if base <= 1e-6:
        return None

    def cost(dx, dy, dth):
        c = math.cos(math.radians(-dth)), math.sin(math.radians(-dth))
        px, py = xx - dx, yy - dy                                         # relative to the guess
        rx, ry = c[0] * px - c[1] * py, c[1] * px + c[0] * py
        uo = rx * u_vec[0] + ry * u_vec[1]
        vo = rx * v_vec[0] + ry * v_vec[1]
        inside = (np.abs(uo) <= lu) & (np.abs(vo) <= lv)
        mx = (uo * RES + tw / 2 - 0.5).astype(np.float32)
        my = (-vo * RES + th / 2 - 0.5).astype(np.float32)
        t = cv2.remap(template.astype(np.float32), mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        pred = np.where(inside, t, B)
        return float(np.sum((pred - A) ** 2))

    best = (math.inf, 0.0, 0.0, 0.0)
    for dth in np.arange(-search_deg, search_deg + 1e-9, 1.0):
        for dx in np.arange(-search_mm, search_mm + 1e-9, 0.1):
            for dy in np.arange(-search_mm, search_mm + 1e-9, 0.1):
                e = cost(dx, dy, dth)
                if e < best[0]:
                    best = (e, dx, dy, dth)
    for step, dstep in ((0.04, 0.5), (0.01, 0.2)):                    # refine around the best
        e0, x0, y0, t0 = best
        for dth in t0 + np.arange(-3, 4) * dstep:
            for dx in x0 + np.arange(-3, 4) * step:
                for dy in y0 + np.arange(-3, 4) * step:
                    e = cost(dx, dy, dth)
                    if e < best[0]:
                        best = (e, dx, dy, dth)
    e, dx, dy, dth = best
    unexplained = e / base
    return Placed(np.asarray(centre_guess, float) + [dx, dy], float(dth), float(max(0.0, 1 - unexplained)),
                  float(1 - unexplained))
