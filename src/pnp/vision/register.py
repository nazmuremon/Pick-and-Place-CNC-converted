"""Board registration (docs/13 §4.5, D9 widened): where the board lies on the bed.

Features, best first: fiducials -> mounting holes -> outline corners -> the operator's clicks. Each is looked for
near where the guess puts it, measured on the board's TOP plane (the camera is tilted: height matters), and kept
only when the finder is sure. Features outside the camera's reach are skipped with that reason, not an error.
A board is rigid: the fit is turn + shift (rigid), from 2 features on. The machine's own axis scale is measured
by the camera calibration, so it is not fitted here. (An affine fit on 4 features near one corner matched them to
0.04 mm and was 1.2 mm off on the far side of the board, simulator 2026-10-12.) With 4 or more features an affine
fit is made too, only to report scale and skew: a "stretched" board points to a wrong feature. Every feature's
residual is kept. A feature that does not fit the others is left out and named.
machine = M · view(side, board point) + t   (board point in the Board model frame, view = turned over for B).
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import numpy as np

from ..board import geometry as bg
from ..board.model import Board

MAX_RESIDUAL_MM = 0.25


@dataclass
class Feature:
    kind: str                         # fiducial | hole | corner | click
    name: str
    board_xy: tuple[float, float]     # Board model frame (top view)
    predicted: tuple[float, float] | None = None
    measured: tuple[float, float] | None = None
    quality: float = 0.0
    why: str = ""                     # why it was not used
    extra: dict = field(default_factory=dict)   # corner: the two edge directions (view frame)


@dataclass
class Registration:
    side: str
    M: list[list[float]]
    t: list[float]
    kind: str                         # rigid | affine
    residuals: dict[str, float]
    rms: float
    max: float
    scale: tuple[float, float]
    angle_deg: float
    skew_deg: float
    features: list[Feature]
    notes: list[str] = field(default_factory=list)

    def to_machine(self, board: Board, p) -> np.ndarray:
        return np.asarray(self.M) @ np.asarray(board.view(self.side, p), float) + np.asarray(self.t)

    def to_board(self, board: Board, m) -> np.ndarray:
        """The inverse: a machine point (mm from HOME) -> board model coordinates."""
        v = np.linalg.solve(np.asarray(self.M, float), np.asarray(m, float) - np.asarray(self.t, float))
        return np.asarray(board.view(self.side, (float(v[0]), float(v[1]))), float)

    def to_dict(self) -> dict:
        d = asdict(self)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> Registration:
        d = dict(d)
        d["features"] = [Feature(**f) for f in d.get("features", [])]
        d["scale"] = tuple(d["scale"])
        return cls(**d)

    def report(self) -> list[str]:
        used = [f for f in self.features if f.measured is not None and f.name in self.residuals]
        head = (f"{self.kind} fit on {len(used)} features: residual {self.rms:.3f} mm RMS, worst {self.max:.3f} mm; "
                f"board turned {self.angle_deg:+.2f}°")
        if self.kind == "affine" or len(used) >= 5:   # from 4 nearby features the scale and skew are only noise
            head += f", scale X {self.scale[0]:.4f} Y {self.scale[1]:.4f}, skew {self.skew_deg:+.2f}°"
        out = [head]
        out += [f"  {f.name} ({f.kind}): {self.residuals[f.name]:.3f} mm" for f in used]
        out += [f"  {f.name} ({f.kind}): not used: {f.why}" for f in self.features if f.name not in self.residuals]
        return out + self.notes


def rotation(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


TOP_EDGE_TURN = {"back": 0.0, "left": 90.0, "front": 180.0, "right": -90.0}


def guess(board: Board, side: str, top_edge: str, centre_xy) -> tuple[np.ndarray, np.ndarray]:
    """The plan: the board's top edge (view +Y) points to `top_edge` of the bed, its middle at centre_xy."""
    M = rotation(TOP_EDGE_TURN[top_edge])
    c = np.array([board.width / 2, board.height / 2])
    return M, np.asarray(centre_xy, float) - M @ c


def features_of(board: Board, side: str) -> list[Feature]:
    """Fiducials on the side that is up, the mounting holes, and the outline's square corners."""
    out = [Feature("fiducial", f.ref or f"FID{i + 1}", (f.x, f.y), extra={"d": f.diameter})
           for i, f in enumerate(board.fiducials) if f.side == side]
    out += [Feature("hole", h.ref or f"hole{i + 1}", (h.x, h.y), extra={"d": h.diameter})
            for i, h in enumerate(board.holes)]
    if board.outline:
        lp = board.outline[0]
        n = len(lp)
        for i in range(n):
            p, a, b = np.array(lp[i]), np.array(lp[i - 1]), np.array(lp[(i + 1) % n])
            d1, d2 = a - p, b - p
            if np.hypot(*d1) < 3 or np.hypot(*d2) < 3:
                continue
            cos = float(d1 @ d2 / (np.hypot(*d1) * np.hypot(*d2)))
            if abs(cos) < 0.26:                                  # 75°..105°: a square corner
                out.append(Feature("corner", f"corner{len([f for f in out if f.kind == 'corner']) + 1}",
                                   (float(p[0]), float(p[1])),
                                   extra={"dir1": (d1 / np.hypot(*d1)).tolist(), "dir2": (d2 / np.hypot(*d2)).tolist()}))
    return out


def fit(board: Board, side: str, feats: list[Feature], affine: bool = False,
        max_residual: float = MAX_RESIDUAL_MM) -> Registration | None:
    """affine=True fits an affine transform (4+ features) instead of the rigid one: only for a board that really
    shrinks or for diagnosis."""
    used = [f for f in feats if f.measured is not None and not f.why]
    if len(used) < 2:
        return None
    while True:
        src = np.array([board.view(side, f.board_xy) for f in used], float)
        dst = np.array([f.measured for f in used], float)
        if affine and len(used) >= 4:                             # 3 points fit an affine exactly: no check left
            X = np.c_[src, np.ones(len(src))]
            sol, *_ = np.linalg.lstsq(X, dst, rcond=None)
            M, t, kind = sol[:2].T, sol[2], "affine"
        else:
            M, t = _similarity(src, dst, scale=False)
            kind = "rigid"
        res = np.hypot(*(src @ M.T + t - dst).T)
        if res.max() <= max_residual or len(used) <= 3:
            break
        worst = int(np.argmax(res))                              # the odd one out (a pad taken for a hole...)
        used[worst].why = f"did not fit the others ({res[worst]:.2f} mm)"
        used = [f for f in used if not f.why]
    Md = M
    if kind == "rigid" and len(used) >= 4:                       # scale and skew from an affine fit, to report only
        X = np.c_[src, np.ones(len(src))]
        Md = np.linalg.lstsq(X, dst, rcond=None)[0][:2].T
    sx, sy = float(np.hypot(*Md[:, 0])), float(np.hypot(*Md[:, 1]))
    angle = math.degrees(math.atan2(M[1, 0], M[0, 0]))
    skew = math.degrees(math.acos(np.clip(Md[:, 0] @ Md[:, 1] / (sx * sy), -1, 1))) - 90.0
    reg = Registration(side, M.tolist(), t.tolist(), kind, {f.name: round(float(r), 4) for f, r in zip(used, res)},
                       float(np.sqrt(np.mean(res ** 2))), float(res.max()), (sx, sy), angle, skew, feats)
    if res.max() > max_residual:
        reg.notes.append(f"Worst residual {res.max():.2f} mm is more than {max_residual} mm: check the features")
    if len(used) >= 5 and (abs(sx - 1) > 0.01 or abs(sy - 1) > 0.01 or abs(skew) > 1.0):
        reg.notes.append("The board looks stretched or skewed: a feature may be wrong, or the axes' steps/mm are off")
    if len(used) == 2:
        reg.notes.append("Only 2 features: no check left over. Confirm the drawn pads on the live picture")
    return reg


def _similarity(src: np.ndarray, dst: np.ndarray, scale: bool = True) -> tuple[np.ndarray, np.ndarray]:
    """Turn + shift (+ one scale when scale=True) least squares (Umeyama / Kabsch in 2D)."""
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - ms, dst - md
    H = a.T @ b
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, d])
    R = Vt.T @ D @ U.T
    k = float((S * np.diag(D)).sum() / (a ** 2).sum()) if scale else 1.0
    M = k * R
    return M, md - M @ ms


def run(rig, board: Board, side: str, M0: np.ndarray, t0: np.ndarray, model, z_top: float,
        reach=None, say=print, kinds=("fiducial", "hole", "corner"), affine: bool = False,
        first_search_mm: float = 6.0, search_mm: float = 2.0) -> Registration | None:
    """Visit the features, best kind first; refit after each one so the next search is closer.
    rig: .head() -> (x, y) machine mm from HOME; .move_to(x, y); .frames(n) -> pictures after the move.
    reach: (xmin, xmax, ymin, ymax) for the camera centre, or None."""
    from . import features as vf
    feats = [f for f in features_of(board, side) if f.kind in kinds]
    feats.sort(key=lambda f: kinds.index(f.kind))
    M, t = np.asarray(M0, float), np.asarray(t0, float)
    ppm = model.px_per_mm(z_top)
    for f in feats:
        n_meas = sum(x.measured is not None for x in feats)
        search = (first_search_mm if n_meas < 2 else search_mm) * ppm    # rough until two features fix the turn
        pred = M @ np.asarray(board.view(side, f.board_xy), float) + t
        f.predicted = (float(pred[0]), float(pred[1]))
        if reach is not None and not (reach[0] <= pred[0] <= reach[1] and reach[2] <= pred[1] <= reach[3]):
            f.why = "outside the camera's reach"
            say(f"   {f.name}: outside the camera's reach, skipped")
            continue
        found, target = None, pred
        for attempt in range(2):                                  # measured again from straight above it
            found = _measure(rig, model, board, side, f, target, M, z_top, ppm, search if attempt == 0 else 0.6 * ppm)
            if found is None or attempt == 1:
                break
            head = np.asarray(rig.head(), float)[:2]
            xy = head + model.px_to_xy([found], z_top)[0]
            if np.hypot(*(xy - head)) < 0.5 or (reach is not None and not (reach[0] <= xy[0] <= reach[1]
                                                                          and reach[2] <= xy[1] <= reach[3])):
                break
            target = xy
        head = np.asarray(rig.head(), float)[:2]
        if found is None:
            f.why = "not found clearly (hidden, over a groove, or far from the plan)"
            say(f"   {f.name}: not found clearly")
            continue
        f.measured = tuple(float(v) for v in head + model.px_to_xy([found], z_top)[0])
        say(f"   {f.name}: {np.hypot(*(np.array(f.measured) - pred)):.2f} mm from the plan")
        reg = fit(board, side, feats, affine=affine)
        if reg is not None:
            M, t = np.asarray(reg.M), np.asarray(reg.t)
        elif n_meas == 0:                                         # one feature: shift the plan by what it showed
            t = t + (np.asarray(f.measured) - pred)
    return fit(board, side, feats, affine=affine)


def _measure(rig, model, board, side, f, pred, M, z_top, ppm, search):
    """One feature in pictures taken with the camera over `pred`: its pixel position, or None."""
    from . import features as vf
    rig.move_to(*pred)
    img = np.mean(np.asarray(rig.frames(5), np.float32), axis=0)
    head = np.asarray(rig.head(), float)[:2]
    near = model.xy_to_px([np.asarray(pred) - head], z_top)[0]
    rel = np.asarray(pred) - head
    if f.kind in ("hole", "fiducial"):
        r_px = f.extra["d"] / 2 * ppm
        if f.kind == "hole":
            delta = model.xy_to_px([rel], z_top - board.thickness)[0] - model.xy_to_px([rel], z_top)[0]
            c = vf.find_hole(img, near, r_px, delta, search)
        else:
            c = vf.find_circle(img, near, r_px, search, polarity="bright")
        if c is not None:
            f.quality = c.score
            return c.centre
        return None
    def px_dir(d):
        q = M @ np.asarray(_view_dir(board, side, d), float)
        a = model.xy_to_px([rel, rel + 3 * q / np.hypot(*q)], z_top)
        return a[1] - a[0]
    delta = model.xy_to_px([rel], z_top - board.thickness)[0] - model.xy_to_px([rel], z_top)[0]
    c = vf.find_corner(img, near, px_dir(f.extra["dir1"]), px_dir(f.extra["dir2"]), search, 10 * ppm + search,
                       1.2 * ppm + search, delta_px=delta)
    if c is None:
        return None
    f.quality = 1.0 / (1.0 + c.edges[0].rms + c.edges[1].rms)
    return c.point


def _view_dir(board: Board, side: str, d) -> np.ndarray:
    """A direction in the Board frame, as seen from `side` (turned over left-right for B)."""
    d = np.asarray(d, float)
    return d if side == "F" else np.array([-d[0], d[1]])
