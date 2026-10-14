"""The camera's exposure (docs/13 §3.5, §4.5): set once for the light at hand, locked, and checked before every
calibration step and job. A lamp added by the operator once made 35 % of the picture pure white."""
from __future__ import annotations

from typing import Callable

import numpy as np


def light(gray: np.ndarray) -> dict:
    """How bright the picture is, away from its edges: the brightest 5 %, the median, the share of pure white."""
    f = np.asarray(gray, np.float32)
    h, w = f.shape[:2]
    mid = f[h // 10: h * 9 // 10, w // 10: w * 9 // 10]
    return {"p95": round(float(np.percentile(mid, 95)), 1), "median": round(float(np.median(mid)), 1),
            "white_pct": round(float(100 * np.mean(mid >= 250)), 2)}


def changed(now: dict, ref: dict | None, tol: float = 0.15) -> str | None:
    """A reason the light is not the one the exposure was set for, or None."""
    if ref is None:
        return "The exposure was never set for this light: run Auto exposure (Machine tab)"
    for k in ("p95", "median"):
        if ref.get(k) and abs(now[k] - ref[k]) > tol * ref[k]:
            return (f"The light changed (picture {k} {now[k]:.0f}, was {ref[k]:.0f}): run Auto exposure (Machine tab) "
                    "before measuring")
    if now["white_pct"] > max(3.0, 2 * ref.get("white_pct", 0.0)):
        return f"{now['white_pct']:.1f} % of the picture is pure white (glare): move the lamp or run Auto exposure"
    return None


def auto(set_exposure: Callable[[float], bool], grab: Callable[[], np.ndarray], start: float = 100.0,
         target: float = 200.0, tol: float = 12.0, max_white: float = 3.0, say=lambda s: None,
         steps: int = 14) -> tuple[float | None, dict]:
    """Find the exposure that puts the brightest 5 % just under white (bisection on a log scale)."""
    e, lo, hi, best = float(start), None, None, None
    seen = None
    for _ in range(steps):
        if not set_exposure(round(e)):
            return None, {}
        seen = light(grab())
        say(f"   exposure {e:.0f}: bright 5 % at {seen['p95']:.0f}, median {seen['median']:.0f}, "
            f"white {seen['white_pct']:.1f} %")
        if abs(seen["p95"] - target) <= tol and seen["white_pct"] < max_white:
            return round(e), seen
        if seen["p95"] > target or seen["white_pct"] >= max_white:
            hi = e
        else:
            lo = e
        best = e
        e = (lo * hi) ** 0.5 if lo and hi else (e / 2 if hi == e else e * 1.6)
        if e < 1 or e > 20000:
            break
    if best is not None and seen is not None and seen["p95"] > 20:
        set_exposure(round(best))
        return round(best), seen
    return None, seen or {}
