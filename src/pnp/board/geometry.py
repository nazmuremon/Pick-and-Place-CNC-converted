"""Small 2D helpers for the board importers: shapes as polygons, transforms, hulls, chaining outline pieces."""
from __future__ import annotations

import math

Point = tuple[float, float]
ARC_SEGMENTS = 8            # per quarter circle


def rot(p: Point, deg: float) -> Point:
    """Rotate counter-clockwise (Y up) by deg."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)


def norm_deg(a: float) -> float:
    """-180 < a <= 180, rounded to 1e-6 so that 180.0000001 and -179.9999999 agree."""
    a = math.fmod(a, 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    a = round(a, 6)
    return 180.0 if a == -180.0 else a + 0.0


def angle_diff(a: float, b: float) -> float:
    return abs(norm_deg(a - b))


def circle(cx: float, cy: float, r: float, n: int = 4 * ARC_SEGMENTS) -> list[Point]:
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def rect(w: float, h: float) -> list[Point]:
    return [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]


def roundrect(w: float, h: float, r: float) -> list[Point]:
    r = max(0.0, min(r, w / 2, h / 2))
    if r <= 1e-9:
        return rect(w, h)
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180),
                       (w / 2 - r, -h / 2 + r, 270)):
        for i in range(ARC_SEGMENTS + 1):
            a = math.radians(a0 + 90 * i / ARC_SEGMENTS)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def oval(w: float, h: float) -> list[Point]:
    return roundrect(w, h, min(w, h) / 2)


def arc_points(start: Point, mid: Point, end: Point, seg_deg: float = 2.0) -> list[Point]:
    """Points along the circle through start, mid, end (start and end included)."""
    (x1, y1), (x2, y2), (x3, y3) = start, mid, end
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-12:
        return [start, end]
    ux = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / d
    uy = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / d
    r = math.hypot(x1 - ux, y1 - uy)
    a1, a2, a3 = (math.atan2(y - uy, x - ux) for x, y in (start, mid, end))
    ccw = (a2 - a1) % (2 * math.pi) < (a3 - a1) % (2 * math.pi)
    span = (a3 - a1) % (2 * math.pi) if ccw else -((a1 - a3) % (2 * math.pi))
    n = max(2, int(abs(math.degrees(span)) / seg_deg) + 1)
    return [(ux + r * math.cos(a1 + span * i / n), uy + r * math.sin(a1 + span * i / n)) for i in range(n + 1)]


def arc_center(start: Point, center: Point, end: Point, ccw: bool, seg_deg: float = 2.0) -> list[Point]:
    """Arc from start to end around center (Gerber G02/G03). A full circle when start == end."""
    r = math.hypot(start[0] - center[0], start[1] - center[1])
    a1 = math.atan2(start[1] - center[1], start[0] - center[0])
    a2 = math.atan2(end[1] - center[1], end[0] - center[0])
    span = (a2 - a1) % (2 * math.pi) if ccw else -((a1 - a2) % (2 * math.pi))
    if abs(span) < 1e-9:
        span = 2 * math.pi if ccw else -2 * math.pi
    n = max(2, int(abs(math.degrees(span)) / seg_deg) + 1)
    return [(center[0] + r * math.cos(a1 + span * i / n), center[1] + r * math.sin(a1 + span * i / n))
            for i in range(n + 1)]


def hull(points: list[Point]) -> list[Point]:
    """Convex hull, counter-clockwise (Andrew's monotone chain)."""
    pts = sorted(set((round(x, 6), round(y, 6)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[Point] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def area(poly: list[Point]) -> float:
    """Signed area (positive = counter-clockwise)."""
    return 0.5 * sum(poly[i - 1][0] * poly[i][1] - poly[i][0] * poly[i - 1][1] for i in range(len(poly)))


def centroid(poly: list[Point]) -> Point:
    """Area centroid of a simple polygon (the mean of the points if it has no area)."""
    a = area(poly)
    if abs(a) < 1e-12:
        return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))
    cx = cy = 0.0
    for i in range(len(poly)):
        x0, y0 = poly[i - 1]
        x1, y1 = poly[i]
        f = x0 * y1 - x1 * y0
        cx += (x0 + x1) * f
        cy += (y0 + y1) * f
    return (cx / (6 * a), cy / (6 * a))


def bbox(points) -> tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)


def inside(p: Point, poly: list[Point]) -> bool:
    x, y = p
    n, ok = len(poly), False
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i - 1], poly[i]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            ok = not ok
    return ok


def chain(pieces: list[list[Point]], tol: float = 0.01) -> list[list[Point]]:
    """Join open polylines whose ends meet into closed loops. Loops that don't close are kept as they are."""
    open_ = [list(p) for p in pieces if len(p) >= 2]
    loops: list[list[Point]] = []

    def near(a: Point, b: Point) -> bool:
        return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol

    while open_:
        cur = open_.pop(0)
        grown = True
        while grown and not near(cur[0], cur[-1]):
            grown = False
            for i, p in enumerate(open_):
                if near(cur[-1], p[0]):
                    cur += p[1:]
                elif near(cur[-1], p[-1]):
                    cur += p[::-1][1:]
                elif near(cur[0], p[-1]):
                    cur = p[:-1] + cur
                elif near(cur[0], p[0]):
                    cur = p[::-1][:-1] + cur
                else:
                    continue
                open_.pop(i)
                grown = True
                break
        if near(cur[0], cur[-1]) and len(cur) > 3:
            cur = cur[:-1]
        loops.append(cur)
    return loops
