#!/usr/bin/env python3
"""Make the bed labels on one A4 sheet of sticker paper or on 18 mm tape (decisions D11-D17).

The labels (settings in config/bed.yaml), every marker 10 mm:
  HOME       ArUco 49. Stick it once and never move it: every position is measured from it.
  PCB 1..4   ArUco 30-33. Only made when pcb_area.corner_labels is true (D17: not used, out of reach).
  SLOT n     ArUco 40-47 (slot 1..8; D17 uses 3..7) + the slot number + a tape mark. Peel the film
             off a cut-tape strip and lay it in the mark: holes on the side the top of the number
             points to, the row of parts on the centre marks. The slots sit about 30 mm apart.

    .venv/bin/python tools/make_labels.py                  # A4 sheet (PDF)
    .venv/bin/python tools/make_labels.py --tape --print   # or: the same labels on the PT-D410, 18 mm tape

Writes hardware/print/bed_labels_a4.pdf (print it at 100 %, "actual size", on matte sticker
paper), a picture of the page, and hardware/print/bed_map.png (where to stick each label).
Check the two 100 mm bars on the sheet with a ruler before you cut the labels out.
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
BED_CONFIG = ROOT / "config" / "bed.yaml"
OUT_DIR = ROOT / "hardware" / "print"
PDF_NAME = "bed_labels_a4.pdf"
PTOUCH = shutil.which("ptouch-print") or "/usr/local/bin/ptouch-print"

# Helvetica-Bold advance widths (1/1000 em) for the characters used in label text.
BOLD_WIDTH = {" ": 278, "B": 722, "C": 722, "E": 667, "H": 722, "M": 833, "O": 778, "P": 667,
              **{d: 556 for d in "0123456789"}}
CAP_HEIGHT = 0.718   # Helvetica cap height (em)


def load_bed(path: Path = BED_CONFIG) -> dict:
    return yaml.safe_load(path.read_text())


# ---------------------------------------------------------------- shapes (mm, origin top-left, y down)

@dataclass(frozen=True)
class Rect:          # filled black
    x: float
    y: float
    w: float
    h: float


@dataclass(frozen=True)
class Text:          # black text; y = baseline
    text: str
    x: float
    y: float
    em: float
    bold: bool = True


@dataclass(frozen=True)
class CutLine:       # thin grey outline: cut here
    x: float
    y: float
    w: float
    h: float


@dataclass
class Label:
    name: str
    w: float
    h: float
    shapes: list = field(default_factory=list)
    marker_id: int | None = None
    marker_xy: tuple[float, float] | None = None     # marker top-left corner in the label

    def moved(self, dx: float, dy: float) -> list:
        out = [CutLine(dx, dy, self.w, self.h)]
        for s in self.shapes:
            if isinstance(s, Text):
                out.append(Text(s.text, s.x + dx, s.y + dy, s.em, s.bold))
            else:
                out.append(type(s)(s.x + dx, s.y + dy, s.w, s.h))
        return out


def text_width(text: str, em: float) -> float:
    return sum(BOLD_WIDTH.get(ch, 600) for ch in text) / 1000 * em


def marker_shapes(dictionary, marker_id: int, x: float, y: float, size: float) -> list[Rect]:
    """The black cells of an ArUco marker (4 x 4 bits + 1 border cell = 6 x 6 cells)."""
    bits = cv2.aruco.generateImageMarker(dictionary, marker_id, 6, borderBits=1)
    cell = size / 6
    return [Rect(x + c * cell, y + r * cell, cell, cell) for r in range(6) for c in range(6) if bits[r, c] < 128]


def marker_label(bed: dict, dictionary, name: str, marker_id: int, text: str) -> Label:
    m, q = float(bed["markers"]["size_mm"]), float(bed["markers"]["quiet_mm"])
    h = m + 2 * q
    em = 0.6 * h
    x_text = q + m + 2.0
    lab = Label(name, x_text + text_width(text, em) + q, h, marker_id=marker_id, marker_xy=(q, q))
    lab.shapes += marker_shapes(dictionary, marker_id, q, q, m)
    lab.shapes.append(Text(text, x_text, h / 2 + CAP_HEIGHT * em / 2, em))
    return lab


@dataclass(frozen=True)
class SlotLayout:
    marker_x: float
    marker_y: float
    digit_x: float
    digit_w: float
    zone_x: float
    zone_y: float
    zone_w: float
    zone_h: float
    w: float
    h: float


def slot_layout(bed: dict) -> SlotLayout:
    m, q = float(bed["markers"]["size_mm"]), float(bed["markers"]["quiet_mm"])
    ft = bed["feed_tray"]
    h, zone_w, zone_h = float(ft["label_height_mm"]), float(ft["zone_length_mm"]), float(ft["zone_height_mm"])
    if h < m + 2 * q or h < zone_h:
        raise ValueError("feed_tray.label_height_mm is too small for the marker or the zone")
    digit_x, digit_w = q + m + 2.0, 8.0
    zone_x = digit_x + digit_w + 2.0
    return SlotLayout(q, (h - m) / 2, digit_x, digit_w, zone_x, (h - zone_h) / 2, zone_w, zone_h, zone_x + zone_w + q, h)


def slot_zone_mm(bed: dict) -> np.ndarray:
    """The 4 corners of a slot's part zone, in mm, in the slot marker's own frame.

    Marker frame: origin = marker centre, +u = from the marker's corner 0 to corner 1 (towards
    the zone), +v = from corner 0 to corner 3. So the zone follows the label at any angle.
    """
    lay = slot_layout(bed)
    m = float(bed["markers"]["size_mm"])
    cx, cy = lay.marker_x + m / 2, lay.marker_y + m / 2
    u0, u1 = lay.zone_x - cx, lay.zone_x + lay.zone_w - cx
    v0, v1 = lay.zone_y - cy, lay.zone_y + lay.zone_h - cy
    return np.array([(u0, v0), (u1, v0), (u1, v1), (u0, v1)])


def slot_label(bed: dict, dictionary, number: int, marker_id: int) -> Label:
    lay = slot_layout(bed)
    m = float(bed["markers"]["size_mm"])
    lab = Label(f"slot_{number}", lay.w, lay.h, marker_id=marker_id, marker_xy=(lay.marker_x, lay.marker_y))
    lab.shapes += marker_shapes(dictionary, marker_id, lay.marker_x, lay.marker_y, m)
    em = min(lay.digit_w / (BOLD_WIDTH["0"] / 1000), 0.6 * lay.h)
    digit = str(number)
    lab.shapes.append(Text(digit, lay.digit_x + (lay.digit_w - text_width(digit, em)) / 2,
                           lay.h / 2 + CAP_HEIGHT * em / 2, em))
    # Corner ticks show the zone. They sit on the zone edge, inside the detector's search margin.
    arm, t = 2.0, 0.3
    x0, y0, x1, y1 = lay.zone_x, lay.zone_y, lay.zone_x + lay.zone_w, lay.zone_y + lay.zone_h
    for x, y, sx, sy in [(x0, y0, 0, 0), (x1, y0, 1, 0), (x1, y1, 1, 1), (x0, y1, 0, 1)]:
        lab.shapes.append(Rect(x - sx * arm, y - sy * t, arm, t))
        lab.shapes.append(Rect(x - sx * t, y - sy * arm, t, arm))
    # Centre marks just outside both zone ends: the row of parts in the tape goes on this line.
    cy, mark_t, mark_len = lay.zone_y + lay.zone_h / 2, 0.5, 1.5
    lab.shapes.append(Rect(x0 - mark_len - 0.3, cy - mark_t / 2, mark_len, mark_t))
    lab.shapes.append(Rect(x1 + 0.3, cy - mark_t / 2, mark_len, mark_t))
    return lab


def make_all(bed: dict) -> dict[str, Label]:
    mk = bed["markers"]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, mk["dictionary"]))
    labels = {"home": marker_label(bed, dictionary, "home", int(mk["home_id"]), "HOME")}
    if bed["pcb_area"].get("corner_labels", True):
        for i, marker_id in enumerate(mk["pcb_corner_ids"], start=1):
            labels[f"pcb_{i}"] = marker_label(bed, dictionary, f"pcb_{i}", int(marker_id), f"PCB {i}")
    for n in slot_numbers(bed):
        labels[f"slot_{n}"] = slot_label(bed, dictionary, n, int(mk["slot_ids"][n - 1]))
    return labels


def slot_numbers(bed: dict) -> list[int]:
    """The slot numbers in use (the printed number = position in markers.slot_ids, from 1)."""
    ft = bed["feed_tray"]
    return [int(n) for n in ft.get("slot_numbers", range(1, int(ft["slots"]) + 1))]


# ---------------------------------------------------------------- the A4 page

def page_layout(bed: dict, labels: dict[str, Label]) -> tuple[list, dict[str, tuple[float, float]]]:
    """All shapes of the page, and where each label's top-left corner is (page mm)."""
    pw, ph = bed["print"]["page_mm"]
    bar = float(bed["print"]["check_bar_mm"])
    margin, gap = 12.0, 7.0
    shapes: list = [
        Text("Bed labels - pick and place robot", margin, 18, 6.0),
        Text("1. Print at 100 % (Actual size), NOT 'fit to page'. Matte sticker paper.", margin, 26, 3.4, False),
        Text(f"2. Measure both bars: each must be {bar:g} mm (+-0.5). If not, the printer scaled the page.", margin, 31,
             3.4, False),
        Text("3. Cut along the thin grey lines. Stick flat on the bed as in hardware/print/bed_map.png.", margin, 36,
             3.4, False),
        Text("Tape: film off, strip in the slot mark, holes on the side the top of the number points to, parts on "
             "the centre marks.", margin, 41, 3.4, False),
    ]
    slots = [lab for name, lab in labels.items() if name.startswith("slot_")]
    shapes.append(Text(f"ArUco {bed['markers']['dictionary']}, {bed['markers']['size_mm']:g} mm.  HOME = "
                       f"{bed['markers']['home_id']}, PCB 1-4 = {bed['markers']['pcb_corner_ids'][0]}-"
                       f"{bed['markers']['pcb_corner_ids'][-1]}, slots 1-{len(slots)} = {slots[0].marker_id}-"
                       f"{slots[-1].marker_id}.  Made by tools/make_labels.py", margin, ph - 10, 2.8, False))
    where: dict[str, tuple[float, float]] = {}
    y = 50.0
    rows = (len(slots) + 1) // 2
    for i, lab in enumerate(slots):           # 2 columns
        col, row = i // rows, i % rows
        where[lab.name] = (margin + col * (lab.w + gap), y + row * (lab.h + gap))
    y = max(where[s.name][1] + s.h for s in slots) + gap + 4
    vx, vy = pw - margin - 6, y               # the down bar: right of the HOME / PCB labels
    x = margin
    for name in [n for n in ("home", "pcb_1", "pcb_2", "pcb_3", "pcb_4") if n in labels]:
        lab = labels[name]
        if x + lab.w > vx - 10:
            x, y = margin, y + lab.h + gap
        where[name] = (x, y)
        x += lab.w + gap
    for name, (lx, ly) in where.items():
        shapes += labels[name].moved(lx, ly)

    # Check bars: across (below the labels) and down (right side), with a tick every 10 mm.
    t = 0.5
    bx, by = margin, y + labels["home"].h + 20
    shapes.append(Rect(bx, by - t / 2, bar, t))
    shapes.append(Rect(vx - t / 2, vy, t, bar))
    for i in range(int(bar // 10) + 1):
        long = 3.0 if i in (0, bar // 10) else 1.5
        shapes.append(Rect(bx + i * 10 - t / 2, by - long, t, 2 * long))
        shapes.append(Rect(vx - long, vy + i * 10 - t / 2, 2 * long, t))
    shapes.append(Text(f"{bar:g} mm", bx + bar + 4, by + 1.5, 4.0))
    shapes.append(Text(f"{bar:g} mm", vx - 16, vy + bar + 7, 4.0))
    return shapes, where


def write_pdf(shapes: list, page_mm: tuple[float, float], out: Path) -> None:
    """Vector PDF at exact size (no scaling anywhere)."""
    pw, ph = page_mm
    pt = 72 / 25.4
    ops = ["0 g"]
    for s in shapes:
        if isinstance(s, Rect):
            ops.append(f"{s.x * pt:.3f} {(ph - s.y - s.h) * pt:.3f} {s.w * pt:.3f} {s.h * pt:.3f} re f")
        elif isinstance(s, CutLine):
            ops.append(f"q 0.6 G 0.3 w {s.x * pt:.3f} {(ph - s.y - s.h) * pt:.3f} {s.w * pt:.3f} {s.h * pt:.3f} re S Q")
        else:
            t = s.text.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
            font = "F2" if s.bold else "F1"
            ops.append(f"BT /{font} {s.em * pt:.2f} Tf {s.x * pt:.3f} {(ph - s.y) * pt:.3f} Td ({t}) Tj ET")
    content = "\n".join(ops).encode()
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        (f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {pw * pt:.3f} {ph * pt:.3f}] /Contents 4 0 R "
         f"/Resources << /Font << /F1 5 0 R /F2 6 0 R >> >> >>").encode(),
        b"<< /Length " + str(len(content)).encode() + b" >>\nstream\n" + content + b"\nendstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold >>",
    ]
    pdf = bytearray(b"%PDF-1.4\n")
    offsets = []
    for n, body in enumerate(objects, start=1):
        offsets.append(len(pdf))
        pdf += f"{n} 0 obj\n".encode() + body + b"\nendobj\n"
    xref = len(pdf)
    pdf += f"xref\n0 {len(objects) + 1}\n0000000000 65535 f \n".encode()
    pdf += "".join(f"{o:010d} 00000 n \n" for o in offsets).encode()
    pdf += f"trailer\n<< /Size {len(objects) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(bytes(pdf))


def render(shapes: list, w_mm: float, h_mm: float, ppm: float) -> np.ndarray:
    """Raster picture of shapes at ppm px/mm (0 = black, 255 = white). Text is only approximate."""
    img = np.full((int(round(h_mm * ppm)), int(round(w_mm * ppm))), 255, np.uint8)
    for s in shapes:
        if isinstance(s, Rect):   # pixel ranges, not fillPoly: fillPoly makes every box 1 px too big
            x0, y0 = int(round(s.x * ppm)), int(round(s.y * ppm))
            x1, y1 = max(x0 + 1, int(round((s.x + s.w) * ppm))), max(y0 + 1, int(round((s.y + s.h) * ppm)))
            img[y0:y1, x0:x1] = 0
        elif isinstance(s, CutLine):
            p0 = (int(s.x * ppm), int(s.y * ppm))
            p1 = (int((s.x + s.w) * ppm), int((s.y + s.h) * ppm))
            cv2.rectangle(img, p0, p1, 160, 1)
        else:
            cap_px = CAP_HEIGHT * s.em * ppm
            font = cv2.FONT_HERSHEY_DUPLEX if s.bold else cv2.FONT_HERSHEY_SIMPLEX
            thick = max(1, int(round(cap_px / (8 if s.bold else 14))))
            (_, th), _ = cv2.getTextSize(s.text, font, 1.0, thick)
            cv2.putText(img, s.text, (int(s.x * ppm), int(s.y * ppm)), font, cap_px / th, 0, thick, cv2.LINE_AA)
    return img


def render_label(lab: Label, ppm: float) -> np.ndarray:
    return render(lab.shapes, lab.w, lab.h, ppm)


# ---------------------------------------------------------------- bed map

def placed_label(bed: dict, labels: dict[str, Label], name: str) -> Label:
    """The label as it lies on the bed: the HOME label may have its text cut off (marker only)."""
    lab = labels[name]
    if name == "home" and bed["layout"].get("home_text_cut_off", True):
        m, q = float(bed["markers"]["size_mm"]), float(bed["markers"]["quiet_mm"])
        return Label(name, lab.marker_xy[0] + m + q, lab.h, [s for s in lab.shapes if isinstance(s, Rect)],
                     lab.marker_id, lab.marker_xy)
    return lab


def label_placements(bed: dict, labels: dict[str, Label]) -> list[tuple[str, tuple[float, float], tuple[float, float]]]:
    """(label name, marker centre in mm from HOME, label direction) from layout in config/bed.yaml.

    Direction = where the label runs from its marker (+X = right, +Y = back).
    """
    lay = bed["layout"]
    slot_dir = tuple(float(v) for v in lay["slot_direction"])
    out = [("home", tuple(lay["home_marker_xy"]), slot_dir)]
    for i, (x, y) in enumerate(lay.get("pcb_corner_marker_xy") or [], start=1):
        if f"pcb_{i}" in labels:
            out.append((f"pcb_{i}", (x, y), (0.0, 1.0) if i in (1, 2) else (0.0, -1.0)))
    for n, xy in sorted(lay["slot_marker_xy"].items()):
        out.append((f"slot_{int(n)}", tuple(xy), slot_dir))
    return out


def _corners_on_bed(lab: Label, m: float, centre, direction) -> list[tuple[float, float]]:
    (cx, cy), (dx, dy) = centre, direction
    mcx, mcy = lab.marker_xy[0] + m / 2, lab.marker_xy[1] + m / 2
    # label +x runs along (dx, dy); label +y (down on the print) runs along (dy, -dx)
    return [(cx + (px - mcx) * dx + (py - mcy) * dy, cy + (px - mcx) * dy - (py - mcy) * dx)
            for px, py in [(0, 0), (lab.w, 0), (lab.w, lab.h), (0, lab.h)]]


def label_footprints(bed: dict, labels: dict[str, Label]) -> dict[str, tuple[float, float, float, float]]:
    """Where each label lies: (x_min, y_min, x_max, y_max) in mm from HOME."""
    m = float(bed["markers"]["size_mm"])
    out = {}
    for name, centre, direction in label_placements(bed, labels):
        pts = _corners_on_bed(placed_label(bed, labels, name), m, centre, direction)
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        out[name] = (min(xs), min(ys), max(xs), max(ys))
    return out


def zone_footprints(bed: dict, labels: dict[str, Label]) -> dict[str, tuple[float, float, float, float]]:
    """Where each slot's tape mark (zone) lies: (x_min, y_min, x_max, y_max) in mm from HOME."""
    zone = slot_zone_mm(bed)
    out = {}
    for name, (cx, cy), (dx, dy) in label_placements(bed, labels):
        if name.startswith("slot_"):
            pts = [(cx + u * dx + v * dy, cy + u * dy - v * dx) for u, v in zone]
            out[name] = (min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts))
    return out


def front_line_l(bed: dict, labels: dict[str, Label]) -> float:
    """Line L: the front end (smallest Y) of the slot labels."""
    feet = label_footprints(bed, labels)
    return min(feet[n][1] for n in feet if n.startswith("slot_"))


def reach_box() -> tuple[tuple[float, float], tuple[float, float]] | None:
    """The measured camera reach in mm from HOME (config/bench.yaml), if there is one."""
    path = ROOT / "config" / "bench.yaml"
    if not path.exists():
        return None
    r = yaml.safe_load(path.read_text()).get("limits", {}).get("reach_from_home")
    return (tuple(r["x"]), tuple(r["y"])) if r else None


def _text(img, text, xy, scale=0.5, color=(0, 0, 0), thick=1, centred=False, box=False, bg=(255, 255, 255)):
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x = int(xy[0] - w / 2) if centred else int(xy[0])
    y = int(xy[1] + h / 2)
    if box:
        cv2.rectangle(img, (x - 4, y - h - 4), (x + w + 4, y + base + 2), bg, -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def _dimension(img, p0, p1, text, color=(200, 90, 0)):
    """Arrow line between two picture points, with its text beside the middle."""
    p0, p1 = tuple(map(int, p0)), tuple(map(int, p1))
    cv2.arrowedLine(img, p0, p1, color, 2, tipLength=0.06)
    cv2.arrowedLine(img, p1, p0, color, 2, tipLength=0.06)
    mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
    vertical = abs(p1[1] - p0[1]) > abs(p1[0] - p0[0])
    _text(img, text, (mid[0] + 8, mid[1]) if vertical else (mid[0], mid[1] - 14), 0.5, color, 1,
          centred=not vertical, box=True)


def _dashed_rect(img, p0, p1, color, step=10):
    (x0, y0), (x1, y1) = p0, p1
    for a, b in [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]:
        n = max(1, int(np.hypot(b[0] - a[0], b[1] - a[1]) // step))
        for i in range(0, n, 2):
            q0 = (int(a[0] + (b[0] - a[0]) * i / n), int(a[1] + (b[1] - a[1]) * i / n))
            q1 = (int(a[0] + (b[0] - a[0]) * (i + 1) / n), int(a[1] + (b[1] - a[1]) * (i + 1) / n))
            cv2.line(img, q0, q1, color, 2)


def bed_map(bed: dict, labels: dict[str, Label], s: float = 3.0) -> np.ndarray:
    """Placement plan in mm from HOME, seen from the front (FRONT = bottom of the picture)."""
    lay = bed["layout"]
    m_mm = float(bed["markers"]["size_mm"])
    feet = label_footprints(bed, labels)
    (ax, ay), (aw, ah) = lay["pcb_area_xy"], bed["pcb_area"]["size_mm"]
    reach = reach_box()
    xs = [v for f in feet.values() for v in (f[0], f[2])] + [ax, ax + aw] + (list(reach[0]) if reach else [])
    ys = [v for f in feet.values() for v in (f[1], f[3])] + [ay, ay + ah] + (list(reach[1]) if reach else [])
    x_lo, x_hi, y_lo, y_hi = min(xs) - 20, max(xs) + 20, min(ys) - 20, max(ys) + 20
    ml, mr, top, bottom = 40, 60, 70, 120
    W, H = int((x_hi - x_lo) * s) + ml + mr, int((y_hi - y_lo) * s) + top + bottom
    img = np.full((H, W, 3), 255, np.uint8)

    def m(x, y):  # mm from HOME -> picture px
        return ml + (x - x_lo) * s, top + (y_hi - y) * s

    cv2.rectangle(img, tuple(map(int, m(x_lo, y_hi))), tuple(map(int, m(x_hi, y_lo))), (225, 225, 225), -1)
    bars = sorted([c[0] for n, c, _ in label_placements(bed, labels) if n.startswith("slot_")] +
                  [lay["home_marker_xy"][0]])
    pitch = float(np.median(np.diff(bars))) if len(bars) > 1 else 30.0
    for g in np.arange(bars[0] - pitch / 2, x_hi, pitch):      # T-slot grooves (nominal)
        if x_lo < g < x_hi:
            cv2.line(img, tuple(map(int, m(g, y_lo))), tuple(map(int, m(g, y_hi))), (130, 130, 130), 3)
    for g in np.arange(bars[0] - pitch / 2, x_lo, -pitch):
        if x_lo < g < x_hi:
            cv2.line(img, tuple(map(int, m(g, y_lo))), tuple(map(int, m(g, y_hi))), (130, 130, 130), 3)

    cv2.rectangle(img, tuple(map(int, m(ax, ay + ah))), tuple(map(int, m(ax + aw, ay))), (60, 150, 60), 3)
    _text(img, f"PCB area {aw} x {ah} mm", m(ax + aw / 2, ay + ah / 2 + 6), 0.6, (40, 120, 40), 2, centred=True,
          box=True)
    _text(img, "(your board)", m(ax + aw / 2, ay + ah / 2 - 6), 0.5, (40, 120, 40), 1, centred=True, box=True)
    if reach:
        (rx0, rx1), (ry0, ry1) = reach
        _dashed_rect(img, m(rx0, ry1), m(rx1, ry0), (200, 0, 200))
        _text(img, "camera reach (measured)", np.add(m(rx0, ry0), (0, 14)), 0.5, (200, 0, 200), 1, box=True)

    ppm = 10.0
    f = s / ppm
    for name, (x, y), (dx, dy) in label_placements(bed, labels):
        lab = placed_label(bed, labels, name)
        pic = render_label(lab, ppm)
        cv2.rectangle(pic, (0, 0), (pic.shape[1] - 1, pic.shape[0] - 1), 160, 1)
        mcx, mcy = (lab.marker_xy[0] + m_mm / 2) * ppm, (lab.marker_xy[1] + m_mm / 2) * ppm
        c, sn = dx, -dy                              # picture y points down
        px, py = m(x, y)
        A = np.array([[f * c, -f * sn, 0], [f * sn, f * c, 0]], float)
        A[:, 2] = (px, py) - A[:, :2] @ (mcx, mcy)
        warped = cv2.warpAffine(pic, A, (W, H), flags=cv2.INTER_AREA, borderValue=0)
        mask = cv2.warpAffine(np.full_like(pic, 255), A, (W, H), flags=cv2.INTER_NEAREST) > 0
        img[mask] = np.repeat(warped[mask][:, None], 3, axis=1)
        cv2.circle(img, (int(px), int(py)), 3, (0, 0, 255), -1)
        if name.startswith("slot_"):
            _text(img, f"slot {name[5:]}", np.add(m(x, y), (0, -32)), 0.42, (0, 0, 0), 1, centred=True, box=True)
    hx, hy = lay["home_marker_xy"]
    _text(img, "HOME", np.add(m(hx, hy), (0, -32)), 0.5, (0, 0, 200), 2, centred=True, box=True)

    # Distances that matter
    slots = sorted(c for n, c, _ in label_placements(bed, labels) if n.startswith("slot_"))
    L = front_line_l(bed, labels)
    nearest = max(slots, key=lambda c: c[0])
    _dimension(img, m(nearest[0], hy + 14), m(hx, hy + 14), f"{hx - nearest[0]:g} mm")
    _text(img, "HOME: on the next bar, in line with the slot markers", np.add(m(nearest[0], hy + 24), (-260, -22)),
          0.45, (200, 90, 0), 1, box=True)
    gap = L - (ay + ah)
    x_gap = ax + aw - 8
    _dimension(img, m(x_gap, ay + ah), m(x_gap, L), f"{gap:.1f} mm")
    for x in np.arange(slots[0][0] - 12, slots[-1][0] + 12, 6):
        cv2.line(img, tuple(map(int, m(x, L))), tuple(map(int, m(x + 3, L))), (0, 0, 220), 2)
    _text(img, "front end of the slot labels", m(slots[0][0] + 45, L - 5), 0.42, (0, 0, 220), 1, box=True)
    _dimension(img, m(ax, ay - 18), m(ax + aw, ay - 18), f"{aw:g} mm")
    _dimension(img, m(ax + 8, ay), m(ax + 8, ay + ah), f"{ah:g} mm")

    sd = np.array(lay["slot_direction"], float)
    holes = np.array([sd[1], -sd[0]]) * -1          # the label's top side: where the holes go
    sx, sy = slots[len(slots) // 2]
    tail = np.array(m(sx, sy - 45), float)
    tip = tail + holes * np.array([1, -1]) * 40
    cv2.arrowedLine(img, tuple(map(int, tail)), tuple(map(int, tip)), (0, 0, 220), 2, tipLength=0.3)
    _text(img, "tape holes", tip + (6, 0), 0.45, (0, 0, 220), 1, box=True)

    _text(img, "BACK", (W / 2, top / 2), 0.7, (0, 0, 0), 2, centred=True)
    _text(img, "FRONT (you)", (W / 2, H - bottom + 22), 0.7, (0, 0, 0), 2, centred=True)
    _text(img, "LEFT", (ml / 2 + 4, top + 14), 0.5, (0, 0, 0), 2, centred=True)
    _text(img, "RIGHT", (W - mr / 2 - 6, top + 14), 0.5, (0, 0, 0), 2, centred=True)
    for i, line in enumerate(["mm from the centre of the HOME marker, +X right, +Y back. Grey = T-slot grooves.",
                              "Purple dashed = where the camera centre can go (measured). Positions are nominal."]):
        _text(img, line, (12, H - bottom + 52 + 20 * i), 0.45)
    return img


# ---------------------------------------------------------------- Brother PT-D410 (18 mm tape)

def scale_label(bar_mm: float) -> Label:
    """A bar along the tape with a tick every 10 mm: measure it to check the printer's tape feed."""
    h, t, x0 = 15.0, 0.5, 3.0
    text = f"{bar_mm:g} mm"
    lab = Label(f"scale_{bar_mm:g}mm", x0 + bar_mm + 4 + text_width(text, 6.0) + 3, h)
    lab.shapes.append(Rect(x0, h / 2 - t / 2, bar_mm, t))
    for i in range(int(bar_mm // 10) + 1):
        half = 5.0 if i in (0, bar_mm // 10) else 2.5
        lab.shapes.append(Rect(x0 + i * 10 - t / 2, h / 2 - half, t, 2 * half))
    lab.shapes.append(Text(text, x0 + bar_mm + 4, h / 2 + CAP_HEIGHT * 6.0 / 2, 6.0))
    return lab


def tape_image(lab: Label, dpi: float, tape_px: int) -> np.ndarray:
    """The label at the printer's resolution, centred on the tape's printable height (0/255 image)."""
    img = np.where(render_label(lab, dpi / 25.4) < 128, 0, 255).astype(np.uint8)
    h = img.shape[0]
    if h > tape_px:
        top = (h - tape_px) // 2
        if (img[:top] == 0).any() or (img[top + tape_px:] == 0).any():
            raise ValueError(f"{lab.name}: the print does not fit the {tape_px} px tape")
        img = img[top:top + tape_px]
    else:
        top = (tape_px - h) // 2
        img = cv2.copyMakeBorder(img, top, tape_px - h - top, 0, 0, cv2.BORDER_CONSTANT, value=255)
    return img


def save_two_color_png(img: np.ndarray, path: Path) -> None:
    """ptouch-print wants a 2-colour palette PNG: index 0 white, index 1 black."""
    from PIL import Image
    idx = Image.fromarray((img < 128).astype(np.uint8), mode="L").convert("P")
    idx.putpalette([255, 255, 255, 0, 0, 0] + [0, 0, 0] * 254)
    idx.save(path, optimize=True)


def run_ptouch(args: list[str], timeout: float = 240) -> tuple[int, str]:
    try:
        r = subprocess.run([PTOUCH, *args], capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except FileNotFoundError:
        return 127, "ptouch-print is not installed"
    except subprocess.TimeoutExpired:
        return 124, "ptouch-print timed out"


def make_tape(bed: dict, labels: dict[str, Label], out_dir: Path) -> list[Path]:
    tp = bed["tape_printer"]
    tape_px = int(tp["tape_px"][int(tp["tape_mm"])])
    items = list(labels.values()) + [scale_label(float(bed["print"]["check_bar_mm"]))]
    out_dir.mkdir(parents=True, exist_ok=True)
    paths, images = [], []
    for lab in items:
        img = tape_image(lab, float(tp["dpi"]), tape_px)
        path = out_dir / f"{lab.name}.png"
        save_two_color_png(img, path)
        paths.append(path)
        images.append(img)
    width = max(i.shape[1] for i in images)
    strip = np.vstack([np.pad(i, ((6, 6), (0, width - i.shape[1])), constant_values=200) for i in images])
    cv2.imwrite(str(out_dir / "preview.png"), strip)
    return paths


def print_tape(bed: dict, paths: list[Path], strip: bool = False) -> None:
    code, out = run_ptouch(["--info"], timeout=15)
    mm = re.search(r"media width = (\d+) mm", out)
    px = re.search(r"maximum printing width for this tape is (\d+)px", out)
    if code or not mm or not px:
        sys.exit("Printer not found or not ready. Switch the PT-D410 on and plug in the USB cable.\n" + out)
    want = int(bed["tape_printer"]["tape_mm"])
    if int(mm.group(1)) != want:
        sys.exit(f"The printer has {mm.group(1)} mm tape, but the labels need {want} mm tape.")
    if strip:                                  # one job, cut marks between labels: least tape wasted
        args = ["--timeout=0"]
        for i, p in enumerate(paths):
            args += (["--cutmark"] if i else []) + ["--image", str(p)]
        code, out = run_ptouch(args)
        if code:
            sys.exit(f"Printing failed:\n{out}")
        print(f"Printed {len(paths)} labels on one strip: {', '.join(p.stem for p in paths)}. "
              "Cut the strip off, then cut at the dashed marks.")
        return
    for n, p in enumerate(paths, start=1):     # one job per label, so each one can be cut off
        code, out = run_ptouch(["--timeout=0", "--image", str(p)])
        if code:
            sys.exit(f"Printing {p.stem} failed:\n{out}")
        print(f"[{n}/{len(paths)}] {p.stem} printed. CUT IT NOW (press the cutter lever).")
        if n < len(paths) and sys.stdin.isatty():
            if input("Press Enter for the next label (q = stop): ").strip().lower() == "q":
                return


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    ap.add_argument("--tape", action="store_true", help="also make the labels for the PT-D410 (hardware/print/tape/)")
    ap.add_argument("--print", action="store_true", help="with --tape: print them on the PT-D410, one at a time")
    ap.add_argument("--only", help="with --print: comma list of label names, e.g. scale_100mm,home,slot_1")
    ap.add_argument("--strip", action="store_true", help="with --print: all in one strip with cut marks (less waste)")
    args = ap.parse_args()

    bed = load_bed()
    labels = make_all(bed)
    if args.tape:
        paths = make_tape(bed, labels, args.out / "tape")
        print(f"Tape labels for {bed['tape_printer']['tape_mm']} mm tape: {len(paths)} PNGs in "
              f"{args.out / 'tape'} (see preview.png)")
        if args.only:
            names = [n.strip() for n in args.only.split(",")]
            unknown = set(names) - {p.stem for p in paths}
            if unknown:
                sys.exit(f"Unknown label(s): {', '.join(sorted(unknown))}. Names: {', '.join(p.stem for p in paths)}")
            paths = [next(p for p in paths if p.stem == n) for n in names]
        if args.print:
            print_tape(bed, paths, args.strip)
        return
    shapes, _ = page_layout(bed, labels)
    pw, ph = bed["print"]["page_mm"]
    args.out.mkdir(parents=True, exist_ok=True)
    pdf = args.out / PDF_NAME
    write_pdf(shapes, (pw, ph), pdf)
    cv2.imwrite(str(args.out / "bed_labels_a4_preview.png"), render(shapes, pw, ph, 6.0))
    cv2.imwrite(str(args.out / "bed_map.png"), bed_map(bed, labels))

    zone = slot_zone_mm(bed)
    out = args.out.resolve()
    shown = out.relative_to(ROOT) if out.is_relative_to(ROOT) else out
    print(f"Wrote {shown}/{PDF_NAME}  (+ bed_labels_a4_preview.png, bed_map.png)")
    print(f"{len(labels)} labels, markers {bed['markers']['size_mm']:g} mm. "
          f"Slot zone {zone[1, 0] - zone[0, 0]:g} x {zone[2, 1] - zone[1, 1]:g} mm, "
          f"starts {zone[0, 0]:g} mm from the marker centre.")
    for name, lab in labels.items():
        print(f"  {name:7s} {lab.w:5.1f} x {lab.h:4.1f} mm")
    print("Print at 100 % (Actual size) on matte sticker paper. Check both bars = "
          f"{bed['print']['check_bar_mm']:g} mm before you cut.")


if __name__ == "__main__":
    main()
