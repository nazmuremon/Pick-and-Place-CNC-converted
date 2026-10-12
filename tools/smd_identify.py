#!/usr/bin/env python3
"""SMD identify: show SMD parts to the camera, and it tells you the package.

How it works (simple, no AI):
1. Print the sheet at 100 % scale:  python tools/smd_identify.py --make-sheet
   It has 4 ArUco markers around a white work area, and a 50 mm bar to check the scale.
2. Put parts on the white area, a few mm apart. Point the camera down at the sheet.
3. The markers give the scale in mm and remove the camera tilt.
4. Each part is measured (length x width in mm) and matched to the package table in
   config/smd_identify.yaml. Chip parts also get a hint: dark top = resistor?, light = capacitor?

Keys in the window:  + / -  threshold    d  debug view    s  save picture    q  quit
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "smd_identify.yaml"
DEFAULT_SHEET = ROOT / "hardware" / "print" / "smd_identify_sheet.pdf"
DEFAULT_CAMERA = "/dev/v4l/by-id/usb-Xitech_Mono_Camera_20240601105-video-index0"
SNAPSHOT_DIR = ROOT / "logs" / "smd_identify"


@dataclass
class Package:
    name: str
    kind: str
    length: float
    width: float


@dataclass
class Part:
    length_mm: float
    width_mm: float
    corners_mm: np.ndarray  # 4 x 2, sheet frame
    match: Package | None
    error: float
    runner_up: Package | None
    hint: str


@dataclass
class Result:
    found: bool
    homography: np.ndarray | None = None  # sheet mm -> camera pixels
    marker_corners: list | None = None
    camera_px_per_mm: float = 0.0
    parts: list[Part] | None = None
    rect_image: np.ndarray | None = None
    mask: np.ndarray | None = None


def load_config(path: Path) -> tuple[dict, list[Package]]:
    cfg = yaml.safe_load(path.read_text())
    packages = []
    for p in cfg["packages"]:
        length, width = sorted(p["size_mm"], reverse=True)
        packages.append(Package(p["name"], p.get("kind", "other"), float(length), float(width)))
    return cfg, packages


def aruco_dictionary(name: str):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_positions_mm(sheet: dict) -> dict[int, np.ndarray]:
    """Corners (TL, TR, BR, BL, the ArUco order) of each marker in the sheet frame (mm, y down)."""
    s = sheet["marker_size_mm"]
    w, h = sheet["area_mm"]
    top_left = [(0, 0), (w - s, 0), (w - s, h - s), (0, h - s)]
    out = {}
    for marker_id, (x, y) in zip(sheet["marker_ids"], top_left):
        out[marker_id] = np.array([(x, y), (x + s, y), (x + s, y + s), (x, y + s)], dtype=np.float32)
    return out


def classify(length: float, width: float, packages: list[Package], tolerance: float):
    """Best and second-best package by relative size error. Best is None if nothing is close enough."""
    scored = sorted(
        (float(np.hypot((length - p.length) / p.length, (width - p.width) / p.width)), p) for p in packages
    )
    best_err, best = scored[0]
    runner_up = scored[1][1] if len(scored) > 1 and scored[1][0] < best_err + 0.08 else None
    if best_err > tolerance:
        return None, best_err, None
    return best, best_err, runner_up


class Identifier:
    def __init__(self, cfg: dict, packages: list[Package], outside_mm: float = 0.0):
        """outside_mm: also look for parts this far outside the marker rectangle (paper around it)."""
        self.sheet = cfg["sheet"]
        self.det = cfg["detection"]
        self.packages = packages
        self.markers_mm = marker_positions_mm(self.sheet)
        self.detector = cv2.aruco.ArucoDetector(aruco_dictionary(self.sheet["dictionary"]), self._aruco_params())
        self.r = float(self.det["rect_px_per_mm"])
        self.m = float(outside_mm)
        w, h = self.sheet["area_mm"]
        self.rect_size = (int(round((w + 2 * self.m) * self.r)), int(round((h + 2 * self.m) * self.r)))
        self.static_mask = self._static_mask()
        self.average: np.ndarray | None = None

    @staticmethod
    def _aruco_params():
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return params

    def _px(self, mm: float) -> int:
        return max(1, int(round(mm * self.r)))

    def _static_mask(self) -> np.ndarray:
        """255 where parts may be: not near the edge, not near a marker."""
        w, h = self.rect_size
        mask = np.full((h, w), 255, np.uint8)
        e = self._px(self.det["edge_margin_mm"])
        mask[:e, :] = 0
        mask[-e:, :] = 0
        mask[:, :e] = 0
        mask[:, -e:] = 0
        m = self.det["marker_margin_mm"]
        for corners in self.markers_mm.values():
            x0, y0 = corners.min(axis=0) - m + self.m
            x1, y1 = corners.max(axis=0) + m + self.m
            cv2.rectangle(mask, (self._px(x0), self._px(y0)), (self._px(x1), self._px(y1)), 0, -1)
        return mask

    def _background(self, img: np.ndarray) -> np.ndarray:
        """Paper brightness everywhere: a large median on a small copy of the image."""
        small_scale = 2.0 / self.r  # work at 2 px/mm
        small = cv2.resize(img, None, fx=small_scale, fy=small_scale, interpolation=cv2.INTER_AREA)
        k = int(self.det["background_window_mm"] * 2) | 1
        small = cv2.medianBlur(np.clip(small, 0, 255).astype(np.uint8), min(k, 255))
        return cv2.resize(small, self.rect_size, interpolation=cv2.INTER_LINEAR).astype(np.float32)

    def process(self, gray: np.ndarray) -> Result:
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return Result(found=False)
        src, dst, sides = [], [], []
        kept = []
        for c, i in zip(corners, ids.flatten()):
            if int(i) in self.markers_mm:
                src.append(self.markers_mm[int(i)])
                dst.append(c.reshape(4, 2))
                sides.append(np.mean(np.linalg.norm(np.roll(c.reshape(4, 2), -1, axis=0) - c.reshape(4, 2), axis=1)))
                kept.append(c)
        if not src:
            return Result(found=False)
        src_pts = np.concatenate(src).astype(np.float32)
        dst_pts = np.concatenate(dst).astype(np.float32)
        method = cv2.RANSAC if len(src_pts) >= 8 else 0
        H, _ = cv2.findHomography(src_pts, dst_pts, method, 3.0)
        if H is None:
            return Result(found=False)
        camera_px_per_mm = float(np.mean(sides)) / self.sheet["marker_size_mm"]
        bias_mm = self.det["edge_bias_px"] / camera_px_per_mm

        # Straighten the work area: rect pixel -> sheet mm -> camera pixel.
        to_camera = H @ np.array([[1 / self.r, 0, -self.m], [0, 1 / self.r, -self.m], [0, 0, 1.0]])
        flags = cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP
        rect = cv2.warpPerspective(gray, to_camera, self.rect_size, flags=flags, borderMode=cv2.BORDER_REPLICATE)
        seen = cv2.warpPerspective(np.full(gray.shape, 255, np.uint8), to_camera, self.rect_size,
                                   flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP, borderValue=0)
        # Erosion removes half the kernel on each side: this keeps edge_margin_mm away from the image edge,
        # so a part cut off by the image edge is not measured (and misnamed) as a smaller part.
        seen = cv2.erode(seen, np.ones((2 * self._px(self.det["edge_margin_mm"]) + 1,) * 2, np.uint8))

        rect = rect.astype(np.float32)
        a = float(self.det["smoothing"])
        if self.average is None or self.average.shape != rect.shape:
            self.average = rect
        else:
            self.average = a * self.average + (1 - a) * rect
        img = self.average

        background = self._background(img)
        diff = np.abs(img - background) / np.maximum(background, 1.0)
        mask = ((diff > self.det["threshold"]) & (seen > 0)).astype(np.uint8) * 255
        close_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self._px(self.det["close_mm"]),) * 2)
        open_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self._px(self.det["open_mm"]),) * 2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_k)

        forbidden = cv2.bitwise_not(cv2.bitwise_and(self.static_mask, seen))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        parts = []
        for c in contours:
            area_mm2 = cv2.contourArea(c) / self.r**2
            if not self.det["min_area_mm2"] <= area_mm2 <= self.det["max_area_mm2"]:
                continue
            x, y, w, h = cv2.boundingRect(c)
            blob = np.zeros((h, w), np.uint8)
            cv2.drawContours(blob, [c - (x, y)], -1, 255, -1)
            if np.any(blob & forbidden[y:y + h, x:x + w]):
                continue  # touches the edge, a marker, or the part of the sheet the camera can't see
            box = cv2.minAreaRect(c)
            length, width = sorted(box[1], reverse=True)
            length = max(0.1, length / self.r - 2 * bias_mm)
            width = max(0.1, width / self.r - 2 * bias_mm)
            match, err, runner_up = classify(length, width, self.packages, self.det["match_tolerance"])
            hint = ""
            if match is not None and match.kind == "chip":
                hint = self._hint(img, background, box)
            elif runner_up is not None and runner_up.kind == "chip":
                hint = f"{self._hint(img, background, box)} (if {runner_up.name})"
            parts.append(Part(length, width, cv2.boxPoints(box) / self.r - self.m, match, err, runner_up, hint))

        return Result(True, H, kept, camera_px_per_mm, parts, img, mask)

    def _hint(self, img: np.ndarray, background: np.ndarray, box) -> str:
        """Chip resistors have a black top; ceramic capacitors are lighter. Only a hint."""
        (cx, cy), (w, h), angle = box
        inner = np.zeros(img.shape, np.uint8)
        cv2.fillPoly(inner, [cv2.boxPoints(((cx, cy), (w * 0.4, h * 0.4), angle)).astype(np.int32)], 255)
        if not inner.any():
            return ""
        ratio = float(img[inner > 0].mean() / max(background[inner > 0].mean(), 1.0))
        return "dark top -> resistor?" if ratio < self.det["dark_top_ratio"] else "light body -> capacitor?"


# ---------------------------------------------------------------- drawing

GREEN, YELLOW, RED, CYAN = (80, 220, 80), (0, 220, 255), (60, 60, 255), (255, 200, 0)


def put_text(img, text, org, color=(255, 255, 255), scale=0.6):
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = org
    cv2.rectangle(img, (x - 3, y - h - 4), (x + w + 3, y + base + 1), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def project(H: np.ndarray, pts_mm: np.ndarray, scale: float) -> np.ndarray:
    pts = cv2.perspectiveTransform(pts_mm.reshape(-1, 1, 2).astype(np.float32), H).reshape(-1, 2)
    return (pts * scale).astype(np.int32)


def draw(frame: np.ndarray, res: Result, cfg: dict, scale: float, threshold: float) -> np.ndarray:
    view = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    if view.ndim == 2:
        view = cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)
    lines = []
    if not res.found:
        lines.append(("Sheet not found: show the printed markers to the camera", RED))
    else:
        w, h = cfg["sheet"]["area_mm"]
        cv2.polylines(view, [project(res.homography, np.array([(0, 0), (w, 0), (w, h), (0, h)]), scale)],
                      True, CYAN, 1, cv2.LINE_AA)
        for c in res.marker_corners:
            cv2.polylines(view, [(c.reshape(4, 2) * scale).astype(np.int32)], True, CYAN, 2, cv2.LINE_AA)
        low = cfg["detection"]["low_resolution_px_per_mm"]
        px = res.camera_px_per_mm
        lines.append((f"Camera: {px:.1f} px/mm" + ("  (low: small parts may be confused)" if px < low else ""),
                      YELLOW if px < low else GREEN))
        counts: dict[str, int] = {}
        for p in res.parts:
            name = p.match.name if p.match else "unknown"
            counts[name] = counts.get(name, 0) + 1
            color = RED if p.match is None else GREEN if p.error < 0.12 else YELLOW
            pts = project(res.homography, p.corners_mm, scale)
            cv2.polylines(view, [pts], True, color, 2, cv2.LINE_AA)
            x, y = pts[:, 0].min(), pts[:, 1].min()
            label = p.match.name if p.match else "unknown"
            if p.runner_up:
                label += f" (or {p.runner_up.name})"
            put_text(view, label, (int(x), int(y) - 26), color, 0.6)
            detail = f"{p.length_mm:.1f} x {p.width_mm:.1f} mm" + (f"  {p.hint}" if p.hint else "")
            put_text(view, detail, (int(x), int(y) - 8), color, 0.5)
        summary = ", ".join(f"{n} x{k}" for n, k in sorted(counts.items())) or "none (put parts on the white area)"
        lines.append((f"Parts: {summary}", (255, 255, 255)))
    lines.append((f"Threshold {threshold:.2f} (+/-)   d debug   s save   q quit", (200, 200, 200)))
    for i, (text, color) in enumerate(lines):
        put_text(view, text, (10, 24 + 24 * i), color, 0.6)
    return view


def debug_view(res: Result) -> np.ndarray | None:
    if not res.found:
        return None
    img = np.clip(res.rect_image, 0, 255).astype(np.uint8)
    both = np.hstack([img, res.mask])
    scale = min(1.0, 1400 / both.shape[1])
    return cv2.resize(both, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


# ---------------------------------------------------------------- printable sheet

def make_sheet(cfg: dict, out: Path) -> None:
    """Write an A4 PDF with the markers at exact size (vector, no scaling)."""
    sheet = cfg["sheet"]
    s = sheet["marker_size_mm"]
    w, h = sheet["area_mm"]
    page_w, page_h = 210.0, 297.0
    ox, oy = (page_w - w) / 2, 45.0  # top-left of the marker rectangle on the page
    left = 20.0  # text margin
    pt = 72 / 25.4
    ops = ["0 g"]

    def box(x_mm, y_mm, w_mm, h_mm):  # page mm, y down -> PDF points, y up
        ops.append(f"{x_mm * pt:.3f} {(page_h - y_mm - h_mm) * pt:.3f} {w_mm * pt:.3f} {h_mm * pt:.3f} re f")

    def text(x_mm, y_mm, size, t):
        t = t.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
        ops.append(f"BT /F1 {size} Tf {x_mm * pt:.2f} {(page_h - y_mm) * pt:.2f} Td ({t}) Tj ET")

    dictionary = aruco_dictionary(sheet["dictionary"])
    for marker_id, corners in marker_positions_mm(sheet).items():
        bits = cv2.aruco.generateImageMarker(dictionary, marker_id, 6, borderBits=1)
        cell = s / 6
        mx, my = corners[0]
        for r in range(6):
            for c in range(6):
                if bits[r, c] < 128:
                    box(ox + mx + c * cell, oy + my + r * cell, cell, cell)

    bar_y = oy + h + 12
    box(ox, bar_y, 50, 1.0)
    for i in range(6):
        box(ox + i * 10 - 0.15, bar_y - 2, 0.3, 2)
    text(left, bar_y + 6, 9, "This bar must be 50 mm. If not, print again at 100 % (Actual size, no 'fit to page').")
    text(left, 20, 16, "SMD identify sheet")
    text(left, 28, 9, "Put parts on the white area between the 4 markers, a few mm apart and away from the markers.")
    text(left, 33, 9, "Point the camera straight down. All 4 markers should be in view if possible.")
    text(left, bar_y + 14, 9, f"Markers: ArUco {sheet['dictionary']}, IDs {sheet['marker_ids']}, {s} mm. "
                            f"Area {w} x {h} mm. Made by tools/smd_identify.py.")

    content = "\n".join(ops).encode()
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        (f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {page_w * pt:.3f} {page_h * pt:.3f}] "
         f"/Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>").encode(),
        b"<< /Length " + str(len(content)).encode() + b" >>\nstream\n" + content + b"\nendstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
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


# ---------------------------------------------------------------- main

def window_open(name: str) -> bool:
    """False after the user closes the window with its X button (Qt raises an error then)."""
    try:
        return cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def open_camera(device: str, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(int(device) if device.isdigit() else device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"Can't open camera {device}. Use --camera /dev/videoN (see: ls /dev/v4l/by-id/)")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)   # the driver picks the nearest size the camera has
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)        # always show a fresh frame
    return cap


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default=DEFAULT_CAMERA if Path(DEFAULT_CAMERA).exists() else "0",
                    help="camera index or /dev/v4l/by-id/... path")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--size", default="1920x1080", help="requested camera size (default: largest near 1080p)")
    ap.add_argument("--make-sheet", nargs="?", const=DEFAULT_SHEET, type=Path, metavar="PDF",
                    help=f"write the printable sheet and exit (default {DEFAULT_SHEET.relative_to(ROOT)})")
    args = ap.parse_args()

    cfg, packages = load_config(args.config)
    if args.make_sheet:
        make_sheet(cfg, args.make_sheet)
        print(f"Sheet written: {args.make_sheet}\nPrint it at 100 % (Actual size) and check the 50 mm bar.")
        return

    width, height = (int(v) for v in args.size.lower().split("x"))
    cap = open_camera(args.camera, width, height)
    ident = Identifier(cfg, packages)
    show_debug = False
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        res = ident.process(gray)
        scale = 1280 / frame.shape[1]
        view = draw(frame, res, cfg, scale, ident.det["threshold"])
        cv2.imshow("SMD identify", view)
        if show_debug:
            dbg = debug_view(res)
            if dbg is not None:
                cv2.imshow("Debug: straightened area | part mask", dbg)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27) or not window_open("SMD identify"):
            break
        if key in (ord("+"), ord("=")):
            ident.det["threshold"] = round(min(0.9, ident.det["threshold"] + 0.01), 2)
        elif key in (ord("-"), ord("_")):
            ident.det["threshold"] = round(max(0.02, ident.det["threshold"] - 0.01), 2)
        elif key == ord("d"):
            show_debug = not show_debug
            if not show_debug:
                cv2.destroyWindow("Debug: straightened area | part mask")
        elif key == ord("s"):
            SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S") + f"-{int(time.time() * 1000) % 1000:03d}"
            cv2.imwrite(str(SNAPSHOT_DIR / f"{stamp}_raw.png"), frame)
            cv2.imwrite(str(SNAPSHOT_DIR / f"{stamp}_view.png"), view)
            print(f"Saved {SNAPSHOT_DIR / stamp}_*.png")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
