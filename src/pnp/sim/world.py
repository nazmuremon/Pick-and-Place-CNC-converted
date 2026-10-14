"""The simulated bed as layers at different heights (docs/13 §4.12), seen through a real camera model.

Layer 0 is the bed with its labels (z = 0, the label surface). On top, at their own heights: strips with parts
in their pockets (paper, black or clear tape), the board drawn from the Board model (holes show the bed below, with
the parallax of the tilted camera), and the ChArUco sheet (on the bed or on a block).

Texture convention: texture pixel i covers [x0 + i/T, x0 + (i+1)/T); its centre is at x0 + (i + 0.5)/T. OpenCV's
pixel centres are whole numbers, so the texture coordinate of a point is (x − x0)·T − 0.5.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

TEX_PX_PER_MM = 10.0
LABEL_LEN_MM, LABEL_W_MM = 87.0, 20.0        # slot label (D14): marker, number, 60 x 16 mm tape mark
ZONE_U_MM, ZONE_V_MM = (17.0, 77.0), 8.0
GREY = {"bed": 55, "bar": 120, "groove": 35, "label": 232, "board": 52, "pad": 190, "silk": 220, "wall": 140}


@dataclass
class Layer:
    name: str
    z: float
    x0: float                      # left edge, mm
    y1: float                      # top edge (largest Y), mm
    gray: np.ndarray               # float32
    alpha: np.ndarray              # float32 0..1
    T: float = TEX_PX_PER_MM

    def px(self, x: float, y: float) -> tuple[float, float]:
        """mm -> drawing coordinates (pixel edges at whole numbers; for cv2 drawing with shift)."""
        return (x - self.x0) * self.T, (self.y1 - y) * self.T

    def poly(self, pts_mm, value: float, alpha: float = 1.0) -> None:
        pts = np.array([self.px(*p) for p in pts_mm], np.float32)
        q = np.round((pts - 0.5) * 16).astype(np.int32)
        cv2.fillPoly(self.gray, [q], float(value), cv2.LINE_AA, shift=4)
        cv2.fillPoly(self.alpha, [q], float(alpha), cv2.LINE_AA, shift=4)

    def circle(self, cx: float, cy: float, r: float, value: float, alpha: float = 1.0) -> None:
        u, v = self.px(cx, cy)
        c = (int(round((u - 0.5) * 16)), int(round((v - 0.5) * 16)))
        rr = int(round(r * self.T * 16))
        cv2.circle(self.gray, c, rr, float(value), -1, cv2.LINE_AA, shift=4)
        cv2.circle(self.alpha, c, rr, float(alpha), -1, cv2.LINE_AA, shift=4)


def new_layer(name: str, z: float, x0: float, x1: float, y0: float, y1: float, fill: float = 0.0,
              alpha: float = 0.0, T: float = TEX_PX_PER_MM) -> Layer:
    w, h = int(math.ceil((x1 - x0) * T)), int(math.ceil((y1 - y0) * T))
    return Layer(name, z, x0, y1, np.full((h, w), fill, np.float32), np.full((h, w), alpha, np.float32), T)


def frame_axes(direction) -> tuple[np.ndarray, np.ndarray]:
    a = np.asarray(direction, float)
    a = a / np.linalg.norm(a)
    return a, np.array([-a[1], a[0]])


def rect_pts(c, along, u0, u1, v0, v1):
    a, n = frame_axes(along)
    c = np.asarray(c, float)
    return [c + a * u + n * v for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]


def put_marker(layer: Layer, dictionary, marker_id: int, centre, along, size_mm: float) -> None:
    """An ArUco marker whose corner 0 -> corner 1 side points along `along`."""
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, 120).astype(np.float32)
    a, n = frame_axes(along)
    h = size_mm / 2
    c = np.asarray(centre, float)
    corners = [c + a * u + n * v for u, v in ((-h, h), (h, h), (h, -h), (-h, -h))]
    dst = np.array([layer.px(*p) for p in corners], np.float32) - 0.5
    src = np.array([[-0.5, -0.5], [119.5, -0.5], [119.5, 119.5], [-0.5, 119.5]], np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    H, W = layer.gray.shape
    warped = cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_AREA, borderValue=255)
    mask = cv2.warpPerspective(np.ones_like(img), M, (W, H), flags=cv2.INTER_LINEAR)
    layer.gray = layer.gray * (1 - mask) + warped * mask
    layer.alpha = np.maximum(layer.alpha, mask)


def put_text(layer: Layer, text: str, centre, height_mm: float, value: float = 20) -> None:
    scale = height_mm * layer.T / 22.0
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
    u, v = layer.px(*centre)
    mask = np.zeros(layer.gray.shape, np.uint8)                     # putText draws only on 8-bit pictures
    cv2.putText(mask, text, (int(u - tw / 2), int(v + th / 2)), cv2.FONT_HERSHEY_SIMPLEX, scale, 255,
                max(1, int(scale * 2)), cv2.LINE_AA)
    a = mask.astype(np.float32) / 255.0
    layer.gray = layer.gray * (1 - a) + float(value) * a


class World:
    def __init__(self, bed: dict, extent_mm: tuple[float, float, float, float], T: float = TEX_PX_PER_MM,
                 trays: list | None = None, dictionary: str | None = None) -> None:
        """trays: the feeder bank's trays (number, marker_id, plan_xy, plan_dir) drawn instead of bed.yaml's slots."""
        self.bed = bed
        self.trays = {int(t.number): t for t in trays} if trays is not None else None
        self.tray_frames: dict[int, tuple] = {}
        self.T = T
        x0, x1, y0, y1 = extent_mm
        self.extent = extent_mm
        mk = bed["markers"]
        self.marker_mm = float(mk["size_mm"])
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary or mk["dictionary"]))
        self.labels: dict[str, tuple[float, float]] = {}
        self.strips: dict[int, Strip] = {}           # what lies on the bed (the simulated nozzle touches it)
        self.boards: dict[str, tuple] = {}           # board copy -> (board, pose, top layer)
        self.layers: list[Layer] = [new_layer("bed", 0.0, x0, x1, y0, y1, GREY["bed"], 1.0, T)]
        self._draw_bed()

    # ---------------------------------------------------------------- the bed
    def _draw_bed(self) -> None:
        L = self.layers[0]
        lay, mk = self.bed["layout"], self.bed["markers"]
        if self.trays is not None:
            slots = {n: t.plan_xy for n, t in self.trays.items() if t.plan_xy is not None}
        else:
            slots = {int(k): v for k, v in (lay.get("slot_marker_xy") or {}).items()}
        along = np.array(lay.get("slot_direction", [0, -1]), float)
        home = np.array(lay.get("home_marker_xy", [0, 0]), float)
        x0, x1, y0, y1 = self.extent
        xs = sorted([float(v[0]) for v in slots.values()] + [float(home[0])])
        pitch = float(np.median(np.diff(xs))) if len(xs) > 1 else 30.0
        bars = set(round(x, 1) for x in xs)
        x = min(xs) - pitch
        while x > x0 - pitch:
            bars.add(round(x, 1))
            x -= pitch
        x = max(xs) + pitch
        while x < x1 + pitch:
            bars.add(round(x, 1))
            x += pitch
        for bx in bars:
            L.poly([(bx - 10, y0), (bx + 10, y0), (bx + 10, y1), (bx - 10, y1)], GREY["bar"])
            L.poly([(bx - 3, y0), (bx + 3, y0), (bx + 3, y1), (bx - 3, y1)], GREY["groove"])
        L.poly(rect_pts(home, along, -7.5, 7.5, -7.5, 7.5), GREY["label"])
        put_marker(L, self.dictionary, int(mk["home_id"]), home, along, self.marker_mm)
        self.labels["home"] = tuple(home)
        for n, xy in slots.items():
            self.draw_tray(n, xy)
        if self.bed.get("pcb_area", {}).get("corner_labels") and lay.get("pcb_corner_marker_xy"):
            for i, (cid, xy) in enumerate(zip(mk["pcb_corner_ids"], lay["pcb_corner_marker_xy"]), start=1):
                c = np.array(xy, float)
                L.poly(rect_pts(c, along, -7.5, 7.5, -7.5, 7.5), GREY["label"])
                put_marker(L, self.dictionary, int(cid), c, along, self.marker_mm)
                self.labels[f"pcb_{i}"] = tuple(c)

    def draw_tray(self, n: int, xy, along=None, marker_id: int | None = None, layer: Layer | None = None) -> None:
        """A slot label (or a holder's top: the same geometry) with its marker and number."""
        lay, mk = self.bed["layout"], self.bed["markers"]
        t = self.trays.get(n) if self.trays is not None else None
        if layer is None and t is not None and t.type != "paper_label":     # a printed holder: its top is higher
            c0 = np.asarray(xy, float)
            pts = rect_pts(c0, along if along is not None else t.plan_dir, -10, LABEL_LEN_MM - 10, -LABEL_W_MM / 2,
                           LABEL_W_MM / 2)
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            layer = self.add(new_layer(f"tray_{n}", t.marker_z, min(xs) - 1, max(xs) + 1, min(ys) - 1, max(ys) + 1,
                                       0.0, 0.0, self.T))
        L = layer or self.layers[0]
        if along is None:
            along = t.plan_dir if t is not None else lay.get("slot_direction", [0, -1])
        along = np.asarray(along, float)
        if marker_id is None:
            marker_id = t.marker_id if t is not None else int(mk["slot_ids"][n - 1])
        c = np.array(xy, float)
        L.poly(rect_pts(c, along, -10, LABEL_LEN_MM - 10, -LABEL_W_MM / 2, LABEL_W_MM / 2), GREY["label"])
        for u0, u1, v0, v1 in ((ZONE_U_MM[0], ZONE_U_MM[0] + 0.4, -ZONE_V_MM, ZONE_V_MM),
                               (ZONE_U_MM[1] - 0.4, ZONE_U_MM[1], -ZONE_V_MM, ZONE_V_MM)):
            L.poly(rect_pts(c, along, u0, u1, v0, v1), 90)
        put_marker(L, self.dictionary, int(marker_id), c, along, self.marker_mm)
        put_text(L, str(n), c + frame_axes(along)[0] * 11.5, 5.0)
        if L is not self.layers[0] and t is not None and t.type != "paper_label":
            # the window between the holder's lips (traygen: 0.8 mm over each tape edge), the tape seen through it.
            # In the label's frame v = −n: the holes' side (−v) reaches 5.25 − 0.8, the far side 2.75 − 0.8 (8 mm tape)
            from ..feeders.bank import HOLE_TO_PART
            if t.type in ("matrix", "stick"):                       # the pockets seen through the top
                (u0, u1), (v0, v1) = t.zone_u, t.zone_v
                L.poly(rect_pts(c, along, u0, u1, -v1, -v0), 0.0, 0.0)
            else:
                hole_side = HOLE_TO_PART[int(t.tape_w)] + 1.75 - 0.8
                far_side = t.tape_w - 1.75 - HOLE_TO_PART[int(t.tape_w)] - 0.8
                L.poly(rect_pts(c, along, 16.0, LABEL_LEN_MM - 7.0, -far_side, hole_side), 0.0, 0.0)
        self.labels[f"slot_{n}"] = tuple(c)
        u, n_ = frame_axes(along)
        self.tray_frames[n] = (c, u, -n_)

    def slot_frame(self, n: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Slot n: marker centre, u (along the label, towards the tape mark), v = the label's +Y = marker corner
        0 -> 3 = u turned −90° (docs/07). The sprocket holes are on −v (on this bed: the right)."""
        if n in self.tray_frames:
            c, u, v = self.tray_frames[n]
            return np.array(c), np.array(u), np.array(v)
        lay = self.bed["layout"]
        c = np.array(lay["slot_marker_xy"][n], float)
        u, n_ = frame_axes(lay.get("slot_direction", [0, -1]))
        return c, u, -n_

    # ---------------------------------------------------------------- layers
    @property
    def board(self):
        return self.boards.get("1") or next(iter(self.boards.values()), None)

    def remove(self, name: str) -> None:
        self.layers = [L for L in self.layers if L.name != name]

    def add(self, layer: Layer) -> Layer:
        self.remove(layer.name)
        self.layers.append(layer)
        self.layers.sort(key=lambda L: L.z)
        return layer


def extent_for(hard_ends: dict, margin: float = 60.0) -> tuple[float, float, float, float]:
    (x0, x1), (y0, y1) = hard_ends["X"], hard_ends["Y"]
    return x0 - margin, x1 + margin, y0 - margin, y1 + margin


# ---------------------------------------------------------------- things on the bed
@dataclass
class Pocket:
    present: bool = True
    du: float = 0.0                  # part offset in its pocket (slot frame), mm
    dv: float = 0.0
    turn: float = 0.0                # part turned in its pocket, degrees


@dataclass
class Strip:
    """A cut-tape strip in a slot's tape mark (D14): pockets on v = 0, sprocket holes at v = −3.5 mm."""
    slot: int
    package_body: tuple[float, float] = (2.0, 1.25)
    look: str = "capacitor"          # capacitor (grey body, metal ends) | resistor (black top, white ends)
    tape: str = "paper"              # paper | black | clear
    pitch: float = 4.0
    first_u: float = 20.0            # the first pocket's centre, mm from the marker along the label
    pockets: list[Pocket] = field(default_factory=lambda: [Pocket() for _ in range(14)])
    z_top: float = 0.3               # the parts' tops (and the tape top) above the label surface
    tape_width: float = 8.0
    part_height: float = 0.3         # an empty pocket's floor is this much lower than z_top
    v_off: float = 0.0               # a matrix tray's row: this far across from the marker's line
    holes: bool = True               # a tape (sprocket holes) or a tray's row of pockets
    stick: bool = False              # a stick: the next part slides into the pick point while the tube has parts
    stock: int = 0                   # parts left in the stick's tube

    @property
    def band(self) -> tuple[float, float]:
        """Across: where the tape (or the tray's row) is, from the pocket row."""
        return (-5.25, -5.25 + self.tape_width) if self.holes else (-self.tape_width / 2, self.tape_width / 2)


def strip_layer(world: World, s: Strip) -> Layer:
    world.strips[(s.slot, s.v_off)] = s
    c, u, v = world.slot_frame(s.slot)
    c = c + v * s.v_off
    b0, b1 = s.band
    ends = [c + u * a + v * b for a in (s.first_u - 4, s.first_u + s.pitch * len(s.pockets) + 2)
            for b in (b0 - 1, b1 + 1)]
    xs, ys = [p[0] for p in ends], [p[1] for p in ends]
    name = f"strip_{s.slot}" if not s.v_off else f"strip_{s.slot}_{s.v_off:g}"
    L = new_layer(name, s.z_top, min(xs), max(xs), min(ys), max(ys), 0.0, 0.0, world.T)
    tape = {"paper": (205, 1.0), "black": (32, 1.0), "clear": (150, 0.25), "tray": (70, 1.0)}[s.tape]
    u0, u1 = s.first_u - 3.5, s.first_u + s.pitch * (len(s.pockets) - 0.5) + 1.5
    # EIA-481: the hole centres 1.75 mm from the tape's edge, the pocket row 3.5 mm from the holes (v = 0)
    L.poly([c + u * a + v * b for a, b in ((u0, b0), (u1, b0), (u1, b1), (u0, b1))], tape[0], tape[1])
    for k in range(len(s.pockets) + 1 if s.holes else 0):        # sprocket holes: see through
        hc = c + u * (s.first_u - 2 + k * s.pitch) + v * (-3.5)
        L.circle(hc[0], hc[1], 0.75, 0.0, 0.0)
    bl, bw = s.package_body
    for k, p in enumerate(s.pockets):
        pc = c + u * (s.first_u + k * s.pitch) + v * 0.0
        # the pocket: slightly bigger than the part, a little darker (paper) / clear shows a rim
        pl, pw = bw + 0.4, bl + 0.4                                   # parts lie across the tape (tape_angle 90)
        L.poly([pc + u * a + v * b for a, b in ((-pl / 2, -pw / 2), (pl / 2, -pw / 2), (pl / 2, pw / 2),
                                                 (-pl / 2, pw / 2))], tape[0] * 0.72, max(tape[1], 0.6))
        if not p.present:
            continue
        ang = math.radians(90.0 + p.turn)                              # the long side across the tape
        a_dir = u * math.cos(ang) + v * math.sin(ang)
        n_dir = np.array([-a_dir[1], a_dir[0]])
        pcen = pc + u * p.du + v * p.dv
        body = [pcen + a_dir * a + n_dir * b for a, b in ((-bl / 2, -bw / 2), (bl / 2, -bw / 2), (bl / 2, bw / 2),
                                                          (-bl / 2, bw / 2))]
        if s.look == "resistor":
            L.poly(body, 210)
            L.poly([pcen + a_dir * a + n_dir * b for a, b in ((-bl / 2 + 0.3, -bw / 2), (bl / 2 - 0.3, -bw / 2),
                                                              (bl / 2 - 0.3, bw / 2), (-bl / 2 + 0.3, bw / 2))], 40)
        else:
            L.poly(body, 200)
            L.poly([pcen + a_dir * a + n_dir * b for a, b in ((-bl / 2 + 0.35, -bw / 2), (bl / 2 - 0.35, -bw / 2),
                                                              (bl / 2 - 0.35, bw / 2), (-bl / 2 + 0.35, bw / 2))], 105)
    return L


@dataclass
class BoardPose:
    """Where a board lies: machine = R(theta) · view(side, p) + (x, y); its top at z_top."""
    x: float
    y: float
    theta: float = 0.0
    side: str = "F"
    z_top: float = 1.6

    def to_machine(self, board, p) -> np.ndarray:
        q = np.asarray(board.view(self.side, p), float)
        a = math.radians(self.theta)
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        return R @ q + np.array([self.x, self.y])


def board_layers(world: World, board, pose: BoardPose, thickness: float | None = None, copy: str = "1") -> list[Layer]:
    """The board's top (with pads) and its bottom surface (holes only): seen at an angle, a hole opens only where
    both overlap, like on the bench. copy: a second board of a panel gets its own layers."""
    top = board_layer(world, board, pose)
    t = board.thickness if thickness is None else thickness
    bot = board_layer(world, board, BoardPose(pose.x, pose.y, pose.theta, pose.side, pose.z_top - t), pads=False,
                      fill=GREY["wall"])                       # where the bottom shows past the top: the FR4 side wall
    sfx = "" if copy == "1" else f"#{copy}"
    top.name, bot.name = f"board{sfx}", f"board_bottom{sfx}"
    world.boards[copy] = (board, pose, top)
    return [bot, top]


def board_layer(world: World, board, pose: BoardPose, margin: float = 3.0, pads: bool = True,
                fill: float = GREY["board"]) -> Layer:
    outer = [pose.to_machine(board, p) for p in board.outline[0]]
    xs, ys = [p[0] for p in outer], [p[1] for p in outer]
    L = new_layer("board", pose.z_top, min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin,
                  0.0, 0.0, world.T)
    L.poly(outer, fill)
    for loop in board.outline[1:]:                                     # cut-outs
        L.poly([pose.to_machine(board, p) for p in loop], 0.0, 0.0)
    for c in (board.components if pads else []):
        for pad in c.pads:
            if pad.layer in ("", pose.side) and pad.plated:
                L.poly([pose.to_machine(board, p) for p in pad.polygon], GREY["pad"])
            if pad.through_hole and pad.drill:
                cx, cy = pose.to_machine(board, pad.centre)
                L.circle(cx, cy, pad.drill / 2, 0.0, 0.0)
    if not pads:                                                 # the bottom surface: the plated holes too
        for c in board.components:
            for pad in c.pads:
                if pad.through_hole and pad.drill:
                    cx, cy = pose.to_machine(board, pad.centre)
                    L.circle(cx, cy, pad.drill / 2, 0.0, 0.0)
    for h in board.holes:
        cx, cy = pose.to_machine(board, (h.x, h.y))
        L.circle(cx, cy, h.diameter / 2, 0.0, 0.0)
    return L


def charuco_layer(world: World, x: float, y: float, theta: float = 0.0, z: float = 0.0, name: str = "charuco") -> Layer:
    """The ChArUco sheet with its corner (0, 0) at (x, y), turned by theta (degrees), at height z."""
    from ..vision import charuco
    T = world.T
    img = charuco.sheet_image(T, margin_mm=4.0).astype(np.float32)
    w_mm, h_mm = img.shape[1] / T, img.shape[0] / T
    a = math.radians(theta)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    corners = [R @ np.array(p) + (x, y) for p in ((-4, -4), (w_mm - 4, -4), (w_mm - 4, h_mm - 4), (-4, h_mm - 4))]
    xs, ys = [p[0] for p in corners], [p[1] for p in corners]
    L = new_layer(name, z, min(xs) - 1, max(xs) + 1, min(ys) - 1, max(ys) + 1, 0.0, 0.0, T)
    # image pixel (i, j) centre is at sheet (i + 0.5)/T − 4, h_mm − 4 − (j + 0.5)/T
    dst = np.array([L.px(*p) for p in (corners[3], corners[2], corners[1], corners[0])], np.float32) - 0.5
    src = np.array([[-0.5, -0.5], [img.shape[1] - 0.5, -0.5], [img.shape[1] - 0.5, img.shape[0] - 0.5],
                    [-0.5, img.shape[0] - 0.5]], np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    H, W = L.gray.shape
    L.gray = cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)
    L.alpha = cv2.warpPerspective(np.ones_like(img), M, (W, H), flags=cv2.INTER_LINEAR)
    return L
