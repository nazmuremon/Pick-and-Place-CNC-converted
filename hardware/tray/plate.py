#!/usr/bin/env python3
"""All the printed bed parts on one print plate: a picture (plate.png) and one STL to print them in one go
(stl/plate_all.stl). The parts lie as they print: upside down, top face on the plate, rails up.

    .venv/bin/python hardware/tray/plate.py                  # paper holders 1-4 (the owner's choice)
    .venv/bin/python hardware/tray/plate.py --kind8 embossed  # or the plastic-tape ones
    .venv/bin/python hardware/tray/plate.py --bed 256 256

Units mm. The picture is drawn with OpenCV only (flat-shaded triangles, far to near).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
STL = HERE / "stl"


def parts(kind8: str) -> list[tuple[str, str]]:
    """(file name, label) of everything to print."""
    out = [(f"holder_{n}_8mm_{kind8}", f"Holder {n} (8 mm {kind8})") for n in (1, 2, 3, 4)]
    out += [("holder_5_12mm_embossed", "Holder 5 (12 mm)"), ("holder_6_16mm_embossed", "Holder 6 (16 mm, large)")]
    out += [(f"pcb_corner_{n}", f"PCB {n}") for n in (1, 2, 3, 4)] + [("home", "HOME")]
    return out


def layout(meshes: list[trimesh.Trimesh], bed: tuple[float, float], gap: float = 5.0) -> list[np.ndarray]:
    """Holders side by side on the left (their length along X), the corner pieces in a 2 x 2 block on the right,
    the HOME piece under them."""
    offsets, y = [], gap
    holders, corners = meshes[:6], meshes[6:]
    for m in holders:
        lo, hi = m.bounds
        offsets.append(np.array([gap - lo[0], y - lo[1], -lo[2]]))
        y += hi[1] - lo[1] + gap
    x0 = gap + max(m.extents[0] for m in holders) + 2 * gap
    cw = max(m.extents[0] for m in corners) + gap
    ch = max(m.extents[1] for m in corners) + gap
    for i, m in enumerate(corners):
        lo = m.bounds[0]
        offsets.append(np.array([x0 + (i % 2) * cw - lo[0], gap + (i // 2) * ch - lo[1], -lo[2]]))
    # (the HOME piece is the 5th "corner": it lands in a third row)
    used = np.max([(m.bounds[1] + o)[:2] for m, o in zip(meshes, offsets)], axis=0)
    if used[0] > bed[0] or used[1] > bed[1]:
        raise SystemExit(f"the parts need {used[0]:.0f} x {used[1]:.0f} mm, the bed is {bed[0]:g} x {bed[1]:g}")
    return offsets


def render(meshes, labels, bed, path: Path, size=(1800, 1250), ss=2) -> None:
    az, el = np.radians(-35.0), np.radians(38.0)
    view = np.array([np.cos(el) * np.sin(az), -np.cos(el) * np.cos(az), np.sin(el)])     # towards the viewer
    right = np.array([np.cos(az), np.sin(az), 0.0])
    up = np.cross(view, right)
    light = np.array([-0.4, -0.6, 0.9]) / np.linalg.norm([-0.4, -0.6, 0.9])

    def proj(p):
        p = np.asarray(p, float)
        return np.stack([p @ right, -(p @ up)], axis=-1), p @ view

    corners = np.array([[0, 0, 0], [bed[0], 0, 0], [bed[0], bed[1], 0], [0, bed[1], 0]], float)
    all_xy = np.vstack([proj(corners)[0]] + [proj(m.vertices)[0] for m in meshes])
    lo, hi = all_xy.min(0), all_xy.max(0)
    W, H = size[0] * ss, size[1] * ss
    margin = 60 * ss
    scale = min((W - 2 * margin) / (hi[0] - lo[0]), (H - 2 * margin - 40 * ss) / (hi[1] - lo[1]))
    shift = np.array([margin, margin + 40 * ss]) - lo * scale

    def px(p):
        return (proj(p)[0] * scale + shift).astype(np.int32)

    img = np.full((H, W, 3), 245, np.uint8)
    cv2.fillPoly(img, [px(corners)], (205, 205, 205), cv2.LINE_AA)                      # the print plate
    for g in np.arange(0, max(bed) + 1, 10):
        for a, b in (([g, 0, 0], [g, bed[1], 0]), ([0, g, 0], [bed[0], g, 0])):
            if g <= (bed[0] if a[1] == 0 and b[1] == bed[1] else bed[1]):
                cv2.line(img, tuple(px(a)), tuple(px(b)), (185, 185, 185), ss, cv2.LINE_AA)
    cv2.polylines(img, [px(corners)], True, (120, 120, 120), 2 * ss, cv2.LINE_AA)

    tris, depth, shade = [], [], []
    colours = [np.array(c, float) for c in ((70, 140, 235), (70, 140, 235), (70, 140, 235), (70, 140, 235),
                                             (80, 180, 90), (60, 110, 220))] + [np.array((40, 170, 230), float)] * 5
    for m, col in zip(meshes, colours):
        v = m.vertices[m.faces]                                       # (n, 3, 3)
        n = m.face_normals
        keep = n @ view > -1e-6                                        # back faces are hidden anyway
        lam = 0.35 + 0.65 * np.clip(n[keep] @ light, 0, 1)
        tris.append(v[keep]); depth.append(v[keep].mean(1) @ view); shade.append(lam[:, None] * col[None, :])
    tris, depth, shade = np.concatenate(tris), np.concatenate(depth), np.concatenate(shade)
    for i in np.argsort(depth):                                       # far first
        cv2.fillConvexPoly(img, px(tris[i]), tuple(int(c) for c in np.clip(shade[i], 0, 255)), cv2.LINE_AA)

    img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    for m, text in zip(meshes, labels):                               # names above each part
        c = m.bounds.mean(0); c[2] = m.bounds[1][2] + 3
        x, y = (proj(c)[0] * scale + shift) / ss
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(img, (int(x - tw / 2 - 4), int(y - th - 6)), (int(x + tw / 2 + 4), int(y + 4)), (255, 255, 255), -1)
        cv2.putText(img, text, (int(x - tw / 2), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)
    title = f"Print plate {bed[0]:g} x {bed[1]:g} mm: holders 1-6, PCB corners 1-4, HOME (upside down, as they print)"
    cv2.putText(img, title, (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (30, 30, 30), 2, cv2.LINE_AA)
    cv2.imwrite(str(path), img)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kind8", choices=["paper", "embossed"], default="paper", help="holders 1-4")
    ap.add_argument("--bed", type=float, nargs=2, default=[220.0, 220.0], metavar=("X", "Y"))
    args = ap.parse_args()
    items = parts(args.kind8)
    meshes = [trimesh.load(STL / f"{name}.stl") for name, _ in items]
    for m, o in zip(meshes, layout(meshes, tuple(args.bed))):
        m.apply_translation(o)
    trimesh.util.concatenate(meshes).export(STL / "plate_all.stl")
    render(meshes, [label for _, label in items], tuple(args.bed), HERE / "plate.png")
    print(f"{HERE / 'plate.png'}\n{STL / 'plate_all.stl'}: {len(meshes)} parts on {args.bed[0]:g} x {args.bed[1]:g}")


if __name__ == "__main__":
    main()
