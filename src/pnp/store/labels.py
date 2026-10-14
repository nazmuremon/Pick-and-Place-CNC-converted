"""Labels through the LabelPrinter capability (docs/13 §4.14, §4.15): drawer labels and part labels.

- ptouch: the Brother PT-D410 through `ptouch-print` (>= 1.7): ONE strip with cut marks by default, never one job per
  label unless asked (it wastes tape; owner rule). The tape's printable height is read from the printer.
- pdf_a4: an A4 PDF (print at 100 %).
- browser_print: the same PDF, the browser prints it.
"""
from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PTOUCH = shutil.which("ptouch-print") or "/usr/local/bin/ptouch-print"


def _font(px: int):
    for name in ("DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except OSError:
            continue
    return ImageFont.load_default()


def label_image(title: str, lines: list[str], height_px: int) -> Image.Image:
    """A label as a 1-bit picture for a tape of height_px (white = no ink)."""
    big, small = _font(max(10, int(height_px * 0.55))), _font(max(8, int(height_px * 0.2)))
    d0 = ImageDraw.Draw(Image.new("L", (10, 10)))
    tw = d0.textlength(title, font=big)
    lw = max([d0.textlength(t, font=small) for t in lines] or [0])
    w = int(tw + 12 + lw + 16)
    img = Image.new("L", (w, height_px), 255)
    d = ImageDraw.Draw(img)
    d.text((6, (height_px - big.size) // 2 - 2), title, font=big, fill=0)
    y = 2
    for t in lines[: max(1, height_px // (small.size + 1))]:
        d.text((tw + 18, y), t, font=small, fill=0)
        y += small.size + 1
    return img


def drawer_labels(drawers: dict[int, list[str]], height_px: int) -> list[Image.Image]:
    return [label_image(f"{n}", items, height_px) for n, items in sorted(drawers.items())]


class PtouchPrinter:
    kind = "ptouch"

    def _run(self, args, timeout=60):
        try:
            r = subprocess.run([PTOUCH, *args], capture_output=True, text=True, timeout=timeout)
            return r.returncode, (r.stdout + r.stderr).strip()
        except FileNotFoundError:
            return 127, "ptouch-print is not installed"
        except subprocess.TimeoutExpired:
            return 124, "ptouch-print timed out"

    def status(self) -> dict:
        code, out = self._run(["--info"], 15)
        if code != 0 or "maximum printing width for this tape" not in out:
            msg = ("Printer not found: is it on and plugged in by USB?" if "No P-Touch printer found" in out else
                   "Printer busy (a CUPS job?)" if "LIBUSB_ERROR_BUSY" in out else out.splitlines()[-1] if out else
                   "ptouch-print failed")
            return {"connected": False, "message": msg}
        m = re.search(r"for this tape is (\d+)px", out)
        mm = re.search(r"media width = (\d+) mm", out)
        return {"connected": True, "tape_px": int(m.group(1)) if m else None, "tape_mm": int(mm.group(1)) if mm else None}

    def print(self, images: list[Image.Image], mode: str = "strip") -> dict:
        st = self.status()
        if not st.get("connected"):
            return {"ok": False, "why": st["message"]}
        with tempfile.TemporaryDirectory(prefix="labels-") as tmp:
            files = []
            for i, im in enumerate(images):
                if im.height > (st["tape_px"] or im.height):
                    return {"ok": False, "why": f"label {i + 1} is {im.height} px high, the tape prints {st['tape_px']}"}
                p = Path(tmp) / f"l{i:03d}.png"
                bw = im.convert("L").point(lambda v: 1 if v < 128 else 0)
                pal = Image.frombytes("P", bw.size, bw.tobytes())
                pal.putpalette([255, 255, 255, 0, 0, 0] + [0, 0, 0] * 254)
                pal.save(p)
                files.append(p)
            if mode == "separate":
                for p in files:
                    code, out = self._run(["--timeout=0", "--image", str(p)])
                    if code:
                        return {"ok": False, "why": out}
            else:                                                     # one strip with cut marks: the least tape
                args = ["--timeout=0"]
                for i, p in enumerate(files):
                    args += (["--cutmark"] if i else []) + ["--image", str(p)]
                code, out = self._run(args, 180)
                if code:
                    return {"ok": False, "why": out}
        return {"ok": True, "printed": len(images)}


class PdfPrinter:
    kind = "pdf_a4"

    def __init__(self, out: Path) -> None:
        self.out = out

    def status(self) -> dict:
        return {"connected": True, "tape_px": 76}

    def print(self, images: list[Image.Image], mode: str = "strip") -> dict:
        """All labels on A4 pages at 180 dpi (the PT-D410's resolution), 100 % scale."""
        dpi, ppm = 180, 180 / 25.4
        page_w, page_h = int(210 * ppm), int(297 * ppm)
        pages, page, y = [], Image.new("L", (page_w, page_h), 255), int(10 * ppm)
        for im in images:
            if y + im.height > page_h - 10 * ppm:
                pages.append(page)
                page, y = Image.new("L", (page_w, page_h), 255), int(10 * ppm)
            page.paste(im, (int(10 * ppm), y))
            ImageDraw.Draw(page).rectangle([int(10 * ppm) - 1, y - 1, int(10 * ppm) + im.width, y + im.height],
                                           outline=160)
            y += im.height + int(4 * ppm)
        pages.append(page)
        self.out.parent.mkdir(parents=True, exist_ok=True)
        pages[0].save(self.out, "PDF", resolution=dpi, save_all=True, append_images=pages[1:])
        return {"ok": True, "printed": len(images), "pdf": str(self.out)}


def build_printer(kind: str, out_pdf: Path):
    return PtouchPrinter() if kind == "ptouch" else PdfPrinter(out_pdf)
