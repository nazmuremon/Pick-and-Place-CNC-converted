#!/usr/bin/env python3
"""Make the HT5023 board fixtures (run once; the files are kept in the repository).

    python3 tests/data/boards/make_samples.py ../ht5023_energy_meter_v1.kicad_pcb

1. Real KiCad 10 exports (`kicad-cli` must be installed): the board (gzipped), Gerber X2 copper + outline, drill
   files, the job file and the X3 position files (zipped), and KiCad's own centroid files (.csv and .pos, mm).
2. SYNTHETIC centroid files in the layouts of other tools, written from KiCad's CSV. They are NOT exports of those
   tools: each uses a different origin, unit, Y direction and bottom rotation rule on purpose, so the tests can
   check that the importer finds each of them (docs/13 §8). The rules chosen here are not claims about the tools.
"""
from __future__ import annotations

import csv
import gzip
import io
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent / "ht5023"
X0, YTOP, YBOT = 65.0, -20.0, -160.0          # the board's outline box in KiCad's CSV coordinates (Y up)


def kicad_exports(pcb: Path) -> None:
    HERE.mkdir(exist_ok=True)
    (HERE / "ht5023.kicad_pcb.gz").write_bytes(gzip.compress(pcb.read_bytes(), 9))
    with tempfile.TemporaryDirectory() as t:
        t = Path(t)
        g = t / "gerber"
        g.mkdir()
        run = lambda *a: subprocess.run(["kicad-cli", "pcb", "export", *a, str(pcb)], check=True,  # noqa: E731
                                        capture_output=True)
        run("gerbers", "-o", str(g), "-l", "F.Cu,B.Cu,Edge.Cuts")
        run("drill", "-o", str(g), "--excellon-separate-th")
        run("pos", "--format", "gerber", "--side", "front", "-o", str(g / "ht5023-top-pos.gbr"))
        run("pos", "--format", "gerber", "--side", "back", "-o", str(g / "ht5023-bottom-pos.gbr"))
        with zipfile.ZipFile(HERE / "ht5023_gerber_x3.zip", "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(g.iterdir()):
                z.write(f, f.name)
        run("pos", "--format", "csv", "--units", "mm", "-o", str(HERE / "kicad-pos.csv"))
        run("pos", "--format", "ascii", "--units", "mm", "-o", str(HERE / "kicad-pos.pos"))


def kicad_rows() -> list[dict]:
    return list(csv.DictReader(io.StringIO((HERE / "kicad-pos.csv").read_text())))


def model_bottom(r: float) -> float:
    return 180.0 - r                            # KiCad's angle (seen from the top) -> seen from below (D26)


def synthetic() -> None:
    rows = kicad_rows()
    # Altium layout: preamble, quoted CSV, origin = the board's bottom-left, mm, bottom angle as seen from the top
    out = ["Altium Designer Pick and Place Locations", "C:\\HT5023\\HT5023.PcbDoc", "",
           "=" * 80, "File Design Information:", "", "Date:       12/10/26", "Units used: mm", "",
           '"Designator","Comment","Layer","Footprint","Center-X(mm)","Center-Y(mm)","Rotation","Description"']
    for r in rows:
        x, y = float(r["PosX"]) - X0, float(r["PosY"]) - YBOT
        layer = "TopLayer" if r["Side"] == "top" else "BottomLayer"
        out.append(f'"{r["Ref"]}","{r["Val"]}","{layer}","{r["Package"]}","{x:.4f}","{y:.4f}","{float(r["Rot"]):g}",""')
    (HERE / "synthetic-altium.csv").write_text("\r\n".join(out) + "\r\n")

    # EasyEDA layout: UTF-16 with BOM, tabs, values with units (mil), Y DOWN from the board's top edge,
    # bottom angle already seen from below (model angle)
    lines = ["Designator\tFootprint\tMid X\tMid Y\tRef X\tRef Y\tPad X\tPad Y\tLayer\tRotation\tComment"]
    for r in rows:
        x_mm, y_mm = float(r["PosX"]) - X0, YTOP - float(r["PosY"])
        rot = float(r["Rot"]) if r["Side"] == "top" else model_bottom(float(r["Rot"]))
        x, y = x_mm / 0.0254, y_mm / 0.0254
        lines.append(f'{r["Ref"]}\t{r["Package"]}\t{x:.2f}mil\t{y:.2f}mil\t{x:.2f}mil\t{y:.2f}mil\t0mil\t0mil\t'
                     f'{"T" if r["Side"] == "top" else "B"}\t{rot % 360:g}\t{r["Val"]}')
    (HERE / "synthetic-easyeda.csv").write_bytes(b"\xff\xfe" + "\r\n".join(lines).encode("utf-16-le"))

    # JLC-style CSV: Designator,Val,Package,Mid X,Mid Y,Rotation,Layer; "mm" on the numbers; origin 10 mm left of
    # and 5 mm below the board; bottom X written negated; bottom angle = −(KiCad angle)
    lines = ["Designator,Val,Package,Mid X,Mid Y,Rotation,Layer"]
    for r in rows:
        x, y = float(r["PosX"]) - X0 + 10.0, float(r["PosY"]) - YBOT + 5.0
        rot = float(r["Rot"])
        if r["Side"] == "bottom":
            x, rot = -x, -rot
        lines.append(f'{r["Ref"]},"{r["Val"]}",{r["Package"]},{x:.4f}mm,{y:.4f}mm,{rot:g},'
                     f'{"Top" if r["Side"] == "top" else "Bottom"}')
    (HERE / "synthetic-jlc.csv").write_text("\n".join(lines) + "\n")

    # Eagle mountsmd style: no header, top and bottom in two files, mm, origin = the board's bottom-left,
    # bottom angle = model + 180
    top, bot = [], []
    for r in rows:
        x, y = float(r["PosX"]) - X0, float(r["PosY"]) - YBOT
        if r["Side"] == "top":
            top.append(f'{r["Ref"]} {x:.3f} {y:.3f} {float(r["Rot"]) % 360:g} {r["Val"].replace(" ", "_")} {r["Package"]}')
        else:
            rot = (model_bottom(float(r["Rot"])) + 180.0) % 360
            bot.append(f'{r["Ref"]} {x:.3f} {y:.3f} {rot:g} {r["Val"].replace(" ", "_")} {r["Package"]}')
    (HERE / "synthetic-eagle.mnt").write_text("\n".join(top) + "\n")
    (HERE / "synthetic-eagle.mnb").write_text("\n".join(bot) + "\n")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        if shutil.which("kicad-cli") is None:
            sys.exit("kicad-cli not found")
        kicad_exports(Path(sys.argv[1]).resolve())
    synthetic()
    print("written to", HERE)
