"""Phase B: the Board model and its importers on the HT5023 (tests/data/boards/ht5023/, made by make_samples.py).

Done when (docs/13 §11): the HT5023 imports from KiCad and from its Gerber X3 identically. Plus: the centroid
files of every layout give the same board, and the package library sorts the parts as docs/11 counts them.
"""
from __future__ import annotations

import math
from pathlib import Path

import pytest
from conftest import BOARDS, PACKAGES

from pnp.board import centroid as ce
from pnp.board import checks, gerber, gerber_import, kicad
from pnp.board.geometry import angle_diff, bbox
from pnp.board.model import Board
from pnp.board.service import BoardService
from pnp.feeders.packages import Library, annotate

X0, YBOT = 65.0, 160.0          # the outline box's left edge and bottom edge in KiCad's file (Y down)


@pytest.fixture(scope="module")
def lib() -> Library:
    return Library.load(PACKAGES)


@pytest.fixture(scope="module")
def k() -> Board:
    return kicad.load(BOARDS / "ht5023.kicad_pcb.gz", "ht5023")


@pytest.fixture(scope="module")
def x3() -> Board:
    return gerber_import.load_files([("g.zip", (BOARDS / "ht5023_gerber_x3.zip").read_bytes())])


def kicad_xy(b: Board, p) -> tuple[float, float]:
    """Board frame -> KiCad's coordinates (Y down), to compare with pcbnew."""
    return (round(p[0] + X0, 4), round(YBOT - p[1], 4))


def same_board(a: Board, b: Board, pos_tol: float = 0.05, pads: bool = True) -> list[str]:
    """Every difference between two boards (empty = the same board)."""
    out = []
    A, B = a.by_ref(), b.by_ref()
    if set(A) != set(B):
        out.append(f"refs differ: {sorted(set(A) ^ set(B))[:10]}")
    for r in sorted(set(A) & set(B)):
        x, y = A[r], B[r]
        if x.side != y.side:
            out.append(f"{r} side {x.side} {y.side}")
        if math.hypot(x.x - y.x, x.y - y.y) > pos_tol:
            out.append(f"{r} position {x.x},{x.y} vs {y.x},{y.y}")
        if angle_diff(x.rotation, y.rotation) > 0.01:
            out.append(f"{r} rotation {x.rotation} vs {y.rotation}")
        if pads:
            if x.smd != y.smd:
                out.append(f"{r} smd {x.smd} {y.smd}")
            pa = [p for p in x.pads if p.plated]
            pb = [p for p in y.pads if p.plated]
            far = [p.number for p in pa if min((math.hypot(p.centre[0] - q.centre[0], p.centre[1] - q.centre[1])
                                                for q in pb if q.number == p.number), default=99) > 0.02]
            if len(pa) != len(pb) or far:
                out.append(f"{r} pads differ: {len(pa)} vs {len(pb)}, far: {far[:5]}")
    return out


# ---------------------------------------------------------------- KiCad
def test_kicad_reads_the_board_as_docs_11_describes_it(k):
    s = k.summary()
    assert (s["components"], s["smd"], s["smd_top"], s["smd_bottom"], s["tht"]) == (206, 169, 31, 138, 33)
    assert (round(k.width, 3), round(k.height, 3)) == (130.0, 140.0) and k.thickness == 1.6
    assert k.thickness_from.startswith("kicad")
    holes = sorted(kicad_xy(k, (h.x, h.y)) + (h.diameter, h.plated) for h in k.holes)
    assert holes == [(69.3, 24.3, 3.2, False), (69.3, 155.7, 3.2, False), (190.7, 24.3, 3.2, False),
                     (190.7, 155.7, 3.2, False)]                                   # docs/11 mounting holes
    assert any("28 parts are SMD by their pads but not marked SMD" in n for n in k.notes)   # the attribute trap


@pytest.mark.parametrize("ref, side, pos, rot_model, pads", [
    # pad centres and bounding-box sizes from pcbnew 10.0.6 on the same file (2026-10-12)
    ("D2", "F", (82.0, 31.4), 0.0, [("1", 80.35, 31.4, 0.9, 1.2), ("2", 83.65, 31.4, 0.9, 1.2)]),
    ("SW1", "F", (91.4, 74.6), 180.0, [("1", 94.5925, 74.6, 1.985, 1.512), ("2", 88.2075, 74.6, 1.985, 1.512)]),
    ("R41", "B", (164.9, 122.6), 90.0, [("1", 164.9, 123.5125, 1.4, 1.025), ("2", 164.9, 121.6875, 1.4, 1.025)]),
    ("U2", "B", (136.3, 126.8), 270.0, [("1", 137.4491, 125.85, 1.072, 0.532),
                                                    ("4", 135.1509, 127.75, 1.072, 0.532)]),
    ("U14", "B", (155.5, 43.9), 270.0, [("1", 158.2051, 41.995, 1.9502, 0.5684),
                                         ("5", 152.7949, 45.805, 1.9502, 0.5684)]),
])
def test_kicad_positions_angles_and_pads_match_pcbnew(k, ref, side, pos, rot_model, pads):
    c = k.component(ref)
    assert c.side == side and kicad_xy(k, (c.x, c.y)) == pos
    assert angle_diff(c.rotation, rot_model) < 1e-6
    by_num = {p.number: p for p in c.pads}
    for num, x, y, w, h in pads:
        p = by_num[num]
        cx, cy = kicad_xy(k, p.centre)
        assert abs(cx - x) < 0.002 and abs(cy - y) < 0.002, (ref, num, cx, cy)
        x0, y0, x1, y1 = bbox(p.polygon)
        assert abs((x1 - x0) - w) < 0.01 and abs((y1 - y0) - h) < 0.01


def test_the_bottom_rule_puts_pin_1_where_the_footprint_name_says(k):
    """LCSC names give pin 1's corner at 0° (-BL bottom-left, -BR bottom-right). With the model's bottom rule
    (180° − KiCad's angle, seen from below turned over left-right) pin 1 is in that corner on BOTH sides.
    A mirrored rule would put the bottom ones in the wrong corner. This is the geometric check of D10 / D26."""
    want = {"BL": (180, 270), "BR": (270, 360), "TL": (90, 180)}
    seen = set()
    for c in k.components:
        m = checks.PIN1_NAME.search(c.footprint)
        if m is None or len(c.pads) < 3:
            continue
        a = checks.pin1_angle(k, c)
        lo, hi = want[m.group(1)]
        assert lo < a < hi, (c.ref, c.side, c.footprint, a)
        seen.add(c.side)
    assert seen == {"F", "B"}


# ---------------------------------------------------------------- Gerber X3 = KiCad (the Phase B acceptance)
def test_gerber_x3_imports_the_same_board_as_kicad(k, x3):
    assert same_board(k, x3, pos_tol=0.001) == []
    assert x3.summary() == k.summary()
    assert sorted((h.x, h.y, h.diameter) for h in x3.holes) == sorted((h.x, h.y, h.diameter) for h in k.holes)
    assert x3.thickness_from.startswith("gbrjob") and x3.notes == []
    assert {c.ref: c.value for c in x3.components} == {c.ref: c.value for c in k.components}


def plain_gerbers() -> list[tuple[str, bytes]]:
    """The Gerber set without its X3 position files."""
    import zipfile
    with zipfile.ZipFile(BOARDS / "ht5023_gerber_x3.zip") as zz:
        return [(n, zz.read(n)) for n in zz.namelist() if "-pos" not in n]


def test_gerber_without_x3_needs_the_component_files():
    with pytest.raises(ValueError, match="No component \\(X3\\) files"):
        gerber_import.load_files(plain_gerbers())
    ref = gerber_import.load_files(plain_gerbers(), require_components=False)
    assert ref.source_tool == "gerber_x2" and len(ref.components) == 202      # the 4 mounting holes have no copper


# ---------------------------------------------------------------- centroid files
CENTROIDS = {
    "kicad-pos.csv": dict(tool="kicad", units="mm", y_up=True, bottom_x_negated=False, bottom_rule="top_view",
                          origin=(65.0, -160.0)),
    "kicad-pos.pos": dict(tool="kicad", units="mm", y_up=True, bottom_x_negated=False, bottom_rule="top_view",
                          origin=(65.0, -160.0)),
    "synthetic-altium.csv": dict(tool="altium", units="mm", y_up=True, bottom_x_negated=False,
                                 bottom_rule="top_view", origin=(0.0, 0.0)),
    "synthetic-easyeda.csv": dict(tool="easyeda", units="mil", y_up=False, bottom_x_negated=False,
                                  bottom_rule="as_is", origin=(0.0, -140.0)),
    "synthetic-jlc.csv": dict(tool="jlc_csv", units="mm", y_up=True, bottom_x_negated=True, bottom_rule="plus_180",
                              origin=(10.0, 5.0)),
}


@pytest.mark.parametrize("name", sorted(CENTROIDS))
def test_every_centroid_layout_gives_the_same_board(k, name):
    data = (BOARDS / name).read_bytes()
    sn = ce.sniff(data, name)
    want = CENTROIDS[name]
    assert sn.spec.tool == want["tool"] and sn.spec.units == want["units"]
    rows = ce.rows(data, sn.spec)
    assert len(rows) == 206
    fit = ce.suggest(rows, sn.spec, k)
    s = fit.spec
    assert (s.y_up, s.bottom_x_negated, s.bottom_rule) == (want["y_up"], want["bottom_x_negated"], want["bottom_rule"])
    assert abs(s.origin[0] - want["origin"][0]) < 0.01 and abs(s.origin[1] - want["origin"][1]) < 0.01
    assert fit.rotation_agree == 1.0 and fit.position_error < 0.01
    b = ce.to_board(rows, s, "x", k, name)
    assert same_board(k, b, pos_tol=0.05, pads=False) == []


def test_eagle_mnt_and_mnb_together(k):
    rows = []
    sn = None
    for f in ("synthetic-eagle.mnt", "synthetic-eagle.mnb"):
        data = (BOARDS / f).read_bytes()
        sn = ce.sniff(data, f)
        assert sn.spec.tool == "eagle_mnt" and any("No header line" in u for u in sn.unsure)
        rows += ce.rows(data, sn.spec)
    assert {r.side for r in rows} == {"F", "B"} and len(rows) == 206
    fit = ce.suggest(rows, sn.spec, k)
    assert fit.spec.bottom_rule == "plus_180"
    assert same_board(k, ce.to_board(rows, fit.spec, "x", k), 0.05, pads=False) == []


def test_without_a_reference_nothing_is_fitted_and_the_operator_is_told():
    data = (BOARDS / "synthetic-easyeda.csv").read_bytes()
    sn = ce.sniff(data)
    assert any("not checked" in u for u in sn.unsure)
    b = ce.to_board(ce.rows(data, sn.spec), sn.spec, "x", None)
    assert b.outline == [] and any("No reference board" in n for n in b.notes)


def test_gerbers_without_x3_check_angles_only_by_the_pads(k):
    ref = gerber_import.load_files(plain_gerbers(), require_components=False)
    data = (BOARDS / "synthetic-jlc.csv").read_bytes()
    sn = ce.sniff(data)
    rows = ce.rows(data, sn.spec)
    fit = ce.suggest(rows, sn.spec, ref)
    assert fit.position_error < 0.01 and any("180° error is NOT ruled out" in r for r in fit.report)
    b = ce.to_board(rows, fit.spec, "x", ref)                  # the 180° choice is the operator's (preview)
    assert all(math.hypot(c.x - k.component(c.ref).x, c.y - k.component(c.ref).y) < 0.05 for c in b.components)


def test_utf16_and_units_are_read():
    assert ce.decode("a\tb".encode("utf-16")) == "a\tb"
    s = ce.sniff("Ref,X(mil),Y(mil),Rot,Side\nR1,100,200,90,top\n")
    assert s.spec.units == "mil" and ce.rows("Ref,X(mil),Y(mil),Rot,Side\nR1,100,200,90,top\n", s.spec)[0].x == 2.54
    with pytest.raises(ValueError, match="no header line"):
        ce.sniff("hello world\nfoo\n")


# ---------------------------------------------------------------- packages
def test_the_package_library_sorts_the_parts_like_docs_11(k, lib):
    a = annotate(k, lib)
    smd = [r for r in a if k.component(r).smd]
    count = {}
    for r in smd:
        count[a[r]["class"]] = count.get(a[r]["class"], 0) + 1
    assert count == {"machine": 148, "bottom_camera": 12, "by_hand": 7, "pad_only": 2}
    assert sorted(r for r in a if a[r]["class"] == "by_hand") == ["L3", "L4", "SW1", "SW3", "SW4", "SW5", "U1"]
    assert a["C41"]["package"] == "0805"                      # a capacitor on a D_0805 footprint (CLAUDE.md)
    assert a["U2"]["package"] == "SOT-23-6" and a["D13"]["how"] == "alias"
    assert k.component("R41").height == lib.packages["0805"].height


def test_package_library_rules(lib):
    assert lib.identify("Resistor_SMD:R_0402_1005Metric").package.id == "0402"
    assert lib.identify("CAPC1608X90N").how in ("imperial", "none")
    assert lib.identify("C0805").package.id == "0805"
    assert lib.identify("whatever", "SOD-123F").package.id == "SOD-123F"
    assert lib.identify("SOT-23-6_L2.9-W1.6-P0.95-LS2.8-BR").package.id == "SOT-23-6"
    assert lib.identify("mystery").package is None
    assert all(p.pickable in ("yes", "bottom_camera", "by_hand") for p in lib.packages.values())
    with pytest.raises(ValueError, match="unknown packages"):
        Library({"packages": {}}, {"aliases": {"X": "nope"}})


# ---------------------------------------------------------------- checks
def test_checks_are_quiet_on_a_good_import_and_catch_wrong_rules(k, lib):
    a = annotate(k, lib)
    polar = {p.id for p in lib.packages.values() if p.polar}
    assert checks.check(k, a, polar) == []
    b = Board.from_dict(k.to_dict())
    for c in b.components:
        if c.side == "B":
            c.rotation += 90.0                                  # a rule 90° off
    assert sum("check the rotation rule" in i.text for i in checks.check(b, a, polar)) > 100
    b = Board.from_dict(k.to_dict())
    for c in b.components:
        if c.side == "B":
            c.rotation = -c.rotation                            # a mirrored rule: only pin 1 shows it
    assert sum("pin 1" in i.text for i in checks.check(b, a, polar)) >= 8
    b = Board.from_dict(k.to_dict())
    b.components[0].x += 500
    b.components.append(b.components[1])
    texts = [i.text for i in checks.check(b, a, polar)]
    assert any("outside the board outline" in t for t in texts) and any("times" in t for t in texts)


def test_the_model_saves_and_loads_unchanged(k, tmp_path):
    k.save(tmp_path / "b.json")
    b = Board.load(tmp_path / "b.json")
    assert b.to_dict() == Board.from_dict(k.to_dict()).to_dict() and same_board(k, b, 1e-9) == []


# ---------------------------------------------------------------- the readers on small inputs
def test_macro_arithmetic_and_a_roundrect_aperture():
    assert gerber._eval("$1+$1x2", {1: 1.0}) == 3.0 and gerber._eval("($1+2)/3", {1: 1.0}) == 1.0
    assert gerber._eval("-$2x-1", {2: 2.0}) == 2.0
    text = """%FSLAX46Y46*%%MOMM*%
%AMRoundRect*
0 comment*
4,1,4,$2,$3,$4,$5,$6,$7,$8,$9,$2,$3,0*
1,1,$1+$1,$2,$3*
1,1,$1+$1,$4,$5*
1,1,$1+$1,$6,$7*
1,1,$1+$1,$8,$9*%
%TA.AperFunction,SMDPad,CuDef*%
%ADD10RoundRect,0.25X-0.5X-0.75X0.5X-0.75X0.5X0.75X-0.5X0.75X0*%
%TD*%
%TO.P,R1,1,A\\u002CB*%
D10*
X1000000Y2000000D03*
%TD*%
M02*"""
    gf = gerber.parse(text, "t.gbr")
    o = gf.objects[0]
    assert o.function == "SMDPad" and o.attrs[".P"][:2] == ["R1", "1"]
    x0, y0, x1, y1 = bbox(o.polygon(gf.macros))
    assert (round(x1 - x0, 3), round(y1 - y0, 3)) == (1.5, 2.0) and (round((x0 + x1) / 2, 3), round((y0 + y1) / 2, 3)) == (1.0, 2.0)
    assert gerber_import._decode(o.attrs[".P"][2]) == "A,B"                  # an escaped comma in a value


def test_regions_arcs_and_inch_files():
    text = """%FSLAX24Y24*%%MOIN*%
G75*
G36*
X0Y0D02*
X10000Y0D01*
G03X10000Y10000I0J5000D01*
X0Y10000D01*
X0Y0D01*
G37*
M02*"""
    gf = gerber.parse(text)
    r = gf.objects[0]
    assert r.kind == "region"
    x0, y0, x1, y1 = bbox(r.points)
    assert abs(x1 - (25.4 + 12.7)) < 0.05 and abs(y1 - 25.4) < 1e-6        # 1 inch square + the half circle


def test_excellon_formats():
    # LZ = leading zeros kept, trailing ones left out: the digits are read from the left (2.4 format in inch)
    d, _ = gerber.parse_excellon("M48\nINCH,LZ\nT1C0.0320\n%\nT1\nX01Y01\nX015Y-0025\nM30\n")
    assert [(round(x.x, 3), round(x.y, 3), round(x.diameter, 4)) for x in d] == [(25.4, 25.4, 0.8128),
                                                                                  (38.1, -6.35, 0.8128)]
    # TZ = trailing zeros kept, leading ones left out: the digits are read from the right
    d, _ = gerber.parse_excellon("M48\nMETRIC,TZ\nT1C0.8\n%\nT1\nX1500Y-250\nM30\n")
    assert (d[0].x, d[0].y) == (1.5, -0.25)
    d, _ = gerber.parse_excellon("M48\nMETRIC,LZ,000.000\nT2C1.0\n%\nT2\nX010000Y-005000\nX1.5Y2.5G85X3.5Y2.5\nM30\n")
    assert (d[0].x, d[0].y) == (10.0, -5.0) and (d[1].x, d[1].y) == (2.5, 2.5)


def test_kicad_reader_handles_arcs_old_text_custom_pads_and_errors(tmp_path):
    pcb = tmp_path / "t.kicad_pcb"
    pcb.write_text('''(kicad_pcb (version 20211014) (general (thickness 0.8))
  (gr_line (start 0 0) (end 20 0) (layer "Edge.Cuts"))
  (gr_arc (start 20 0) (mid 25 5) (end 20 10) (layer "Edge.Cuts"))
  (gr_line (start 20 10) (end 0 10) (layer "Edge.Cuts"))
  (gr_line (start 0 10) (end 0 0) (layer "Edge.Cuts"))
  (footprint "Lib:R_0603_1608Metric" (layer "F.Cu") (at 5 5 90)
    (fp_text reference "R\\"1" (at 0 0) (layer "F.SilkS"))
    (fp_text value "10k" (at 0 0) (layer "F.Fab"))
    (attr smd)
    (pad "1" smd roundrect (at -0.8 0 90) (size 0.8 0.9) (layers "F.Cu") (roundrect_rratio 0.25))
    (pad "2" smd custom (at 0.8 0 90) (size 0.5 0.5) (layers "F.Cu")
      (options (clearance outline) (anchor rect))
      (primitives (gr_poly (pts (xy -0.4 -0.45) (xy 0.4 -0.45) (xy 0.4 0.45) (xy -0.4 0.45)) (width 0))))))''')
    b = kicad.load(pcb)
    assert b.width == pytest.approx(25.0, abs=0.005) and b.height == pytest.approx(10.0, abs=0.005)   # arc in 2° steps
    assert b.thickness == 0.8
    c = b.component('R"1')
    assert c.value == "10k" and c.rotation == 90 and len(c.pads) == 2 and c.smd
    p2 = next(p for p in c.pads if p.number == "2")
    assert abs(p2.centre[0] - 5.0) < 1e-6 and abs(p2.centre[1] - (10 - 5 + 0.8)) < 1e-6     # rotated 90° CCW
    bad = tmp_path / "bad.kicad_pcb"
    bad.write_text("(kicad_pcb (version 1)")
    with pytest.raises(ValueError, match="open"):
        kicad.load(bad)
    bad.write_text("(kicad_pcb (version 1))")
    with pytest.raises(ValueError, match="no board outline"):
        kicad.load(bad)


# ---------------------------------------------------------------- the service and the page's API
def test_service_imports_saves_and_previews(tmp_path):
    svc = BoardService(tmp_path / "boards", PACKAGES)
    iid = svc.new_import()
    svc.add_file(iid, "../evil/ht5023.kicad_pcb.gz", (BOARDS / "ht5023.kicad_pcb.gz").read_bytes())
    assert [p.name for p in (svc.imports / iid / "files").iterdir()] == ["ht5023.kicad_pcb.gz"]
    r = svc.run(iid)
    assert r["kind"] == "kicad" and r["classes"]["machine"] == 148 and r["issues"] == []
    assert svc.current() is None
    assert svc.save(iid, "HT5023 v1") == "HT5023_v1" and svc.current_name() == "HT5023_v1"
    b = svc.current()
    pv = svc.preview(b, "B")
    r41 = next(c for c in pv["components"] if c["ref"] == "R41")
    assert r41["xy"] == [round(b.width - b.component("R41").x, 3), round(b.component("R41").y, 3)]   # seen from below
    assert r41["pads"] and "pads" not in next(c for c in pv["components"] if c["ref"] == "D2")       # D2 is on top
    svc.set_thickness(1.2)
    assert svc.current().thickness == 1.2 and svc.current().thickness_from == "set by the operator"
    with pytest.raises(ValueError):
        svc.set_thickness(9)
    # a centroid file next: the current board is its reference
    iid2 = svc.new_import()
    svc.add_file(iid2, "synthetic-altium.csv", (BOARDS / "synthetic-altium.csv").read_bytes())
    r = svc.run(iid2)
    assert r["kind"] == "centroid" and r["reference"] == "kicad" and r["fit"]["rotation_agree"] == 1.0
    with pytest.raises(KeyError):
        svc.run("../../etc")


def test_the_page_api_imports_a_board(machine):
    from fastapi.testclient import TestClient

    from pnp.ui.server import create_app
    m = machine(connect=False)
    with TestClient(create_app(m)) as c:
        assert c.get("/api/board").json()["board"] is None
        r = c.post("/api/board/upload?name=ht5023_gerber_x3.zip", content=(BOARDS / "ht5023_gerber_x3.zip").read_bytes())
        iid = r.json()["import_id"]
        r = c.post("/api/board/import", json={"import_id": iid}).json()
        assert r["ok"] and r["kind"] == "gerber_x3" and r["board"]["smd"] == 169
        pv = c.get(f"/api/board/import/{iid}/preview?side=F").json()
        assert pv["classes"]["machine"] == 148 and len(pv["outline"]) >= 1
        assert c.post("/api/board/save", json={"import_id": iid, "name": "ht5023"}).json()["name"] == "ht5023"
        cur = c.get("/api/board?side=B").json()
        assert cur["current"] == "ht5023" and cur["board"]["side"] == "B"
        assert c.post("/api/board/thickness", json={"mm": 0.1}).status_code == 400
        assert c.post("/api/board/import", json={"import_id": "nope"}).status_code == 400


def test_the_command_line_imports(tmp_path, capsys, monkeypatch):
    from pnp.cli import main
    cfg = tmp_path / "cfg"
    cfg.mkdir()
    for f in ("packages.yaml", "packages.user.yaml"):
        (cfg / f).write_bytes((PACKAGES / f).read_bytes())
    assert main(["--config", str(cfg), "import", str(BOARDS / "kicad-pos.csv"), "--no-reference"]) == 0
    out = capsys.readouterr().out
    assert "centroid: 206 parts" in out and "No reference board" in out
    assert main(["--config", str(cfg), "import", str(BOARDS / "ht5023.kicad_pcb.gz"), "--save"]) == 0
    assert "Saved as the current board" in capsys.readouterr().out
    assert main(["--config", str(cfg), "import", str(tmp_path / "missing.csv")]) == 1


def test_fixture_readme_says_what_is_synthetic():
    text = (Path(BOARDS).parent / "make_samples.py").read_text()
    assert "SYNTHETIC" in text and "NOT exports of those" in text
