"""Phase E2: the parts store and the hand-assembly view (Solder Helper merged, docs/13 §4.15): the drawers come out
as on today's page, bag codes find their line (LCSC, MPN, links), a scanned bag says "tray" or "drawer", the hand list
follows Top SMD -> Bottom SMD -> THT, one placed state for the machine and the hand view, labels."""
from __future__ import annotations

import json

import pytest

from conftest import BOARDS
from pnp.store.lines import board_lines, cat_of, value_num
from pnp.store.drawers import auto_assign
from pnp.store.scan import find_line, parse_code
from pnp.store.store import PartsStore


@pytest.fixture
def board_m(machine):
    m = machine(connect=False)
    iid = m.boards.new_import()
    m.boards.add_file(iid, "ht5023.kicad_pcb.gz", (BOARDS / "ht5023.kicad_pcb.gz").read_bytes())
    m.boards.run(iid)
    m.boards.save(iid, "ht5023")
    return m


def test_the_drawers_are_the_same_as_on_todays_page(board_m):
    exp = json.loads((BOARDS / "solder_helper_drawers.json").read_text())
    b = board_m.boards.current()
    lines = board_lines(b)
    assert [ln.key for ln in lines] == exp["lines"]                 # 94 lines, the same order
    for n in ("10", "25"):
        assert auto_assign(lines, int(n)) == exp["drawers"][n]


def test_kinds_and_values_sort_like_the_old_page():
    assert [cat_of(r) for r in ("R1", "C2", "FB1", "LED3", "Y1", "CN4", "SW1")] == ["R", "C", "L", "D", "X", "J", "M"]
    assert value_num("4.7k") == pytest.approx(4700) and value_num("100nF") == pytest.approx(1e-7)


def test_a_bag_code_finds_its_line(board_m):
    lines = board_lines(board_m.boards.current())
    ln = next(x for x in lines if x.lcsc)
    code = next(iter(ln.lcsc))
    assert parse_code(f"{{pbn:PICK2210,on:SO2210,pc:{code},pm:XYZ,qty:100,mc:,cc:1}}") == \
        {"pc": code, "pm": "XYZ", "qty": 100}
    assert find_line(parse_code(code), lines, {}).key == ln.key
    with_mpn = next(x for x in lines if x.mpn)
    assert find_line({"pc": "", "pm": next(iter(with_mpn.mpn)).lower()}, lines, {}).key == with_mpn.key
    assert find_line(parse_code("C999999999"), lines, {}) is None
    assert find_line(parse_code("C999999999"), lines, {"C999999999": ln.key}).key == ln.key   # a link by hand


def test_the_store_keeps_moves_links_and_quantities(tmp_path, board_m):
    lines = board_lines(board_m.boards.current())
    st = PartsStore(tmp_path / "store.json")
    st.set_count(12)
    k = lines[0].key
    st.move(k, 7, lines)
    assert st.drawers(lines)[k] == 7
    r = st.scan("{pc:C999999999,pm:,qty:5}", lines)
    assert not r["ok"]
    st.link("C999999999", lines[3].key)
    r = st.scan("{pc:C999999999,pm:,qty:5}", lines)
    assert r["ok"] and r["line"]["key"] == lines[3].key
    st2 = PartsStore(tmp_path / "store.json")                        # on the server: every browser sees the same
    assert st2.drawers(lines)[k] == 7 and st2.d["qty"][lines[3].key] == 5
    with pytest.raises(ValueError):
        st.set_count(1000)


def test_a_scanned_bag_says_tray_or_drawer(board_m):
    m = board_m
    lines = board_lines(m.boards.current())
    from pnp.feeders.packages import annotate
    cls = annotate(m.boards.current(), m.boards.library())
    machine_ln = next(x for x in lines if x.lcsc and any(cls[r]["class"] == "machine" for r in x.refs))
    hand_ln = next(x for x in lines if x.lcsc and all(cls[r]["class"] in ("tht", "by_hand") for r in x.refs))
    m.feeder_state.load_line(4, machine_ln.feeder_key, machine_ln.value, machine_ln.feeder_key.split("|")[1])
    r = m.parts.scan(next(iter(machine_ln.lcsc)))
    assert r["ok"] and r["where"] == "Tray 4"
    r = m.parts.scan(next(iter(hand_ln.lcsc)))
    assert r["ok"] and r["where"].startswith("Drawer") and "by hand" in r["where"]
    r = m.parts.scan(next(iter(hand_ln.lcsc)), tray=5)               # a by-hand part never goes into a tray
    assert not r["ok"]
    r = m.parts.scan(next(iter(machine_ln.lcsc)), tray=3)            # scan to load: the bag decides the line (R5)
    assert r["ok"] and m.feeder_state.get(3)["line"] == machine_ln.feeder_key and m.feeder_state.get(3)["scanned"]


def test_the_hand_list_and_the_shared_placed_state(board_m):
    m = board_m
    h = m.parts.hand()
    p = h["passes"]
    assert [p[k]["name"] for k in ("F", "B", "T")] == ["Top SMD", "Bottom SMD", "THT by hand"]
    assert p["F"]["total"] >= 31 and p["B"]["total"] >= 136 and p["T"]["total"] > 20     # docs/11: 31 top, 136 bottom SMD
    u1 = next(r for g in p["F"]["groups"] + p["B"]["groups"] for r in g["refs"] if r["ref"] == "U1")
    assert "by hand" in u1["why"]                                     # the LQFP-100: the reason is shown
    allrefs = [r["ref"] for ps in p.values() for g in ps["groups"] for r in g["refs"]]
    a, b = allrefs[0], allrefs[1]
    m.placed.mark("ht5023", a, "machine")                            # the machine placed it
    m.parts.mark([b], True)                                          # by hand
    h = m.parts.hand()
    refs = {r["ref"]: r for ps in h["passes"].values() for g in ps["groups"] for r in g["refs"]}
    assert refs[a]["placed"] and refs[a]["by"] == "machine"
    assert refs[b]["placed"] and refs[b]["by"] == "hand"
    assert b in m.placed.get("ht5023")                               # the job planner skips it ("already placed")


def test_the_old_solder_helper_progress_imports_once(board_m):
    m = board_m
    lines = board_lines(m.boards.current())
    r = m.parts.import_old({"placed": ["R2", "C3"], "drawers": {"count": 14, "over": {lines[0].key: 9}},
                            "links": {"C12345": lines[1].key}})
    assert r["ok"] and r["placed"] == 2
    assert {"R2", "C3"} <= set(m.placed.get("ht5023"))
    assert m.parts.store.d["drawer_count"] == 14 and m.parts.drawers_view()["count"] == 14
    assert m.parts.store.drawers(lines)[lines[0].key] == 9


def test_labels_as_a_pdf(board_m):
    m = board_m
    from pnp.store.labels import PdfPrinter, label_image
    img = label_image("7", ["100nF 0805", "10k 0603"], 76)
    assert img.height == 76 and img.width > 76
    r = m.parts.print_labels("drawers")                              # the bench profile: ptouch; here no printer
    if m.profile.label_printer.type == "ptouch":
        assert not r["ok"] and ("not" in r["why"].lower() or "fail" in r["why"].lower())
    out = PdfPrinter(m.data / "labels" / "t.pdf").print([img] * 30)
    assert out["ok"] and (m.data / "labels" / "t.pdf").stat().st_size > 1000


def test_the_hand_tab_is_served_with_its_qr_decoders(machine):
    from fastapi.testclient import TestClient
    from pnp.ui.server import create_app
    m = machine(connect=False)
    c = TestClient(create_app(m))
    html = c.get("/").text
    assert 'id="tab-hand"' in html and "/vendor/zxing-reader.js" in html and "/static/hand.js" in html
    assert c.get("/vendor/zxing_reader.wasm").headers["content-type"] == "application/wasm"
    assert c.get("/vendor/../server.py").status_code == 404
    assert c.get("/api/store/hand").json()["ok"] is False             # no board yet: says so
