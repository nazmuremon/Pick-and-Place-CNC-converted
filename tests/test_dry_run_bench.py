"""The dry pick and place (tools/dry_run_bench.py): board maths, hole finding, parts in a slot, and the
board registration and the pick and place against a simulated head and camera."""
import argparse
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import bed_test_run as t  # noqa: E402
import dry_run_bench as d  # noqa: E402
import make_labels as ml  # noqa: E402

J_TRUE = np.array([[-7.26, -0.49], [-0.47, 7.40]])   # px per mm of head move, camera turned ~3.8 deg (bench)
FRAME = (400, 640)
CENTRE = np.array([(FRAME[1] - 1) / 2, (FRAME[0] - 1) / 2])
BENCH = yaml.safe_load((ROOT / "config" / "bench.yaml").read_text())
BED = ml.load_bed()
ZONE = ml.slot_zone_mm(BED)
U_DIR, V_DIR = np.array([0.0, -1.0]), np.array([-1.0, 0.0])   # slot labels run to the front; v points left
SLOT_XY = np.array(BED["layout"]["slot_marker_xy"][3], float)  # slot 3 marker, HOME at 0, 0
PAPER, HOLE, POCKET, BLACK = 180, 126, 148, 72      # tape holes 30 %, empty pockets 18 %, black tops 60 % darker


@pytest.fixture(scope="module")
def board():
    if not d.BOARD_JS.exists():
        pytest.skip(f"no board data: {d.BOARD_JS}")
    return d.load_board()


def rot(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def test_farads_reads_the_spellings_on_the_board():
    for v in ("100nF", "0.1uF", "100n", "0.1 µF", "0.1UF"):
        assert d.farads(v) == pytest.approx(1e-7)
    assert d.farads("10uF") == pytest.approx(1e-5)
    assert d.farads("10K") is None and d.farads("1N4007") is None


def test_parts_to_place_finds_every_100nF_on_the_bottom(board):
    todo = d.parts_to_place(board, "B", "100nF")
    refs = [p["ref"] for p in todo]
    assert len(refs) == 15                                   # the HT5023 v1 B side (dry run 2026-10-06)
    assert {"C6", "C48"} <= set(refs)                        # C6 = '0.1uF', C48 = '100nF'
    assert any(p["fp"].startswith("D_0805") for p in todo)   # capacitors on a D_ footprint are not lost
    assert all(p["side"] == "B" and p["smd"] for p in todo)
    assert [p["y"] for p in todo] == sorted(p["y"] for p in todo)   # front to back
    with pytest.raises(ValueError):
        d.parts_to_place(board, "B", "10k")


@pytest.mark.parametrize("side, top_edge, corner", [
    ("F", "back", ("left", "back")),
    ("F", "front", ("right", "front")),
    ("B", "back", ("right", "back")),
    ("B", "front", ("left", "front")),
])
def test_orientation_puts_hole_H1_in_the_right_corner(side, top_edge, corner):
    """H1 is the KiCad top-left hole (seen from the F side). Lay the board down and see where it ends up."""
    holes = np.array([[69.3, 24.3], [190.7, 24.3], [190.7, 155.7], [69.3, 155.7]])
    bed = (d.orientation(side, top_edge) @ holes.T).T
    x = "left" if bed[0, 0] == bed[:, 0].min() else "right"
    y = "front" if bed[0, 1] == bed[:, 1].min() else "back"
    assert (x, y) == corner


def test_fit_affine_and_describe_give_back_scale_angle_and_shift():
    src = np.array([[0, 0], [120, 0], [120, 130], [0, 130]], float)
    M_true = rot(0.35) @ np.diag([0.995, 0.999])
    dst = src @ M_true.T + (22.0, 2.3)
    M, c = d.fit_affine(src, dst)
    assert np.allclose(M, M_true) and np.allclose(c, (22.0, 2.3))
    f = d.describe(M)
    assert f["scale_x"] == pytest.approx(0.995, abs=1e-4) and f["scale_y"] == pytest.approx(0.999, abs=1e-4)
    assert f["angle_deg"] == pytest.approx(0.35, abs=0.01) and f["skew_deg"] == pytest.approx(0.0, abs=0.01)


def test_best_hole_set_picks_the_holes_among_pads_and_refuses_a_far_board():
    q = np.array([[69.3, 24.3], [190.7, 24.3], [190.7, 155.7], [69.3, 155.7]])
    plan = np.array([-209.0, -244.0])
    true = q @ rot(0.4).T + plan + (20.0, 3.0)
    rng = np.random.default_rng(1)
    cands, where = [], []
    for p in true:                                 # each hole hidden among pads and vias around it
        pts = p + rng.uniform(-15, 15, (8, 2))
        k = int(rng.integers(0, 9))
        cands.append(np.insert(pts, k, p, axis=0))
        where.append(k)
    idx, res, M, c = d.best_hole_set(cands, q, plan)
    assert list(idx) == where and res.max() < 1e-6
    assert d.best_hole_set([c_ + (60.0, 0.0) for c_ in cands], q, plan) is None   # 80 mm off the plan


def test_hole_candidates_rank_a_hole_above_a_round_pad():
    """A hole is light inside (the bed shows through); a round pad is a light ring around a dark drill."""
    img = np.full((200, 300), 60, np.uint8)
    cv2.circle(img, (80, 100), 12, 190, -1)            # hole, 3.2 mm at ~7.4 px/mm
    cv2.circle(img, (210, 100), 10, 220, 5)            # pad ring with a dark centre
    found = d.hole_candidates(img, r_px=11.8)
    assert len(found) >= 1
    assert np.hypot(*(found[0] - (80, 100))) < 1.5


class World:
    """What the camera sees, in machine mm: a dark bed with the board's holes (bright) and round pads, and
    the white slot 3 label with a strip of tape: grey tape holes, grey empty pockets, black-topped parts.
    bend: lens bending. A pixel r mm (by J) from the picture centre really sees r * (1 + bend * r^2) mm.
    aa: samples per pixel side. 1 = hard edges (fast); 3 = grey edges like a real lens gives (sub-pixel)."""

    def __init__(self, bend: float = 0.0, aa: int = 1) -> None:
        self.holes, self.rings, self.tape_holes, self.spots = [], [], [], []
        self.paper = []      # (corner, opposite corner): white paper or tape on the bed
        self.marks = []      # (centre, radius): nozzle marks on the paper
        self.placed = []     # (centre, length, width, angle deg from +X): black-topped parts lying anywhere
        self.label = False
        self.bend, self.aa = bend, aa

    @staticmethod
    def slot_xy(u, v) -> np.ndarray:
        return SLOT_XY + u * U_DIR + v * V_DIR

    def strip(self, spots) -> "World":
        """spots: (u, v, length across the slot, width along it, grey) in the slot 3 frame."""
        self.label = True
        self.tape_holes = [self.slot_xy(u, 3.0) for u in np.arange(9.0, 76.0, 4.0)]
        self.spots = [(self.slot_xy(u, v), length, width, grey) for u, v, length, width, grey in spots]
        return self

    def picture(self, pos) -> np.ndarray:
        n = self.aa
        if n > 1:
            subs = [self._picture(pos, ((i + 0.5) / n - 0.5, (j + 0.5) / n - 0.5)).astype(float)
                    for i in range(n) for j in range(n)]
            return np.round(np.mean(subs, axis=0)).astype(np.uint8)
        return self._picture(pos, (0.0, 0.0))

    def _picture(self, pos, sub) -> np.ndarray:
        h, w = FRAME
        v, u = np.mgrid[:h, :w]
        lin = np.linalg.solve(J_TRUE, (CENTRE - np.c_[u.ravel() + sub[0], v.ravel() + sub[1]]).T).T
        mm = pos + lin * (1 + self.bend * np.sum(lin ** 2, axis=1, keepdims=True))   # what each pixel sees
        img = np.full(h * w, 55, np.uint8)
        if self.label:
            rel = mm - SLOT_XY
            img[(rel @ U_DIR > -10) & (rel @ U_DIR < 80) & (np.abs(rel @ V_DIR) < 10)] = PAPER
        for lo, hi in self.paper:
            lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
            img[np.all((mm >= lo) & (mm <= hi), axis=1)] = PAPER
        for c in self.tape_holes:
            img[np.hypot(*(mm - c).T) < 0.75] = HOLE
        for c, length, width, grey in self.spots:
            rel = mm - c
            img[(np.abs(rel @ V_DIR) <= length / 2) & (np.abs(rel @ U_DIR) <= width / 2)] = grey
        for c, length, width, ang in self.placed:
            e = np.array([np.cos(np.radians(ang)), np.sin(np.radians(ang))])
            rel = mm - c
            img[(np.abs(rel @ e) <= length / 2) & (np.abs(rel @ np.array([-e[1], e[0]])) <= width / 2)] = BLACK
        for c, r in self.marks:
            img[np.hypot(*(mm - c).T) <= r] = 60
        for r in self.rings:
            dist = np.hypot(*(mm - r).T)
            img[(dist > 0.9) & (dist < 1.6)] = 210
        for hole in self.holes:
            img[np.hypot(*(mm - hole).T) < d.HOLE_D_MM / 2] = 185
        return img.reshape(h, w)


def test_find_slot_parts_leaves_holes_pockets_and_dirt_out_and_never_guesses():
    w = World().strip([
        (21, -0.5, 2.2, 1.4, BLACK), (25, -0.5, 2.2, 1.4, BLACK), (29, -0.5, 2.2, 1.4, BLACK),
        (33, -0.5, 2.4, 1.6, POCKET),                                   # empty pocket
        (37, -0.5, 2.2, 1.4, BLACK), (38.6, -0.5, 2.2, 1.4, BLACK),     # 0.2 mm apart
        (45, 6.8, 3.0, 1.5, BLACK),                                     # over the edge of the tape mark
        (50, 0.0, 0.4, 0.4, BLACK),                                     # dirt
        (53, 0.0, 9.0, 1.5, BLACK),                                     # a piece of black tape
        (58.8, -0.5, 2.2, 1.4, BLACK),                                  # cut by the picture edge
    ])
    pos = w.slot_xy(32, 0)
    parts = d.find_slot_parts(w.picture(pos), J_TRUE, pos, CENTRE, SLOT_XY, U_DIR, V_DIR, ZONE, BENCH["slot_parts"])
    assert [p["uv"][0] for p in parts] == pytest.approx([21, 25, 29, 37, 38.6, 45, 53], abs=0.15)
    assert [p["unclear"] for p in parts] == [None, None, None, "touches another dark spot",
                                             "touches another dark spot", "across the edge of the tape mark",
                                             "too big for one part"]
    for p in parts[:3]:
        assert p["size_mm"] == pytest.approx((2.2, 1.4), abs=0.25)
        assert abs(abs(p["angle_deg"]) - 90) < 5                     # long side across the slot
        assert np.hypot(*(p["xy"] - w.slot_xy(*p["uv"]))) < 1e-9


def test_find_slot_parts_on_a_real_picture_of_slot_3():
    """Xitech camera, 2026-10-06: a cut strip of 8 pockets on slot 3, black-topped parts in pockets 3-5."""
    img = cv2.imread(str(ROOT / "tests" / "data" / "slot3_real_parts.png"), cv2.IMREAD_GRAYSCALE)
    J = np.array([[-7.1348, -0.4454], [-0.5861, 7.2052]])          # that run's camera calibration
    home = np.array([3.799, -0.026])
    u_dir, v_dir = np.array([0.019, -1.0]), np.array([-1.0, -0.013])
    cfg = {**BENCH["slot_parts"], "dark": 0.40, "max_dark": 1.0, "min_aspect": 0.0}   # that strip (white label under it)
    parts = d.find_slot_parts(img, J, home + (-29.369, -33.698), CENTRE, home + (-29.368, -1.7),
                              u_dir / np.linalg.norm(u_dir), v_dir / np.linalg.norm(v_dir), ZONE, cfg)
    assert [p["unclear"] for p in parts] == [None, None, None]
    assert [p["uv"][0] for p in parts] == pytest.approx([19.1, 23.1, 27.2], abs=0.3)
    assert all(2.0 < p["size_mm"][0] < 2.6 and 1.2 < p["size_mm"][1] < 1.7 for p in parts)


class FakeGrbl:
    """Z only: the dry run's dips go to Grbl directly. fail_at: that move stops halfway with an error."""

    def __init__(self, fail_at: int | None = None) -> None:
        self.z, self.moves, self.fail_at = 0.0, [], fail_at

    def status(self):
        return "Idle", {"X": 0.0, "Y": 0.0, "Z": self.z}

    def move_axes(self, dists, feed):
        assert set(dists) == {"Z"} and feed <= 500          # Grbl's max rate for the Z motor ($110)
        dz = dists["Z"]
        if len(self.moves) == self.fail_at:
            self.z += dz / 2
            self.moves.append(dz / 2)
            raise d.GrblError("the link dropped halfway")
        self.z += dz
        self.moves.append(dz)


class SimDryRun(d.DryRun):
    """The dry-run jobs with a fake head (no lost steps) and a fake camera looking at a World."""

    def __init__(self, board, world: World, tmp_path) -> None:
        self.board, self.bed, self.out_dir, self.world = board, BED, tmp_path, world
        self.args = argparse.Namespace(side="B", top_edge="front", value="100nF", slot=3)
        self.A = d.orientation("B", "front")
        parts = {p["ref"]: p for p in board["parts"]}
        self.hole_xy = np.array([[parts[h]["x"], parts[h]["y"]] for h in d.HOLES])
        self.todo = d.parts_to_place(board, "B", "100nF")
        self.parts_cfg, self.dry_cfg = BENCH["slot_parts"], BENCH["dry_run"]
        self.reach = BENCH["limits"]["reach_from_home"]
        self.grbl, self.z = FakeGrbl(), 0.0
        self.home, self.pos, self.J = np.zeros(2), np.zeros(2), J_TRUE
        self.nominal = {"home": np.zeros(2), "slot_3": SLOT_XY.copy()}
        self.frame_shape, self.report, self.overlay, self.messages = FRAME, {}, [], []
        self.M = self.c = None
        self.step_name, self.moving, self.msg = "", False, ""
        self.lock = threading.Lock()
        self.markers = type("NoMarkers", (), {"find": staticmethod(lambda f: {})})()
        self.cam = self

    def say(self, text):
        self.messages.append(text)

    def move(self, dist):
        rel = self.pos + np.asarray(dist, float) - self.home
        assert self.in_reach(rel), f"move outside the reach: {rel}"
        self.pos = self.pos + np.asarray(dist, float)

    def frames_after(self, t0, n):
        return [self.world.picture(self.pos)] * n

    def locate(self, name, **kw):
        self.move_to(self.clip_to_reach(SLOT_XY))
        return SLOT_XY.copy()

    def look(self, name, frames=3):
        corners = SLOT_XY + np.array([[-5, -5], [5, -5], [5, 5], [-5, 5]]) @ np.vstack([U_DIR, V_DIR])
        return CENTRE - (corners - self.pos) @ J_TRUE.T


def test_register_finds_a_shifted_turned_board_and_the_parts_land_on_their_pads(board, tmp_path, monkeypatch):
    monkeypatch.setattr(d.time, "sleep", lambda s: None)
    world = World()
    sim = SimDryRun(board, world, tmp_path)
    T = sim.nominal_T() + (22.0, 2.3)                # like the real run: 22 mm right of the plan, turned 0.35 deg
    M_true = rot(0.35) @ np.diag([0.995, 0.999])     # and the X steps/mm 0.5 % off

    def true_map(xy):                                # board mm -> machine mm, where the board really lies
        return np.asarray(xy, float) @ M_true.T + T

    world.holes = true_map(sim.hole_xy)
    rng = np.random.default_rng(2)                   # round pads near each hole: they must not win
    world.rings = np.concatenate([h + rng.uniform(-12, 12, (3, 2)) for h in world.holes])

    assert sim.register()
    reg = sim.report["registration"]
    assert max(reg["residuals_mm"]) < 0.1
    assert reg["board_angle_deg"] == pytest.approx(0.35, abs=0.05)
    assert reg["scale_x"] == pytest.approx(0.995, abs=0.002)
    for part in sim.todo:
        for pad in part["pads"]:
            centre = np.array(pad["p"][0]).reshape(-1, 2).mean(axis=0)
            err = np.hypot(*(sim.board_to_machine(centre) - true_map(centre)))
            assert err < 0.1, f"{part['ref']} pad {pad['n']}: {err:.3f} mm off"


def test_pick_and_place_takes_the_real_parts_then_asks_for_a_refill(board, tmp_path, monkeypatch):
    monkeypatch.setattr(d.time, "sleep", lambda s: None)
    part_uv = [(19.1, -0.4), (23.1, -0.4), (27.2, -0.6)]           # where the real run found them
    world = World().strip([(u, v, 2.2, 1.4, BLACK) for u, v in part_uv] + [(31.1, -0.5, 2.4, 1.6, POCKET)])
    sim = SimDryRun(board, world, tmp_path)
    sim.M, sim.c = np.eye(2), sim.nominal_T()        # the board lies where the plan puts it

    sim.pick_and_place()
    picks = sim.report["picks"]
    assert [p["part"] for p in picks] == [1, 2, 3]
    for p, (u, v) in zip(picks, part_uv):
        assert np.hypot(*(np.array(p["machine_xy_from_home"]) - world.slot_xy(u, v))) < 0.1
        assert p["centre_error_mm"] < 0.1
    assert [q["ref"] for q in sim.report["placements"]] == [q["ref"] for q in sim.todo[:3]]
    assert [q["ref"] for q in sim.report["not_placed"]] == [q["ref"] for q in sim.todo[3:]]
    assert any("slot_3 is empty" in m for m in sim.messages)
    assert (tmp_path / f"pick_{sim.todo[0]['ref']}.png").exists()
    dip = float(BENCH["dry_run"]["z_dip_mm"])
    assert sim.grbl.moves == [-dip, dip] * 6                  # down and up at 3 picks and 3 places
    assert sim.grbl.z == 0 and sim.z == 0


def test_dip_brings_z_back_up_even_when_the_move_down_fails(board, tmp_path, monkeypatch):
    monkeypatch.setattr(d.time, "sleep", lambda s: None)
    sim = SimDryRun(board, World(), tmp_path)
    sim.grbl = FakeGrbl(fail_at=0)
    with pytest.raises(d.GrblError):
        sim.dip("pick 1")
    assert sim.grbl.z == pytest.approx(0.0) and sim.z == pytest.approx(0.0)


@pytest.mark.parametrize("bend", [1.9e-4, -1.9e-4])     # ~1.5 mm off at 20 mm from the picture centre, both ways
def test_search_slot_counts_every_part_once_even_with_a_bent_lens(board, tmp_path, monkeypatch, bend):
    """Run 2026-10-06 17:56: a full strip, and parts far from the picture centre were counted twice."""
    monkeypatch.setattr(d.time, "sleep", lambda s: None)
    us = 19.0 + 4.0 * np.arange(15)                 # a full strip in the tape mark: 15 parts at 4 mm pitch
    world = World(bend).strip([(u, -0.5, 2.2, 1.4, BLACK) for u in us])
    sim = SimDryRun(board, world, tmp_path)
    parts = sim.search_slot("slot_3", (SLOT_XY, U_DIR, V_DIR))
    assert [p["unclear"] for p in parts] == [None] * 15
    assert [p["uv"][0] for p in parts] == pytest.approx(us, abs=1.0)


def test_find_home_stops_with_a_message_when_no_label_is_in_view(board, tmp_path, monkeypatch):
    """Run 2026-10-07: the head was moved while the PC was off; the drive back found no label (crashed)."""
    monkeypatch.setattr(t, "POSITION_FILE", tmp_path / "head_position.yaml")   # no memory: cannot drive back
    monkeypatch.setattr(t, "CAMERA_FILE", tmp_path / "camera_calibration.yaml")
    sim = SimDryRun(board, World(), tmp_path)        # the camera sees no label
    sim.home = sim.J = None                          # a fresh start: not homed, never calibrated
    assert sim.find_home() is False
    assert "jog near HOME" in sim.messages[-1]


def test_holes_by_shift_finds_the_board_when_one_hole_is_missing():
    """Run 2026-10-07: the board was 27 mm off the plan and pads pushed one hole out of its candidates."""
    q = np.array([[69.3, 24.3], [190.7, 24.3], [190.7, 155.7], [69.3, 155.7]])
    guess = q + (-209.0, -244.0)
    true = q @ rot(-0.3).T + (-209.0, -244.0) + (26.5, -21.5)
    rng = np.random.default_rng(5)
    cands = [np.vstack([true[k], true[k] + rng.uniform(-30, 30, (11, 2))]) for k in range(4)]
    cands[1] = cands[1][1:]                              # H2 itself is not among its candidates
    P = d.holes_by_shift(cands, guess, q)
    assert P is not None and np.abs(P - true).max() < 0.05
    assert d.holes_by_shift([cands[0], cands[1][:0], cands[2][1:], cands[3]], guess, q) is None   # only 2 agree


def test_register_works_when_one_hole_cannot_be_seen(board, tmp_path, monkeypatch):
    monkeypatch.setattr(d.time, "sleep", lambda s: None)
    world = World()
    sim = SimDryRun(board, world, tmp_path)
    # where the board lay in this test before the PCB labels moved the plan (D21): inside the old reach
    T = sim.nominal_T() + (np.array([-144.0, -224.0]) - BED["layout"]["pcb_area_xy"]) + (26.5, -21.5)

    def true_map(xy):
        return np.asarray(xy, float) @ rot(-0.3).T + T

    holes = true_map(sim.hole_xy)
    rng = np.random.default_rng(2)
    world.rings = np.concatenate([h + rng.uniform(-12, 12, (3, 2)) for h in holes])
    world.holes = np.delete(holes, 1, axis=0)        # H2 covered: the camera never sees it
    assert sim.register()
    for part in sim.todo:
        centre = np.array(part["pads"][0]["p"][0]).reshape(-1, 2).mean(axis=0)
        assert np.hypot(*(sim.board_to_machine(centre) - true_map(centre))) < 0.15, part["ref"]


def test_places_outside_the_reach_are_refused(board, tmp_path, monkeypatch):
    monkeypatch.setattr(d.time, "sleep", lambda s: None)
    world = World().strip([(u, -0.5, 2.2, 1.4, BLACK) for u in (19.0, 23.0, 27.0)])
    sim = SimDryRun(board, world, tmp_path)
    sim.M, sim.c = np.eye(2), sim.nominal_T() + (60.0, -30.0)     # the board pushed far right and to the front
    outside = [p["ref"] for p in sim.todo if not sim.in_reach(sim.board_to_machine([p["x"], p["y"]]) - sim.home)]
    assert outside                                                  # the test needs some
    sim.pick_and_place()
    refused = [q["ref"] for q in sim.report["not_placed"] if q["why"] == "outside the reach"]
    assert refused == [r for r in outside if r in refused] and set(refused) <= set(outside)
    placed = [q["ref"] for q in sim.report["placements"]]
    assert not set(placed) & set(outside) and len(placed) == 3


def test_no_blind_dips_with_a_nozzle_on_the_head(board, tmp_path, monkeypatch):
    monkeypatch.setattr(d.time, "sleep", lambda s: None)
    sim = SimDryRun(board, World(), tmp_path)
    sim.nozzle_mounted = True
    sim.dip("pick 1")
    sim.dip("place C1")
    assert sim.grbl.moves == [] and sum("no Z dips" in x for x in sim.messages) == 1
