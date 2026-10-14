"""The pick and place engine behind the web page (ui/app.py).

It is the bench machine of tools/nozzle_calibrate.py (camera calibration, HOME, the remembered camera height D19,
the heights k / d / m / t, the nozzle offsets, the real one-part pick and place with all its locks), plus what the
page needs:
  - the reach: drive to the 4 ends by hand, then save it (the machine has no limit switches),
  - the board: 4 corner holes set by hand (click the hole in the picture, then "Set"), or found automatically,
  - the feeders: which component line (value + package) is in which slot,
  - the job: every assigned slot, part by part, with pause / resume / stop and a refill stop when a slot is empty,
  - the live picture with the overlays (red cross, labels, nozzle spot, board holes and pads).
Settings: config/bench.yaml, config/bed.yaml. Saved data: config/feeder.yaml, config/board_registration.yaml,
config/nozzle_calibration.yaml, logs/job_state.yaml (parts already placed on this board).
"""
from __future__ import annotations

import datetime
import re
import sys
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import bed_test_run as t  # noqa: E402
import dry_run_bench as d  # noqa: E402
import make_labels as ml  # noqa: E402
import nozzle_calibrate as nc  # noqa: E402
from bench import load_bench  # noqa: E402
from pnp_paths import CONFIG_DIR, LOGS_DIR  # noqa: E402

FEEDER_FILE = CONFIG_DIR / "feeder.yaml"
FEEDER_EXAMPLE = CONFIG_DIR / "feeder.example.yaml"
REG_FILE = CONFIG_DIR / "board_registration.yaml"
JOB_FILE = LOGS_DIR / "job_state.yaml"
BED_FILE = CONFIG_DIR / "bed.yaml"
BENCH_FILE = CONFIG_DIR / "bench.yaml"
HEIGHT_NAMES = ("part", "place", "move", "paper")

# Bed survey (wide view, camera high): short camera-checked steps; near the front labels even shorter, because
# the front end of travel is only ~3 mm past the point where the front PCB labels come into view (owner, 2026-10-08)
SURVEY_STEP_MM = 10.0
SURVEY_SLOW_STEP_MM = 1.0
SURVEY_SLOW_MM = 60.0          # this far (mm) before the planned front labels reach the picture: slow steps
SURVEY_MAX_FRONT_MM = 300.0
SURVEY_LEFT_MIN_MM = -130.0    # never further left than this from HOME (the old reach ended near -154)
SURVEY_FEED = 600.0            # mm/min
SURVEY_EDGE_PX = 4             # a marker this close to the picture edge may be cut off: not measured


# ---------------------------------------------------------------- bed survey maths

def best_sightings(sightings: dict[str, list]) -> dict[str, dict]:
    """name -> the sighting nearest the picture centre (least lens bending), with the spread of all of them.
    A sighting is (distance from the picture centre in px, xy mm from HOME)."""
    out = {}
    for name, seen in sightings.items():
        if not seen:
            continue
        dist, xy = min(seen, key=lambda s: s[0])
        pts = np.array([s[1] for s in seen], float)
        out[name] = {"xy": np.asarray(xy, float), "centre_px": float(dist), "views": len(seen),
                     "spread_mm": float(np.max(np.hypot(*(pts - pts.mean(axis=0)).T))) if len(seen) > 1 else 0.0}
    return out


def pcb_area_from_labels(pcb: dict[str, np.ndarray], size_mm, marker_mm: float, quiet_mm: float,
                         holders: dict | None = None) -> tuple[list, dict]:
    """The board area between the 4 PCB corner labels (PCB 1 front-left, 2 front-right, 3 back-right, 4 back-left):
    the board centred between the marker centres. Returns the front-left corner (mm from HOME) and the gaps from
    each board edge to the nearest label edge (to check the labels against the board).
    With the 3D printed corner pieces (hardware/tray/pcb_corner.py, bed.yaml pcb_area.corner_holders) the board
    lies in the back-left L instead: left edge = the left markers + left_arm_mm, back edge = the back markers +
    edge_mm. The gaps then say how far the right arm's end and the front marks are from the board's edges."""
    p = {i: np.asarray(pcb[f"pcb_{i}"], float) for i in range(1, 5)}
    if holders:
        w, h = float(size_mm[0]), float(size_mm[1])
        edge = float(holders["edge_mm"])
        left = float(p[1][0] + p[4][0]) / 2 + float(holders["left_arm_mm"])
        back = float(p[3][1] + p[4][1]) / 2 + edge
        right_mark = float(p[2][0] + p[3][0]) / 2 - float(holders["right_arm_mm"])
        front_mark = float(p[1][1] + p[2][1]) / 2 + edge
        info = {"by": "corner pieces (board in the back-left L)",
                "gap_x_mm": round(right_mark - (left + w), 1),    # + = room at the right edge
                "gap_y_mm": round((back - h) - front_mark, 1),    # + = the front marks are in front of the board
                "square_deg": round(float(np.degrees(np.arctan2((p[3] - p[4])[1], (p[3] - p[4])[0]))), 2)}
        return [round(left, 1), round(back - h, 1)], info
    left, right = (p[1][0] + p[4][0]) / 2, (p[2][0] + p[3][0]) / 2
    front, back = (p[1][1] + p[2][1]) / 2, (p[3][1] + p[4][1]) / 2
    w, h = float(size_mm[0]), float(size_mm[1])
    left, right, front, back = (float(v) for v in (left, right, front, back))
    cx, cy = (left + right) / 2, (front + back) / 2
    half = marker_mm / 2 + quiet_mm                    # marker centre -> label edge
    info = {"marker_box_mm": [round(right - left, 1), round(back - front, 1)],
            "gap_x_mm": round((right - left - w) / 2 - half, 1),   # board side edge -> label edge (label is
            "gap_y_mm": round((back - front - h) / 2 - half, 1),   # beside the board corner)
            "square_deg": round(float(np.degrees(np.arctan2((p[2] - p[1])[1], (p[2] - p[1])[0]))), 2)}
    return [round(cx - w / 2, 1), round(cy - h / 2, 1)], info


def layout_text(text: str, slots: dict[int, np.ndarray], pcb: dict[str, np.ndarray] | None,
                area: list | None, stamp: str) -> str:
    """config/bed.yaml with the measured label positions written into layout (the comments are kept)."""
    for n, xy in slots.items():
        text = re.sub(rf"(?m)^(    {int(n)}: )\[[^\]]*\]", rf"\g<1>[{xy[0]:.1f}, {xy[1]:.1f}]", text)
    if pcb:
        pts = ", ".join(f"[{pcb[f'pcb_{i}'][0]:.1f}, {pcb[f'pcb_{i}'][1]:.1f}]" for i in range(1, 5))
        text = re.sub(r"(?m)^(  pcb_corner_marker_xy: ).*$",
                      rf"\g<1>[{pts}]   # PCB 1-4 marker centres, bed survey {stamp}", text)
    if area:
        text = re.sub(r"(?m)^(  pcb_area_xy: )\[[^\]]*\]", rf"\g<1>[{area[0]:.1f}, {area[1]:.1f}]", text)
    return text


# ---------------------------------------------------------------- component lines

def ohms(value: str) -> float | None:
    """'10k', '4.7K', '1k5', '100R', '2.2M', '50R', '0R', '100' -> ohms. None if it is not a resistance."""
    v = value.strip().replace("Ω", "").replace("ohm", "").replace(" ", "")
    m = re.fullmatch(r"(\d+(?:\.\d+)?)([rRkKmM]?)(\d*)", v)
    if not m:
        return None
    mult = {"": 1.0, "r": 1.0, "k": 1e3, "m": 1e6}[m.group(2).lower()]
    whole = float(m.group(1))
    if m.group(3):                                   # 1k5 = 1.5 k
        whole = float(f"{m.group(1)}.{m.group(3)}")
    return whole * mult


def value_key(value: str) -> tuple[str, float | str]:
    """('C', farads), ('R', ohms) or ('?', the text). A voltage after a slash (100nF/50V) is dropped."""
    v = value.split("/")[0].strip()
    m = re.fullmatch(r"(\d+(?:\.\d+)?)\s*([pnuµ])\s*f?", v.lower())       # 100nF, 0.1uF, 75pF, 100n (not 2.2M!)
    if m:
        return "C", float(m.group(1)) * {"p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6}[m.group(2)]
    r = ohms(v)
    if r is not None:
        return "R", r
    return "?", value.strip()


def pretty(kind: str, x) -> str:
    if kind == "C":
        for unit, s in ((1e-6, "uF"), (1e-9, "nF"), (1e-12, "pF")):
            if x >= unit * 0.999:
                return f"{x / unit:.4g} {s}"
        return f"{x:.3g} F"
    if kind == "R":
        for unit, s in ((1e6, "M"), (1e3, "k"), (1.0, "")):
            if x >= unit * 0.999 or unit == 1.0:
                return f"{x / unit:.4g} {s}ohm"
    return str(x)


def board_lines(board: dict, side: str, feeder: dict) -> list[dict]:
    """The SMD parts on one side, grouped into lines (same value, same package). supported = the machine can
    place it now: a two-pad part that has no polarity (resistor, capacitor) and a known package. The others
    are placed by hand for now (polarity and parts with more pads need checks the camera can't do yet)."""
    fmap, pkgs = feeder.get("footprint_map", {}) or {}, feeder.get("packages", {}) or {}
    groups: dict[str, dict] = {}
    for p in board["parts"]:
        if p["side"] != side or not p["smd"] or p["dnp"]:
            continue
        kind, val = value_key(p["val"])
        package = fmap.get(p["fp"])
        key = f"{kind}:{val:.6g}:{package or p['fp']}" if kind != "?" else f"?:{val}:{package or p['fp']}"
        g = groups.setdefault(key, {"key": key, "kind": kind, "value": pretty(kind, val) if kind != "?" else val,
                                    "package": package or p["fp"], "footprints": set(), "values": set(),
                                    "refs": [], "pads": len(p["pads"])})
        g["footprints"].add(p["fp"])
        g["values"].add(p["val"])
        g["refs"].append(p["ref"])
    out = []
    for g in groups.values():
        pk = pkgs.get(g["package"], {})
        why = None
        if g["package"] not in pkgs:
            why = "package not known (config/feeder.yaml footprint_map)"
        elif pk.get("polarized"):
            why = "has a polarity: placed by hand for now"
        elif g["pads"] != 2:
            why = f"{g['pads']} pads: placed by hand for now"
        elif g["kind"] not in ("C", "R"):
            why = "not a resistor or capacitor"
        out.append({**g, "footprints": sorted(g["footprints"]), "values": sorted(g["values"]),
                    "refs": sorted(g["refs"], key=lambda r: (re.sub(r"\d", "", r), int(re.sub(r"\D", "", r) or 0))),
                    "count": len(g["refs"]), "supported": why is None, "why": why,
                    "label": f"{g['value']}  {g['package']}  x{len(g['refs'])}"})
    return sorted(out, key=lambda g: (not g["supported"], g["kind"], g["package"], g["value"]))


def load_feeder(path: Path = FEEDER_FILE, example: Path = FEEDER_EXAMPLE) -> dict:
    """config/feeder.yaml, or the example's packages and footprint map with no slot filled."""
    if path.exists():
        cfg = yaml.safe_load(path.read_text()) or {}
    else:
        cfg = yaml.safe_load(example.read_text()) or {}
        cfg["slots"] = {}
    cfg.setdefault("slots", {})
    cfg["slots"] = {int(k): v for k, v in (cfg["slots"] or {}).items()}
    cfg.setdefault("job", {"side": "B", "top_edge": "front"})
    return cfg


def save_feeder(cfg: dict, path: Path = FEEDER_FILE) -> None:
    path.write_text("# Written by the pick and place page (ui/app.py): what is in each slot (D11, D17), the board side\n"
                    "# for the job, and the package data (from feeder.example.yaml). Edit the packages here if needed.\n"
                    + yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))


# ---------------------------------------------------------------- the machine for the page

class PnPMachine(nc.NozzleCal):
    TITLE = "PICK AND PLACE"
    KEYS_HELP = ""

    def __init__(self, grbl, cam, markers, nominal, out_dir, board, bed) -> None:
        super().__init__(grbl, cam, markers, nominal, out_dir, board, bed)
        self.setup_pnp(bed)

    def setup_pnp(self, bed: dict) -> None:
        self.bed = bed
        self.events: deque = deque(maxlen=300)
        self.feeder = load_feeder(FEEDER_FILE, FEEDER_EXAMPLE)
        self.corners: dict[str, np.ndarray] = {}      # hole -> machine xy of the camera centred on it
        self.reg_info: dict = {}
        self.run_state = "idle"
        self.pause_flag = threading.Event()
        self.job_info: dict = {}
        self.job_log: list[dict] = []
        self.reach_mode = False
        self.saved_reach = None
        self.placed_refs: set[str] = set()
        self.click_mode = "move"                  # "land": the next click in the picture marks the nozzle's landing
        self.land: dict | None = None             # where the head was when the nozzle landed
        self.turn_deg = 0.0                       # nozzle turn from the servo centre (deg, + = anticlockwise)
        self.pick_mark: dict | None = None        # "the nozzle is ON a part here": Grbl's X / Y / Z, used after Home
        self.job_limit: int | None = None
        job = self.feeder.get("job", {})
        self.set_side(job.get("side", "B"), job.get("top_edge", "front"), save=False)

    # ---- messages for the page
    def say(self, text: str) -> None:
        super().say(text)
        self.events.append((datetime.datetime.now().strftime("%H:%M:%S"), str(text)))

    # ---- board side and the parts placed so far
    def set_side(self, side: str, top_edge: str, save: bool = True) -> None:
        if side not in ("F", "B") or top_edge not in ("front", "back"):
            raise ValueError("side F / B, top edge front / back")
        self.args.side, self.args.top_edge = side, top_edge
        self.A = d.orientation(side, top_edge)
        self.M = self.c = None
        self.corners = {}
        self.slot_view = None
        self.placed_refs = self.load_placed()
        if save:
            self.feeder["job"] = {"side": side, "top_edge": top_edge}
            save_feeder(self.feeder, FEEDER_FILE)
        if self.home is not None:
            self.apply_saved_registration()

    def load_placed(self) -> set[str]:
        try:
            st = yaml.safe_load(JOB_FILE.read_text()) or {}
        except OSError:
            return set()
        if st.get("side") != self.args.side:
            return set()
        return set(st.get("placed", []))

    def save_placed(self) -> None:
        JOB_FILE.write_text(yaml.safe_dump({"side": self.args.side, "placed": sorted(self.placed_refs),
                                            "time": datetime.datetime.now().isoformat(timespec="seconds"),
                                            "note": "parts already on this board: a job never places them again. "
                                                    "New board: the page's 'New board' button clears this"}))

    def job_new_board(self) -> None:
        self.placed_refs = set()
        self.save_placed()
        self.say("New board: no part is placed yet. The board is registered again (its holes)")
        self.M = self.c = None
        self.corners = {}

    # ---- feeders
    def lines(self) -> list[dict]:
        return board_lines(self.board, self.args.side, self.feeder)

    def assign(self, slot: int, key: str | None) -> None:
        slot = int(slot)
        if slot not in ml.slot_numbers(self.bed):
            raise ValueError(f"slot {slot} is not used on this bed")
        if not key:
            self.feeder["slots"].pop(slot, None)
        else:
            line = next((g for g in self.lines() if g["key"] == key), None)
            if line is None:
                raise ValueError("no such component line on this side")
            if not line["supported"]:
                raise ValueError(f"{line['label']}: {line['why']}")
            self.feeder["slots"][slot] = {"line": key, "value": line["value"], "footprint": line["package"],
                                          "qty": line["count"], "side": self.args.side}
        save_feeder(self.feeder, FEEDER_FILE)

    def assignments(self) -> list[tuple[int, dict]]:
        by_key = {g["key"]: g for g in self.lines()}
        out = []
        for slot, a in sorted(self.feeder["slots"].items()):
            g = by_key.get(a.get("line"))
            if g is not None and g["supported"] and a.get("side") == self.args.side:
                out.append((int(slot), g))
        return out

    # ---- moves the owner asks for on the page
    def by_hand_move(self, target: np.ndarray) -> None:
        self.by_hand = True                       # the owner watches: the jog clearance applies
        try:
            self.move_to(self.clip_to_reach(np.asarray(target, float)))
        finally:
            self.by_hand = False

    def job_goto_px(self, u: float, v: float) -> None:
        """Click in the picture: bring that spot to the red cross."""
        if self.J is None:
            self.say("The camera is not calibrated yet: press Home first")
            return
        if self.click_mode == "land" and self.land is not None:
            self.landing_clicked(u, v)
            return
        if self.click_mode == "pointer":
            self.pointer_clicked(u, v)
            return
        if self.z_locked or not self.nozzle_up(float(self.ncfg["jog_clearance_mm"])):
            self.say("Lift the nozzle first: no X / Y moves with the nozzle down")
            return
        self.by_hand_move(self.pos + np.linalg.solve(self.J, self.centre_px() - np.array([u, v], float)))

    def job_goto_xy(self, x: float, y: float) -> None:
        """Click on the bed map: the camera goes to that spot (mm from HOME)."""
        if self.home is None:
            self.say("Press Home first: the map is in mm from HOME")
            return
        if self.z_locked or not self.nozzle_up(float(self.ncfg["jog_clearance_mm"])):
            self.say("Lift the nozzle first: no X / Y moves with the nozzle down")
            return
        self.by_hand_move(self.home + np.array([x, y], float))

    # ---- the nozzle's turn (the 4th axis is the servo, D18)
    def servo_fit(self) -> dict | None:
        fit = (self.cal.get("servo") or {}).get("fit")
        return fit if fit and float(fit.get("worst_deg", 99)) <= 1.0 else None

    def job_turn(self, delta_deg: float) -> None:
        self.job_turn_to(self.turn_deg + float(delta_deg))

    def job_turn_to(self, deg: float) -> None:
        if self.no_servo:
            self.say("The servo is not wired (--no-servo)")
            return
        if not self.servo_build():
            return
        s, fit = self.scfg, self.servo_fit()
        centre = float(s["centre_us"])
        deg = float(np.clip(deg, -135.0, 135.0))
        pulse = centre + ((deg - float(fit["offset_deg"])) / float(fit["deg_per_us"]) if fit else deg / nc.NOMINAL_DEG_PER_US)
        pulse = float(np.clip(pulse, float(s["min_us"]), float(s["max_us"])))
        self.servo(pulse)
        self.turn_deg = deg
        self.say(f"Nozzle turned to {deg:+.0f} deg ({pulse:.0f} us)" + ("" if fit else ": nominal, the servo table (v) is "
                                                                         "not measured"))

    # ---- where the nozzle lands: land it, then click the spot in the picture
    def job_land(self, surface: str) -> None:
        """The nozzle comes down HERE (the head as it is now) to the pick height (surface 'part', on a part or soft
        tape: it leaves a mark) or to the place height ('place': with a part held, the pump lets it go there). Then
        the camera looks where it should have landed and the next click in the picture is the landing spot."""
        if surface not in ("part", "place"):
            raise ValueError("surface: part or place")
        h = self.touch_z(surface)
        if h is None:
            return
        if self.z_locked or not self.z_ok or not self.nozzle_up():
            self.say("Home first, and the nozzle up at the camera height")
            return
        if not self.no_servo and self.servo_build():
            self.servo(float(self.scfg["centre_us"]))
            self.turn_deg = 0.0
        head = self.pos.copy()
        target = h - float(self.ncfg["press_mm"]) if surface == "part" else h
        self.say(f"Landing: the nozzle goes down to Z {target:+.2f} here")
        self.z_from_above(target)
        time.sleep(float(self.ncfg["dwell_s"]))
        if self.pump_on:
            self.pump(False)                      # a part held: let it go on the board
            time.sleep(float(self.ncfg["release_s"]))
        self.z_to(0)
        self.land = {"surface": surface, "head": head}
        key = "offset_pick" if surface == "part" else "offset_place"
        guess = (self.cal.get(key) or {}).get("xy") or self.cal.get("offset_guess_mm") or self.ncfg.get("offset_guess_mm")
        if guess is not None:
            self.by_hand_move(head + np.array(guess, float))
        self.click_mode = "land"
        self.say("Now CLICK the centre of the mark (or of the part) in the picture: there the nozzle landed. "
                 "Not in view? Use the map or the arrows first, then click it")

    def job_pick_here(self) -> None:
        """The owner brought the nozzle tip down ON a part in a slot (the pick height, maybe holding it): keep the
        head's X / Y and Grbl's Z here, let the part go (it stays in its pocket). After Home (camera height
        checked) the pick height is saved from it, and one click on that part gives the PICK offset."""
        self.pick_mark = {"xy": self.pos.copy(), "grbl_z": float(self.grbl.status()[1]["Z"])}
        if self.pump_on:
            self.pump(False)
            time.sleep(float(self.ncfg["release_s"]))
        self.say("Pick spot kept (height and place). The part stays in its pocket. Now press Home: it lifts, finds "
                 "HOME and the camera height, then asks you to click that part")

    def apply_pick_mark(self) -> None:
        """After Home: the pick height from the kept Grbl Z, then wait for the click on the part (PICK offset)."""
        mark, self.pick_mark = self.pick_mark, None
        z = mark["grbl_z"] - self.z_base                       # mm from Z0, the remembered camera height
        gap = float(self.ncfg["move_clearance_mm"])
        touch = z + float(self.ncfg["press_mm"])               # the kept spot is the pick itself (pressed)
        if touch > -gap:
            self.say(f"The pick is only {-z:.1f} mm below the camera height: Z0 must be at least {gap:g} mm above it. "
                     "Set a higher camera height (0 twice), then do the pick spot again")
            return
        self.cal["touch_part"] = {"z": round(touch, 3), "ref": self.ref_id(), "by": "pick spot kept by the owner",
                                  "date": datetime.datetime.now().isoformat(timespec="seconds")}
        nc.save_cal(self.cal)
        self.land = {"surface": "part", "head": mark["xy"]}
        self.click_mode = "land"
        self.say(f"Pick height saved: Z {z:+.2f} mm from the camera height. Now CLICK the centre of THAT part in the "
                 "picture (bring it into view with the map or the arrows first, not by clicking the picture)")

    def job_land_cancel(self) -> None:
        self.click_mode, self.land = "move", None
        self.say("Landing calibration cancelled")

    def landing_clicked(self, u: float, v: float) -> None:
        point = self.pos + np.linalg.solve(self.J, self.centre_px() - np.array([u, v], float))
        off = point - self.land["head"]
        key, what = ("offset_pick", "PICK") if self.land["surface"] == "part" else ("offset_place", "PLACE")
        self.cal[key] = {"xy": np.round(off, 3).tolist(), "by": "landing + click", "ref": self.ref_id(),
                         "date": datetime.datetime.now().isoformat(timespec="seconds")}
        self.cal["offset_guess_mm"] = np.round(off, 1).tolist()
        place = self.cal.get("offset_place") or {}
        if key == "offset_pick" and place.get("ref") != self.ref_id():
            # no place offset at this Z0 yet: the same, for now. The tilted camera sees the board top (1.6 mm higher
            # than the parts in the tape) a little further from the camera: up to a few 0.1 mm. Check after a place
            self.cal["offset_place"] = {**self.cal[key], "by": "same as the pick offset (estimate)"}
        nc.save_cal(self.cal)
        self.click_mode, self.land = "move", None
        self.say(f"{what} offset saved: the nozzle lands at camera + X{off[0]:+.2f} Y{off[1]:+.2f} mm")

    def job_camera_height(self) -> None:
        """The page asked twice (a confirm box): this height is the camera height Z0."""
        self.new_ref_armed = True
        self.job_new_reference()

    # ---- reach: drive to the 4 ends by hand
    def job_reach_start(self) -> None:
        if self.home is None:
            self.say("Press Home first: the reach is measured from HOME")
            return
        self.saved_reach, self.reach = self.reach, None
        self.reach_stale, self.reach_mode, self.extremes = False, True, None
        self.say("REACH: no reach guard now. Jog slowly to the LEFT, RIGHT, FRONT and BACK ends, stop about 5 mm "
                 "before each end, then press Save reach")

    def job_reach_save(self) -> None:
        e = self.extremes
        if not self.reach_mode or e is None or e[1] - e[0] < 20 or e[3] - e[2] < 20:
            self.say("Reach NOT saved: drive to all 4 ends first (at least 20 mm each way)")
            return
        t.save_reach(e)
        self.reach = load_bench()["limits"]["reach_from_home"]
        self.reach_stale, self.reach_mode = False, False
        self.say(f"Reach saved: X {self.reach['x']} Y {self.reach['y']} mm from HOME")

    def job_reach_cancel(self) -> None:
        if self.reach_mode:
            self.reach, self.reach_mode = self.saved_reach, False
            self.reach_stale = bool(load_bench()["limits"].get("reach_stale", False))
            self.say("Reach measuring cancelled: the old reach is used again")

    # ---- board: 4 corner holes by hand, or found automatically
    def corner_target(self, hole: str) -> np.ndarray:
        q = self.A @ self.hole_xy[d.HOLES.index(hole)]
        if self.M is not None:
            return self.M @ q + self.c
        return q + self.nominal_T()

    def job_goto_corner(self, hole: str) -> None:
        if self.home is None:
            self.say("Press Home first")
            return
        if not self.nozzle_up(float(self.ncfg["jog_clearance_mm"])) or self.z_locked:
            self.say("Lift the nozzle first: no X / Y moves with the nozzle down")
            return
        self.by_hand_move(self.corner_target(hole))
        self.say(f"{hole}: the camera is where the hole should be. Click the hole's centre in the picture (or "
                 "Auto-centre), then Set")

    def job_centre_hole(self) -> None:
        if self.J is None or self.z_locked:
            self.say("Press Home first")
            return
        xy = self.centre_on("hole")
        self.say("Hole centred: press Set" if xy is not None else "No round hole near the red cross: click its centre")

    def job_set_corner(self, hole: str) -> None:
        if self.home is None or not self.z_ok:
            self.say("Home and the camera height first (Setup steps 1-2): the corners are measured at the camera height")
            return
        self.corners[hole] = self.pos.copy()
        rel = self.pos - self.home
        self.say(f"{hole} set at X{rel[0]:+.2f} Y{rel[1]:+.2f} from HOME ({len(self.corners)} of 4)")

    def job_fit_board(self) -> None:
        holes = [h for h in d.HOLES if h in self.corners]
        if len(holes) < 3:
            self.say("Set at least 3 corner holes (4 is better: the 4th checks the others)")
            return
        idx = [d.HOLES.index(h) for h in holes]
        q = (self.A @ self.hole_xy[idx].T).T
        P = np.array([self.corners[h] for h in holes])
        M, c = d.fit_affine(q, P)
        f = d.describe(M)
        res = np.hypot(*(q @ M.T + c - P).T)
        if not (0.96 < f["scale_x"] < 1.04 and 0.96 < f["scale_y"] < 1.04 and abs(f["skew_deg"]) < 2):
            self.say(f"The corners don't fit the board's holes (scale {f['scale_x']:.3f} / {f['scale_y']:.3f}, skew "
                     f"{f['skew_deg']:+.1f} deg): wrong hole, wrong side or top edge? Set them again")
            return
        self.M, self.c = M, c
        self.reg_info = {"holes": holes, "residuals_mm": np.round(res, 3).tolist(),
                         "angle_deg": round(f["angle_deg"], 2), "scale": [round(f["scale_x"], 4), round(f["scale_y"], 4)],
                         "by": "corners set by hand"}
        self.save_registration()
        self.overlay = [(self._board_outline(), (0, 255, 0), "")]
        self.say(f"Board registered on {len(holes)} holes: worst {res.max():.2f} mm, angle {f['angle_deg']:+.2f} deg"
                 + ("" if len(holes) == 4 else " (3 holes fit exactly: no check. Set the 4th to be sure)"))

    def job_auto_board(self) -> None:
        if self.home is None or self.J is None:
            self.say("Press Home first")
            return
        if self.z_locked or not self.nozzle_up():
            self.say("Lift the nozzle first")
            return
        self.todo = []
        if self.register():
            holes = self.report["registration"]["holes_mm_from_home"]
            self.corners = {h: np.array(v, float) + self.home for h, v in holes.items()}
            self.reg_info = {"holes": list(holes), "residuals_mm": self.report["registration"]["residuals_mm"],
                             "angle_deg": self.report["registration"]["board_angle_deg"],
                             "scale": [self.report["registration"]["scale_x"], self.report["registration"]["scale_y"]],
                             "by": "holes found by the camera"}
            self.save_registration()

    def save_registration(self) -> None:
        REG_FILE.write_text(yaml.safe_dump({
            "side": self.args.side, "top_edge": self.args.top_edge, "camera_height": self.ref_id(),
            "M": np.round(self.M, 6).tolist(), "c_from_home": np.round(self.c - self.home, 4).tolist(),
            "corners_from_home": {h: np.round(p - self.home, 4).tolist() for h, p in self.corners.items()},
            **self.reg_info, "time": datetime.datetime.now().isoformat(timespec="seconds"),
            "note": "machine = M @ (side/top-edge turn @ KiCad xy) + c. Valid while the board stays put"},
            sort_keys=False))

    def apply_saved_registration(self) -> bool:
        """The board registered before (same side, top edge and camera height): use it again after Home."""
        try:
            reg = yaml.safe_load(REG_FILE.read_text()) or {}
        except OSError:
            return False
        if reg.get("side") != self.args.side or reg.get("top_edge") != self.args.top_edge \
                or reg.get("camera_height") != self.ref_id() or self.home is None:
            return False
        self.M = np.array(reg["M"], float)
        self.c = np.array(reg["c_from_home"], float) + self.home
        self.corners = {h: np.array(p, float) + self.home for h, p in (reg.get("corners_from_home") or {}).items()}
        self.reg_info = {k: reg[k] for k in ("holes", "residuals_mm", "angle_deg", "scale", "by") if k in reg}
        self.say("Board registration of before is used again (same side and camera height). Board moved? Set the "
                 "corners again")
        return True

    # ---- a camera that does NOT move with Z (2026-10-08: the label size stays the same when Z moves)
    def camera_moves_with_z(self) -> bool:
        return bool(load_bench()["nozzle"].get("camera_moves_with_z", True))

    def camera_height_still_right(self) -> bool:
        if not self.camera_moves_with_z():
            self.checked_for_r = True                 # nothing to check: the camera's height never changes
            return True
        return super().camera_height_still_right()

    def picture_shift_px(self, a: np.ndarray, b: np.ndarray) -> float:
        """How far the picture moved between two frames (px), on its upper half: the nozzle's shadow at the bottom
        grows when the nozzle is down (2026-10-11: 154 px read on the upper 3/4 for a camera that does not move)."""
        h = a.shape[0] // 2
        fa, fb = (np.float32(cv2.cvtColor(x, cv2.COLOR_BGR2GRAY) if x.ndim == 3 else x)[:h] for x in (a, b))
        (dx, dy), _ = cv2.phaseCorrelate(fa, fb)
        return float(np.hypot(dx, dy))

    def start_from_pick_mark(self) -> bool:
        """The nozzle tip is ON a part in a slot (the pick spot). The camera doesn't move with Z, so Z can't be
        checked by the label size: the pick spot is the Z reference, kept in Grbl's own step count. Z0 = the
        moving height = nozzle.place_above_pick_mm + nozzle.move_clearance_mm above the pick spot. The pick, place
        and moving heights are set from it; the part stays in its pocket (pump off). Parts in the 3D printed trays
        sit higher than the board: bed.yaml feed_tray.tape_floor_mm is taken off the place height."""
        mark = self.pick_mark
        nz = load_bench()["nozzle"]
        floor = float((self.bed.get("feed_tray") or {}).get("tape_floor_mm", 0.0))
        gap, press = float(nz["move_clearance_mm"]), float(nz["press_mm"])
        place_up = float(nz["place_above_pick_mm"]) - floor
        up = max(0.0, place_up) + gap
        self.z_locked = None                          # the owner put the tip on the part: Z is known
        if hasattr(self.grbl, "reset_seen"):
            self.grbl.reset_seen = False
        self.pump(False)
        before = self.frame()
        self.z_base = float(mark["grbl_z"])           # the pick spot = 0 while lifting
        self.z = self.z_now()
        self.say(f"Lifting {up - self.z:.1f} mm from the pick spot to the moving height (3 mm above the board)")
        self.z_to(up)
        shift = self.picture_shift_px(before, self.frame())
        if shift > 8.0:                               # a camera on the Z slide moves tens of px for 4.9 mm
            self.z_ok = False
            self.say(f"The picture moved {shift:.0f} px when Z went up: the camera DOES move with Z. Set config/bench.yaml "
                     "nozzle.camera_moves_with_z: true, then press Home (the camera height check)")
            return False
        self.z_base += up                             # Z0 = here
        self.z, self.z_ok = 0.0, True
        old = self.cal.get("z_reference") or {}
        if mark.get("same_as_ref") and old.get("camera_moves_with_z") is False and old.get("id"):
            self.say("Same pick spot as the remembered heights: the offsets and the board registration stay valid")
            self.land, self.click_mode, self.pick_mark = None, "move", None
            return True
        ref = datetime.datetime.now().isoformat(timespec="seconds")
        self.cal["z_reference"] = {"label": None, "px_mm": None, "f_mm": None, "id": ref, **self.ref_tags(),
                                   "camera_moves_with_z": False,
                                   "note": f"Z0 = the moving height, {up:g} mm above the pick spot the owner set "
                                           "(the camera does not move with Z)"}
        stamp = {"ref": ref, "by": "pick spot (nozzle tip on a part)", "date": ref}
        self.cal["touch_part"] = {"z": round(-up + press, 3), **stamp}   # the pick goes press_mm below: the spot
        self.cal["touch_place"] = {"z": round(-up + place_up, 3), **stamp,
                                   "note": "pick spot + nozzle.place_above_pick_mm - feed_tray.tape_floor_mm"}
        self.cal["touch_move"] = {"z": 0.0, **stamp}
        nc.save_cal(self.cal)
        self.land = {"surface": "part", "head": np.asarray(mark["xy"], float)}
        self.click_mode = "land"
        self.pick_mark = None
        self.say(f"Heights from the pick spot: pick {-up:+.2f}, place {-up + place_up:+.2f}, moving 0 (Z0). The picture "
                 f"did not move ({shift:.1f} px): the camera stays put when Z moves")
        return True

    def find_home(self) -> bool:
        """The camera high on the head (2026-10-08, a wide view): calibrate on the label nearest the picture centre
        (small moves to the left and the front only) and measure HOME in the picture: no drive to centre a label,
        which could go past the back end (the reach behind the bed survey's start is not known)."""
        if self.camera_moves_with_z():
            return super().find_home()
        self.stage("1/4 calibrate the camera and measure HOME")
        t.CAL_MOVE = float(load_bench()["camera_check"].get("cal_move_mm", t.CAL_MOVE))
        seen = self.visible() or self.search_for_labels()
        if not seen:
            self.say("No label in view: jog until HOME or a slot label is in the picture, then Home")
            return False
        ref = self.calibration_ref(seen)
        if ref is None:
            self.say("No label stays in view for the calibration moves (8 mm left and front): jog so that a label is "
                     "in the upper half of the picture, then Home")
            return False
        if not self._calibrate_once(ref):
            self.say("The strict camera check failed: a rough calibration is used")
            if not self._calibrate_once(ref, strict=False):
                return False
        seen = self.visible()
        if "home" not in seen:                        # HOME from a surveyed label in view, then go and look at it
            known = [n for n in seen if n in self.nominal]
            if not known:
                self.say("HOME is not in the picture: jog until it is (it sits behind slot 3's marker row), then Home")
                return False
            n = known[0]
            at = self.pos + np.linalg.solve(self.J, self.centre_px() - seen[n].mean(axis=0))
            home_guess = at - (self.nominal[n] - self.nominal["home"])
            view = np.array(load_bench()["nozzle"].get("home_view_mm", [-24.0, -23.0]), float)
            self.say(f"HOME is not in view: going near it (from {n.upper().replace('_', ' ')})")
            self.move_checked(home_guess + view - self.pos)
            if "home" not in self.visible():
                self.say("HOME is still not in the picture: jog until it is, then Home")
                return False
        self.job_home()
        return self.home is not None

    def calibration_ref(self, seen: dict) -> str | None:
        """The label that stays best in view during the calibration moves (CAL_MOVE to the left, then to the
        front), predicted with the last calibration: the most room to the picture edges. The bottom of the picture
        (nozzle.shadow_bottom_frac: the nozzle's shadow) counts as an edge."""
        J = self.J if self.J is not None else t.saved_camera_J()
        h, w = self.frame_shape[:2]
        bottom = h * (1.0 - float(load_bench()["nozzle"].get("shadow_bottom_frac", 0.0)))
        best, room = None, 4.0
        for name, c in seen.items():
            worst = np.inf
            for d in ([0.0, 0.0], [-t.CAL_MOVE, 0.0], [0.0, -t.CAL_MOVE]):
                p = c + (np.zeros(2) if J is None else J @ np.array(d))
                # the nozzle's shadow is at the bottom MIDDLE only (2026-10-11: HOME at the left was refused)
                in_shadow = p[:, 0].max() > 0.3 * w and p[:, 0].min() < 0.66 * w
                low = bottom if in_shadow else h - 1
                worst = min(worst, p[:, 0].min(), w - 1 - p[:, 0].max(), p[:, 1].min(), low - p[:, 1].max())
            if worst > room:
                best, room = name, worst
        return best

    def job_start_fixed_camera(self) -> None:
        """Home when the camera does not move with Z: Z0 comes from the pick spot (above). Home only goes back up to
        Z0 (Grbl's count) and finds HOME again. Z not known (unclean end, Grbl reset): the pick spot again."""
        if self.pick_mark is not None:
            if not self.start_from_pick_mark():
                return
        elif self.z_locked or getattr(self.grbl, "reset_seen", False) or not self.z_ok:
            self.z_ok = False
            self.z_locked = None                      # the owner pressed unlock: the nozzle is clear; Z is set next
            if hasattr(self.grbl, "reset_seen"):
                self.grbl.reset_seen = False
            self.say("Z is not known: jog the nozzle tip down ONTO the first part in slot 3 (Z down 1 / 0.1), press "
                     "'Use as PICK' (Setup 3), then Home")
            return
        else:
            self.z_to(0)
        self.M = self.c = None
        self.slot_view, self.nozzle_target, self.checked_for_r = None, None, True
        if not self.find_home():
            return
        self.apply_saved_registration()
        self.say("Ready." + (" Now the PICK offset: CLICK the centre of the part the nozzle was on (its pocket)"
                             if self.click_mode == "land" else ""))

    def servo(self, pulse: float) -> None:
        """Every servo angle is reached from the same side (from a shorter pulse): the MG995's gear play then
        always sits the same way (2026-10-11: the same pulse stopped ~20 deg apart from the two sides)."""
        a = float(self.scfg.get("approach_us", 0) or 0)
        low = max(float(self.scfg["min_us"]), float(pulse) - a)
        if a > 0 and not self.no_servo and low < float(pulse):
            super().servo(low)
        super().servo(pulse)

    def job_jog_z(self, dz: float) -> None:
        if getattr(self, "z_at_top", False):
            if dz > 0:
                self.say("Z is at its TOP end: it can't go higher")
                return
            self.z_at_top = False
        super().job_jog_z(dz)

    def job_start(self) -> None:
        if not self.camera_moves_with_z():
            self.job_start_fixed_camera()
            return
        if getattr(self, "z_at_top", False):          # the owner put Z at its top end: never lift into it
            self.lifted_at = float(self.grbl.status()[1]["Z"])
        super().job_start()
        if self.home is not None and self.z_ok:
            self.apply_saved_registration()
            if self.pick_mark is not None:
                self.apply_pick_mark()

    # ---- bed survey: every label from wide pictures (camera high), Z not moved
    def survey_record(self, sightings: dict) -> dict[str, np.ndarray]:
        """One fresh picture: every label in it, measured from where the head is (mm from HOME)."""
        found = self.visible()
        h, w = self.frame_shape[:2]
        for name, c in found.items():
            if c.min() < SURVEY_EDGE_PX or c[:, 0].max() > w - 1 - SURVEY_EDGE_PX or c[:, 1].max() > h - 1 - SURVEY_EDGE_PX:
                continue                              # may be cut off by the picture edge
            px = c.mean(axis=0)
            xy = self.pos + np.linalg.solve(self.J, self.centre_px() - px) - self.home
            sightings.setdefault(name, []).append((float(np.hypot(*(px - self.centre_px()))), xy))
        return found

    def survey_leg(self, direction, dist: float, sightings: dict, stop=None, slow_from: float | None = None) -> bool:
        """Drive dist mm along direction in short steps (1 mm steps after slow_from mm). After every step each
        label seen before and after must have moved as the calibration says, else the head is blocked (an end).
        stop(labels in view) True = stop here and return True."""
        direction = np.asarray(direction, float)
        done, before = 0.0, self.survey_record(sightings)
        while done < dist - 1e-6:
            if self.stop_flag.is_set():
                raise t.Stopped()
            step = SURVEY_SLOW_STEP_MM if slow_from is not None and done >= slow_from else SURVEY_STEP_MM
            step = min(step, dist - done)
            self.move(direction * step)
            done += step
            after = self.survey_record(sightings)
            expected = self.J @ (direction * step)
            for name in set(before) & set(after):
                seen = after[name].mean(axis=0) - before[name].mean(axis=0)
                if seen @ expected < 0.5 * (expected @ expected):
                    raise t.Stalled(f"{name} moved only {np.hypot(*seen) / np.hypot(*expected) * 100:.0f} % of the "
                                    "step in the picture (an end of travel?)")
            if stop is not None and stop(after):
                return True
            before = after
        return False

    def view_half_mm(self) -> np.ndarray:
        """How far the picture reaches from its centre: [left, right, front, back] in mm (negative = left / front)."""
        h, w = self.frame_shape[:2]
        offs = np.array([np.linalg.solve(self.J, self.centre_px() - np.array(p, float))
                         for p in ((0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1))])
        return np.array([offs[:, 0].min(), offs[:, 0].max(), offs[:, 1].min(), offs[:, 1].max()])

    def job_bed_survey(self) -> None:
        """Where every bed label really is (HOME, the slots, PCB 1-4), from wide pictures: for a camera high above
        the bed (Z at its top); Z is not moved. Start with HOME in the picture. The head goes only to the FRONT and
        to the LEFT of where it starts (the start is near the back and right ends), in short camera-checked steps,
        and to the front only until PCB 1 or PCB 2 comes into view: the front end is ~3 mm further (owner,
        2026-10-08). Saves the label positions and the PCB area (between the PCB labels) into config/bed.yaml
        and the reach driven here into config/bench.yaml."""
        if self.z_locked:
            self.say(f"LOCKED ({self.z_locked}): unlock first")
            return
        if self.reach_mode or self.run_state != "idle":
            self.say("Finish the reach measuring or the job first")
            return
        if not self.nozzle_up():
            self.say("Lift the nozzle first: the survey moves at this height")
            return
        seen = self.visible()
        if "home" not in seen:
            self.say("Bed survey: HOME must be in the picture to start. Jog until it is (Z high = a wide view)")
            return
        self.stage("Bed survey 1/4: calibrating the camera here (8 mm moves to the left and the front)")
        ref = min(seen, key=lambda n: np.hypot(*(seen[n].mean(axis=0) - self.centre_px())))
        if not self._calibrate_once(ref):
            self.say("The strict camera check failed: a rough calibration is used for the survey")
            if not self._calibrate_once(ref, strict=False):
                return
        c = self.look("home", frames=6)
        if c is None:
            self.say("HOME left the picture: jog until it is in view, then survey again")
            return
        self.home = self.pos + np.linalg.solve(self.J, self.centre_px() - c.mean(axis=0))
        self.extremes, self.M, self.c, self.corners = None, None, None, {}
        start, view = self.pos.copy(), self.view_half_mm()
        plan = {n: np.asarray(xy, float) - self.nominal["home"] for n, xy in self.nominal.items()}
        sightings: dict[str, list] = {}
        saved_reach, self.reach = self.reach, None    # the old reach is from before the camera moved: not used
        feed, self.feed = self.feed, min(self.feed, SURVEY_FEED)
        try:
            fronts = [plan[n][1] for n in ("pcb_1", "pcb_2") if n in plan]
            slow = None
            if fronts:                                # 1 mm steps from 60 mm before the plan says they show up
                y_at_edge = max(fronts) + 7.5 - view[2]
                slow = max(0.0, (start[1] - self.home[1]) - y_at_edge - SURVEY_SLOW_MM)
            self.stage("Bed survey 2/4: to the FRONT until PCB 1 or PCB 2 is in view")
            hit = self.survey_leg((0, -1), SURVEY_MAX_FRONT_MM, sightings,
                                  stop=lambda f: bool({"pcb_1", "pcb_2"} & set(f)), slow_from=slow)
            if not hit:
                self.say(f"PCB 1 / PCB 2 not seen within {SURVEY_MAX_FRONT_MM:g} mm to the front")
            lefts = [plan[n][0] for n in plan if n.startswith("slot_") or n in ("pcb_1", "pcb_4")]
            left_x = max(min(lefts) - 0.5 * view[0], SURVEY_LEFT_MIN_MM) if lefts else SURVEY_LEFT_MIN_MM
            self.stage("Bed survey 3/4: to the LEFT along the front, then back")
            self.survey_leg((-1, 0), (self.pos[0] - self.home[0]) - left_x, sightings)
            self.survey_leg((0, 1), start[1] - self.pos[1], sightings)
            self.stage("Bed survey 4/4: to the RIGHT, back to the start, HOME drift check")
            self.survey_leg((1, 0), start[0] - self.pos[0], sightings)
        finally:
            self.feed = feed
            self.reach = saved_reach
        c = self.look("home", frames=6)
        drift = None if c is None else self.pos + np.linalg.solve(self.J, self.centre_px() - c.mean(axis=0)) - self.home
        self.save_survey(best_sightings(sightings), plan, drift)

    def save_survey(self, best: dict, plan: dict, drift) -> None:
        stamp = datetime.datetime.now().isoformat(timespec="seconds")
        labels = [{"label": n, "plan_mm": None if n not in plan else np.round(plan[n], 1).tolist(),
                   "measured_mm": None if n not in best else np.round(best[n]["xy"], 2).tolist(),
                   **({} if n not in best else {"px_from_centre": round(best[n]["centre_px"]), "views": best[n]["views"],
                                                "spread_mm": round(best[n]["spread_mm"], 2)})}
                  for n in sorted(set(plan) | set(best))]
        missing = [n for n in plan if n not in best]
        slots = {int(n.split("_")[1]): best[n]["xy"] for n in best if n.startswith("slot_")}
        pcb = {n: best[n]["xy"] for n in best if n.startswith("pcb_")}
        estimated = None
        if len(pcb) == 3:                             # one front label may stay out of view (the front end is close):
            (i,) = [i for i in range(1, 5) if f"pcb_{i}" not in pcb]      # it is the 4th corner of the rectangle
            a, b, c = (pcb[f"pcb_{(i + k - 1) % 4 + 1}"] for k in (1, 2, 3))
            estimated, pcb[f"pcb_{i}"] = f"pcb_{i}", a + c - b
        area, info = (None, {})
        if len(pcb) == 4:
            area, info = pcb_area_from_labels(pcb, self.bed["pcb_area"]["size_mm"], float(self.markers.size_mm),
                                              float(self.bed["markers"].get("quiet_mm", 2.5)),
                                              self.bed["pcb_area"].get("corner_holders"))
        e = self.extremes
        data = {"date": stamp, "note": "marker centres in mm from HOME (+X right, +Y back), camera high, wide view",
                "camera_px_per_mm_matrix": np.round(self.J, 3).tolist(),
                "home_drift_after_survey_mm": None if drift is None else np.round(drift, 3).tolist(),
                "driven_mm_from_home": None if e is None else {"x": [round(float(e[0]), 1), round(float(e[1]), 1)],
                                                               "y": [round(float(e[2]), 1), round(float(e[3]), 1)]},
                "pcb_area_xy": area, "pcb_labels": info, "missing": missing, "labels": labels,
                "pcb_estimated_from_the_other_3": estimated}
        (self.out_dir / "survey.yaml").write_text(yaml.safe_dump(data, sort_keys=False))
        if drift is None or np.hypot(*drift) > 0.5:
            self.say("Survey NOT saved to the config: " + ("HOME was not seen at the end" if drift is None else
                     f"HOME moved {np.hypot(*drift):.2f} mm during the trip (lost steps?)") +
                     f". Data: {self.out_dir.name}/survey.yaml")
            return
        BED_FILE.write_text(layout_text(BED_FILE.read_text(), slots, pcb if len(pcb) == 4 else None, area, stamp[:10]))
        self.bed = ml.load_bed(BED_FILE)
        for n, b in best.items():
            self.nominal[n] = b["xy"] + self.nominal["home"]
        if e is not None:
            t.save_reach(e)
            self.reach = load_bench()["limits"]["reach_from_home"]
            self.reach_stale = False
        self.say(f"Bed survey saved: {len(best)} labels" + (f" (NOT seen: {', '.join(missing)})" if missing else "")
                 + (f"; {estimated.upper().replace('_', ' ')} completed from the other 3 PCB labels" if estimated else "")
                 + (f". PCB area from the PCB labels: front-left corner X{area[0]:+.1f} Y{area[1]:+.1f}; label box "
                    f"{info['marker_box_mm'][0]} x {info['marker_box_mm'][1]} mm, board edge to label {info['gap_x_mm']} "
                    f"/ {info['gap_y_mm']} mm (X / Y)" if area else "")
                 + f". HOME drift {np.hypot(*drift):.2f} mm. Reach (camera centre, at THIS camera height): "
                 f"X {self.reach['x']} Y {self.reach['y']}")

    # ---- nozzle calibration by picking: try offsets on a real part until it is picked, then find the middle
    def part_near(self, xy, radius: float = 2.5) -> np.ndarray | None:
        """Bring the camera onto the part (dark top) nearest xy (machine mm) and measure it there. None = none."""
        xy = np.asarray(xy, float)
        for _ in range(3):
            self.move_to(self.clip_to_reach(xy))
            spots = [s for s in d.dark_spots(self.frame(), self.J, self.pos, self.centre_px(), self.parts_cfg)
                     if np.hypot(*(s[0] - xy)) < radius and 1.0 <= s[1] <= 6.0]
            if not spots:
                return None
            xy = min(spots, key=lambda s: np.hypot(*(s[0] - xy)))[0]
            if np.hypot(*(xy - self.pos)) < 0.05:
                break
        return xy

    def try_pick(self, part_xy: np.ndarray, off: np.ndarray, pick_z: float) -> str:
        """Pick the part with this offset, look into the pocket, and put a picked part back. 'picked', 'missed'
        (still in its pocket) or 'lost' (gone, and not back after the put-back)."""
        head = part_xy - off
        self.z_to(0)
        self.move_to(head)
        self.pump(True)
        self.z_from_above(pick_z)
        time.sleep(float(self.ncfg["dwell_s"]))
        self.z_to(0)
        self.move_to(self.clip_to_reach(part_xy))
        tol = float(self.ncfg["after_check_mm"])
        left = [s for s in d.dark_spots(self.frame(), self.J, self.pos, self.centre_px(), self.parts_cfg)
                if np.hypot(*(s[0] - part_xy)) < tol]
        if left:
            self.pump(False)
            return "missed"
        self.move_to(head)                            # put it back into its pocket (no valve: wait for the vacuum)
        self.z_from_above(pick_z)
        self.pump(False)
        time.sleep(max(2.5, float(self.ncfg["release_s"])))
        self.z_to(0)
        return "picked" if self.part_near(part_xy, radius=tol + 1.0) is not None else "lost"

    def job_nozzle_by_picking(self) -> None:
        """The nozzle misses the parts (2026-10-08: by about 1 mm): find the PICK offset by picking a real part in
        the pick slot. Offsets around the present one are tried (rings of 0.5 / 1.0 / 1.5 mm) until the part is
        picked, then along X and along Y in 0.3 mm steps to both ends of the offsets that pick: the middle is the
        new offset. Every picked part goes back into its pocket. The PLACE offset moves by the same amount."""
        if self.home is None or not self.z_ok or self.J is None:
            self.say("Home first")
            return
        pick_touch, off_e = self.touch_z("part"), self.cal.get("offset_pick")
        if pick_touch is None or not off_e or off_e.get("ref") != self.ref_id():
            self.say("Set the pick height and a first PICK offset first")
            return
        pick_z = pick_touch - float(self.ncfg["press_mm"])
        off0 = np.array(off_e["xy"], float)
        if getattr(self, "pick_hint", None) is not None:
            off0 = np.array(self.pick_hint, float)    # an offset that picked before: start there
        slot = f"slot_{int(self.ncfg.get('pick_slot', 3))}"
        marker = self.home + self.nominal[slot] - self.nominal["home"]

        def next_part():
            """The part nearest the marker in the slot's tape mark (the strip may have moved): None = empty."""
            for dy in (-30.0, -55.0):
                self.move_to(self.clip_to_reach(marker + np.array([0.0, dy])))
                spots = [s for s in d.dark_spots(self.frame(), self.J, self.pos, self.centre_px(), self.parts_cfg)
                         if 1.0 <= s[1] <= 6.0 and abs(s[0][0] - marker[0]) < 8 and -80 < s[0][1] - marker[1] < -12]
                if spots:
                    return self.part_near(max(spots, key=lambda s: s[0][1])[0])
            return None

        self.stage("Nozzle by picking: looking for a part in the pick slot")
        part = next_part()
        if part is None:
            self.say("No part seen in the pick slot")
            return
        tries = []

        def attempt(off):
            nonlocal part
            if part is None:
                raise RuntimeError("the part is gone: put parts in the pick slot and start again")
            got = self.try_pick(part, off, pick_z)
            tries.append({"offset": np.round(off, 2).tolist(), "result": got})
            self.say(f"   offset X{off[0]:+.2f} Y{off[1]:+.2f}: {got}")
            if got == "lost":                         # picked, but not back in its pocket: take the next part
                part = next_part()
            else:
                part = self.part_near(part) or next_part()
            return got != "missed"

        try:
            ring = [np.zeros(2)] + [r * np.array([np.cos(a), np.sin(a)]) for r, n in ((0.5, 8), (1.0, 8), (1.5, 12))
                                    for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]
            ok = next((off0 + dv for dv in ring if attempt(off0 + dv)), None)
            if ok is None:
                self.say("No offset within 1.5 mm picks the part: check the pick height and the vacuum")
                return
            mid = ok.copy()
            for axis in (0, 1):
                ends = []
                for sign in (1, -1):
                    k = 0
                    while k < 7:
                        trial = mid.copy()
                        trial[axis] += sign * 0.3 * (k + 1)
                        if not attempt(trial):
                            break
                        k += 1
                    ends.append(mid[axis] + sign * 0.3 * k)
                mid[axis] = (ends[0] + ends[1]) / 2
                self.say(f"   {'XY'[axis]}: picks from {min(ends):+.2f} to {max(ends):+.2f}: middle {mid[axis]:+.2f}")
        finally:
            self.pump(False)
            self.z_to(0)
            (self.out_dir / "nozzle_by_picking.yaml").write_text(yaml.safe_dump({"tries": tries}, sort_keys=False))
        delta = mid - off0
        now = datetime.datetime.now().isoformat(timespec="seconds")
        self.cal["offset_pick"] = {"xy": np.round(mid, 3).tolist(), "by": "pick search", "ref": self.ref_id(), "date": now}
        place = self.cal.get("offset_place") or {}
        if place.get("ref") == self.ref_id():
            self.cal["offset_place"] = {**place, "xy": np.round(np.array(place["xy"], float) + delta, 3).tolist(),
                                        "by": f"{place.get('by', '')}; moved like the pick offset (pick search)",
                                        "date": now}
        nc.save_cal(self.cal)
        self.say(f"Nozzle calibrated by picking: offset moved X{delta[0]:+.2f} Y{delta[1]:+.2f} mm "
                 f"(now X{mid[0]:+.2f} Y{mid[1]:+.2f}); {len(tries)} picks")

    # ---- nozzle marks with an ink spot: tip position at each servo angle, servo axis, runout
    def job_nozzle_marks(self) -> None:
        """The needle is dipped into the ink spot, the servo turns, the needle presses a mark on clean paper, and the
        camera finds the mark: the tip's offset from the camera at each servo pulse (nozzle.mark_pulses_us; the
        centre twice = repeat check), the servo axis and the runout. The ink spot and the paper lie flat on the bed
        (same touch height: Setup 3 Paper). The PICK and PLACE offsets become the centre pulse's offset."""
        if self.home is None or not self.z_ok or self.J is None:
            self.say("Home first")
            return
        ink, paper, touch = self.spot("ink", "Ink spot here"), self.spot("paper", "Paper here"), self.touch_z("paper")
        if ink is None or paper is None or touch is None:
            return
        if not (self.no_servo or self.servo_build()):
            return
        cfg = self.ncfg
        od, press, spacing = float(cfg["od_mm"]), float(cfg["press_mm"]), float(cfg["spacing_mm"])
        dark, centre = float(cfg["mark_dark"]), float(self.scfg["centre_us"])
        big = float(cfg.get("mark_max_area", 4.0))     # ink spreads on paper: dots up to this x the needle's area
        pulses = [float(p) for p in cfg["mark_pulses_us"]] if not self.no_servo else [centre] * 3
        guess = self.cal.get("offset_pick") or {}
        o_est = np.array(guess.get("xy") or self.cal.get("offset_guess_mm") or cfg["offset_guess_mm"], float)
        spots = [paper + nc.grid(k, spacing) for k in range(len(pulses))]
        for s in spots + [ink]:
            if not self.in_reach(s - o_est - self.home):
                self.say("The nozzle cannot reach the paper or the ink spot from here: put them nearer slot 3")
                return
        self.z_to(0)
        self.stage("MARKS: is the paper clean?")
        self.move_to(self.clip_to_reach(np.mean(spots, axis=0)))
        img = self.frame()
        if any(nc.find_mark(img, self.J, self.pos, self.centre_px(), s, 2.0, od, dark, big) is not None for s in spots):
            self.say("There are marks on the paper already: use a clean piece (or click 'Paper here' on a clean place)")
            return
        found = []
        try:
            for k, (pulse, s) in enumerate(zip(pulses, spots)):
                self.stage(f"MARKS [{k + 1}/{len(pulses)}] servo {pulse:.0f} us: ink, then mark")
                if not self.no_servo:
                    self.servo(pulse)
                dip = ink + np.array([(k % 3 - 1) * 1.5, -(k // 3) * 1.5])     # a fresh place on the ink each time
                self.move_to(dip - o_est)
                self.z_from_above(touch - press)
                time.sleep(float(cfg["dwell_s"]))
                self.z_to(0)
                head = s - o_est
                self.move_to(head)
                self.z_from_above(touch - press)
                time.sleep(float(cfg["dwell_s"]))
                self.z_to(0)
                self.move_to(self.clip_to_reach(s))
                img = self.frame()
                m = nc.find_mark(img, self.J, self.pos, self.centre_px(), s, 15.0 if not found else 3.0, od, dark, big)
                if m is None:
                    cv2.imwrite(str(self.out_dir / f"mark_{k + 1}_missing.png"), img)
                    self.say(f"No mark found at spot {k + 1}: more ink? Or the paper height (Setup 3 Paper) is too "
                             f"high. Picture: mark_{k + 1}_missing.png")
                    return
                o_est = m - head
                found.append((pulse, o_est))
                self.overlay = [(self._circle(m, od / 2), (0, 255, 0), f"{pulse:.0f} us")]
                self.say(f"   {pulse:.0f} us: nozzle tip = camera + X{o_est[0]:+.3f} Y{o_est[1]:+.3f} mm")
        finally:
            self.z_to(0)
            if not self.no_servo:
                self.servo(centre)
        by: dict[float, list] = {}
        for p, o in found:
            by.setdefault(p, []).append(o)
        repeat = max((float(np.hypot(*(v[0] - v[-1]))) for v in by.values() if len(v) > 1), default=None)
        by_pulse = {int(p): np.mean(v, axis=0) for p, v in by.items()}
        pts = np.array(list(by_pulse.values()))
        if len(by_pulse) >= 3 and np.ptp(pts, axis=0).max() > 0.05:
            axis, runout = nc.fit_circle(list(by_pulse.values()))
        else:
            axis, runout = pts.mean(axis=0), 0.0
        now = datetime.datetime.now().isoformat(timespec="seconds")
        self.cal["offset"] = {
            "by_pulse_us": {p: np.round(o, 3).tolist() for p, o in sorted(by_pulse.items())},
            "shaft_xy": np.round(axis, 3).tolist(), "runout_mm": round(float(runout), 3),
            "repeat_mm": None if repeat is None else round(repeat, 3), "ref": self.ref_id(), "date": now,
            "note": "ink marks on paper: nozzle tip = camera centre + offset; shaft = the servo axis"}
        tip = by_pulse.get(int(centre), pts.mean(axis=0))
        repeats = {int(p): round(float(np.hypot(*(v[0] - v[-1]))), 3) for p, v in by.items() if len(v) > 1}
        table_msg = ""
        if runout >= 0.5 and int(centre) in by_pulse and len(by_pulse) >= 3:
            # the needle is off the servo axis: where a mark lies on the circle around the axis IS the servo angle
            ps = sorted(by_pulse)
            ang = np.unwrap([np.arctan2(*(by_pulse[p] - axis)[::-1]) for p in ps])
            turns = np.degrees(ang - ang[ps.index(int(centre))])
            fit = nc.fit_table(ps, turns, centre)
            self.cal["servo"] = {
                "pulse_to_turn_deg": [[int(p), round(float(tu), 2)] for p, tu in zip(ps, turns)], "fit": fit,
                "by": "ink marks around the servo axis (the needle is off the axis)", "repeat_mm": repeats,
                "approach_us": float(self.scfg.get("approach_us", 0) or 0), "date": now,
                "note": f"turn of the nozzle from the pick at {centre:.0f} us (the centre), in degrees, + = "
                        "anticlockwise seen from above"}
            table_msg = (f" SERVO: {fit['deg_per_us'] * 1000:+.1f} deg per 1000 us, worst point {fit['worst_deg']:.2f} "
                         "deg" + (" (used for the turns)" if fit["worst_deg"] <= 1.0 else
                                  " (more than 1 deg: NOT used, turns stay nominal)") + ".")
        for key in ("offset_pick", "offset_place"):
            self.cal[key] = {"xy": np.round(tip, 3).tolist(), "by": "ink marks on paper (centre pulse)",
                             "ref": self.ref_id(), "date": now}
        nc.save_cal(self.cal)
        (self.out_dir / "nozzle_marks.yaml").write_text(yaml.safe_dump(self.cal["offset"], sort_keys=False))
        self.say(f"MARKS DONE: tip at camera + X{tip[0]:+.2f} Y{tip[1]:+.2f} mm (now the PICK and PLACE offset); "
                 f"servo axis X{axis[0]:+.2f} Y{axis[1]:+.2f}; runout {runout:.2f} mm (the tip moves this much off the "
                 "axis when the servo turns: corrected at every place); "
                 + ("repeat not measured" if repeat is None else f"repeat {repeat:.2f} mm ({repeats})") + "." + table_msg)

    def job_write_e(self, x: float, y: float) -> None:
        """Accuracy test: a dotted letter E in ink on the paper, centred at (x, y) mm from HOME, readable from the
        front of the machine. Fresh ink before every dot; the servo is not turned (the tip stays where it is)."""
        if self.home is None or not self.z_ok:
            self.say("Home first")
            return
        ink, touch = self.spot("ink", "Ink spot here"), self.touch_z("paper")
        off = self.cal.get("offset_pick") or {}
        if ink is None or touch is None or not off.get("xy"):
            return
        tip, press = np.array(off["xy"], float), float(self.ncfg["press_mm"])
        p = 1.5
        dots = [(-p, k * p) for k in (2, 1, 0, -1, -2)] + [(0, 2 * p), (p, 2 * p), (0, 0), (0, -2 * p), (p, -2 * p)]
        centre = self.home + np.array([x, y], float)
        try:
            for i, (dx, dy) in enumerate(dots):
                self.stage(f"Letter E: dot {i + 1} of {len(dots)}")
                self.z_to(0)
                self.move_to(ink + np.array([(i % 4 - 1.5) * 1.5, -(i // 4) * 1.5]) - tip)
                self.z_from_above(touch - press)
                time.sleep(float(self.ncfg["dwell_s"]))
                self.z_to(0)
                self.move_to(centre + np.array([dx, dy]) - tip)
                self.z_from_above(touch - press)
                time.sleep(float(self.ncfg["dwell_s"]))
        finally:
            self.z_to(0)
        self.move_to(self.clip_to_reach(centre))
        f = self.frame()
        cv2.imwrite(str(self.out_dir / "letter_E_camera.png"), f)
        cv2.imwrite(str(self.out_dir / "letter_E_from_front.png"), cv2.rotate(f, cv2.ROTATE_180))
        self.say("Letter E done (read it from the front of the machine). Pictures: letter_E_*.png")

    def job_dot(self, x: float, y: float) -> None:
        """One ink dot at (x, y) mm from HOME (a target drawn on the paper): fresh ink, servo not turned."""
        if self.home is None or not self.z_ok:
            self.say("Home first")
            return
        ink, touch = self.spot("ink", "Ink spot here"), self.touch_z("paper")
        off = self.cal.get("offset_pick") or {}
        if ink is None or touch is None or not off.get("xy"):
            return
        tip, press, target = np.array(off["xy"], float), float(self.ncfg["press_mm"]), self.home + np.array([x, y])
        try:
            self.z_to(0)
            self.move_to(ink + np.array([1.5, 1.5]) - tip)
            self.z_from_above(touch - press)
            time.sleep(float(self.ncfg["dwell_s"]))
            self.z_to(0)
            self.move_to(target - tip)
            self.z_from_above(touch - press)
            time.sleep(float(self.ncfg["dwell_s"]))
        finally:
            self.z_to(0)
        self.move_to(self.clip_to_reach(target))
        cv2.imwrite(str(self.out_dir / "dot_camera.png"), self.frame())
        self.say(f"Dot made at X{x:+.2f} Y{y:+.2f}. Picture: dot_camera.png")

    # ---- pad check: correct each place on the part's own pads (the board registration may be a little off)
    def board_to_machine(self, xy) -> np.ndarray:
        out = super().board_to_machine(xy)
        shift = getattr(self, "place_shift", None)
        return out if shift is None else out + shift

    def pads_shift(self, part: dict) -> np.ndarray | None:
        """The camera looks at the part's place: each pad (light copper on the dark board) is found near where the
        registration puts it. Both found, and they agree: the shift (mm) from the registration to the real pads."""
        cfg = load_bench()["nozzle"].get("pad_check") or {}
        if not cfg.get("enabled", False) or len(part["pads"]) != 2:
            return None
        self.place_shift = None
        pads = [np.array(pad["p"][0], float).reshape(-1, 2) for pad in part["pads"]]
        target = super().board_to_machine([part["x"], part["y"]])
        self.z_to(0)
        self.move_to(self.clip_to_reach(target))
        img = np.asarray(self.frame(), np.float32)
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img, (0, 0), 1.0)
        ppm, search = self.px_per_mm(), float(cfg.get("search_mm", 1.5))
        shifts = []
        for poly in pads:
            exp = super().board_to_machine(poly.mean(axis=0))
            size = np.ptp(poly, axis=0)                      # pad size in board mm (turned pads: same area)
            area = float(size[0] * size[1]) * ppm ** 2
            c = self.to_px(exp.reshape(1, 2))[0]
            r = int((max(size) / 2 + search) * ppm)
            x0, y0 = int(c[0]) - r, int(c[1]) - r
            if x0 < 0 or y0 < 0 or x0 + 2 * r >= img.shape[1] or y0 + 2 * r >= img.shape[0]:
                return None
            win = img[y0:y0 + 2 * r, x0:x0 + 2 * r]
            _, mask = cv2.threshold(np.clip(win, 0, 255).astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            n, lab, st, cen = cv2.connectedComponentsWithStats(mask)
            ok = [k for k in range(1, n) if 0.4 * area <= st[k, 4] <= 2.5 * area]
            if not ok:
                return None
            k = min(ok, key=lambda k: np.hypot(*(cen[k] + (x0, y0) - c)))
            found = self.pos + np.linalg.solve(self.J, self.centre_px() - (cen[k] + (x0, y0)))
            shifts.append(found - exp)
        if np.hypot(*(shifts[0] - shifts[1])) > float(cfg.get("agree_mm", 0.4)):
            self.say(f"   pads of {part['ref']}: the two pads don't agree: no correction")
            return None
        shift = np.mean(shifts, axis=0)
        if np.hypot(*shift) > search:
            return None
        self.say(f"   pads of {part['ref']} found: place corrected by X{shift[0]:+.2f} Y{shift[1]:+.2f} mm")
        return shift

    def pocket_changed(self, before: np.ndarray, after: np.ndarray, got: dict) -> bool:
        """Did the part leave its pocket? Measured INSIDE the part's own outline (80 % of its size) at the picture
        centre, before and after the pick (same camera spot). On this strip a part is mid-grey (~75), the paper tape
        white (~160) and an empty pocket shows the black tape or the white label under it: a change of 40 or more.
        (The first version averaged a square twice the part's size, mostly tape: it never saw a change.)"""
        ppm = self.px_per_mm()
        length, width = (float(v) for v in got["size_mm"])
        hu, hv = max(2, int(0.4 * length * ppm)), max(2, int(0.4 * width * ppm))   # long side across the strip (u)
        c = self.centre_px().astype(int)
        a, b = (cv2.GaussianBlur(np.asarray(x, np.float32), (0, 0), 0.8) for x in (before, after))
        if a.ndim == 3:
            a, b = (cv2.cvtColor(x, cv2.COLOR_BGR2GRAY) for x in (a, b))
        box = (slice(c[1] - hv, c[1] + hv + 1), slice(c[0] - hu, c[0] + hu + 1))
        inside = float(np.abs(a[box] - b[box]).mean())
        limit = float(self.ncfg.get("pick_change_grey", 20.0))
        self.last_pocket = {"change": round(inside, 1), "before": round(float(a[box].mean()), 1),
                            "after": round(float(b[box].mean()), 1), "limit": limit}
        return inside > limit

    def at_pick(self, got: dict, n: int, part: dict):
        """The real pick (tools/nozzle_calibrate.py at_pick) with two changes for this bench: the servo is not
        commanded again when it is already at the centre (every command moves the tip a little: MG995), and the pick
        is checked by the change in the pocket, not by its darkness."""
        if getattr(self, "real", None) is None:
            return super().at_pick(got, n, part)
        before = self.frame()                         # the camera is centred on the part (centre_on_part)
        shift = self.pads_shift(part)
        self.place_shift = shift
        if shift is not None:
            self.report.setdefault("pad_shifts", {})[part["ref"]] = np.round(shift, 3).tolist()
        r = self.real
        head = got["xy"] - r["pick_off"]
        turn = self.needed_turn(got, part)
        centre = float(self.scfg["centre_us"])
        pulse = centre if turn is None or self.no_servo else self.pulse_for_turn(turn)
        target = self.board_to_machine([part["x"], part["y"]])
        place_head = target - r["place_off"] - self.runout_at(pulse)
        for what, h in (("part", head), ("place", place_head)):
            if not self.in_reach(h - self.home):
                rel = h - self.home
                self.say(f"   the nozzle cannot reach the {what} (head at X{rel[0]:+.1f} Y{rel[1]:+.1f} from HOME, "
                         "outside the reach): skipped")
                self.place_shift = None
                return what
        self.pending = {"turn": turn, "pulse": pulse, "head": place_head}
        self.z_to(0)
        if not self.no_servo and getattr(self, "pulse", None) != centre:
            self.servo(centre)
        self.move_to(head)
        self.pump(True)
        self.say(f"   pick {n}: nozzle down to Z {r['pick_z']:+.2f}, pump on")
        self.z_from_above(r["pick_z"])
        time.sleep(float(self.ncfg["dwell_s"]))
        self.z_to(0)
        self.move_to(self.clip_to_reach(got["xy"]))
        after = self.frame()
        changed = self.pocket_changed(before, after, got)
        lp = self.last_pocket
        self.report.setdefault("pocket_checks", {})[part["ref"]] = {**lp, "picked": changed}
        cv2.imwrite(str(self.out_dir / f"pocket_{part['ref']}_before.png"), np.asarray(before))
        cv2.imwrite(str(self.out_dir / f"pocket_{part['ref']}_after.png"), np.asarray(after))
        if not changed:
            # MISSED: the part is still in its pocket, so nothing should be on the nozzle. The needle is moved back
            # over the pocket first (during the check it is 28 mm behind the camera), then the pump goes off: if the
            # check was ever wrong, the part drops back into its own pocket, never somewhere else.
            self.move_to(head)
            self.pump(False)
            time.sleep(float(self.ncfg["release_s"]))
            self.place_shift = None
            self.say(f"   part {n}: STILL IN ITS POCKET (brightness {lp['before']:.0f} -> {lp['after']:.0f}, change "
                     f"{lp['change']:.0f} < {lp['limit']:.0f}): the pick missed. The next part is tried for this place")
            return "missed"
        self.say(f"   part {n}: picked (the pocket is empty now: brightness {lp['before']:.0f} -> {lp['after']:.0f})")
        self.z_to(r["move_z"])
        return True

    def at_place(self, target, part, got):
        try:
            return super().at_place(target, part, got)
        finally:
            self.place_shift = None

    # ---- nozzle pointer by eye: the needle over a target, nudged in tiny steps, then saved as the offset
    def job_pointer_target(self) -> None:
        """The next click in the picture is the target (the owner's own selection, 2026-10-11: the red cross was not
        what the owner selected, and a wrong offset was saved)."""
        if self.home is None or self.J is None or not self.z_ok:
            self.say("Home first")
            return
        self.pointer = None
        self.click_mode = "pointer"
        self.say("Now CLICK the target (the centre of a part) in the live picture")

    def pointer_clicked(self, u: float, v: float) -> None:
        point = self.pos + np.linalg.solve(self.J, self.centre_px() - np.array([u, v], float))
        self.pointer = {"target": point}
        self.click_mode = "move"
        rel = point - self.home
        self.say(f"Target set at X{rel[0]:+.2f} Y{rel[1]:+.2f} (magenta cross). Now 'Nozzle over the target'")

    def job_pointer_go(self) -> None:
        """The head moves so the needle should be over the target, then down to 0.5 mm above the pick height."""
        ptr, off = getattr(self, "pointer", None), self.cal.get("offset_pick") or {}
        pick = self.touch_z("part")
        if not ptr or not off.get("xy") or pick is None:
            self.say("Set the target first (and the pick height)")
            return
        tip = np.array(off["xy"], float)
        self.z_to(0)
        self.move_to(ptr["target"] - tip)
        ptr["low_z"] = pick - float(self.ncfg["press_mm"]) + 0.5
        self.z_from_above(ptr["low_z"])
        self.say("The needle is 0.5 mm above the target height. LOOK at it and nudge (fine steps) until it is exactly "
                 "over the target, then Save. (Z +-0.1 to see better; it never goes below the pick height)")

    def job_pointer_nudge(self, dx: float, dy: float) -> None:
        """A tiny X / Y move with the needle low (only in pointer mode, at most 0.5 mm per press)."""
        ptr = getattr(self, "pointer", None)
        d = np.array([dx, dy], float)
        if not ptr or "low_z" not in ptr:
            self.say("Press 'Nozzle over the target' first")
            return
        if np.hypot(*d) > 0.5:
            self.say("Nudges are at most 0.5 mm")
            return
        pick = self.touch_z("part", quiet=True)
        if pick is None or self.z < pick - float(self.ncfg["press_mm"]) - 1e-6:
            self.say("The needle is below the pick height: lift it (Z up) before nudging")
            return
        self.restoring = True                         # tiny moves just above a flat part: allowed with the needle low
        try:
            self.move(d)
        finally:
            self.restoring = False
        tip = ptr["target"] - self.pos
        self.say(f"Nudged X{dx:+.2f} Y{dy:+.2f}: the needle offset would be X{tip[0]:+.3f} Y{tip[1]:+.3f}")

    def job_pointer_save(self) -> None:
        ptr = getattr(self, "pointer", None)
        off = self.cal.get("offset_pick") or {}
        if not ptr or "low_z" not in ptr or not off.get("xy"):
            self.say("Nothing to save: target, nozzle over it, nudge, then Save")
            return
        tip = ptr["target"] - self.pos
        delta = tip - np.array(off["xy"], float)
        now = datetime.datetime.now().isoformat(timespec="seconds")
        self.cal["offset_pick"] = {"xy": np.round(tip, 3).tolist(), "by": "nozzle pointer by eye", "ref": self.ref_id(),
                                   "date": now}
        place = self.cal.get("offset_place") or {}
        if place.get("xy"):
            self.cal["offset_place"] = {**place, "xy": np.round(np.array(place["xy"], float) + delta, 3).tolist(),
                                        "by": "moved like the pick offset (nozzle pointer by eye)", "ref": self.ref_id(),
                                        "date": now}
        self.cal["offset_guess_mm"] = np.round(tip, 1).tolist()
        nc.save_cal(self.cal)
        self.z_to(0)
        self.pointer = None
        self.say(f"Nozzle pointer SAVED: tip at camera + X{tip[0]:+.3f} Y{tip[1]:+.3f} mm (changed X{delta[0]:+.3f} "
                 f"Y{delta[1]:+.3f}); the place offset moved the same. Nozzle lifted")

    def job_pointer_cancel(self) -> None:
        self.z_to(0)
        self.pointer = None
        self.say("Nozzle pointer cancelled, nozzle lifted")

    # ---- camera exposure for the light on the bed (2026-10-11: a new lamp, 35 % of the picture white)
    def picture_light(self) -> dict:
        """How bright the picture is: the brightest 0.5 % (should sit just under white) and how much is clipped."""
        f = np.asarray(self.cam.frames_after(time.monotonic() + 0.6, 3)[-1], np.float32)
        h, w = f.shape[:2]
        mid = f[h // 10: h * 8 // 10, w // 10: w * 9 // 10]          # not the edges, not the nozzle's shadow
        return {"p995": float(np.percentile(mid, 95)), "median": float(np.median(mid)),
                "clipped_pct": float(100 * np.mean(mid >= 250))}

    def job_set_exposure(self, value: float) -> None:
        """The exposure by hand (page: - / +), kept like the automatic one."""
        if not hasattr(self.cam, "set_control"):
            return
        self.cam.set_control("AUTO_EXPOSURE", 1.0)
        self.cam.set_control("EXPOSURE", round(float(value)))
        light = self.picture_light()
        path = BENCH_FILE
        text = re.sub(r"(?m)^(  exposure: )\d+(.*)$", rf"\g<1>{round(float(value))}  # set by hand", path.read_text())
        path.write_text(text)
        self.say(f"Exposure {round(float(value))}: bright 5 % at {light['p995']:.0f}, median {light['median']:.0f}, "
                 f"white {light['clipped_pct']:.1f} %. Saved")

    def job_auto_exposure(self) -> None:
        """The camera's own auto exposure is switched off (it would change the brightness between the dark board
        and the white labels), and the exposure is set so the brightest parts of the picture sit just under white.
        Saved in config/bench.yaml camera.exposure and used at every start."""
        cfg = load_bench().get("camera", {}) or {}
        target, tol = float(cfg.get("target_p95", 200)), 12.0        # a lamp from one side: some glare is allowed
        max_white = float(cfg.get("max_white_pct", 3.0))
        if not hasattr(self.cam, "set_control"):
            self.say("This camera can't be set")
            return
        self.stage("Auto exposure: measuring")
        self.cam.set_control("AUTO_EXPOSURE", float(cfg.get("manual_mode", 1)))   # V4L2: 1 = manual
        e = self.cam.get_control("EXPOSURE") or 0.0
        if e <= 0:
            e = 156.0
        lo, hi, best = None, None, None
        for _ in range(12):
            self.cam.set_control("EXPOSURE", round(e))
            light = self.picture_light()
            self.say(f"   exposure {e:.0f}: bright 5 % at {light['p995']:.0f}, median {light['median']:.0f}, "
                     f"white {light['clipped_pct']:.1f} %")
            if abs(light["p995"] - target) <= tol and light["clipped_pct"] < max_white:
                best = e
                break
            if light["p995"] > target or light["clipped_pct"] >= max_white:
                hi = e
            else:
                lo = e
            e = (lo * hi) ** 0.5 if lo and hi else (e / 2 if hi == e else e * 1.6)
            if e < 1 or e > 10000:
                break
            best = e
        if best is None:
            self.say("Auto exposure: the camera does not react to the exposure setting")
            return
        best = round(best)
        self.cam.set_control("EXPOSURE", best)
        light = self.picture_light()
        path = BENCH_FILE
        text = path.read_text()
        block = (f"camera:\n  manual_mode: 1        # V4L2 auto exposure off (1 = manual): the brightness must not change\n"
                 f"  exposure: {best}        # set by the page's Auto exposure {datetime.date.today().isoformat()} "
                 f"(brightest {light['p995']:.0f}, white {light['clipped_pct']:.1f} %)\n"
                 f"  target_p95: {target:g}     # the brightest 5 % of the picture should sit here (white = 255)\n"
                 f"  max_white_pct: {max_white:g}    # at most this much pure white (the lamp's glare)\n")
        if re.search(r"(?m)^camera:\n(?:  .*\n)+", text):
            text = re.sub(r"(?m)^camera:\n(?:  .*\n)+", block, text)
        else:
            text = text.rstrip("\n") + "\n\n# Head camera settings for the light on the bed (ui/engine.py job_auto_exposure)\n" + block
        path.write_text(text)
        self.say(f"Auto exposure DONE: exposure {best} (brightest {light['p995']:.0f}, median {light['median']:.0f}, "
                 f"white {light['clipped_pct']:.1f} %). Saved: used at every start")

    def slot_frame(self, slot: str):
        """The slot label, or (not seen: it sits low in the picture, under the nozzle's shadow) the slot position
        from the bed survey (config/bed.yaml layout, ~1 mm): enough, the camera centres on every part itself."""
        got = super().slot_frame(slot)
        if got is not None or self.home is None or slot not in self.nominal:
            return got
        u = np.array(self.bed["layout"].get("slot_direction", [0, -1]), float)
        u /= np.linalg.norm(u)
        v = np.array([-u[1], u[0]]) * -1.0           # the label's v: u turned -90 deg (marker corner 0 -> 3)
        self.say(f"   {slot} label not seen: its surveyed position is used")
        return self.home + self.nominal[slot] - self.nominal["home"], u, v

    def centre_on_part(self, part: dict, frame) -> dict | None:
        got = super().centre_on_part(part, frame)
        if got is None:                               # say what the close look saw (debug, 2026-10-11)
            seen = self.parts_here(frame)
            near = [(round(float(np.hypot(*(p["xy"] - self.pos))), 2), p["unclear"], [round(v, 1) for v in p["size_mm"]])
                    for p in seen]
            self.say(f"   close look at X{(self.pos - self.home)[0]:+.1f} Y{(self.pos - self.home)[1]:+.1f}: "
                     f"{len(seen)} spots (distance mm, why not, size): {near[:6]}")
            cv2.imwrite(str(self.out_dir / f"closelook_{len(list(self.out_dir.glob('closelook_*')))}.png"),
                        np.asarray(self.cam.frames_after(time.monotonic(), 1)[0]))
        return got

    # ---- the job
    def wait_if_paused(self) -> None:
        while self.pause_flag.is_set():
            if self.stop_flag.is_set():
                raise t.Stopped()
            time.sleep(0.1)
        if self.stop_flag.is_set():
            raise t.Stopped()

    def job_run_one(self) -> None:
        self.job_run("real", limit=1)

    def job_run(self, mode: str = "dry", limit: int | None = None) -> None:
        """Every assigned slot, part by part. mode 'dry': the camera visits the parts and the places (no Z, no
        pump). mode 'real': the nozzle picks, turns and places (needs the Setup steps)."""
        if self.reach_mode:
            self.say("Save or cancel the reach measuring first")
            return
        plan_slots = self.assignments()
        if not plan_slots:
            self.say("Assign components to slots first (Feeders)")
            return
        if self.home is None or self.J is None:
            self.say("Press Home first")
            return
        if mode == "real":
            plan = self.real_plan()
            if plan is None:
                return
            self.z_to(0)
            if not getattr(self, "checked_for_r", False) and not self.camera_height_still_right():
                return
            self.real = plan
        else:
            if self.z_locked or not self.nozzle_up():
                self.say("Lift the nozzle first (the dry run moves at this height)")
                return
            self.real = None
        if self.M is None:
            self.say("Register the board first (Setup: board corners, or Auto-find holes)")
            return
        total = sum(len([r for r in g["refs"] if r not in self.placed_refs]) for _, g in plan_slots)
        self.job_info = {"mode": mode, "total": total, "done": 0, "slot": None, "line": None, "skipped": []}
        self.job_limit = limit
        self.run_state = "running"
        self.pause_flag.clear()
        self.say(f"JOB ({'REAL' if mode == 'real' else 'DRY: camera only'}): {total} parts from "
                 f"{len(plan_slots)} slot(s)")
        try:
            for slot, line in plan_slots:
                self.run_line(slot, line)
                if self.job_limit is not None and self.job_info["done"] >= self.job_limit:
                    break
        except BaseException:
            if mode == "real":
                self.interrupted()
            raise
        finally:
            self.run_state = "idle"
            self.real = None
            self.save_job_report()
        self.say(f"JOB DONE: {self.job_info['done']} placed"
                 + (f", {len(self.job_info['skipped'])} skipped ({', '.join(self.job_info['skipped'][:8])})"
                    if self.job_info["skipped"] else ""))

    def run_line(self, slot: int, line: dict) -> None:
        refs = set(line["refs"]) - self.placed_refs
        parts = sorted([p for p in self.board["parts"] if p["ref"] in refs], key=lambda p: (p["y"], p["x"]))
        if not parts:
            return
        self.args.slot, self.args.value = slot, line["value"]
        self.todo, self.slot_view, self.next_k = parts, None, None
        self.job_info.update(slot=slot, line=line["label"])
        if not self.prepare_slot(reset=True):
            self.say(f"Slot {slot} not found: its {len(parts)} parts are skipped")
            self.job_info["skipped"] += [p["ref"] for p in parts]
            return
        while True:
            self.wait_if_paused()
            before = len(self.report["placements"])
            more = self.place_next()
            for pl in self.report["placements"][before:]:
                self.placed_refs.add(pl["ref"])
                self.job_info["done"] += 1
            if self.real is not None:
                self.save_placed()
            if self.job_limit is not None and self.job_info["done"] >= self.job_limit:
                self.say(f"Stopped after {self.job_limit} part(s), as asked: look at it, then Start again")
                break
            if more:
                continue
            if self.slot_view is None and self.next_k is not None and self.next_k < len(self.todo):
                self.say(f"Slot {slot} is EMPTY: refill it with the same parts ({line['label']}), then press Resume. "
                         "Or Stop")
                self.run_state = f"paused: refill slot {slot}"
                self.pause_flag.set()
                self.wait_if_paused()
                self.run_state = "running"
                if not self.prepare_slot(reset=False):
                    break
                continue
            break
        self.job_info["skipped"] += [x["ref"] for x in self.report.get("not_placed", []) if x["ref"] in refs]
        self.job_log.append({"slot": slot, "line": line["label"], **{k: self.report.get(k) for k in
                             ("placements", "not_placed", "picks", "slot_parts")}})

    def save_job_report(self) -> None:
        (self.out_dir / "job_report.yaml").write_text(yaml.safe_dump(
            {"date": datetime.datetime.now().isoformat(timespec="seconds"), "side": self.args.side,
             "top_edge": self.args.top_edge, "info": self.job_info, "slots": self.job_log,
             "placed_on_board": sorted(self.placed_refs)}, sort_keys=False))

    # ---- what the page shows
    def map_data(self) -> dict:
        """The bed map in mm from HOME (+X right, +Y back): reach, labels, PCB area, board, pads, holes."""
        home = self.home if self.home is not None else None
        lab = {n: (np.asarray(xy) - self.nominal["home"]).round(2).tolist() for n, xy in self.nominal.items()}
        lay = self.bed.get("layout", {})
        area = lay.get("pcb_area_xy")
        size = self.bed.get("pcb_area", {}).get("size_mm", [130, 140])
        out = {"labels": lab, "marker_mm": float(self.markers.size_mm) if hasattr(self.markers, "size_mm") else 10.0,
               "reach": None if self.reach is None else {"x": self.reach["x"], "y": self.reach["y"]},
               "pcb_area": None if area is None else [area[0], area[1], area[0] + size[0], area[1] + size[1]],
               "board": None, "parts": [], "holes": {}}
        if home is not None and self.M is not None:
            x0, y0, x1, y1 = self.board["bbox"]
            out["board"] = (self.board_to_machine([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]) - home).round(2).tolist()
            todo = {r for _, g in self.assignments() for r in g["refs"]}
            for p in self.board["parts"]:
                if p["side"] == self.args.side and p["smd"] and not p["dnp"]:
                    xy = (self.board_to_machine([p["x"], p["y"]]) - home).round(2).tolist()
                    out["parts"].append([xy[0], xy[1], p["ref"], 2 if p["ref"] in self.placed_refs else
                                         (1 if p["ref"] in todo else 0)])
        if home is not None:
            out["holes"] = {h: (self.corner_target(h) - home).round(2).tolist() for h in d.HOLES}
        return out

    def status(self) -> dict:
        def f(v):
            return None if v is None else round(float(v), 3)
        rel = (self.pos - self.home) if self.home is not None else self.pos
        heights = {n: f(self.touch_z(n, quiet=True)) for n in HEIGHT_NAMES}
        offs = {}
        for key in ("offset_pick", "offset_place"):
            e = self.cal.get(key)
            offs[key] = None if not e else {"xy": e["xy"], "ok": bool(e.get("ref")) and e.get("ref") == self.ref_id()}
        extremes = getattr(self, "extremes", None)
        ext = None if extremes is None else [round(float(v), 1) for v in extremes]
        with self.lock:
            msg = self.msg
        return {
            "pos": [round(float(rel[0]), 2), round(float(rel[1]), 2)], "z": round(float(self.z), 2),
            "homed": self.home is not None, "camera_ok": self.J is not None,
            "camera_height": {"set": bool(self.cal.get("z_reference")), "checked": bool(self.z_ok)},
            "locked": self.z_locked, "busy": bool(getattr(self, "busy", False)),
            "moving": bool(getattr(self, "moving", False)),
            "msg": msg, "step": self.step_name, "pump": bool(self.pump_on),
            "heights": heights, "offsets": offs,
            "offset_step": 2 if self.nozzle_target else 1,
            "click_mode": self.click_mode, "land_surface": None if not self.land else self.land["surface"],
            "turn_deg": round(self.turn_deg, 1), "pick_mark": self.pick_mark is not None,
            "nozzle": None if self.home is None or not self.cal.get("offset_pick") else
                      [round(float(v), 2) for v in (self.pos - self.home + np.array(self.cal["offset_pick"]["xy"], float))],
            "offset_kind": self.nozzle_target[0] if self.nozzle_target else None,
            "reach": {"stale": bool(getattr(self, "reach_stale", False)), "measuring": self.reach_mode, "extremes": ext,
                      "x": None if self.reach is None else self.reach["x"], "y": None if self.reach is None else self.reach["y"]},
            "board": {"side": self.args.side, "top_edge": self.args.top_edge, "registered": self.M is not None,
                      "corners": sorted(self.corners), "info": self.reg_info},
            "run": {"state": self.run_state, **{k: v for k, v in self.job_info.items() if k != "skipped"},
                    "skipped": len(self.job_info.get("skipped", []))},
            "placed": len(self.placed_refs),
            "events": list(self.events)[-14:],
        }

    def render(self, frame: np.ndarray) -> np.ndarray:
        """The live picture with the overlays (no text panel: the page shows the status)."""
        view = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy()
        h, w = view.shape[:2]
        for name, c in self.markers.find(frame).items():
            cv2.polylines(view, [c.astype(np.int32)], True, (255, 180, 0), 2)
            cv2.putText(view, name.upper().replace("_", " "), tuple(int(v) for v in c.mean(axis=0) + (-30, -32)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 220, 120), 2, cv2.LINE_AA)
        if self.J is not None and not self.moving:
            for poly, colour, text in list(self.overlay):
                px = self.to_px(poly)
                cv2.polylines(view, [px.astype(np.int32)], len(poly) > 6, colour, 2, cv2.LINE_AA)
            if self.home is not None:
                for hole in d.HOLES:                  # the 4 corner holes (where they are, or should be)
                    p = self.to_px(self.corner_target(hole).reshape(1, 2))[0]
                    if -40 < p[0] < w + 40 and -40 < p[1] < h + 40:
                        r = int(d.HOLE_D_MM / 2 * self.px_per_mm())
                        colour = (0, 255, 0) if hole in self.corners else (0, 200, 255)
                        cv2.circle(view, tuple(int(v) for v in p), r, colour, 2, cv2.LINE_AA)
                        cv2.putText(view, hole, (int(p[0]) + r + 4, int(p[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    colour, 2, cv2.LINE_AA)
            off = self.cal.get("offset_pick")
            if off:                                   # where the nozzle tip comes down (at the pick height)
                p = self.to_px((self.pos + np.array(off["xy"], float)).reshape(1, 2))[0]
                if 0 <= p[0] < w and 0 <= p[1] < h:
                    cv2.circle(view, tuple(int(v) for v in p), 7, (255, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(view, "nozzle", (int(p[0]) + 10, int(p[1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 0), 1, cv2.LINE_AA)
        ptr = getattr(self, "pointer", None)
        if ptr and self.J is not None and not self.moving:     # the pointer target the owner clicked
            p = self.to_px(np.asarray(ptr["target"], float).reshape(1, 2))[0]
            if 0 <= p[0] < w and 0 <= p[1] < h:
                cv2.drawMarker(view, (int(p[0]), int(p[1])), (255, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.drawMarker(view, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 40, 1)
        return view
