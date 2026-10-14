"""The machine's vision operations (Phase C): camera calibration, exposure, camera homing, click-to-go, board
registration, the stall check and the live overlay. Used by Machine (machine.py); every long step runs on the
machine's worker thread and moves only through the machine's checked moves."""
from __future__ import annotations

import math
import random
import time
from typing import TYPE_CHECKING

import numpy as np

from .capabilities.base import Availability, Capability, CapabilityError
from .motion.state import Refusal, Refused
from .vision import calibrate as vcal
from .vision import exposure as vexp
from .vision import markers as vmk
from .vision import register as vreg
from .vision.camera_model import CameraModel
from .vision.motion_check import moved_as_predicted

if TYPE_CHECKING:
    from .machine import Machine

SETTLE_S = 0.15                 # after a move: wait, then use pictures taken after that (CLAUDE.md: stale frames)


def move_homography(model: CameraModel, d) -> np.ndarray:
    """How the picture of the bed (z = 0) changes when the head moves by d (mm): a homography."""
    import cv2
    w, h = model.size
    px = np.array([[x, y] for x in np.linspace(0.1 * w, 0.9 * w, 5) for y in np.linspace(0.1 * h, 0.9 * h, 5)])
    xy = model.px_to_xy(px, 0.0)
    after = model.xy_to_px(xy - np.asarray(d, float), 0.0)
    H, _ = cv2.findHomography(px.astype(np.float32), after.astype(np.float32))
    return H


class MachineRig:
    """The head and the camera for the vision procedures. frame='raw': counted controller positions (camera
    calibration, before Home); frame='home': mm from HOME (registration, after Home)."""

    def __init__(self, m: Machine, frame: str = "raw") -> None:
        self.m, self.frame_kind = m, frame
        self.cam = m.cameras.get("camera_head")

    def head(self):
        raw = self.m.raw()
        if self.frame_kind == "home":
            pos = self.m.position()
            return (pos["X"], pos["Y"])
        dz = raw.get("Z", 0.0) if (self.m.profile.cameras.head and self.m.profile.cameras.head.moves_with_z) else 0.0
        return (raw["X"], raw["Y"], dz)

    def _check(self, delta, target_home=None) -> None:
        if self.m.worker.cancel.is_set():
            from .motion.state import Stopped
            raise Stopped()
        r = self.m.limits.check_step(delta)
        if r is None and target_home is not None:
            r = self.m.limits.check_target(target_home, "This move")
        if r is not None:
            raise Refused(r)

    def move_by(self, dx: float, dy: float) -> None:
        delta = {"X": float(dx), "Y": float(dy)}
        pos = self.m.position()
        target = {"X": pos["X"] + dx, "Y": pos["Y"] + dy} if pos and pos.get("X") is not None else None
        self._check(delta, target)
        self._done(self.m._move(delta, None, self.m.limits.feed(delta, None, self.m.safety.homed), stall_check=True))

    def _done(self, result: str) -> None:
        if result == "stopped":                     # STOP, a lock, a stall: nothing more may move after it
            from .motion.state import Stopped
            if self.m.safety.lock is not None:
                raise Refused(Refusal(self.m.safety.lock.title, self.m.safety.lock.todo))
            raise Stopped()

    def move_to(self, x: float, y: float) -> None:
        pos = self.m.position()
        delta = {"X": float(x) - pos["X"], "Y": float(y) - pos["Y"]}
        self._check(delta, {"X": float(x), "Y": float(y)})
        home = self.m.session.home_raw
        self._done(self.m._move(delta, {"X": home["X"] + x, "Y": home["Y"] + y}, self.m.limits.feed(delta, None, True),
                                stall_check=True))

    def frames(self, n: int):
        return self.cam.frames_after(time.monotonic() + SETTLE_S, n, timeout=5.0)

    def frame(self):
        return self.frames(1)[0]


class CameraHoming(Capability):
    """Home = the HOME label measured in the picture with the calibrated camera (D4, D17: HOME may sit just
    outside the reach, so it is measured where it is seen, not driven onto)."""
    role, kind, restores_after_reset = "homing", "camera_marker", False

    def __init__(self, v: VisionOps) -> None:
        self.v = v

    def availability(self) -> Availability:
        if self.v.model() is None:
            return Availability.no(f"The camera is not calibrated ({self.v.model_why})",
                                   "Calibrate the camera first (Setup, step 3)")
        if self.v.m.markers is None:
            return Availability.no("No bed file (bed.yaml) with the HOME label", "Add config/bed.yaml")
        return Availability.yes()

    def home(self, ctl) -> dict[str, float]:
        self.require()
        rig = MachineRig(self.v.m)
        frames = rig.frames(5)
        offset, why = vmk.home_offset(self.v.m.markers, frames, self.v.model(), 0.0)
        if offset is None:
            raise CapabilityError(why)
        raw = ctl.raw()
        return {"X": raw["X"] + float(offset[0]), "Y": raw["Y"] + float(offset[1])}


class VisionOps:
    def __init__(self, m: Machine) -> None:
        self.m = m
        self.views: dict[int, list] = {0: [], 1: []}
        self.block_mm: float | None = None
        self.model_why = ""
        self._model: CameraModel | None = None
        self._model_key = None
        self.last_report: list[str] = []

    # ---------------------------------------------------------------- the camera model
    def model(self) -> CameraModel | None:
        head = self.m.profile.cameras.head
        if head is None:
            self.model_why = "no head camera"
            return None
        e = self.m.calib.get("camera_head")
        key = (head.hardware_id, None if e is None else e.get("meta", {}).get("date"))
        if key != self._model_key:
            v, why = self.m.calib.value("camera_head", {"camera": head.hardware_id})
            self._model = None if v is None else CameraModel.from_dict(v)
            if self._model is not None:
                self._model.moves_with_z = bool(head.moves_with_z)
            self.model_why = why
            self._model_key = key
        return self._model

    def board_top_z(self, board) -> float:
        support = float(((self.m.bed or {}).get("pcb_area") or {}).get("support_mm") or 0.0)
        return support + float(board.thickness)

    # ---------------------------------------------------------------- exposure
    def _light(self):
        cap = self.m.caps.lights.get("light_head")
        return cap if cap is not None and cap.kind != "none" and cap.availability().ok else None

    def _exp_key(self) -> str:
        """The exposure belongs to a light state (docs/13 §4.14): with a head light that is off, its own value."""
        light = self._light()
        return "exposure_head_off" if light is not None and not getattr(light, "on", False) else "exposure_head"

    def auto_exposure(self) -> None:
        cam = self.m.cameras.get("camera_head")
        if cam is None or cam.kind == "none":
            raise CapabilityError("No head camera")
        head = self.m.profile.cameras.head

        def grab():
            time.sleep(0.4)
            return cam.frames_after(time.monotonic(), 1)[0]
        light = self._light()
        states = [("exposure_head", True), ("exposure_head_off", False)] if light is not None else [("exposure_head", None)]
        for key, on in states:
            if on is not None:
                light.set(on)
            start = (self.m.calib.get(key) or {}).get("value", {}).get("exposure") or head.exposure.value or 100
            self.m.say("Auto exposure: measuring the light" + ("" if on is None else f" (head light {'on' if on else 'off'})"),
                       kind="camera")
            value, lt = vexp.auto(cam.set_exposure, grab, float(start), say=lambda s: self.m.say(s, "debug", "camera"))
            if value is None:
                raise CapabilityError("The camera does not react to the exposure setting")
            self.m.calib.put(key, {"exposure": value, "light": lt}, {"camera": head.hardware_id},
                             "auto exposure (brightest 5 % just under white)")
            self.m.say(f"Exposure {value}: bright 5 % at {lt['p95']:.0f}, median {lt['median']:.0f}, white "
                       f"{lt['white_pct']:.1f} %. Saved", kind="camera")
        if light is not None:                                       # the light stays on for the work
            light.set(True)
            self.apply_saved_exposure()

    def apply_saved_exposure(self) -> None:
        cam = self.m.cameras.get("camera_head")
        head = self.m.profile.cameras.head
        if cam is None or head is None or not hasattr(cam, "set_exposure"):
            return
        v, _ = self.m.calib.value(self._exp_key(), {"camera": head.hardware_id})
        if v is not None:
            cam.set_exposure(v["exposure"])
        elif head.exposure.value is not None and not head.exposure.auto:
            cam.set_exposure(head.exposure.value)

    def light_problem(self) -> str | None:
        cam = self.m.cameras.get("camera_head")
        head = self.m.profile.cameras.head
        if cam is None or head is None or cam.latest() is None:
            return None
        v, _ = self.m.calib.value(self._exp_key(), {"camera": head.hardware_id})
        return vexp.changed(vexp.light(cam.latest()[1]), None if v is None else v.get("light"))

    # ---------------------------------------------------------------- camera calibration
    def calibrate(self, step: str, block_mm: float | None = None) -> None:
        head = self.m.profile.cameras.head
        if head is None:
            raise CapabilityError("No head camera in the profile")
        if step in ("bed", "block"):
            problem = self.light_problem()
            if problem and "never set" not in problem:
                self.m.say(problem, "warn", "camera")
            plane = 0 if step == "bed" else 1
            if plane == 1:
                if not block_mm or block_mm <= 0:
                    raise Refused(Refusal("The block height is missing", "Measure the block with calipers and type it in"))
                self.block_mm = float(block_mm)
            self.m.say("Camera calibration: pictures of the sheet " + ("on the bed" if plane == 0 else
                       f"on the {self.block_mm:g} mm block"), kind="camera")
            views = vcal.collect(MachineRig(self.m), plane, say=lambda s: self.m.say(s, "debug", "camera"))
            self.views[plane] = views
            self.m.say(f"   {len(views)} good pictures" + ("" if len(views) >= 3 else
                       ": too few. Is the whole sheet near the picture centre, flat and lit?"),
                       "info" if len(views) >= 3 else "warn", "camera")
            return
        if step == "solve":
            views = self.views[0] + self.views[1]
            res = vcal.solve(views, tuple(head.resolution), self.block_mm if self.views[1] else None)
            self.last_report = res.report()
            if max(abs(s_ - 1) for s_ in res.move_scale) > 0.03 or res.rms_px > 2.0:
                for line in self.last_report:
                    self.m.say("Camera calibration: " + line, "warn", "camera")
                raise CapabilityError("Not saved: the head did not move as counted (it hit an end or lost steps) or the "
                                      "sheet moved. Put the sheet where the head has room all round (25 mm), then take "
                                      "the pictures again")
            self.m.calib.put("camera_head", res.model.to_dict(), {"camera": head.hardware_id},
                             "ChArUco sheet, bed" + (f" + {self.block_mm:g} mm block" if self.views[1] else ""),
                             res.model.quality)
            for line in self.last_report:
                self.m.say("Camera calibration: " + line, "warn" if line in res.warnings else "info", "camera")
            self._model_key = None
            self.m._install_vision_caps()
            return
        raise ValueError(f"unknown calibration step {step}")

    # ---------------------------------------------------------------- click on the picture
    def click_target(self, u: float, v: float) -> np.ndarray:
        """A click (u, v as fractions of the picture) -> how far the head must move to bring it to the centre."""
        model = self.model()
        if model is None:
            raise Refused(Refusal("The camera is not calibrated", "Calibrate it first (Setup, step 3)"))
        w, h = model.size
        px = [[u * w, v * h]]
        d = model.px_to_xy(px, 0.0)[0]
        # on the registered board the spot is higher than the bed: the tilted camera sees it shifted (~0.47 mm/mm)
        board, reg = self.registration()
        pos = self.m.position()
        if reg is not None and board.outline and pos and pos.get("X") is not None:
            z = self.board_top_z(board)
            d_top = model.px_to_xy(px, z)[0]
            spot = reg.to_board(board, np.array([pos["X"], pos["Y"]]) + d_top)
            if _inside_polygon(spot, board.outline[0]):
                return d_top
        return d

    # ---------------------------------------------------------------- board registration
    @staticmethod
    def rkey(copy: str = "1") -> str:
        """Board copy 1 keeps the old key; a panel's other copies: board_registration#<copy> (Phase G)."""
        return "board_registration" if str(copy) == "1" else f"board_registration#{copy}"

    def register(self, side: str, top_edge: str, copy: str = "1", centre_xy=None) -> None:
        board = self.m.boards.current()
        if board is None:
            raise Refused(Refusal("No board", "Import the board first (Board tab)"))
        model = self.model()
        if model is None:
            raise Refused(Refusal("The camera is not calibrated", "Calibrate it first (Setup, step 3)"))
        if not self.m.safety.homed:
            raise Refused(Refusal("Not homed", "Home first (Machine tab)"))
        lay = (self.m.bed or {}).get("layout", {}) or {}
        area = ((self.m.bed or {}).get("pcb_area") or {}).get("size_mm") or [board.width, board.height]
        if centre_xy is not None:                                  # a panel's other board: where it lies (planned)
            centre = np.asarray(centre_xy, float)
        elif lay.get("pcb_area_xy") is None:
            raise Refused(Refusal("No board area on the bed", "Write layout.pcb_area_xy in bed.yaml (or survey it)"))
        else:
            centre = np.array(lay["pcb_area_xy"], float) + np.array(area, float) / 2
        M0, t0 = vreg.guess(board, side, top_edge, centre)
        tr = self.m.limits.travel
        reach = (tr["X"][0], tr["X"][1], tr["Y"][0], tr["Y"][1]) if tr["X"] and tr["Y"] else None
        self.m.say(f"Registering {board.name}" + ("" if str(copy) == "1" else f" copy {copy}") +
                   f" ({side} side up, its top edge at the {top_edge})", kind="board")
        reg = vreg.run(MachineRig(self.m, "home"), board, side, M0, t0, model, self.board_top_z(board), reach,
                       say=lambda s: self.m.say(s, "debug", "board"))
        if reg is None:
            raise CapabilityError("Fewer than 2 features found: register by clicks (Board tab), or check the side "
                                  "and the top edge")
        head = self.m.profile.cameras.head
        self.m.calib.put(self.rkey(copy), {"board": board.name, "side": side, "top_edge": top_edge, "copy": str(copy),
                                           "registration": reg.to_dict()},
                         {"camera": head.hardware_id, "board": board.name, "token": self.m.session.token},
                         "features: " + ", ".join(sorted({f.kind for f in reg.features if f.measured})),
                         {"rms_mm": round(reg.rms, 4), "max_mm": round(reg.max, 4)})
        for line in reg.report():
            self.m.say("Board: " + line, "warn" if "not used" in line or "check" in line.lower() else "info", "board")
        self.m.say("Look at the live picture: the drawn pads must sit on the board's pads (Machine tab, overlay)",
                   kind="board")

    def registration(self, copy: str = "1"):
        board = self.m.boards.current()
        head = self.m.profile.cameras.head
        if board is None or head is None:
            return None, None
        v, _ = self.m.calib.value(self.rkey(copy), {"camera": head.hardware_id, "board": board.name,
                                                    "token": self.m.session.token})
        if v is None:
            return board, None
        return board, vreg.Registration.from_dict(v["registration"])

    def registrations(self) -> list[tuple[str, object]]:
        """Every registered copy of the current board (a panel, Phase G): [(copy, Registration)], copy 1 first."""
        out = []
        copies = ["1"] + sorted(k.split("#", 1)[1] for k in self.m.calib.keys() if k.startswith("board_registration#"))
        for c in copies:
            _, reg = self.registration(c)
            if reg is not None:
                out.append((c, reg))
        return out

    # ---------------------------------------------------------------- the stall check
    def stall_check(self, before, delta: dict[str, float]) -> bool | None:
        """After an automatic move: did the picture move as the model says it should? None = can't tell."""
        model = self.model()
        cam = self.m.cameras.get("camera_head")
        if model is None or cam is None or before is None:
            return None
        d = np.array([delta.get("X", 0.0), delta.get("Y", 0.0)])
        exp = model.shift_px(d)
        w, h = model.size
        if np.hypot(*exp) < 3 or abs(exp[0]) > 0.45 * w or abs(exp[1]) > 0.45 * h:
            return None                                          # too small to see / the views hardly overlap
        H = move_homography(model, d)
        after = cam.frames_after(time.monotonic() + SETTLE_S, 1)[0]
        return moved_as_predicted(before, after, H, 0.05)

    # ---------------------------------------------------------------- the overlay on the live picture
    def overlay(self) -> dict:
        model = self.model()
        out = {"pads": [], "holes": [], "outline": [], "w": None, "h": None}
        if model is None:
            return out
        w, h = model.size
        out["w"], out["h"] = w, h
        board, reg = self.registration()
        pos = self.m.position()
        if reg is None or not pos or pos.get("X") is None:
            return out
        head = np.array([pos["X"], pos["Y"]])
        z = self.board_top_z(board)
        reach = 0.6 * max(w, h) / model.px_per_mm(z)

        def px(pts):
            q = model.xy_to_px(np.asarray(pts, float) - head, z)
            return [[round(float(a) / w, 4), round(float(b) / h, 4)] for a, b in q]

        for c in board.components:
            if c.side != reg.side:
                continue
            m = reg.to_machine(board, (c.x, c.y))
            if np.hypot(*(m - head)) > reach:
                continue
            for pad in c.pads:
                if pad.plated and pad.layer in ("", reg.side):
                    out["pads"].append(px([reg.to_machine(board, p) for p in pad.polygon]))
        for hole in board.holes:
            m = reg.to_machine(board, (hole.x, hole.y))
            if np.hypot(*(m - head)) < reach:
                out["holes"].append(px([m + hole.diameter / 2 * np.array([math.cos(a), math.sin(a)])
                                        for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)]))
        if board.outline:
            pts = [reg.to_machine(board, p) for p in board.outline[0]]
            if min(np.hypot(*(np.asarray(p) - head)) for p in pts) < 2 * reach:
                out["outline"].append(px(pts + [pts[0]]))
        return out

    # ---------------------------------------------------------------- status
    def status(self) -> dict:
        model = self.model()
        board, reg = self.registration()
        exp, _ = self.m.calib.value("exposure_head", {"camera": self.m.profile.cameras.head.hardware_id}) \
            if self.m.profile.cameras.head else (None, "")
        return {"model": None if model is None else {"tilt_deg": round(model.tilt_deg, 2),
                                                      "height_mm": round(model.height_mm, 1),
                                                      "px_per_mm": round(model.px_per_mm(0), 3),
                                                      "rms_px": model.quality.get("rms_px")},
                "model_why": self.model_why, "views": {k: len(v) for k, v in self.views.items()},
                "block_mm": self.block_mm, "exposure": exp,
                "registration": None if reg is None else {"board": board.name, "side": reg.side, "kind": reg.kind,
                                                          "rms": round(reg.rms, 3), "max": round(reg.max, 3),
                                                          "angle": round(reg.angle_deg, 2)}}

    # ---------------------------------------------------------------- the simulator's props
    def sim_prop(self, what: str, **kw) -> str:
        from . import sim
        cam = self.m.cameras.get("camera_head")
        g = self.m.sim_grbl
        if g is None or cam is None or not hasattr(cam, "world"):
            raise Refused(Refusal("Only in the simulator", ""))
        w = cam.world
        hx, hy = g.phys["X"], g.phys["Y"]
        if what == "charuco":
            z = float(kw.get("z", 0.0))
            w.remove("charuco")
            w.add(sim.charuco_layer(w, hx - 30.0, hy - 22.0, theta=random.uniform(-6, 6), z=z))
            return f"ChArUco sheet laid under the camera (on a {z:g} mm block)" if z else \
                "ChArUco sheet laid under the camera"
        if what == "charuco_off":
            w.remove("charuco")
            return "ChArUco sheet taken away"
        if what == "board":
            board = self.m.boards.current()
            if board is None:
                raise Refused(Refusal("No board", "Import the board first (Board tab)"))
            lay = (self.m.bed or {}).get("layout", {}) or {}
            x0, y0 = kw.get("xy") or lay.get("pcb_area_xy", [hx, hy])
            side = kw.get("side", "B")
            copy = str(kw.get("copy", "1"))
            pose = sim.BoardPose(x0 + random.uniform(-2, 2), y0 + random.uniform(-2, 2), random.uniform(-1.5, 1.5),
                                 side, board.thickness)
            for L in sim.board_layers(w, board, pose, copy=copy):
                w.add(L)
            self.sim_board_poses = getattr(self, "sim_board_poses", {})
            self.sim_board_poses[copy] = pose
            if copy == "1":
                self.sim_board_pose = pose
            return f"{board.name} laid on the bed, {side} side up (a little off the plan, like a real board)"
        if what == "strip":
            slot = int(kw.get("slot", 3))
            h = float(kw.get("height", 1.35))                      # 0805 (package library); tape on the label
            floor = float(((self.m.bed or {}).get("feed_tray") or {}).get("tape_floor_mm") or 0.0)
            w.add(sim.strip_layer(w, sim.Strip(slot=slot, z_top=floor + h, part_height=h)))
            return f"A strip of 0805 parts laid in slot {slot}"
        if what == "tray":                                          # the operator put a new tray on the bed
            n = int(kw["n"])
            t = self.m.feeders.trays[n]
            if t.plan_xy is None:
                raise Refused(Refusal(f"Tray {n} has no planned place", "Add it with a place on the bed"))
            if w.trays is not None:
                w.trays[n] = t
            xy = np.asarray(t.plan_xy, float) + [random.uniform(-1, 1), random.uniform(-1, 1)]
            w.draw_tray(n, xy, t.plan_dir, t.marker_id)
            return f"Tray {n} put on the bed near its planned place"
        if what == "strip_in":                                      # a strip of the tray's line
            n = int(kw["n"])
            st = self.m.feeder_state.get(n)
            pk = self.m.boards.library().packages.get(st.get("package") or "0805")
            t = self.m.feeders.trays[n]
            h = float(pk.height or 1.0) if pk else 1.35
            body = tuple(pk.body) if pk else (2.0, 1.25)
            pitch = float((pk.tape or {}).get("pitch", 4.0)) if pk else 4.0
            if t.type in ("matrix", "stick"):                       # rows of a tray, or a stick's pick point
                for r, us, v in t.cells():
                    w.add(sim.strip_layer(w, sim.Strip(
                        slot=n, z_top=t.tape_floor_mm + h, part_height=h, package_body=body, pitch=t.pitch_u,
                        first_u=us[0], pockets=[sim.Pocket() for _ in us], v_off=v, holes=False, tape="tray",
                        tape_width=t.pitch_v - 0.6 if t.type == "matrix" else 5.0, stick=t.type == "stick",
                        stock=int(kw.get("stock", 30)))))
                return f"Tray {n} filled with {st.get('value', '')} {pk.id if pk else ''}"
            npk = int((t.zone_mm - 4.0) // pitch)
            noz = sim.nozzle_of(g)
            if noz is not None:                                    # a new strip: the old one goes, with its parts
                c, u, v = w.slot_frame(n)

                def in_zone(pl):                                   # lying in this tray's tape, not on the board
                    d = np.array([pl.x, pl.y]) - c
                    return t.zone_u[0] - 5 <= float(d @ u) <= t.zone_u[1] + 5 and abs(float(d @ v)) <= 8.0
                old = [pl for pl in noz.placed if in_zone(pl)]
                for pl in old:
                    w.remove(f"placed_{pl.uid}")
                noz.placed = [pl for pl in noz.placed if pl not in old]
            w.add(sim.strip_layer(w, sim.Strip(slot=n, z_top=t.tape_floor_mm + h, part_height=h, package_body=body,
                                               pitch=pitch, pockets=[sim.Pocket() for _ in range(npk)])))
            return f"A strip of {st.get('value', '')} {pk.id if pk else '0805'} laid in tray {n}"
        if what == "mark_sheet":
            noz = sim.nozzle_of(g)
            if noz is None:
                raise Refused(Refusal("This simulator profile has no nozzle", "Add sim.nozzle_offset_mm"))
            x, y = noz.tip_xy()
            noz.lay_mark_sheet(x - 2.0, y - 25.0, float(kw.get("z", 0.1)), size=(14.0, 70.0))   # fits between labels
            return "A sheet of paper (0.1 mm) laid under the nozzle, with ink on the needle"
        raise ValueError(f"unknown simulator prop {what}")


def _inside_polygon(p, poly) -> bool:
    import cv2
    return cv2.pointPolygonTest(np.asarray(poly, np.float32).reshape(-1, 1, 2), (float(p[0]), float(p[1])), False) >= 0
