"""The job runner (docs/13 §4.7, §4.8).

Every part goes planned -> picking -> picked -> placing -> placed -> verified, or skipped / failed with a reason.
The job's state is written to `jobs/<board>/<time>/state.json` after every change, so a crash or a power cut
resumes at the right part. A pause always says why and what to do: a refill (the tray is empty), a load (the next
load's lines), a part held on the nozzle (STOP, an unclear pick check: the pump stays on until the operator chooses
"place it", "pick again" or "drop over the tape"), a HOME drift, or an error.

Modes: real (pick and place), dry (camera only: the pocket and the pads are looked at, nothing moves down), and
pick_test (N picks, each part put back into its pocket: the pick success rate with a photo of every miss).
"""
from __future__ import annotations

import json
import math
import os
import time
import traceback
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from ..motion.state import Refusal, Refused, Stopped
from .pickplace import PartHeld, Picked, PickPlace
from .planner import PlanPart

if TYPE_CHECKING:
    from ..machine import Machine

DONE = ("verified", "placed", "skipped", "failed")
ANGLE_MIN_PX = 15.0              # a part shorter than this in the picture: its angle is not checked (bench: 1206 yes, 0805 no)


@dataclass
class Job:
    id: str
    folder: str
    board: str
    side: str
    mode: str                                   # real | dry | pick_test
    parts: list[PlanPart]
    skipped: list[dict]
    loads: list[dict]                           # [{index, trays: {tray: line}}]
    status: str = "ready"                       # ready | running | paused | done | aborted
    pause: dict = field(default_factory=dict)   # {kind, why, todo, tray, ref}
    current_load: int = 1
    overrides: list[str] = field(default_factory=list)
    home_checks: list[dict] = field(default_factory=list)
    picks: list[dict] = field(default_factory=list)     # pick_test: one entry per pick
    n_picks: int = 0
    created: str = ""
    started: float = 0.0
    ended: float = 0.0
    log: list[str] = field(default_factory=list)

    # ---------------------------------------------------------------- file
    @property
    def path(self) -> Path:
        return Path(self.folder) / "state.json"

    def save(self) -> None:
        p = self.path
        p.parent.mkdir(parents=True, exist_ok=True)
        d = asdict(self)
        tmp = p.with_suffix(".tmp")
        tmp.write_text(json.dumps(d, indent=1, default=_num))
        os.replace(tmp, p)

    @classmethod
    def load(cls, path: Path) -> Job:
        d = json.loads(Path(path).read_text())
        d["parts"] = [PlanPart(**p) for p in d["parts"]]
        return cls(**d)

    # ---------------------------------------------------------------- views
    def counts(self) -> dict:
        out: dict[str, int] = {}
        for p in self.parts:
            out[p.state] = out.get(p.state, 0) + 1
        return out

    def summary(self) -> dict:
        c = self.counts()
        d = {"id": self.id, "board": self.board, "side": self.side, "mode": self.mode, "status": self.status,
             "pause": self.pause, "load": self.current_load, "loads": len(self.loads), "counts": c,
             "parts": len(self.parts), "done": sum(c.get(s, 0) for s in DONE), "overrides": self.overrides,
             "folder": self.folder, "report": str(Path(self.folder) / "report.html")}
        if self.mode == "pick_test":
            ok = sum(1 for p in self.picks if p["verdict"] == "picked")
            d["picks"] = {"done": len(self.picks), "of": self.n_picks, "picked": ok,
                          "rate": round(ok / len(self.picks), 3) if self.picks else None}
        return d

    def note(self, text: str) -> None:
        self.log.append(f"{time.strftime('%H:%M:%S')} {text}")


def _num(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


class JobRunner:
    def __init__(self, m: Machine, job: Job) -> None:
        self.m, self.job = m, job
        self.pp: PickPlace | None = None
        self.board, self.reg = m.vision.registration()

    # ---------------------------------------------------------------- the loop
    def run(self) -> None:
        j, m = self.job, self.m
        j.status, j.pause = "running", {}
        j.started = j.started or time.time()
        j.save()
        try:
            if j.mode != "dry":
                self.pp = PickPlace(m, self.board, self.reg)
            if j.mode == "pick_test":
                self._pick_test()
            else:
                self._parts()
            if j.status == "running":
                self._home_check(final=True)
            if j.status == "running":
                j.status, j.ended = "done", time.time()
                m.say(f"Job done: {self._count_line()}", kind="job")
        except Stopped:
            self._after_stop()
        except PartHeld as err:
            self._pause("held", str(err), "The pump stays on. Choose: place it, pick again, or drop it over the tape",
                        ref=self._current_ref())
        except Refused as err:
            self._pause("error", err.refusal.why, err.refusal.todo, ref=self._current_ref())
        except Exception as err:                                  # noqa: BLE001 - reported, the job pauses
            self._pause("error", f"{err!r}", "Look at the log; Resume when it is fixed", ref=self._current_ref())
            m.say(f"Job error: {err!r}", "error", "bug", trace=traceback.format_exc(limit=6))
        finally:
            j.save()
            from .report import write_report
            write_report(j)
            m.jobs.after_change(j)

    def _current_ref(self) -> str | None:
        return next((p.ref for p in self.job.parts if p.state in ("picking", "picked", "placing")), None)

    def _pause(self, kind: str, why: str, todo: str, tray: int | None = None, ref: str | None = None) -> None:
        self.job.status = "paused"
        self.job.pause = {"kind": kind, "why": why, "todo": todo, "tray": tray, "ref": ref}
        self.job.note(f"paused ({kind}): {why}")
        self.m.say(f"Job paused: {why}. {todo}", "warn", "job")

    def _after_stop(self) -> None:
        """STOP: the nozzle lifts; a part on the nozzle stays held until the operator chooses."""
        m = self.m
        m.worker.cancel.clear()
        held = next((p for p in self.job.parts if p.state in ("picking", "picked", "placing")), None)
        try:
            if self.pp is not None and m.safety.lock is None:
                self.pp.lift()
        except Refused:
            pass
        if held is not None and bool(getattr(m.caps.vacuum, "pump_on", False)):
            self._pause("held", f"Stopped by you with {held.ref} on the way",
                        "Choose: place it, pick again (the pump goes off over the tape), or drop it over the tape",
                        ref=held.ref)
        else:
            if held is not None:
                held.state = "planned"
            self._pause("stopped", "Stopped by you", "Resume, or Abort")

    def _count_line(self) -> str:
        c = self.job.counts()
        return ", ".join(f"{v} {k}" for k, v in sorted(c.items()))

    # ---------------------------------------------------------------- parts
    def _parts(self) -> None:
        j, m = self.job, self.m
        since_check = 0
        for part in j.parts:
            if part.state in DONE:
                continue
            if m.worker.cancel.is_set():
                raise Stopped()
            if part.load != j.current_load:
                if any(p.load == j.current_load and p.state not in DONE for p in j.parts):
                    continue
                j.current_load = part.load
                if not self._load_ready(part.load):
                    return
            if part.nozzle and part.nozzle != m.nozzle_current():
                self._pause("nozzle", f"The next parts need nozzle {part.nozzle} (now: {m.nozzle_current()})",
                            f"Lift, put on nozzle {part.nozzle}, then 'Nozzle changed'", ref=part.ref)
                self.job.pause["nozzle"] = part.nozzle
                return
            if part.state in ("picking", "placing"):
                self._pause("held", f"{part.ref} was on the way when the job stopped",
                            "Look at the nozzle. Choose: place it, pick again, or drop it over the tape", ref=part.ref)
                return
            if j.mode == "dry":
                self._dry_one(part)
            else:
                if not self._real_one(part):
                    return
            j.save()
            m.jobs.after_change(j)
            since_check += 1
            every = m.profile.job.home_check_every
            if every and since_check >= every:
                since_check = 0
                if not self._home_check():
                    return

    def _load_ready(self, index: int) -> bool:
        """Every line the load's remaining parts need sits in a tray with a confirmed first part (the load sheet's
        tray, or any other: the operator may put a line elsewhere)."""
        load = next((ld for ld in self.job.loads if ld["index"] == index), None)
        if load is None:
            return True
        st = self.m.feeder_state
        need = {p.line for p in self.job.parts if p.load == index and p.state not in DONE}
        have = {st.get(int(t)).get("line") for t in st.d if st.template(int(t)) is not None}
        suggest = {line: t for t, line in load["trays"].items()}
        missing = [f"tray {suggest.get(line, '?')}: {line}" for line in sorted(need - have)]
        if missing:
            self._pause("load", f"Load {index} of {len(self.job.loads)}: put these strips in and confirm their first "
                        f"part: {'; '.join(missing)}", "Feeders tab: load each tray, then Resume")
            return False
        return True

    def _tray_for(self, part: PlanPart) -> int | None:
        st = self.m.feeder_state
        trays = [t for t in st.trays_for(part.line) if not st.get(t).get("empty") and t in self.m.feeders.trays]
        return trays[0] if trays else None

    def _slot(self, tray: int, package: str):
        s, pk = self.m.setup.slot_setup(tray, package)
        if s.template is None:
            raise Refused(Refusal(f"Tray {tray}: its first part is not confirmed", "Feeders tab: look at it, say Yes"))
        return s, pk

    def _pick_any(self, tray: int, part: PlanPart, folder: Path, lifted):
        """Pick from the tray: a strip / stick in one go, a matrix tray row by row. None = the tray is empty."""
        setups = self.m.setup.slot_setups(tray, part.package)
        if setups[0].template is None:
            raise Refused(Refusal(f"Tray {tray}: its first part is not confirmed", "Feeders tab: look at it, say Yes"))
        st = self.m.feeder_state
        for s in setups:
            try:
                picked, s_place = self.pp.pick(s, part.height, part.symmetric, part.angle, folder, on_lifted=lifted)
            except Refused as err:
                if "no part left" in err.refusal.why or "has no part" in err.refusal.why:
                    continue                                     # the next row of a matrix tray
                raise
            for u in s.used:                                     # what is picked stays picked (rows: row * 1000 + k)
                key = s.row * 1000 + int(u)
                if key not in st.get(tray).get("used", []):
                    st.use(tray, key)
            return picked, s_place, s
        return None

    def _real_one(self, part: PlanPart) -> bool:
        """One part. False = the job paused (refill, a held part)."""
        j, m, pp = self.job, self.m, self.pp
        cfg = m.profile.job
        while True:
            tray = self._tray_for(part)
            if tray is None:
                trays = m.feeder_state.trays_for(part.line)
                self._pause("refill", f"No part of {part.line} left" + (f" in tray {trays[0]}" if trays else ""),
                            f"Refill tray {trays[0]} with {part.value} {part.package}, then 'Refilled'"
                            if trays else "Load this line into a tray (Feeders tab)",
                            tray=trays[0] if trays else None, ref=part.ref)
                return False
            part.state, part.tray = "picking", tray
            j.save()
            folder = Path(j.folder) / "photos" / part.ref

            def lifted(info, part=part):
                part.pick = info
                j.save()
            got = self._pick_any(tray, part, folder, lifted)
            if got is None:
                m.feeder_state.mark_empty(tray)
                part.state = "planned"
                j.note(f"tray {tray} empty")
                continue                                         # a spare tray of the same line, or a refill
            picked, s_place, _s = got
            part.attempts += 1
            if picked.check == "missed":
                part.state = "planned"
                part.result.setdefault("misses", []).append(picked.photos)
                j.note(f"{part.ref}: pick missed (tray {tray})")
                if part.attempts >= cfg.max_pick_attempts:
                    part.state, part.reason = "failed", f"missed {part.attempts} picks (photos in the report)"
                    return True
                continue
            part.state = "picked"
            part.pick = {"tray": tray, "xy": [float(v) for v in picked.part_xy], "angle": picked.part_angle,
                         "servo_pick": picked.servo_pick, "servo_place": s_place, "pocket_u": picked.pocket_u}
            j.save()
            return self._place(part, picked, s_place, folder)

    def _place(self, part: PlanPart, picked: Picked, s_place, folder: Path) -> bool:
        j, m, pp = self.job, self.m, self.pp
        cfg = m.profile.job
        part.state = "placing"
        j.save()
        self._use_copy(part)
        res = pp.place(part.ref, picked, s_place, part.height, folder, use_bottom=part.bottom,
                       symmetric=part.symmetric)
        part.result.update({"photos": {**picked.photos, **res.photos}, "turn": res.turn, "why": res.why,
                            "error": None if res.error is None else [round(float(v), 3) for v in res.error],
                            "angle_error": None if res.angle_error is None else round(float(res.angle_error), 2)})
        from ..machine_job import placed_key
        m.placed.mark(placed_key(self.board.name, part.copy), part.ref, "machine")
        if res.error is None:
            part.state, part.reason = "failed", res.why or "not seen at the place"
        else:
            e = float(np.hypot(*res.error))
            # the angle is checked only when the part is long enough in the picture to measure it (the bench camera:
            # an 0805 is 10 pixels long, its angle reads only to about ±3°)
            k = pp.model.px_per_mm(m.vision.board_top_z(self.board) + part.height)
            angle_ok = res.angle_error is None or max(picked.body) * k < ANGLE_MIN_PX \
                or abs(res.angle_error) <= cfg.angle_tolerance_deg
            if max(picked.body) * k < ANGLE_MIN_PX:
                part.result["angle_note"] = "too small in the picture to check its angle"
            if e <= cfg.place_tolerance_mm and angle_ok:
                part.state = "verified"
            else:
                part.state = "failed"
                part.reason = f"placed {e:.2f} mm" + (f", {res.angle_error:+.1f}°" if res.angle_error is not None else "") \
                    + " from its place: check it"
        m.say(f"   {part.ref}: {part.state}" + (f" ({part.reason})" if part.reason else ""), "debug", "job")
        return True

    def _use_copy(self, part: PlanPart) -> None:
        """A panel: the registration of the part's own board."""
        rg = self.reg
        if part.copy != "1":
            _, rg = self.m.vision.registration(part.copy)
            if rg is None:
                raise Refused(Refusal(f"Board copy {part.copy} is not registered (any more)", "Register it (Board tab)"))
        self.reg_now = rg
        if self.pp is not None:
            self.pp.reg = rg

    def _dry_one(self, part: PlanPart) -> None:
        from . import dryrun
        from ..machine_vision import MachineRig
        tray = self._tray_for(part)
        if tray is None:
            part.state, part.reason = "skipped", "no tray holds this line (dry run: not stopped)"
            return
        s, _ = self._slot(tray, part.package)
        dp = dryrun.DryPart(part.ref, tray)
        z_board = self.m.vision.board_top_z(self.board)
        self._use_copy(part)
        dryrun._one(MachineRig(self.m, "home"), self.m.vision.model(), self.board, self.reg_now, s, dp, z_board,
                    lambda img, f: True, lambda t: None, None)
        for u in s.used:
            if u not in self.m.feeder_state.get(tray).get("used", []):
                self.m.feeder_state.use(tray, u)
        part.tray = tray
        if dp.why:
            part.state, part.reason = "failed", dp.why
        else:
            part.state = "verified" if dp.pads_seen else "failed"
            part.reason = "" if dp.pads_seen else "pads not seen at the place"
            part.result = {"pick_xy": dp.pick_xy, "place_xy": dp.place_xy, "dry": True}

    # ---------------------------------------------------------------- operator choices for a held part
    def place_held(self) -> None:
        part = next((p for p in self.job.parts if p.state in ("picked", "placing", "picking")), None)
        if part is None or not part.pick:
            raise Refused(Refusal("No held part with a known pick", "Use 'drop over the tape' or 'pick again'"))
        self.pp = self.pp or PickPlace(self.m, self.board, self.reg)
        s, _ = self._slot(part.pick["tray"], part.package)
        pk = part.pick
        picked = Picked(pk["tray"], pk["pocket_u"], np.asarray(pk["xy"]), pk["angle"], pk["servo_pick"], "picked", {},
                        s.template, s.u, s.v, s.spec.body)
        self._place(part, picked, pk["servo_place"], Path(self.job.folder) / "photos" / part.ref)
        self.job.status, self.job.pause = "paused", {"kind": "stopped", "why": f"{part.ref} placed", "todo": "Resume"}

    def drop_held(self) -> None:
        """Over the tape: the tip over the part's own pocket (or the tray's zone), just above it, pump off."""
        m = self.m
        part = next((p for p in self.job.parts if p.state in ("picked", "placing", "picking")), None)
        self.pp = self.pp or PickPlace(m, self.board, self.reg)
        tray = (part.pick or {}).get("tray") if part is not None else None
        tray = tray or (part.tray if part is not None else None)
        if tray is None:
            raise Refused(Refusal("Not known which tray the part came from", "Lift by hand and take it off"))
        s, _ = self._slot(tray, part.package)
        xy = np.asarray(part.pick["xy"]) if part.pick else s.centre + s.u * sum(s.spec.u_range) / 2
        self.pp.tip_over(xy, self.pp.servo())
        self.pp.down(s.z_tape + 0.5, "tape")
        m.caps.vacuum.release()
        self.pp.lift()
        if part is not None:
            part.state, part.pick = "planned", {}
        self.job.status, self.job.pause = "paused", {"kind": "stopped", "why": "Dropped over the tape", "todo": "Resume"}

    def nozzle_empty(self) -> None:
        m = self.m
        if getattr(m.caps.vacuum, "pump_on", False):
            m.caps.vacuum.pump(False)
        for p in self.job.parts:
            if p.state in ("picking", "picked", "placing"):
                p.state, p.pick = "planned", {}
        self.job.status, self.job.pause = "paused", {"kind": "stopped", "why": "The nozzle is empty", "todo": "Resume"}

    # ---------------------------------------------------------------- HOME drift
    def _home_check(self, final: bool = False) -> bool:
        """HOME measured again with the camera from where Home was done: the drift says if steps were lost (K1)."""
        m = self.m
        view = m.session.home_view
        if m.profile.homing.method != "camera_marker" or view is None:
            return True
        from ..machine_vision import MachineRig
        from ..vision import markers as vmk
        rig = MachineRig(m, "home")
        if self.pp is not None:
            self.pp.lift()
        rig.move_to(*view)
        off, why = vmk.home_offset(m.markers, rig.frames(5), m.vision.model(), 0.0)
        if off is None:
            self.job.home_checks.append({"t": time.time(), "why": why})
            return True
        drift = np.asarray(rig.head(), float) + off                  # HOME is at (0, 0)
        d = float(np.hypot(*drift))
        self.job.home_checks.append({"t": time.time(), "drift": [round(float(v), 3) for v in drift], "mm": round(d, 3),
                                     "final": final})
        if d > m.profile.job.drift_limit_mm:
            self._pause("drift", f"HOME is {d:.2f} mm from where it should be: steps were lost",
                        "Home again (Machine tab), then Resume. Check the parts placed since the last check")
            return False
        return True

    # ---------------------------------------------------------------- pick test
    def _pick_test(self) -> None:
        """N picks; every part goes back into its own pocket. Each miss keeps its photos (docs/13 §8)."""
        j, m, pp = self.job, self.m, self.pp
        trays = [int(t) for t in j.loads[0]["trays"]] if j.loads else []
        if not trays:
            raise Refused(Refusal("No tray for the pick test", "Load at least one tray (Feeders tab)"))
        while len(j.picks) < j.n_picks:
            if m.worker.cancel.is_set():
                raise Stopped()
            free = [t for t in trays if not m.feeder_state.get(t).get("empty")]
            if not free:
                t0 = trays[0]
                self._pause("refill", "Every tray of the pick test is empty", f"Refill the trays, then 'Refilled'",
                            tray=t0)
                return
            tray = free[len(j.picks) % len(free)]
            st = m.feeder_state.get(tray)
            pk = m.boards.library().packages.get(st.get("package", "0805"))
            folder = Path(j.folder) / "photos" / f"pick_{len(j.picks) + 1:03d}"
            probe = PlanPart("pick test", st.get("value", ""), st.get("package", "0805"), st.get("line", ""), 0.0, 0.0,
                             0.0, float(pk.height or 1.0) if pk else 1.0, True)
            probe.angle = None
            got = self._pick_any(tray, probe, folder, None)
            if got is None:
                m.feeder_state.mark_empty(tray)
                continue
            picked, _, s = got
            rec = {"n": len(j.picks) + 1, "tray": tray, "verdict": picked.check, "photos": picked.photos}
            if picked.check == "picked":                          # back into its pocket, pressed in gently
                pp.tip_over(picked.part_xy, picked.servo_pick)
                # a stick's pick point holds the next part already: let it go just above, onto the tray
                pp.down(s.z_tape + 1.0 if s.stick else s.z_tape - pp.rules.press, "tape")
                m.caps.vacuum.release()
                pp.lift()
            else:
                rec["photos_kept"] = True
            j.picks.append(rec)
            if len(j.picks) % 10 == 0:
                j.save()
                m.jobs.after_change(j)
        ok = sum(1 for p in j.picks if p["verdict"] == "picked")
        m.say(f"Pick test: {ok} of {len(j.picks)} picked ({100 * ok / max(1, len(j.picks)):.0f} %)", kind="job")
