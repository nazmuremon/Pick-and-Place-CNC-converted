"""The machine's job and feeder operations (Phase E, docs/13 §4.4.1, §4.7): the plan, start / pause / resume,
the operator's choices for a held part, refills, the trays (Add tray, Remove tray, load a line) and the shared
placed state (one per board, also used by the hand-assembly view, §4.15)."""
from __future__ import annotations

import json
import os
import shutil
import time
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from .calibration import chain
from .calibration.nozzle import NozzleOffset
from .feeders.loads import line_key
from .job.planner import make_plan, tip_reach_fn
from .job.runner import DONE, Job, JobRunner
from .motion.state import Refusal, Refused

if TYPE_CHECKING:
    from .machine import Machine


class PlacedStore:
    """Which parts of a board are on it, by the machine or by hand (docs/13 §4.15: one placed state, on the server,
    shared by every browser). data/boards/<board>/placed.json."""

    def __init__(self, root: Path) -> None:
        self.root = root

    def _path(self, board: str) -> Path:
        return self.root / board / "placed.json"

    def get(self, board: str) -> dict[str, dict]:
        p = self._path(board)
        try:
            return json.loads(p.read_text()) if p.is_file() else {}
        except json.JSONDecodeError:
            return {}

    def _put(self, board: str, d: dict) -> None:
        p = self._path(board)
        p.parent.mkdir(parents=True, exist_ok=True)
        tmp = p.with_suffix(".tmp")
        tmp.write_text(json.dumps(d, indent=1))
        os.replace(tmp, p)

    def mark(self, board: str, ref: str, by: str, placed: bool = True) -> None:
        d = self.get(board)
        if placed:
            d[ref] = {"by": by, "t": time.strftime("%Y-%m-%d %H:%M:%S")}
        else:
            d.pop(ref, None)
        self._put(board, d)

    def clear(self, board: str) -> None:
        self._put(board, {})


def placed_key(board: str, copy: str = "1") -> str:
    """The placed state of one board of a panel: copy 1 = the board's own name (the hand view uses it too)."""
    return board if str(copy) == "1" else f"{board}#{copy}"


class JobOps:
    def __init__(self, m: Machine) -> None:
        self.m = m
        self.root = m.data / "jobs"
        self.job: Job | None = None
        self.runner: JobRunner | None = None
        self._resume_last()

    # ---------------------------------------------------------------- the job on disk
    def _resume_last(self) -> None:
        """After a crash or a restart: the last job that was not finished comes back, paused."""
        states = sorted(self.root.glob("*/*/state.json"), key=lambda p: p.stat().st_mtime)
        for p in reversed(states):
            try:
                j = Job.load(p)
            except (OSError, ValueError, TypeError, KeyError):
                continue
            if j.status in ("running", "paused", "ready"):
                if j.status == "running":
                    held = next((x for x in j.parts if x.state in ("picking", "picked", "placing")), None)
                    j.status = "paused"
                    j.pause = ({"kind": "held", "why": f"The program stopped while {held.ref} was on the way",
                                "todo": "Look at the nozzle. Choose: place it, pick again, or drop it over the tape",
                                "ref": held.ref} if held else
                               {"kind": "stopped", "why": "The program stopped during the job", "todo": "Resume"})
                    j.note("resumed after the program stopped")
                    j.save()
                self.job = j
            return

    def after_change(self, job: Job) -> None:
        pass                                                       # the page polls the status

    # ---------------------------------------------------------------- feeders
    def tip_offset(self) -> np.ndarray | None:
        v = chain.value(self.m, "nozzle_offset")
        if v is None:
            est = self.m.setup.offset_estimate()
            return None if est is None else np.asarray(est.axis, float)
        return np.asarray(NozzleOffset.from_dict(v).axis, float)

    def trays_view(self) -> list[dict]:
        m = self.m
        frames = m.setup.slot_frames()
        surveyed = chain.value(m, "bed_survey")
        tip = self.tip_offset()
        out = []
        for n, t in sorted(m.feeders.trays.items()):
            st = m.feeder_state.get(n)
            reach = "unknown"
            if n in frames and tip is not None:
                reach = m.feeders.reach(t, frames[n], m.limits.travel, tip)
            pitch = 4.0
            if st.get("package"):
                pk = m.boards.library().packages.get(st["package"])
                if pk is not None and pk.tape:
                    pitch = float(pk.tape.get("pitch", 4.0))
            out.append({"number": n, "marker": t.marker_id, "type": t.type, "tape_w": t.tape_w, "kind": t.kind,
                        "zone_mm": t.zone_mm, "tape_floor_mm": t.tape_floor_mm, "plan_xy": t.plan_xy,
                        "measured": bool(surveyed and t.name in surveyed.get("labels", {})), "reach": reach,
                        "line": st.get("line"), "confirmed": bool(st.get("template")), "used": len(st.get("used", [])),
                        "capacity": t.capacity(pitch), "empty": bool(st.get("empty"))})
        return out

    def reachable_trays(self) -> list:
        return [self.m.feeders.trays[v["number"]] for v in self.trays_view() if v["reach"] == "reachable"]

    def add_tray(self, number: int, tape_w: int = 8, kind: str = "paper", zone_mm: float = 60.0,
                 type_: str = "holder", x: float | None = None, y: float | None = None, bank: str = "back",
                 **grid) -> dict:
        """grid: a matrix tray / a stick: rows, cols, pitch_u, pitch_v, first_u, tape_floor_mm, top_mm."""
        m = self.m
        plan = None if x is None or y is None else (float(x), float(y))
        floor = grid.pop("tape_floor_mm", None)
        try:
            t = m.feeders.add(int(number), int(tape_w), kind, float(zone_mm), type_, plan, tape_floor_mm=floor, **grid)
        except ValueError as err:
            return {"ok": False, "why": str(err)}
        m.markers = m._markers()
        files = {}
        out = m.data / "trays"
        try:
            if type_ in ("matrix", "stick"):
                files["pdf"] = str(write_label_pdf(t, out / f"label_{t.number}.pdf", m.feeders.dictionary))
            elif type_ == "holder":
                from .feeders.traygen import make_holder_stl
                p = make_holder_stl(t.number, t.marker_id, t.tape_w, t.kind, out / f"holder_{t.number}_{t.tape_w}mm_"
                                    f"{t.kind}.stl", m.feeders.dictionary)
                files["stl"] = str(p)
            else:
                files["pdf"] = str(write_label_pdf(t, out / f"label_{t.number}.pdf", m.feeders.dictionary))
        except ImportError as err:
            m.say(f"Tray {t.number}: the STL needs the 'trays' extra (manifold3d, trimesh): {err}", "warn", "feeders")
        m.say(f"Tray {t.number} added: marker {t.marker_id} ({m.feeders.dictionary}), {t.tape_w} mm {t.kind} tape. "
              + ("Print it, put it on the bed at the planned place, then Survey (Setup, step 8)" if files else
                 "Put it on the bed, then Survey"), kind="feeders")
        return {"ok": True, "tray": t.number, "marker": t.marker_id, "files": files}

    def remove_tray(self, number: int) -> dict:
        m = self.m
        if int(number) not in m.feeders.trays:
            return {"ok": False, "why": f"No tray {number}"}
        if self.job is not None and self.job.status in ("running",):
            return {"ok": False, "why": "A job is running", "todo": "Pause or finish it first"}
        t = m.feeders.remove(int(number))
        m.feeder_state.unload(int(number))
        m.markers = m._markers()
        m.say(f"Tray {number} removed: marker {t.marker_id} is free again", kind="feeders")
        return {"ok": True}

    def load_tray(self, number: int, line: str, scanned: str = "") -> dict:
        """A strip of `line` (value|package) goes into the tray. Then: look at it and confirm the first part."""
        m = self.m
        if int(number) not in m.feeders.trays:
            return {"ok": False, "why": f"No tray {number}"}
        if "|" not in line:
            return {"ok": False, "why": "A line is 'value|package'"}
        value, package = line.split("|", 1)
        m.feeder_state.load_line(int(number), line, value, package, scanned)
        m.say(f"Tray {number}: {value} {package} loaded. Look at it and confirm the first part", kind="feeders")
        return {"ok": True}

    def refilled(self, number: int) -> dict:
        self.m.feeder_state.refill(int(number))
        self.m.say(f"Tray {number} refilled (a new strip of the same line)", kind="feeders")
        if self.job is not None and self.job.status == "paused" and self.job.pause.get("kind") == "refill":
            self.job.pause = {"kind": "stopped", "why": f"Tray {number} refilled", "todo": "Resume"}
            self.job.save()
        return {"ok": True}

    # ---------------------------------------------------------------- the plan
    def plan(self):
        m = self.m
        board, reg = m.vision.registration()
        regs = m.vision.registrations()                            # a panel: every registered copy
        if board is None or not regs:
            raise Refused(Refusal("The board is not registered", "Setup, step 9"))
        tip = self.tip_offset()
        if tip is None:
            raise Refused(Refusal("The nozzle offset is not measured", "Setup, step 5"))
        from .feeders.packages import annotate
        lib = m.boards.library()
        classes = annotate(board, lib)
        placed = {c: set(m.placed.get(placed_key(board.name, c))) for c, _ in regs}
        reach = tip_reach_fn(m.limits.travel, tip)

        def capacity(tray, line):
            return tray.capacity(line.pitch)
        policy = m.profile.job.use_bottom_camera
        bottom = "none" if chain.value(m, "bottom_camera") is None or policy == "never" else policy
        nset = m.caps.later.get("nozzles")
        nozzle_for = nset.select if nset is not None and len(m.profile.nozzles.items) > 1 else None
        return make_plan(board, classes, lib, regs, reach, placed, self.reachable_trays(), capacity, bottom=bottom,
                         nozzle_for=nozzle_for, nozzle_now=m.nozzle_current())

    def plan_view(self) -> dict:
        try:
            p = self.plan()
        except Refused as err:
            return {"ok": False, "why": err.refusal.why, "todo": err.refusal.todo}
        return {"ok": True, "summary": p.summary(), "skipped": p.skipped,
                "loads": [{"index": ld.index, "trays": {str(k): v for k, v in ld.trays.items()}} for ld in p.loads],
                "lines": [{"key": ln.key, "value": ln.value, "package": ln.package, "count": ln.count}
                          for ln in p.lines], "trays": self.trays_view()}

    def stale_inputs(self, mode: str) -> tuple[list[str], list[str]]:
        """(missing, stale) results the job needs."""
        m = self.m
        need = ["camera_head"] if mode == "dry" else ["camera_head", "nozzle_offset", "bed_survey"]
        if mode == "real":
            need.append("first_part")
            if m.caps.rotation.kind == "servo_pwm":
                need.append("rotation_table")
        missing, stale = [], []
        for k in need:
            st = chain.status(m, k)["state"]
            (missing if st == "missing" else stale if st == "stale" else []).append(chain.CHAIN[k].title)
        return missing, stale

    # ---------------------------------------------------------------- start / resume / stop
    def start(self, mode: str = "real", n_picks: int = 100, override: bool = False,
              refs: list[str] | None = None) -> dict:
        """refs: only these parts (the others wait for another job); None = the whole plan."""
        m = self.m
        if mode not in ("real", "dry", "pick_test"):
            return {"ok": False, "why": f"Unknown mode {mode}"}
        if self.job is not None and self.job.status in ("running", "paused"):
            return {"ok": False, "why": f"Job {self.job.id} is not finished", "todo": "Resume it, or Abort it"}
        r = m._check("auto_z_down" if mode != "dry" else "goto_xy")
        if r is not None:
            return {"ok": False, "why": r.why, "todo": r.todo}
        missing, stale = self.stale_inputs(mode)
        if missing:
            return {"ok": False, "why": "Not measured yet: " + ", ".join(missing), "todo": "Do these set-up steps first"}
        if stale and not override:
            return {"ok": False, "why": "Stale: " + ", ".join(stale),
                    "todo": "Measure them again (Setup), or start anyway (written into the report)"}
        try:
            if mode == "pick_test":
                board, reg = m.vision.registration()
                trays = [n for n in sorted(m.feeders.trays) if m.feeder_state.template(n) is not None]
                if not trays:
                    return {"ok": False, "why": "No tray is loaded and confirmed", "todo": "Feeders tab: load a tray"}
                name = board.name if board is not None else "pick_test"
                job = self._new_job(name, reg.side if reg else "-", mode, [], [],
                                    [{"index": 1, "trays": {str(t): m.feeder_state.get(t).get("line") for t in trays}}])
                job.n_picks = int(n_picks)
            else:
                p = self.plan()
                if refs:
                    keep = set(refs)
                    p.parts = [x for x in p.parts if x.ref in keep]
                if not p.parts:
                    return {"ok": False, "why": "Nothing to place", "todo": "See why in the plan (skipped parts)"}
                job = self._new_job(p.board, p.side, mode, p.parts, p.skipped,
                                    [{"index": ld.index, "trays": {str(k): v for k, v in ld.trays.items()}}
                                     for ld in p.loads])
        except Refused as err:
            return {"ok": False, "why": err.refusal.why, "todo": err.refusal.todo}
        job.overrides = stale
        job.save()
        self.job = job
        self.runner = JobRunner(m, job)
        m.say(f"Job {job.id} starts: {mode}, {len(job.parts) or job.n_picks} {'parts' if job.parts else 'picks'}"
              + (f" (stale, by your choice: {', '.join(stale)})" if stale else ""), kind="job")
        return m._submit("Job", self.runner.run)

    def _new_job(self, board: str, side: str, mode: str, parts, skipped, loads) -> Job:
        stamp = time.strftime("%Y%m%d-%H%M%S")
        folder = self.root / board / stamp
        n = 2
        while folder.exists():
            folder = self.root / board / f"{stamp}-{n}"
            n += 1
        return Job(folder.name, str(folder), board, side, mode, parts, skipped, loads,
                   created=time.strftime("%Y-%m-%d %H:%M:%S"))

    def resume(self) -> dict:
        j = self.job
        if j is None or j.status != "paused":
            return {"ok": False, "why": "No paused job"}
        if j.pause.get("kind") == "held":
            return {"ok": False, "why": "A part may be on the nozzle", "todo": "Choose: place it, pick again, or drop it"}
        nxt = next((p for p in j.parts if p.state not in DONE and p.nozzle), None)
        if nxt is not None and nxt.nozzle != self.m.nozzle_current():
            return {"ok": False, "why": f"The next part needs nozzle {nxt.nozzle}", "todo": "Put it on, then 'Nozzle changed'"}
        if j.mode != "dry" and chain.status(self.m, "nozzle_offset")["state"] != "valid":
            return {"ok": False, "why": f"Nozzle {self.m.nozzle_current()}: its offset is not measured",
                    "todo": "Setup, step 5, then Resume"}
        r = self.m._check("auto_z_down" if j.mode != "dry" else "goto_xy")
        if r is not None:
            return {"ok": False, "why": r.why, "todo": r.todo}
        self.runner = JobRunner(self.m, j)
        return self.m._submit("Job", self.runner.run)

    def nozzle_changed(self, nid: str) -> dict:
        """The operator put another nozzle on (the job asked for it)."""
        r = self.m.set_nozzle(nid)
        if not r["ok"]:
            return r
        j = self.job
        if j is not None and j.status == "paused" and j.pause.get("kind") == "nozzle":
            if chain.status(self.m, "nozzle_offset")["state"] != "valid":
                j.pause = {"kind": "nozzle", "why": f"Nozzle {nid}: its offset is not measured",
                           "todo": "Setup, step 5 (and 5b with a bottom camera), then Resume", "nozzle": nid}
            else:
                j.pause = {"kind": "stopped", "why": f"Nozzle {nid} is on", "todo": "Resume"}
            j.save()
        return {"ok": True}

    def answer(self, choice: str) -> dict:
        """The operator's choice for a held part: place | repick (the nozzle is empty) | drop (over the tape)."""
        j = self.job
        if j is None or j.status != "paused" or j.pause.get("kind") != "held":
            return {"ok": False, "why": "No part waits for a choice"}
        runner = self.runner or JobRunner(self.m, j)
        self.runner = runner
        fn = {"place": runner.place_held, "repick": runner.nozzle_empty, "drop": runner.drop_held}.get(choice)
        if fn is None:
            return {"ok": False, "why": f"Unknown choice {choice}"}

        def go():
            fn()
            j.save()
            from .job.report import write_report
            write_report(j)
        return self.m._submit("Held part", go)

    def abort(self) -> dict:
        j = self.job
        if j is None or j.status not in ("paused", "ready"):
            return {"ok": False, "why": "Only a paused job can be aborted", "todo": "Press STOP first"}
        if j.pause.get("kind") == "held":
            return {"ok": False, "why": "A part may be on the nozzle", "todo": "Place it, or drop it over the tape first"}
        j.status, j.ended = "aborted", time.time()
        j.note("aborted by the operator")
        j.save()
        from .job.report import write_report
        write_report(j)
        self.m.say(f"Job {j.id} aborted", "warn", "job")
        self.retention()
        return {"ok": True}

    def retention(self) -> None:
        """Keep the photos of the newest N jobs; older jobs keep their report and state only (docs/13 §4.11)."""
        keep = self.m.profile.job.keep_photos_jobs
        folders = sorted((p.parent for p in self.root.glob("*/*/state.json")), key=lambda p: p.stat().st_mtime)
        for f in folders[:-keep]:
            if (f / "photos").is_dir():
                shutil.rmtree(f / "photos", ignore_errors=True)

    def status(self) -> dict | None:
        j = self.job
        if j is None:
            return None
        d = j.summary()
        d["parts_list"] = [{"ref": p.ref if p.copy == "1" else f"{p.ref} (copy {p.copy})", "state": p.state,
                            "reason": p.reason, "x": round(p.x, 2), "y": round(p.y, 2), "tray": p.tray, "load": p.load}
                           for p in j.parts]
        if j.status in ("done", "aborted") and self.m.worker.current is None:
            pass
        return d

    def jobs_list(self, limit: int = 20) -> list[dict]:
        out = []
        for p in sorted(self.root.glob("*/*/state.json"), key=lambda p: p.stat().st_mtime, reverse=True)[:limit]:
            try:
                j = Job.load(p)
            except (OSError, ValueError, TypeError, KeyError):
                continue
            out.append({"id": j.id, "board": j.board, "mode": j.mode, "status": j.status, "created": j.created,
                        "counts": j.counts(), "report": str(Path(j.folder) / "report.html")})
        return out


def write_label_pdf(tray, path: Path, dictionary: str, dpi: int = 600) -> Path:
    """A paper slot label for a new tray (D13 / D14 geometry: 87 x 20 mm, 10 mm marker, number, 60 mm zone marks),
    with a 100 mm check bar. Print at 100 % ("actual size")."""
    import cv2
    from PIL import Image, ImageDraw
    ppm = dpi / 25.4
    page = Image.new("L", (int(210 * ppm), int(297 * ppm)), 255)
    d = ImageDraw.Draw(page)
    x0, y0 = int(20 * ppm), int(20 * ppm)
    d.rectangle([x0, y0, x0 + int(87 * ppm), y0 + int(20 * ppm)], outline=0, width=2)
    mk = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)),
                                       tray.marker_id, int(10 * ppm))
    page.paste(Image.fromarray(mk), (x0 + int(5 * ppm), y0 + int(5 * ppm)))
    d.text((x0 + int(17 * ppm), y0 + int(7 * ppm)), str(tray.number), fill=0)
    for u in (17.0 + 10.0, 17.0 + 10.0 + tray.zone_mm):                # the zone ends (marker centre at 10 mm)
        x = x0 + int(u * ppm)
        d.rectangle([x, y0 + int(2 * ppm), x + int(0.4 * ppm), y0 + int(18 * ppm)], fill=90)
    y = int(60 * ppm)
    d.rectangle([x0, y, x0 + int(100 * ppm), y + int(2 * ppm)], fill=0)
    d.text((x0, y + int(4 * ppm)), f"Tray {tray.number}, marker {tray.marker_id} ({dictionary}). 100 mm bar: print at "
           "100 % and measure it.", fill=0)
    path.parent.mkdir(parents=True, exist_ok=True)
    page.save(path, "PDF", resolution=dpi)
    return path
