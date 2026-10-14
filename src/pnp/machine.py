"""The machine process: it owns the controller, the cameras and the safety state, and exposes ONE API to the page
(docs/13 §7). Long operations run one at a time on a worker thread, with a cancel point (STOP).

Every action returns {"ok": True} or {"ok": False, "why": ..., "todo": ...} at once; what happens later is told
through events (the page's log and status).
"""
from __future__ import annotations

import queue
import threading
import time
import traceback
from pathlib import Path
from typing import Callable

import yaml

from . import capabilities, paths
from . import sim as simulator
from .capabilities.base import CapabilityError
from .board.service import BoardService
from .calibration import chain, wizard
from .calibration.nozzle import RotationTable
from .calibration.store import CalibrationStore
from .capabilities.camera import build_camera
from .events import EventLog
from .machine_setup import SetupOps
from .machine_vision import CameraHoming, VisionOps
from .motion.controller import Controller
from .motion.grbl import ControllerError, GrblLink, LinkLost
from .motion.limits import Limits
from .motion.state import Lock, Refusal, Refused, SafetyState
from .motion.transport import open_port
from .profile import Profile, ProfileError, Watcher, diff, load, missing
from .session import Session, SessionStore
from .vision.markers import Markers

POSITION_TOL = 0.02          # mm: after a move the controller must report the target this closely


def _ok() -> dict:
    return {"ok": True}


def _no(why: str, todo: str = "") -> dict:
    return {"ok": False, "why": why, "todo": todo}


class Worker:
    """One long operation at a time. STOP sets `cancel`; operations check it between steps."""

    def __init__(self, on_error: Callable[[str, BaseException], None]) -> None:
        self.q: queue.Queue = queue.Queue()
        self.current: str | None = None
        self.started = 0.0
        self.cancel = threading.Event()
        self.on_error = on_error
        self.alive = True
        self.thread = threading.Thread(target=self._run, name="machine-worker", daemon=True)
        self.thread.start()

    def submit(self, name: str, fn: Callable, *args) -> bool:
        if self.current is not None or not self.q.empty():
            return False
        self.current, self.started = name, time.monotonic()      # busy from now on (no race with a 2nd click)
        self.cancel.clear()
        self.q.put((name, fn, args))
        return True

    def _run(self) -> None:
        while self.alive:
            try:
                name, fn, args = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                fn(*args)
            except BaseException as err:                         # noqa: BLE001 - every failure is reported
                self.on_error(name, err)
            finally:
                self.current = None

    def wait(self, timeout: float = 30.0) -> bool:
        """Tests: wait until the worker is free."""
        deadline = time.monotonic() + timeout
        while (self.current is not None or not self.q.empty()) and time.monotonic() < deadline:
            time.sleep(0.01)
        return self.current is None

    def stop(self) -> None:
        self.alive = False
        self.cancel.set()


class Machine:
    def __init__(self, profile_path: Path, *, sim: bool = False, cfg_dir: Path | None = None,
                 data_dir: Path | None = None, sim_speed: float | None = None, watch: bool = True,
                 seed: int = 0, sim_controller=None, sim_world=None) -> None:
        """sim_controller: an already running SimGrbl to reuse (tests: a program restart while the controller
        keeps running)."""
        self.profile_path = Path(profile_path)
        self.profile: Profile = load(self.profile_path)
        self.cfg_dir = cfg_dir or self.profile_path.parent.parent
        self.data = data_dir or paths.data_dir()
        self.data.mkdir(parents=True, exist_ok=True)
        self.sim_mode = sim
        self.events = EventLog(self.data / ("events_sim.jsonl" if sim else "events.jsonl"))
        self.events.context = self._context
        self.store = SessionStore(self.data / ("session_sim.yaml" if sim else "session.yaml"))
        old = self.store.load()
        self.session = old if old is not None and old.profile == self.profile.name else Session(self.profile.name)
        self.safety = SafetyState()
        self.limits = Limits(self.profile)
        self.profile_error: ProfileError | None = None
        self.reconnect_needed = False
        self.ctl: Controller | None = None
        self._connecting = False
        self.bed = self._load_bed()
        self.boards = BoardService(self.data / "boards", self.cfg_dir)
        self.calib = CalibrationStore(self.data / "calibration_sim.yaml" if sim else self.cfg_dir / "calibration.yaml",
                                      self.data / "config_history")
        from .feeders.bank import FeederBank, FeederState
        self.feeders = FeederBank(self.cfg_dir, self.bed)
        self.feeder_state = FeederState(self.data / ("feeder_state_sim.json" if sim else "feeder_state.json"))
        self.markers = self._markers()
        self.vision = VisionOps(self)
        self.setup = SetupOps(self)
        self.refresh_limits()
        from .machine_job import JobOps, PlacedStore
        self.placed = PlacedStore(self.data / "boards")
        self.jobs = JobOps(self)
        from .machine_store import StoreOps
        self.parts = StoreOps(self)                                # the parts store (self.store is the session)
        self.sim_grbl = None
        self._own_sim = sim_controller is None
        if sim:
            if self.profile.sim is None:
                raise ProfileError(self.profile_path, ["no sim: section: this profile can't run in the simulator"])
            self.sim_grbl, self.cameras = simulator.build(self.profile, self.bed or _EMPTY_BED, seed, sim_speed,
                                                          sim_controller, trays=list(self.feeders.trays.values()),
                                                          dictionary=self.feeders.dictionary, world=sim_world)
        else:
            self.cameras = {"camera_head": build_camera("camera_head", self.profile.cameras.head),
                            "camera_bottom": build_camera("camera_bottom", self.profile.cameras.bottom)}
        self.caps = capabilities.build(self.profile, None, self.cameras)
        self._install_vision_caps()
        self.vision.apply_saved_exposure()
        self.worker = Worker(self._op_failed)
        self.watcher = Watcher(self.profile_path, self._profile_changed).start() if watch else None
        mode = "SIMULATOR" if sim else "machine"
        self.events.add("start", f"Started ({mode}) with profile {self.profile.name} ({self.profile_path.name})")

    # ================================================================ helpers
    def _load_bed(self) -> dict | None:
        try:
            return yaml.safe_load((self.cfg_dir / self.profile.bed_file).read_text())
        except (OSError, yaml.YAMLError):
            return None

    def _markers(self):
        if not self.bed:
            return None
        return Markers(self.bed, self.feeders.names(), self.feeders.dictionary)

    def _context(self) -> dict:
        return {"mode": self.safety.mode, "pos": self.position(), "busy": self.worker.current if hasattr(self, "worker")
                else None}

    def say(self, text: str, level: str = "info", kind: str = "info", **data) -> None:
        self.events.add(kind, text, level, **data)

    def _save(self) -> None:
        s = self.session
        s.profile = self.profile.name
        s.lock = None if self.safety.lock is None else {"code": self.safety.lock.code,
                                                        "detail": self.safety.lock.detail,
                                                        "since": self.safety.lock.since}
        if not self.safety.homed:
            s.home_raw = None
        if not self.safety.z_ref:
            s.z_ref_raw = None
        try:
            self.store.save(s)
        except OSError as err:
            self.say(f"Could not save the session state: {err}", "warn")

    def _lock(self, code: str, detail: str = "") -> None:
        first = self.safety.lock is None or self.safety.lock.code != code
        self.safety.set_lock(code, detail)
        self._save()
        if first:
            self.say(f"LOCKED: {self.safety.lock.title}. {detail}".strip(), "error", "lock", code=code)

    def _op_failed(self, name: str, err: BaseException) -> None:
        from .motion.state import Stopped
        if isinstance(err, Stopped):
            self.say(f"{name}: stopped", "warn", "stop")
        elif isinstance(err, Refused):
            self.say(f"{name}: {err.refusal.why}. {err.refusal.todo}", "warn", "refused")
        elif isinstance(err, CapabilityError):
            self.say(f"{name}: {err}", "warn", "refused")
        elif isinstance(err, ControllerError):
            self.say(f"{name} stopped: {err}", "error", "controller")
        else:
            self.say(f"{name} failed: {err!r}", "error", "bug", trace=traceback.format_exc(limit=6))

    def _check(self, action: str) -> Refusal | None:
        return self.safety.refuse(action)

    def _submit(self, name: str, fn: Callable, *args) -> dict:
        if not self.worker.submit(name, fn, *args):
            return _no(f"The machine is busy ({self.worker.current})", "Wait, or press STOP")
        return _ok()

    # ================================================================ positions
    def raw(self) -> dict[str, float]:
        return self.ctl.raw() if self.ctl is not None else {}

    def position(self) -> dict[str, float | None] | None:
        """The head in the travel frame: X / Y from HOME when homed, Z above the Z reference when set."""
        raw = self.raw()
        if not raw:
            return None
        s = self.session
        out: dict[str, float | None] = {"X": None, "Y": None, "Z": None}
        if self.safety.homed and s.home_raw:
            out["X"], out["Y"] = raw["X"] - s.home_raw["X"], raw["Y"] - s.home_raw["Y"]
        if self.safety.z_ref and s.z_ref_raw is not None:
            out["Z"] = raw["Z"] - s.z_ref_raw
        return out

    # ================================================================ connect
    def connect(self) -> dict:
        return self._submit("Connect", self._connect)

    def _connect(self) -> None:
        self._close_link()
        p = self.profile
        self._connecting = True
        try:
            port = self.sim_grbl.open_port() if self.sim_mode else open_port(p.controller.port, p.controller.baud)
        except (OSError, ValueError) as err:
            self._connecting = False
            self.say(f"Can't open the controller port: {err}", "error", "connect")
            return
        link = GrblLink(port, p.controller.poll_s, p.controller.watchdog_s)
        link.listeners.append(self._on_link)
        link.start()
        ctl = Controller(link, p)
        try:
            info = ctl.connect(self.session.token)
        except ControllerError as err:
            self._connecting = False
            link.close()
            self.say(f"The controller does not answer on {p.controller.port or 'the USB port'}: {err}", "error",
                     "connect")
            return
        self.ctl = ctl
        self._connecting = False
        self.reconnect_needed = False
        s, had = self.session, self.session.home_raw is not None or self.session.z_ref_raw is not None
        same = bool(p.controller.session_token and s.token and info.token_found == s.token)
        self.safety.connected = True
        self.safety.lock = None
        if same:
            self.safety.homed = s.home_raw is not None
            self.safety.z_ref = s.z_ref_raw is not None
            if s.lock and s.lock.get("code") != "link_lost":
                self.safety.lock = Lock(s.lock["code"], s.lock.get("detail", ""), s.lock.get("since", 0.0))
            self.say(f"Connected: {info.version}. The controller was not reset since {s.saved or 'the last session'}: "
                     "the position is kept" + (" (Home and Z too)" if had else ""), kind="connect")
        else:
            self.safety.homed = self.safety.z_ref = False
            s.home_raw, s.z_ref_raw = None, None
            if had or s.lock:
                self.safety.set_lock("reset", "The controller was reset since the last session (power, USB or a "
                                              "reset)" if p.controller.session_token else
                                     "The session token is off: a reset can't be ruled out")
            self.say(f"Connected: {info.version or 'controller'}" + (" (new session)" if not had else ""),
                     kind="connect")
        if info.state.startswith("Alarm"):
            self.safety.set_lock("alarm", "The controller is in ALARM")
        s.token = info.token
        for k, (want, have) in info.mismatched.items():
            self.say(f"Controller setting {k} is {have}, the profile expects {want}", "warn", "settings")
        for n in info.notes:
            self.say(n, "warn", "connect")
        if p.controller.session_token and info.token is None and not info.state.startswith("Alarm"):
            self.say("Could not write the session token (T): a reset can't be seen after a restart", "warn")
        self.caps = capabilities.build(self.profile, ctl, self.cameras)
        self._install_vision_caps()
        self._save()
        if self.safety.lock is not None:
            self.say(f"LOCKED: {self.safety.lock.title}. {self.safety.lock.detail}", "error", "lock",
                     code=self.safety.lock.code)

    def _install_vision_caps(self) -> None:
        """Capabilities that need the camera and its calibration (Phase C), and the measured tables (Phase D)."""
        if self.profile.homing.method == "camera_marker":
            self.caps.homing = CameraHoming(self.vision)
        self.install_calibrated_caps()

    def nozzle_current(self) -> str:
        """The nozzle on the head now: the operator's last change (kept in the session), else the profile's."""
        n = (self.session.extra or {}).get("nozzle") if hasattr(self, "session") else None
        ids = [x.id for x in self.profile.nozzles.items]
        return n if n in ids else self.profile.nozzles.current

    def set_nozzle(self, nid: str) -> dict:
        ids = [x.id for x in self.profile.nozzles.items]
        if nid not in ids:
            return _no(f"No nozzle {nid}", f"Nozzles: {', '.join(ids)}")
        self.session.extra["nozzle"] = nid
        self._save()
        later = self.caps.later.get("nozzles")
        if later is not None:
            later.current = nid
        self.install_calibrated_caps()
        st = chain.status(self, "nozzle_offset")["state"]
        self.say(f"Nozzle {nid} is on the head" + ("" if st == "valid" else ": measure its offset (Setup, step 5)"),
                 "info" if st == "valid" else "warn", "setup")
        return _ok()

    def install_calibrated_caps(self) -> None:
        vac = self.caps.vacuum
        if self.sim_grbl is not None and self.profile.vacuum.sensor is not None:
            noz = simulator.nozzle_of(self.sim_grbl)
            vac.reader = None if noz is None else noz.pressure
        sv = chain.value(self, "vacuum_sensor")
        if sv is not None and hasattr(vac, "threshold"):
            vac.threshold = sv["threshold"]
        rot = self.caps.rotation
        v = chain.value(self, "rotation_table")
        if rot.kind == "servo_pwm":
            rot.table = RotationTable.from_dict(v) if v is not None and "pulses" in v else None
        elif rot.kind == "stepper_axis":
            rot.table = v if v is not None and v.get("kind") == "stepper" else None

    def refresh_limits(self) -> None:
        """The travel: the measured one (Setup, step 2) when valid, else the profile's."""
        self.limits = Limits(self.profile, chain.value(self, "travel") if hasattr(self, "calib") else None)

    def _close_link(self) -> None:
        if self.ctl is not None:
            self.ctl.link.listeners.clear()
            self.ctl.link.close()
            self.ctl = None
        self.safety.connected = False

    def disconnect(self) -> dict:
        return self._submit("Disconnect", self._disconnect)

    def _disconnect(self) -> None:
        self._close_link()
        self.caps = capabilities.build(self.profile, None, self.cameras)
        self._install_vision_caps()
        self.say("Disconnected. The controller keeps its position while it has power", kind="connect")

    def _on_link(self, kind: str, data: dict) -> None:
        """Called on the link's reader thread."""
        if kind == "status":
            if self.caps.estop.kind == "controller_input" and self.ctl is not None and self.caps.estop.triggered(self.ctl):
                if self.safety.lock is None or self.safety.lock.code != "estop":
                    self._lock("estop", "The E-stop input is active")
            return
        if self._connecting:
            return                                                    # the start-up banner of an Uno is normal
        if kind == "reset":
            self.session.token = None
            self._lock("reset", f"Seen: {data.get('banner', '')}")
        elif kind == "alarm":
            self._lock("estop" if data.get("code") == "10" else "alarm", data.get("line", ""))
        elif kind == "link_lost":
            self._lock("link_lost", data.get("why", ""))
        elif kind == "message":
            self.say(f"Controller: {data.get('line')}", "debug", "controller")

    # ================================================================ motion
    def jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, feed: float | None = None) -> dict:
        delta = {a: float(v) for a, v in (("X", dx), ("Y", dy), ("Z", dz)) if abs(float(v)) > 1e-6}
        if not delta:
            return _ok()
        if any(a in delta for a in "XY"):
            action = "jog_xy"
        else:
            action = "jog_z_up" if delta["Z"] > 0 else "jog_z_down"
        r = self._check(action) or self.limits.check_step(delta)
        if r is None and any(a in delta for a in "XY") and "Z" in delta:
            r = Refusal("X / Y and Z together are not allowed in a jog", "Jog one at a time")
        pos = self.position()
        if r is None and pos is not None:
            target = {a: pos[a] + d for a, d in delta.items() if pos.get(a) is not None}
            r = self.limits.check_target(target, "This jog")
        if r is not None:
            return _no(r.why, r.todo)
        f = self.limits.feed(delta, feed, self.safety.homed)
        return self._submit("Jog", self._move, delta, None, f)

    def goto(self, x: float, y: float) -> dict:
        r = self._check("goto_xy") or self.limits.check_goto({"X": float(x), "Y": float(y)})
        if r is not None:
            return _no(r.why, r.todo)
        pos = self.position()
        delta = {"X": float(x) - pos["X"], "Y": float(y) - pos["Y"]}
        r = self.limits.check_step(delta)
        if r is not None:
            return _no(r.why, r.todo)
        home = self.session.home_raw
        raw_target = {"X": home["X"] + float(x), "Y": home["Y"] + float(y)}
        return self._submit("Go to", self._move, delta, raw_target, self.limits.feed(delta, None, True), True)

    def _move(self, delta: dict[str, float], raw_target: dict[str, float] | None, feed: float,
              stall_check: bool = False, on_stall: str = "lock") -> str:
        """One checked move. Returns "ok", "stall" (on_stall="return": the caller handles it), "unknown" (the picture
        could not tell) or "stopped". A stall locks the machine unless on_stall="return"."""
        ctl = self.ctl
        if ctl is None:
            raise Refused(Refusal("The controller is not connected", "Press Connect"))
        if self.safety.lock is not None:
            raise Refused(Refusal(self.safety.lock.title, self.safety.lock.todo))
        before = ctl.raw()
        cam = self.cameras.get("camera_head")
        picture = None
        if stall_check and cam is not None and getattr(cam, "latest", None) and cam.latest() is not None:
            picture = cam.frames_after(time.monotonic() + 0.05, 1)[0]
        t = ctl.jog_to(raw_target, feed) if raw_target is not None else ctl.jog(delta, feed)
        st = ctl.wait_idle(t, Limits.travel_time(delta, feed) * 3 + 5)
        if self.worker.cancel.is_set() or self.safety.lock is not None:
            return "stopped"
        after = self.axes_raw(st)
        want = raw_target if raw_target is not None else {a: before[a] + d for a, d in delta.items()}
        off = {a: after[a] - v for a, v in want.items() if abs(after[a] - v) > POSITION_TOL}
        if off:
            self._lock("stall", "The controller reports " + ", ".join(f"{a} {v:+.3f} mm off" for a, v in off.items())
                       + " after the move")
            return "stopped"
        if picture is None:
            return "ok"
        verdict = self.vision.stall_check(picture, delta)
        if verdict is False:
            if on_stall == "return":
                return "stall"
            self._lock("stall", "The picture did not move as commanded: the head hit an end or lost steps")
            return "stopped"
        return "ok" if verdict else "unknown"

    def _move_z(self, z: float, feed: float | None = None) -> None:
        """Z to a height above the Z reference (an automatic move: the caller checked what is under the tip)."""
        if not self.safety.z_ref:
            raise Refused(Refusal("Z has no reference", "Set Z first (Setup, step 4)"))
        r = self.limits.check_target({"Z": z}, "This Z move")
        if r is not None:
            raise Refused(r)
        dz = z - self.position()["Z"]
        if abs(dz) < 1e-4:
            return
        if self._move({"Z": dz}, None, min(feed or self.limits.max_feed_z, self.limits.max_feed_z)) != "ok":
            from .motion.state import Stopped
            if self.safety.lock is not None:
                raise Refused(Refusal(self.safety.lock.title, self.safety.lock.todo))
            raise Stopped()

    def axes_raw(self, st) -> dict[str, float]:
        return self.ctl.axes.to_machine(st.mpos)

    def stop(self) -> dict:
        """STOP: at once, from any thread. Cancels jogs (no position loss) and holds any other motion."""
        self.worker.cancel.set()
        if self.ctl is not None:
            try:
                self.caps.estop.stop(self.ctl)
            except Exception as err:                                  # the link may be gone
                self.say(f"STOP could not reach the controller: {err}", "error")
        self.say("STOP", "warn", "stop")
        return _ok()

    def resume(self) -> dict:
        if self.ctl is not None:
            self.ctl.link.realtime(b"~")
        return _ok()

    # ================================================================ references
    def home(self) -> dict:
        r = self._check("home")
        if r is not None:
            return _no(r.why, r.todo)
        a = self.caps.homing.availability()
        if not a.ok:
            return _no(a.why, a.todo)
        return self._submit("Home", self._home)

    def _home(self) -> None:
        home_raw = self.caps.homing.home(self.ctl)
        if self.safety.lock is not None:
            return
        self.session.home_raw = {"X": float(home_raw["X"]), "Y": float(home_raw["Y"])}
        self.safety.homed = True
        pos = self.position()
        self.session.home_view = [round(pos["X"], 3), round(pos["Y"], 3)]
        self._save()
        self.say(f"Homed ({self.caps.homing.kind}): X / Y are now measured from HOME", kind="home")

    def set_z(self, surface_z: float = 0.0, note: str = "") -> dict:
        r = self._check("z_reference")
        if r is not None:
            return _no(r.why, r.todo)
        a = self.caps.z_reference.availability()
        if not a.ok:
            return _no(a.why, a.todo)
        return self._submit("Set Z", self._set_z, float(surface_z), note)

    def _set_z(self, surface_z: float, note: str) -> None:
        self.ctl.sync()
        self.session.z_ref_raw = float(self.caps.z_reference.establish(self.ctl, surface_z))
        self.session.z_surface_note = note
        self.safety.z_ref = True
        self._save()
        self.say(f"Z reference set ({self.caps.z_reference.kind}): the tip is at Z {surface_z:+.2f} mm"
                 + (f" ({note})" if note else ""), kind="z_reference")

    def unlock_alarm(self) -> dict:
        r = self._check("unlock_alarm")
        if r is not None:
            return _no(r.why, r.todo)
        return self._submit("Unlock alarm", self._unlock)

    def _unlock(self) -> None:
        if self.caps.estop.triggered(self.ctl):
            raise Refused(Refusal("The E-stop is still pressed", "Release it first"))
        try:
            self.ctl.unlock()
        except ControllerError as err:
            raise Refused(Refusal(f"The controller refuses to unlock ({err})",
                                  "Is the E-stop still pressed or a door open? Release it, then Unlock again")) from None
        self.safety.lock = Lock("reset", "After an ALARM the position is not trusted", time.time())
        self.session.token = self.ctl.write_token(self.session.token)
        self._save()
        self.say("Alarm unlocked. The position is not trusted: lift the nozzle clear, press 'Nozzle is clear', "
                 "then Home", "warn", "lock")

    def confirm_clear(self) -> dict:
        r = self._check("confirm_clear")
        if r is not None:
            return _no(r.why, r.todo)
        self.safety.clear_lock()
        self.safety.homed = self.safety.z_ref = False
        self._save()
        self.say("You said the nozzle is clear: unlocked. Not homed: slow jogs only until Home", kind="lock")
        return _ok()

    # ================================================================ head
    def turn_to(self, deg: float) -> dict:
        r = self._check("turn")
        if r is not None:
            return _no(r.why, r.todo)
        a = self.caps.rotation.availability()
        if not a.ok:
            return _no(a.why, a.todo)
        return self._submit("Turn", self._turn, float(deg))

    def _turn(self, deg: float) -> None:
        self.caps.rotation.turn_to(deg)
        self.say(f"Nozzle turned to {deg:+.1f}°" + (f" (±{self.caps.rotation.accuracy_deg:g}° with this axis)"
                                                       if getattr(self.caps.rotation, "accuracy_deg", None) else ""))

    def servo_limp(self) -> dict:
        if self.ctl is None:
            return _no("Not connected", "Press Connect")
        return self._submit("Servo limp", self.caps.rotation.limp)

    def pump(self, on: bool) -> dict:
        r = self._check("pump")
        if r is not None:
            return _no(r.why, r.todo)
        a = self.caps.vacuum.availability()
        if not a.ok:
            return _no(a.why, a.todo)
        return self._submit("Pump", self._pump, bool(on))

    def _pump(self, on: bool) -> None:
        self.caps.vacuum.pump(on)
        self.say("Pump ON" if on else "Pump OFF", kind="pump")

    def light(self, role: str, on: bool) -> dict:
        cap = self.caps.lights.get(role)
        if cap is None:
            return _no(f"No light {role}")
        r = self._check("light")
        if r is not None:
            return _no(r.why, r.todo)
        a = cap.availability()
        if not a.ok:
            return _no(a.why, a.todo)
        def go():
            cap.set(bool(on))
            if role == "light_head":
                self.vision.apply_saved_exposure()                 # the exposure of this light state
        return self._submit("Light", go)

    def set_exposure(self, value: float | None) -> dict:
        cam = self.cameras.get("camera_head")
        if cam is None or not hasattr(cam, "set_exposure"):
            return _no("No head camera")
        ok = cam.set_exposure(value)
        self.say(f"Exposure set to {value}" + ("" if ok else " (the camera did not take it)") +
                 ". To keep it, write it in the profile (cameras.head.exposure.value)", kind="camera")
        return _ok() if ok else _no("The camera did not take the exposure", "Check the camera")

    # ================================================================ vision (Phase C)
    def auto_exposure(self) -> dict:
        if self.cameras.get("camera_head") is None:
            return _no("No head camera")
        return self._submit("Auto exposure", self.vision.auto_exposure)

    def calibrate(self, step: str, block_mm: float | None = None) -> dict:
        if step not in ("bed", "block", "solve"):
            return _no(f"Unknown step {step}")
        if step != "solve":
            r = self._check("jog_xy")
            if r is not None:
                return _no(r.why, r.todo)
        if step == "solve" and len(self.vision.views[0]) < 3:
            return _no("No pictures of the sheet on the bed yet", "Do step 1 (sheet on the bed) first")
        return self._submit("Camera calibration", self.vision.calibrate, step, block_mm)

    def click(self, u: float, v: float) -> dict:
        """A click on the live picture: bring that spot to the picture centre."""
        if not (0 <= u <= 1 and 0 <= v <= 1):
            return _no("The click is outside the picture")
        try:
            d = self.vision.click_target(u, v)
        except Refused as err:
            return _no(err.refusal.why, err.refusal.todo)
        if self.safety.homed and self.limits.xy_known:
            pos = self.position()
            return self.goto(pos["X"] + float(d[0]), pos["Y"] + float(d[1]))
        return self.jog(float(d[0]), float(d[1]))

    def register_board(self, side: str, top_edge: str, copy: str = "1", x: float | None = None,
                       y: float | None = None) -> dict:
        """copy: a panel's other boards (2, 3, ...) with their planned centre (x, y mm from HOME)."""
        if side not in ("F", "B") or top_edge not in ("back", "front", "left", "right"):
            return _no("Choose the side (F / B) and where the board's top edge is")
        copy = str(copy or "1")
        if copy != "1" and (x is None or y is None):
            return _no("Where does this copy lie?", "Type its centre (X, Y mm from HOME)")
        r = self._check("goto_xy")
        if r is not None:
            return _no(r.why, r.todo)
        return self._submit("Board registration", self.vision.register, side, top_edge, copy,
                            None if x is None else (float(x), float(y)))

    def overlay(self) -> dict:
        return self.vision.overlay() | {"tip": self.tip_spot()}

    def tip_spot(self) -> list[float] | None:
        """Where the nozzle tip would land, as a fraction of the picture (the ring on the live picture)."""
        model = self.vision.model()
        est = self.setup.offset_estimate()
        if model is None or est is None:
            return None
        rot = self.caps.rotation
        ang = getattr(rot, "angle", None) if rot.kind != "none" else None
        q = model.xy_to_px([est.tip(ang)], 0.0)[0]
        w, h = model.size
        return [round(float(q[0]) / w, 4), round(float(q[1]) / h, 4)]

    # ================================================================ the wizard (Phase D)
    def wizard(self) -> list[dict]:
        return [s.as_dict() for s in wizard.build(self)]

    def undo(self, key: str) -> dict:
        if key not in chain.CHAIN:
            return _no(f"Nothing to undo for {key}")
        if self.worker.current is not None:
            return _no(f"The machine is busy ({self.worker.current})", "Wait, or press STOP")
        if not self.calib.undo(chain.skey(self, key)):
            return _no("Nothing to undo", "")
        self.refresh_limits()
        self.install_calibrated_caps()
        stale = [chain.CHAIN[k].title for k in chain.dependants(key) if chain.entry(self, k) is not None]
        self.say(f"Undo: {chain.CHAIN[key].title} is back to the value before" +
                 (f". Now stale: {', '.join(stale)}" if stale else ""), "warn", "setup")
        return _ok()

    SETUP_ACTIONS = {"vacuum_open", "vacuum_closed", "switch_test", "bottom_calibrate", "creep", "end_here", "save_travel", "nozzle_marks", "pointer_tip", "pointer_click", "accept",
                     "rotation_marks", "survey", "look_at_slot", "confirm", "first_part", "first_part_answer",
                     "outputs_test"}

    def setup_action(self, action: str, params: dict) -> dict:
        """One wizard action (the Setup tab). Long ones run on the worker; the result comes as events."""
        s = self.setup
        if action not in self.SETUP_ACTIONS:
            return _no(f"Unknown set-up action {action}")
        needs_conn = action not in ("confirm", "first_part_answer", "accept")
        if needs_conn:
            r = self._check("jog_xy")
            if r is not None:
                return _no(r.why, r.todo)
        try:
            if action in ("creep", "end_here"):
                d = str(params.get("dir", ""))
                if d not in ("X-", "X+", "Y-", "Y+"):
                    return _no("Choose an end: X-, X+, Y- or Y+")
                fn = s.creep if action == "creep" else s.end_here
                return self._submit(f"Travel {d}", fn, d[0], 1 if d[1] == "+" else -1)
            if action == "save_travel":
                return self._submit("Save the travel", s.save_travel)
            if action in ("nozzle_marks", "rotation_marks"):
                r = self._check("auto_z_down")
                if r is not None:
                    return _no(r.why, r.todo)
                sheet = params.get("sheet_mm")
                if sheet in (None, ""):
                    return _no("Type the sheet's height (mm above the labels)", "Measure the paper with a caliper")
                fn = s.nozzle_marks if action == "nozzle_marks" else s.rotation_marks
                return self._submit("Nozzle marks" if action == "nozzle_marks" else "Angle table", fn, float(sheet))
            if action == "pointer_tip":
                return self._submit("Pointer", s.pointer_tip_here)
            if action == "pointer_click":
                return self._submit("Pointer", s.pointer_click, float(params["u"]), float(params["v"]))
            if action == "accept":
                s.accept(str(params.get("key", "nozzle_offset")))
                return _ok()
            if action == "survey":
                return self._submit("Bed survey", s.survey)
            if action in ("vacuum_open", "vacuum_closed"):
                return self._submit("Vacuum sensor", s.vacuum_read, action == "vacuum_closed")
            if action == "switch_test":
                return self._submit("Homing switches", s.switch_test)
            if action == "bottom_calibrate":
                return self._submit("Bottom camera", s.bottom_calibrate)
            if action == "look_at_slot":
                return self._submit("Look at the slot", s.look_at_slot, int(params["slot"]),
                                    str(params.get("package") or "0805"))
            if action == "confirm":
                return s.confirm_candidate(str(params.get("yes")).lower() in ("1", "true", "yes"))
            if action == "first_part":
                r = self._check("auto_z_down")
                if r is not None:
                    return _no(r.why, r.todo)
                return self._submit("First part", s.first_part, int(params["slot"]), str(params["ref"]))
            if action == "first_part_answer":
                ok = str(params.get("ok")).lower() in ("1", "true", "yes")
                s.first_part_answer(ok, float(params.get("dx") or 0), float(params.get("dy") or 0),
                                    float(params.get("dtheta") or 0))
                return _ok()
            if action == "outputs_test":
                return self._submit("Outputs test", self._outputs_test)
        except Refused as err:
            return _no(err.refusal.why, err.refusal.todo)
        except (KeyError, ValueError) as err:
            return _no(f"Missing or wrong value: {err}")
        return _no("not handled")

    def _outputs_test(self) -> None:
        v = self.caps.vacuum
        if v.availability().ok:
            v.pump(True)
            self.ctl.gcode("G4 P0.5")
            v.pump(False)
            self.say("Outputs test: pump on for 0.5 s, then off. Did it click twice?", kind="setup")
        rot = self.caps.rotation
        if rot.kind != "none" and rot.availability().ok:
            back = rot.angle if rot.angle is not None else 0.0
            rot.turn_to(back + 10.0 if back + 10.0 <= rot.range_deg[1] else back - 10.0)
            rot.turn_to(back)
            self.say(f"Outputs test: the nozzle turned 10° and back to {back:+.0f}°", kind="setup")
        for role, cap in self.caps.lights.items():
            if cap.kind != "none" and cap.availability().ok:
                cap.set(True)
                cap.set(False)
                self.say(f"Outputs test: {role} on and off", kind="setup")

    def sim_event(self, what: str, n: int | None = None) -> dict:
        """Simulator only: make something happen to the virtual machine (training, tests)."""
        g = self.sim_grbl
        if g is None:
            return _no("Only in the simulator")
        if what in ("tray", "strip_in") and n is not None:
            try:
                text = self.vision.sim_prop(what, n=int(n))
            except (Refused, KeyError) as err:
                return _no(getattr(getattr(err, "refusal", None), "why", str(err)))
            self.say("Simulator: " + text, kind="sim")
            return _ok()
        if what == "power_cycle":
            g.power_cycle()
            self.say("Simulator: power dip (the controller restarted)", "warn", "sim")
        elif what == "estop":
            g.trigger_estop()
            self.say("Simulator: E-stop pressed", "warn", "sim")
        elif what == "release_estop":
            g.release_estop()
            self.say("Simulator: E-stop released", kind="sim")
        elif what in ("charuco", "charuco_block", "charuco_off", "board", "board_F", "strip", "mark_sheet"):
            try:
                text = self.vision.sim_prop({"charuco_block": "charuco", "board_F": "board"}.get(what, what),
                                            z=5.0 if what == "charuco_block" else (0.1 if what == "mark_sheet" else 0.0),
                                            side="F" if what == "board_F" else "B")
            except Refused as err:
                return _no(err.refusal.why, err.refusal.todo)
            self.say("Simulator: " + text, kind="sim")
        else:
            return _no(f"Unknown simulator event {what}")
        return _ok()

    # ================================================================ profile reload
    def _profile_changed(self, new: Profile | ProfileError) -> None:
        if isinstance(new, ProfileError):
            self.profile_error = new
            self.say(f"The profile file has errors: the running profile stays. {'; '.join(new.problems)}", "error",
                     "profile")
            return
        if new.name != self.profile.name:
            self.profile_error = ProfileError(self.profile_path, [f"name changed to {new.name!r}: restart the program "
                                                                  "to switch machines"])
            self.say(self.profile_error.problems[0], "error", "profile")
            return
        change = diff(self.profile, new)
        self.profile_error = None
        if not change.fields:
            return
        old = self.profile
        self.profile = new
        self.refresh_limits()
        if self.ctl is not None:
            self.ctl.profile = new
            self.ctl.link.poll_s, self.ctl.link.watchdog_s = new.controller.poll_s, new.controller.watchdog_s
        if not self.sim_mode and old.cameras != new.cameras:
            for c in self.cameras.values():
                if hasattr(c, "close"):
                    c.close()
            self.cameras = {"camera_head": build_camera("camera_head", new.cameras.head),
                            "camera_bottom": build_camera("camera_bottom", new.cameras.bottom)}
        self.caps = capabilities.build(new, self.ctl, self.cameras)
        self._install_vision_caps()
        self.bed = self._load_bed() or self.bed
        self.markers = self._markers()
        self.say("Profile reloaded: " + ", ".join(change.fields[:8]) + (" ..." if len(change.fields) > 8 else ""),
                 kind="profile")
        if change.rehome and self.safety.homed:
            self.safety.homed = False
            self._save()
            self.say("The axis map, the travel frame or the homing method changed: Home again", "warn", "profile")
        if change.reconnect:
            self.reconnect_needed = True
            self.say("The controller port / type / axis map changed: it is used after Connect (Machine tab)", "warn",
                     "profile")

    # ================================================================ status
    def status(self) -> dict:
        ctl = self.ctl
        st = ctl.link.status if ctl is not None else None
        rot = self.caps.rotation
        return {
            "sim": self.sim_mode,
            "profile": {"name": self.profile.name, "title": self.profile.title, "file": str(self.profile_path),
                        "errors": self.profile_error.problems if self.profile_error else [],
                        "missing": [m.as_dict() for m in missing(self.profile)],
                        "reconnect_needed": self.reconnect_needed},
            "safety": self.safety.as_dict() | {"lock": None if self.safety.lock is None else
                                                {"code": self.safety.lock.code, "title": self.safety.lock.title,
                                                 "detail": self.safety.lock.detail, "todo": self.safety.lock.todo}},
            "banner": self.safety.banner(),
            "busy": self.worker.current,
            "controller": None if ctl is None else {
                "state": st.state if st else "?", "raw": self.raw(), "version": ctl.info.version,
                "firmware_ok": ctl.info.firmware_ok, "token": self.session.token,
                "mismatched": {k: list(v) for k, v in ctl.info.mismatched.items()},
                "spindle": st.spindle if st else None, "accessories": st.accessories if st else "",
                "pins": st.pins if st else "", "lost": ctl.link.lost},
            "position": self.position(),
            "limits": {"travel": {k: (list(v) if v else None) for k, v in self.limits.travel.items()},
                       "max_step": self.limits.max_step, "slow_feed": self.limits.slow_feed,
                       "max_feed_xy": self.limits.max_feed_xy, "max_feed_z": self.limits.max_feed_z},
            "rotation": {"kind": rot.kind, "angle": getattr(rot, "angle", None), "pulse": getattr(rot, "pulse", None),
                         "range": getattr(rot, "range_deg", None), "accuracy": getattr(rot, "accuracy_deg", None)},
            "pump": bool(getattr(self.caps.vacuum, "pump_on", False)),
            "capabilities": self.caps.describe(),
            "vision": self.vision.status(),
            "job": self.jobs.status(),
            "sim_truth": None if self.sim_grbl is None else {
                "head": dict(self.sim_grbl.phys), "lost_steps": dict(self.sim_grbl.lost_steps),
                "servo_angle": self.sim_grbl.servo_angle},
        }

    def map_data(self) -> dict:
        """The bed map: labels and the board area from bed.yaml, the travel, the head (when homed)."""
        bed = self.bed or {}
        lay = bed.get("layout", {}) or {}
        out = {"travel": {k: (list(v) if v else None) for k, v in self.limits.travel.items()},
               "home": lay.get("home_marker_xy", [0, 0]), "slot_direction": lay.get("slot_direction", [0, -1]),
               "slots": {str(k): v for k, v in (lay.get("slot_marker_xy") or {}).items()},
               "pcb_area": None, "pcb_corners": lay.get("pcb_corner_marker_xy") if
               (bed.get("pcb_area") or {}).get("corner_labels") else None,
               "head": None}
        if lay.get("pcb_area_xy") is not None and bed.get("pcb_area"):
            out["pcb_area"] = {"xy": lay["pcb_area_xy"], "size": bed["pcb_area"]["size_mm"]}
        pos = self.position()
        if pos and pos.get("X") is not None:
            out["head"] = [pos["X"], pos["Y"]]
        if self.sim_grbl is not None:
            out["sim_head"] = [self.sim_grbl.phys["X"], self.sim_grbl.phys["Y"]]
        return out

    # ================================================================ shutdown
    def close(self) -> None:
        if self.watcher is not None:
            self.watcher.stop()
        self.worker.stop()
        self._close_link()
        for c in self.cameras.values():
            if hasattr(c, "close"):
                c.close()
        if self.sim_grbl is not None and self._own_sim:
            self.sim_grbl.shutdown()
        self.say("Closed. The controller keeps its position while it has power", kind="start")


_EMPTY_BED = {"markers": {"dictionary": "DICT_4X4_50", "size_mm": 10, "home_id": 49, "slot_ids": [],
                          "pcb_corner_ids": []}, "layout": {"home_marker_xy": [0, 0]}, "pcb_area": {}}

__all__ = ["LinkLost", "Machine"]
