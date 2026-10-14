"""A virtual Grbl / grblHAL that speaks the real line protocol (docs/13 §4.12).

The driver, the limits and the capabilities run against it unchanged. Mechanics:
- every motor counts its steps (that is MPos). The head's REAL position follows, until an axis reaches the
  frame's hard end (profile sim.hard_ends): then the motor loses steps, like on the bench (2026-10-06).
- a reset (power, first port open, Ctrl-X) clears the parser state, so T goes back to 0; a power reset also
  sets MPos to 0 while the head stays where it is.
- M3 / M5 / M7 / M8 / M9 / G4 / G38.2 answer only after the moves before them, like Grbl.
- M3 S<us> turns the virtual servo; the gear play puts it up to ±play/2 off, depending on the side it came from.
- $H homes against virtual switches at the frame's max ends (profile sim.switches).
- trigger_estop() raises the grblHAL E-stop alarm (ALARM:10, Pn:E).
"""
from __future__ import annotations

import math
import queue
import random
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, field

from ..motion.axes import AxisMap
from ..profile.schema import Profile

PLANNER_BLOCKS = 15
TICK = 0.01
WORD = re.compile(r"([A-Z])\s*([-+]?\d*\.?\d+)")


@dataclass
class Block:
    kind: str                                   # move | jog | probe
    target: dict[str, float]                    # controller letter -> counted position
    feed: float                                 # mm/min
    done: threading.Event = field(default_factory=threading.Event)
    probe_hit: dict[str, float] | None = None


class SimPort:
    """A pyserial-like port to the virtual controller."""

    def __init__(self, grbl: SimGrbl) -> None:
        self.grbl = grbl
        self.timeout = 0.05
        self.is_open = True
        self.rx: queue.Queue[bytes] = queue.Queue()
        self._line = b""

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise OSError("port closed")
        for b in data:
            c = bytes([b])
            if c in (b"?", b"!", b"~", b"\x18", b"\x85"):
                self.grbl.realtime(c)
            elif c == b"\n":
                line, self._line = self._line.decode(errors="replace").strip(), b""
                self.grbl.lines.put(line)
            elif c != b"\r":
                self._line += c
        return len(data)

    def read(self, n: int = 1) -> bytes:
        try:
            return self.rx.get(timeout=self.timeout)
        except queue.Empty:
            return b""

    def reset_input_buffer(self) -> None:
        while not self.rx.empty():
            self.rx.get_nowait()

    def close(self) -> None:
        self.is_open = False
        self.grbl.detach(self)


class SimGrbl:
    def __init__(self, profile: Profile, seed: int | None = None, reset_on_first_open: bool = True) -> None:
        if profile.sim is None:
            raise ValueError(f"profile {profile.name} has no sim: section")
        s = profile.sim
        self.profile = profile
        self.flavor = profile.controller.type
        self.axes = AxisMap.from_profile(profile)
        self.rot_letter = profile.rotation.stepper_axis.letter if profile.rotation.type == "stepper_axis" else None
        self.letters = "XYZ" + (self.rot_letter or "")
        self.ends = {a: tuple(v) for a, v in s.hard_ends.items()}
        self.phys = {a: float(s.start.get(a, 0.0)) for a in "XYZ"}       # the head's real position (machine axes)
        self.counted = {L: 0.0 for L in self.letters}
        self.speed = s.speed
        self.switches = s.switches
        self.probe_surface = s.probe_surface_z
        self.play = s.servo_play_deg
        servo = profile.rotation.servo_pwm
        self.servo_cfg = servo
        self.rng = random.Random(seed)
        self.settings = {"$0": "10", "$1": "25", "$10": "1", "$20": "0", "$21": "0", "$22": "0", "$27": "1.000",
                         "$30": "1000", "$32": "0", "$100": "250", "$101": "250", "$102": "250",
                         "$110": "2000", "$111": "2000", "$112": "500"}
        self.settings.update(profile.controller.expected_settings)
        self.settings["$22"] = "1" if self.switches else self.settings["$22"]
        if self.switches:                                  # max travel ($130-$132): the frame between the switches
            for a in ("X", "Y"):
                L, _ = self.axes.to_ctrl[a]
                if L in "XYZ":
                    self.settings[f"${130 + 'XYZ'.index(L)}"] = f"{self.ends[a][1] - self.ends[a][0] - 1.0:.3f}"
        self.rot_scale, self.rot_backlash = s.rot_scale, s.rot_backlash_deg
        self._rot_dir = 1.0
        self.version = profile.controller.firmware or ("1.1h.20190825" if self.flavor == "grbl" else "1.1f.20230610")
        self.state_alarm: str | None = None
        self.hold = False
        self.estop = False
        self.tool = 0
        self.distance = "G90"
        self.motion = "G0"
        self.feed = 0.0
        self.spindle_us = 0
        self.flood = self.mist = False
        self.dout: dict[int, bool] = {}            # grblHAL digital outputs (M62-M65 P<n>): a light, a valve
        self.servo_angle = 0.0
        self.servo_from: str | None = None
        self.lost_steps = {a: 0.0 for a in "XYZ"}
        self.servo_scale = s.servo_scale
        self.hooks: list = []                    # the simulated nozzle: hook(kind, data), called with the lock held
        self.z_floor = None                      # () -> the lowest tip Z before the Z motor loses steps
        self.resets = 0
        self.log: list[str] = []                 # every line received (tests read it)
        self.queue: deque[Block] = deque()
        self.lines: queue.Queue[str] = queue.Queue()
        self.lock = threading.RLock()
        self.port_: SimPort | None = None
        self.opened = 0
        self.reset_on_first_open = reset_on_first_open
        self.alive = True
        self._threads = [threading.Thread(target=self._motion_loop, name="sim-motion", daemon=True),
                         threading.Thread(target=self._line_loop, name="sim-parser", daemon=True)]
        for t in self._threads:
            t.start()

    # ---------------------------------------------------------------- ports
    def open_port(self) -> SimPort:
        with self.lock:
            p = SimPort(self)
            self.port_ = p
            self.opened += 1
            if self.opened == 1 and self.reset_on_first_open:
                self.power_cycle()                       # the Uno resets when its port opens
            return p

    def detach(self, p: SimPort) -> None:
        with self.lock:
            if self.port_ is p:
                self.port_ = None

    def _out(self, text: str) -> None:
        p = self.port_
        if p is not None and p.is_open:
            p.rx.put(text.encode() + b"\r\n")

    def shutdown(self) -> None:
        self.alive = False

    # ---------------------------------------------------------------- test hooks
    def power_cycle(self) -> None:
        """Power dip / port open on an Uno: everything restarts, MPos = 0, the head stays where it is."""
        with self.lock:
            self.counted = {L: 0.0 for L in self.letters}
            self._reset(power=True)

    def trigger_estop(self) -> None:
        with self.lock:
            self.estop = True
            self._abort_motion()
            self._alarm("10")

    def release_estop(self) -> None:
        with self.lock:
            self.estop = False

    # ---------------------------------------------------------------- state
    def _state(self) -> str:
        if self.state_alarm is not None:
            return "Alarm"
        if self.hold:
            return "Hold:0"
        if self.queue:
            return "Jog" if self.queue[0].kind == "jog" else "Run"
        return "Idle"

    def status_line(self) -> str:
        with self.lock:
            pos = ",".join(f"{self.counted[L]:.3f}" for L in self.letters)
            acc = ("S" if self.spindle_us else "") + ("F" if self.flood else "") + ("M" if self.mist else "")
            feed = self.queue[0].feed if self.queue else 0
            pins = "E" if self.estop else ""
            line = f"<{self._state()}|MPos:{pos}|FS:{feed:.0f},{self.spindle_us}"
            line += f"|Pn:{pins}" if pins else ""
            line += f"|A:{acc}" if acc else ""
            return line + ">"

    def _reset(self, power: bool = False) -> None:
        moving = bool(self.queue) and not self.hold
        self._abort_motion()
        self.hold = False
        self.tool, self.distance, self.motion, self.feed = 0, "G90", "G0", 0.0
        self.spindle_us, self.flood, self.mist = 0, False, False
        self.resets += 1
        banner = "Grbl 1.1h ['$' for help]" if self.flavor == "grbl" else "GrblHAL 1.1f ['$' or '$HELP' for help]"
        if moving and not power:
            self.state_alarm = "3"
            self._out("ALARM:3")
        elif power:
            self.state_alarm = "10" if self.estop else None
        self._out(banner)
        if self.state_alarm is not None:
            self._out("[MSG:'$H'|'$X' to unlock]")

    def _abort_motion(self) -> None:
        while self.queue:
            self.queue.popleft().done.set()

    def _alarm(self, code: str) -> None:
        self.state_alarm = code
        self._out(f"ALARM:{code}")

    # ---------------------------------------------------------------- realtime
    def realtime(self, c: bytes) -> None:
        with self.lock:
            if c == b"?":
                self._out(self.status_line())
            elif c == b"!":
                if self.queue and all(b.kind == "jog" for b in self.queue):
                    self._abort_motion()                 # a feed hold cancels jogs (Grbl jogging.md)
                elif self.queue and self.state_alarm is None:
                    self.hold = True
            elif c == b"~":
                self.hold = False
            elif c == b"\x85":
                if self.queue and all(b.kind == "jog" for b in self.queue):
                    self._abort_motion()
                    self.hold = False
            elif c == b"\x18":
                self._reset()

    # ---------------------------------------------------------------- motion
    def _motion_loop(self) -> None:
        last = time.monotonic()
        while self.alive:
            time.sleep(TICK)
            now = time.monotonic()
            dt, last = (now - last) * self.speed, now
            with self.lock:
                if self.hold or self.state_alarm is not None or not self.queue:
                    continue
                b = self.queue[0]
                rem = {L: b.target[L] - self.counted[L] for L in b.target}
                dist = math.sqrt(sum(d * d for d in rem.values()))
                step = b.feed / 60.0 * dt
                frac = 1.0 if dist <= step or dist == 0 else step / dist
                for L, d in rem.items():
                    self._step(L, d * frac)
                if b.kind == "probe" and self.probe_surface is not None and self.phys["Z"] <= self.probe_surface:
                    # back to the crossing: a long tick (a busy PC) must not move the trip point
                    L, sign = self.axes.to_ctrl["Z"]
                    self._step(L, sign * (self.probe_surface - self.phys["Z"]))
                    b.probe_hit = dict(self.counted)
                    frac = 1.0
                if frac >= 1.0:
                    self.queue.popleft()
                    b.done.set()
                self._emit("tick", {})

    def _emit(self, kind: str, data: dict) -> None:
        for h in self.hooks:
            h(kind, data)

    def _step(self, letter: str, d: float) -> None:
        """Move one motor by d (controller units); the head follows unless the frame stops it."""
        self.counted[letter] += d
        if letter == self.rot_letter and d:                # a stepper nozzle: scale error and backlash
            self._rot_dir = 1.0 if d > 0 else -1.0
            self.servo_angle = self.counted[letter] * self.rot_scale - self._rot_dir * self.rot_backlash / 2
            return
        for a, (L, sign) in self.axes.to_ctrl.items():
            if L != letter:
                continue
            lo, hi = self.ends[a]
            if a == "Z" and self.z_floor is not None:
                lo = self.z_floor()                      # the nozzle: what is under the tip, minus the spring
            want = self.phys[a] + sign * d
            got = min(max(want, lo), hi)
            if got != want:
                self.lost_steps[a] += abs(want - got)
                if a == "Z" and want < got:
                    self._emit("lost_z", {"mm": got - want})
            self.phys[a] = got

    def _wait_queue(self) -> None:
        while True:
            with self.lock:
                if not self.queue or self.state_alarm is not None:
                    return
            time.sleep(TICK)

    def _queue(self, b: Block) -> None:
        while True:
            with self.lock:
                if len(self.queue) < PLANNER_BLOCKS:
                    self.queue.append(b)
                    return
            time.sleep(TICK)

    # ---------------------------------------------------------------- lines
    def _line_loop(self) -> None:
        while self.alive:
            try:
                line = self.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            self.log.append(line)
            try:
                reply = self._execute(line)
            except _Err as e:
                reply = f"error:{e.code}"
            if reply:
                self._out(reply)

    def _execute(self, line: str) -> str | None:
        line = re.sub(r"\(.*?\)|;.*$", "", line).strip().upper()
        if not line:
            return "ok"
        if line.startswith("$"):
            return self._dollar(line)
        if self.state_alarm is not None:
            raise _Err(9)
        return self._gcode(line)

    def _dollar(self, line: str) -> str | None:
        if line == "$$":
            for k in sorted(self.settings, key=lambda k: int(k[1:])):
                self._out(f"{k}={self.settings[k]}")
            return "ok"
        if line == "$I":
            self._out(f"[VER:{self.version}:]")
            self._out("[OPT:VM,15,128]")
            return "ok"
        if line == "$G":
            self._out(f"[GC:{self.motion} G54 G17 G21 {self.distance} G94 M{'3' if self.spindle_us else '5'} "
                      f"M{'8' if self.flood else '9'} T{self.tool} F{self.feed:g} S{self.spindle_us}]")
            return "ok"
        if line == "$X":
            if self.estop:
                return "error:79" if self.flavor == "grblhal" else "error:9"
            with self.lock:
                self.state_alarm = None
            self._out("[MSG:Caution: Unlocked]")
            return "ok"
        if line == "$H":
            if not self.switches:
                raise _Err(5)
            return self._home()
        if line.startswith("$J="):
            return self._jog(line[3:])
        m = re.fullmatch(r"\$(\d+)=(\S+)", line)
        if m:
            self.settings[f"${m.group(1)}"] = m.group(2)
            return "ok"
        raise _Err(3)

    def _home(self) -> str:
        """Drive X and Y to their max ends (the switches), back off 1 mm, call that MPos 0."""
        with self.lock:
            self.state_alarm = None
        for a in "XY":
            L, sign = self.axes.to_ctrl[a]
            span = self.ends[a][1] - self.ends[a][0]
            target = dict(self.counted)
            target[L] = self.counted[L] + sign * (span + 5)
            b = Block("move", {L: target[L]}, 1000.0)
            self._queue(b)
            b.done.wait()
            with self.lock:                              # the switch stops the motor: no steps are lost
                self.counted[L] -= sign * self.lost_steps[a]
                self.lost_steps[a] = 0.0
            back = Block("move", {L: self.counted[L] - sign * 1.0}, 300.0)
            self._queue(back)
            back.done.wait()
        with self.lock:
            for a in "XY":
                self.counted[self.axes.letter(a)] = 0.0
        return "ok"

    def _words(self, text: str) -> list[tuple[str, float]]:
        out = [(k, float(v)) for k, v in WORD.findall(text)]
        if not out and text:
            raise _Err(1)
        return out

    def _targets(self, words: list[tuple[str, float]], distance: str, g53: bool) -> dict[str, float]:
        t = {}
        for k, v in words:
            if k in self.letters:
                t[k] = self.counted[k] + v if distance == "G91" else v
        del g53                                           # no work offsets in the simulator: G53 = G54
        return t

    def _jog(self, text: str) -> str:
        if self.state_alarm is not None:
            raise _Err(9)
        with self.lock:
            st = self._state()
        if st not in ("Idle", "Jog"):
            raise _Err(8)
        words = self._words(text)
        gs = {f"G{v:g}" for k, v in words if k == "G"}
        distance = "G91" if "G91" in gs else ("G90" if "G90" in gs else self.distance)
        feed = next((v for k, v in words if k == "F"), None)
        if feed is None or feed <= 0:
            raise _Err(22)
        target = self._targets(words, distance, "G53" in gs)
        if not target:
            raise _Err(26)
        self._queue(Block("jog", target, feed))
        return "ok"

    def _gcode(self, line: str) -> str | None:
        words = self._words(line)
        gs = [f"G{v:g}" for k, v in words if k == "G"]
        ms = [int(v) for k, v in words if k == "M"]
        vals = {k: v for k, v in words if k not in "GM"}
        if self.state_alarm is not None:
            raise _Err(9)
        for g in gs:
            if g in ("G90", "G91"):
                self.distance = g
            elif g in ("G0", "G1"):
                self.motion = g
            elif g not in ("G4", "G38.2", "G53", "G21", "G17", "G54", "G94", "G10", "G92", "G20", "G43.1",
                           "G49", "G2", "G3"):
                raise _Err(20)
        if "F" in vals:
            self.feed = vals["F"]
        if "T" in vals:
            if vals["T"] > 255 or vals["T"] < 0:
                raise _Err(4)
            self.tool = int(vals["T"])
        if "G10" in gs or "G92" in gs:
            return "ok"
        if "G4" in gs:
            self._wait_queue()
            time.sleep(vals.get("P", 0.0) / self.speed)
            return "ok"
        target = self._targets(words, "G91" if "G91" in gs else self.distance, "G53" in gs)
        if "G38.2" in gs:
            return self._probe(target)
        if target:
            feed = self.feed if self.motion == "G1" else 2000.0
            if feed <= 0:
                raise _Err(22)
            self._queue(Block("move", target, feed))
        for m in ms:
            self._wait_queue()                           # spindle / coolant run after the moves before them
            with self.lock:
                if m == 3:
                    self._servo(int(vals.get("S", 0)))
                elif m == 5:
                    self.spindle_us = 0
                elif m == 7:
                    self.mist = True
                elif m == 8:
                    self.flood = True
                elif m == 9:
                    self.flood = self.mist = False
                elif m in (62, 63, 64, 65) and self.flavor == "grblhal":
                    self.dout[int(vals.get("P", 0))] = m in (62, 64)
                elif m not in (0, 1, 2, 30, 6):
                    raise _Err(20)
                self._emit("output", {"flood": self.flood, "mist": self.mist, "dout": dict(self.dout)})
        return "ok"

    def _probe(self, target: dict[str, float]) -> str | None:
        if not target:
            raise _Err(26)
        b = Block("probe", target, self.feed or 100.0)
        self._wait_queue()
        self._queue(b)
        b.done.wait()
        if b.probe_hit is None:
            with self.lock:
                self._alarm("5")                         # G38.2: no contact within the travel
            return None
        pos = ",".join(f"{b.probe_hit[L]:.3f}" for L in self.letters)
        self._out(f"[PRB:{pos}:1]")
        return "ok"

    def _servo(self, us: int) -> None:
        """Pulse → angle with gear play: from below it stops a bit short, from above a bit past (±play/2)."""
        if us <= 0:
            self.spindle_us = 0
            return
        us = min(max(us, 500), 2500)
        c = self.servo_cfg
        nominal = 0.0
        if c is not None and c.nominal_range_deg:
            nominal = (us - c.centre_us) * c.nominal_range_deg / (c.max_us - c.min_us) * self.servo_scale
        side = -1.0 if (self.spindle_us and us > self.spindle_us) else 1.0
        self.servo_angle = nominal + side * self.play / 2 * (0.6 + 0.4 * self.rng.random())
        self.spindle_us = us


class _Err(Exception):
    def __init__(self, code: int) -> None:
        self.code = code
