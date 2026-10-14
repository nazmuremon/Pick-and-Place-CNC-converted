"""The machine profile: one YAML file per machine (`config/profiles/<name>.yaml`, docs/13 §4.2).

Everything that is special about a machine lives here, never in code. A field that is null is "TBD": the
capability that needs it is switched off and the page says what is missing (profile/check.py).

Units: mm, degrees, seconds; feed rates in mm/min.
"""
from __future__ import annotations

import re
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

SCHEMA_VERSION = 1

MachineAxis = Literal["X", "Y", "Z"]
Letter = Literal["X", "Y", "Z", "A", "B", "C"]
GCODE_LINE = re.compile(r"^\s*([GMT]\d+(\.\d+)?)(\s+[A-Z]-?[\d.{}a-z_]+)*\s*$")


class Base(BaseModel):
    model_config = ConfigDict(extra="forbid")


def _range(v: tuple[float, float] | None) -> tuple[float, float] | None:
    if v is not None and not v[0] < v[1]:
        raise ValueError(f"[min, max] must have min < max, got {list(v)}")
    return v


def _gcode(v: str | None) -> str | None:
    """Outputs are switched only through the controller (D3): a G / M code line, never a pin or a shell command."""
    if v is not None and not GCODE_LINE.match(v):
        raise ValueError(f"{v!r} is not a G-code line like 'M8' or 'M3 S{{us}}'")
    return v


# ------------------------------------------------------------------ controller and motion
class AxisTarget(Base):
    letter: Letter
    sign: Literal[1, -1] = 1


class Controller(Base):
    type: Literal["grbl", "grblhal"]
    port: str | None = None            # a device (/dev/ttyACM0), a pyserial URL (socket://127.0.0.1:7450 = port
                                       # keeper, `pnp portd`), or null = the first USB serial port
    baud: int = 115200
    poll_s: float = Field(0.2, ge=0.05, le=2.0)       # status poll ('?')
    watchdog_s: float = Field(2.0, ge=0.5, le=30.0)   # no status for this long = the link is lost
    open_timeout_s: float = Field(4.0, ge=0.5, le=30.0)  # an Uno that resets on open answers after ~1.5 s
    firmware: str | None = None        # $I must contain this text (e.g. our servo build); null = any
    axis_map: dict[MachineAxis, AxisTarget]
    expected_settings: dict[str, str] = {}             # checked at Connect, e.g. {"$22": "0"}
    session_token: bool = True         # keep a number in the controller's tool word T to see a reset (D22)

    @field_validator("axis_map")
    @classmethod
    def _axes(cls, v: dict[str, AxisTarget]) -> dict[str, AxisTarget]:
        if sorted(v) != ["X", "Y", "Z"]:
            raise ValueError("axis_map needs X, Y and Z")
        letters = [t.letter for t in v.values()]
        if len(set(letters)) != 3:
            raise ValueError(f"axis_map uses a controller letter twice: {letters}")
        return v


class Travel(Base):
    """Where the head may go. frame 'home' = mm from the HOME marker centre (camera homing, no switches);
    'controller' = the controller's machine coordinates (switch homing). The camera centre is the head's
    reference point; the nozzle reach is derived from this and the nozzle offset (docs/13 §4.8)."""
    frame: Literal["home", "controller"] = "home"
    x: tuple[float, float] | None = None
    y: tuple[float, float] | None = None
    z: tuple[float, float] | None = None     # nozzle tip height above the Z reference surface

    @field_validator("x", "y", "z")
    @classmethod
    def _ranges(cls, v):
        return _range(v)


class Motion(Base):
    max_feed_xy: float = Field(gt=0)
    max_feed_z: float = Field(gt=0)
    jog_feed_not_homed: float = Field(gt=0)          # XY jogs before Home (docs/07 safety rule 1)
    max_step_mm: float = Field(gt=0)                 # the largest single move
    z_clearance_mm: float | None = Field(None, gt=0)  # the moving height: this far above the highest pick / place
                                                      # surface (null = TBD: no automatic Z moves)
    travel: Travel = Travel()


# ------------------------------------------------------------------ references
class Homing(Base):
    method: Literal["camera_marker", "switches", "manual"]
    # camera_marker: the HOME label of bed.yaml is found in the picture (Phase C);
    # switches: $H with the controller's switches; manual: the operator puts the camera cross on HOME


class Probe(Base):
    feed: float = Field(gt=0)            # mm/min
    max_depth_mm: float = Field(gt=0)    # G38.2 searches at most this far down
    retract_mm: float = Field(ge=0)


class ZReference(Base):
    method: Literal["touch_off", "probe", "switch", "none"]
    probe: Probe | None = None

    @model_validator(mode="after")
    def _probe(self) -> ZReference:
        if self.method == "probe" and self.probe is None:
            raise ValueError("method 'probe' needs the probe: section (feed, max_depth_mm, retract_mm)")
        return self


# ------------------------------------------------------------------ head: rotation, vacuum, light
class ServoPwm(Base):
    command: str = "M3 S{us}"            # {us} = the pulse width in microseconds
    limp: str = "M5"                     # no pulses: the servo goes limp
    min_us: int = Field(ge=0)
    max_us: int
    centre_us: int
    nominal_range_deg: float | None = None   # the datasheet turn over min_us..max_us, used until the angle table
                                             # is measured (Phase D)
    settle_s: float | None = None        # wait after every turn (no feedback). null = TBD: no turns
    approach_us: int = 0                 # every angle is reached from this much shorter a pulse (gear play)
    accuracy_deg: float | None = None    # measured repeat (±): the planner plans around it

    @field_validator("command", "limp")
    @classmethod
    def _cmd(cls, v):
        return _gcode(v)

    @model_validator(mode="after")
    def _order(self) -> ServoPwm:
        if not self.min_us <= self.centre_us <= self.max_us:
            raise ValueError("need min_us <= centre_us <= max_us")
        return self


class StepperAxis(Base):
    letter: Literal["A", "B", "C"]       # the controller axis (its steps/mm setting = steps per degree)
    feed_deg_min: float = Field(gt=0)
    approach_deg: float = Field(0.0, ge=0)   # every angle is reached from this much below (the backlash; the
                                             # Setup step measures the backlash and uses at least that)
    accuracy_deg: float | None = None
    range_deg: tuple[float, float] | None = None


class Rotation(Base):
    type: Literal["none", "servo_pwm", "stepper_axis"]
    servo_pwm: ServoPwm | None = None
    stepper_axis: StepperAxis | None = None

    @model_validator(mode="after")
    def _section(self) -> Rotation:
        if self.type != "none" and getattr(self, self.type) is None:
            raise ValueError(f"type '{self.type}' needs the {self.type}: section")
        return self


class Output(Base):
    """One controller output by its role. Both lines go to the controller, which runs them after the moves
    before them (D3). The keys are start / stop because YAML reads a bare on / off as true / false."""
    start: str
    stop: str

    @field_validator("start", "stop")
    @classmethod
    def _cmd(cls, v):
        return _gcode(v)


class VacuumSensor(Base):
    input: str                           # how the controller reports it (grblHAL aux input), Phase G
    held_threshold: float | None = None


class Vacuum(Base):
    type: Literal["pump_only", "pump_valve", "pump_valve_sensor"]
    pump: Output
    valve: Output | None = None
    sensor: VacuumSensor | None = None
    release_s: float | None = None       # without a valve: wait after the pump stops (a guess, docs/13 §3.4)
    pick_dwell_s: float | None = None    # the nozzle stays down this long with the pump on (null = TBD: no picks)

    @model_validator(mode="after")
    def _parts(self) -> Vacuum:
        if self.type in ("pump_valve", "pump_valve_sensor") and self.valve is None:
            raise ValueError(f"type '{self.type}' needs valve: (start / stop)")
        if self.type == "pump_valve_sensor" and self.sensor is None:
            raise ValueError("type 'pump_valve_sensor' needs sensor:")
        return self


class Light(Base):
    via: Literal["controller", "gpio", "usb_relay"]   # never for the pump or valve (D3)
    start: str | None = None             # controller: the G / M code lines (not on / off: YAML booleans)
    stop: str | None = None
    gpio_pin: int | None = None

    @field_validator("start", "stop")
    @classmethod
    def _cmd(cls, v):
        return _gcode(v)

    @model_validator(mode="after")
    def _how(self) -> Light:
        if self.via == "controller" and (self.start is None or self.stop is None):
            raise ValueError("a controller light needs start: and stop:")
        if self.via == "gpio" and self.gpio_pin is None:
            raise ValueError("a gpio light needs gpio_pin:")
        return self


class Lights(Base):
    head: Light | None = None
    bottom: Light | None = None


# ------------------------------------------------------------------ cameras, nozzles, the rest
class Exposure(Base):
    auto: bool = False
    manual_mode: float = 1               # the V4L2 AUTO_EXPOSURE value for "manual" (1 on most UVC cameras)
    value: float | None = None           # set by Auto exposure in the page


class Camera(Base):
    driver: Literal["v4l2", "picamera2"]
    device: str
    resolution: tuple[int, int]
    fourcc: str | None = "MJPG"
    hardware_id: str                     # change it when the camera, its lens or its mount changes: the
                                         # calibrations made with the old one become stale (§4.10)
    moves_with_z: bool | None = None     # the camera sits on the Z slide (null = TBD)
    tilted: bool | None = None           # the camera does not look straight down (null = TBD)
    exposure: Exposure = Exposure()


class Cameras(Base):
    head: Camera | None = None
    bottom: Camera | None = None


class Nozzle(Base):
    id: str
    od_mm: float | None = None
    packages: list[str] = []             # which packages it picks; empty = not decided yet
    press_mm: float | None = Field(None, ge=0)   # a pick / place / mark goes this far below the touch (spring)
    spring_mm: float | None = Field(None, gt=0)  # how far the spring can be pressed (null = TBD: not checked)
    offset_guess_mm: tuple[float, float] | None = None   # where the tip is from the camera centre, roughly: the
                                                         # first ink mark is searched there (Setup, step 5)

    @model_validator(mode="after")
    def _press(self) -> Nozzle:
        if self.press_mm is not None and self.spring_mm is not None and self.press_mm >= self.spring_mm:
            raise ValueError(f"press_mm {self.press_mm} must be less than the spring's travel {self.spring_mm}")
        return self


class Nozzles(Base):
    change: Literal["manual", "rack"] = "manual"
    current: str
    items: list[Nozzle]

    @model_validator(mode="after")
    def _current(self) -> Nozzles:
        ids = [n.id for n in self.items]
        if len(set(ids)) != len(ids):
            raise ValueError(f"nozzle ids must be unique: {ids}")
        if self.current not in ids:
            raise ValueError(f"current nozzle {self.current!r} is not in items {ids}")
        return self


class EStop(Base):
    type: Literal["software", "controller_input"] = "software"


class LabelPrinter(Base):
    type: Literal["browser_print", "ptouch", "pdf_a4"] = "browser_print"
    model: str | None = None


class Job(Base):
    """How a job runs (docs/13 §4.7, §8). Defaults are the brief's targets, not hardware numbers."""
    place_tolerance_mm: float = Field(0.15, gt=0)    # a place measured farther than this is 'failed: check it'
    angle_tolerance_deg: float = Field(3.0, gt=0)
    max_pick_attempts: int = Field(2, ge=1)           # misses on one part before it is 'failed'
    home_check_every: int = Field(20, ge=0)           # parts between HOME drift checks (0 = only at the end)
    drift_limit_mm: float = Field(0.1, gt=0)          # HOME farther than this: pause, Home again
    keep_photos_jobs: int = Field(20, ge=1)           # the photos of older jobs are deleted (the reports stay)
    use_bottom_camera: Literal["always", "needed", "never"] = "always"   # with a calibrated bottom camera: look
                                                      # at every part from below, or only parts that need it


class SimCameraModel(Base):
    """The simulator's TRUE head camera (the program must measure it): a pinhole with lens bending."""
    f_px: float = Field(gt=0)
    height_mm: float = Field(gt=0)                       # optical centre above the label surface
    tilt_deg: float = 0.0                                # off straight down
    tilt_dir_deg: float = 90.0                           # the direction it leans to, in the bed plane
    turn_deg: float = 0.0                                # turned about its own axis
    dist: tuple[float, float] = (0.0, 0.0)               # k1, k2


class SimBottomCamera(Base):
    """The simulator's TRUE bottom camera (the program must measure it): fixed in the bed, looking up."""
    xy: tuple[float, float]                              # its optical axis on the bed, mm from HOME
    px_per_mm: float = Field(gt=0)                       # at the focus height
    focus_z: float                                       # the height (above the labels) that is sharp
    turn_deg: float = 0.0
    mirror: bool = True                                  # a camera looking up sees the world mirrored


class SimMachine(Base):
    """How the simulator builds this machine (`pnp sim`). Plain guesses are fine here: nothing real moves."""
    hard_ends: dict[MachineAxis, tuple[float, float]]   # the frame's ends, mm from HOME (Z: tip above the bed):
                                                        # the virtual motors lose steps beyond them
    start: dict[MachineAxis, float]                      # where the head is when the simulator starts
    camera_px_per_mm: float = 7.3                        # used when camera: is not given (straight down, 80 mm high)
    camera_turn_deg: float = 0.0
    camera: SimCameraModel | None = None
    servo_play_deg: float = 0.0
    servo_scale: float = 1.0                             # the real turn / the nominal turn (the angle table finds it)
    nozzle_offset_mm: tuple[float, float] | None = None  # the servo axis from the camera centre (null: no nozzle sim)
    nozzle_runout_mm: float = 0.0                        # the needle tip's distance from the servo axis
    nozzle_runout_phase_deg: float = 0.0
    nozzle_spring_mm: float = 2.0                        # press the nozzle's spring takes before Z loses steps
    speed: float = Field(4.0, gt=0)                      # time factor: 4 = four times faster than real
    switches: bool = False                               # the virtual machine has XY limit switches ($H)
    probe_surface_z: float | None = None                 # the probe triggers at this tip height
    bottom_camera: SimBottomCamera | None = None
    rot_scale: float = 1.0                               # stepper rotation: the real turn / the commanded turn
    rot_backlash_deg: float = 0.0                        # stepper rotation: the play between the two directions


class Profile(Base):
    schema_version: Literal[1] = SCHEMA_VERSION
    name: str
    title: str = ""
    controller: Controller
    motion: Motion
    homing: Homing
    z_reference: ZReference
    rotation: Rotation
    vacuum: Vacuum
    lights: Lights = Lights()
    cameras: Cameras = Cameras()
    nozzles: Nozzles
    estop: EStop = EStop()
    label_printer: LabelPrinter = LabelPrinter()
    job: Job = Job()
    bed_file: str = "bed.yaml"           # the bed layout (markers, labels), relative to the config dir
    sim: SimMachine | None = None

    @model_validator(mode="after")
    def _letters(self) -> Profile:
        used = {t.letter for t in self.controller.axis_map.values()}
        if self.rotation.type == "stepper_axis" and self.rotation.stepper_axis.letter in used:
            raise ValueError(f"rotation axis {self.rotation.stepper_axis.letter} is already a machine axis")
        if self.homing.method == "switches" and self.motion.travel.frame != "controller":
            raise ValueError("switch homing measures the travel in the controller frame: travel.frame: controller")
        return self
