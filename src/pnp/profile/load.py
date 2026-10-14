"""Load a profile file: YAML → migrations → schema check, with errors that name the field and the line."""
from __future__ import annotations

import hashlib
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import yaml
from pydantic import ValidationError

from .. import paths
from .migrations import migrate
from .schema import Profile


class ProfileError(ValueError):
    """The profile file can't be used. `problems` = one plain line per problem."""

    def __init__(self, path: Path, problems: list[str]) -> None:
        self.path, self.problems = path, problems
        super().__init__(f"{path.name}: " + "; ".join(problems))


def _lines(text: str) -> dict[tuple, int]:
    """YAML path (keys / list indexes) → line number (1-based), to point at the field in an error."""
    out: dict[tuple, int] = {}

    def walk(node, path: tuple) -> None:
        out[path] = node.start_mark.line + 1
        if isinstance(node, yaml.MappingNode):
            for k, v in node.value:
                key = k.value
                out[path + (key,)] = k.start_mark.line + 1
                walk(v, path + (key,))
        elif isinstance(node, yaml.SequenceNode):
            for i, v in enumerate(node.value):
                walk(v, path + (i,))

    try:
        root = yaml.compose(text)
    except yaml.YAMLError:
        return out
    if root is not None:
        walk(root, ())
    return out


def parse(text: str, path: Path = Path("profile.yaml")) -> Profile:
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as err:
        raise ProfileError(path, [f"not valid YAML: {err}"]) from None
    if not isinstance(raw, dict):
        raise ProfileError(path, ["the file must be a YAML mapping (key: value lines)"])
    try:
        raw = migrate(raw)
    except ValueError as err:
        raise ProfileError(path, [str(err)]) from None
    try:
        return Profile.model_validate(raw)
    except ValidationError as err:
        where = _lines(text)
        problems = []
        for e in err.errors():
            loc = tuple(x for x in e["loc"] if not (isinstance(x, str) and x.startswith(("function-", "tuple["))))
            line = next((where[loc[:n]] for n in range(len(loc), -1, -1) if loc[:n] in where), None)
            name = ".".join(str(x) for x in loc) or "(top)"
            msg = e["msg"].removeprefix("Value error, ")
            problems.append(f"{name}: {msg}" + (f" (line {line})" if line else ""))
        raise ProfileError(path, problems) from None


def load(path: Path) -> Profile:
    try:
        text = path.read_text()
    except OSError as err:
        raise ProfileError(path, [f"can't read it: {err.strerror}"]) from None
    return parse(text, path)


def resolve(name_or_path: str | None, cfg_dir: Path | None = None) -> Path:
    """A profile by name (config/profiles/<name>.yaml) or by path. None: the only profile there is."""
    pdir = paths.profiles_dir(cfg_dir)
    if name_or_path:
        p = Path(name_or_path)
        if p.suffix in (".yaml", ".yml") and p.is_file():
            return p
        q = pdir / f"{name_or_path}.yaml"
        if q.is_file():
            return q
        raise FileNotFoundError(f"No profile {name_or_path!r}. Profiles: {', '.join(available(cfg_dir)) or 'none'} "
                                f"(in {pdir})")
    names = available(cfg_dir)
    if len(names) == 1:
        return pdir / f"{names[0]}.yaml"
    raise FileNotFoundError(f"Choose a profile with --profile: {', '.join(names) or 'none found in ' + str(pdir)}")


def available(cfg_dir: Path | None = None) -> list[str]:
    pdir = paths.profiles_dir(cfg_dir)
    return sorted(p.stem for p in pdir.glob("*.yaml")) if pdir.is_dir() else []


def digest(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()[:16]
    except OSError:
        return ""


@dataclass
class Change:
    """What a reload changed, and what it needs before it takes effect."""
    fields: list[str]
    reconnect: bool        # controller type / port / baud / axis map: used at the next Connect
    rehome: bool           # the axis map, the travel frame or the homing method: the position is not trusted


RECONNECT = ("controller.type", "controller.port", "controller.baud", "controller.axis_map")
REHOME = ("controller.axis_map", "motion.travel.frame", "homing.method")


def diff(old: Profile, new: Profile) -> Change:
    def flat(d, pre=""):
        out = {}
        for k, v in d.items():
            key = f"{pre}{k}"
            if isinstance(v, dict) and k != "axis_map":
                out.update(flat(v, key + "."))
            else:
                out[key] = v
        return out

    a, b = flat(old.model_dump()), flat(new.model_dump())
    fields = sorted(k for k in a.keys() | b.keys() if a.get(k) != b.get(k))
    return Change(fields, any(f.startswith(RECONNECT) for f in fields), any(f.startswith(REHOME) for f in fields))


class Watcher:
    """Checks the profile file every `period` seconds; calls on_change(profile | ProfileError) when it changed.
    A file with errors never replaces the running profile: the caller keeps the old one and shows the errors."""

    def __init__(self, path: Path, on_change: Callable[[Profile | ProfileError], None], period: float = 1.0) -> None:
        self.path, self.on_change, self.period = path, on_change, period
        self.last = digest(path)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="profile-watcher", daemon=True)

    def start(self) -> Watcher:
        self._thread.start()
        return self

    def check(self) -> bool:
        """One check now. True if the file changed."""
        d = digest(self.path)
        if d == self.last or not d:
            return False
        self.last = d
        try:
            self.on_change(load(self.path))
        except ProfileError as err:
            self.on_change(err)
        return True

    def _run(self) -> None:
        while not self._stop.wait(self.period):
            self.check()

    def stop(self) -> None:
        self._stop.set()
