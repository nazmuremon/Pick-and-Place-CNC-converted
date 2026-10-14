"""pnp: the pick and place program (docs/13).

    pnp serve  [--profile NAME] [--lan]     the machine process and its page (http://127.0.0.1:8000)
    pnp sim    [--profile NAME]             the same page against the simulator: nothing real moves
    pnp doctor [--profile NAME]             the checks, in the terminal
    pnp portd  [--device DEV]               hold the USB port open (restarts never reset an Uno)
    pnp profile list | check [NAME]
    pnp import FILE... [--save]             import a board (KiCad, Gerber X3, centroid) and show what was read
    pnp init                                the first run: a few questions, then a profile (docs/user/install.md)
    pnp export OUT.zip [--photos]           everything that makes this machine this machine, in one zip
    pnp restore IN.zip                      unpack a bundle (what it replaces is kept in a zip first)

The chosen profile is remembered (data dir, active_profile.txt).
"""
from __future__ import annotations

import argparse
import sys
import threading
import webbrowser
from pathlib import Path

from . import __version__, paths
from . import profile as prof

ACTIVE = "active_profile.txt"


def _pick_profile(name: str | None, cfg_dir: Path | None) -> Path:
    remembered = paths.data_dir() / ACTIVE
    if name is None and remembered.is_file():
        name = remembered.read_text().strip() or None
        try:
            return prof.resolve(name, cfg_dir)
        except FileNotFoundError:
            name = None
    p = prof.resolve(name, cfg_dir)
    remembered.write_text(p.stem if p.parent == paths.profiles_dir(cfg_dir) else str(p))
    return p


def cmd_serve(a: argparse.Namespace) -> int:
    import uvicorn

    from .machine import Machine
    from .ui.server import create_app

    try:
        path = _pick_profile(a.profile, a.config)
        m = Machine(path, sim=a.sim, cfg_dir=a.config, sim_speed=a.sim_speed)
    except (FileNotFoundError, prof.ProfileError) as err:
        print(err, file=sys.stderr)
        return 2
    if not a.no_connect:
        m.connect()
    host = "0.0.0.0" if a.lan else a.host
    url = f"http://127.0.0.1:{a.port}"
    print(f"{'SIMULATOR' if a.sim else 'Machine'} page: {url}   profile: {m.profile.name}   (Ctrl+C to stop)",
          flush=True)
    if a.open:
        threading.Timer(1.5, lambda: webbrowser.open(url)).start()
    server = uvicorn.Server(uvicorn.Config(create_app(m), host=host, port=a.port, log_level="warning",
                                           timeout_graceful_shutdown=3))
    try:
        server.run()
    finally:
        m.close()
    return 0


def cmd_doctor(a: argparse.Namespace) -> int:
    from . import doctor
    try:
        path = _pick_profile(a.profile, a.config)
        p = prof.load(path)
    except (FileNotFoundError, prof.ProfileError) as err:
        print(f"[FAIL] Profile: {err}")
        return 1
    m = None
    if a.connect:
        from .machine import Machine
        m = Machine(path, cfg_dir=a.config, watch=False)
        m.connect()
        m.worker.wait(15)
    try:
        checks = doctor.run(p, path, m, cfg_dir=a.config, data_dir=paths.data_dir())
    finally:
        if m is not None:
            m.close()
    bad = 0
    for c in checks:
        mark = {True: " OK ", False: "FAIL", None: "note"}[c.ok]
        bad += c.ok is False
        print(f"[{mark}] {c.name}: {c.detail}" + (f"\n         -> {c.fix}" if c.fix and c.ok is not True else ""))
    return 1 if bad else 0


def cmd_portd(a: argparse.Namespace) -> int:
    from .motion.portd import serve
    host, _, port = a.listen.partition(":")
    try:
        serve(a.device, a.baud, host or "127.0.0.1", int(port or 7450))
    except KeyboardInterrupt:
        pass
    except OSError as err:
        print(f"Port keeper: {err}", file=sys.stderr)
        return 1
    return 0


def cmd_profile(a: argparse.Namespace) -> int:
    if a.what == "list":
        for n in prof.available(a.config):
            print(n)
        return 0
    try:
        path = prof.resolve(a.name, a.config)
        p = prof.load(path)
    except (FileNotFoundError, prof.ProfileError) as err:
        print(err)
        if isinstance(err, prof.ProfileError):
            for line in err.problems:
                print(f"  - {line}")
        return 1
    print(f"{p.name}: valid ({path})")
    for mi in prof.missing(p):
        print(f"  TBD {mi.field}: {mi.effect}. {mi.fix}")
    return 0


def cmd_import(a: argparse.Namespace) -> int:
    from .board.service import BoardService
    svc = BoardService(paths.data_dir() / "boards", a.config or paths.config_dir())
    iid = svc.new_import()
    try:
        for f in a.files:
            svc.add_file(iid, f.name, f.read_bytes())
        r = svc.run(iid, fit=not a.no_fit, use_current=not a.no_reference, name=a.name)
    except (OSError, ValueError, KeyError) as err:
        print(f"Import failed: {err}", file=sys.stderr)
        return 1
    b = r["board"]
    print(f"{r['kind']}: {b['components']} parts, {b['smd']} SMD ({b['smd_top']} top, {b['smd_bottom']} bottom), "
          f"{b['tht']} through-hole, {b['holes']} holes, {b['size'][0]} x {b['size'][1]} mm, {b['thickness']} mm thick")
    print("What happens to the parts:", ", ".join(f"{v} {k}" for k, v in sorted(r["classes"].items())))
    for line in (r.get("fit") or {}).get("report", []):
        print("  fit:", line)
    for line in (r.get("sniff") or {}).get("unsure", []):
        print("  check:", line)
    for n in r["notes"]:
        print("  note:", n)
    for i in r["issues"]:
        print(f"  {i['level']}: {i['text']}")
    if a.save:
        print("Saved as the current board:", svc.save(iid, a.name))
    return 0


def cmd_init(a: argparse.Namespace) -> int:
    from . import firstrun
    cfg = a.config or paths.config_dir()
    try:
        out = firstrun.run(cfg)
    except (prof.ProfileError, ValueError) as err:
        print(f"The profile is not valid: {err}", file=sys.stderr)
        return 1
    print(f"Profile written: {out}\nNext: pnp doctor --profile {out.stem}, then pnp serve --profile {out.stem} --open")
    return 0


def cmd_export(a: argparse.Namespace) -> int:
    from . import bundle
    a.out.write_bytes(bundle.export(a.config or paths.config_dir(), paths.data_dir(), a.photos))
    print(f"Written: {a.out} ({a.out.stat().st_size // 1024} kB)")
    return 0


def cmd_restore(a: argparse.Namespace) -> int:
    from . import bundle
    try:
        r = bundle.restore(a.file.read_bytes(), a.config or paths.config_dir(), paths.data_dir())
    except (ValueError, OSError) as err:
        print(f"Not restored: {err}", file=sys.stderr)
        return 1
    print(f"Restored {r['files']} files (bundle of {r['created']}, version {r['from_version']}). "
          f"What was here before: {r['backup']}")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="pnp", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", action="version", version=__version__)
    ap.add_argument("--config", type=Path, default=None, help="the config folder (default: config/ or ~/.config/pnp)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, sim in (("serve", False), ("sim", True)):
        s = sub.add_parser(name, help="the page" + (" against the simulator" if sim else ""))
        s.add_argument("--profile", help="profile name (config/profiles/<name>.yaml) or a file")
        s.add_argument("--host", default="127.0.0.1")
        s.add_argument("--port", type=int, default=8000)
        s.add_argument("--lan", action="store_true", help="also from a phone / tablet on the network")
        s.add_argument("--open", action="store_true", help="open the browser")
        s.add_argument("--no-connect", action="store_true", help="do not connect to the controller at start")
        s.add_argument("--sim-speed", type=float, default=None, help="simulator time factor")
        s.set_defaults(fn=cmd_serve, sim=sim)
    d = sub.add_parser("doctor", help="run the checks")
    d.add_argument("--profile")
    d.add_argument("--connect", action="store_true", help="also connect to the controller (not while serve runs)")
    d.set_defaults(fn=cmd_doctor)
    pd = sub.add_parser("portd", help="hold the USB port open (port keeper)")
    pd.add_argument("--device", default=None, help="serial device (default: the first USB serial port)")
    pd.add_argument("--baud", type=int, default=115200)
    pd.add_argument("--listen", default="127.0.0.1:7450")
    pd.set_defaults(fn=cmd_portd)
    pr = sub.add_parser("profile", help="list or check profiles")
    pr.add_argument("what", choices=["list", "check"])
    pr.add_argument("name", nargs="?")
    pr.set_defaults(fn=cmd_profile)
    im = sub.add_parser("import", help="import a board and show what was read")
    im.add_argument("files", nargs="+", type=Path)
    im.add_argument("--name")
    im.add_argument("--save", action="store_true", help="make it the current board")
    im.add_argument("--no-fit", action="store_true", help="centroid: do not fit the settings to a reference")
    im.add_argument("--no-reference", action="store_true", help="centroid: do not use the current board as reference")
    im.set_defaults(fn=cmd_import)
    it = sub.add_parser("init", help="first run: questions -> a profile")
    it.set_defaults(fn=cmd_init)
    ex = sub.add_parser("export", help="the export bundle (zip)")
    ex.add_argument("out", type=Path)
    ex.add_argument("--photos", action="store_true", help="also the job photos (big)")
    ex.set_defaults(fn=cmd_export)
    rs = sub.add_parser("restore", help="unpack an export bundle")
    rs.add_argument("file", type=Path)
    rs.set_defaults(fn=cmd_restore)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
