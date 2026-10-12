#!/usr/bin/env python3
"""The pick and place page: live camera, jog, set-up steps, feeders and the job, in the web browser.

    .venv/bin/python ui/app.py               # opens http://127.0.0.1:8000 in the browser
    .venv/bin/python ui/app.py --lan         # also from a phone / tablet on the same network
    .venv/bin/python ui/app.py --no-servo    # the servo is not wired

Close it with the Quit button (the nozzle goes up first) or Ctrl+C in the terminal.
The engine is ui/engine.py (the bench machine with every lock: see docs/07-software.md "Web UI").
"""
from __future__ import annotations

import argparse
import contextlib
import datetime
import re
import sys
import threading
import time
import webbrowser
from pathlib import Path

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ui import engine as e  # noqa: E402

STATIC = Path(__file__).resolve().parent / "static"
FPS = 12


def jpeg(img: np.ndarray, quality: int = 75) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else b""


def mjpeg_frames(m, cam, stop: threading.Event | None = None, limit: int | None = None):
    """multipart/x-mixed-replace stream of the live picture with the overlays."""
    last, n = 0.0, 0
    while (stop is None or not stop.is_set()) and (limit is None or n < limit):
        item = cam.latest()
        if item is None or item[0] == last:
            time.sleep(0.02)
            continue
        last = item[0]
        try:
            img = m.render(item[1])
        except Exception:                             # an overlay problem must not stop the picture
            img = item[1]
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg(img) + b"\r\n"
        n += 1
        time.sleep(1.0 / FPS)


def actions(m) -> dict:
    """Page buttons -> (job, how its arguments come from the request). Every job runs on the machine's thread."""
    return {
        "home": (m.job_start, []), "jog": (m.job_jog, ["dx", "dy"]), "jog_z": (m.job_jog_z, ["dz"]),
        "pump": (m.job_pump, []), "servo_centre": (m.job_centre, []),
        "height": (m.job_touch, ["name"]), "camera_height": (m.job_camera_height, []),
        "offset": (m.job_nozzle_point, []), "click": (m.job_goto_px, ["u", "v"]),
        "reach_start": (m.job_reach_start, []), "reach_save": (m.job_reach_save, []),
        "reach_cancel": (m.job_reach_cancel, []),
        "corner_go": (m.job_goto_corner, ["hole"]), "corner_centre": (m.job_centre_hole, []),
        "corner_set": (m.job_set_corner, ["hole"]), "board_fit": (m.job_fit_board, []),
        "board_auto": (m.job_auto_board, []), "run": (m.job_run, ["mode"]), "new_board": (m.job_new_board, []),
        "goto": (m.job_goto_xy, ["x", "y"]), "turn": (m.job_turn, ["delta"]), "turn_to": (m.job_turn_to, ["deg"]),
        "land": (m.job_land, ["surface"]), "land_cancel": (m.job_land_cancel, []),
        "pick_here": (m.job_pick_here, []), "run_one": (m.job_run_one, []), "survey": (m.job_bed_survey, []), "nozzle_pick": (m.job_nozzle_by_picking, []), "spot": (m.job_set_spot, ["name"]), "marks": (m.job_nozzle_marks, []), "write_e": (m.job_write_e, ["x", "y"]), "dot": (m.job_dot, ["x", "y"]), "pointer_target": (m.job_pointer_target, []), "pointer_go": (m.job_pointer_go, []),
        "pointer_nudge": (m.job_pointer_nudge, ["dx", "dy"]), "pointer_save": (m.job_pointer_save, []),
        "pointer_cancel": (m.job_pointer_cancel, []), "exposure": (m.job_auto_exposure, []), "exposure_set": (m.job_set_exposure, ["value"]),
    }


def create_app(m, cam, on_quit=None, streams_stop: threading.Event | None = None) -> FastAPI:
    streams_stop = streams_stop or threading.Event()   # set at shutdown: the live streams end (else the
                                                        # program waits for the browser forever, 2026-10-08)

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        yield
        streams_stop.set()

    app = FastAPI(title="Pick and place", lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=STATIC), name="static")
    table = actions(m)

    @app.get("/")
    def index():
        return FileResponse(STATIC / "index.html")

    @app.get("/video")
    def video():
        return StreamingResponse(mjpeg_frames(m, cam, streams_stop), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/status")
    def status():
        return m.status()

    @app.post("/api/action/{name}")
    def action(name: str, body: dict | None = None):
        if name not in table:
            raise HTTPException(404, f"no action {name}")
        fn, keys = table[name]
        body = body or {}
        try:
            args = [body[k] for k in keys]
        except KeyError as k:
            raise HTTPException(400, f"missing {k}")
        if name == "jog":
            args = [(float(args[0]), float(args[1]))]
        elif name in ("jog_z",):
            args = [float(args[0])]
        elif name in ("click", "goto", "write_e", "dot", "pointer_nudge"):
            args = [float(args[0]), float(args[1])]
        elif name in ("turn", "turn_to", "exposure_set"):
            args = [float(args[0])]
        elif name == "land" and args[0] not in ("part", "place"):
            raise HTTPException(400, "surface: part or place")
        elif name == "height" and args[0] not in e.HEIGHT_NAMES:
            raise HTTPException(400, "height: part, place, move or paper")
        elif name in ("corner_go", "corner_set") and args[0] not in e.d.HOLES:
            raise HTTPException(400, "hole: H1 H2 H3 H4")
        elif name == "spot" and args[0] not in ("ink", "paper"):
            raise HTTPException(400, "spot: ink or paper")
        elif name == "run" and args[0] not in ("dry", "real"):
            raise HTTPException(400, "mode: dry or real")
        if not m.submit(fn, *args):
            return {"ok": False, "why": "The machine is busy: wait, or press STOP"}
        return {"ok": True}

    @app.post("/api/stop")
    def stop():
        m.pause_flag.clear()
        m.stop_flag.set()
        m.say("STOP pressed: stopping after the current move")
        return {"ok": True}

    @app.post("/api/pause")
    def pause():
        m.pause_flag.set()
        m.say("Pause: the job stops after this part")
        return {"ok": True}

    @app.post("/api/resume")
    def resume():
        m.pause_flag.clear()
        return {"ok": True}

    @app.get("/api/map")
    def bed_map():
        return m.map_data()

    @app.get("/api/lines")
    def lines():
        return {"side": m.args.side, "top_edge": m.args.top_edge, "slots": e.ml.slot_numbers(m.bed),
                "lines": [{k: g[k] for k in ("key", "label", "value", "package", "count", "refs", "supported", "why",
                                             "values")} for g in m.lines()],
                "assigned": {str(s): a for s, a in m.feeder["slots"].items()}}

    @app.post("/api/feeder")
    def feeder(body: dict):
        if m.run_state != "idle":
            return {"ok": False, "why": "Not while the job runs"}
        try:
            m.assign(int(body["slot"]), body.get("key"))
        except (ValueError, KeyError) as err:
            return {"ok": False, "why": str(err)}
        return {"ok": True}

    @app.post("/api/side")
    def side(body: dict):
        if m.busy or m.run_state != "idle":
            return {"ok": False, "why": "The machine is busy"}
        try:
            m.set_side(body["side"], body["top_edge"])
        except (ValueError, KeyError) as err:
            return {"ok": False, "why": str(err)}
        m.say(f"Board: {m.args.side} side up, KiCad top edge at the {m.args.top_edge}. Register the board again")
        return {"ok": True}

    @app.get("/api/pictures")
    def pictures():
        files = sorted(m.out_dir.glob("*.png"), key=lambda p: p.stat().st_mtime, reverse=True)[:12]
        return [p.name for p in files]

    @app.get("/pictures/{name}")
    def picture(name: str):
        if not re.fullmatch(r"[\w.\-]+\.png", name) or not (m.out_dir / name).is_file():
            raise HTTPException(404)
        return FileResponse(m.out_dir / name)

    @app.post("/api/quit")
    def quit_():
        if on_quit is None:
            return JSONResponse({"ok": False}, 400)
        streams_stop.set()
        threading.Timer(0.3, on_quit).start()
        return {"ok": True}

    return app


def main() -> None:
    import uvicorn

    import camera_rectify as cr
    from bench import Grbl, load_bench, mark_parked, was_parked
    from smd_identify import DEFAULT_CAMERA

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--lan", action="store_true", help="also reachable from other devices on the network")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--no-servo", action="store_true", help="the servo is not wired: never turn it")
    ap.add_argument("--at-pick", action="store_true",
                    help="the program starts with the nozzle tip ON a part in a slot (the pick height): that spot is "
                         "the pick (height + offset, after Home and one click on the part)")
    ap.add_argument("--pick-spot", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="the pick spot in Grbl's machine axes (mm), when Grbl was NOT reset since the nozzle tip was "
                         "on that part (the port reopened without a reset, stty -hupcl): checked against Grbl's position")
    ap.add_argument("--pick-spot-from-here", nargs=3, type=float, metavar=("DX", "DY", "DZ"),
                    help="the pick spot is this far (mm, machine axes) from where the head is now: Grbl was reset but "
                         "the head did not move (checked on the picture)")
    ap.add_argument("--no-home", action="store_true", help="do not press Home by itself at the start")
    ap.add_argument("--z-top", action="store_true",
                    help="Z is at its TOP end (the owner put it there): the nozzle is clear of everything, so no "
                         "lock, no lift into the end, and no Home at start (use the Bed survey or Home)")
    args = ap.parse_args()
    bed = e.ml.load_bed()
    labels = e.ml.make_all(bed)
    nominal = {name: np.array(c, float) for name, c, _ in e.ml.label_placements(bed, labels)}
    out_dir = ROOT / "logs" / "pnp" / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    cam = cr.open_camera(DEFAULT_CAMERA)
    camcfg = load_bench().get("camera") or {}
    if camcfg.get("exposure") is not None and hasattr(cam, "set_control"):   # the exposure set for this light
        cam.set_control("AUTO_EXPOSURE", float(camcfg.get("manual_mode", 1)))
        cam.set_control("EXPOSURE", float(camcfg["exposure"]))
    parked = was_parked()                         # before the port opens (that marks it "not parked")
    grbl = Grbl(cfg=load_bench())
    grbl.check_ready()
    m = e.PnPMachine(grbl, cam, e.t.Markers(bed), nominal, out_dir, e.d.load_board(), bed)
    m.feed = float(load_bench()["limits"]["max_feed"])
    m.no_servo = args.no_servo
    m.started_parked = parked
    if args.pick_spot:
        x, y, z = args.pick_spot
        st = grbl.status()[1]
        if max(abs(st["X"]), abs(st["Y"]), abs(st["Z"])) < 1e-3:
            grbl.close()
            cam.close()
            sys.exit("Grbl reports 0 0 0: it was reset (the pick spot is lost). Put the nozzle tip on the part again "
                     "and start with --at-pick")
        m.pos = np.array([st["X"], st["Y"]], float)      # Grbl kept its position: so does the program
        m.pick_mark = {"xy": np.array([x, y], float), "grbl_z": float(z), "same_as_ref": True}
        m.say(f"Pick spot kept from before (Grbl X{x:g} Y{y:g} Z{z:g}; now at X{st['X']:.2f} Y{st['Y']:.2f} "
              f"Z{st['Z']:.2f}): press Home")
    if args.pick_spot_from_here:
        st = grbl.status()[1]
        dx, dy, dz = args.pick_spot_from_here
        m.pos = np.array([st["X"], st["Y"]], float)
        m.pick_mark = {"xy": m.pos + (dx, dy), "grbl_z": st["Z"] + dz, "same_as_ref": True}
        m.say(f"Pick spot: X{dx:+g} Y{dy:+g} Z{dz:+g} mm from here: press Home")
    if args.at_pick:
        m.pick_mark = {"xy": m.pos.copy(), "grbl_z": 0.0}      # Grbl starts at 0 where the head is: the pick spot
        m.say("Started at the pick spot: press Home (it lifts first), then click that part")
    m.z_at_top = args.z_top
    if args.z_top:
        m.say("Z is at its TOP end (you said so): nothing is locked. Setup 0: Bed survey, or Home")
    elif not parked:
        m.z_locked = "The last session did not end cleanly: the nozzle may be down"
        m.say("LOCKED: the last session did not end cleanly, the nozzle may be down. Lift it with Z+ until it is "
              "clear of everything, then press Home")
    m.start()
    if not m.z_locked and not args.z_top and not args.no_home:
        m.submit(m.job_start)

    host = "0.0.0.0" if args.lan else "127.0.0.1"
    server = uvicorn.Server(uvicorn.Config(create_app(m, cam, on_quit=lambda: setattr(server, "should_exit", True)),
                                           host=host, port=args.port, log_level="warning",
                                           timeout_graceful_shutdown=3))
    url = f"http://127.0.0.1:{args.port}"
    print(f"Pick and place page: {url}  (Quit button or Ctrl+C to close)", flush=True)
    if not args.no_browser:
        threading.Timer(1.5, lambda: webbrowser.open(url)).start()
    parked_now = False
    try:
        server.run()
    finally:
        m.stop_flag.set()
        m.pause_flag.clear()
        t0 = time.monotonic()
        while m.busy and time.monotonic() - t0 < 60:
            time.sleep(0.05)
        try:
            if m.busy:
                print("The machine is still busy: Z NOT lifted. The next start is locked.", flush=True)
            else:
                try:
                    parked_now = m.park()
                finally:
                    if m.pump_on:
                        print("The pump goes off now: a part on the nozzle drops here", flush=True)
                    try:
                        m.pump(False)
                    except Exception:
                        pass
        finally:
            mark_parked(parked_now, "ui/app.py")
            grbl.close()
            cam.close()
            print("Bye." if parked_now else "Bye (not parked: the next start is locked until you lift Z and press Home).",
                  flush=True)


if __name__ == "__main__":
    main()
