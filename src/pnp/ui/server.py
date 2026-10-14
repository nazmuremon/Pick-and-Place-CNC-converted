"""The page's server (docs/13 §4.9): FastAPI on the machine, one page in the browser.

- Static files are read from disk on every request and the page links them with a version from their
  content: editing the page's code needs only a browser reload, never a program restart.
- /ws pushes the status (5 per second) and every event; the page reconnects by itself.
- Requests from another web site are refused (Origin must match Host): only this page may move the machine.
"""
from __future__ import annotations

import asyncio
import contextlib
import hashlib
import json
import time
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, PlainTextResponse, Response, StreamingResponse

from .. import doctor
from ..machine import Machine

STATIC = Path(__file__).resolve().parent / "static"
STATUS_PERIOD = 0.2
VIDEO_FPS = 12


def asset_version(static: Path = STATIC) -> str:
    h = hashlib.sha256()
    for p in sorted(static.glob("*")):
        if p.is_file():
            h.update(p.name.encode())
            h.update(p.read_bytes())
    return h.hexdigest()[:10]


def _same_origin(origin: str | None, host: str | None) -> bool:
    if not origin:
        return True                                    # not a browser cross-site request (curl, tests)
    return urlparse(origin).netloc == (host or "")


def _num(body: dict, key: str, default=None) -> float:
    v = body.get(key, default)
    if v is None:
        raise HTTPException(400, f"missing {key}")
    try:
        return float(v)
    except (TypeError, ValueError):
        raise HTTPException(400, f"{key} must be a number") from None


def actions(m: Machine) -> dict:
    """Page buttons → machine calls. Each returns {"ok": ...} at once (long work runs on the machine's worker)."""
    return {
        "connect": lambda b: m.connect(),
        "disconnect": lambda b: m.disconnect(),
        "jog": lambda b: m.jog(_num(b, "dx", 0), _num(b, "dy", 0), _num(b, "dz", 0)),
        "goto": lambda b: m.goto(_num(b, "x"), _num(b, "y")),
        "home": lambda b: m.home(),
        "set_z": lambda b: m.set_z(_num(b, "surface_z", 0), str(b.get("note", ""))[:80]),
        "unlock_alarm": lambda b: m.unlock_alarm(),
        "confirm_clear": lambda b: m.confirm_clear(),
        "resume": lambda b: m.resume(),
        "turn": lambda b: m.turn_to(_num(b, "deg")),
        "servo_limp": lambda b: m.servo_limp(),
        "pump": lambda b: m.pump(bool(b.get("on"))),
        "light": lambda b: m.light(str(b.get("role", "")), bool(b.get("on"))),
        "sim": lambda b: m.sim_event(str(b.get("event", "")), None if b.get("n") in (None, "") else int(b["n"])),
        "add_tray": lambda b: m.jobs.add_tray(int(_num(b, "number")), int(_num(b, "tape_w", 8)), str(b.get("kind") or "paper"),
                                              _num(b, "zone_mm", 60), str(b.get("type") or "holder"),
                                              None if b.get("x") in (None, "") else _num(b, "x"),
                                              None if b.get("y") in (None, "") else _num(b, "y"),
                                              **{k: (int(_num(b, k)) if k in ("rows", "cols") else _num(b, k))
                                                 for k in ("rows", "cols", "pitch_u", "pitch_v", "first_u",
                                                           "tape_floor_mm", "top_mm")
                                                 if b.get(k) not in (None, "")
                                                 and str(b.get("type")) in ("matrix", "stick")}),
        "remove_tray": lambda b: m.jobs.remove_tray(int(_num(b, "number"))),
        "load_tray": lambda b: m.jobs.load_tray(int(_num(b, "number")), str(b.get("line", "")), str(b.get("scanned", ""))),
        "refilled": lambda b: m.jobs.refilled(int(_num(b, "number"))),
        "job_start": lambda b: m.jobs.start(str(b.get("mode") or "real"), int(_num(b, "n_picks", 100)),
                                            bool(b.get("override")), b.get("refs") or None),
        "job_resume": lambda b: m.jobs.resume(),
        "job_answer": lambda b: m.jobs.answer(str(b.get("choice", ""))),
        "job_abort": lambda b: m.jobs.abort(),
        "nozzle_changed": lambda b: m.jobs.nozzle_changed(str(b.get("nozzle", ""))),
        "scan": lambda b: m.parts.scan(str(b.get("text", "")), None if b.get("tray") in (None, "") else int(b["tray"])),
        "link": lambda b: m.parts.link(str(b.get("text", "")), str(b.get("key", ""))),
        "drawer_move": lambda b: m.parts.move(str(b.get("key", "")), int(_num(b, "drawer"))),
        "drawer_count": lambda b: m.parts.set_count(int(_num(b, "count"))),
        "hand_mark": lambda b: m.parts.mark([str(r) for r in b.get("refs", [])], bool(b.get("placed", True))),
        "print_labels": lambda b: m.parts.print_labels(str(b.get("which") or "drawers"), str(b.get("mode") or "strip")),
        "import_solder_helper": lambda b: m.parts.import_old(b.get("data") or {}),
        "auto_exposure": lambda b: m.auto_exposure(),
        "calibrate": lambda b: m.calibrate(str(b.get("step", "")), None if b.get("block_mm") in (None, "")
                                           else _num(b, "block_mm")),
        "click": lambda b: m.click(_num(b, "u"), _num(b, "v")),
        "register": lambda b: m.register_board(str(b.get("side", "")), str(b.get("top_edge", "")),
                                               str(b.get("copy") or "1"),
                                               None if b.get("x") in (None, "") else _num(b, "x"),
                                               None if b.get("y") in (None, "") else _num(b, "y")),
        "exposure": lambda b: m.set_exposure(None if b.get("value") in (None, "") else _num(b, "value")),
        "setup": lambda b: m.setup_action(str(b.get("action", "")), {k: v for k, v in b.items() if k != "action"}),
        "undo": lambda b: m.undo(str(b.get("key", ""))),
    }


def jpeg(img: np.ndarray, quality: int = 75) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else b""


def create_app(m: Machine, static: Path = STATIC) -> FastAPI:
    stopping = asyncio.Event()

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        yield
        stopping.set()                                 # the live streams end, so the server can stop

    app = FastAPI(title="Pick and place", lifespan=lifespan)
    table = actions(m)

    @app.middleware("http")
    async def guard(request: Request, call_next):
        if request.method != "GET" and not _same_origin(request.headers.get("origin"), request.headers.get("host")):
            return JSONResponse({"ok": False, "why": "Requests from other web sites are refused"}, 403)
        return await call_next(request)

    @app.get("/")
    def index():
        html = (static / "index.html").read_text().replace("__V__", asset_version(static))
        return HTMLResponse(html, headers={"Cache-Control": "no-store"})

    @app.get("/static/{name}")
    def asset(name: str):
        p = static / name
        if "/" in name or ".." in name or not p.is_file():
            raise HTTPException(404)
        return FileResponse(p, headers={"Cache-Control": "no-cache"})

    @app.get("/vendor/{name}")
    def vendor(name: str):
        """The QR decoders (ZXing WASM, jsQR): shipped, no CDN (works offline on the Pi)."""
        p = static / "vendor" / name
        if "/" in name or ".." in name or not p.is_file():
            raise HTTPException(404)
        kind = "application/wasm" if name.endswith(".wasm") else None
        return FileResponse(p, media_type=kind, headers={"Cache-Control": "max-age=86400"})

    @app.get("/api/status")
    def status():
        return m.status()

    @app.get("/api/map")
    def bed_map():
        return m.map_data()

    @app.get("/api/events")
    def events(n: int = 200):
        return m.events.tail(max(1, min(n, 200)))

    @app.get("/api/profile")
    def profile():
        try:
            text = m.profile_path.read_text()
        except OSError as err:
            text = f"(can't read: {err})"
        return {"file": str(m.profile_path), "text": text, "status": m.status()["profile"],
                "capabilities": m.caps.describe()}

    @app.get("/api/doctor")
    def run_doctor():
        return [c.as_dict() for c in doctor.run(m.profile, m.profile_path, m, cfg_dir=m.cfg_dir, data_dir=m.data)]

    @app.post("/api/stop")
    def stop():
        return m.stop()

    @app.post("/api/action/{name}")
    async def action(name: str, request: Request):
        if name not in table:
            raise HTTPException(404, f"no action {name}")
        try:
            body = await request.json()
        except (json.JSONDecodeError, ValueError):
            body = {}
        if not isinstance(body, dict):
            raise HTTPException(400, "the body must be a JSON object")
        return table[name](body)

    @app.get("/video")
    def video():
        cam = m.cameras.get("camera_head")
        if cam is None or not hasattr(cam, "latest"):
            raise HTTPException(404, "no head camera")

        def frames():
            last = 0.0
            while not stopping.is_set():
                item = cam.latest()
                if item is None or item[0] == last:
                    time.sleep(0.02)
                    continue
                last = item[0]
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg(item[1]) + b"\r\n"
                time.sleep(1.0 / VIDEO_FPS)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/snapshot.jpg")
    def snapshot():
        cam = m.cameras.get("camera_head")
        item = cam.latest() if cam is not None and hasattr(cam, "latest") else None
        if item is None:
            raise HTTPException(404, "no picture")
        return StreamingResponse(iter([jpeg(item[1], 90)]), media_type="image/jpeg")

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        if not _same_origin(sock.headers.get("origin"), sock.headers.get("host")):
            await sock.close(code=1008)
            return
        await sock.accept()
        loop = asyncio.get_running_loop()
        q: asyncio.Queue = asyncio.Queue(maxsize=500)

        def on_event(ev: dict) -> None:
            loop.call_soon_threadsafe(lambda: q.full() or q.put_nowait(ev))

        m.events.listeners.append(on_event)
        try:
            await sock.send_json({"type": "hello", "version": asset_version(static), "events": m.events.tail(200)})
            while not stopping.is_set():
                await sock.send_json({"type": "status", "status": m.status()})
                try:
                    while True:
                        ev = await asyncio.wait_for(q.get(), timeout=STATUS_PERIOD)
                        await sock.send_json({"type": "event", "event": ev})
                        if q.empty():
                            break
                except asyncio.TimeoutError:
                    pass
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            if on_event in m.events.listeners:
                m.events.listeners.remove(on_event)

    # ---------------------------------------------------------------- boards (Phase B)
    def _board_call(fn):
        try:
            return fn()
        except (KeyError, ValueError, OSError) as err:
            return JSONResponse({"ok": False, "why": str(err).strip("'\""),
                                 "todo": "Check the files (Board tab)"}, 400)

    @app.post("/api/board/upload")
    async def board_upload(request: Request, name: str, import_id: str = ""):
        data = await request.body()

        def go():
            iid = import_id or m.boards.new_import()
            return {"ok": True, "import_id": iid, "files": m.boards.add_file(iid, name, data)}
        return _board_call(go)

    @app.post("/api/board/import")
    def board_import(body: dict):
        return _board_call(lambda: {"ok": True, **m.boards.run(str(body.get("import_id", "")), body.get("spec"),
                                                                bool(body.get("fit", True)),
                                                                bool(body.get("use_current", True)),
                                                                body.get("name") or None)})

    @app.get("/api/board/import/{iid}/preview")
    def board_import_preview(iid: str, side: str = "F"):
        return _board_call(lambda: m.boards.preview(m.boards.pending(iid), "B" if side == "B" else "F"))

    @app.post("/api/board/save")
    def board_save(body: dict):
        def go():
            name = m.boards.save(str(body.get("import_id", "")), body.get("name") or None)
            m.say(f"Board {name} is now the current board", kind="board")
            return {"ok": True, "name": name}
        return _board_call(go)

    @app.get("/api/board")
    def board_current(side: str = "F"):
        b = m.boards.current()
        if b is None:
            return {"ok": True, "board": None, "names": m.boards.names()}
        return {"ok": True, "board": m.boards.preview(b, "B" if side == "B" else "F"), "names": m.boards.names(),
                "current": m.boards.current_name()}

    @app.post("/api/board/use")
    def board_use(body: dict):
        return _board_call(lambda: (m.boards.use(str(body.get("name", ""))), {"ok": True})[1])

    @app.post("/api/board/thickness")
    def board_thickness(body: dict):
        def go():
            b = m.boards.set_thickness(_num(body, "mm"))
            m.say(f"Board thickness set to {b.thickness:g} mm", kind="board")
            return {"ok": True}
        return _board_call(go)

    @app.get("/api/overlay")
    def overlay():
        return m.overlay()

    @app.get("/api/feeders")
    def feeders():
        return {"trays": m.jobs.trays_view(), "dictionary": m.feeders.dictionary}

    @app.get("/api/job/plan")
    def job_plan():
        return m.jobs.plan_view()

    @app.get("/api/jobs")
    def jobs():
        return {"jobs": m.jobs.jobs_list(), "current": m.jobs.status()}

    @app.get("/jobs/{board}/{job}/{path:path}")
    def job_file(board: str, job: str, path: str):
        """A job's report and photos (relative links inside the report work)."""
        root = (m.data / "jobs").resolve()
        p = (root / board / job / path).resolve()
        if root not in p.parents or not p.is_file():
            raise HTTPException(404)
        return FileResponse(p, headers={"Cache-Control": "no-store"})

    @app.get("/api/trays/file/{name}")
    def tray_file(name: str):
        p = (m.data / "trays" / name).resolve()
        if p.parent != (m.data / "trays").resolve() or not p.is_file():
            raise HTTPException(404)
        return FileResponse(p, filename=name)

    @app.get("/api/store/drawers")
    def store_drawers():
        return m.parts.drawers_view()

    @app.get("/api/store/hand")
    def store_hand():
        return m.parts.hand()

    @app.get("/api/labels/file/{name}")
    def label_file(name: str):
        p = (m.data / "labels" / name).resolve()
        if p.parent != (m.data / "labels").resolve() or not p.is_file():
            raise HTTPException(404)
        return FileResponse(p, filename=name)

    @app.get("/api/bundle.zip")
    def bundle_zip(photos: bool = False):
        from .. import bundle
        data = bundle.export(m.cfg_dir, m.data, photos)
        name = f"pnp_{m.profile.name}_{time.strftime('%Y%m%d-%H%M')}.zip"
        return Response(data, media_type="application/zip",
                        headers={"Content-Disposition": f'attachment; filename="{name}"'})

    @app.post("/api/bundle")
    async def bundle_restore(request: Request):
        from .. import bundle
        if m.worker.current is not None or (m.jobs.job is not None and m.jobs.job.status == "running"):
            return JSONResponse({"ok": False, "why": "The machine is busy", "todo": "Wait, or stop the job"})
        try:
            r = bundle.restore(await request.body(), m.cfg_dir, m.data)
        except (ValueError, OSError) as err:
            return JSONResponse({"ok": False, "why": str(err)})
        m.say(f"Bundle restored: {r['files']} files (of {r['created']}). Restart the program to use it all", "warn")
        return {"ok": True, **r}

    @app.get("/api/events.jsonl")
    def events_file():
        p = m.events.path
        if p is None or not p.is_file():
            raise HTTPException(404)
        return FileResponse(p, filename=p.name)

    @app.get("/api/wizard")
    def wizard_steps():
        return {"steps": m.wizard()}

    @app.get("/api/setup/candidate.png")
    def candidate():
        c = m.setup.candidate
        if not c or not Path(c["image"]).is_file():
            raise HTTPException(404)
        return FileResponse(c["image"], headers={"Cache-Control": "no-store"})

    @app.get("/api/photo")
    def photo(path: str):
        """A photo the machine took (first part, job reports): only from its data folder."""
        p = Path(path).resolve()
        root = m.data.resolve()
        if root not in p.parents or p.suffix.lower() not in (".png", ".jpg") or not p.is_file():
            raise HTTPException(404)
        return FileResponse(p, headers={"Cache-Control": "no-store"})

    @app.get("/charuco.pdf")
    def charuco_pdf():
        from ..vision import charuco
        path = m.data / "charuco_sheet.pdf"
        charuco.write_pdf(path)
        return FileResponse(path, media_type="application/pdf", filename="charuco_sheet.pdf")

    @app.get("/health")
    def health():
        return PlainTextResponse("ok")

    return app
