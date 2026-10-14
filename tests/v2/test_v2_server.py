"""The page's server: cache busting, actions, the cross-site guard, the WebSocket, doctor, the port keeper,
the command line."""
from __future__ import annotations

import os
import socket
import threading
import time

import pytest
from conftest import PROFILE, wait_until
from fastapi.testclient import TestClient

from pnp.ui.server import asset_version, create_app


@pytest.fixture
def client(machine):
    m = machine()
    with TestClient(create_app(m)) as c:
        yield m, c


def test_the_page_links_its_files_with_their_version(client):
    m, c = client
    r = c.get("/")
    assert r.status_code == 200 and r.headers["cache-control"] == "no-store"
    v = asset_version()
    assert f"app.js?v={v}" in r.text and "__V__" not in r.text
    r = c.get("/static/app.js")
    assert r.status_code == 200 and r.headers["cache-control"] == "no-cache"
    assert c.get("/static/../server.py").status_code == 404


def test_actions_run_and_refusals_come_back(client):
    m, c = client
    assert c.post("/api/action/jog", json={"dx": 2}).json() == {"ok": True}
    assert m.worker.wait()
    r = c.post("/api/action/goto", json={"x": 0, "y": -40}).json()
    assert not r["ok"] and r["todo"]
    assert c.post("/api/action/nope", json={}).status_code == 404
    assert c.post("/api/action/turn", json={"deg": "abc"}).status_code == 400
    assert c.post("/api/stop").json()["ok"]


def test_requests_from_other_sites_are_refused(client):
    m, c = client
    r = c.post("/api/action/jog", json={"dx": 2}, headers={"Origin": "http://evil.example"})
    assert r.status_code == 403
    r = c.post("/api/action/jog", json={"dx": 2}, headers={"Origin": "http://testserver"})
    assert r.status_code == 200


def test_the_websocket_sends_hello_status_and_events(client):
    m, c = client
    with c.websocket_connect("/ws") as ws:
        hello = ws.receive_json()
        assert hello["type"] == "hello" and hello["version"] == asset_version()
        assert ws.receive_json()["type"] == "status"
        m.say("from the test", kind="test")
        got = [ws.receive_json() for _ in range(6)]
        assert any(g["type"] == "event" and g["event"]["text"] == "from the test" for g in got)


def test_status_map_profile_doctor_and_video(client):
    m, c = client
    assert c.get("/api/status").json()["safety"]["mode"] == "not_homed"
    assert "slots" in c.get("/api/map").json()
    p = c.get("/api/profile").json()
    assert "bench_uno" in p["text"] and p["capabilities"]
    checks = {x["name"]: x for x in c.get("/api/doctor").json()}
    assert checks["Profile"]["ok"] and checks["Controller"]["ok"] and checks["Session token"]["ok"]
    assert checks["Camera (head)"]["ok"]
    assert c.get("/snapshot.jpg").content[:2] == b"\xff\xd8"


def test_the_port_keeper_passes_bytes_and_keeps_the_port():
    if os.name != "posix":
        pytest.skip("posix only")
    from pnp.motion.portd import serve
    master, slave = os.openpty()
    stop = threading.Event()
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    t = threading.Thread(target=serve, args=(os.ttyname(slave), 115200, "127.0.0.1", port),
                         kwargs={"log": lambda m: None, "stop": stop.is_set}, daemon=True)
    t.start()
    try:
        assert wait_until(lambda: _can_connect(port), 3)
        for _ in range(2):                                       # a second client after the first left
            with socket.create_connection(("127.0.0.1", port), timeout=2) as cl:
                cl.sendall(b"?")
                assert os.read(master, 10) == b"?"
                os.write(master, b"<Idle>\r\n")
                assert cl.recv(100) == b"<Idle>\r\n"
            time.sleep(0.1)
    finally:
        stop.set()
        t.join(timeout=3)
        os.close(master)
        os.close(slave)


def _can_connect(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.2) as c:
            c.close()
        time.sleep(0.1)                                          # let the keeper see that client leave
        return True
    except OSError:
        return False


def test_the_command_line_checks_profiles(tmp_path, capsys):
    from pnp.cli import main
    (tmp_path / "profiles").mkdir()
    (tmp_path / "profiles" / "bench.yaml").write_text(PROFILE.read_text())
    assert main(["--config", str(tmp_path), "profile", "check", "bench"]) == 0
    assert "valid" in capsys.readouterr().out
    (tmp_path / "profiles" / "bad.yaml").write_text("name: x\n")
    assert main(["--config", str(tmp_path), "profile", "check", "bad"]) == 1
    assert main(["--config", str(tmp_path), "profile", "list"]) == 0
    assert capsys.readouterr().out.split()[-2:] == ["bad", "bench"]
