"""The machine process on the simulator: connect, jog, limits, Home, Z, locks and recovery, restarts, reload."""
from __future__ import annotations

import time

import pytest
import yaml
from conftest import manual_homing, wait_until

from pnp.machine import Machine
from pnp.profile import load
from pnp.sim.controller import SimGrbl


def texts(m: Machine) -> list[str]:
    return [e["text"] for e in m.events.tail(50)]


def test_connect_starts_not_homed_and_jogs_slowly(machine):
    m = machine()
    assert m.safety.mode == "not_homed" and m.session.token
    assert m.jog(dx=5)["ok"]
    assert m.worker.wait()
    assert m.raw()["X"] == pytest.approx(5)
    jog = [line for line in m.sim_grbl.log if line.startswith("$J=")][-1]
    assert jog.endswith("F300")                                        # slow before Home


def test_refusals_say_why_and_what_to_do(machine):
    m = machine()
    r = m.jog(dx=500)
    assert not r["ok"] and "largest single move" in r["why"] and r["todo"]
    r = m.goto(-10, -40)
    assert not r["ok"] and r["why"] == "The position is not known yet"
    r = m.jog(dx=1, dz=1)
    assert not r["ok"] and "together" in r["why"]
    r = m.home()
    assert not r["ok"] and "calibrated" in r["why"] and r["todo"]    # camera homing needs the camera model


def test_one_operation_at_a_time(machine):
    m = machine()
    m.sim_grbl.speed = 1.0
    assert m.jog(dy=-30)["ok"]
    r = m.jog(dy=-5)
    assert not r["ok"] and "busy" in r["why"]
    m.stop()
    assert m.worker.wait()


def test_manual_home_then_goto_inside_the_travel_and_refused_outside(machine):
    m = machine(manual_homing)
    m.jog(dx=3)
    m.worker.wait()
    assert m.home()["ok"] and m.worker.wait()
    assert m.safety.homed and m.position()["X"] == pytest.approx(0)
    assert m.goto(-20, -40)["ok"] and m.worker.wait()
    assert m.position()["X"] == pytest.approx(-20) and m.position()["Y"] == pytest.approx(-40)
    assert m.sim_grbl.phys["X"] == pytest.approx(-24 + 3 - 20)          # the head really went there
    r = m.goto(50, -40)
    assert not r["ok"] and "outside the travel" in r["why"]
    r = m.jog(dy=30)                                                    # -40 + 30 = -10 > -19.4
    assert not r["ok"] and "outside the travel" in r["why"]
    jog = [line for line in m.sim_grbl.log if line.startswith("$J=")][-1]
    assert "G53" in jog and jog.endswith("F1000")                       # homed: full speed, absolute


def test_z_touch_off_sets_z_from_the_surface(machine):
    m = machine()
    m.jog(dz=-3)
    m.worker.wait()
    assert m.set_z(1.6, "on a board")["ok"] and m.worker.wait()
    assert m.safety.z_ref and m.position()["Z"] == pytest.approx(1.6)
    m.jog(dz=2)
    m.worker.wait()
    assert m.position()["Z"] == pytest.approx(3.6)


def test_a_power_dip_locks_and_only_the_recovery_unlocks(machine):
    m = machine(manual_homing)
    m.home()
    m.worker.wait()
    m.sim_event("power_cycle")
    assert wait_until(lambda: m.safety.mode == "locked")
    assert m.safety.lock.code == "reset" and not m.safety.homed
    for r in (m.jog(dx=1), m.jog(dz=-1), m.goto(-20, -40), m.home(), m.turn_to(0)):
        assert not r["ok"] and r["why"] == "The controller was reset"
    assert m.jog(dz=1)["ok"] and m.worker.wait()                       # lifting is allowed
    assert m.pump(False)["ok"] and m.worker.wait()                     # and the pump
    assert m.confirm_clear()["ok"]
    assert m.safety.mode == "not_homed"
    assert any("LOCKED" in t for t in texts(m))


def test_an_alarm_needs_unlock_then_the_nozzle_clear(machine):
    m = machine()
    m.sim_event("estop")
    assert wait_until(lambda: m.safety.lock is not None and m.safety.lock.code == "estop")
    assert not m.confirm_clear()["ok"]
    m.sim_event("release_estop")
    assert m.unlock_alarm()["ok"] and m.worker.wait()
    assert m.safety.lock.code == "reset"                               # position still not trusted
    assert m.confirm_clear()["ok"] and m.safety.mode == "not_homed"


def test_a_restart_without_a_reset_keeps_home_and_z(make_cfg, data_dir):
    path = make_cfg(manual_homing)
    g = SimGrbl(load(path))                                            # the controller outlives the program
    m1 = Machine(path, sim=True, data_dir=data_dir, watch=False, sim_controller=g)
    try:
        m1.connect()
        m1.worker.wait()
        m1.home()
        m1.worker.wait()
        m1.jog(dz=-2)
        m1.worker.wait()
        m1.set_z(0.0)
        m1.worker.wait()
        m1.jog(dx=-7)
        m1.worker.wait()
        token = m1.session.token
        m1.close()                                                     # the program stops; Grbl keeps running

        m2 = Machine(path, sim=True, data_dir=data_dir, watch=False, sim_controller=g)
        m2.connect()
        m2.worker.wait()
        assert m2.session.token == token
        assert m2.safety.mode == "homed" and m2.safety.z_ref
        assert m2.position()["X"] == pytest.approx(-7) and m2.position()["Z"] == pytest.approx(0)
        assert any("not reset" in t for t in texts(m2))
        m2.close()

        g.power_cycle()                                                # now a reset while the program was off
        time.sleep(0.1)
        m3 = Machine(path, sim=True, data_dir=data_dir, watch=False, sim_controller=g)
        m3.connect()
        m3.worker.wait()
        assert m3.safety.mode == "locked" and m3.safety.lock.code == "reset"
        assert m3.session.token not in (None, token)
        m3.close()
    finally:
        g.shutdown()


def test_a_lock_survives_a_restart_when_the_controller_was_not_reset(make_cfg, data_dir):
    path = make_cfg()
    g = SimGrbl(load(path))
    m1 = Machine(path, sim=True, data_dir=data_dir, watch=False, sim_controller=g)
    try:
        m1.connect()
        m1.worker.wait()
        m1._lock("stall", "test")
        m1.close()
        m2 = Machine(path, sim=True, data_dir=data_dir, watch=False, sim_controller=g)
        m2.connect()
        m2.worker.wait()
        assert m2.safety.lock is not None and m2.safety.lock.code == "stall"
        m2.close()
    finally:
        g.shutdown()


def test_the_servo_turns_from_the_same_side_and_waits(machine):
    m = machine()
    m.sim_grbl.log.clear()
    assert m.turn_to(90)["ok"] and m.worker.wait()
    log = [x for x in m.sim_grbl.log if x.startswith(("M3", "G4"))]
    assert log == ["M3 S1967", "M3 S2167", "G4 P1"]                    # 200 µs short first, then the settle
    assert m.caps.rotation.angle == 90
    assert abs(m.sim_grbl.servo_angle - 90) <= 10                       # the play of the profile
    r = m.turn_to(200)
    assert m.worker.wait()
    assert any("outside the servo range" in t for t in texts(m)) or not r["ok"]


def test_the_servo_needs_its_firmware_and_settle_time(machine):
    m = machine(lambda r: r["rotation"]["servo_pwm"].update(settle_s=None))
    r = m.turn_to(0)
    assert not r["ok"] and "settle time is TBD" in r["why"]
    m2 = machine(connect=False)
    m2.sim_grbl.version = "1.1h.20190825"                              # stock Grbl: no servo pulses
    m2.connect()
    m2.worker.wait()
    r = m2.turn_to(0)
    assert not r["ok"] and "firmware" in r["why"], (r, texts(m2), m2.ctl.info)


def test_pump_on_and_off_through_the_profile_codes(machine):
    m = machine(lambda r: r["vacuum"]["pump"].update(start="M7"))
    assert m.pump(True)["ok"] and m.worker.wait()
    assert m.sim_grbl.mist and not m.sim_grbl.flood
    assert m.pump(False)["ok"] and m.worker.wait()
    assert not m.sim_grbl.mist


def test_a_profile_reload_applies_or_keeps_the_old_one(machine):
    m = machine(manual_homing)
    m.home()
    m.worker.wait()
    raw = yaml.safe_load(m.profile_path.read_text())
    raw["motion"]["max_feed_xy"] = 700
    m.profile_path.write_text(yaml.safe_dump(raw, sort_keys=False))
    from pnp.profile import Watcher
    w = Watcher(m.profile_path, m._profile_changed)
    w.last = ""
    assert w.check()
    assert m.limits.max_feed_xy == 700 and m.safety.homed and not m.reconnect_needed
    m.profile_path.write_text("controller: [")
    assert w.check()
    assert m.profile.motion.max_feed_xy == 700 and m.profile_error is not None
    raw["controller"]["axis_map"]["X"]["sign"] = 1
    m.profile_path.write_text(yaml.safe_dump(raw, sort_keys=False))
    assert w.check()
    assert m.profile_error is None and m.reconnect_needed and not m.safety.homed


def test_e_stop_input_on_grblhal_locks(machine):
    def grblhal(r):
        r["controller"].update(type="grblhal", firmware=None)
        r["estop"]["type"] = "controller_input"
    m = machine(grblhal)
    m.sim_event("estop")
    assert wait_until(lambda: m.safety.lock is not None and m.safety.lock.code == "estop")
    r = m.unlock_alarm()
    assert m.worker.wait()
    assert any("still pressed" in t or "E-stop still pressed?" in t for t in texts(m)) or not r["ok"], texts(m)
    assert m.safety.lock.code == "estop"
    m.sim_event("release_estop")
    assert m.unlock_alarm()["ok"] and m.worker.wait()
    assert m.safety.lock.code == "reset"


def test_status_and_map_have_what_the_page_needs(machine):
    m = machine(manual_homing)
    m.home()
    m.worker.wait()
    s = m.status()
    assert s["sim"] and s["banner"]["level"] == "warn" and s["controller"]["version"] == "1.1h.20261007"
    assert {c["role"] for c in s["capabilities"]} >= {"homing", "rotation", "vacuum", "camera_head", "feeders"}
    mp = m.map_data()
    assert mp["head"] == [pytest.approx(0), pytest.approx(0)] and set(mp["slots"]) == {"3", "4", "5", "6", "7"}
    assert mp["travel"]["X"] == [-117.4, 10.0]


def test_events_are_written_as_json_lines(machine, data_dir):
    m = machine()
    m.say("hello", kind="test")
    lines = (data_dir / "events_sim.jsonl").read_text().splitlines()
    import json
    ev = json.loads(lines[-1])
    assert ev["text"] == "hello" and ev["state"]["mode"] == "not_homed"


def test_the_camera_sees_the_bed_labels_at_the_head(machine):
    import cv2
    m = machine()
    cam = m.cameras["camera_head"]
    frame = cam.frames_after(time.monotonic(), 1, 5)[0]
    d = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                cv2.aruco.DetectorParameters())
    _, ids, _ = d.detectMarkers(frame)
    assert ids is not None and {49, 42} <= set(ids.ravel().tolist())     # HOME and slot 3 at the start view
