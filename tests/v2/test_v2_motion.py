"""The Grbl link and the controller against the virtual Grbl; status parsing; axis map; the safety state."""
from __future__ import annotations

import itertools
import os
import time

import pytest
from conftest import PROFILE, raw_profile, wait_until

from pnp import profile as prof
from pnp.motion.axes import AxisMap
from pnp.motion.controller import Controller
from pnp.motion.grbl import AlarmSeen, ControllerError, GrblLink, parse_status
from pnp.motion.limits import Limits
from pnp.motion.state import ACTIONS, SafetyState
from pnp.motion.transport import clear_hupcl
from pnp.sim.controller import SimGrbl


@pytest.fixture
def bench():
    p = prof.load(PROFILE)
    g = SimGrbl(p, seed=1)
    g.speed = 50
    link = GrblLink(g.open_port(), 0.05, 1.0)
    seen = []
    link.listeners.append(lambda k, d: seen.append((k, d)))
    link.start()
    yield p, g, link, seen
    link.close()
    g.shutdown()


# ---------------------------------------------------------------- pure functions
def test_status_lines_parse_with_mpos_wpos_and_fields():
    st, wco = parse_status("<Jog|MPos:1.000,-2.500,3.000|Bf:15,127|FS:500,1500|Pn:XE|A:SF>", None, 1.0)
    assert st.state == "Jog" and st.mpos == {"X": 1.0, "Y": -2.5, "Z": 3.0}
    assert (st.feed, st.spindle, st.pins, st.accessories) == (500.0, 1500.0, "XE", "SF")
    st, wco = parse_status("<Idle|WPos:1,1,1|F:0|WCO:10,20,30>", None, 2.0)
    assert st.mpos == {"X": 11.0, "Y": 21.0, "Z": 31.0}
    st, _ = parse_status("<Hold:0|WPos:0,0,0>", wco, 3.0)        # WCO comes only now and then: the last one
    assert st.base == "Hold" and st.mpos == {"X": 10.0, "Y": 20.0, "Z": 30.0}


def test_the_axis_map_round_trips_every_axis_and_sign():
    for letters in itertools.permutations("XYZ"):
        for signs in itertools.product((1, -1), repeat=3):
            m = AxisMap({a: (L, s) for a, L, s in zip("XYZ", letters, signs)})
            pos = {"X": 1.5, "Y": -2.25, "Z": 7.0}
            assert m.to_machine(m.to_controller(pos)) == pos


def test_the_bench_axis_map_makes_the_right_words():
    m = AxisMap.from_profile(prof.load(PROFILE))
    assert m.words({"X": 2, "Y": 3, "Z": 1}) == "Y-2.000 Z3.000 X-1.000"
    assert m.to_machine({"X": -1.0, "Y": -2.0, "Z": 3.0}) == {"X": 2.0, "Y": 3.0, "Z": 1.0}


def test_hupcl_is_cleared_on_a_tty():
    if os.name != "posix":
        pytest.skip("posix only")
    import termios
    master, slave = os.openpty()
    try:
        attrs = termios.tcgetattr(slave)
        attrs[2] |= termios.HUPCL
        termios.tcsetattr(slave, termios.TCSANOW, attrs)
        assert clear_hupcl(slave)
        assert not termios.tcgetattr(slave)[2] & termios.HUPCL
    finally:
        os.close(master)
        os.close(slave)


def test_limits_cap_steps_feeds_and_targets():
    lim = Limits(prof.load(PROFILE))
    assert lim.check_step({"X": 401}) is not None and lim.check_step({"X": 400}) is None
    assert lim.feed({"X": 5}, None, homed=False) == 300          # slow before Home
    assert lim.feed({"X": 5}, 5000, homed=True) == 1000          # never above the cap
    assert lim.feed({"Z": 1}, None, homed=False) == 300          # Z: its own cap
    assert lim.check_target({"X": 10.005}) is None               # rounding on the edge is inside
    r = lim.check_target({"X": 10.2})
    assert r is not None and "outside the travel" in r.why
    raw = raw_profile()
    raw["motion"]["travel"]["x"] = None
    lim2 = Limits(prof.parse(__import__("yaml").safe_dump(raw)))
    assert lim2.check_goto({"X": 0, "Y": -50}).why.startswith("The X / Y travel is not known")


def test_the_safety_state_allows_only_what_each_mode_permits():
    s = SafetyState()
    assert all(s.refuse(a) is not None for a in ACTIONS)                     # disconnected: nothing
    s.connected = True
    assert s.refuse("jog_xy") is None and s.refuse("goto_xy") is not None    # not homed
    s.homed = True
    assert s.refuse("goto_xy") is None
    assert "heights" in s.refuse("auto_z_down").why or "Home" in s.refuse("auto_z_down").why
    s.set_lock("reset")
    assert not s.homed and not s.z_ref
    assert s.refuse("jog_z_up") is None and s.refuse("pump") is None
    for a in ("jog_xy", "jog_z_down", "goto_xy", "home", "turn", "unlock_alarm"):
        assert s.refuse(a) is not None, a
    assert s.refuse("confirm_clear") is None
    s.set_lock("alarm")
    assert s.refuse("confirm_clear") is not None and s.refuse("unlock_alarm") is None
    s.set_lock("reset")                                      # a milder lock never hides the alarm
    assert s.lock.code == "alarm"
    b = s.banner()
    assert b["level"] == "locked" and b["todo"]


# ---------------------------------------------------------------- the link against the virtual Grbl
def test_send_returns_replies_and_errors_raise(bench):
    p, g, link, _ = bench
    assert link.wait_status(0, 3).state in ("Idle", "Alarm")
    assert "1.1h.20261007" in link.build_info()
    assert link.settings()["$22"] == "0"
    with pytest.raises(ControllerError, match="error:"):
        link.send("G999")


def test_a_reset_banner_and_an_alarm_are_events(bench):
    _, g, link, seen = bench
    link.wait_status(0, 3)
    seen.clear()
    g.power_cycle()
    assert wait_until(lambda: any(k == "reset" for k, _ in seen))
    g.trigger_estop()
    assert wait_until(lambda: any(k == "alarm" and d["code"] == "10" for k, d in seen))
    with pytest.raises(ControllerError):
        link.send("G0 X1")                                  # locked by the alarm: error:9


def test_the_watchdog_says_when_the_controller_is_silent():
    class Dead:
        def write(self, b): return len(b)
        def read(self, n=1):
            time.sleep(0.05)
            return b""
        def close(self): pass

    link = GrblLink(Dead(), 0.05, 0.3)
    seen = []
    link.listeners.append(lambda k, d: seen.append(k))
    link.start()
    try:
        assert wait_until(lambda: "link_lost" in seen, 3)
        with pytest.raises(ControllerError):
            link.send("$I", timeout=0.5)
    finally:
        link.close()


def test_connect_writes_a_token_and_finds_it_again(bench):
    p, g, link, _ = bench
    c = Controller(link, p)
    info = c.connect(None)
    assert info.firmware_ok and not info.mismatched and info.token and g.tool == info.token
    info2 = Controller(link, p).connect(info.token)
    assert info2.token_found == info.token and info2.token == info.token          # not reset: kept
    g.power_cycle()
    time.sleep(0.2)
    info3 = Controller(link, p).connect(info.token)
    assert info3.token_found == 0 and info3.token not in (None, 0, info.token)     # reset: a new one


def test_wrong_firmware_and_settings_are_reported(bench):
    p, g, link, _ = bench
    g.version = "1.1h.20190825"
    g.settings["$1"] = "25"
    info = Controller(link, p).connect(None)
    assert not info.firmware_ok and info.mismatched == {"$1": ("255", "25")}


def test_jog_moves_in_machine_axes_and_stop_cancels_it(bench):
    p, g, link, _ = bench
    c = Controller(link, p)
    c.connect(None)
    t = c.jog({"X": 5.0}, 1000)
    c.wait_idle(t, 5)
    assert c.raw()["X"] == pytest.approx(5.0) and g.counted["Y"] == pytest.approx(-5.0)   # X is Grbl −Y
    g.speed = 1.0                                                                          # slow: stop it midway
    t = c.jog({"X": 50.0}, 300)
    time.sleep(0.3)
    c.stop()
    st = c.wait_idle(t, 5)
    assert st.state == "Idle" and 5.0 < c.raw()["X"] < 55.0 - 1


def test_m_codes_wait_for_the_moves_before_them(bench):
    p, g, link, _ = bench
    g.speed = 5.0
    c = Controller(link, p)
    c.connect(None)
    c.gcode("G91 G1 Y-20 F600")                    # machine X +20: 2 s at 600 mm/min, 0.4 s here
    t0 = time.monotonic()
    c.gcode("M8")
    assert time.monotonic() - t0 > 0.25 and g.flood
    assert g.counted["Y"] == pytest.approx(-20.0)


def test_the_frame_end_makes_lost_steps_that_the_controller_does_not_see(bench):
    p, g, link, _ = bench
    c = Controller(link, p)
    c.connect(None)
    start = g.phys["X"]
    room = p.sim.hard_ends["X"][1] - start
    t = c.jog({"X": room + 10}, 1000)
    c.wait_idle(t, 10)
    assert c.raw()["X"] == pytest.approx(room + 10)              # Grbl thinks it got there
    assert g.phys["X"] == pytest.approx(p.sim.hard_ends["X"][1])  # the head stopped at the end
    assert g.lost_steps["X"] == pytest.approx(10)


def test_probe_finds_the_surface_or_alarms(tmp_path):
    raw = raw_profile()
    raw["sim"]["probe_surface_z"] = 4.0
    raw["sim"]["speed"] = 50
    p = prof.parse(__import__("yaml").safe_dump(raw))
    g = SimGrbl(p)
    link = GrblLink(g.open_port(), 0.05, 1.0).start()
    try:
        c = Controller(link, p)
        c.connect(None)
        from pnp.capabilities.references import ProbeZ
        z0 = ProbeZ(100, 30, 2).establish(c, surface_z=0.0)
        # the head started at tip Z 20 with the counter at 0: the surface (tip 4) is 16 mm lower
        assert z0 == pytest.approx(-16.0, abs=0.2)
        assert g.phys["Z"] == pytest.approx(6.0, abs=0.2)                     # retracted 2 mm
        from pnp.capabilities.base import CapabilityError
        with pytest.raises((CapabilityError, AlarmSeen, ControllerError)):
            ProbeZ(100, 1, 0).establish(c, 0.0)                              # 1 mm is not enough: ALARM:5
    finally:
        link.close()
        g.shutdown()


def test_switch_homing_zeroes_at_the_switches(tmp_path):
    raw = raw_profile()
    raw["sim"]["switches"] = True
    raw["sim"]["speed"] = 200
    raw["homing"]["method"] = "switches"
    raw["motion"]["travel"]["frame"] = "controller"
    p = prof.parse(__import__("yaml").safe_dump(raw))
    g = SimGrbl(p)
    link = GrblLink(g.open_port(), 0.05, 2.0).start()
    try:
        c = Controller(link, p)
        c.connect(None)
        from pnp.capabilities.references import SwitchHoming
        assert SwitchHoming(60).home(c) == {"X": 0.0, "Y": 0.0}
        assert c.raw()["X"] == pytest.approx(0) and c.raw()["Y"] == pytest.approx(0)
        assert g.phys["X"] == pytest.approx(p.sim.hard_ends["X"][1] - 1.0)       # backed off the switch
        assert g.phys["Y"] == pytest.approx(p.sim.hard_ends["Y"][1] - 1.0)
        assert g.lost_steps["X"] == 0 and g.lost_steps["Y"] == 0
    finally:
        link.close()
        g.shutdown()
