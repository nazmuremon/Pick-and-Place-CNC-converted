"""Phase G: a vacuum valve, a vacuum sensor and a switched head light (docs/13 §4.14): each a profile change plus,
where needed, one Setup step. The simulator's needle has vacuum only with the pump AND the valve open; its sensor reads
~8 open, ~50 sealed, ~55 with a part; the light off makes the picture 45 % darker."""
from __future__ import annotations

import numpy as np

from pnp import sim
from pnp.calibration import chain
from simwizard import run


def valve_and_sensor(raw):
    raw["vacuum"] = {"type": "pump_valve_sensor", "pump": {"start": "M8", "stop": "M9"},
                     "valve": {"start": "M7", "stop": "M9"}, "sensor": {"input": "TBD", "held_threshold": None},
                     "pick_dwell_s": 0.5}


def _over_a_part(m):
    w = m.cameras["camera_head"].world
    s = sim.Strip(slot=3, z_top=1.35, part_height=1.35)
    w.add(sim.strip_layer(w, s))
    c, u, _v = w.slot_frame(3)
    part = c + u * (s.first_u + 2 * s.pitch)
    noz = sim.nozzle_of(m.sim_grbl)
    with m.sim_grbl.lock:
        tip = noz.tip_xy() - [m.sim_grbl.phys["X"], m.sim_grbl.phys["Y"]]
        m.sim_grbl.phys["X"], m.sim_grbl.phys["Y"] = part - tip
    run(m, m.jog(dz=-(m.sim_grbl.phys["Z"] - 1.35) - 0.3))
    return noz, s


def test_no_vacuum_at_the_needle_without_the_valve(machine):
    m = machine(valve_and_sensor)
    noz, s = _over_a_part(m)
    run(m, m.pump(True))                                          # the pump alone: the valve is closed
    assert noz.held is None
    m.ctl.gcode("M7")                                             # the valve opens: the part comes up
    assert noz.held is not None and not s.pockets[2].present
    m.caps.vacuum.release()                                       # the valve closes (M9 stops both)
    assert noz.held is None


def test_the_vacuum_sensor_step_and_the_held_answer(machine):
    m = machine(valve_and_sensor)
    st = {x["id"]: x for x in m.wizard()}
    assert st["vacuum"]["n"] == "6b" and st["vacuum"]["state"] == "missing"
    assert m.caps.vacuum.held() is None                           # not set up yet: can't tell
    run(m, m.setup_action("vacuum_open", {}))                     # the needle in the air
    run(m, m.jog(dz=-(m.sim_grbl.phys["Z"]) - 0.2))               # pressed on the bed: sealed
    run(m, m.setup_action("vacuum_closed", {}))
    v = chain.value(m, "vacuum_sensor")
    assert v is not None and 20 < v["threshold"] < 40 and v["closed"] - v["open"] > 30
    assert {x["id"]: x["state"] for x in m.wizard()}["vacuum"] == "valid"
    run(m, m.jog(dz=10))
    noz, _ = _over_a_part(m)
    m.caps.vacuum.pick()
    run(m, m.jog(dz=5))
    assert m.caps.vacuum.can_sense and m.caps.vacuum.held() is True
    m.caps.vacuum.release()
    m.caps.vacuum.pump(True)
    m.ctl.gcode("M7")
    assert m.caps.vacuum.held() is False                          # pump on, nothing on the needle


def test_a_real_controller_sensor_is_tbd_not_guessed(machine):
    m = machine(valve_and_sensor)
    m.caps.vacuum.reader = None                                   # what a real machine has today
    st = {x["id"]: x for x in m.wizard()}
    assert st["vacuum"]["state"] == "later" and "TBD" in st["vacuum"]["why"]
    r = m.setup_action("vacuum_open", {})
    assert r["ok"]
    m.worker.wait(20)
    assert "TBD" in " ".join(e["text"] for e in m.events.tail(2))


def head_light(raw):
    raw["controller"]["type"] = "grblhal"                          # digital outputs: M62 / M63 P<n>
    raw["controller"]["firmware"] = None
    raw["lights"] = {"head": {"via": "controller", "start": "M62 P0", "stop": "M63 P0"}}


def test_the_exposure_is_kept_for_each_light_state(machine):
    m = machine(head_light)
    run(m, m.auto_exposure(), 120)
    on = chain.value(m, "exposure_head")
    off = chain.value(m, "exposure_head_off")
    assert on is not None and off is not None
    assert off["exposure"] > 1.4 * on["exposure"]                  # a darker picture needs more exposure
    cam = m.cameras["camera_head"]
    assert m.sim_grbl.dout.get(0) is True                         # the light stays on for the work
    run(m, m.light("light_head", False))
    assert cam.gain == off["exposure"] / cam.ref_exposure          # the saved value of the 'off' state
    import time
    time.sleep(0.4)
    assert m.vision.light_problem() is None                        # the picture matches the saved light
    run(m, m.light("light_head", True))
    assert cam.gain == on["exposure"] / cam.ref_exposure
