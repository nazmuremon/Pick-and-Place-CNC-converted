"""Phase G: the Setup steps for XY limit switches and for a stepper rotation axis (docs/13 §4.14): a profile change
plus one wizard step, the job and calibration code unchanged."""
from __future__ import annotations

import math

import numpy as np
import pytest

from pnp import sim
from pnp.calibration import chain
from simwizard import ready_machine, run


def switches(raw):
    raw["homing"]["method"] = "switches"
    raw["motion"]["travel"] = {"frame": "controller", "x": None, "y": None, "z": None}
    raw["sim"]["switches"] = True
    raw["controller"]["expected_settings"].pop("$22", None)


def test_the_switch_step_homes_three_times_and_takes_the_travel(machine):
    m = machine(switches)
    st = {s["id"]: s for s in m.wizard()}
    assert st["travel"]["title"] == "Homing switches" and st["travel"]["state"] == "missing"
    run(m, m.home())
    run(m, m.setup_action("switch_test", {}))
    t = chain.value(m, "travel")
    span_x = m.profile.sim.hard_ends["X"][1] - m.profile.sim.hard_ends["X"][0]
    assert t["x"][1] == -1.0 and t["x"][0] == pytest.approx(-(span_x - 1.0) + 1.0)
    assert m.limits.source == "measured" and m.safety.homed
    assert {s["id"]: s["state"] for s in m.wizard()}["travel"] == "valid"
    run(m, m.goto(-30.0, -30.0))                                    # go-to works inside the switches' travel
    assert m.sim_grbl.lost_steps["X"] == 0 and m.sim_grbl.lost_steps["Y"] == 0


def stepper(raw):
    raw["rotation"] = {"type": "stepper_axis", "stepper_axis": {"letter": "A", "feed_deg_min": 3600}}
    raw["sim"]["rot_scale"] = 0.98
    raw["sim"]["rot_backlash_deg"] = 1.5


@pytest.fixture(scope="module")
def sm(tmp_path_factory):
    m = ready_machine(tmp_path_factory.mktemp("pnp_stepper"), stepper)    # includes step 6: the axis is measured
    try:
        yield m
    finally:
        m.close()


def test_the_stepper_step_measures_scale_backlash_and_repeat(sm):
    t = chain.value(sm, "rotation_table")
    assert t["kind"] == "stepper"
    assert abs(t["scale"] - 0.98) < 0.01, t
    # the marks read the backlash only to about 1° at 5 px/mm (1.5° moves the tip 0.04 mm); every turn comes from
    # 2° beyond it, so the true 1.5° play is always taken up the same way
    assert abs(t["backlash_deg"] - 1.5) < 1.0, t
    assert t["accuracy_deg"] < 1.0
    assert sm.caps.rotation.table is not None and sm.caps.rotation.approach() > 1.5
    assert {s["id"]: s["state"] for s in sm.wizard()}["rotation"] == "valid"


@pytest.mark.parametrize("a,b", [(-90.0, 90.0), (0.0, 45.0), (-30.0, 120.0), (60.0, -60.0)])
def test_a_stepper_turns_by_what_was_asked(sm, a, b):
    """A place turns the part by the difference of two angles (the part's own angle is measured by the camera):
    the scale and the backlash must be gone from that difference."""
    m = sm
    run(m, m.turn_to(a))
    ra = m.sim_grbl.servo_angle                                   # the sim's true angle
    run(m, m.turn_to(b))
    rb = m.sim_grbl.servo_angle
    assert abs((rb - ra) - (b - a)) < 0.8, (a, b, ra, rb)          # without the table: 3.6° at 180° (scale 0.98)
