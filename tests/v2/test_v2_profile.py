"""The machine profile: schema, errors with line numbers, migrations, TBD list, reload diff and watcher."""
from __future__ import annotations

import pytest
import yaml
from conftest import PROFILE, raw_profile

from pnp import profile as prof
from pnp.profile.schema import SCHEMA_VERSION


def dump(raw: dict) -> str:
    return yaml.safe_dump(raw, sort_keys=False)


def test_the_bench_profile_is_valid():
    p = prof.load(PROFILE)
    assert p.name == "bench_uno"
    assert p.controller.axis_map["X"].letter == "Y" and p.controller.axis_map["X"].sign == -1
    assert p.rotation.servo_pwm.nominal_range_deg == 270
    assert p.vacuum.pump.start == "M8" and p.vacuum.pump.stop == "M9"
    assert p.motion.travel.x == (-117.4, 10.0)


def test_only_the_z_travel_and_the_spring_are_tbd_on_the_bench():
    assert [m.field for m in prof.missing(prof.load(PROFILE))] == ["motion.travel.z", "nozzles.items[N22G].spring_mm"]


def test_an_error_names_the_field_and_its_line():
    text = PROFILE.read_text().replace("    X: {letter: Y, sign: -1}", "    X: {letter: Y, sign: 2}")
    with pytest.raises(prof.ProfileError) as err:
        prof.parse(text)
    line = next(n for n, t in enumerate(text.splitlines(), 1) if "sign: 2" in t)
    assert err.value.problems == [f"controller.axis_map.X.sign: Input should be 1 or -1 (line {line})"]


@pytest.mark.parametrize("change, words", [
    (lambda r: r["controller"]["axis_map"]["Y"].update(letter="Y"), "letter twice"),
    (lambda r: r["vacuum"]["pump"].update(start="rm -rf /"), "not a G-code line"),
    (lambda r: r["rotation"]["servo_pwm"].update(max_us=1000), "min_us <= centre_us <= max_us"),
    (lambda r: r["motion"]["travel"].update(x=[10, -10]), "min < max"),
    (lambda r: r.update(typo_field=1), "Extra inputs are not permitted"),
    (lambda r: r["z_reference"].update(method="probe"), "needs the probe"),
    (lambda r: r["rotation"].update(type="stepper_axis"), "needs the stepper_axis"),
    (lambda r: r["homing"].update(method="switches"), "travel.frame: controller"),
    (lambda r: r["nozzles"].update(current="N99"), "not in items"),
])
def test_bad_profiles_are_refused_with_a_reason(change, words):
    raw = raw_profile()
    change(raw)
    with pytest.raises(prof.ProfileError) as err:
        prof.parse(dump(raw))
    assert any(words in p for p in err.value.problems), err.value.problems


def test_yaml_on_and_off_would_be_booleans_so_the_keys_are_start_and_stop():
    """A bare `on:` is True in YAML: the schema must not use it (it did once, 2026-10-12)."""
    text = PROFILE.read_text().replace("{start: M8, stop: M9}", "{on: M8, off: M9}")
    with pytest.raises(prof.ProfileError):
        prof.parse(text)


def test_a_newer_schema_is_refused_and_bad_yaml_is_reported():
    raw = raw_profile()
    raw["schema_version"] = SCHEMA_VERSION + 1
    with pytest.raises(prof.ProfileError, match="newer than this program"):
        prof.parse(dump(raw))
    with pytest.raises(prof.ProfileError, match="not valid YAML"):
        prof.parse("a: [1, 2")
    with pytest.raises(prof.ProfileError, match="mapping"):
        prof.parse("- 1\n- 2\n")


def test_tbd_fields_switch_capabilities_off_with_a_fix():
    raw = raw_profile()
    raw["motion"]["travel"]["x"] = None
    raw["rotation"]["servo_pwm"]["settle_s"] = None
    raw["cameras"]["head"]["exposure"]["value"] = None
    raw["cameras"]["head"]["tilted"] = None
    fields = {m.field: m for m in prof.missing(prof.parse(dump(raw)))}
    assert {"motion.travel.x / y", "rotation.servo_pwm.settle_s", "cameras.head.exposure.value",
            "cameras.head.moves_with_z / tilted"} <= set(fields)
    assert all(m.fix for m in fields.values())


def test_diff_says_what_needs_a_reconnect_or_a_new_home():
    a = prof.load(PROFILE)
    raw = raw_profile()
    raw["motion"]["max_feed_xy"] = 800
    c = prof.diff(a, prof.parse(dump(raw)))
    assert c.fields == ["motion.max_feed_xy"] and not c.reconnect and not c.rehome
    raw["controller"]["axis_map"]["X"]["sign"] = 1
    c = prof.diff(a, prof.parse(dump(raw)))
    assert c.reconnect and c.rehome
    raw = raw_profile()
    raw["controller"]["port"] = "socket://127.0.0.1:7450"
    c = prof.diff(a, prof.parse(dump(raw)))
    assert c.reconnect and not c.rehome


def test_the_watcher_reports_a_good_and_a_bad_file(tmp_path):
    p = tmp_path / "m.yaml"
    p.write_text(PROFILE.read_text())
    got = []
    w = prof.Watcher(p, got.append)
    assert not w.check()
    p.write_text(PROFILE.read_text().replace("max_feed_xy: 1000", "max_feed_xy: 900"))
    assert w.check() and isinstance(got[-1], prof.Profile) and got[-1].motion.max_feed_xy == 900
    p.write_text("controller: [")
    assert w.check() and isinstance(got[-1], prof.ProfileError)
    assert not w.check()                                   # unchanged since: no second report


def test_resolve_by_name_path_or_the_only_one(tmp_path):
    (tmp_path / "profiles").mkdir()
    (tmp_path / "profiles" / "one.yaml").write_text(PROFILE.read_text())
    assert prof.resolve(None, tmp_path).name == "one.yaml"
    assert prof.resolve("one", tmp_path).name == "one.yaml"
    (tmp_path / "profiles" / "two.yaml").write_text(PROFILE.read_text())
    with pytest.raises(FileNotFoundError, match="one, two"):
        prof.resolve(None, tmp_path)
    with pytest.raises(FileNotFoundError, match="No profile 'three'"):
        prof.resolve("three", tmp_path)
