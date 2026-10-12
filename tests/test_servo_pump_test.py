"""tools/servo_pump_test.py against a fake Grbl: what it sends, and that it refuses a stock Grbl."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import servo_pump_test as spt  # noqa: E402
from bench import load_bench  # noqa: E402


class FakeGrbl:
    def __init__(self, version: str) -> None:
        self.version, self.sent = version, []

    def send(self, line, timeout=5.0):
        self.sent.append(line)
        return [f"[VER:{self.version}:]", "[OPT:VM,15,128]"] if line == "$I" else []


def test_refuses_a_grbl_without_the_servo_build():
    g, said = FakeGrbl("1.1h.20190825"), []
    assert not spt.run(g, load_bench(), True, True, None, False, say=said.append)
    assert g.sent == ["$I"] and "flash" in said[0]


def test_pump_then_sweep_ends_held_at_the_centre():
    bench = load_bench()
    g = FakeGrbl(bench["nozzle_servo"]["firmware"])
    assert spt.run(g, bench, True, True, None, False, say=lambda s: None)
    assert g.sent[1:4] == ["M8", "G4 P3", "M9"]
    servo = [c for c in g.sent if c.startswith("M3")]
    assert servo == ["M3 S1500", "M3 S500", "M3 S2500", "M3 S1500", "M3 S1000", "M3 S1500", "M3 S2000", "M3 S1500"]
    assert "M5" not in g.sent


def test_hold_is_clamped_and_off_stops_both():
    bench = load_bench()
    g = FakeGrbl(bench["nozzle_servo"]["firmware"])
    spt.run(g, bench, False, False, 4000, False, say=lambda s: None)
    assert g.sent[-1] == "M3 S2500"
    spt.run(g, bench, False, False, None, True, say=lambda s: None)
    assert g.sent[-2:] == ["M5", "M9"]
