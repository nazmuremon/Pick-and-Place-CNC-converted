"""The nozzle-servo pulses of the bench Grbl (firmware/grbl-1.1h-servo/grbl/servo_spindle.c), run on the PC
against a model of the Uno's Timer2 (firmware/servo_host_test/). 1 tick = 2 us."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
GRBL = ROOT / "firmware" / "grbl-1.1h-servo" / "grbl"
HOST = ROOT / "firmware" / "servo_host_test"
FRAME = 40 * 256                     # ticks between pulse starts (20.48 ms)
LATE = 60                            # interrupts up to 120 us late (Grbl's step interrupt, serial)


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    if shutil.which("gcc") is None:
        pytest.skip("no gcc")
    exe = tmp_path_factory.mktemp("servo") / "servo_sim"
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Werror", "-DSERVO_HOST_TEST", f"-I{HOST}", f"-I{GRBL}",
                    str(GRBL / "servo_spindle.c"), str(HOST / "servo_host_sim.c"), "-o", str(exe)], check=True)

    def run(script: str) -> list[tuple[str, int]]:
        out = subprocess.run([str(exe)], input=script, capture_output=True, text=True, check=True).stdout
        return [(kind, int(t)) for kind, t in (line.split() for line in out.splitlines())]
    return run


def pulses(edges):
    """(start tick, width in ticks) of every complete pulse."""
    out, rise = [], None
    for kind, t in edges:
        if kind == "rise":
            rise = t
        elif rise is not None:
            out.append((rise, t - rise))
            rise = None
    return out


@pytest.mark.parametrize("us, ticks", [(1500, 750), (500, 250), (2500, 1250), (777, 389), (1001, 501),
                                       (2499, 1250), (3000, 1250), (100, 250)])
def test_pulse_width_is_exact_and_clamped_even_with_late_interrupts(sim, us, ticks):
    got = pulses(sim(f"us {us}\nstart\nrun {12 * FRAME} {LATE}\n"))
    assert len(got) >= 10
    assert {w for _, w in got} == {ticks}
    starts = [s for s, _ in got]
    assert all(b - a == FRAME for a, b in zip(starts, starts[1:]))   # one pulse every 20.48 ms


def test_a_new_width_is_used_from_the_next_pulse_on(sim):
    got = pulses(sim(f"us 1000\nstart\nrun {3 * FRAME + 100} {LATE}\nus 2000\nrun {4 * FRAME} {LATE}\n"))
    widths = [w for _, w in got]
    assert set(widths) <= {500, 1000} and widths[0] == 500 and widths[-1] == 1000
    assert widths == sorted(widths)                                  # never back to the old width


def test_stop_ends_a_pulse_at_once_and_no_pulse_follows_until_start(sim):
    edges = sim(f"us 2000\nstart\nrun {FRAME} 0\nrun 300 0\nstop\nrun {3 * FRAME} {LATE}\n")
    assert edges[-1][0] == "fall"                                    # the cut pulse ended at stop ...
    assert [k for k, _ in edges].count("rise") == 1                  # ... and nothing came after it
    again = pulses(sim(f"us 2000\nstart\nrun {FRAME} 0\nrun 300 0\nstop\nrun {FRAME} 0\nstart\n"
                       f"run {4 * FRAME} {LATE}\n"))
    assert [w for _, w in again][1:] == [1000] * (len(again) - 1)   # clean pulses after a restart


def test_no_pulses_before_start(sim):
    assert sim(f"us 1500\nrun {5 * FRAME} {LATE}\n") == []
