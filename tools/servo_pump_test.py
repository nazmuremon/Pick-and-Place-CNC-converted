#!/usr/bin/env python3
"""Pump relay and nozzle servo test on the bench Uno (decision D18). Only the servo moves.

    .venv/bin/python tools/servo_pump_test.py              # pump 3 s, then a servo sweep
    .venv/bin/python tools/servo_pump_test.py --servo      # only the servo sweep
    .venv/bin/python tools/servo_pump_test.py --pump       # only the pump
    .venv/bin/python tools/servo_pump_test.py --us 1500    # hold one pulse width (microseconds)
    .venv/bin/python tools/servo_pump_test.py --off        # servo limp, pump off

Needs our Grbl build (config/bench.yaml nozzle_servo.firmware): D11 ("Z+/Z-" pin on the CNC shield)
sends the servo pulses, CoolEn (A3) drives the pump relay. The sweep ends with the servo held at the
centre. Opening the port again (any bench tool) resets the Uno: the servo goes limp until the next M3.
"""
from __future__ import annotations

import argparse

from bench import Grbl, load_bench, mark_parked, was_parked

PUMP_S = 3.0


def sweep_us(cfg: dict) -> list[float]:
    """Centre, both ends, back to the centre, then the quarter points."""
    lo, hi, c = float(cfg["min_us"]), float(cfg["max_us"]), float(cfg["centre_us"])
    q = (hi - lo) / 4
    return [c, lo, hi, c, lo + q, c, hi - q, c]


def firmware_ok(grbl, cfg: dict) -> bool:
    return any(f"VER:{cfg['firmware']}:" in line for line in grbl.send("$I"))


def run(grbl, bench: dict, pump: bool, servo: bool, hold_us: float | None, off: bool, say=print) -> bool:
    s, p = bench["nozzle_servo"], bench["pump"]
    if not firmware_ok(grbl, s):
        say(f"This Uno does not run the servo Grbl build ({s['firmware']}): flash firmware/grbl-1.1h-servo.hex "
            "first (see firmware/README.md). Nothing sent.")
        return False
    if off:
        grbl.send("M5")
        grbl.send(p["stop"])
        say("Servo limp (no pulses), pump off.")
        return True
    if hold_us is not None:
        us = min(max(hold_us, float(s["min_us"])), float(s["max_us"]))
        grbl.send(f"M3 S{us:.0f}")
        say(f"Servo held at {us:.0f} us. It stays there until M5, a reset, or the next bench tool opens the port.")
        return True
    if pump:
        say(f"Pump: {p['start']} for {PUMP_S:g} s. The relay must click on, the pump run, then click off.")
        grbl.send(p["start"])
        grbl.send(f"G4 P{PUMP_S:g}", timeout=PUMP_S + 5)
        grbl.send(p["stop"])
    if servo:
        settle = float(s["settle_s"])
        for us in sweep_us(s):
            say(f"Servo: M3 S{us:.0f} ({us:.0f} us)")
            grbl.send(f"M3 S{us:.0f}")
            grbl.send(f"G4 P{settle:g}", timeout=settle + 5)
        say(f"Sweep done: the servo is held at the centre ({float(s['centre_us']):.0f} us).")
    return True


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pump", action="store_true", help="only the pump")
    ap.add_argument("--servo", action="store_true", help="only the servo sweep")
    ap.add_argument("--us", type=float, help="hold this pulse width (microseconds)")
    ap.add_argument("--off", action="store_true", help="servo limp, pump off")
    args = ap.parse_args()
    bench = load_bench()
    parked = was_parked()                       # before the port opens (that marks it "not parked")
    grbl = Grbl(cfg=bench)
    try:
        grbl.check_ready()
        both = not (args.pump or args.servo)
        run(grbl, bench, args.pump or both, args.servo or both, args.us, args.off)
    finally:
        grbl.close()
        mark_parked(parked, "servo_pump_test (no Z moves)")


if __name__ == "__main__":
    main()
