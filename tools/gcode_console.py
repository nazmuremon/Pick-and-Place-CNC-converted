#!/usr/bin/env python3
"""Simple serial console for Grbl / grblHAL.

Usage:
    .venv/bin/python tools/gcode_console.py [--port /dev/ttyUSB0] [--baud 115200]

Without --port it picks the first USB serial port it finds.

Inside the console:
    <any G-code or $ command>   send the line, wait for ok / error
    ?                           realtime status report
    !                           feed hold
    ~                           resume
    ctrl-x  (or: reset)         Grbl soft reset
    quit / exit / ctrl-d        leave

Everything sent and received is logged to logs/console/<timestamp>.log.
"""

from __future__ import annotations

import argparse
import datetime
import sys
import threading
import time
from pathlib import Path

import serial
from serial.tools import list_ports

from pnp_paths import LOGS_DIR

REALTIME = {"?", "!", "~"}
SOFT_RESET = b"\x18"
LOG_DIR = LOGS_DIR / "console"


def find_port() -> str:
    """Return the first USB serial port (CH340 clones show up as ttyUSB, real Unos as ttyACM)."""
    candidates = [p.device for p in list_ports.comports() if "USB" in p.device or "ACM" in p.device]
    if not candidates:
        sys.exit("No USB serial port found. Is the Arduino plugged in? (ls /dev/ttyUSB* /dev/ttyACM*)")
    if len(candidates) > 1:
        print(f"Several ports found, using the first: {candidates}")
    return candidates[0]


class Logger:
    def __init__(self) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.path = LOG_DIR / f"{stamp}.log"
        self._file = self.path.open("a", encoding="utf-8")
        self._lock = threading.Lock()

    def write(self, direction: str, text: str) -> None:
        t = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with self._lock:
            self._file.write(f"{t} {direction} {text}\n")
            self._file.flush()


def reader(ser: serial.Serial, log: Logger, stop: threading.Event) -> None:
    """Print every line the controller sends."""
    buf = b""
    while not stop.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            print("\n*** serial port lost ***")
            stop.set()
            return
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").strip()
            if text:
                print(f"  {text}")
                log.write("<", text)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="serial port (default: first USB port)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    port = args.port or find_port()
    log = Logger()
    print(f"Opening {port} at {args.baud} baud. Log: {log.path}")

    # Opening the port resets an Arduino Uno (DTR). Give Grbl time to boot,
    # then flush its banner so we start clean.
    from bench import mark_parked
    mark_parked(False, "gcode_console")      # G-code by hand can move Z: the nozzle tool starts locked next time
    ser = serial.Serial(port, args.baud, timeout=0.1)
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.write(b"\r\n")
    time.sleep(0.2)

    stop = threading.Event()
    t = threading.Thread(target=reader, args=(ser, log, stop), daemon=True)
    t.start()

    print("Connected. Type a command ($I to identify, $$ for settings). 'quit' to exit.")
    try:
        while not stop.is_set():
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line.lower() in {"quit", "exit"}:
                break
            if line.lower() == "reset" or line == "\x18":
                ser.write(SOFT_RESET)
                log.write(">", "<soft reset>")
                continue
            if line in REALTIME:
                ser.write(line.encode())  # realtime: no newline
                log.write(">", line)
                continue
            ser.write(line.encode() + b"\n")
            log.write(">", line)
            time.sleep(0.05)  # let the reply print before the next prompt
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        ser.close()
        print(f"\nBye. Log saved: {log.path}")


if __name__ == "__main__":
    main()
