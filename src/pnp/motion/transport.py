"""Opening the link to the controller without resetting it (docs/13 §5.1).

An Arduino Uno resets when DTR falls: the USB serial chip pulses its RESET pin through a capacitor. Linux raises
DTR when a port opens and drops it when the port closes (HUPCL). So:
- clear HUPCL right after opening: the next close keeps DTR up, and the next open makes no edge (no reset);
- better still, let the port keeper (`pnp portd`) hold the port open for good, and connect to it with
  `socket://127.0.0.1:7450`: a program restart then never touches the USB port.
The very first open after plugging in still resets an Uno. That is fine: the session token (D22) shows it.
"""
from __future__ import annotations

import sys

import serial
from serial.tools import list_ports


def find_port() -> str | None:
    ports = sorted(p.device for p in list_ports.comports() if "ACM" in p.device or "USB" in p.device)
    return ports[0] if ports else None


def clear_hupcl(fd: int) -> bool:
    """Keep DTR up when the port closes. Linux / macOS only; False where it can't be done."""
    if sys.platform.startswith("win"):
        return False
    try:
        import termios
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return True
    except (ImportError, OSError, ValueError):
        return False


def open_port(port: str | None, baud: int, timeout: float = 0.05):
    """A pyserial object for a device or a pyserial URL (socket://, rfc2217://, loop://)."""
    if port is None:
        port = find_port()
        if port is None:
            raise OSError("No USB serial port found. Is the controller plugged in?")
    if "://" in port:
        return serial.serial_for_url(port, baudrate=baud, timeout=timeout)
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, timeout
    s.write_timeout = 2.0
    s.open()                                   # DTR stays asserted (pyserial's default): never cycle it
    clear_hupcl(s.fileno())
    return s
