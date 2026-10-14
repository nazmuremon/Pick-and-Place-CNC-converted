"""The port keeper (`pnp portd`, D22): holds the controller's USB serial port open for good and passes its bytes
to ONE client on a local TCP port. The program connects with `socket://127.0.0.1:7450`, so restarting the
program (a code or config change) never closes the USB port and never resets an Arduino Uno.

Only 127.0.0.1 by default: anyone who can reach this port can move the machine.
"""
from __future__ import annotations

import select
import socket
import time
from typing import Callable

from .transport import open_port


def serve(device: str | None, baud: int = 115200, host: str = "127.0.0.1", port: int = 7450,
          log: Callable[[str], None] = print, stop: Callable[[], bool] = lambda: False) -> None:
    ser = open_port(device, baud, timeout=0)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(2)
    log(f"Port keeper: {ser.port} at {baud} baud, clients on {host}:{port}")
    client: socket.socket | None = None
    seen = b""
    try:
        while not stop():
            socks = [srv, ser] + ([client] if client else [])
            ready, _, _ = select.select(socks, [], [], 0.5)
            for r in ready:
                if r is srv:
                    conn, addr = srv.accept()
                    if client is not None:
                        conn.sendall(b"[MSG:port keeper: another program is connected]\r\n")
                        conn.close()
                        log(f"Refused a second client from {addr}")
                        continue
                    client = conn
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    log(f"Client connected from {addr}")
                elif r is ser:
                    data = ser.read(4096)
                    if not data:
                        continue
                    seen = (seen + data)[-200:]
                    if client is not None:
                        try:
                            client.sendall(data)
                        except OSError:
                            client.close()
                            client = None
                    elif b"Grbl" in seen:
                        log("The controller printed its start banner while no program was connected (a reset)")
                        seen = b""
                elif r is client:
                    try:
                        data = client.recv(4096)
                    except OSError:
                        data = b""
                    if not data:
                        log("Client left (the USB port stays open)")
                        client.close()
                        client = None
                    else:
                        ser.write(data)
    finally:
        if client is not None:
            client.close()
        srv.close()
        ser.close()
        time.sleep(0.05)
