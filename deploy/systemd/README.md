# Running the program as a service

Two user services (no root needed):

| Unit | Does |
|---|---|
| `pnp-portd.service` | the **port keeper**: holds the controller's USB port open for good (D22). A restart of the program then never resets an Arduino Uno. |
| `pnp.service` | the machine program and its page, `pnp serve` |

## Install

1. Install the program so `pnp` is in `~/.local/bin` (`pipx install .` or `uv tool install .` in this folder).
   From a checkout with a venv instead: change `%h/.local/bin/pnp` in both files to the venv's `bin/pnp`
   (or `.../.venv/bin/python -m pnp`, with `Environment=PYTHONPATH=<checkout>/src`).
2. Find the controller's stable device name: `ls /dev/serial/by-id/`. Put it in `pnp-portd.service` (`--device`).
3. In the machine profile, set `controller.port: socket://127.0.0.1:7450` (the port keeper).
4. Copy and start:

       mkdir -p ~/.config/systemd/user
       cp deploy/systemd/pnp-portd.service deploy/systemd/pnp.service ~/.config/systemd/user/
       systemctl --user daemon-reload
       systemctl --user enable --now pnp-portd pnp
       loginctl enable-linger $USER        # start at boot, without a login

## Every day

- Page: http://127.0.0.1:8000 (or the Pi's address with `--lan`).
- A page change: reload the browser. A profile change: just save the file (the program reads it again).
- A Python change: `systemctl --user restart pnp`. The port keeper keeps the USB port open, so the controller
  is NOT reset and the program finds its position again (the session token, D22).
- Log: `journalctl --user -u pnp -f`. Events: `var/events.jsonl` (checkout) or `~/.local/share/pnp/`.
- Stop everything: `systemctl --user stop pnp pnp-portd`.
