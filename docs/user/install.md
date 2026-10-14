# Install and first run

For the person who uses the machine. Simple steps; no YAML editing.

The complete guide, from install to troubleshooting, is [GUIDE.md](GUIDE.md).

## 1. Install (a PC or a Raspberry Pi 5 with Raspberry Pi OS Bookworm 64-bit)

You need Python 3.11 or newer and `pipx` (Raspberry Pi OS: `sudo apt install pipx`).

```
pipx install "git+<the repository's address>"      # or, from a copy of the folder:  pipx install ./pick_and_place_robot
pnp --version
```

- No root is needed. The program keeps its settings in `~/.config/pnp` and what it writes (jobs, photos, the event
  log) in `~/.local/share/pnp`.
- A bottom camera on the Pi's camera port needs `picamera2`: `sudo apt install python3-picamera2`, then
  `pipx install --system-site-packages ...`.
- Printing trays ("Add tray" makes an STL): `pipx inject pnp manifold3d trimesh`.
- The labels on a Brother PT-D410: install `ptouch-print` 1.7 or newer (its udev rule too).

## 2. Try it without a machine

```
pnp sim --open
```
The whole page runs against a simulated machine (a virtual controller and camera). Nothing real moves. Use it to learn
the steps below.

## 3. Your machine

```
pnp init                 # a few questions -> your machine's profile; unknown things stay "TBD"
pnp doctor               # the checks: green, or red with one line on what to do
pnp serve --open         # the page: http://127.0.0.1:8000  (--lan: also from a phone or tablet)
```

To start it with the computer, install the service: [../../deploy/systemd/README.md](../../deploy/systemd/README.md).

## 4. Set the machine up

Open the **Setup** tab and do the steps in order ([first-run.md](first-run.md)). Then a dry run: load a tray (Feeders
tab), Job tab, mode "Dry run (camera only)", Start. The camera visits each pick and each place; nothing goes down.

## 5. Back up, or copy to a second machine

Settings tab -> "Download the bundle" (or `pnp export backup.zip`). On the other computer: "Import a bundle" (or
`pnp restore backup.zip`). The bundle never contains the controller's position: the new machine homes itself.
