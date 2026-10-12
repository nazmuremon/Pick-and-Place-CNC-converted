# Firmware for the bench Uno (decision D18)

The bench Uno runs **Grbl 1.1h with a nozzle servo** on the spindle pin. It was flashed on 2026-10-07.
`$I` shows `[VER:1.1h.20261007:]` and `[OPT:VM,…]`. The tools check that version before they send servo commands.

| File | What it is |
|---|---|
| `grbl-1.1h-servo/` | Grbl 1.1h.20190825 (github.com/gnea/grbl) plus our changes, listed below |
| `grbl-1.1h-servo.hex` | The build that is on the Uno now |
| `grbl-1.1h-stock.hex` | Plain Grbl 1.1h from the same source, to go back |
| `servo_host_test/` | Runs the servo pulse code on the PC (`tests/test_servo_firmware.py`) |

## What the build changes

- **D11** (the "Z+ / Z−" end-stop pin on the CNC shield) sends **hobby-servo pulses** instead of the spindle PWM:
  - `M3 S<us>` sets the pulse width in microseconds (500–2500, in 2 µs steps), one pulse every 20.48 ms.
  - `M5`, `S0`, a reset or an alarm stop the pulses, and the servo goes limp.
  - Like every spindle command, Grbl runs it only after the moves before it have finished. The servo
    still needs time to turn, so the app waits after each turn (`config/bench.yaml` `nozzle_servo.settle_s`).
  - The code is in `grbl/servo_spindle.c` and is called from `grbl/spindle_control.c`. Both pulse edges come
    from the Timer2 compare hardware, so a late interrupt never changes the pulse width.
- **`ENABLE_M7`** is on: the mist output **A4** (`M7`) is free for the vacuum valve later.
- The pump relay is on flood coolant, **CoolEn (A3)**: `M8` on, `M9` off.
- Everything else is plain Grbl 1.1h: steps, X/Y/Z pins and all `$` settings.

Needed settings: `$32=0` (laser mode off) and `$30=2500`, so the status report shows the pulse width.
The settings from before the flash are saved in `logs/grbl_settings_before_servo_firmware_2026-10-07.txt`.

## Build and flash

```bash
cd firmware/grbl-1.1h-servo && make          # avr-gcc; about 31.4 kB of the 32.25 kB the Uno has
cp grbl.hex ../grbl-1.1h-servo.hex
cd .. && avrdude -p atmega328p -c arduino -P /dev/ttyACM0 -b 115200 -D -U flash:w:grbl-1.1h-servo.hex:i
```

Close every bench tool first, because the port must be free. Flashing keeps the `$` settings (they are in EEPROM).

**To go back to plain Grbl:** run the same `avrdude` line with `grbl-1.1h-stock.hex`. Then move the pump relay
back to whatever pin you choose; the servo has no output on stock Grbl.
