# 04 – Electrical

> Some values depend on open questions Q2–Q4 (driver model, motors, pump). They are marked _TBD_.

## 1. Wiring overview

```
            230 V AC
               │
     ┌─────────┼──────────────────┐
     ▼         ▼                  ▼
  24 V PSU   12 V PSU        Pi 5 USB-C PSU (5 V 5 A, 27 W)
     │         │                  │
     │ E-STOP pole 1 (cuts 24 V and 12 V DC, via a DC-rated relay/contactor)
     ▼         ▼                  ▼
  drivers    pump, valve     Raspberry Pi 5 ──USB──▶ Pico (powered by USB)
  X Y Z      LED rings,           │         └─USB──▶ top camera
             servo buck (D18)     │
     ▲         ▲                  └─CSI─────────────▶ bottom camera
     │         │
     │     MOSFET modules ◀── Pico outputs (flood = pump, mist = valve)
     │     MG995 nozzle servo ◀── Pico spindle PWM, 50 Hz (D18; power from the servo buck)
     │
  STEP/DIR/EN ◀── 5 V buffer ◀── Pico GPIO (3.3 V)
                    ▲
               5 V buck (from 24 V) for the buffer and driver signal side

 E-STOP pole 2 ──▶ Pico E-stop input
```
The Pico output mapping (spindle PWM = nozzle servo, flood = pump, mist = valve, D18) is planned:
check it when there is a Pico (the owner has none now). The bench Uno already works this way (§10).

## 2. Power budget

| Rail | Loads | Size |
|---|---|---|
| 24 V | 3 existing drivers | 24 V, ~5 A (_TBD after Q3_) |
| 12 V | Pump (~0.5–1 A), valve (~0.3 A), 2 LED rings (~0.3 A), the servo buck converter (D18, _TBD_) | 12 V, 3 A |
| 5–6 V (servo) | MG995 nozzle servo (D18). It runs on 4.8–7.2 V and draws over 1 A when it stalls | Its own supply: for example a buck converter from the 12 V PSU, current rating _TBD_. About 1000 µF at the servo plug. **Never from the Uno or Pico 5 V pin** (§3) |
| 5 V (Pi) | Pi 5 + USB camera + Pico | Official 27 W USB-C PSU. A weaker PSU limits USB current |
| 5 V (signals) | 5 V buffer, driver opto inputs (up to ~15 mA each) | Small 5 V 1 A buck from 24 V |

The Pi 5 needs an **active cooler**, because vision keeps the CPU busy.

## 3. Motion signals

The Pico's GPIO is **3.3 V and is not 5 V tolerant**. Most external drivers want 5 V signals at 7–16 mA.

- Put a **74ACT245** between the Pico and the drivers. It accepts 3.3 V inputs, runs from 5 V and can drive 24 mA per output.
  A 74HCT245 also works if your driver needs less than ~6 mA. Check Q2.
  - Tie the 245's DIR pin to 5 V (A→B).
  - Tie OE to GND.
- **Opto-input drivers:** use common-anode wiring.
  - PUL+, DIR+ and ENA+ go to +5 V.
  - PUL−, DIR− and ENA− go to the buffer outputs.
  - If your driver only accepts 24 V signals, use an NPN transistor or ULN2803 instead (Q2).
- **Nozzle servo (MG995, D18)** is not on the buffer. Its signal comes from the spindle PWM
  output of the motion controller itself. There is no second controller and no extra USB cable.
  - Bench: our Grbl build on the Uno, pin D11 (§10). `M3 S<us>` = pulse width in microseconds
    (500–2500), one pulse every 20.48 ms. `M5` = no pulses (servo limp). See [../firmware/README.md](../firmware/README.md).
  - Pico: grblHAL spindle PWM at 50 Hz, planned. To be checked when there is a Pico.
  - Wire colours: brown = GND, red = +V, orange = signal (to the spindle PWM pin).
  - Power: its own 5–6 V supply (§2), a large capacitor (about 1000 µF) at the servo plug, and
    a common GND with the controller.
  - **Never power it from the Uno or Pico 5 V pin.** A voltage dip resets Grbl, and the position is lost.

### Pico pin map
Fill this in from the grblHAL RP2040 map you choose in the Web Builder (for example the generic Pico map).
Don't guess the pins. Copy them from the map file after you flash.
The outputs below follow D18 (planned, to be checked when there is a Pico).

| Signal | Pico GPIO | Goes to |
|---|---|---|
| X STEP / DIR | _TBD_ | Buffer → X driver |
| Y STEP / DIR | _TBD_ | Buffer → Y driver |
| Z STEP / DIR | _TBD_ | Buffer → Z driver |
| Steppers ENABLE | _TBD_ | Buffer → ENA of all drivers |
| Spindle PWM (nozzle servo, 50 Hz) | _TBD_ | MG995 signal (orange wire) |
| Flood coolant (pump) | _TBD_ | MOSFET 1 |
| Mist coolant (valve) | _TBD_ | MOSFET 2 |
| E-stop / reset input | _TBD_ | E-stop pole 2 (NC contact) |
| Driver ALARM | _TBD_ | Driver ALM outputs (open collector, pull-up to **3.3 V**) |
| Limit X / Y / Z (spare) | _TBD_ | Keep free for switches later |

## 4. I/O map (what the software uses)

| Function | grblHAL | G-code | Hardware |
|---|---|---|---|
| Axes | X, Y, Z | `G0`/`G1` | 3 drivers |
| Nozzle angle | Spindle PWM, 50 Hz (D18; planned on grblHAL, to be checked) | `M3 S<us>` (pulse width from the calibration table) / `M5` = limp | MG995 servo |
| Keep motors powered | Stepper enable | `$1=255` | ENA of all drivers |
| Vacuum pump | Flood coolant | `M8` / `M9` | MOSFET 1 → pump (12 V) |
| Vacuum valve (3/2-way) | Mist coolant | `M7` = vacuum, `M9` = vent | MOSFET 2 → valve (12 V) |
| Top LED ring | Pi GPIO12 (hardware PWM) | — | MOSFET 3 → LED ring |
| Bottom LED ring | Pi GPIO13 (hardware PWM) | — | MOSFET 4 → LED ring |
| E-stop | E-stop input | — | NC mushroom button, 2 poles |
| Driver faults | Fault / E-stop input (check what the RP2040 map offers) | — | ALM outputs; otherwise wire them into the E-stop chain |
| Vacuum sensor (phase 7) | — | — | XGZP6847A (3.3 V version) → ADS1115 → Pi I2C |

**Why the pump, valve and servo use spindle/coolant:** grblHAL (and Grbl) waits for all queued moves to finish
before it runs `M3/M5/M7/M8/M9`. So `G1 Z-4.2` followed by `M7` opens the valve only after Z has arrived.
**Note:** `M9` turns off both mist and flood: it vents the valve **and** stops the pump. The app sends `M8`
again after each place (see [07-software.md](07-software.md)). (Before D18 the pump was on the spindle output.)

LED rings aren't timing-critical, so they run from the Pi. Use a MOSFET, not a GPIO directly.

## 5. Valve and vacuum

- The 3/2-way valve must be **normally closed and rated for vacuum**. Connect it like this:
  - common port → nozzle
  - NC port → pump inlet
  - exhaust port → air
- Valve off = nozzle vented (part released). Valve on = nozzle connected to the pump (part held).
- The pump runs (`M8`) while the head works. `M9` (vent) also stops it, so the app turns it on again with `M8` after each place (D18).

## 6. MOSFET outputs

- Use logic-level N-MOSFETs as low-side switches (AO3400, IRLZ44N, or ready-made modules).
- Gate: 100 Ω series resistor, and 10 kΩ to GND so the output stays off during boot.
- **Put a flyback diode across every coil**, pump and valve (1N5819 or 1N4007, cathode to +12 V).

## 7. Safety

- **E-stop pole 1** cuts 24 V and 12 V DC through a relay or contactor rated for DC. A plain AC switch can arc on DC.
- **E-stop pole 2** tells grblHAL, so the software knows about the stop.
- **After an E-stop or driver ALARM, the position is unknown.** The app must refuse to move until the camera homing is done again.
- Put the pump, valve and PSUs in a closed box. Add fuses on 24 V and 12 V.

## 8. Grounding and cables

- Use a star ground at the PSU minus terminals. Join the grounds of the Pico, buffer and 5 V buck,
  and of the servo supply and the motion controller (Uno or Pico, D18).
- For opto-isolated drivers, keep the motor-power side separate from the signal side.
- Keep the motor cables away from the camera USB cable and the CSI ribbon. Use shielded motor cable if you can.
- Cable chain: high-flex USB cable for the top camera, 3-wire servo cable (brown, red, orange), 2-wire for the LED ring, vacuum tube.

## 9. Wiring checklist (phase 1–2)

- [ ] Check the voltage of every PSU with no load connected
- [ ] Buffer powered from 5 V, DIR and OE tied, check outputs with a scope or logic analyser
- [ ] One axis at a time: step, direction and enable work, motor current set
- [ ] E-stop cuts motion and grblHAL shows an alarm
- [ ] Pump switches from `M8`/`M9`, valve from `M7`/`M9`, flyback diodes fitted
- [ ] Nozzle servo (D18): own 5–6 V supply (not the Uno or Pico 5 V pin), about 1000 µF at the plug, common GND; it turns with `M3 S<us>` and goes limp with `M5`
- [ ] Photos of the wiring saved in `hardware/wiring/`

## 10. Bench (Uno) wiring

The test bench (until the Pico is wired) is an Arduino Uno + CNC shield V3 with **our Grbl 1.1h servo build**
(`$I` = `VER:1.1h.20261007`, flashed 2026-10-07, D18). Build, flash and rollback: [../firmware/README.md](../firmware/README.md).
Wiring diagram (pump relay, servo supply, pins): [hardware/wiring/bench_pump_servo_wiring.html](../hardware/wiring/bench_pump_servo_wiring.html).
Some shield labels don't match Grbl 1.1:

| Function | Uno pin (shield label) | G-code | Hardware |
|---|---|---|---|
| Nozzle servo signal | D11 (printed "Z+ / Z-" on the end-stop header: the signal pin, not the GND pin) | `M3 S<us>` (500–2500 µs) / `M5` = limp | MG995 orange wire. Servo power from its own 5–6 V supply (§2) |
| Vacuum pump | A3 ("CoolEn", flood) | `M8` / `M9` | Relay module 1 |
| Vacuum valve (later) | A4 (mist; `ENABLE_M7` is on in our build) | `M7` / `M9` | Relay module 2 |

- The shield pin labelled **"SpnEn" (D12) is NOT the spindle** in Grbl 1.1: it is the Z limit input. That stays so.
- D11 is Grbl's only PWM pin. In our build it sends servo pulses: one every 20.48 ms, in 2 µs steps.
  `M5`, `S0`, a reset or an alarm stop the pulses (servo limp). Needed Grbl settings: `$30=2500`
  (the status report shows the pulse width), `$32=0` (laser mode off).
- Test: `tools/servo_pump_test.py` (pump 3 s, servo sweep; `--us`, `--off`). Settings: `config/bench.yaml` `nozzle_servo` and `pump`.
- Relay modules must be **active-HIGH**, with a **10 kΩ pull-down** on IN (the relay stays off while the Uno boots).
- **One buck converter set to 5.0 V** (from the pump supply) feeds the servo **and** the relay module's VCC: nothing takes power from the Uno, and no 5 V pin of the shield is needed (2026-10-07). The MG995 runs on 4.8–7.2 V.
- **Common ground:** buck OUT−, servo brown and relay GND go to the **CoolEn row's GND pin**. In every row of the shield's header block the pin at the board edge is GND, and the inner one is the signal.
- Pump and valve through relay COM/NO, from the 12 V PSU, with a fuse. A flyback diode across
  the pump and across the valve (1N5819 or 1N4007, stripe to +).
