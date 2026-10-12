# 10 – Bill of Materials

Prices are rough, in US$, and will change. Check the ones marked _TBD_ after answering the
[open questions](01-requirements.md#open-questions).

## Already have

| Item | Use | Notes |
|---|---|---|
| CNC frame, rails, screws, motors | X, Y, Z motion | Broken part: _TBD (Q1)_ |
| 3 × microstep/servo stepper drivers | X, Y, Z | Model: _TBD (Q2)_ |
| Raspberry Pi 5, 8 GB | Main computer | Add an active cooler |
| DC vacuum pump + nozzle | Vacuum | Voltage and vacuum level: _TBD (Q4)_ |
| Laser printer | Bed labels on A4 matte sticker paper (D13: HOME, PCB corners, feed slots), ChArUco calibration board, 50 mm scale bar (on matte paper) | Always print at 100 % ("Actual size") |
| Brother PT-D410 label printer + 18 mm TZe tape | Bed labels (D15, `make_labels.py --tape`), part labels for a load (optional, through solder_helper) | Already used by solder_helper. The bed labels were printed on it (2026-10-06) |

## To buy: required

| # | Item | Qty | Use | ~US$ |
|---|---|---|---|---|
| 1 | Raspberry Pi Pico (or Pico 2) | 1 (+1 spare) | grblHAL motion controller | 5 |
| 2 | 74ACT245 (or 74HCT245) + DIP socket | 1 | 3.3 V → 5 V step/direction/enable buffer | 1 |
| 3 | 5 V 1 A buck converter | 1 | Buffer and driver signal side | 2 |
| 4 | MG995 hobby servo | 1 | Nozzle rotation (D18). No hollow shaft: the nozzle sits on the servo horn. Signal from the spindle PWM pin of Grbl/grblHAL: no extra controller | _TBD_ |
| 5 | 5–6 V buck converter (from the 12 V PSU), current rating _TBD_ (the MG995 draws over 1 A when it stalls) | 1 | Servo power (D18). Never the Uno or Pico 5 V pin | _TBD_ |
| 6 | Electrolytic capacitor, about 1000 µF | 1 | At the servo plug (D18) | _TBD_ |
| 7 | 3/2-way mini solenoid valve, 12 V, NC, **vacuum rated** | 1 | Vacuum / vent switching | 5–10 |
| 8 | Logic-level MOSFET modules | 4 | Pump, valve, 2 LED rings | 4 |
| 9 | Flyback diodes (1N5819 / 1N4007) | 4 | Pump and valve | 1 |
| 10 | Blunt luer-lock dispensing needles, assorted 14G–25G | 1 set | Nozzles | 5 |
| 11 | Small compression springs + 3D-printed holder | few | Spring nozzle | 2 |
| 12 | USB camera, 1080p, **manual focus**, UVC | 1 | Top camera | 25–45 |
| 13 | Raspberry Pi Camera Module 3 + Pi 5 camera cable | 1 | Bottom camera | 25–30 |
| 14 | LED ring (~40–60 mm) + diffuser | 2 | Lighting | 10 |
| 15 | Silicone tube 2–4 mm inner diameter (soft) | 2 m | Vacuum line, and the loop to the nozzle on the servo (D18) | 3 |
| 16 | Cable chain + high-flex USB cable | 1 | Head cables | 10–15 |
| 17 | E-stop mushroom button, 2 poles (NC) + DC-rated relay/contactor | 1 | Safety | 10–15 |
| 18 | 12 V 3 A power supply | 1 | Pump, valve, LEDs, the servo buck converter (D18) | 8 |
| 19 | Official Raspberry Pi 5 27 W USB-C power supply | 1 | Pi 5 | 12 |
| 20 | Raspberry Pi 5 active cooler | 1 | Pi 5 | 5 |
| 21 | 3 mm dowel pins | 2 (+ spares) | PCB holder | 5 |
| 22 | Fuses + holders (24 V, 12 V) | 2 | Protection | 3 |
| 23 | PETG filament (matte black) | 1 kg | Head plate, PCB holder | 20 |
| 24 | A4 **matte** sticker paper (laser or inkjet, to match your printer). Not glossy (glare) | 1 pack | Bed labels (D13). One sheet holds all 13 labels; keep spares for reprints | ~5 (rough) |
| | **Total (approx.)** | | | **~$165–225**, plus the _TBD_ items |

24 V power supply: use the CNC's own supply if it can deliver ~5 A. Otherwise add one (~$20) _(TBD after Q3)_.

## Optional / later

| Item | Use | ~US$ |
|---|---|---|
| XGZP6847A vacuum sensor (3.3 V) + ADS1115 | Checking picks (phase 7) | 10 |
| 3 × micro limit switches | Automatic rough homing (phase 7) | 3 |
| Small rotary union | Instead of the silicone tube loop on the nozzle servo (D18) | _TBD_ |
| Dial indicator + magnetic base | Backlash and repeatability checks (phase 0–1) | 20 |
| Logic analyser (8 channels, cheap) | Checking step/direction signals | 10 |
| Raspberry Pi AI HAT+ | Only if classic vision is ever not enough | 70+ |
