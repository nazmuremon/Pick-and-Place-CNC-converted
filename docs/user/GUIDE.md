# The complete guide: using the pick and place program

This guide is for anybody: you do not need to know the code. It explains what the program does, how to install
it, how to set a machine up the first time, how to run a job, how to finish a board by hand, and what to do when
something goes wrong. The words are kept simple on purpose.

> **Status (October 2026):** the program has been built and tested against its **simulator** (a virtual machine
> inside the program). It has not yet been run on the real machine for every step. Try everything in the simulator
> first, and keep a hand near **STOP** the first time you do each step on a real machine.

---

## Contents

1. [What the program does](#1-what-the-program-does)
2. [What you need](#2-what-you-need)
3. [Install it](#3-install-it)
4. [Try it without a machine (the simulator)](#4-try-it-without-a-machine-the-simulator)
5. [Make a profile for your machine](#5-make-a-profile-for-your-machine)
6. [Start the program and open the page](#6-start-the-program-and-open-the-page)
7. [A tour of the page](#7-a-tour-of-the-page)
8. [Safety first: STOP, locks and limits](#8-safety-first-stop-locks-and-limits)
9. [The first set-up, step by step](#9-the-first-set-up-step-by-step)
10. [Import a board](#10-import-a-board)
11. [Trays: add, load, refill](#11-trays-add-load-refill)
12. [Run a job](#12-run-a-job)
13. [Finish the board by hand (the Hand tab)](#13-finish-the-board-by-hand-the-hand-tab)
14. [Every day: start, stop, after a power cut](#14-every-day-start-stop-after-a-power-cut)
15. [Back up, copy to another machine, get help](#15-back-up-copy-to-another-machine-get-help)
16. [Upgrades: more hardware later](#16-upgrades-more-hardware-later)
17. [Troubleshooting](#17-troubleshooting)
18. [Keyboard shortcuts](#18-keyboard-shortcuts)
19. [Words used in this guide](#19-words-used-in-this-guide)

---

## 1. What the program does

The program runs a small home-made **pick and place machine**: a machine that takes tiny electronic parts (SMD
parts) out of tape strips and puts them on a printed circuit board (PCB), on the solder paste, before the board goes
into the oven.

- It moves the machine through a **controller** (an Arduino with Grbl, or a Pico with grblHAL). The computer never
  makes the motor pulses itself.
- It **sees** with a camera on the head: it finds the machine's home mark, the trays, the parts in the tape and the
  board's holes, and it checks every pick and every place.
- It reads your **board files** (KiCad, Gerber X3, or a "pick and place" / centroid file from Altium, EasyEDA, JLC,
  Eagle, or any CSV).
- It plans the job: which parts the machine places, which tray holds which part, and **why** every other part is
  left for your hands.
- It also helps you **solder the rest by hand** (the Hand tab): which drawer holds which part, where each part goes,
  and what is still missing. It replaces the older "Solder Helper" page.

Everything happens in a **web page** in your browser. The program itself runs on the machine's computer (a
Raspberry Pi 5, or any Linux PC).

---

## 2. What you need

**Hardware (the machine):**

- An XY(Z) machine with stepper motors and a controller running **Grbl 1.1** or **grblHAL**.
- A **vacuum pump** and a needle (nozzle) on the Z axis, switched by the controller (for example the coolant output).
- A **USB camera** on the head, looking down at the bed.
- A way to turn the nozzle (optional): a hobby servo on the controller's spindle pin, or a stepper axis.
- Printed **labels with markers** on the bed (a HOME label, one per tray), and trays or labels for the tape strips.
- Optional upgrades: limit switches, a Z probe, a bottom camera, a valve and vacuum sensor, a head light.

**Computer:**

- A Raspberry Pi 5 with Raspberry Pi OS (Bookworm, 64-bit), or a Linux PC.
- Python 3.11 or newer.
- A web browser (on the same computer, or a phone / tablet / PC on the same network).

**Optional:**

- A Brother **PT-D410** label printer (for drawer labels), with `ptouch-print`.
- A USB or Bluetooth **barcode scanner** for the LCSC part bags (the camera works too).
- A 3D printer for the strip holders ("Add tray" makes the STL file).

---

## 3. Install it

Open a terminal on the machine's computer.

**From a copy of the project folder:**

```
sudo apt install pipx            # once (Raspberry Pi OS / Debian / Ubuntu)
pipx install ./pick_and_place_robot
pnp --version
```

**Optional parts:**

| You want | Run |
|---|---|
| "Add tray" to make 3D-printable holders (STL) | `pipx inject pnp manifold3d trimesh` |
| A bottom camera on the Raspberry Pi camera port | `sudo apt install python3-picamera2`, then install with `pipx install --system-site-packages ./pick_and_place_robot` |
| Labels on a Brother PT-D410 | install `ptouch-print` 1.7 or newer (with its udev rule) |

**Where the program keeps its files** (no root needed):

| What | Where |
|---|---|
| Your settings (the profile, the bed, the trays, the package list) | `~/.config/pnp/` |
| What it writes by itself (jobs, photos, calibrations, the event log) | `~/.local/share/pnp/` |

When you run it from inside the project folder instead, it uses `config/` and `var/` there.

**To start it with the computer** (a service that restarts by itself): see `deploy/systemd/README.md`.

---

## 4. Try it without a machine (the simulator)

```
pnp sim --open
```

This opens the page against a **simulated machine**: a virtual controller, a virtual camera that draws the bed, a
virtual needle that picks up virtual parts. Nothing real moves. A purple **SIMULATOR** badge is shown at the top.

Use it to learn every step of this guide before you touch the real machine. In the simulator, the Machine tab has a
**Simulator** card with "props": lay the calibration sheet on the bed, put a board down, lay a strip of parts in a
tray, press a virtual E-stop, cause a power dip. Use them to practise the set-up and the recovery.

---

## 5. Make a profile for your machine

A **profile** is one file that describes your machine: which controller, which axis is which motor, how big the
travel is, which camera, which nozzles, how the pump is switched. Everything special about your machine is in it,
never in the program.

The easy way:

```
pnp init
```

It asks a few questions (a name, the controller, the port, homing, Z reference, rotation, camera) and writes the
profile. **Anything it does not know stays "TBD"** (null): the program switches that function off and tells you what
is missing. It also copies an example bed file and the package library into your config folder.

Then check it:

```
pnp profile check my_pnp     # is the profile valid, what is still TBD
pnp doctor                   # all the checks, green or red, each with what to do
```

Two example profiles come with the program: `bench_uno` (an Arduino Uno test bench) and `pico_grblhal` (a Pico with
grblHAL). Field-by-field explanations: [docs/builder/profiles.md](../builder/profiles.md).

> **Never invent a number.** If you do not know a value (a spring travel, a settle time), leave it TBD. The program
> refuses the actions that need it and the Setup tab shows how to measure it.

---

## 6. Start the program and open the page

```
pnp serve --open                 # the real machine
pnp serve --lan                  # also reachable from a phone / tablet on your network
```

The page is at **http://127.0.0.1:8000** (or the computer's address on your network, with `--lan`).

- The page reconnects by itself if the program restarts.
- Saving the profile file is enough: the program reads it again by itself. A file with errors never replaces the
  running profile.
- **Only one program may use the controller's USB port.** Do not run two of them at the same time.

---

## 7. A tour of the page

At the top: the **banner** (one message: what state the machine is in and what to do next) and the big red
**STOP** button. Below, the tabs (keys 1 to 9 switch between them):

| Tab | What it is for |
|---|---|
| **Machine** | The live camera picture, the bed map, jogging X / Y / Z, the nozzle turn, the pump, the light, the exposure. Click on the picture: the head goes there. The red cross is the picture centre; the blue ring is where the nozzle tip would land. |
| **Setup** | The set-up wizard: every calibration step in order, with its status (valid / stale / missing), how to do it, and Undo. On the right: a close-up of the picture around the nozzle tip, for touching off Z. |
| **Feeders** | The trays on the bed: add, remove, load a strip, refill, the load sheet. |
| **Board** | Import a board, see it from the top and from the bottom, check it, register it on the machine. |
| **Job** | The plan, start / pause / resume / abort a job, the parts and their states, the reports. |
| **Hand** | Finish the board by hand: what is left, which drawer holds it, where it goes. Bag scanning and labels. |
| **Doctor** | All the checks: green, or red with one line on what to do. |
| **Settings** | The profile (what is TBD, what each capability can do), export / import of everything. |
| **Logs** | The last 200 events (what the machine did, warnings, errors). |

The **mode line** under the camera picture always says what a click on the picture will do now.

---

## 8. Safety first: STOP, locks and limits

- **STOP** (the red button, or the **Space** key on most tabs) stops the machine at once. A jog stops without losing
  the position. In a job, the nozzle lifts; if a part is on the nozzle, the pump stays on and the program asks you
  what to do with it.
- **The machine has no limit switches** (unless you add them): the program refuses every move outside the
  **travel** it measured. Measure the travel before long moves (Setup, step 2).
- **Before Home**, X / Y jogs are slow and "go to" is not allowed.
- **Z goes down by itself** only after Home, with a valid Z reference and nozzle offset, and only over a tray, the
  registered board or the mark sheet. X / Y never move with the nozzle low.
- **Locks.** When something makes the position unsure (a controller reset, an ALARM, a stall, the USB link lost, the
  E-stop), the machine **locks**: only Z up, the pump and the light work. The banner says what happened and what to
  do. The usual way out: lift the nozzle with Z up, press **"Nozzle is clear"**, then **Home** (and set Z again).
  Every lock and its way out: [locks.md](locks.md).
- The camera checks every automatic move: if the picture did not move as it should (the head hit an end, the motor
  lost steps), the machine locks.

---

## 9. The first set-up, step by step

Open the **Setup** tab. Do the steps **in order**: a step stays grey until the steps it needs are valid. Each step
has a **"How"** box with three lines: what **you** do, what **the machine** does, and when it is **done**. Each
saved result can be **undone** (it keeps the last five values).

Status words:

- **valid**: measured, and still true.
- **stale**: something it was built on changed (for example the camera was calibrated again): measure it again.
- **missing**: not done yet.

### Step 1: Connect and check

Plug in the controller and the camera. Press **Connect**. The program reads the controller's version and settings
and compares them with the profile. Press **Test the outputs**: the pump clicks on and off, the nozzle turns a
little. The step lists what is still TBD in the profile.

### Step 2: Travel (how far the head may go)

1. **Home** first (step 4 comes later in the list, but Home works as soon as the camera is calibrated; on the very
   first day you can also use the travel written in the profile).
2. For each end (X−, X+, Y−, Y+): press **Creep**. The head moves 2 mm at a time; after each step the camera checks
   the picture moved. The first step it does not see is the end. The end is kept a step and 2 mm inside. Keep a hand
   near STOP.
3. Or jog there yourself and press **End here**.
4. After an end was found the head may have lost a step: jog back and **Home** again.
5. **Save the travel**.

With limit switches this step is **"Homing switches"**: Home once, jog until the HOME label is in view, press **Test
the switches** (it homes 3 times and measures how well they repeat; the travel comes from the controller).

### Step 3: Camera (exposure, lens, tilt, heights)

1. Lights on. Press **Find and lock the exposure** (the camera's automatic exposure is turned off on purpose: a
   lamp change must not change what the camera sees).
2. Print the **calibration sheet** (link in the step) at **100 % / "Actual size"**. Measure both 100 mm bars: they must
   be 100.0 mm. Stick one sheet on something flat.
3. Lay it flat on the bed where the head has room around it (25 mm left and right, 15 mm front and back), press
   **Sheet on the bed**: the head takes 9 pictures.
4. Put the sheet on a block whose height you measured with a caliper, type the height, press **Sheet on the block**.
5. Take the sheet away, press **Solve and save**. The step shows how tilted the camera is, how high, how many pixels
   per mm, and whether the head moves as counted (lost steps show here).

### Step 4: Home and the Z reference

1. Jog until the **HOME label** is in the picture. Press **Home**.
2. The Z touch-off: jog the nozzle down over the HOME label in 1 mm steps until it is close, then 0.1, then 0.02
   (the Z step buttons beside the close-up on the right), until the tip **just touches** (the needle's spring starts
   to give). Type the height of what it touches (0 for the label surface) and press **Set Z here**.
   With a Z probe the button is **Probe Z** instead.

### Step 5: Nozzle offset (where the tip is, seen from the camera)

**With ink (best):** lay a small sheet of white paper on the bed **between two trays** (not on a tray), under the
nozzle (the blue ring on the picture). Measure the paper's thickness and type it. Put a little ink on the needle tip.
Press **Make marks**. The nozzle touches the paper at 5 angles; the camera measures every mark. The marks give the
offset and how far the needle is off the servo's axis.

**By eye (no ink):** jog the tip onto a sharp corner (for example a label corner), press **The tip is on it**, lift,
press **Then click the spot**, and click that exact corner on the picture.

An offset that changes by more than 1 mm waits for you to press **Accept**.

**With more than one nozzle:** do this step for each nozzle (put it on, tell the program with "Nozzle changed" on the
Job tab, measure).

### Step 5b: Bottom camera (only if you have one)

Jog the **bare** needle over the bottom camera and down until the needle is sharp in the bottom picture. Press
**Measure**. From then on every part is looked at from below before it is placed, and its real position and angle on
the needle are corrected.

### Step 6: Rotation

- **Servo:** the same paper, with ink: **Measure the angle table**. It finds how far the servo really turns for each
  pulse, and how well it repeats.
- **Stepper axis:** **Measure the axis** (its real steps per degree, its backlash, its repeat).
- **No rotation:** this step is skipped; parts that must turn are placed by hand.

### Step 6b: Vacuum sensor (only if you have one)

Needle in the air: **Open**. Press the tip on the paper (or hold a part on it): **Closed**. A part counts as held
above half way between the two.

### Step 7: Heights

Nothing to type. The program works every height out: pick = the part's top − a small press; place = the board's
top + the part's height − the press; moving = above the highest thing + a clearance. Check the numbers; if one looks
wrong, fix its source (the press and clearance in the profile, the board's thickness, the tray's height).

### Step 8: Bed survey

Put the trays where the bed map shows them. Press **Survey**. The camera finds every tray label and measures where
it really is and which way it points. Do this again whenever you move or add a tray.

### Step 9: Board

See [section 10](#10-import-a-board). The step is valid when the board is registered.

### Step 10: First part

1. Lay a strip of a simple part (a resistor or a capacitor) in a tray. Choose the tray and the package, press
   **Look at the slot**. The program shows the first part it found: say **Yes, a part** (or No).
2. Type a board part of that package (for example R41), press **Place it**.
3. Look at the before / after photos. If it is good, press **OK**. If it is off, type how far to move it (dx, dy in
   mm, dθ in degrees) and press **Move it by this**: the correction is used by every job. Place another to check.

When all steps show **valid**, the machine is ready for jobs.

---

## 10. Import a board

**Board** tab → **Import a board**:

| You have | Give it |
|---|---|
| KiCad | the `.kicad_pcb` file (everything is in it: outline, parts, pads, holes, thickness) |
| Gerbers | a `.zip` with the Gerbers, the **X3 position files** (KiCad: Fabrication outputs → Component placement, Gerber), the drill files and the job file |
| A centroid / pick and place file | the `.csv` / `.pos` / `.txt` / `.mnt`; add the Gerbers or use the current board as the reference for the outline and pads |

After the import:

- Look at the board **from the top and from the bottom**. Every part has a colour: placed by the machine, needs the
  bottom camera, by hand, through-hole, pad only, mounting hole, unknown. A short white line shows each part's 0°
  direction; a red dot shows pin 1.
- Read **Check before the first place**: parts whose centre is not on their pads, two-pad parts whose pads do not
  lie along their angle, pin 1 in the wrong corner. Fix the file, or the package library, before you place.
- Check the **thickness** (it is written in the KiCad file; otherwise type it).
- A part counts as SMD when it has **no through-hole pads**. The program never trusts a tool's "SMD" flag.

**Register the board on the machine:**

1. Lay the board in the board area.
2. Choose the **side up** (top F, or bottom B) and where the board's **top edge** points (back, front, left, right).
   This matters: the holes alone cannot tell a board turned round.
3. Press **Register**. The camera looks at the holes and corners it can reach and fits the board's position and turn.
4. On the **Machine** tab, the board's pads are drawn on the live picture: they must sit **on the real pads**. If they
   do not, the side or the top edge is wrong.

**A panel (several boards of the same kind):** register copy 1 as above; for copy 2, 3, … type the copy number and
where its middle lies (X / Y in mm from HOME), then Register. The job then places the parts on every copy.

---

## 11. Trays: add, load, refill

### The trays on the bed

**Feeders** tab: every tray with its number, its marker, its kind, whether the **nozzle** can reach it
("reachable", "partly", "camera only", "out of reach"), the line it holds and how many parts are used.

### Add a tray

Choose the number (1–99), the tape width (8, 12, 16 or 24 mm), the kind, and where you will put it (X / Y in mm from
HOME). Kinds:

- **3D printed holder**: the program makes the **STL** file (download link). Print it.
- **paper label**: the program makes a **PDF**; print it at 100 % and check the 100 mm bar.
- **matrix tray** (a grid of pockets): give the rows, columns, pitch, the first pocket's distance from the marker,
  and **measure** two heights: where the parts sit, and the tray's top (where its marker is).
- **stick** (a tube that feeds one pick point): the pick point's distance from the marker and the same two heights.

The program picks a free marker id for it. Put the tray on the bed, then **Setup → step 8 (Survey)**.

### Load a strip into a tray

1. Cut a strip from the reel. Peel the cover film. Lay it in the tray: **sprocket holes to the side the top of the
   tray number points to**, parts on the centre line. Hold it flat.
2. Choose the tray and the **line** (value + package) and press **Strip is in: look at it**.
   Or, with a tray chosen, **scan the bag's QR code**: the bag decides the line.
3. The program shows the first part it found: **Yes, a part**.

> **The camera checks the size of a part, never its value.** A 10 k and a 1 k resistor look the same. The line you
> choose (or the bag you scan) is the truth. **Empty a tray before you load another line into it.**

### The load sheet

When a board has more lines than there are trays, the job is split into **loads**: "Load 1 of 4: tray 3 = 100 nF
0805, tray 4 = 10 k 0603, …". The Feeders tab shows each load. A line with more parts than one strip holds gets a
second tray when one is free.

### Refill

When a tray is empty during a job, the job pauses: lay a new strip **of the same line** and press **Refilled**, then
Resume.

---

## 12. Run a job

**Job** tab.

1. **Plan**: how many parts the machine places, in how many loads, about how long, and a list of every part it
   does **not** place with the reason (needs the bottom camera, by hand, a polar part whose angle in the tape is not
   known yet, out of reach, already placed, no tray wide enough…).
2. Load the trays of **load 1** (Feeders tab) and confirm each first part.
3. Choose the **mode**:
   - **Pick and place**: the real job.
   - **Dry run (camera only)**: the camera visits each pick and each place; nothing goes down. Do this first with a
     new board.
   - **Pick test**: N picks, each part put back; it reports the pick success rate with a photo of every miss.
4. Press **Start**.

While it runs:

- Each part goes **planned → picking → picked → placing → placed → verified** (or **failed** with a reason, or
  **skipped**). The camera checks the pocket is empty after each pick, and measures where each part landed.
- **Pause reasons** and what to do:

| The job says | Do |
|---|---|
| Load 2 of 4: put these strips in… | load those trays (Feeders tab), confirm the first parts, **Resume** |
| No part of … left: refill tray 4 | lay a new strip of the same line, **Refilled**, **Resume** |
| The next parts need nozzle N18G | lift, change the nozzle, **Nozzle changed**, **Resume** (a new nozzle needs its offset first: Setup step 5) |
| A part may be on the nozzle | look: **The part is on the nozzle: place it**, **The nozzle is empty: pick again**, or **Drop it over the tape** |
| HOME is … mm from where it should be | the machine lost steps: **Home** again, then **Resume**; check the parts placed since the last check |
| Stale: … | a calibration is out of date: redo that Setup step (or tick "start with stale calibration": it is written in the report) |

- **STOP** at any time. **Abort** ends a paused job (its report stays).
- **After a crash or a power cut**, start the program again: the job comes back **paused** and asks what is on the
  nozzle.

**The report:** the link **Open the report** shows every part's state, how far off it landed, the photos (before and
after each pick and place), the HOME checks and the parts left for your hands. Old jobs keep their reports; their
photos are deleted after a number of jobs (a setting).

---

## 13. Finish the board by hand (the Hand tab)

The **Hand** tab lists what is left for the soldering iron, in the order **Top SMD (1) → Bottom SMD (2) → THT by hand
(3)**:

- Every **line** (value + footprint) with its **drawer** and the parts it goes to.
- Click a line (or press ↑ ↓): its places light up on the board, pin 1 in red.
- **Enter** = this line is placed, go to the next. **Space** = placed / not placed (only while the machine is idle;
  otherwise Space is STOP). **F** = flip the view (top / bottom). **S** = scan a bag. **D** = the drawers.
- Parts the machine placed show as done. **The machine never places a part you marked placed.** The placed state is
  shared by every browser on the network.
- **Scan a bag** (camera QR, a USB / Bluetooth scanner anywhere on the page, or type the LCSC / part number): it says
  **"Tray 4"** (a machine line) or **"Drawer 7, by hand"**, and how many the board needs. An unknown code can be
  linked to the selected line with **Link**.
- **Drawers**: choose how many drawers you have; the lines are spread over them (kinds not mixed); move any line to
  another drawer. **Print drawer labels** / **Print part labels** on the PT-D410 as **one strip with cut marks** (or
  separate labels), or as a PDF.
- **Import old progress**: once, the placed marks, drawer moves and links of the old Solder Helper page (open the
  page in the browser you used it in).

---

## 14. Every day: start, stop, after a power cut

**Start of a session:**

1. Power the machine, start the program (or it starts as a service), open the page.
2. **Connect** (if it did not connect by itself).
3. If the controller was not reset since last time, Home and Z are kept. Otherwise the banner asks for **Home** and
   **Set Z** (step 4) again: nothing else needs measuring again.
4. Lights on (if switched).

**End of a session:** lift the nozzle, pump off. Closing the program keeps the controller's position as long as the
controller has power.

**After a power cut, a USB unplug or a reset:** the machine is **locked**. Lift the nozzle with Z up (by hand if the
power is off), press **Nozzle is clear**, **Home**, **Set Z**. A job comes back paused.

**After changing hardware:**

| You changed | Then |
|---|---|
| The camera, its lens or its mount | change `cameras.head.hardware_id` in the profile: the calibrations made with the old one show **stale**; redo step 3 and what follows |
| The nozzle | redo step 5 (and 5b) for it |
| A tray's place, or added a tray | step 8 (Survey) |
| HOME label moved | step 2 (travel) and step 8 |

---

## 15. Back up, copy to another machine, get help

- **Settings → Download the bundle** (or `pnp export backup.zip`): one zip with the profile, the bed, the trays, the
  calibrations, the boards, the parts store and the placed state. It never contains the controller's position.
- **Import a bundle** (or `pnp restore backup.zip`): what was there is saved in a zip first. Restart the program after.
- For help, send the bundle and the **event log** (Settings → "the event log").
- `pnp doctor` lists every check with what to do.

---

## 16. Upgrades: more hardware later

Every upgrade is **a change in the profile plus one Setup step**:

| Upgrade | What changes |
|---|---|
| XY limit switches | homing by `$H`; step 2 becomes "Homing switches" |
| Z probe | step 4: "Probe Z" instead of touching off by eye |
| A stepper for the nozzle turn | step 6 measures the axis |
| A bottom camera | step 5b; parts are looked at from below; SOIC-type parts become machine parts |
| A switched head light | the exposure is kept for light on and light off |
| A vacuum valve and sensor | step 6b; the pick check asks the sensor first |
| More nozzles | each package names its nozzle; the job pauses for each change |
| More trays, matrix trays, sticks | "Add tray", then Survey |
| A Pico with grblHAL | `controller.type: grblhal` |

Details and what is still TBD: [docs/builder/upgrades.md](../builder/upgrades.md).

---

## 17. Troubleshooting

| You see | Likely cause | Do |
|---|---|---|
| "Not connected" | the controller's USB, its power, or another program holds the port | check the cable and power; close the other program; Connect |
| Locked: "The controller was reset" | a power dip, a USB unplug, a reset | lift Z, Nozzle is clear, Home, Set Z |
| Locked: "A move did not happen as commanded" | the head hit an end or a motor lost steps | lift Z, Nozzle is clear, Home; check belts, couplers, driver current; slow down |
| "HOME is not in the picture" | the head is too far from the HOME label | jog until HOME is in view, then Home |
| "HOME measures … mm, not 10 mm" | the camera calibration does not fit (the camera or Z moved) | redo step 3 |
| Home works but places are off | stale nozzle offset, wrong board side / top edge | check step 5; check the drawn pads sit on the real pads |
| "No part found in the tape mark" | the strip is not in its tray, holes on the wrong side, film not peeled | reload the strip, Look at the slot again |
| Every pick "missed" | pick height wrong (tray height), the tip not on the parts | check step 7's numbers and the tray's height; redo step 5 |
| "Pick check unclear" | a hand in the picture, the light changed | look at the nozzle; choose place / pick again / drop |
| Picture too dark or too bright | the light changed | Machine tab: Find and lock the exposure |
| A part lands at a wrong angle | servo not repeatable | redo step 6; prefer parts that need no turn; consider a stepper or a bottom camera |
| The page shows old buttons | the browser kept old files | reload the page (the program changes the file names when they change) |
| "Stale: …" | something a result was built on changed | redo the step the message names |

The **Logs** tab shows what happened just before; the **Doctor** tab shows what is wrong now.

---

## 18. Keyboard shortcuts

Keys do nothing while you type in a box. **?** shows the list in the page.

| Key | What it does |
|---|---|
| **Space** | STOP (on the Hand tab while the machine is idle: placed / not placed) |
| ← → ↑ ↓ | jog X / Y (↑ = to the back) |
| Page Up / Page Down | jog Z up / down |
| [ ] | smaller / bigger X / Y step |
| { } | smaller / bigger Z step |
| h | Home |
| p | pump on / off |
| 1 … 9 | the tabs (on the Hand tab: 1 / 2 / 3 = the passes) |
| Hand tab: Enter, ↑ ↓, F, S, D | placed + next, the line, flip, scan, drawers |
| ? | the key list |

---

## 19. Words used in this guide

| Word | Meaning |
|---|---|
| **Board** | the printed circuit board (PCB) the parts go on |
| **Line** | one kind of part: a value + a package (for example 100 nF 0805) |
| **Package** | the part's body type and size (0805, SOT-23, SOIC-8, …) |
| **Tray / slot** | a place on the bed that holds one strip (or a grid of pockets, or a stick) |
| **Strip** | a cut piece of the paper or plastic tape the parts come in |
| **HOME** | the marker label all positions are measured from (mm from HOME, +X right, +Y to the back) |
| **Travel** | how far the head may move; nothing goes outside it |
| **Z reference** | Z = 0: the height the nozzle tip touched (the label surface) |
| **Nozzle offset** | where the nozzle tip is, seen from the camera's centre |
| **Registration** | measuring where the board lies on the bed (its holes and corners) |
| **Load** | the set of lines that fit in the trays at one time |
| **Profile** | the one file that describes your machine |
| **TBD** | "to be decided": a value not known yet; what needs it is switched off |
| **Valid / stale / missing** | measured and true / out of date / not done |
| **Simulator** | the virtual machine inside the program, for learning and testing |
