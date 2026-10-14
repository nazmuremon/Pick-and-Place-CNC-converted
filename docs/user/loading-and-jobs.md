# Loading trays and running a job

## Trays (Feeders tab)
- The list shows every tray, what it holds, and whether the NOZZLE reaches it ("camera only" = it can be seen but not
  picked from).
- **Load a tray:** cut a strip, peel the cover film, lay it in the tray (holes to the side the number's top points to,
  parts on the centre line). Choose the tray and the line, "Strip is in: look at it", then say Yes when the picture
  shows a part. Or scan the bag's QR code with a tray chosen: the bag decides the line.
- The camera checks the size of a part, never its value: the line you choose (or the bag you scan) is the truth.
  Empty a tray before you load another line into it.
- **Matrix tray or stick:** "Add tray", kind "matrix tray" (rows, columns, pitch, the first pocket's distance from
  the marker) or "stick" (the pick point's distance). Measure where the parts sit and the tray's top (where its marker
  is) above the label surface and type them: the program never guesses heights.
- **Add a tray:** number, tape width, kind, where you will put it. The program makes the STL (or the paper label PDF):
  print it, put it on the bed, then Setup step 8 (Survey).

## The job (Job tab)
- **Plan** says what the machine places, in how many loads, and why every other part is not placed by it (bottom
  camera, by hand, polar parts whose angle in the tape is not known yet, out of reach, already placed).
- **Modes:** "Pick and place", "Dry run (camera only)", "Pick test" (N picks, each part put back: the pick success
  rate, with a photo of every miss).
- **Start.** When a load is done, the job pauses: "Load 2 of 4: ..." - load those trays, Resume.
- **A tray is empty:** the job pauses; refill it with the same line, "Refilled", Resume.
- **STOP (Space):** the nozzle lifts. If a part is on the nozzle, the pump stays on until you choose: "place it",
  "the nozzle is empty: pick again", or "drop it over the tape".
- **A crash or a power cut:** start the program again; the job comes back paused and asks the same question.
- **Another nozzle:** when the next parts need it, the job pauses: "put on nozzle N18G". Change it, press
  "Nozzle changed". A nozzle used for the first time needs its offset (Setup, step 5).
- **A panel (several boards):** register each copy (Board tab: copy 2, its centre); the job places the parts on
  every copy.
- The **report** (link on the Job tab) has every part's state, its error, and the photos.

## By hand (Hand tab)
What the machine did not place, in the order Top SMD (1), Bottom SMD (2), THT (3), each line with its drawer. Enter =
the line is placed, next. F flips the view, S scans a bag, D opens the drawers (labels on the PT-D410 as one strip).
Parts the machine placed show as done; the machine never places a part you placed. Your old Solder Helper progress:
"Import old progress" in the browser you used it in.
