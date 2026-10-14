# What every lock means

The banner at the top says one thing and what to do. The machine has one state:

| Banner | Why | What to do |
|---|---|---|
| Not connected | no link to the controller | Connect (Machine tab) |
| The controller was reset | power dip, USB, or a reset: the position is lost | Lift the nozzle (Z up), "Nozzle is clear", Home, set Z |
| The controller is in ALARM | the controller stopped (an end, a limit, an error) | Look at the machine, "Unlock alarm", lift, "Nozzle is clear", Home, set Z |
| A move did not happen as commanded | the camera saw the head not move (an end, lost steps) | Lift, "Nozzle is clear", Home |
| The controller stopped answering | the cable or the controller's power | Check them, Connect |
| E-stop | the E-stop button | Release it, "Unlock alarm", lift, "Nozzle is clear", Home, set Z |
| The last session did not end cleanly | the program stopped with the nozzle maybe down | Lift, "Nozzle is clear", Home |
| Not homed: slow jogs only | the position is not known yet | Jog until HOME is in view, Home |
| Homed. Z has no reference yet | the touch-off is not done | Setup step 4 |
