#!/usr/bin/env python3
"""3D printed SMD strip holders: the command line. The generator is src/pnp/feeders/traygen.py (the page's
"Add tray" uses it too); see its help:  .venv/bin/python hardware/tray/smd_tray.py --help
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from pnp.feeders.traygen import *  # noqa: E402,F401,F403  (pcb_corner.py and the tests use the names)
from pnp.feeders.traygen import main  # noqa: E402

if __name__ == "__main__":
    main()
