# Real pictures from the bench (fixtures for tests/v2/test_v2_vision.py)

Raw grey frames of the Xitech head camera (640 x 400, tilted ~25°), copied from `logs/pnp/` (2026-10-11).
No camera calibration exists for these sessions: the tests work in pixels, with what was seen on the bench.

| Folder | Pictures | What the tests expect |
|---|---|---|
| `pick/` | the pocket before and after a real pick, the part at the picture centre (320, 200) | picked: C23, C5, C9, C50 (16:45); missed: C48 (16:45), C32 (16:04); unclear (a hand in view): C24 (14:49), C6 (16:42) — from the job reports |
| `strip/` | the start of run 16:45 (HOME, slot 4 and the slot-3 strip in view) | with a template from the 3rd pocket: 9 parts. `pick/..._C9_before`: 2 empty pockets then 7 parts; `_after`: 3 empty then 6 |
| `holes/` | the HT5023's four corners with their mounting holes (B side up) | corners H1, H3, H4 found (H2's edge runs along a bar's edge: refused is fine); holes H2, H3, H4 found, H1 refused or found |

Add more as the bench makes them (black tape, clear tape, fiducials, ink marks).
