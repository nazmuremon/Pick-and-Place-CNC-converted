"""Vision (Phase C, docs/13 §4.5): camera calibration per plane (ChArUco), exposure lock and the light check,
markers and the bed survey, parts in a pocket (model-based), holes / fiducials / board corners, ink marks.

Takes over, as pure functions with their tests and fixtures: tools/bed_test_run.py Markers and
moved_as_commanded, tools/dry_run_bench.py find_slot_parts / dark_spots / fit_circle / hole_candidates,
tools/nozzle_calibrate.py find_mark / offset_for, the corner edge fit (§5.9).
"""
