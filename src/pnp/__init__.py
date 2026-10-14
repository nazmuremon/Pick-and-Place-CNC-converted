"""Pick and place program v2 (docs/13-build-prompt-v2.md).

Packages:
  profile/       the machine profile: schema, loading, migrations, reload
  capabilities/  small interfaces (homing, Z reference, rotation, vacuum, light, cameras, ...) and their
                 real / none / sim versions, chosen by the profile (docs/13 §4.14)
  motion/        the controller driver (Grbl, grblHAL), axis map, travel limits, the lock state
  sim/           the simulator: a virtual controller and a virtual camera
  ui/            the web page (FastAPI)
  vision/, board/, feeders/, store/, calibration/, job/: later phases (docs/13 §11)
"""
__version__ = "2.0.0a1"
