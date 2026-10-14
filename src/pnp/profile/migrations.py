"""Profile migrations: an old file is brought up to the current schema_version before the check.

Add one function per version step: MIGRATIONS[n] turns a version-n dict into version n + 1.
"""
from __future__ import annotations

from typing import Callable

from .schema import SCHEMA_VERSION

MIGRATIONS: dict[int, Callable[[dict], dict]] = {}


def migrate(raw: dict) -> dict:
    v = raw.get("schema_version", SCHEMA_VERSION)
    if not isinstance(v, int) or v < 1:
        return raw                                   # the schema check says what is wrong
    if v > SCHEMA_VERSION:
        raise ValueError(f"profile schema_version {v} is newer than this program ({SCHEMA_VERSION}): update it")
    while v < SCHEMA_VERSION:
        raw = MIGRATIONS[v](dict(raw))
        v += 1
        raw["schema_version"] = v
    return raw
