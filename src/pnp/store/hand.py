"""The hand list (docs/13 §4.15): after the machine job, what is left for the soldering iron, in the order
Top SMD -> Bottom SMD -> THT (docs/11), each line with its drawer and the reason it is by hand (the machine
skipped or failed it, by-hand package, not in reach, THT). The placed state is the one the machine writes."""
from __future__ import annotations

from .lines import StoreLine, in_bom, nat_key

PASSES = {"F": "Top SMD", "B": "Bottom SMD", "T": "THT by hand"}


def pass_of(c) -> str:
    return "T" if any(p.through_hole for p in c.pads) else c.side


def passes(board, lines: list[StoreLine], drawers: dict[str, int], placed: dict[str, dict],
           reasons: dict[str, str]) -> dict:
    """{pass: {name, total, done, groups: [{key, value, footprint, drawer, refs: [{ref, placed, by, why}]}]}}"""
    comp = {c.ref: c for c in board.components if in_bom(c)}
    out = {}
    for ps, name in PASSES.items():
        groups = []
        for ln in lines:
            refs = [r for r in ln.refs if r in comp and pass_of(comp[r]) == ps]
            if not refs:
                continue
            groups.append({"key": ln.key, "value": ln.value, "footprint": ln.footprint, "drawer": drawers.get(ln.key),
                           "feeder_key": ln.feeder_key,
                           "refs": [{"ref": r, "placed": r in placed, "by": placed.get(r, {}).get("by"),
                                     "why": reasons.get(r, "")} for r in sorted(refs, key=nat_key)]})
        total = sum(len(g["refs"]) for g in groups)
        done = sum(1 for g in groups for r in g["refs"] if r["placed"])
        out[ps] = {"name": name, "total": total, "done": done, "groups": groups}
    return out
