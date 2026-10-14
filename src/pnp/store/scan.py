"""A bag's code -> its line (docs/13 §4.15): the LCSC bag QR code ({pbn:...,pc:C8598,pm:...,qty:100,...}), a plain
LCSC number (C8598) or a manufacturer part number. Codes the board does not name can be linked to a line by hand;
the links are kept in the store."""
from __future__ import annotations

import re

from .lines import StoreLine, norm_mpn


def parse_code(text: str) -> dict | None:
    t = (text or "").strip()

    def get(k):
        m = re.search(r"(?:^|[{,])\s*" + k + r"\s*:\s*([^,}]*)", t, re.I)
        return m.group(1).strip() if m else ""
    if re.search(r"\bpc\s*:", t, re.I) or re.search(r"\bpbn\s*:", t, re.I):
        pc, pm = get("pc").upper(), get("pm")
        q = get("qty")
        return {"pc": pc, "pm": pm, "qty": int(q) if q.isdigit() else None} if pc or pm else None
    if re.fullmatch(r"C\d{3,9}", t, re.I):
        return {"pc": t.upper(), "pm": "", "qty": None}
    if t:
        return {"pc": "", "pm": t, "qty": None}                # typed: a part number
    return None


def find_line(code: dict, lines: list[StoreLine], links: dict[str, str]) -> StoreLine | None:
    by_key = {ln.key: ln for ln in lines}
    pc = code.get("pc") or ""
    if pc:
        for ln in lines:
            if pc in ln.lcsc:
                return ln
        if pc in links and links[pc] in by_key:
            return by_key[links[pc]]
    pm = norm_mpn(code.get("pm", ""))
    if pm:
        for ln in lines:
            if pm in ln.mpn:
                return ln
        if f"MPN:{pm}" in links and links[f"MPN:{pm}"] in by_key:
            return by_key[links[f"MPN:{pm}"]]
    return None


def link_key(code: dict) -> str | None:
    if code.get("pc"):
        return code["pc"]
    pm = norm_mpn(code.get("pm", ""))
    return f"MPN:{pm}" if pm else None
