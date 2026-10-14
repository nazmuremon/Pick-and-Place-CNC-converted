"""Makes solder_helper_drawers.json: the drawers that today's Solder Helper page (../solder_helper/index.html, its
board_data.js from pcbnew) gives for the HT5023, for 10 and 25 drawers. A line-by-line Python copy of the page's
JavaScript (parts filter, lineKey = value + "\\0" + footprint, the line order, the drawer DP). Run once; the v2 test
checks that the program, from its own KiCad import, gives the same drawers (docs/13 §4.15).

    .venv/bin/python tests/data/boards/make_solder_helper_drawers.py ../solder_helper/board_data.js
"""
import json
import re
import sys
from pathlib import Path

src = Path(sys.argv[1]).read_text()
D = json.loads(src[src.index("{"): src.rindex("}") + 1])
parts = [dict(p, th=any(q.get("th") for q in p["pads"])) for p in D["parts"] if p["pads"] and not p["nobom"] and not p["dnp"]]


def cat_of(ref):
    pre = re.sub(r"\d+$", "", ref).upper()
    for k, names in (("R", ("R",)), ("C", ("C",)), ("L", ("L", "F", "FB")), ("D", ("D", "LED", "IR")), ("Q", ("Q",)),
                     ("X", ("X", "Y")), ("U", ("U", "IC")), ("J", ("J", "CN", "P"))):
        if pre in names:
            return k
    return "M"


def value_num(v):
    m = re.match(r"^([\d.]+)\s*([pnuµmkKMR]?)", v or "")
    if not m:
        return 0
    return float(m.group(1)) * {"p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6, "m": 1e-3, "k": 1e3, "K": 1e3, "M": 1e6,
                                "R": 1}.get(m.group(2), 1)


def nat(s):
    return [(0, int(t), "") if t.isdigit() else (1, 0, t.lower()) for t in re.findall(r"\d+|\D+", s or "")]


CATS = ["R", "C", "L", "D", "Q", "X", "U", "J", "M"]
lines = {}
for p in parts:
    k = p["val"] + "\0" + p["fp"]
    lines.setdefault(k, {"key": k, "val": p["val"], "fp": p["fp"], "parts": []})["parts"].append(p)
ls = list(lines.values())
for ln in ls:
    ln["parts"].sort(key=lambda p: nat(p["ref"]))
    ln["cat"] = cat_of(ln["parts"][0]["ref"])
    ln["th"] = any(p["th"] for p in ln["parts"])
ls.sort(key=lambda ln: (CATS.index(ln["cat"]), ln["th"], value_num(ln["val"]), nat(ln["val"]), nat(ln["fp"])))


def assign(count):
    n = len(ls)
    K = max(1, min(count, n))
    target = n / K
    cost = lambda i, j: (j - i - target) ** 2 + 12 * (len({ls[t]["cat"] for t in range(i, j)}) - 1)  # noqa: E731
    INF = float("inf")
    dp = [[INF] * (n + 1) for _ in range(K + 1)]
    cut = [[0] * (n + 1) for _ in range(K + 1)]
    dp[0][0] = 0
    for k in range(1, K + 1):
        for j in range(k, n + 1):
            for i in range(k - 1, j):
                v = dp[k - 1][i] + cost(i, j)
                if v < dp[k][j]:
                    dp[k][j], cut[k][j] = v, i
    out, j = {}, n
    for k in range(K, 0, -1):
        i = cut[k][j]
        for t in range(i, j):
            out[ls[t]["key"]] = k
        j = i
    return out


res = {"source": "solder_helper/board_data.js (pcbnew), the page's algorithm", "lines": [ln["key"] for ln in ls],
       "drawers": {str(c): assign(c) for c in (10, 25)}}
Path(__file__).with_name("ht5023").joinpath("solder_helper_drawers.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
print(len(ls), "lines")
