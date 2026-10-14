"""Drawers (docs/13 §4.15): every line lives in exactly one drawer. Auto-assign splits the sorted lines into N
drawers of even size without mixing kinds (the same small dynamic program as Solder Helper); the operator's moves
override it and are kept."""
from __future__ import annotations


def auto_assign(lines, count: int) -> dict[str, int]:
    """lines: in their order (store/lines.py), each with .key and .cat. Returns key -> drawer (1..count)."""
    n = len(lines)
    if n == 0:
        return {}
    K = max(1, min(int(count), n))
    target = n / K
    INF = float("inf")

    def cost(i: int, j: int) -> float:
        return (j - i - target) ** 2 + 12 * (len({lines[t].cat for t in range(i, j)}) - 1)
    dp = [[INF] * (n + 1) for _ in range(K + 1)]
    cut = [[0] * (n + 1) for _ in range(K + 1)]
    dp[0][0] = 0.0
    for k in range(1, K + 1):
        for j in range(k, n + 1):
            for i in range(k - 1, j):
                v = dp[k - 1][i] + cost(i, j)
                if v < dp[k][j]:
                    dp[k][j], cut[k][j] = v, i
    out: dict[str, int] = {}
    j = n
    for k in range(K, 0, -1):
        i = cut[k][j]
        for t in range(i, j):
            out[lines[t].key] = k
        j = i
    return out
