"""A small S-expression reader for KiCad files (no `pcbnew` needed)."""
from __future__ import annotations

import re

TOKEN = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()"]+')


class Node(list):
    """(name arg arg (child ...) ...): node.name is the first word."""

    @property
    def name(self) -> str:
        return self[0] if self and isinstance(self[0], str) else ""

    def all(self, name: str) -> list[Node]:
        return [c for c in self[1:] if isinstance(c, Node) and c.name == name]

    def first(self, name: str) -> Node | None:
        return next((c for c in self[1:] if isinstance(c, Node) and c.name == name), None)

    def value(self, name: str, i: int = 1, default=None):
        n = self.first(name)
        return n[i] if n is not None and len(n) > i else default

    def floats(self, name: str) -> list[float]:
        n = self.first(name)
        return [] if n is None else [float(v) for v in n[1:] if isinstance(v, str) and _is_num(v)]

    def words(self) -> list[str]:
        return [v for v in self[1:] if isinstance(v, str)]


def _is_num(v: str) -> bool:
    try:
        float(v)
        return True
    except ValueError:
        return False


def parse(text: str) -> Node:
    stack: list[Node] = [Node()]
    for m in TOKEN.finditer(text):
        t = m.group(0)
        if t == "(":
            n = Node()
            stack[-1].append(n)
            stack.append(n)
        elif t == ")":
            if len(stack) == 1:
                raise ValueError(f"unbalanced ')' at character {m.start()}")
            stack.pop()
        elif t[0] == '"':
            stack[-1].append(t[1:-1].replace('\\"', '"').replace("\\\\", "\\"))
        else:
            stack[-1].append(t)
    if len(stack) != 1:
        raise ValueError("the file ends inside an open '('")
    root = stack[0]
    if len(root) != 1 or not isinstance(root[0], Node):
        raise ValueError("not one S-expression")
    return root[0]
