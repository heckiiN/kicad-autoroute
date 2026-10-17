"""Minimal Specctra DSN reader/writer.

Parses a DSN file into nested lists of Atom/list nodes and writes it back.
Handles KiCad's `(string_quote ")` declaration, quoted names and comments-free files.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Atom:
    text: str
    quoted: bool = False

    def __str__(self) -> str:
        return self.text


Node = "Atom | list"


def _tokenize(src: str):
    i, n = 0, len(src)
    quote = '"'
    prev_was_string_quote = False
    while i < n:
        c = src[i]
        if c.isspace():
            i += 1
            continue
        if c in "()":
            yield c
            i += 1
            prev_was_string_quote = False
            continue
        if prev_was_string_quote:
            # The token after `string_quote` is the quote character itself.
            quote = c
            yield Atom(c)
            i += 1
            prev_was_string_quote = False
            continue
        if c == quote:
            j = src.find(quote, i + 1)
            if j < 0:
                raise ValueError(f"unterminated quoted string at offset {i}")
            yield Atom(src[i + 1 : j], quoted=True)
            i = j + 1
            continue
        j = i
        while j < n and not src[j].isspace() and src[j] not in "()":
            j += 1
        text = src[i:j]
        yield Atom(text)
        prev_was_string_quote = text == "string_quote"
        i = j


def parse(src: str) -> list:
    stack: list[list] = [[]]
    for tok in _tokenize(src):
        if tok == "(":
            stack.append([])
        elif tok == ")":
            node = stack.pop()
            stack[-1].append(node)
        else:
            stack[-1].append(tok)
    if len(stack) != 1 or len(stack[0]) != 1:
        raise ValueError("unbalanced parentheses in DSN")
    return stack[0][0]


def read(path: str | Path) -> list:
    # latin-1 maps every byte, so unknown encodings round-trip byte for byte.
    return parse(Path(path).read_text(encoding="latin-1"))


def _needs_quotes(text: str) -> bool:
    return text == "" or any(ch.isspace() or ch in '()"' for ch in text)


def dumps(node, indent: int = 0) -> str:
    if isinstance(node, Atom):
        if node.text == '"' and not node.quoted:
            return node.text  # the string_quote declaration itself
        if node.quoted or _needs_quotes(node.text):
            return f'"{node.text}"'
        return node.text
    inner_atoms = all(isinstance(x, Atom) for x in node)
    if inner_atoms:
        return "(" + " ".join(dumps(x) for x in node) + ")"
    pad = "  " * (indent + 1)
    head = []
    rest = list(node)
    while rest and isinstance(rest[0], Atom):
        head.append(dumps(rest.pop(0)))
    body = "\n".join(pad + dumps(x, indent + 1) for x in rest)
    return "(" + " ".join(head) + "\n" + body + "\n" + "  " * indent + ")"


def write(tree, path: str | Path) -> None:
    Path(path).write_text(dumps(tree) + "\n", encoding="utf-8")


# ---- tree helpers -------------------------------------------------------------------------


def head(node) -> str | None:
    if isinstance(node, list) and node and isinstance(node[0], Atom):
        return node[0].text
    return None


def children(node, name: str) -> list[list]:
    return [c for c in node if head(c) == name]


def child(node, name: str):
    found = children(node, name)
    return found[0] if found else None


def atoms(node) -> list[str]:
    return [x.text for x in node if isinstance(x, Atom)]
