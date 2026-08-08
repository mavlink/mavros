#!/usr/bin/env python3
"""Convert per-package CHANGELOG.rst files to markdown under docs/changelog/.

The RST changelogs use a simple structure (a title, then "version (date)"
headers with a "---" underline, then "* item" bullets). mkdocs renders
Markdown, so we convert them. RST inline links "text <url>" become Markdown
links "text (url)".
"""

from __future__ import annotations

import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
PKGS = ["mavros", "mavros_extras", "libmavconn", "mavros_msgs", "test_mavros"]
OUT_DIR = REPO / "docs" / "changelog"

LINK_RE = re.compile(r"([^<\s]+)\s+<((?:https?|standardese)://[^>]+)>")


def convert(text: str, pkg: str) -> str:
    lines = text.splitlines()
    out: list[str] = []
    in_title = True
    for line in lines:
        stripped = line.strip()
        # RST title: "Changelog for package X" with a '^' underline.
        if "Changelog for package" in stripped:
            out.append(f"# {stripped}")
            in_title = False
            continue
        if in_title and stripped.startswith("^"):
            continue
        # Version header: "2.14.0 (2025-12-23)" followed by a "---" underline.
        if re.match(r"^\d+\.\d+\.\d+ \(\d{4}-\d{2}-\d{2}\)$", stripped.strip()):
            out.append("")
            out.append(f"## {stripped}")
            out.append("")
            continue
        if set(stripped) == {"-"}:
            continue
        if stripped.startswith("^"):
            continue
        # Bullet items.
        if stripped.startswith("* "):
            item = stripped[2:]
            out.append(f"- {item}")
            continue
        # Wrap RST links into Markdown links.
        converted = LINK_RE.sub(r"[\1](\2)", line)
        out.append(converted)
    return "\n".join(out).strip() + "\n"


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for pkg in PKGS:
        src = REPO / pkg / "CHANGELOG.rst"
        if not src.exists():
            print(f"skip {pkg}: no CHANGELOG.rst", file=sys.stderr)
            continue
        md = convert(src.read_text(encoding="utf-8"), pkg)
        (OUT_DIR / f"{pkg}.md").write_text(md, encoding="utf-8")
        print(f"wrote docs/changelog/{pkg}.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())