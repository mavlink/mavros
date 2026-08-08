#!/usr/bin/env python3
"""Convert per-package CHANGELOG.rst files to markdown under docs/changelog/.

Requires the system `pandoc` binary (e.g. `apt install pandoc`) for robust
RST -> markdown conversion.
"""

from __future__ import annotations

import pathlib
import shutil
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
PKGS = ["mavros", "mavros_extras", "libmavconn", "mavros_msgs", "test_mavros", "mavros_examples"]
OUT_DIR = REPO / "docs" / "changelog"


def convert(src: pathlib.Path) -> str:
    out = subprocess.run(
        ["pandoc", "-f", "rst", "-t", "gfm", str(src)],
        check=True,
        capture_output=True,
        text=True,
    )
    return out.stdout


def main() -> int:
    if shutil.which("pandoc") is None:
        print("error: `pandoc` not found. Install it (e.g. `apt install pandoc`).", file=sys.stderr)
        return 1
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for pkg in PKGS:
        src = REPO / pkg / "CHANGELOG.rst"
        if not src.exists():
            print(f"skip {pkg}: no CHANGELOG.rst", file=sys.stderr)
            continue
        (OUT_DIR / f"{pkg}.md").write_text(convert(src), encoding="utf-8")
        print(f"wrote docs/changelog/{pkg}.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())