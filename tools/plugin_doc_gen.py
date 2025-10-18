"""
This script parses C++ files to extract ROS API description,
and then generate markdown files for mkdocs.
"""

import os
import subprocess
import sys
import typing as ty
from functools import cache

from clang.cindex import CursorKind, Index, TranslationUnit

from mavros_cog import PluginInfo, load_all_plugin_infos


def find_default_cpp_include_paths() -> ty.Iterable[str]:
    env = os.environ.copy()
    env.update(
        {
            "LC_ALL": "C",
            "LANG": "C",
        }
    )

    proc = subprocess.run(
        ["g++", "-x", "c++", "--std", "c++20", "-v", "-E", "/dev/null"],
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )

    incl_found = False
    for line in proc.stderr.splitlines():
        if line == "#include <...> search starts here:":
            incl_found = True
            continue
        if line == "End of search list.":
            return
        if incl_found:
            yield line.strip()


@cache
def cpp_include_paths() -> list[str]:
    return list(find_default_cpp_include_paths())


def parse_file(path: str) -> TranslationUnit:
    idx = Index.create()

    args = [
        "-x",
        "c++",
        "--std",
        "c++20",
        "-v",
    ]

    for include in cpp_include_paths():
        args += ["-I", include]

    print(args)

    tu = idx.parse(path, args=args)

    return tu


def main():

    tu = parse_file(sys.argv[1])

    for cursor in tu.cursor.walk_preorder():
        # Check if the cursor represents a C++ class declaration
        if cursor.kind == CursorKind.CLASS_DECL:
            print(f"Class found: {cursor.spelling}")
            # Iterate through the children of the class cursor to find methods
            for child in cursor.get_children():
                if child.kind == CursorKind.CXX_METHOD:
                    print(f"  Method: {child.spelling}")

if __name__=='__main__':
    main()
