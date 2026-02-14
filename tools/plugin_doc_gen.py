"""Extract ROS API docs from MAVROS plugin source files.

Primary mode uses real libclang AST traversal. If AST parsing is incomplete in
the local environment (e.g. missing ROS headers), a regex fallback can be used
to keep early documentation generation useful.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import pathlib
import re
import subprocess
import sys
import typing as ty
from functools import cache

try:
    from clang.cindex import CompilationDatabase, Cursor, CursorKind, Index, TranslationUnit
except Exception:
    CompilationDatabase = ty.Any  # type: ignore[misc,assignment]
    Cursor = ty.Any  # type: ignore[misc,assignment]
    CursorKind = ty.Any  # type: ignore[misc,assignment]
    Index = ty.Any  # type: ignore[misc,assignment]
    TranslationUnit = ty.Any  # type: ignore[misc,assignment]
    CLANG_AVAILABLE = False
else:
    CLANG_AVAILABLE = True


DEFAULT_PLUGIN_DIRS = ("mavros/src/plugins", "mavros_extras/src/plugins")
WORKSPACE_INCLUDE_DIRS = ("mavros/include", "mavros_msgs/include", "mavros_extras/include")
SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

REGISTER_PLUGIN_RE = re.compile(r"MAVROS_PLUGIN_REGISTER\((?P<klass>[a-zA-Z0-9_:]+)\)")
PLUGIN_NAME_RE = re.compile(r"@plugin\s+(?P<name>[a-z0-9_]+)")
PLUGIN_BRIEF_RE = re.compile(r"@brief\s+(?P<brief>.+)")
PLUGIN_NS_RE = re.compile(r":\s*Plugin\s*\(\s*[^,]+,\s*\"(?P<ns>[^\"]+)\"\s*\)")

PUBLISHER_CALL = "create_publisher"
SUBSCRIPTION_CALL = "create_subscription"
SERVICE_CALL = "create_service"
CLIENT_CALL = "create_client"
PARAM_WATCH_CALL = "node_declare_and_watch_parameter"
PARAM_DECLARE_CALL = "declare_parameter"
MF_SUBSCRIBE_CALL = "subscribe"

TARGET_CALLS = {
    PUBLISHER_CALL,
    SUBSCRIPTION_CALL,
    SERVICE_CALL,
    CLIENT_CALL,
    PARAM_WATCH_CALL,
    PARAM_DECLARE_CALL,
    MF_SUBSCRIBE_CALL,
}


@dataclasses.dataclass(frozen=True)
class ApiEntry:
    name: str
    type_name: str
    line: int

    def key(self) -> tuple[str, str]:
        return (self.name, self.type_name)

    @property
    def rendered_type(self) -> str:
        return self.type_name or "<unknown>"


@dataclasses.dataclass(frozen=True)
class PluginApi:
    plugin: str
    path: pathlib.Path
    class_name: str
    namespace: str
    brief: str
    description: str
    publishers: list[ApiEntry]
    subscribers: list[ApiEntry]
    services: list[ApiEntry]
    clients: list[ApiEntry]
    parameters: list[ApiEntry]


def _clean_doxygen_comment(block: str) -> str:
    cleaned = []
    for raw in block.splitlines():
        line = raw.strip()
        if line.startswith("*"):
            line = line[1:].strip()
        if not line or line.startswith("@plugin") or line.startswith("@brief"):
            continue
        cleaned.append(line)
    return " ".join(cleaned).strip()


def _extract_plugin_meta(path: pathlib.Path, text: str) -> tuple[str, str, str, str, str]:
    blocks = re.findall(r"/\*\*(.*?)\*/", text, flags=re.DOTALL)
    plugin_block = next((b for b in blocks if "@plugin" in b), "")
    name_match = PLUGIN_NAME_RE.search(plugin_block)
    brief_match = PLUGIN_BRIEF_RE.search(plugin_block)
    klass_match = REGISTER_PLUGIN_RE.search(text)
    ns_match = PLUGIN_NS_RE.search(text)

    plugin = name_match.group("name") if name_match else path.stem
    brief = brief_match.group("brief").strip() if brief_match else ""
    description = _clean_doxygen_comment(plugin_block)
    class_name = klass_match.group("klass") if klass_match else ""
    namespace = ns_match.group("ns") if ns_match else ""
    return plugin, class_name, namespace, brief or description, description


def _dedup_entries(entries: list[ApiEntry]) -> list[ApiEntry]:
    seen: set[tuple[str, str]] = set()
    unique: list[ApiEntry] = []
    for entry in sorted(entries, key=lambda e: (e.line, e.name, e.type_name)):
        key = entry.key()
        if key in seen:
            continue
        seen.add(key)
        unique.append(entry)
    return unique


def _cursor_iter(root: Cursor) -> ty.Iterator[Cursor]:
    stack = [root]
    while stack:
        node = stack.pop()
        yield node
        try:
            children = list(node.get_children())
        except Exception:
            continue
        stack.extend(reversed(children))


def _node_in_file(node: Cursor, path: pathlib.Path) -> bool:
    try:
        if node.location.file is None:
            return False
        return pathlib.Path(node.location.file.name).resolve() == path.resolve()
    except Exception:
        return False


def _line_of(node: Cursor) -> int:
    try:
        return int(node.location.line)
    except Exception:
        return 0


def _call_name(cursor: Cursor) -> str:
    try:
        if cursor.spelling:
            return cursor.spelling
        if cursor.displayname:
            return cursor.displayname.split("(")[0].split("<")[0].strip()
    except Exception:
        pass

    try:
        tokens = [t.spelling for t in cursor.get_tokens()]
    except Exception:
        return ""

    for idx, token in enumerate(tokens):
        if token == "(" and idx > 0:
            return tokens[idx - 1].split("::")[-1]

    for candidate in TARGET_CALLS:
        if candidate in tokens:
            return candidate
    return ""


def _tokens(cursor: Cursor) -> list[str]:
    try:
        return [t.spelling for t in cursor.get_tokens()]
    except Exception:
        return []


def _first_call_argument(tokens: list[str]) -> str:
    depth = 0
    seen_open = False
    first_arg: list[str] = []

    for tok in tokens:
        if tok == "(":
            depth += 1
            seen_open = True
            continue
        if not seen_open:
            continue
        if tok == ")":
            depth -= 1
            if depth <= 0:
                break
        if depth == 1 and tok == ",":
            break
        if depth >= 1:
            first_arg.append(tok)

    raw = " ".join(first_arg).strip()
    m = re.search(r'"([^"]+)"', raw)
    return m.group(1) if m else raw


def _extract_template_arg(call_name: str, tokens: list[str]) -> str:
    text = " ".join(tokens)
    m = re.search(rf"{re.escape(call_name)}\s*<\s*(?P<type>[^>]+)\s*>", text)
    return m.group("type").strip() if m else ""


def _find_message_filter_subscribers(
    tu: TranslationUnit, source_path: pathlib.Path
) -> dict[str, str]:
    out: dict[str, str] = {}
    for node in _cursor_iter(tu.cursor):
        if not _node_in_file(node, source_path):
            continue
        if node.kind not in (CursorKind.FIELD_DECL, CursorKind.VAR_DECL):
            continue
        try:
            tname = node.type.spelling
        except Exception:
            continue
        m = re.search(r"message_filters::Subscriber<\s*(?P<type>[^>]+)\s*>", tname)
        if m and node.spelling:
            out[node.spelling] = m.group("type").strip()
    return out


def _extract_ast_api(
    tu: TranslationUnit, source_path: pathlib.Path, source_text: str
) -> tuple[list[ApiEntry], list[ApiEntry], list[ApiEntry], list[ApiEntry], list[ApiEntry]]:
    publishers: list[ApiEntry] = []
    subscribers: list[ApiEntry] = []
    services: list[ApiEntry] = []
    clients: list[ApiEntry] = []
    parameters: list[ApiEntry] = []

    mf_subs = _find_message_filter_subscribers(tu, source_path)
    for match in re.finditer(
        r"message_filters::Subscriber<(?P<type>[^>]+)>\s+(?P<var>[a-zA-Z_][a-zA-Z0-9_]*)\s*;",
        source_text,
    ):
        mf_subs.setdefault(match.group("var"), match.group("type").strip())

    for node in _cursor_iter(tu.cursor):
        if not _node_in_file(node, source_path):
            continue
        if node.kind not in (CursorKind.CALL_EXPR, CursorKind.UNEXPOSED_EXPR):
            continue

        call = _call_name(node)
        if call not in TARGET_CALLS:
            continue

        call_tokens = _tokens(node)
        line = _line_of(node)

        if call in (PUBLISHER_CALL, SUBSCRIPTION_CALL, SERVICE_CALL, CLIENT_CALL):
            name = _first_call_argument(call_tokens)
            type_name = _extract_template_arg(call, call_tokens)
            if not name:
                continue
            entry = ApiEntry(name=name, type_name=type_name, line=line)
            if call == PUBLISHER_CALL:
                publishers.append(entry)
            elif call == SUBSCRIPTION_CALL:
                subscribers.append(entry)
            elif call == SERVICE_CALL:
                services.append(entry)
            else:
                clients.append(entry)
            continue

        if call in (PARAM_WATCH_CALL, PARAM_DECLARE_CALL):
            name = _first_call_argument(call_tokens)
            if name:
                parameters.append(ApiEntry(name=name, type_name="", line=line))
            continue

        if call == MF_SUBSCRIBE_CALL:
            if "." not in call_tokens:
                continue
            try:
                dot_idx = call_tokens.index(".")
            except ValueError:
                continue
            if dot_idx < 1:
                continue
            var_name = call_tokens[dot_idx - 1]
            if var_name not in mf_subs:
                continue
            topic = _first_call_argument(call_tokens)
            if topic:
                subscribers.append(
                    ApiEntry(name=topic, type_name=mf_subs[var_name], line=line)
                )

    # Some libclang builds don't expose message_filters::Subscriber.subscribe(...)
    # as CALL_EXPR/UNEXPOSED_EXPR nodes. Recover those from source text while
    # still using AST-derived variable->type bindings.
    sub_re = re.compile(
        r"(?P<var>[a-zA-Z_][a-zA-Z0-9_]*)\s*\.\s*subscribe\s*\(\s*[^,]+,\s*(?P<topic>[^,\)]+)"
    )
    for match in sub_re.finditer(source_text):
        var_name = match.group("var")
        raw_topic = match.group("topic").strip()
        literal = re.search(r'"([^"]+)"', raw_topic)
        topic = literal.group(1) if literal else raw_topic
        subscribers.append(
            ApiEntry(
                name=topic,
                type_name=mf_subs.get(var_name, ""),
                line=source_text.count("\n", 0, match.start()) + 1,
            )
        )

    return (
        _dedup_entries(publishers),
        _dedup_entries(subscribers),
        _dedup_entries(services),
        _dedup_entries(clients),
        _dedup_entries(parameters),
    )


def _extract_api_regex_fallback(text: str) -> tuple[list[ApiEntry], ...]:
    def entries(pattern: re.Pattern[str], name_group: str, type_group: str) -> list[ApiEntry]:
        out: list[ApiEntry] = []
        for m in pattern.finditer(text):
            raw = m.group(name_group).strip()
            lit = re.search(r'"([^"]+)"', raw)
            name = lit.group(1) if lit else raw
            line = text.count("\n", 0, m.start()) + 1
            out.append(ApiEntry(name=name, type_name=m.group(type_group).strip(), line=line))
        return _dedup_entries(out)

    publishers = entries(
        re.compile(
            r"create_publisher<(?P<type>[^>]+)>\s*\(\s*(?P<name>[^,]+)\s*,",
            flags=re.DOTALL,
        ),
        "name",
        "type",
    )
    subscribers = entries(
        re.compile(
            r"create_subscription<(?P<type>[^>]+)>\s*\(\s*(?P<name>[^,]+)\s*,",
            flags=re.DOTALL,
        ),
        "name",
        "type",
    )
    services = entries(
        re.compile(
            r"create_service<(?P<type>[^>]+)>\s*\(\s*(?P<name>[^,]+)\s*,",
            flags=re.DOTALL,
        ),
        "name",
        "type",
    )
    clients = entries(
        re.compile(
            r"create_client<(?P<type>[^>]+)>\s*\(\s*(?P<name>[^,\)]+)\s*[\),]",
            flags=re.DOTALL,
        ),
        "name",
        "type",
    )

    params: list[ApiEntry] = []
    for pattern in (
        re.compile(r'node_declare_and_watch_parameter\s*\(\s*"(?P<name>[^"]+)"'),
        re.compile(r'node->declare_parameter(?:<[^>]+>)?\s*\(\s*"(?P<name>[^"]+)"'),
    ):
        for m in pattern.finditer(text):
            params.append(
                ApiEntry(
                    name=m.group("name"),
                    type_name="",
                    line=text.count("\n", 0, m.start()) + 1,
                )
            )

    return (
        _dedup_entries(publishers),
        _dedup_entries(subscribers),
        _dedup_entries(services),
        _dedup_entries(clients),
        _dedup_entries(params),
    )


def find_default_cpp_include_paths() -> ty.Iterable[str]:
    env = os.environ.copy()
    env.update({"LC_ALL": "C", "LANG": "C"})
    proc = subprocess.run(
        ["g++", "-x", "c++", "--std", "c++20", "-v", "-E", "/dev/null"],
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )

    in_list = False
    for line in proc.stderr.splitlines():
        line = line.strip()
        if line == "#include <...> search starts here:":
            in_list = True
            continue
        if line == "End of search list.":
            return
        if in_list and line:
            yield line


@cache
def default_cpp_include_paths() -> tuple[str, ...]:
    try:
        return tuple(find_default_cpp_include_paths())
    except Exception:
        return tuple()


def _default_clang_args(extra_clang_args: list[str]) -> list[str]:
    args = ["-x", "c++", "-std=c++20"]
    for inc in WORKSPACE_INCLUDE_DIRS:
        args += ["-I", str((REPO_ROOT / inc).resolve())]
    for inc in default_cpp_include_paths():
        args += ["-isystem", inc]
    args += extra_clang_args
    return args


def _compile_db_args(
    compile_commands: list[CompilationDatabase], source_path: pathlib.Path
) -> list[str] | None:
    if not compile_commands:
        return None

    for db in compile_commands:
        try:
            commands = db.getCompileCommands(str(source_path))
        except Exception:
            continue
        if not commands:
            continue

        argv = [a for a in commands[0].arguments]
        # Drop compiler executable, output options, and positional source files.
        out: list[str] = []
        skip_next = False
        expects_value = False
        value_flags = {
            "-o",
            "--output",
            "-I",
            "-isystem",
            "-include",
            "-isysroot",
            "--sysroot",
            "-imacros",
            "-iquote",
            "-MF",
            "-MT",
            "-MQ",
            "-Xclang",
            "-stdlib",
        }
        for arg in argv[1:]:
            if skip_next:
                skip_next = False
                continue
            if expects_value:
                out.append(arg)
                expects_value = False
                continue
            if arg in ("-c",):
                continue
            if arg in ("-o", "--output"):
                skip_next = True
                continue
            if arg in value_flags:
                out.append(arg)
                expects_value = True
                continue
            if not arg.startswith("-"):
                # Drop positional source inputs from compile command; `idx.parse()`
                # already receives the source path as the main TU input.
                if re.search(r"\.(c|cc|cpp|cxx|c\+\+|m|mm)$", arg):
                    continue
            if pathlib.Path(arg).resolve() == source_path.resolve():
                continue
            # Keep plugin-specific defines/includes from build system.
            out.append(arg)
        return out

    return None


def _parse_translation_unit(
    idx: Index,
    source_path: pathlib.Path,
    compile_commands: list[CompilationDatabase],
    extra_clang_args: list[str],
) -> TranslationUnit:
    if not CLANG_AVAILABLE:
        raise RuntimeError("clang.cindex is unavailable")
    args = _compile_db_args(compile_commands, source_path) or _default_clang_args(
        extra_clang_args
    )
    return idx.parse(
        str(source_path),
        args=args,
        options=TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD,
    )


def parse_plugin_file(
    idx: Index,
    source_path: pathlib.Path,
    compile_commands: list[CompilationDatabase],
    extra_clang_args: list[str],
    allow_regex_fallback: bool,
) -> PluginApi | None:
    text = source_path.read_text(encoding="utf-8")
    plugin, class_name, namespace, brief, description = _extract_plugin_meta(source_path, text)
    if not plugin:
        return None

    publishers: list[ApiEntry]
    subscribers: list[ApiEntry]
    services: list[ApiEntry]
    clients: list[ApiEntry]
    parameters: list[ApiEntry]

    try:
        tu = _parse_translation_unit(idx, source_path, compile_commands, extra_clang_args)
        publishers, subscribers, services, clients, parameters = _extract_ast_api(
            tu, source_path, text
        )
        if allow_regex_fallback and not any(
            (publishers, subscribers, services, clients, parameters)
        ):
            publishers, subscribers, services, clients, parameters = _extract_api_regex_fallback(
                text
            )
    except Exception:
        if not allow_regex_fallback:
            raise
        publishers, subscribers, services, clients, parameters = _extract_api_regex_fallback(text)

    return PluginApi(
        plugin=plugin,
        path=source_path,
        class_name=class_name,
        namespace=namespace,
        brief=brief,
        description=description,
        publishers=publishers,
        subscribers=subscribers,
        services=services,
        clients=clients,
        parameters=parameters,
    )


def load_plugins(
    plugin_dirs: list[pathlib.Path],
    compile_commands: list[CompilationDatabase],
    extra_clang_args: list[str],
    allow_regex_fallback: bool,
    wanted_plugins: set[str] | None = None,
) -> list[PluginApi]:
    if CLANG_AVAILABLE:
        idx = Index.create()
    elif not allow_regex_fallback:
        raise RuntimeError("clang.cindex is unavailable and fallback is disabled")
    else:
        idx = None
    out: list[PluginApi] = []
    for plugin_dir in plugin_dirs:
        if not plugin_dir.exists():
            continue
        for source_path in sorted(plugin_dir.glob("*.cpp")):
            if wanted_plugins is not None:
                try:
                    source_text = source_path.read_text(encoding="utf-8")
                except Exception:
                    source_text = ""
                plugin_name_match = PLUGIN_NAME_RE.search(source_text)
                plugin_name = (
                    plugin_name_match.group("name")
                    if plugin_name_match is not None
                    else source_path.stem
                )
                if plugin_name not in wanted_plugins:
                    continue
            parsed = parse_plugin_file(
                idx,
                source_path,
                compile_commands=compile_commands,
                extra_clang_args=extra_clang_args,
                allow_regex_fallback=allow_regex_fallback,
            )
            if parsed is not None:
                out.append(parsed)
    return sorted(out, key=lambda item: item.plugin)


def _render_api_section(title: str, entries: list[ApiEntry]) -> list[str]:
    lines = [f"### {title}"]
    if not entries:
        lines.append("- None")
        lines.append("")
        return lines
    for entry in entries:
        lines.append(f"- `{entry.name}` ({entry.rendered_type})")
    lines.append("")
    return lines


def render_markdown(plugins: list[PluginApi]) -> str:
    lines = [
        "# MAVROS Plugin API",
        "",
        f"_Generated for {len(plugins)} plugins._",
        "",
    ]
    for plugin in plugins:
        try:
            shown_path = plugin.path.relative_to(REPO_ROOT).as_posix()
        except ValueError:
            shown_path = plugin.path.as_posix()
        lines.extend(
            [
                f"## `{plugin.plugin}`",
                "",
                f"- File: `{shown_path}`",
                f"- Class: `{plugin.class_name or '<unknown>'}`",
                f"- Namespace: `{plugin.namespace or '<unknown>'}`",
            ]
        )
        if plugin.brief:
            lines.append(f"- Brief: {plugin.brief}")
        lines.append("")
        if plugin.description:
            lines.append(plugin.description)
            lines.append("")
        lines.extend(_render_api_section("Publishers", plugin.publishers))
        lines.extend(_render_api_section("Subscribers", plugin.subscribers))
        lines.extend(_render_api_section("Services", plugin.services))
        lines.extend(_render_api_section("Clients", plugin.clients))
        lines.extend(_render_api_section("Parameters", plugin.parameters))
    return "\n".join(lines).rstrip() + "\n"


def render_json(plugins: list[PluginApi]) -> str:
    payload = []
    for plugin in plugins:
        item = dataclasses.asdict(plugin)
        try:
            item["path"] = plugin.path.relative_to(REPO_ROOT).as_posix()
        except ValueError:
            item["path"] = plugin.path.as_posix()
        payload.append(item)
    return json.dumps(payload, indent=2) + "\n"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plugin-dir",
        action="append",
        default=[],
        help="Plugin source directory. May be passed multiple times.",
    )
    parser.add_argument(
        "--plugin",
        action="append",
        default=[],
        help="Only include plugin(s) by @plugin name. May be passed multiple times.",
    )
    parser.add_argument(
        "--compile-commands",
        action="append",
        default=[],
        help="Path to directory containing compile_commands.json. Repeat for multiple packages.",
    )
    parser.add_argument(
        "--clang-arg",
        action="append",
        default=[],
        help="Extra arg forwarded to clang parser. May be repeated.",
    )
    parser.add_argument(
        "--no-regex-fallback",
        action="store_true",
        help="Disable regex fallback when AST extraction yields no API entries.",
    )
    parser.add_argument(
        "--format",
        choices=("markdown", "json"),
        default="markdown",
        help="Output format.",
    )
    parser.add_argument(
        "--output",
        help="Write output to file. Defaults to stdout.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    plugin_dirs = (
        [pathlib.Path(p).resolve() for p in args.plugin_dir]
        if args.plugin_dir
        else [(REPO_ROOT / p).resolve() for p in DEFAULT_PLUGIN_DIRS]
    )

    compile_commands: list[CompilationDatabase] = []
    for compile_commands_dir in args.compile_commands:
        compile_commands.append(CompilationDatabase.fromDirectory(compile_commands_dir))

    plugins = load_plugins(
        plugin_dirs=plugin_dirs,
        compile_commands=compile_commands,
        extra_clang_args=args.clang_arg,
        allow_regex_fallback=not args.no_regex_fallback,
        wanted_plugins=set(args.plugin) if args.plugin else None,
    )

    body = render_json(plugins) if args.format == "json" else render_markdown(plugins)
    if args.output:
        out_path = pathlib.Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(body, encoding="utf-8")
    else:
        sys.stdout.write(body)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
