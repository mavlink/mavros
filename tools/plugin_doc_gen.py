"""Generate plugin docs by collecting data from C++ extractor output.

This wrapper intentionally does not perform Python-side AST parsing.
Collection is delegated to the C++ extractor binary.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import os
import pathlib
import subprocess
import sys
import tempfile
import typing as ty

try:
    from loguru import logger
except Exception:
    logger = None

try:
    from jinja2 import Environment, FileSystemLoader
except Exception:
    Environment = None  # type: ignore[assignment]
    FileSystemLoader = None  # type: ignore[assignment]


DEFAULT_PLUGIN_DIRS = ("mavros/src/plugins", "mavros_extras/src/plugins")
SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_MARKDOWN_TEMPLATE = SCRIPT_DIR / "templates" / "plugin.md.j2"


@dataclasses.dataclass(frozen=True)
class ApiEntry:
    name: str
    type_name: str
    line: int
    default_value: str = ""
    description: str = ""

    @property
    def rendered_type(self) -> str:
        return self.type_name or "<unknown>"


@dataclasses.dataclass(frozen=True)
class MavlinkSubEntry:
    handler: str
    message_type: str
    message_name: str
    msg_id_expr: str
    line: int
    dialect: str = ""
    msg_id: int | None = None
    description: str = ""


@dataclasses.dataclass(frozen=True)
class MavlinkPubEntry:
    argument: str
    message_type: str
    message_name: str
    msg_id_expr: str
    line: int
    dialect: str = ""
    msg_id: int | None = None
    description: str = ""


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
    mavlink_subscriptions: list[MavlinkSubEntry]
    mavlink_publications: list[MavlinkPubEntry]


def setup_logging(level: str = "INFO", json_logs: bool = False) -> None:
    if logger is not None:
        logger.remove()
        logger.add(
            sys.stderr,
            level=level.upper(),
            serialize=json_logs,
            backtrace=False,
            diagnose=False,
        )
    else:
        logging.basicConfig(
            level=getattr(logging, level.upper(), logging.INFO),
            stream=sys.stderr,
            format="%(levelname)s %(message)s",
        )


def log_event(level: str, message: str, **fields: ty.Any) -> None:
    if logger is not None:
        logger.bind(**fields).log(level.upper(), message)
    else:
        suffix = f" {fields}" if fields else ""
        logging.log(getattr(logging, level.upper(), logging.INFO), f"{message}{suffix}")


def detect_repo_root() -> pathlib.Path:
    env_root = os.environ.get("MAVROS_REPO_ROOT")
    if env_root:
        candidate = pathlib.Path(env_root).resolve()
        if (candidate / "mavros/src/plugins").exists():
            return candidate

    for base in [pathlib.Path.cwd().resolve(), SCRIPT_DIR]:
        for candidate in [base, *base.parents]:
            if (candidate / "mavros/src/plugins").exists() and (
                candidate / "mavros_extras/src/plugins"
            ).exists():
                return candidate

    for explicit in (
        pathlib.Path("/ws/src/mavros"),
        pathlib.Path.home() / "ros2/src/mavros",
    ):
        if (explicit / "mavros/src/plugins").exists() and (
            explicit / "mavros_extras/src/plugins"
        ).exists():
            return explicit.resolve()

    return REPO_ROOT


def plugin_to_dict(plugin: PluginApi) -> dict[str, ty.Any]:
    repo_root = detect_repo_root()
    try:
        path = plugin.path.relative_to(repo_root).as_posix()
    except ValueError:
        path = plugin.path.as_posix()

    def entry_to_dict(e: ApiEntry) -> dict[str, ty.Any]:
        out: dict[str, ty.Any] = {
            "name": e.name,
            "type_name": e.type_name,
            "line": e.line,
        }
        if e.default_value:
            out["default_value"] = e.default_value
        if e.description:
            out["description"] = e.description
        return out

    def mavlink_sub_to_dict(s: MavlinkSubEntry) -> dict[str, ty.Any]:
        out: dict[str, ty.Any] = {
            "handler": s.handler,
            "message_type": s.message_type,
            "message_name": s.message_name,
            "msg_id_expr": s.msg_id_expr,
            "dialect": s.dialect,
            "line": s.line,
        }
        if s.msg_id is not None:
            out["msg_id"] = s.msg_id
        if s.description:
            out["description"] = s.description
        return out

    def mavlink_pub_to_dict(s: MavlinkPubEntry) -> dict[str, ty.Any]:
        out: dict[str, ty.Any] = {
            "argument": s.argument,
            "message_type": s.message_type,
            "message_name": s.message_name,
            "msg_id_expr": s.msg_id_expr,
            "dialect": s.dialect,
            "line": s.line,
        }
        if s.msg_id is not None:
            out["msg_id"] = s.msg_id
        if s.description:
            out["description"] = s.description
        return out

    return {
        "plugin": plugin.plugin,
        "path": path,
        "class_name": plugin.class_name,
        "namespace": plugin.namespace,
        "brief": plugin.brief,
        "description": plugin.description,
        "publishers": [entry_to_dict(e) for e in plugin.publishers],
        "subscribers": [entry_to_dict(e) for e in plugin.subscribers],
        "services": [entry_to_dict(e) for e in plugin.services],
        "clients": [entry_to_dict(e) for e in plugin.clients],
        "parameters": [entry_to_dict(e) for e in plugin.parameters],
        "mavlink_subscriptions": [mavlink_sub_to_dict(s) for s in plugin.mavlink_subscriptions],
        "mavlink_publications": [mavlink_pub_to_dict(s) for s in plugin.mavlink_publications],
    }


def plugin_from_dict(item: dict[str, ty.Any]) -> PluginApi:
    def load_entries(entries: list[dict[str, ty.Any]]) -> list[ApiEntry]:
        return [
            ApiEntry(
                name=e["name"],
                type_name=e.get("type_name", ""),
                line=e["line"],
                default_value=e.get("default_value", ""),
                description=e.get("description", ""),
            )
            for e in entries
        ]

    def load_mavlink_entries(entries: list[dict[str, ty.Any]]) -> list[MavlinkSubEntry]:
        return [
            MavlinkSubEntry(
                handler=e.get("handler", ""),
                message_type=e.get("message_type", ""),
                message_name=e.get("message_name", ""),
                msg_id_expr=e.get("msg_id_expr", ""),
                dialect=e.get("dialect", ""),
                msg_id=e.get("msg_id"),
                line=e.get("line", 0),
                description=e.get("description", ""),
            )
            for e in entries
        ]

    def load_mavlink_pub_entries(entries: list[dict[str, ty.Any]]) -> list[MavlinkPubEntry]:
        return [
            MavlinkPubEntry(
                argument=e.get("argument", ""),
                message_type=e.get("message_type", ""),
                message_name=e.get("message_name", ""),
                msg_id_expr=e.get("msg_id_expr", ""),
                line=e.get("line", 0),
                dialect=e.get("dialect", ""),
                msg_id=e.get("msg_id"),
                description=e.get("description", ""),
            )
            for e in entries
        ]

    return PluginApi(
        plugin=item["plugin"],
        path=pathlib.Path(item["path"]),
        class_name=item["class_name"],
        namespace=item["namespace"],
        brief=item["brief"],
        description=item["description"],
        publishers=load_entries(item["publishers"]),
        subscribers=load_entries(item["subscribers"]),
        services=load_entries(item["services"]),
        clients=load_entries(item["clients"]),
        parameters=load_entries(item["parameters"]),
        mavlink_subscriptions=load_mavlink_entries(item.get("mavlink_subscriptions", [])),
        mavlink_publications=load_mavlink_pub_entries(item.get("mavlink_publications", [])),
    )


def render_json(plugins: list[PluginApi]) -> str:
    return json.dumps([plugin_to_dict(p) for p in plugins], indent=2) + "\n"


def _render_api_section(title: str, entries: list[ApiEntry]) -> list[str]:
    lines = [f"### {title}"]
    if not entries:
        lines.append("- None")
        lines.append("")
        return lines
    for entry in entries:
        if title == "Parameters":
            extras = []
            if entry.type_name:
                extras.append(f"type: {entry.type_name}")
            if entry.default_value:
                extras.append(f"default: `{entry.default_value}`")
            if entry.description:
                extras.append(f"desc: {entry.description}")
            suffix = f" [{', '.join(extras)}]" if extras else ""
            lines.append(f"- `{entry.name}`{suffix}")
        else:
            extra = f" - {entry.description}" if entry.description else ""
            lines.append(f"- `{entry.name}` ({entry.rendered_type}){extra}")
    lines.append("")
    return lines


def render_markdown(plugins: list[PluginApi]) -> str:
    repo_root = detect_repo_root()
    lines = [
        "# MAVROS Plugin API",
        "",
        f"_Generated for {len(plugins)} plugins._",
        "",
    ]
    for plugin in plugins:
        try:
            shown_path = plugin.path.relative_to(repo_root).as_posix()
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
        lines.append("### MAVLink Subscriptions")
        if not plugin.mavlink_subscriptions:
            lines.append("- None")
            lines.append("")
        else:
            for sub in plugin.mavlink_subscriptions:
                msg = sub.message_name or "<unknown>"
                extras = []
                if sub.handler:
                    extras.append(f"handler: {sub.handler}")
                if sub.dialect:
                    extras.append(f"dialect: {sub.dialect}")
                if sub.msg_id is not None:
                    extras.append(f"msg_id: {sub.msg_id}")
                if sub.msg_id_expr:
                    extras.append(f"id: `{sub.msg_id_expr}`")
                if sub.description:
                    extras.append(f"desc: {sub.description}")
                suffix = f" [{', '.join(extras)}]" if extras else ""
                lines.append(f"- `{msg}`{suffix}")
            lines.append("")
        lines.append("### MAVLink Publications")
        if not plugin.mavlink_publications:
            lines.append("- None")
            lines.append("")
        else:
            for pub in plugin.mavlink_publications:
                msg = pub.message_name or "<unknown>"
                extras = []
                if pub.argument:
                    extras.append(f"arg: `{pub.argument}`")
                if pub.dialect:
                    extras.append(f"dialect: {pub.dialect}")
                if pub.msg_id is not None:
                    extras.append(f"msg_id: {pub.msg_id}")
                if pub.msg_id_expr:
                    extras.append(f"id: `{pub.msg_id_expr}`")
                if pub.description:
                    extras.append(f"desc: {pub.description}")
                suffix = f" [{', '.join(extras)}]" if extras else ""
                lines.append(f"- `{msg}`{suffix}")
            lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def render_plugin_markdown_with_template(
    plugin: PluginApi, template_path: pathlib.Path, repo_root: pathlib.Path
) -> str:
    if Environment is None or FileSystemLoader is None:
        raise RuntimeError(
            "Jinja2 is required for templated markdown output. Install dependencies with `uv sync`."
        )
    # nosemgrep: python.flask.security.xss.audit.direct-use-of-jinja2.direct-use-of-jinja2
    # This renderer is used only for offline docs generation, not for request/response HTML.
    env = Environment(loader=FileSystemLoader(str(template_path.parent)), autoescape=False)
    template = env.get_template(template_path.name)
    try:
        shown_path = plugin.path.relative_to(repo_root).as_posix()
    except ValueError:
        shown_path = plugin.path.as_posix()
    # nosemgrep: python.flask.security.xss.audit.direct-use-of-jinja2.direct-use-of-jinja2
    body = template.render(plugin=plugin, shown_path=shown_path)
    return body.rstrip() + "\n"


def write_markdown_files(
    plugins: list[PluginApi], output_dir: pathlib.Path, template_path: pathlib.Path
) -> list[pathlib.Path]:
    repo_root = detect_repo_root()
    output_dir.mkdir(parents=True, exist_ok=True)
    written: list[pathlib.Path] = []
    for plugin in plugins:
        stem = pathlib.Path(plugin.path).stem or plugin.plugin
        out_path = output_dir / f"{stem}.md"
        body = render_plugin_markdown_with_template(plugin, template_path, repo_root)
        out_path.write_text(body, encoding="utf-8")
        written.append(out_path)
    return written


def load_plugins_via_cpp(
    plugin_dirs: list[pathlib.Path],
    wanted_plugins: set[str] | None,
    jobs: int,
    cpp_bin: pathlib.Path,
) -> list[PluginApi]:
    if not cpp_bin.exists():
        raise FileNotFoundError(f"C++ extractor not found: {cpp_bin}")

    with tempfile.NamedTemporaryFile(prefix="plugin-doc-cpp-", suffix=".json", delete=False) as tmp:
        tmp_path = pathlib.Path(tmp.name)

    cmd = [str(cpp_bin), "--jobs", str(max(1, jobs)), "--output", str(tmp_path)]
    for plugin_dir in plugin_dirs:
        cmd += ["--plugin-dir", str(plugin_dir)]
    if wanted_plugins:
        for plugin in sorted(wanted_plugins):
            cmd += ["--plugin", plugin]

    log_event("info", "Starting C++ collection", phase="collect_cpp", jobs=jobs, bin=str(cpp_bin))
    subprocess.run(cmd, check=True)
    payload = json.loads(tmp_path.read_text(encoding="utf-8"))
    tmp_path.unlink(missing_ok=True)
    plugins = [plugin_from_dict(item) for item in payload]
    log_event("info", "C++ collection finished", phase="collect_cpp", plugins=len(plugins))
    return sorted(plugins, key=lambda item: item.plugin)


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
        "--jobs",
        type=int,
        default=1,
        help="Number of worker processes used by C++ collector.",
    )
    parser.add_argument(
        "--cpp-bin",
        default="tools/build/plugin_doc_extract",
        help="Path to C++ collector binary.",
    )
    parser.add_argument(
        "--collect-output",
        help="Write collected raw plugin data as JSON before rendering.",
    )
    parser.add_argument(
        "--input-json",
        help="Skip collection and render from pre-collected JSON file.",
    )
    parser.add_argument(
        "--format",
        choices=("markdown", "json"),
        default="markdown",
        help="Output format.",
    )
    parser.add_argument(
        "--template",
        default=str(DEFAULT_MARKDOWN_TEMPLATE),
        help="Jinja2 template used for per-plugin markdown files.",
    )
    parser.add_argument(
        "--output-dir",
        help="Directory for per-plugin markdown output (<source_stem>.md).",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="Log level (TRACE, DEBUG, INFO, WARNING, ERROR).",
    )
    parser.add_argument(
        "--log-json",
        action="store_true",
        help="Enable structured JSON logs on stderr.",
    )
    parser.add_argument(
        "--output",
        help="Write output to file. Defaults to stdout.",
    )

    # Accepted for compatibility with old invocations; ignored.
    parser.add_argument("--collector", default="cpp", help=argparse.SUPPRESS)
    parser.add_argument("--compile-commands", action="append", default=[], help=argparse.SUPPRESS)
    parser.add_argument("--clang-arg", action="append", default=[], help=argparse.SUPPRESS)
    parser.add_argument("--no-regex-fallback", action="store_true", help=argparse.SUPPRESS)

    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    setup_logging(level=args.log_level, json_logs=args.log_json)
    repo_root = detect_repo_root()
    plugin_dirs = (
        [pathlib.Path(p).resolve() for p in args.plugin_dir]
        if args.plugin_dir
        else [(repo_root / p).resolve() for p in DEFAULT_PLUGIN_DIRS]
    )

    if args.input_json:
        payload = json.loads(pathlib.Path(args.input_json).read_text(encoding="utf-8"))
        plugins = [plugin_from_dict(item) for item in payload]
        log_event("info", "Loaded input JSON", phase="render", plugins=len(plugins))
    else:
        wanted_plugins = set(args.plugin) if args.plugin else None
        cpp_bin = pathlib.Path(args.cpp_bin).resolve()
        plugins = load_plugins_via_cpp(
            plugin_dirs=plugin_dirs,
            wanted_plugins=wanted_plugins,
            jobs=max(1, args.jobs),
            cpp_bin=cpp_bin,
        )
        if args.collect_output:
            collect_path = pathlib.Path(args.collect_output)
            collect_path.parent.mkdir(parents=True, exist_ok=True)
            collect_path.write_text(render_json(plugins), encoding="utf-8")
            log_event(
                "info",
                "Wrote collected JSON",
                path=str(collect_path),
                plugins=len(plugins),
            )

    if args.format == "json":
        body = render_json(plugins)
        if args.output:
            out_path = pathlib.Path(args.output)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(body, encoding="utf-8")
        else:
            sys.stdout.write(body)
        return 0

    if args.output_dir:
        template_path = pathlib.Path(args.template).resolve()
        if not template_path.exists():
            raise FileNotFoundError(f"Template not found: {template_path}")
        output_dir = pathlib.Path(args.output_dir)
        written = write_markdown_files(plugins, output_dir=output_dir, template_path=template_path)
        log_event("info", "Wrote markdown files", phase="render", files=len(written), path=str(output_dir))
        return 0

    body = render_markdown(plugins)
    if args.output:
        out_path = pathlib.Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(body, encoding="utf-8")
    else:
        sys.stdout.write(body)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
