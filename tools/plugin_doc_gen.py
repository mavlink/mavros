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
PLUGIN_INDEX_TEMPLATE = SCRIPT_DIR / "templates" / "plugin_index.md.j2"


@dataclasses.dataclass(frozen=True)
class ApiEntry:
    name: str
    type_name: str
    line: int
    default_value: str = ""
    description: str = ""
    qos: dict[str, ty.Any] | None = None


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
        if e.qos:
            out["qos"] = e.qos
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
        "mavlink_subscriptions": [
            mavlink_sub_to_dict(s) for s in plugin.mavlink_subscriptions
        ],
        "mavlink_publications": [
            mavlink_pub_to_dict(s) for s in plugin.mavlink_publications
        ],
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
                qos=e.get("qos"),
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

    def load_mavlink_pub_entries(
        entries: list[dict[str, ty.Any]],
    ) -> list[MavlinkPubEntry]:
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
        mavlink_subscriptions=load_mavlink_entries(
            item.get("mavlink_subscriptions", [])
        ),
        mavlink_publications=load_mavlink_pub_entries(
            item.get("mavlink_publications", [])
        ),
    )


def render_json(plugins: list[PluginApi]) -> str:
    return json.dumps([plugin_to_dict(p) for p in plugins], indent=2) + "\n"


def parse_qos(qos: dict[str, ty.Any] | str | None) -> dict[str, ty.Any]:
    """Normalize the `qos` field (dict from the extractor, or a JSON string)."""
    if not qos:
        return {}
    if isinstance(qos, dict):
        return qos
    try:
        return json.loads(qos)
    except json.JSONDecodeError:
        return {}


def qos_slug(q: dict[str, ty.Any]) -> str:
    """Stable, human-readable anchor for a QoS profile."""
    s = q.get("name", "") if q.get("kind") == "named" else q.get("config", "")
    out = []
    for c in s:
        if c.isalnum() or c == "_":
            out.append(c.lower())
        elif c == "/":
            out.append("-")
        else:
            out.append("_")
    return "".join(out) or "q"


def qos_key(q: dict[str, ty.Any]) -> str:
    """Canonical key for deduplicating QoS profiles."""
    if q.get("kind") == "named":
        return "named:" + q.get("name", "")
    return "inline:" + q.get("config", "")


def build_qos_registry(plugins: list[PluginApi]) -> dict[str, dict[str, ty.Any]]:
    """Collect distinct QoS profiles across all plugins and assign ids/labels."""
    reg: dict[str, dict[str, ty.Any]] = {}
    for pl in plugins:
        for ent in pl.publishers + pl.subscribers + pl.services:
            q = parse_qos(ent.qos)
            if not q:
                continue
            k = qos_key(q)
            if k not in reg:
                reg[k] = {
                    "key": k,
                    "kind": q.get("kind", ""),
                    "name": q.get("name", ""),
                    "config": q.get("config", ""),
                    "uses": [],
                }
            reg[k]["uses"].append((pl.plugin, ent.name, str(q.get("var", "")) or ""))
    for k, e in reg.items():
        if e["kind"] == "named":
            e["id"] = qos_slug(e)
            e["label"] = e["name"]
        else:
            var_hints = [f"{p}/{v}" for (p, _n, v) in e["uses"] if v]
            if var_hints:
                e["id"] = qos_slug({"kind": "inline", "config": var_hints[0]})
                e["label"] = var_hints[0]
            else:
                e["id"] = qos_slug(e)
                e["label"] = e["config"]
    return reg


def qos_link(ent: ApiEntry, plugin: str, reg: dict[str, dict[str, ty.Any]]) -> str:
    """Markdown for the QoS link of an entity, or empty if none."""
    q = parse_qos(ent.qos)
    if not q:
        return ""
    e = reg.get(qos_key(q))
    if not e:
        return ""
    if e["kind"] == "named":
        return f'[{e["name"]}](../qos.md#{e["id"]} "{e["name"]} QoS profile")'
    var = q.get("var")
    if var:
        return f'[{var}](../qos.md#{e["id"]} "{e["config"]}")'
    return f"[{e['config']}](../qos.md#{e['id']})"


def render_qos_appendix(reg: dict[str, dict[str, ty.Any]]) -> str:
    """Render the QoS appendix page listing each distinct profile once."""
    lines = [
        "# QoS profiles",
        "",
        (
            "This page lists every QoS profile used by the MAVROS plugins. "
            "Standard `rclcpp::*` profiles link to the rclcpp API docs."
        ),
        "",
    ]
    by_kind = sorted(reg.values(), key=lambda e: (e["kind"], e["label"]))
    cur_kind = None
    for e in by_kind:
        if e["kind"] != cur_kind:
            cur_kind = e["kind"]
            lines.append(f"## {cur_kind.capitalize()}")
            lines.append("")
            if cur_kind == "inline":
                lines.append("| Id | Config | Topics |")
                lines.append("|----|--------|--------|")
            else:
                lines.append("| Id | Profile | Topics |")
                lines.append("|----|---------|--------|")
        if cur_kind == "inline":
            uses = ", ".join(sorted({f"`{p}`" for p, _n, _v in e["uses"]}))
            lines.append(f"| `{e['id']}` | `{e['config']}` | {uses} |")
        else:
            uses = ", ".join(sorted({f"`{p}`" for p, _n, _v in e["uses"]}))
            rclcpp = (
                e["name"]
                if e["name"].startswith(
                    (
                        "SensorDataQoS",
                        "ServicesQoS",
                        "ParametersQoS",
                        "ParameterEventsQoS",
                        "RosoutQoS",
                        "SystemDefaultQoS",
                    )
                )
                else ""
            )
            name_cell = (
                f"[`{e['name']}`](https://docs.ros.org/en/rolling/p/rclcpp/classrclcpp_1_1{e['name']}.html)"
                if rclcpp
                else f"`{e['name']}`"
            )
            lines.append(f"| `{e['id']}` | {name_cell} | {uses} |")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def render_plugin_index(
    std_plugins: list[PluginApi], extras_plugins: list[PluginApi]
) -> str:
    """Render the combined plugin index page from a Jinja template."""
    env = Environment(
        loader=FileSystemLoader(str(PLUGIN_INDEX_TEMPLATE.parent)), autoescape=False
    )
    template = env.get_template(PLUGIN_INDEX_TEMPLATE.name)
    return (
        template.render(
            std_plugins=sorted(std_plugins, key=lambda x: x.plugin),
            extras_plugins=sorted(extras_plugins, key=lambda x: x.plugin),
        ).rstrip()
        + "\n"
    )


def render_plugin_markdown_with_template(
    plugin: PluginApi,
    template_path: pathlib.Path,
    repo_root: pathlib.Path,
    qos_reg: dict[str, dict[str, ty.Any]] | None = None,
) -> str:
    if Environment is None or FileSystemLoader is None:
        raise RuntimeError(
            "Jinja2 is required for templated markdown output. Install dependencies with `uv sync`."
        )
    # This renderer is used only for offline docs generation, not for request/response HTML.
    # nosemgrep: python.flask.security.xss.audit.direct-use-of-jinja2.direct-use-of-jinja2
    env = Environment(
        loader=FileSystemLoader(str(template_path.parent)), autoescape=False
    )
    template = env.get_template(template_path.name)
    try:
        shown_path = plugin.path.relative_to(repo_root).as_posix()
    except ValueError:
        shown_path = plugin.path.as_posix()
    # nosemgrep: python.flask.security.xss.audit.direct-use-of-jinja2.direct-use-of-jinja2
    body = template.render(
        plugin=plugin,
        shown_path=shown_path,
        qos_link_fn=(lambda ent: qos_link(ent, plugin.plugin, qos_reg))
        if qos_reg
        else None,
    )
    return body.rstrip() + "\n"


def write_markdown_files(
    plugins: list[PluginApi],
    output_dir: pathlib.Path,
    template_path: pathlib.Path,
    qos_reg: dict[str, dict[str, ty.Any]] | None = None,
) -> list[pathlib.Path]:
    repo_root = detect_repo_root()
    output_dir.mkdir(parents=True, exist_ok=True)
    written: list[pathlib.Path] = []
    for plugin in plugins:
        stem = pathlib.Path(plugin.path).stem or plugin.plugin
        out_path = output_dir / f"{stem}.md"
        body = render_plugin_markdown_with_template(
            plugin, template_path, repo_root, qos_reg
        )
        out_path.write_text(body, encoding="utf-8")
        written.append(out_path)
    return written


def load_plugins_via_cpp(
    plugin_dirs: list[pathlib.Path],
    wanted_plugins: set[str] | None,
    jobs: int,
    cpp_bin: pathlib.Path,
    compile_commands_dir: str = "",
) -> list[PluginApi]:
    if not cpp_bin.exists():
        raise FileNotFoundError(f"C++ extractor not found: {cpp_bin}")

    with tempfile.NamedTemporaryFile(
        prefix="plugin-doc-cpp-", suffix=".json", delete=False
    ) as tmp:
        tmp_path = pathlib.Path(tmp.name)

    cmd = [str(cpp_bin), "--jobs", str(max(1, jobs)), "--output", str(tmp_path)]
    if compile_commands_dir:
        cmd += ["--compile-commands-dir", compile_commands_dir]
    for plugin_dir in plugin_dirs:
        cmd += ["--plugin-dir", str(plugin_dir)]
    if wanted_plugins:
        for plugin in sorted(wanted_plugins):
            cmd += ["--plugin", plugin]

    log_event(
        "info",
        "Starting C++ collection",
        phase="collect_cpp",
        jobs=jobs,
        bin=str(cpp_bin),
    )
    subprocess.run(cmd, check=True)
    payload = json.loads(tmp_path.read_text(encoding="utf-8"))
    tmp_path.unlink(missing_ok=True)
    plugins = [plugin_from_dict(item) for item in payload]
    log_event(
        "info", "C++ collection finished", phase="collect_cpp", plugins=len(plugins)
    )
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
        "--compile-commands-dir",
        help="Directory containing compile_commands.json for the clang extractor.",
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
        action="append",
        default=[],
        help="Skip collection and render from pre-collected JSON file(s). May be repeated.",
    )
    parser.add_argument(
        "--plugin-index",
        help="Write a combined plugin index page (from collected/index JSON files).",
    )
    parser.add_argument(
        "--qos-appendix",
        help="Write the QoS appendix page (from collected/index JSON files).",
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
        plugins = []
        for input_path in args.input_json:
            payload = json.loads(pathlib.Path(input_path).read_text(encoding="utf-8"))
            plugins.extend(plugin_from_dict(item) for item in payload)
        log_event("info", "Loaded input JSON", phase="render", plugins=len(plugins))
    else:
        wanted_plugins = set(args.plugin) if args.plugin else None
        cpp_bin = pathlib.Path(args.cpp_bin).resolve()
        plugins = load_plugins_via_cpp(
            plugin_dirs=plugin_dirs,
            wanted_plugins=wanted_plugins,
            jobs=max(1, args.jobs),
            cpp_bin=cpp_bin,
            compile_commands_dir=args.compile_commands_dir,
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

    qos_reg = build_qos_registry(plugins)

    if args.plugin_index:
        std_plugins = [p for p in plugins if "mavros_extras" not in p.path.as_posix()]
        extras_plugins = [p for p in plugins if "mavros_extras" in p.path.as_posix()]
        idx = pathlib.Path(args.plugin_index)
        idx.parent.mkdir(parents=True, exist_ok=True)
        idx.write_text(
            render_plugin_index(std_plugins, extras_plugins), encoding="utf-8"
        )
        log_event(
            "info",
            "Wrote plugin index",
            phase="render",
            std=len(std_plugins),
            extras=len(extras_plugins),
            path=str(idx),
        )
        return 0

    if args.qos_appendix:
        qos_path = pathlib.Path(args.qos_appendix)
        qos_path.parent.mkdir(parents=True, exist_ok=True)
        qos_path.write_text(render_qos_appendix(qos_reg), encoding="utf-8")
        log_event(
            "info",
            "Wrote QoS appendix",
            phase="render",
            profiles=len(qos_reg),
            path=str(qos_path),
        )
        return 0

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
        written = write_markdown_files(
            plugins, output_dir=output_dir, template_path=template_path, qos_reg=qos_reg
        )
        log_event(
            "info",
            "Wrote markdown files",
            phase="render",
            files=len(written),
            path=str(output_dir),
        )
        return 0

    raise SystemExit(
        "Nothing to do: pass --output-dir, --format json, --plugin-index, or --qos-appendix"
    )


if __name__ == "__main__":
    raise SystemExit(main())
